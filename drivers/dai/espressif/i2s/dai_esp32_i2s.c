/*
 * Copyright (c) 2026 Espressif Systems / SOF Project
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/dai.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>

#include <soc/i2s_struct.h>
#if defined(CONFIG_SOC_SERIES_ESP32P4)
#include <soc/hp_sys_clkrst_struct.h>
#include <soc/lp_clkrst_struct.h>
#include <soc/io_mux_struct.h>
#elif defined(CONFIG_SOC_SERIES_ESP32C6)
#include <soc/pcr_struct.h>
#include <soc/io_mux_reg.h>
#include <soc/gpio_periph.h>
#elif defined(CONFIG_SOC_SERIES_ESP32S3)
#include <soc/system_struct.h>
#include <soc/io_mux_reg.h>
#include <soc/gpio_periph.h>
#endif
#include <soc/gpio_struct.h>
#include <soc/gpio_sig_map.h>
#include <hal/gpio_ll.h>
#include <hal/i2s_ll.h>
#include <esp_rom_gpio.h>
#include <zephyr/drivers/dma/dma_esp32.h>

LOG_MODULE_REGISTER(dai_esp32_i2s, CONFIG_DAI_LOG_LEVEL);

#define DT_DRV_COMPAT espressif_esp32_dai_i2s

struct dai_esp32_i2s_config {
	uint32_t reg_base;
	uint32_t irq;
	uint32_t dai_index;
	uint32_t fifo_depth;
	const struct pinctrl_dev_config *pincfg;
};

struct dai_esp32_i2s_data {
	struct dai_config cfg;
	struct dai_properties tx_props;
	struct dai_properties rx_props;
	enum dai_state tx_state;
	enum dai_state rx_state;
	bool is_slave;
	uint32_t sample_rate;
	uint8_t channels;
	uint8_t word_size;
	struct k_spinlock lock;
};

static int dai_esp32_i2s_probe(const struct device *dev)
{
	const struct dai_esp32_i2s_config *cfg = dev->config;
	LOG_INF("ESP32 DAI I2S probe (dai_index=%u)", cfg->dai_index);
	return 0;
}

static int dai_esp32_i2s_remove(const struct device *dev)
{
	return 0;
}

static int dai_esp32_i2s_config_set(const struct device *dev,
				    const struct dai_config *cfg,
				    const void *bespoke_cfg,
				    size_t size)
{
	const struct dai_esp32_i2s_config *dev_cfg = dev->config;
	(void)dev_cfg;
	struct dai_esp32_i2s_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	LOG_INF("DAI I2S config_set: rate=%u, format=0x%x, options=0x%x, channels=%u",
		cfg->rate, cfg->format, cfg->options, cfg->channels);

	data->cfg = *cfg;
	data->sample_rate = cfg->rate ? cfg->rate : 48000;
	data->channels = cfg->channels ? cfg->channels : 2;
	data->word_size = cfg->word_size ? cfg->word_size : 16;

	/* Determine Clock Provider (Master vs Slave) */
	enum dai_clock_provider provider = cfg->format & DAI_FORMAT_CLOCK_PROVIDER_MASK;
	if (provider == DAI_CBC_CFC || provider == DAI_CBC_CFP) {
		/* Codec is clock consumer -> ESP32 is Master (generates BCLK & WS) */
		data->is_slave = false;
	} else {
		/* Codec is clock provider -> ESP32 is Slave */
		data->is_slave = true;
	}

	/* Reset I2S0 peripheral */
	I2S0.tx_conf.tx_reset = 1;
	I2S0.tx_conf.tx_reset = 0;
	I2S0.tx_conf.tx_fifo_reset = 1;
	I2S0.tx_conf.tx_fifo_reset = 0;
	I2S0.rx_conf.rx_reset = 1;
	I2S0.rx_conf.rx_reset = 0;
	I2S0.rx_conf.rx_fifo_reset = 1;
	I2S0.rx_conf.rx_fifo_reset = 0;

	/* Enable internal I2S0 peripheral clock gate */
#if defined(CONFIG_SOC_SERIES_ESP32P4)
	I2S0.clk_gate.clk_en = 1;
#endif

	/* Configure Master/Slave Mode */
	I2S0.tx_conf.tx_slave_mod = data->is_slave ? 1 : 0;
	I2S0.rx_conf.rx_slave_mod = data->is_slave ? 1 : 0;

	/* Reset TX and RX units */
	I2S0.tx_conf.tx_reset = 1;
	I2S0.tx_conf.tx_reset = 0;
	I2S0.rx_conf.rx_reset = 1;
	I2S0.rx_conf.rx_reset = 0;

	/* Configure TDM / Philips I2S format */
	I2S0.tx_conf.tx_pdm_en = 0;
	I2S0.tx_conf.tx_tdm_en = 1;
	I2S0.tx_conf.tx_pcm_bypass = 1;
	I2S0.rx_conf.rx_pdm_en = 0;
	I2S0.rx_conf.rx_tdm_en = 1;
	I2S0.rx_conf.rx_pcm_bypass = 1;
	I2S0.tx_pcm2pdm_conf.val = 0;
	I2S0.tx_pcm2pdm_conf1.val = 0;
#if defined(CONFIG_SOC_SERIES_ESP32P4)
	I2S0.rx_pdm2pcm_conf.val = 0;
#endif
	I2S0.rx_timing.val = 0;
	I2S0.tx_timing.val = 0;

	uint32_t slot_bits = (data->word_size > 16) ? 32 : 16;

	/* Configure 2-slot TDM (Standard Stereo I2S: exactly 2 slots, channels 0 and 1 active) */
	I2S0.tx_tdm_ctrl.val = (1 << 16) | 0x0003;
	I2S0.rx_tdm_ctrl.val = (1 << 16) | 0x0003;

	I2S0.tx_conf1.tx_bits_mod = data->word_size > 0 ? (data->word_size - 1) : 15;
	I2S0.tx_conf1.tx_tdm_chan_bits = slot_bits - 1;
	I2S0.tx_conf1.tx_half_sample_bits = slot_bits - 1;
	I2S0.tx_conf1.tx_tdm_ws_width = slot_bits - 1;

	I2S0.rx_conf1.rx_bits_mod = data->word_size > 0 ? (data->word_size - 1) : 15;
	I2S0.rx_conf1.rx_tdm_chan_bits = slot_bits - 1;
	I2S0.rx_conf1.rx_half_sample_bits = slot_bits - 1;
	I2S0.rx_conf1.rx_tdm_ws_width = slot_bits - 1;

	/* 1ms buffer in bytes for 48kHz stereo 16-bit = 192 bytes (96 samples).
	 * Hardware EOF bit length = (I2S_RX_BITS_MOD + 1) * (REG_RX_EOF_NUM + 1).
	 * With 16-bit audio (rx_bits_mod = 15), bit length is 16 * (rx_eof_num + 1) bits = 2 * (rx_eof_num + 1) bytes.
	 * To trigger in_suc_eof at exactly 192 bytes (one descriptor): rx_eof_num = 96 - 1 = 95.
	 */
	uint32_t slot_bytes = slot_bits / 8;
	uint32_t period_bytes = cfg->block_size ? cfg->block_size :
				(((data->sample_rate ? data->sample_rate : 48000) / 1000) *
				 (data->channels ? data->channels : 2) * (slot_bytes ? slot_bytes : 2));
	I2S0.rx_eof_num.rx_eof_num = period_bytes > 0 ? (period_bytes - 1) : 0;

	/* Philips standard: 1 bit MSB shift, WS low for left channel, left align */
#if defined(CONFIG_SOC_SERIES_ESP32P4)
	I2S0.tx_conf.tx_msb_shift = 1;
#elif defined(CONFIG_SOC_SERIES_ESP32C6) || defined(CONFIG_SOC_SERIES_ESP32S3)
	I2S0.tx_conf1.tx_msb_shift = 1;
#endif
	I2S0.tx_conf.tx_ws_idle_pol = 0;
	I2S0.tx_conf.tx_left_align = 1;
	I2S0.tx_conf.tx_big_endian = 0;
	I2S0.tx_conf.tx_bit_order = 0;
	I2S0.tx_conf.tx_chan_equal = 0;
#if defined(CONFIG_SOC_SERIES_ESP32S3)
	I2S0.conf_single_data = 0;
#else
	I2S0.conf_single_data.val = 0;
#endif

#if defined(CONFIG_SOC_SERIES_ESP32P4)
	I2S0.rx_conf.rx_msb_shift = 1;
#elif defined(CONFIG_SOC_SERIES_ESP32C6) || defined(CONFIG_SOC_SERIES_ESP32S3)
	I2S0.rx_conf1.rx_msb_shift = 1;
#endif
	I2S0.rx_conf.rx_ws_idle_pol = 0;
	I2S0.rx_conf.rx_left_align = 1;
	I2S0.rx_conf.rx_big_endian = 0;
	I2S0.rx_conf.rx_bit_order = 0;

	/* Clock Generation:
	 * Enable APB and peripheral clocks in HP_SYS_CLKRST, PCR, or SYSTEM */
#if defined(CONFIG_SOC_SERIES_ESP32P4)
	HP_SYS_CLKRST.soc_clk_ctrl2.reg_i2s0_apb_clk_en = 1;
	LP_AON_CLKRST.hp_clk_ctrl.hp_pad_i2s0_mclk_en = 1;
	HP_SYS_CLKRST.peri_clk_ctrl13.reg_i2s0_tx_clk_src_sel = 0; /* XTAL 40M */
	HP_SYS_CLKRST.peri_clk_ctrl13.reg_i2s0_tx_clk_en = 1;
	HP_SYS_CLKRST.peri_clk_ctrl11.reg_i2s0_rx_clk_src_sel = 0; /* XTAL 40M */
	HP_SYS_CLKRST.peri_clk_ctrl11.reg_i2s0_rx_clk_en = 1;
	I2S0.clk_gate.clk_en = 1;
#elif defined(CONFIG_SOC_SERIES_ESP32C6)
	PCR.i2s_conf.i2s_clk_en = 1;
	PCR.i2s_conf.i2s_rst_en = 0;
	PCR.i2s_tx_clkm_conf.i2s_tx_clkm_sel = 0; /* XTAL 40M */
	PCR.i2s_tx_clkm_conf.i2s_tx_clkm_en = 1;
	PCR.i2s_rx_clkm_conf.i2s_rx_clkm_sel = 0; /* XTAL 40M */
	PCR.i2s_rx_clkm_conf.i2s_rx_clkm_en = 1;
#elif defined(CONFIG_SOC_SERIES_ESP32S3)
	SYSTEM.perip_clk_en0.i2s0_clk_en = 1;
	SYSTEM.perip_rst_en0.i2s0_rst = 0;
	I2S0.tx_clkm_conf.tx_clk_sel = 0; /* XTAL 40M */
	I2S0.tx_clkm_conf.tx_clk_active = 1;
	I2S0.tx_clkm_conf.clk_en = 1;
	I2S0.rx_clkm_conf.rx_clk_sel = 0; /* XTAL 40M */
	I2S0.rx_clkm_conf.rx_clk_active = 1;
#endif

	slot_bits = (data->word_size > 16) ? 32 : 16;
	uint32_t bclk = (data->sample_rate ? data->sample_rate : 48000) * 2 * slot_bits;
	/* ESP32 hardware requires bclk_div >= 8 in slave mode for internal oversampling */
	uint32_t bclk_div = data->is_slave ? 8 : 2;
	uint32_t mclk = bclk * bclk_div;

	/* Calculate fractional divider for XTAL (40 MHz) */
	uint32_t sclk = 40000000;
	uint32_t div_int = sclk / mclk;
	uint32_t rem = sclk % mclk;
	uint32_t div_x = 0, div_y = 0, div_z = 0, div_yn1 = 0;
	if (rem) {
		uint32_t a = mclk, b = rem;
		while (b) {
			uint32_t t = b;
			b = a % b;
			a = t;
		}
		uint32_t num = rem / a;
		uint32_t den = mclk / a;
		while (den > 512) {
			den /= 2;
			num /= 2;
		}
		if (den && num) {
			div_yn1 = (num * 2 > den) ? 1 : 0;
			div_z = div_yn1 ? (den - num) : num;
			div_x = (den / div_z) - 1;
			div_y = den % div_z;
		}
	}

	/* Set MCLK dividers with hardware workaround for BOTH TX and RX */
	i2s_ll_tx_set_raw_clk_div(&I2S0, div_int, div_x, div_y, div_z, div_yn1);
	i2s_ll_rx_set_raw_clk_div(&I2S0, div_int, div_x, div_y, div_z, div_yn1);

#if defined(CONFIG_SOC_SERIES_ESP32P4)
	I2S0.tx_conf.tx_bck_div_num = bclk_div - 1;
	I2S0.rx_conf.rx_bck_div_num = bclk_div - 1;
#elif defined(CONFIG_SOC_SERIES_ESP32C6) || defined(CONFIG_SOC_SERIES_ESP32S3)
	I2S0.tx_conf1.tx_bck_div_num = bclk_div - 1;
	I2S0.rx_conf1.rx_bck_div_num = bclk_div - 1;
#endif
	I2S0.tx_conf.tx_stop_en = 0;

#if defined(CONFIG_SOC_SERIES_ESP32P4)
	/* Float PDM GPIOs (GPIO 3, 4, 5) to High-Z in I2S mode */
	GPIO.func_out_sel_cfg[3].out_sel = 256;
	GPIO.func_out_sel_cfg[3].oen_sel = 1;
	GPIO.func_out_sel_cfg[4].out_sel = 256;
	GPIO.func_out_sel_cfg[4].oen_sel = 1;
	GPIO.func_out_sel_cfg[5].out_sel = 256;
	GPIO.func_out_sel_cfg[5].oen_sel = 1;
	GPIO.enable_w1tc.val = (1 << 3) | (1 << 4) | (1 << 5);
#endif

	if (data->is_slave) {
		/* Attach MCLK to RX module */
#if defined(CONFIG_SOC_SERIES_ESP32P4)
		_i2s_ll_mclk_bind_to_rx_clk(&I2S0);
#else
		i2s_ll_mclk_bind_to_rx_clk(&I2S0);
#endif

		I2S0.tx_conf.tx_slave_mod = 1;
		I2S0.rx_conf.rx_slave_mod = 1;
		I2S0.tx_conf.tx_start = 0;
		I2S0.tx_conf.sig_loopback = 0;

#if defined(CONFIG_SOC_SERIES_ESP32C6)
		/* ESP32-C6 Slave GPIO Matrix:
		 * BCLK In: GPIO 18 -> I2SI_BCK_IN_IDX (16) & I2SO_BCK_IN_IDX (13)
		 * WS In:   GPIO 19 -> I2SI_WS_IN_IDX (17) & I2SO_WS_IN_IDX (14)
		 * DIN In:  GPIO 21 -> I2SI_SD_IN_IDX (15)
		 * DOUT Out:GPIO 20 <- I2SO_SD_OUT_IDX (15)
		 */
		const uint8_t c6_pins[] = {18, 19, 20, 21};
		for (int i = 0; i < 4; i++) {
			uint8_t p = c6_pins[i];
			GPIO.func_out_sel_cfg[p].out_sel = SIG_GPIO_OUT_IDX;
			GPIO.func_out_sel_cfg[p].oen_sel = 1;
			GPIO.func_out_sel_cfg[p].oen_inv_sel = 0;
			GPIO.pin[p].pad_driver = 0;
			esp_rom_gpio_pad_unhold(p);
			esp_rom_gpio_pad_select_gpio(p);
			PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[p]);
			esp_rom_gpio_pad_set_drv(p, 3);
		}
		GPIO.enable_w1tc.val = (1 << 18) | (1 << 19) | (1 << 21);
		GPIO.enable_w1ts.val = (1 << 20);

		esp_rom_gpio_connect_in_signal(18, I2SI_BCK_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(18, I2SO_BCK_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(19, I2SI_WS_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(19, I2SO_WS_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(21, I2SI_SD_IN_IDX, false);
		esp_rom_gpio_connect_out_signal(20, I2SO_SD_OUT_IDX, false, false);
		GPIO.func_out_sel_cfg[20].out_sel = I2SO_SD_OUT_IDX;

		LOG_INF("DAI I2S set to SLAVE mode on ESP32-C6 (BCLK:G18, WS:G19, DIN:G21, DOUT:G20)");
#elif defined(CONFIG_SOC_SERIES_ESP32S3)
		/* ESP32-S3 Slave GPIO Matrix:
		 * BCLK In: GPIO 18 (D9)  -> I2S0I_BCK_IN_IDX (26) & I2S0O_BCK_IN_IDX (22)
		 * WS In:   GPIO 21 (D10) -> I2S0I_WS_IN_IDX (27) & I2S0O_WS_IN_IDX (24)
		 * DIN In:  GPIO 8  (D5)  -> I2S0I_SD_IN_IDX (25)
		 * DOUT Out:GPIO 7  (D4)  <- I2S0O_SD_OUT_IDX (25)
		 */
		const uint8_t s3_pins[] = {18, 21, 7, 8};
		for (int i = 0; i < 4; i++) {
			uint8_t p = s3_pins[i];
			GPIO.func_out_sel_cfg[p].func_sel = SIG_GPIO_OUT_IDX;
			GPIO.func_out_sel_cfg[p].oen_sel = 1;
			GPIO.func_out_sel_cfg[p].oen_inv_sel = 0;
			GPIO.pin[p].pad_driver = 0;
			esp_rom_gpio_pad_unhold(p);
			esp_rom_gpio_pad_select_gpio(p);
			PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[p]);
			esp_rom_gpio_pad_set_drv(p, 3);
		}
		GPIO.enable_w1tc = (1 << 18) | (1 << 21) | (1 << 8);
		GPIO.enable_w1ts = (1 << 7);

		esp_rom_gpio_connect_in_signal(18, I2S0I_BCK_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(18, I2S0O_BCK_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(21, I2S0I_WS_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(21, I2S0O_WS_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(8, I2S0I_SD_IN_IDX, false);
		esp_rom_gpio_connect_out_signal(7, I2S0O_SD_OUT_IDX, false, false);
		GPIO.func_out_sel_cfg[7].func_sel = I2S0O_SD_OUT_IDX;

		LOG_INF("DAI I2S set to SLAVE mode on ESP32-S3 (BCLK:G18/D9, WS:G21/D10, DIN:G8/D5, DOUT:G7/D4)");
#else
		/* Slave GPIO Matrix:
		 * BCLK In: GPIO 21 (Pin 11) -> I2S0_I_BCK_PAD_IN_IDX (29) & I2S0_O_BCK_PAD_IN_IDX (25)
		 * WS In:   GPIO 22 (Pin 12) -> I2S0_I_WS_PAD_IN_IDX (30) & I2S0_O_WS_PAD_IN_IDX (27)
		 * DIN In:  GPIO 23 (Pin 7)  -> I2S0_I_SD_PAD_IN_IDX (28) (Connected to Pallas GPIO 20)
		 * DOUT Out:GPIO 20 (Pin 13) <- I2S0_O_SD_PAD_OUT_IDX (28)
		 */
		GPIO.func_out_sel_cfg[20].out_sel = 256;
		GPIO.func_out_sel_cfg[20].oen_sel = 1;
		GPIO.func_out_sel_cfg[20].oen_inv_sel = 0;

		GPIO.func_out_sel_cfg[21].out_sel = 256;
		GPIO.func_out_sel_cfg[21].oen_sel = 1;
		GPIO.func_out_sel_cfg[21].oen_inv_sel = 0;

		GPIO.func_out_sel_cfg[22].out_sel = 256;
		GPIO.func_out_sel_cfg[22].oen_sel = 1;
		GPIO.func_out_sel_cfg[22].oen_inv_sel = 0;

		GPIO.func_out_sel_cfg[23].out_sel = 256;
		GPIO.func_out_sel_cfg[23].oen_sel = 1;
		GPIO.func_out_sel_cfg[23].oen_inv_sel = 0;

		GPIO.enable_w1tc.val = (1 << 20) | (1 << 21) | (1 << 22) | (1 << 23);

		GPIO.pin[20].pad_driver = 0;
		GPIO.pin[21].pad_driver = 0;
		GPIO.pin[22].pad_driver = 0;
		GPIO.pin[23].pad_driver = 0;

		esp_rom_gpio_pad_unhold(20);
		esp_rom_gpio_pad_unhold(21);
		esp_rom_gpio_pad_unhold(22);
		esp_rom_gpio_pad_unhold(23);

		esp_rom_gpio_pad_select_gpio(20);
		esp_rom_gpio_pad_select_gpio(21);
		esp_rom_gpio_pad_select_gpio(22);
		esp_rom_gpio_pad_select_gpio(23);

		IO_MUX.gpio[20].val = 0;
		IO_MUX.gpio[20].mcu_sel = 1;
		IO_MUX.gpio[20].fun_ie = 1;
		IO_MUX.gpio[20].fun_wpu = 0;
		IO_MUX.gpio[20].fun_wpd = 0;

		IO_MUX.gpio[21].val = 0;
		IO_MUX.gpio[21].mcu_sel = 1;
		IO_MUX.gpio[21].fun_ie = 1;
		IO_MUX.gpio[21].fun_wpu = 0;
		IO_MUX.gpio[21].fun_wpd = 0;

		IO_MUX.gpio[22].val = 0;
		IO_MUX.gpio[22].mcu_sel = 1;
		IO_MUX.gpio[22].fun_ie = 1;
		IO_MUX.gpio[22].fun_wpu = 0;
		IO_MUX.gpio[22].fun_wpd = 0;

		IO_MUX.gpio[23].val = 0;
		IO_MUX.gpio[23].mcu_sel = 1;
		IO_MUX.gpio[23].fun_ie = 1;
		IO_MUX.gpio[23].fun_wpu = 0;
		IO_MUX.gpio[23].fun_wpd = 0;

		/* Identify board to select I2S DIN pin:
		 * On Ceres (MAC 0x6c) and Pallas (0x17), the cross-connection wire is on GPIO 23.
		 * On Spider (0xc7) and Aphid (0x15) DUT bridges, DUT DOUT connects to GPIO 20.
		 */
		uint8_t mac[6] = {0};
		extern int esp_efuse_mac_get_default(uint8_t *mac);
		esp_efuse_mac_get_default(mac);
		int din_pin = (mac[5] == 0xc7 || mac[5] == 0x15) ? 20 : 23;

		esp_rom_gpio_connect_in_signal(21, I2S0_I_BCK_PAD_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(21, I2S0_O_BCK_PAD_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(22, I2S0_I_WS_PAD_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(22, I2S0_O_WS_PAD_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(din_pin, I2S0_I_SD_PAD_IN_IDX, false);

		LOG_INF("DAI I2S set to SLAVE mode (external BCLK/WS on G21/G22, DIN on G%d)", din_pin);
#endif
	} else {
#if defined(CONFIG_SOC_SERIES_ESP32P4)
		_i2s_ll_mclk_bind_to_tx_clk(&I2S0);
#else
		i2s_ll_mclk_bind_to_tx_clk(&I2S0);
#endif

#if defined(CONFIG_SOC_SERIES_ESP32C6)
		/* ESP32-C6 Master GPIO Matrix:
		 * BCLK Out: GPIO 18 <- I2SO_BCK_OUT_IDX (13) & connects internal I2SI_BCK_IN_IDX (16)
		 * WS Out:   GPIO 19 <- I2SO_WS_OUT_IDX (14) & connects internal I2SI_WS_IN_IDX (17)
		 * DOUT Out: GPIO 20 <- I2SO_SD_OUT_IDX (15)
		 * DIN In:   GPIO 21 -> I2SI_SD_IN_IDX (15)
		 */
		const uint8_t c6_pins[] = {18, 19, 20, 21};
		for (int i = 0; i < 4; i++) {
			uint8_t p = c6_pins[i];
			GPIO.func_out_sel_cfg[p].oen_sel = 1;
			GPIO.func_out_sel_cfg[p].oen_inv_sel = 0;
			GPIO.pin[p].pad_driver = 0;
			esp_rom_gpio_pad_unhold(p);
			esp_rom_gpio_pad_select_gpio(p);
			PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[p]);
			esp_rom_gpio_pad_set_drv(p, 3);
		}

		I2S0.tx_conf.tx_slave_mod = 0;
		I2S0.rx_conf.rx_slave_mod = 0;
		I2S0.tx_conf.sig_loopback = 0;

		esp_rom_gpio_connect_out_signal(18, I2SO_BCK_OUT_IDX, false, false);
		esp_rom_gpio_connect_in_signal(18, I2SI_BCK_IN_IDX, false);
		GPIO.func_out_sel_cfg[18].out_sel = I2SO_BCK_OUT_IDX;

		esp_rom_gpio_connect_out_signal(19, I2SO_WS_OUT_IDX, false, false);
		esp_rom_gpio_connect_in_signal(19, I2SI_WS_IN_IDX, false);
		GPIO.func_out_sel_cfg[19].out_sel = I2SO_WS_OUT_IDX;

		esp_rom_gpio_connect_out_signal(20, I2SO_SD_OUT_IDX, false, false);
		GPIO.func_out_sel_cfg[20].out_sel = I2SO_SD_OUT_IDX;

		esp_rom_gpio_connect_in_signal(21, I2SI_SD_IN_IDX, false);
		GPIO.func_out_sel_cfg[21].out_sel = SIG_GPIO_OUT_IDX;

		GPIO.enable_w1ts.val = (1 << 18) | (1 << 19) | (1 << 20);
		GPIO.enable_w1tc.val = (1 << 21);

		LOG_INF("DAI I2S set to MASTER mode on ESP32-C6 (BCLK:G18, WS:G19, DOUT:G20, DIN:G21)");
#elif defined(CONFIG_SOC_SERIES_ESP32S3)
		/* ESP32-S3 Master GPIO Matrix:
		 * BCLK Out: GPIO 18 (D9)  <- I2S0O_BCK_OUT_IDX (22) & connects internal I2S0I_BCK_IN_IDX (26)
		 * WS Out:   GPIO 21 (D10) <- I2S0O_WS_OUT_IDX (24) & connects internal I2S0I_WS_IN_IDX (27)
		 * DOUT Out: GPIO 7  (D4)  <- I2S0O_SD_OUT_IDX (25)
		 * DIN In:   GPIO 8  (D5)  -> I2S0I_SD_IN_IDX (25)
		 */
		const uint8_t s3_pins[] = {18, 21, 7, 8};
		for (int i = 0; i < 4; i++) {
			uint8_t p = s3_pins[i];
			GPIO.func_out_sel_cfg[p].func_sel = SIG_GPIO_OUT_IDX;
			GPIO.func_out_sel_cfg[p].oen_sel = 1;
			GPIO.func_out_sel_cfg[p].oen_inv_sel = 0;
			GPIO.pin[p].pad_driver = 0;
			esp_rom_gpio_pad_unhold(p);
			esp_rom_gpio_pad_select_gpio(p);
			PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[p]);
			esp_rom_gpio_pad_set_drv(p, 3);
		}

		I2S0.tx_conf.tx_slave_mod = 0;
		I2S0.rx_conf.rx_slave_mod = 0;
		I2S0.tx_conf.sig_loopback = 0;

		esp_rom_gpio_connect_out_signal(18, I2S0O_BCK_OUT_IDX, false, false);
		esp_rom_gpio_connect_in_signal(18, I2S0I_BCK_IN_IDX, false);
		GPIO.func_out_sel_cfg[18].func_sel = I2S0O_BCK_OUT_IDX;

		esp_rom_gpio_connect_out_signal(21, I2S0O_WS_OUT_IDX, false, false);
		esp_rom_gpio_connect_in_signal(21, I2S0I_WS_IN_IDX, false);
		GPIO.func_out_sel_cfg[21].func_sel = I2S0O_WS_OUT_IDX;

		esp_rom_gpio_connect_out_signal(7, I2S0O_SD_OUT_IDX, false, false);
		GPIO.func_out_sel_cfg[7].func_sel = I2S0O_SD_OUT_IDX;

		esp_rom_gpio_connect_in_signal(8, I2S0I_SD_IN_IDX, false);
		GPIO.func_out_sel_cfg[8].func_sel = SIG_GPIO_OUT_IDX;

		GPIO.enable_w1ts = (1 << 18) | (1 << 21) | (1 << 7);
		GPIO.enable_w1tc = (1 << 8);

		LOG_INF("DAI I2S set to MASTER mode on ESP32-S3 (BCLK:G18/D9, WS:G21/D10, DOUT:G7/D4, DIN:G8/D5)");
#else
		/* Master Mode: Drive BCLK (GPIO 21), WS (GPIO 22), DOUT (both GPIO 20 and GPIO 23), DIN (GPIO 23/20) */
		uint8_t mac[6] = {0};
		extern int esp_efuse_mac_get_default(uint8_t *mac);
		esp_efuse_mac_get_default(mac);
		int din_pin = (mac[5] == 0xc7 || mac[5] == 0x15) ? 20 : 23;

		I2S0.tx_conf.tx_slave_mod = 0;
		I2S0.rx_conf.rx_slave_mod = 0;
		I2S0.tx_conf.sig_loopback = 0;

		esp_rom_gpio_connect_out_signal(21, I2S0_O_BCK_PAD_OUT_IDX, false, false);
		esp_rom_gpio_connect_out_signal(22, I2S0_O_WS_PAD_OUT_IDX, false, false);
		esp_rom_gpio_connect_out_signal(20, I2S0_O_SD_PAD_OUT_IDX, false, false);
		esp_rom_gpio_connect_out_signal(23, I2S0_O_SD_PAD_OUT_IDX, false, false);
		esp_rom_gpio_connect_in_signal(din_pin, I2S0_I_SD_PAD_IN_IDX, false);

		GPIO.func_out_sel_cfg[21].out_sel = 25; // I2S0_O_BCK_OUT
		GPIO.func_out_sel_cfg[21].oen_sel = 1;
		GPIO.func_out_sel_cfg[21].oen_inv_sel = 0;

		GPIO.func_out_sel_cfg[22].out_sel = 27; // I2S0_O_WS_OUT
		GPIO.func_out_sel_cfg[22].oen_sel = 1;
		GPIO.func_out_sel_cfg[22].oen_inv_sel = 0;

		GPIO.func_out_sel_cfg[20].out_sel = 28; // I2S0_O_SD_OUT
		GPIO.func_out_sel_cfg[20].oen_sel = 1;
		GPIO.func_out_sel_cfg[20].oen_inv_sel = 0;

		GPIO.func_out_sel_cfg[23].out_sel = 28; // I2S0_O_SD_OUT
		GPIO.func_out_sel_cfg[23].oen_sel = 1;
		GPIO.func_out_sel_cfg[23].oen_inv_sel = 0;

		GPIO.enable_w1ts.val = (1 << 20) | (1 << 21) | (1 << 22) | (1 << 23);

		GPIO.pin[20].pad_driver = 0;
		GPIO.pin[21].pad_driver = 0;
		GPIO.pin[22].pad_driver = 0;
		GPIO.pin[23].pad_driver = 0;

		esp_rom_gpio_pad_unhold(20);
		esp_rom_gpio_pad_unhold(21);
		esp_rom_gpio_pad_unhold(22);
		esp_rom_gpio_pad_unhold(23);

		esp_rom_gpio_pad_select_gpio(20);
		esp_rom_gpio_pad_select_gpio(21);
		esp_rom_gpio_pad_select_gpio(22);
		esp_rom_gpio_pad_select_gpio(23);

		IO_MUX.gpio[20].val = 0;
		IO_MUX.gpio[20].mcu_sel = 1;
		IO_MUX.gpio[20].fun_ie = 1;
		IO_MUX.gpio[20].fun_wpu = 0;
		IO_MUX.gpio[20].fun_wpd = 0;
		esp_rom_gpio_pad_set_drv(20, 3);

		IO_MUX.gpio[21].val = 0;
		IO_MUX.gpio[21].mcu_sel = 1;
		IO_MUX.gpio[21].fun_ie = 1;
		esp_rom_gpio_pad_set_drv(21, 3);

		IO_MUX.gpio[22].val = 0;
		IO_MUX.gpio[22].mcu_sel = 1;
		IO_MUX.gpio[22].fun_ie = 1;
		esp_rom_gpio_pad_set_drv(22, 3);

		IO_MUX.gpio[23].val = 0;
		IO_MUX.gpio[23].mcu_sel = 1;
		IO_MUX.gpio[23].fun_ie = 1;
		esp_rom_gpio_pad_set_drv(23, 3);
#endif

		/* Initialize master clock generator (starts when stream triggers) */
		I2S0.tx_conf.tx_start = 0;

		LOG_INF("DAI I2S set to MASTER mode (XTAL 40M: N=%u,x=%u,y=%u,z=%u,yn1=%u, bclk_div=%u, BCLK=%u Hz, WS=%u Hz)",
			div_int, div_x, div_y, div_z, div_yn1, bclk_div, bclk, data->sample_rate);
	}

	/* Latch clock divider and configuration registers into hardware clock domains */
	I2S0.tx_conf.tx_update = 1;
	I2S0.rx_conf.rx_update = 1;
	int timeout = 1000;
	while ((I2S0.tx_conf.tx_update || I2S0.rx_conf.rx_update) && --timeout > 0) {
		k_busy_wait(1);
	}
#if defined(CONFIG_SOC_SERIES_ESP32P4)
	LOG_INF("DAI I2S config_set complete (tx_upd=%u, rx_upd=%u, rem_timeout=%d, rx_conf=0x%08x, peri12=0x%08x)",
		(unsigned int)I2S0.tx_conf.tx_update,
		(unsigned int)I2S0.rx_conf.rx_update,
		timeout,
		(unsigned int)I2S0.rx_conf.val,
		(unsigned int)HP_SYS_CLKRST.peri_clk_ctrl12.val);
#else
	LOG_INF("DAI I2S config_set complete (tx_upd=%u, rx_upd=%u, rem_timeout=%d, rx_conf=0x%08x)",
		(unsigned int)I2S0.tx_conf.tx_update,
		(unsigned int)I2S0.rx_conf.rx_update,
		timeout,
		(unsigned int)I2S0.rx_conf.val);
#endif

	data->tx_state = DAI_STATE_READY;
	data->rx_state = DAI_STATE_READY;

	k_spin_unlock(&data->lock, key);
	return 0;
}

static int dai_esp32_i2s_config_get(const struct device *dev,
				    struct dai_config *cfg,
				    enum dai_dir dir)
{
	struct dai_esp32_i2s_data *data = dev->data;
	*cfg = data->cfg;
	return 0;
}

static int dai_esp32_i2s_trigger(const struct device *dev,
				 enum dai_dir dir,
				 enum dai_trigger_cmd cmd)
{
	struct dai_esp32_i2s_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (cmd == DAI_TRIGGER_COPY) {
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	LOG_INF("DAI I2S trigger: dir=%d, cmd=%d", dir, cmd);

	switch (cmd) {
	case DAI_TRIGGER_PREPARE:
	case DAI_TRIGGER_RESET:
	case DAI_TRIGGER_DRAIN:
	case DAI_TRIGGER_PRE_START:
		break;
	case DAI_TRIGGER_START:
		if (dir == DAI_DIR_TX || dir == DAI_DIR_BOTH) {
			if (!I2S0.tx_conf.tx_start) {
				I2S0.tx_conf.tx_reset = 1;
				I2S0.tx_conf.tx_reset = 0;
			}
			I2S0.tx_conf.tx_fifo_reset = 1;
			I2S0.tx_conf.tx_fifo_reset = 0;
			I2S0.tx_conf.tx_update = 1;
			int timeout_tx = 1000;
			while (I2S0.tx_conf.tx_update && --timeout_tx > 0) {
				k_busy_wait(1);
			}

			if (data->is_slave) {
#if defined(CONFIG_SOC_SERIES_ESP32S3)
				const int ws_pin = 21;
#elif defined(CONFIG_SOC_SERIES_ESP32C6)
				const int ws_pin = 19;
#elif defined(CONFIG_SOC_SERIES_ESP32P4)
				const int ws_pin = 22;
#else
				const int ws_pin = -1;
#endif
				if (ws_pin >= 0) {
					int t = 10000;
					while (!gpio_ll_get_level(&GPIO, ws_pin) && --t > 0) {
					}
					if (t > 0) {
						t = 10000;
						while (gpio_ll_get_level(&GPIO, ws_pin) && --t > 0) {
						}
					}
				}
			}

			I2S0.tx_conf.tx_start = 1;
			data->tx_state = DAI_STATE_RUNNING;
#if defined(CONFIG_SOC_SERIES_ESP32P4)
			LOG_INF("[DAI START] bck_cnt=%u, tx_start=%u, tx_update=%u, peri13=0x%08x, peri14=0x%08x | GPIO21: out=%u, oen=%u, en=%u, iomux=0x%08x",
				(unsigned int)I2S0.bck_cnt.tx_bck_cnt,
				(unsigned int)I2S0.tx_conf.tx_start,
				(unsigned int)I2S0.tx_conf.tx_update,
				(unsigned int)HP_SYS_CLKRST.peri_clk_ctrl13.val,
				(unsigned int)HP_SYS_CLKRST.peri_clk_ctrl14.val,
				(unsigned int)GPIO.func_out_sel_cfg[21].out_sel,
				(unsigned int)GPIO.func_out_sel_cfg[21].oen_sel,
				(unsigned int)((GPIO.enable.val >> 21) & 1),
				(unsigned int)IO_MUX.gpio[21].val);
#else
			LOG_INF("[DAI START] tx_start=%u, tx_update=%u",
				(unsigned int)I2S0.tx_conf.tx_start,
				(unsigned int)I2S0.tx_conf.tx_update);
#endif
		}
		if (dir == DAI_DIR_RX || dir == DAI_DIR_BOTH) {
			if (!I2S0.rx_conf.rx_start) {
				I2S0.rx_conf.rx_reset = 1;
				I2S0.rx_conf.rx_reset = 0;
			}
			I2S0.rx_conf.rx_fifo_reset = 1;
			I2S0.rx_conf.rx_fifo_reset = 0;
			I2S0.rx_conf.rx_update = 1;
			int timeout_rx = 1000;
			while (I2S0.rx_conf.rx_update && --timeout_rx > 0) {
				k_busy_wait(1);
			}

			if (data->is_slave) {
				/* In slave mode with external continuous BCLK/WS,
				 * synchronize the RX start to the falling edge of WS (1 -> 0 transition).
				 * In Philips I2S (rx_ws_idle_pol = 0), WS=0 indicates Left channel
				 * and WS=1 indicates Right channel. Asserting rx_start immediately
				 * on the falling edge ensures Slot 0 receives Left channel first,
				 * eliminating the 1-slot channel swap ambiguity.
				 *
				 * If the clock is not running yet, the timeout expires cleanly
				 * and rx_start is asserted.
				 */
#if defined(CONFIG_SOC_SERIES_ESP32S3)
				const int ws_pin = 21;
#elif defined(CONFIG_SOC_SERIES_ESP32C6)
				const int ws_pin = 19;
#elif defined(CONFIG_SOC_SERIES_ESP32P4)
				const int ws_pin = 22;
#else
				const int ws_pin = -1;
#endif
				if (ws_pin >= 0) {
					/* Step 1: Wait for WS to be 1 (Right channel) */
					int t = 10000;
					while (!gpio_ll_get_level(&GPIO, ws_pin) && --t > 0) {
					}
					/* Step 2: Wait for WS falling edge to 0 (Left channel start) */
					if (t > 0) {
						t = 10000;
						while (gpio_ll_get_level(&GPIO, ws_pin) && --t > 0) {
						}
					}
				}
			} else {
				/* In master mode, start TX clock if not already running so BCLK/WS are driven */
				if (!I2S0.tx_conf.tx_start) {
					I2S0.tx_conf.tx_start = 1;
					I2S0.tx_conf.tx_update = 1;
				}
			}

			I2S0.rx_conf.rx_start = 1;
			data->rx_state = DAI_STATE_RUNNING;
#if defined(CONFIG_SOC_SERIES_ESP32P4)
			LOG_INF("[DAI RX START] rx_start=%u, rx_upd=%u, rx_slave_mod=%u, rx_eof=%u | in_sig28=%u, G20_out=%u, G23_out=%u, en20=%u, en23=%u",
				(unsigned int)I2S0.rx_conf.rx_start,
				(unsigned int)I2S0.rx_conf.rx_update,
				(unsigned int)I2S0.rx_conf.rx_slave_mod,
				(unsigned int)I2S0.rx_eof_num.rx_eof_num,
				(unsigned int)GPIO.func_in_sel_cfg[28].in_sel,
				(unsigned int)GPIO.func_out_sel_cfg[20].out_sel,
				(unsigned int)GPIO.func_out_sel_cfg[23].out_sel,
				(unsigned int)((GPIO.enable.val >> 20) & 1),
				(unsigned int)((GPIO.enable.val >> 23) & 1));
#else
			LOG_INF("[DAI RX START] rx_start=%u, rx_upd=%u, rx_slave_mod=%u, rx_eof=%u",
				(unsigned int)I2S0.rx_conf.rx_start,
				(unsigned int)I2S0.rx_conf.rx_update,
				(unsigned int)I2S0.rx_conf.rx_slave_mod,
				(unsigned int)I2S0.rx_eof_num.rx_eof_num);
#endif
		}
		break;
	case DAI_TRIGGER_STOP:
	case DAI_TRIGGER_POST_STOP:
	case DAI_TRIGGER_PAUSE:
	case DAI_TRIGGER_DROP:
		if (dir == DAI_DIR_TX || dir == DAI_DIR_BOTH) {
#if defined(CONFIG_SOC_SERIES_ESP32P4)
			LOG_INF("[DAI STOP] bck_cnt=%u, tx_start=%u",
				(unsigned int)I2S0.bck_cnt.tx_bck_cnt,
				(unsigned int)I2S0.tx_conf.tx_start);
#else
			LOG_INF("[DAI STOP] tx_start=%u",
				(unsigned int)I2S0.tx_conf.tx_start);
#endif
			I2S0.tx_conf.tx_start = 0;
			I2S0.tx_conf.tx_update = 1;
			data->tx_state = DAI_STATE_READY;
		}
		if (dir == DAI_DIR_RX || dir == DAI_DIR_BOTH) {
			I2S0.rx_conf.rx_start = 0;
			I2S0.rx_conf.rx_update = 1;
			if (!data->is_slave && data->tx_state != DAI_STATE_RUNNING) {
				I2S0.tx_conf.tx_start = 0;
				I2S0.tx_conf.tx_update = 1;
			}
			data->rx_state = DAI_STATE_READY;
		}
		break;
	default:
		k_spin_unlock(&data->lock, key);
		return -EINVAL;
	}

	k_spin_unlock(&data->lock, key);
	return 0;
}

static const struct dai_properties *dai_esp32_i2s_get_properties(const struct device *dev,
								 enum dai_dir dir,
								 int stream_id)
{
	const struct dai_esp32_i2s_config *cfg = dev->config;
	struct dai_esp32_i2s_data *data = dev->data;

	if (dir == DAI_DIR_TX) {
		data->tx_props.fifo_address = (uint32_t)&I2S0.tx_conf;
		data->tx_props.fifo_depth = cfg->fifo_depth;
		data->tx_props.dma_hs_id = ESP_GDMA_TRIG_PERIPH_I2S0;
		return &data->tx_props;
	} else if (dir == DAI_DIR_RX) {
		data->rx_props.fifo_address = (uint32_t)&I2S0.rx_conf;
		data->rx_props.fifo_depth = cfg->fifo_depth;
		data->rx_props.dma_hs_id = ESP_GDMA_TRIG_PERIPH_I2S0;
		return &data->rx_props;
	}

	return NULL;
}

static DEVICE_API(dai, dai_esp32_i2s_driver_api) = {
	.probe = dai_esp32_i2s_probe,
	.remove = dai_esp32_i2s_remove,
	.config_set = dai_esp32_i2s_config_set,
	.config_get = dai_esp32_i2s_config_get,
	.get_properties = dai_esp32_i2s_get_properties,
	.trigger = dai_esp32_i2s_trigger,
};

static int dai_esp32_i2s_init(const struct device *dev)
{
	const struct dai_esp32_i2s_config *cfg = dev->config;
	int ret;

#if defined(CONFIG_SOC_SERIES_ESP32P4)
	HP_SYS_CLKRST.soc_clk_ctrl2.reg_i2s0_apb_clk_en = 1;
#elif defined(CONFIG_SOC_SERIES_ESP32C6)
	PCR.i2s_conf.i2s_clk_en = 1;
	PCR.i2s_conf.i2s_rst_en = 0;
#elif defined(CONFIG_SOC_SERIES_ESP32S3)
	SYSTEM.perip_clk_en0.i2s0_clk_en = 1;
	SYSTEM.perip_rst_en0.i2s0_rst = 0;
#endif
#if defined(CONFIG_SOC_SERIES_ESP32P4)
	I2S0.clk_gate.clk_en = 1;
#endif

	if (cfg->pincfg) {
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0 && ret != -ENOENT) {
			LOG_ERR("Failed to apply pinctrl (%d)", ret);
			return ret;
		}
	}

	return 0;
}

#define DAI_ESP32_I2S_DEVICE_INIT(inst)								\
	PINCTRL_DT_INST_DEFINE(inst);								\
												\
	static const struct dai_esp32_i2s_config dai_esp32_i2s_config_##inst = {		\
		.reg_base = DT_INST_REG_ADDR(inst),						\
		.irq = DT_INST_IRQN(inst),							\
		.dai_index = DT_INST_PROP_OR(inst, dai_index, 0),				\
		.fifo_depth = DT_INST_PROP_OR(inst, fifo_depth, 32),				\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),					\
	};											\
												\
	static struct dai_esp32_i2s_data dai_esp32_i2s_data_##inst = {				\
		.cfg.type = DAI_ESP32_I2S,							\
		.cfg.dai_index = DT_INST_PROP_OR(inst, dai_index, 0),				\
		.tx_state = DAI_STATE_NOT_READY,						\
		.rx_state = DAI_STATE_NOT_READY,						\
	};											\
												\
	DEVICE_DT_INST_DEFINE(inst,								\
			      dai_esp32_i2s_init,						\
			      NULL,								\
			      &dai_esp32_i2s_data_##inst,					\
			      &dai_esp32_i2s_config_##inst,					\
			      POST_KERNEL,							\
			      CONFIG_DAI_INIT_PRIORITY,						\
			      &dai_esp32_i2s_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAI_ESP32_I2S_DEVICE_INIT)

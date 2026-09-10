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
#include <soc/hp_sys_clkrst_struct.h>
#include <soc/lp_clkrst_struct.h>
#include <soc/gpio_struct.h>
#include <soc/io_mux_struct.h>
#include <soc/gpio_sig_map.h>
#include <hal/i2s_ll.h>
#include <esp_rom_gpio.h>
#include <zephyr/drivers/dma/dma_esp32.h>
#include <soc/rtc.h>
#include <soc/lpperi_struct.h>

LOG_MODULE_REGISTER(dai_esp32_pdm, CONFIG_DAI_LOG_LEVEL);

#define DT_DRV_COMPAT espressif_esp32_dai_pdm

struct dai_esp32_pdm_config {
	uint32_t reg_base;
	uint32_t irq;
	uint32_t dai_index;
	uint32_t fifo_depth;
	const struct pinctrl_dev_config *pincfg;
};

struct dai_esp32_pdm_data {
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

static int dai_esp32_pdm_probe(const struct device *dev)
{
	const struct dai_esp32_pdm_config *cfg = dev->config;
	LOG_INF("ESP32 DAI PDM probe (dai_index=%u)", cfg->dai_index);
	return 0;
}

static int dai_esp32_pdm_remove(const struct device *dev)
{
	return 0;
}

static void pdm_hw_configure(bool is_pdm1, bool is_slave, uint32_t sample_rate,
			     uint8_t channels, uint8_t word_size, size_t block_size)
{
	ARG_UNUSED(is_pdm1);

	uint32_t bytes_per_sample = (word_size > 16) ? 4 : 2;
	uint32_t num_channels = channels ? channels : 2;
	uint32_t rate = sample_rate ? sample_rate : 48000;
	uint32_t frames_per_period = rate / 1000;
	if (frames_per_period == 0) {
		frames_per_period = 48;
	}
	uint32_t period_bytes = block_size ? block_size : (frames_per_period * num_channels * bytes_per_sample);

	/* I2S0 Hardware PDM Configuration */
	I2S0.tx_conf.tx_reset = 1;
	I2S0.tx_conf.tx_reset = 0;
	I2S0.tx_conf.tx_fifo_reset = 1;
	I2S0.tx_conf.tx_fifo_reset = 0;

	/* Enable TX PDM and disable TDM */
	I2S0.tx_conf.tx_pdm_en = 1;
	I2S0.tx_conf.tx_tdm_en = 0;
	I2S0.tx_pcm2pdm_conf.pcm2pdm_conv_en = 1;
	I2S0.tx_pcm2pdm_conf.tx_pdm_dac_mode_en = 0;   /* Digital PDM Codec bitstream mode */
	I2S0.tx_pcm2pdm_conf.tx_pdm_dac_2out_en = (num_channels > 1) ? 1 : 0;
	I2S0.tx_pcm2pdm_conf.tx_pdm_sinc_osr2 = 2;     /* Matched oversampling ratio (fp/fs = 2) */
	I2S0.tx_pcm2pdm_conf.tx_pdm_prescale = 0;
	I2S0.tx_pcm2pdm_conf.tx_pdm_hp_in_shift = 0;   /* I2S_PDM_SIG_SCALING_DIV_2 to prevent saturation */
	I2S0.tx_pcm2pdm_conf.tx_pdm_lp_in_shift = 1;
	I2S0.tx_pcm2pdm_conf.tx_pdm_sinc_in_shift = 1;
	I2S0.tx_pcm2pdm_conf.tx_pdm_sigmadelta_in_shift = 1;
	I2S0.tx_pcm2pdm_conf.tx_pdm_sigmadelta_dither = 0;
	I2S0.tx_pcm2pdm_conf.tx_pdm_sigmadelta_dither2 = 1;
	I2S0.tx_pcm2pdm_conf1.tx_pdm_fp = 960;
	I2S0.tx_pcm2pdm_conf1.tx_pdm_fs = 480;
	I2S0.tx_pcm2pdm_conf1.tx_iir_hp_mult12_5 = 7;
	I2S0.tx_pcm2pdm_conf1.tx_iir_hp_mult12_0 = 6;

	I2S0.tx_conf.tx_mono = (num_channels == 1) ? 1 : 0;
	I2S0.tx_conf.tx_chan_mod = 0;
	I2S0.tx_conf.tx_ws_idle_pol = 0;
	I2S0.tx_conf.tx_slave_mod = is_slave ? 1 : 0;
	I2S0.tx_conf.tx_stop_en = 0;

	/* Channel active masks */
	I2S0.tx_tdm_ctrl.tx_tdm_chan0_en = 1;
	I2S0.tx_tdm_ctrl.tx_tdm_chan1_en = (num_channels > 1) ? 1 : 0;
	I2S0.tx_tdm_ctrl.tx_tdm_tot_chan_num = (num_channels > 1) ? 1 : 0;

	/* PDM TX sample configuration: 32-bit slot width (tx_bits_mod = 31, tx_tdm_chan_bits = 31)
	 * to satisfy frequency and consume 64 bits per stereo frame at 6.144 MHz -> exact 48 kHz decimation -> 0xf87fc000 */
	I2S0.tx_conf1.val = 0xf87fc000;

	/* PDM RX Configuration */
	I2S0.rx_conf.rx_reset = 1;
	I2S0.rx_conf.rx_reset = 0;
	I2S0.rx_conf.rx_fifo_reset = 1;
	I2S0.rx_conf.rx_fifo_reset = 0;

	I2S0.rx_conf.rx_pdm_en = 1;
	I2S0.rx_conf.rx_tdm_en = 0;
	I2S0.rx_conf.rx_slave_mod = is_slave ? 1 : 0;
	I2S0.rx_conf.rx_pcm_bypass = 1;
	I2S0.rx_conf1.val = 0x787bc000;
	I2S0.rx_conf1.rx_tdm_ws_width = 0;
	I2S0.rx_pdm2pcm_conf.rx_pdm2pcm_en = 1;
	I2S0.rx_pdm2pcm_conf.rx_pdm_sinc_dsr_16_en = (rate <= 48000) ? 1 : 0; /* 128x downsampling for <= 48kHz */
	I2S0.rx_pdm2pcm_conf.rx_pdm2pcm_amplify_num = 1;
	I2S0.rx_pdm2pcm_conf.rx_pdm_hp_bypass = 1; /* Bypass HP filter to eliminate step-response ringing/saturation */
	I2S0.rx_pdm2pcm_conf.rx_iir_hp_mult12_5 = 7;
	I2S0.rx_pdm2pcm_conf.rx_iir_hp_mult12_0 = 6;

	/* SD input delay mode:
	 * In Slave RX mode, external clock and data are received concurrently.
	 * Delaying the Serial Data (SD) input by one half-cycle of the fast peripheral
	 * clock (pos edge) aligns the data setup and hold times for both rising and
	 * falling clock edges, eliminating single-channel distortion / negative excursions.
	 */
	I2S0.rx_timing.rx_sd_in_dm = is_slave ? 1 : 0;

	I2S0.rx_tdm_ctrl.rx_tdm_pdm_chan0_en = 1;
	I2S0.rx_tdm_ctrl.rx_tdm_pdm_chan1_en = (num_channels > 1) ? 1 : 0;
	I2S0.rx_tdm_ctrl.rx_tdm_tot_chan_num = (num_channels > 1) ? 1 : 0;

	/* Hardware EOF generates interrupt when period_bytes have transferred.
	 * In ESP32-P4 GDMA/I2S hardware, rx_eof_num is the byte count to trigger
	 * in_suc_eof: rx_eof_num = period_bytes - 1.
	 */
	I2S0.rx_eof_num.rx_eof_num = period_bytes > 0 ? (period_bytes - 1) : 0;

	I2S0.tx_conf.tx_update = 1;
	I2S0.rx_conf.rx_update = 1;
}

static int dai_esp32_pdm_config_set(const struct device *dev,
				    const struct dai_config *cfg,
				    const void *bespoke_cfg,
				    size_t size)
{
	const struct dai_esp32_pdm_config *dev_cfg = dev->config;
	struct dai_esp32_pdm_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	bool is_pdm1 = (dev_cfg->dai_index == 2);

	LOG_INF("DAI PDM%d config_set: rate=%u, format=0x%x, channels=%u, block_size=%zu",
		is_pdm1 ? 1 : 0, cfg->rate, cfg->format, cfg->channels, cfg->block_size);

	data->cfg = *cfg;
	data->sample_rate = cfg->rate ? cfg->rate : 48000;
	data->channels = cfg->channels ? cfg->channels : 2;
	data->word_size = cfg->word_size ? cfg->word_size : 16;

	enum dai_clock_provider provider = cfg->format & DAI_FORMAT_CLOCK_PROVIDER_MASK;
	if (provider == DAI_CBP_CFP || provider == DAI_CBP_CFC) {
		data->is_slave = true;
	} else {
		data->is_slave = false;
	}

	pdm_hw_configure(is_pdm1, data->is_slave, data->sample_rate,
			 data->channels, data->word_size, cfg->block_size);

	/* Enable APB and peripheral clocks in HP_SYS_CLKRST and LP_AON_CLKRST */
	HP_SYS_CLKRST.soc_clk_ctrl2.reg_i2s0_apb_clk_en = 1;
	LP_AON_CLKRST.hp_clk_ctrl.hp_pad_i2s0_mclk_en = 1;

	/* Configure APLL for high-precision 98.304 MHz audio clock.
	 * APLL = 98.304000 MHz (sdm0=149, sdm1=212, sdm2=5, o_div=0).
	 * Master TX: N = 2 integer divider (49.152 MHz MCLK) + bclk_div = 8 -> exact 6.144000 MHz BCLK.
	 * Slave RX:  N = 4 integer divider (24.576 MHz MCLK) + bclk_div = 8.
	 */
	uint32_t o_div = 0, sdm0 = 0, sdm1 = 0, sdm2 = 0;
	uint32_t real_apll = rtc_clk_apll_coeff_calc(98304000, &o_div, &sdm0, &sdm1, &sdm2);
	LPPERI.clk_en.ck_en_lp_i2cmst = 1;
	rtc_clk_apll_enable(true);
	if (real_apll) {
		rtc_clk_apll_coeff_set(o_div, sdm0, sdm1, sdm2);
	}
	LOG_INF("APLL calc: real=%u, o_div=%u, sdm0=%u, sdm1=%u, sdm2=%u",
		real_apll, o_div, sdm0, sdm1, sdm2);

	HP_SYS_CLKRST.peri_clk_ctrl11.reg_i2s0_rx_clk_src_sel = 1; /* APLL */
	HP_SYS_CLKRST.peri_clk_ctrl11.reg_i2s0_rx_clk_en = 1;
	HP_SYS_CLKRST.peri_clk_ctrl13.reg_i2s0_tx_clk_src_sel = 1; /* APLL */
	HP_SYS_CLKRST.peri_clk_ctrl13.reg_i2s0_tx_clk_en = 1;
	I2S0.clk_gate.clk_en = 1;

	/* Configure MCLK and BCLK dividers:
	 * - Master TX: MCLK div = 2 (49.152 MHz) + BCLK div = 8 (6.144 MHz).
	 * - Slave RX:  MCLK div = 2 (49.152 MHz) + BCLK div = 8 (internal engine rate matches Master).
	 *   Decimation filter downsamples MCLK / 512 = exact 48000 Hz sample rate.
	 */
	if (data->is_slave) {
		i2s_ll_rx_set_raw_clk_div(&I2S0, 2, 1, 1, 0, 0);
		i2s_ll_tx_set_raw_clk_div(&I2S0, 2, 1, 1, 0, 0);
		uint32_t bclk_div = 8;
		I2S0.rx_conf.rx_bck_div_num = bclk_div - 1;
		I2S0.tx_conf.tx_bck_div_num = bclk_div - 1;
	} else {
		i2s_ll_tx_set_raw_clk_div(&I2S0, 2, 1, 1, 0, 0);
		i2s_ll_rx_set_raw_clk_div(&I2S0, 2, 1, 1, 0, 0);
		uint32_t bclk_div = 8;
		I2S0.tx_conf.tx_bck_div_num = bclk_div - 1;
		I2S0.rx_conf.rx_bck_div_num = bclk_div - 1;
	}

	/* Ensure loopback is disabled */
	I2S0.tx_conf.sig_loopback = 0;

	if (data->is_slave) {
		_i2s_ll_mclk_bind_to_rx_clk(&I2S0);
	} else {
		_i2s_ll_mclk_bind_to_tx_clk(&I2S0);
	}

	/* PDM0 GPIO routing:
	 * Pallas (Master): drives PDM CLK on GPIO 4 (Pin 18) and PDM DOUT on BOTH GPIO 3 & 5 (Pin 16 & 19)
	 * Ceres (Slave): receives PDM CLK on GPIO 4 (Pin 18) and PDM DIN on GPIO 5 (or GPIO 3)
	 */
	if (data->is_slave) {
		/* Slave Mode: Clock In on GPIO 4, Data In on GPIO 5 (or GPIO 3) */
		GPIO.func_out_sel_cfg[3].out_sel = 256;
		GPIO.func_out_sel_cfg[3].oen_sel = 1;
		GPIO.func_out_sel_cfg[4].out_sel = 256;
		GPIO.func_out_sel_cfg[4].oen_sel = 1;
		GPIO.func_out_sel_cfg[5].out_sel = 256;
		GPIO.func_out_sel_cfg[5].oen_sel = 1;
		GPIO.enable_w1tc.val = (1 << 3) | (1 << 4) | (1 << 5);

		esp_rom_gpio_pad_unhold(3);
		esp_rom_gpio_pad_unhold(4);
		esp_rom_gpio_pad_unhold(5);
		esp_rom_gpio_pad_select_gpio(3);
		esp_rom_gpio_pad_select_gpio(4);
		esp_rom_gpio_pad_select_gpio(5);

		IO_MUX.gpio[3].val = 0;
		IO_MUX.gpio[3].mcu_sel = 1;
		IO_MUX.gpio[3].fun_ie = 1;
		IO_MUX.gpio[3].fun_wpu = 0;
		IO_MUX.gpio[3].fun_wpd = 0;

		IO_MUX.gpio[4].val = 0;
		IO_MUX.gpio[4].mcu_sel = 1;
		IO_MUX.gpio[4].fun_ie = 1;
		IO_MUX.gpio[4].fun_wpu = 0;
		IO_MUX.gpio[4].fun_wpd = 0;

		IO_MUX.gpio[5].val = 0;
		IO_MUX.gpio[5].mcu_sel = 1;
		IO_MUX.gpio[5].fun_ie = 1;
		IO_MUX.gpio[5].fun_wpu = 0;
		IO_MUX.gpio[5].fun_wpd = 0;

		/* Connect both I2S0_I_WS_PAD_IN_IDX (Signal 30) and I2S0_I_BCK_PAD_IN_IDX (Signal 29) to GPIO 4 (PDM CLK in) */
		esp_rom_gpio_connect_in_signal(4, I2S0_I_WS_PAD_IN_IDX, false);
		esp_rom_gpio_connect_in_signal(4, I2S0_I_BCK_PAD_IN_IDX, false);

		/* Connect I2S0_I_SD_PAD_IN_IDX (Signal 28) to GPIO 5 (connected to Pallas GPIO 3 via orange wire) */
		int din_pin = 5;
		esp_rom_gpio_connect_in_signal(din_pin, I2S0_I_SD_PAD_IN_IDX, false);

		LOG_INF("DAI PDM0 configured in SLAVE mode (CLK on G4, DIN on G%d)", din_pin);
	} else {
		/* Master Mode: Drive PDM CLK (GPIO 4) and PDM DOUT on BOTH GPIO 3 & 5 */
		esp_rom_gpio_connect_out_signal(4, I2S0_O_WS_PAD_OUT_IDX, false, false);
		esp_rom_gpio_connect_out_signal(3, I2S0_O_SD_PAD_OUT_IDX, false, false);
		esp_rom_gpio_connect_out_signal(5, I2S0_O_SD_PAD_OUT_IDX, false, false);

		GPIO.func_out_sel_cfg[4].out_sel = 27; /* I2S0_O_WS_OUT */
		GPIO.func_out_sel_cfg[4].oen_sel = 1;
		GPIO.func_out_sel_cfg[4].oen_inv_sel = 0;

		GPIO.func_out_sel_cfg[3].out_sel = 28; /* I2S0_O_SD_OUT */
		GPIO.func_out_sel_cfg[3].oen_sel = 1;
		GPIO.func_out_sel_cfg[3].oen_inv_sel = 0;

		GPIO.func_out_sel_cfg[5].out_sel = 28; /* I2S0_O_SD_OUT */
		GPIO.func_out_sel_cfg[5].oen_sel = 1;
		GPIO.func_out_sel_cfg[5].oen_inv_sel = 0;

		GPIO.enable_w1ts.val = (1 << 3) | (1 << 4) | (1 << 5);

		esp_rom_gpio_pad_unhold(3);
		esp_rom_gpio_pad_unhold(4);
		esp_rom_gpio_pad_unhold(5);
		esp_rom_gpio_pad_select_gpio(3);
		esp_rom_gpio_pad_select_gpio(4);
		esp_rom_gpio_pad_select_gpio(5);

		IO_MUX.gpio[3].val = 0;
		IO_MUX.gpio[3].mcu_sel = 1;
		IO_MUX.gpio[3].fun_ie = 0;
		esp_rom_gpio_pad_set_drv(3, 3);

		IO_MUX.gpio[4].val = 0;
		IO_MUX.gpio[4].mcu_sel = 1;
		IO_MUX.gpio[4].fun_ie = 0;
		esp_rom_gpio_pad_set_drv(4, 3);

		IO_MUX.gpio[5].val = 0;
		IO_MUX.gpio[5].mcu_sel = 1;
		IO_MUX.gpio[5].fun_ie = 0;
		esp_rom_gpio_pad_set_drv(5, 3);

		/* Start master clock */
		I2S0.tx_conf.tx_start = 1;

		LOG_INF("DAI PDM0 configured in MASTER mode (internal PDM CLK on G4, DOUT on G3 & G5, rate=3.072MHz)");
	}

	I2S0.tx_conf.tx_update = 1;
	I2S0.rx_conf.rx_update = 1;
	int timeout = 1000;
	while ((I2S0.tx_conf.tx_update || I2S0.rx_conf.rx_update) && --timeout > 0) {
		k_busy_wait(1);
	}
	LOG_INF("DAI PDM0 config_set complete (tx_upd=%u, rx_upd=%u, timeout=%d)",
		(unsigned int)I2S0.tx_conf.tx_update,
		(unsigned int)I2S0.rx_conf.rx_update,
		timeout);

	data->tx_state = DAI_STATE_READY;
	data->rx_state = DAI_STATE_READY;

	k_spin_unlock(&data->lock, key);
	return 0;
}

static int dai_esp32_pdm_config_get(const struct device *dev,
				    struct dai_config *cfg,
				    enum dai_dir dir)
{
	struct dai_esp32_pdm_data *data = dev->data;
	*cfg = data->cfg;
	return 0;
}

static int dai_esp32_pdm_trigger(const struct device *dev,
				 enum dai_dir dir,
				 enum dai_trigger_cmd cmd)
{
	const struct dai_esp32_pdm_config *cfg = dev->config;
	struct dai_esp32_pdm_data *data = dev->data;
	bool is_pdm1 = (cfg->dai_index == 2);
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	if (cmd == DAI_TRIGGER_COPY) {
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	LOG_INF("DAI PDM%d trigger: dir=%d, cmd=%d", is_pdm1 ? 1 : 0, dir, cmd);

	switch (cmd) {
	case DAI_TRIGGER_PREPARE:
	case DAI_TRIGGER_RESET:
	case DAI_TRIGGER_DRAIN:
	case DAI_TRIGGER_PRE_START:
		break;
	case DAI_TRIGGER_START:
		if (dir == DAI_DIR_TX || dir == DAI_DIR_BOTH) {
			I2S0.tx_conf.tx_fifo_reset = 1;
			I2S0.tx_conf.tx_fifo_reset = 0;
			I2S0.tx_conf.tx_update = 1;
			int to = 1000;
			while (I2S0.tx_conf.tx_update && --to > 0) {
				k_busy_wait(1);
			}
			I2S0.tx_conf.tx_start = 1;
			I2S0.tx_conf.tx_update = 1;
			LOG_INF("[DAI PDM START TX] tx_start=1");
			data->tx_state = DAI_STATE_RUNNING;
		}
		if (dir == DAI_DIR_RX || dir == DAI_DIR_BOTH) {
			I2S0.rx_conf.rx_fifo_reset = 1;
			I2S0.rx_conf.rx_fifo_reset = 0;
			I2S0.rx_conf.rx_update = 1;
			int to = 1000;
			while (I2S0.rx_conf.rx_update && --to > 0) {
				k_busy_wait(1);
			}
			I2S0.rx_conf.rx_start = 1;
			I2S0.rx_conf.rx_update = 1;
			LOG_INF("[DAI PDM START RX] rx_start=1");
			data->rx_state = DAI_STATE_RUNNING;
		}
		break;
	case DAI_TRIGGER_STOP:
	case DAI_TRIGGER_POST_STOP:
	case DAI_TRIGGER_PAUSE:
	case DAI_TRIGGER_DROP:
		if (dir == DAI_DIR_TX || dir == DAI_DIR_BOTH) {
			if (data->is_slave) {
				I2S0.tx_conf.tx_start = 0;
				I2S0.tx_conf.tx_update = 1;
			}
			LOG_INF("[DAI PDM STOP TX] is_slave=%d", data->is_slave);
			data->tx_state = DAI_STATE_READY;
		}
		if (dir == DAI_DIR_RX || dir == DAI_DIR_BOTH) {
			I2S0.rx_conf.rx_start = 0;
			I2S0.rx_conf.rx_update = 1;
			LOG_INF("[DAI PDM STOP RX]");
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

static const struct dai_properties *dai_esp32_pdm_get_properties(const struct device *dev,
								 enum dai_dir dir,
								 int stream_id)
{
	const struct dai_esp32_pdm_config *cfg = dev->config;
	struct dai_esp32_pdm_data *data = dev->data;

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

static DEVICE_API(dai, dai_esp32_pdm_driver_api) = {
	.probe = dai_esp32_pdm_probe,
	.remove = dai_esp32_pdm_remove,
	.config_set = dai_esp32_pdm_config_set,
	.config_get = dai_esp32_pdm_config_get,
	.get_properties = dai_esp32_pdm_get_properties,
	.trigger = dai_esp32_pdm_trigger,
};

static int dai_esp32_pdm_init(const struct device *dev)
{
	const struct dai_esp32_pdm_config *cfg = dev->config;
	int ret;

	HP_SYS_CLKRST.soc_clk_ctrl2.reg_i2s0_apb_clk_en = 1;
	I2S0.clk_gate.clk_en = 1;

	if (cfg->pincfg) {
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0 && ret != -ENOENT) {
			LOG_ERR("Failed to apply pinctrl (%d)", ret);
			return ret;
		}
	}

	return 0;
}

#define DAI_ESP32_PDM_DEVICE_INIT(inst)								\
	PINCTRL_DT_INST_DEFINE(inst);								\
												\
	static const struct dai_esp32_pdm_config dai_esp32_pdm_config_##inst = {		\
		.reg_base = DT_INST_REG_ADDR(inst),						\
		.irq = DT_INST_IRQN(inst),							\
		.dai_index = DT_INST_PROP_OR(inst, dai_index, 1),				\
		.fifo_depth = DT_INST_PROP_OR(inst, fifo_depth, 32),				\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),					\
	};											\
												\
	static struct dai_esp32_pdm_data dai_esp32_pdm_data_##inst = {				\
		.cfg.type = DAI_ESP32_PDM,							\
		.cfg.dai_index = DT_INST_PROP_OR(inst, dai_index, 1),				\
		.tx_state = DAI_STATE_NOT_READY,						\
		.rx_state = DAI_STATE_NOT_READY,						\
	};											\
												\
	DEVICE_DT_INST_DEFINE(inst,								\
			      dai_esp32_pdm_init,						\
			      NULL,								\
			      &dai_esp32_pdm_data_##inst,					\
			      &dai_esp32_pdm_config_##inst,					\
			      POST_KERNEL,							\
			      CONFIG_DAI_INIT_PRIORITY,						\
			      &dai_esp32_pdm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DAI_ESP32_PDM_DEVICE_INIT)

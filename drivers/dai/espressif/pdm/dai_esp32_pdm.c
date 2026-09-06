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
#include <soc/gpio_struct.h>
#include <soc/io_mux_struct.h>
#include <hal/i2s_ll.h>
#include <esp_rom_gpio.h>

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

static void pdm_hw_configure(bool is_pdm1, bool is_slave)
{
	/* Configure I2S1 (PDM0) or I2S2 (PDM1) */
	if (!is_pdm1) {
		/* I2S1 Hardware PDM Configuration */
		I2S1.tx_conf.tx_reset = 1;
		I2S1.tx_conf.tx_reset = 0;
		I2S1.tx_conf.tx_fifo_reset = 1;
		I2S1.tx_conf.tx_fifo_reset = 0;

		I2S1.tx_conf.tx_pdm_en = 1;
		I2S1.tx_conf.tx_tdm_en = 0;
		I2S1.tx_pcm2pdm_conf.pcm2pdm_conv_en = 1;
		I2S1.tx_pcm2pdm_conf.tx_pdm_dac_mode_en = 0;   /* Digital PDM Codec bitstream mode */
		I2S1.tx_pcm2pdm_conf.tx_pdm_dac_2out_en = 0;   /* Single data line PDM multiplexed output */
		I2S1.tx_pcm2pdm_conf.tx_pdm_sinc_osr2 = 2;     /* 64x oversampling ratio */
		I2S1.tx_pcm2pdm_conf.tx_pdm_prescale = 0;
		I2S1.tx_pcm2pdm_conf.tx_pdm_hp_in_shift = 1;
		I2S1.tx_pcm2pdm_conf.tx_pdm_lp_in_shift = 1;
		I2S1.tx_pcm2pdm_conf.tx_pdm_sinc_in_shift = 1;
		I2S1.tx_pcm2pdm_conf.tx_pdm_sigmadelta_in_shift = 1;
		I2S1.tx_pcm2pdm_conf.tx_pdm_sigmadelta_dither = 0;
		I2S1.tx_pcm2pdm_conf.tx_pdm_sigmadelta_dither2 = 0;
		I2S1.tx_pcm2pdm_conf1.tx_pdm_fp = 64;
		I2S1.tx_pcm2pdm_conf1.tx_pdm_fs = 64;
		I2S1.tx_pcm2pdm_conf1.tx_iir_hp_mult12_5 = 7;
		I2S1.tx_pcm2pdm_conf1.tx_iir_hp_mult12_0 = 7;

		I2S1.tx_conf.tx_mono = 0;
		I2S1.tx_conf.tx_chan_mod = 0;
		I2S1.tx_conf.tx_ws_idle_pol = 0;
		I2S1.tx_conf.tx_slave_mod = is_slave ? 1 : 0;
		I2S1.tx_conf.tx_stop_en = 0;
		I2S1.tx_conf.tx_update = 1;

		/* PDM RX Configuration */
		I2S1.rx_conf.rx_pdm_en = 1;
		I2S1.rx_conf.rx_tdm_en = 0;
		I2S1.rx_conf.rx_slave_mod = is_slave ? 1 : 0;
		I2S1.rx_conf.rx_update = 1;
	} else {
		/* I2S2 Hardware PDM Configuration */
		I2S2.tx_conf.tx_reset = 1;
		I2S2.tx_conf.tx_reset = 0;
		I2S2.tx_conf.tx_fifo_reset = 1;
		I2S2.tx_conf.tx_fifo_reset = 0;

		I2S2.tx_conf.tx_pdm_en = 1;
		I2S2.tx_conf.tx_tdm_en = 0;
		I2S2.tx_pcm2pdm_conf.pcm2pdm_conv_en = 1;
		I2S2.tx_pcm2pdm_conf.tx_pdm_dac_mode_en = 0;
		I2S2.tx_pcm2pdm_conf.tx_pdm_dac_2out_en = 0;
		I2S2.tx_pcm2pdm_conf.tx_pdm_sinc_osr2 = 2;
		I2S2.tx_pcm2pdm_conf.tx_pdm_prescale = 0;
		I2S2.tx_pcm2pdm_conf.tx_pdm_hp_in_shift = 1;
		I2S2.tx_pcm2pdm_conf.tx_pdm_lp_in_shift = 1;
		I2S2.tx_pcm2pdm_conf.tx_pdm_sinc_in_shift = 1;
		I2S2.tx_pcm2pdm_conf.tx_pdm_sigmadelta_in_shift = 1;
		I2S2.tx_pcm2pdm_conf1.tx_pdm_fp = 64;
		I2S2.tx_pcm2pdm_conf1.tx_pdm_fs = 64;
		I2S2.tx_pcm2pdm_conf1.tx_iir_hp_mult12_5 = 7;
		I2S2.tx_pcm2pdm_conf1.tx_iir_hp_mult12_0 = 7;

		I2S2.tx_conf.tx_mono = 0;
		I2S2.tx_conf.tx_chan_mod = 0;
		I2S2.tx_conf.tx_ws_idle_pol = 0;
		I2S2.tx_conf.tx_slave_mod = is_slave ? 1 : 0;
		I2S2.tx_conf.tx_stop_en = 0;
		I2S2.tx_conf.tx_update = 1;

		I2S2.rx_conf.rx_pdm_en = 1;
		I2S2.rx_conf.rx_tdm_en = 0;
		I2S2.rx_conf.rx_slave_mod = is_slave ? 1 : 0;
		I2S2.rx_conf.rx_update = 1;
	}
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

	LOG_INF("DAI PDM%d config_set: rate=%u, format=0x%x, channels=%u",
		is_pdm1 ? 1 : 0, cfg->rate, cfg->format, cfg->channels);

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

	pdm_hw_configure(is_pdm1, data->is_slave);

	if (!is_pdm1) {
		/* PDM0 GPIO routing */
		if (data->is_slave) {
			/* GPIO 4: PDM Bit Clock In (Pin 18)
			 * GPIO 5: PDM Data Out (Pin 16)
			 * GPIO 6: PDM RX Clock In (Pin 15)
			 * GPIO 3: PDM RX Data In (Pin 19)
			 */
			GPIO.func_out_sel_cfg[4].out_sel = 256;
			GPIO.func_out_sel_cfg[4].oen_sel = 1;
			GPIO.func_out_sel_cfg[6].out_sel = 256;
			GPIO.func_out_sel_cfg[6].oen_sel = 1;
			GPIO.enable_w1tc.val = (1 << 3) | (1 << 4) | (1 << 6);

			GPIO.func_out_sel_cfg[5].out_sel = 34; /* I2S1_O_SD_OUT */
			GPIO.func_out_sel_cfg[5].oen_sel = 0;
			GPIO.func_out_sel_cfg[5].oen_inv_sel = 0;
			GPIO.enable_w1ts.val = (1 << 5);

			esp_rom_gpio_connect_in_signal(4, 31, false); /* I2S1_O_BCK_IN */
			esp_rom_gpio_connect_in_signal(4, 35, false); /* I2S1_I_BCK_IN */
			esp_rom_gpio_connect_in_signal(6, 33, false); /* I2S1_O_WS_IN */
			esp_rom_gpio_connect_in_signal(6, 36, false); /* I2S1_I_WS_IN */
			esp_rom_gpio_connect_in_signal(3, 34, false); /* I2S1_I_SD_IN */

			LOG_INF("DAI PDM0 configured in SLAVE mode (external clock)");
		} else {
			/* Master Mode */
			GPIO.func_out_sel_cfg[4].out_sel = 31; /* I2S1_O_BCK_OUT */
			GPIO.func_out_sel_cfg[4].oen_sel = 0;
			GPIO.func_out_sel_cfg[5].out_sel = 34; /* I2S1_O_SD_OUT */
			GPIO.func_out_sel_cfg[5].oen_sel = 0;
			GPIO.func_out_sel_cfg[6].out_sel = 33; /* I2S1_O_WS_OUT */
			GPIO.func_out_sel_cfg[6].oen_sel = 0;
			GPIO.enable_w1ts.val = (1 << 4) | (1 << 5) | (1 << 6);

			esp_rom_gpio_connect_in_signal(3, 34, false); /* I2S1_I_SD_IN */

			LOG_INF("DAI PDM0 configured in MASTER mode (internal clock)");
		}
	}

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

	LOG_DBG("DAI PDM%d trigger: dir=%d, cmd=%d", is_pdm1 ? 1 : 0, dir, cmd);

	switch (cmd) {
	case DAI_TRIGGER_PRE_START:
	case DAI_TRIGGER_START:
		if (dir == DAI_DIR_TX || dir == DAI_DIR_BOTH) {
			if (!is_pdm1) {
				I2S1.tx_conf.tx_start = 1;
				I2S1.tx_conf.tx_update = 1;
			} else {
				I2S2.tx_conf.tx_start = 1;
				I2S2.tx_conf.tx_update = 1;
			}
			data->tx_state = DAI_STATE_RUNNING;
		}
		if (dir == DAI_DIR_RX || dir == DAI_DIR_BOTH) {
			if (!is_pdm1) {
				I2S1.rx_conf.rx_start = 1;
				I2S1.rx_conf.rx_update = 1;
			} else {
				I2S2.rx_conf.rx_start = 1;
				I2S2.rx_conf.rx_update = 1;
			}
			data->rx_state = DAI_STATE_RUNNING;
		}
		break;
	case DAI_TRIGGER_STOP:
	case DAI_TRIGGER_POST_STOP:
	case DAI_TRIGGER_PAUSE:
	case DAI_TRIGGER_DROP:
		if (dir == DAI_DIR_TX || dir == DAI_DIR_BOTH) {
			if (!is_pdm1) {
				I2S1.tx_conf.tx_start = 0;
				I2S1.tx_conf.tx_update = 1;
			} else {
				I2S2.tx_conf.tx_start = 0;
				I2S2.tx_conf.tx_update = 1;
			}
			data->tx_state = DAI_STATE_READY;
		}
		if (dir == DAI_DIR_RX || dir == DAI_DIR_BOTH) {
			if (!is_pdm1) {
				I2S1.rx_conf.rx_start = 0;
				I2S1.rx_conf.rx_update = 1;
			} else {
				I2S2.rx_conf.rx_start = 0;
				I2S2.rx_conf.rx_update = 1;
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

static const struct dai_properties *dai_esp32_pdm_get_properties(const struct device *dev,
								 enum dai_dir dir,
								 int stream_id)
{
	const struct dai_esp32_pdm_config *cfg = dev->config;
	struct dai_esp32_pdm_data *data = dev->data;
	bool is_pdm1 = (cfg->dai_index == 2);

	if (dir == DAI_DIR_TX) {
		data->tx_props.fifo_address = is_pdm1 ? (uint32_t)&I2S2.tx_conf : (uint32_t)&I2S1.tx_conf;
		data->tx_props.fifo_depth = cfg->fifo_depth;
		data->tx_props.dma_hs_id = is_pdm1 ? 4 : 2;
		return &data->tx_props;
	} else if (dir == DAI_DIR_RX) {
		data->rx_props.fifo_address = is_pdm1 ? (uint32_t)&I2S2.rx_conf : (uint32_t)&I2S1.rx_conf;
		data->rx_props.fifo_depth = cfg->fifo_depth;
		data->rx_props.dma_hs_id = is_pdm1 ? 5 : 3;
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

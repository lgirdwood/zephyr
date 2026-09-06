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
	if (provider == DAI_CBP_CFP || provider == DAI_CBP_CFC) {
		/* Codec is BCLK provider -> ESP32 is Slave */
		data->is_slave = true;
	} else {
		/* ESP32 is Master (generates BCLK & WS) */
		data->is_slave = false;
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

	/* Configure Master/Slave Mode */
	I2S0.tx_conf.tx_slave_mod = data->is_slave ? 1 : 0;
	I2S0.rx_conf.rx_slave_mod = data->is_slave ? 1 : 0;

	/* Configure TDM / Philips I2S format */
	I2S0.tx_conf.tx_pdm_en = 0;
	I2S0.tx_conf.tx_tdm_en = 1;
	I2S0.rx_conf.rx_pdm_en = 0;
	I2S0.rx_conf.rx_tdm_en = 1;

	I2S0.tx_tdm_ctrl.tx_tdm_tot_chan_num = data->channels > 0 ? (data->channels - 1) : 0;
	I2S0.rx_tdm_ctrl.rx_tdm_tot_chan_num = data->channels > 0 ? (data->channels - 1) : 0;
	I2S0.tx_conf1.tx_bits_mod = data->word_size > 0 ? (data->word_size - 1) : 15;
	I2S0.tx_conf1.tx_tdm_chan_bits = data->word_size > 0 ? (data->word_size - 1) : 15;
	I2S0.rx_conf1.rx_bits_mod = data->word_size > 0 ? (data->word_size - 1) : 15;
	I2S0.rx_conf1.rx_tdm_chan_bits = data->word_size > 0 ? (data->word_size - 1) : 15;

	if (data->is_slave) {
		/* Configure Slave GPIO Matrix Inputs:
		 * BCLK In: GPIO 21 (Pin 11) -> I2S0_I_BCK_IN (16) & I2S0_O_BCK_IN (13)
		 * WS In:   GPIO 22 (Pin 12) -> I2S0_I_WS_IN (18) & I2S0_O_WS_IN (15)
		 * DIN In:  GPIO 20 (Pin 13) -> I2S0_I_SD_IN (16)
		 * DOUT Out:GPIO 23 (Pin 7)  <- I2S0_O_SD_OUT (16)
		 */
		GPIO.func_out_sel_cfg[21].out_sel = 256;
		GPIO.func_out_sel_cfg[21].oen_sel = 1;
		GPIO.func_out_sel_cfg[22].out_sel = 256;
		GPIO.func_out_sel_cfg[22].oen_sel = 1;
		GPIO.enable_w1tc.val = (1 << 20) | (1 << 21) | (1 << 22);

		GPIO.func_out_sel_cfg[23].out_sel = 16; /* I2S0_O_SD_OUT */
		GPIO.func_out_sel_cfg[23].oen_sel = 0;
		GPIO.func_out_sel_cfg[23].oen_inv_sel = 0;
		GPIO.enable_w1ts.val = (1 << 23);

		esp_rom_gpio_connect_in_signal(21, 16, false); /* I2S0_I_BCK_IN */
		esp_rom_gpio_connect_in_signal(21, 13, false); /* I2S0_O_BCK_IN */
		esp_rom_gpio_connect_in_signal(22, 18, false); /* I2S0_I_WS_IN */
		esp_rom_gpio_connect_in_signal(22, 15, false); /* I2S0_O_WS_IN */
		esp_rom_gpio_connect_in_signal(20, 16, false); /* I2S0_I_SD_IN */

		LOG_INF("DAI I2S set to SLAVE mode (external BCLK/WS)");
	} else {
		/* Master Mode: Drive BCLK, WS, DOUT outputs */
		GPIO.func_out_sel_cfg[21].out_sel = 13; /* I2S0_O_BCK_OUT */
		GPIO.func_out_sel_cfg[21].oen_sel = 0;
		GPIO.func_out_sel_cfg[22].out_sel = 15; /* I2S0_O_WS_OUT */
		GPIO.func_out_sel_cfg[22].oen_sel = 0;
		GPIO.func_out_sel_cfg[23].out_sel = 16; /* I2S0_O_SD_OUT */
		GPIO.func_out_sel_cfg[23].oen_sel = 0;
		GPIO.enable_w1ts.val = (1 << 21) | (1 << 22) | (1 << 23);

		esp_rom_gpio_connect_in_signal(20, 16, false); /* I2S0_I_SD_IN */

		LOG_INF("DAI I2S set to MASTER mode (internal BCLK/WS)");
	}

	I2S0.tx_conf.tx_update = 1;
	I2S0.rx_conf.rx_update = 1;

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

	LOG_DBG("DAI I2S trigger: dir=%d, cmd=%d", dir, cmd);

	switch (cmd) {
	case DAI_TRIGGER_PRE_START:
	case DAI_TRIGGER_START:
		if (dir == DAI_DIR_TX || dir == DAI_DIR_BOTH) {
			I2S0.tx_conf.tx_start = 1;
			I2S0.tx_conf.tx_update = 1;
			data->tx_state = DAI_STATE_RUNNING;
		}
		if (dir == DAI_DIR_RX || dir == DAI_DIR_BOTH) {
			I2S0.rx_conf.rx_start = 1;
			I2S0.rx_conf.rx_update = 1;
			data->rx_state = DAI_STATE_RUNNING;
		}
		break;
	case DAI_TRIGGER_STOP:
	case DAI_TRIGGER_POST_STOP:
	case DAI_TRIGGER_PAUSE:
	case DAI_TRIGGER_DROP:
		if (dir == DAI_DIR_TX || dir == DAI_DIR_BOTH) {
			I2S0.tx_conf.tx_start = 0;
			I2S0.tx_conf.tx_update = 1;
			data->tx_state = DAI_STATE_READY;
		}
		if (dir == DAI_DIR_RX || dir == DAI_DIR_BOTH) {
			I2S0.rx_conf.rx_start = 0;
			I2S0.rx_conf.rx_update = 1;
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
		data->tx_props.dma_hs_id = 0;
		return &data->tx_props;
	} else if (dir == DAI_DIR_RX) {
		data->rx_props.fifo_address = (uint32_t)&I2S0.rx_conf;
		data->rx_props.fifo_depth = cfg->fifo_depth;
		data->rx_props.dma_hs_id = 1;
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

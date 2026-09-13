/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_dai_spdif

#include <zephyr/drivers/dai.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include "spdif.h"

LOG_MODULE_REGISTER(dai_nxp_spdif, CONFIG_DAI_LOG_LEVEL);

static void spdif_isr(const void *parameter)
{
	const struct device *dev = parameter;
	struct spdif_data *data = dev->data;
	SPDIF_Type *base = (SPDIF_Type *)data->regmap;
	uint32_t status;

	status = SPDIF_GetStatusFlag(base);
	if (status & (kSPDIF_TxFIFOError | kSPDIF_RxFIFOError)) {
		LOG_WRN("SPDIF FIFO error, status: 0x%08x", status);
	}

	SPDIF_ClearStatusFlags(base, status & ~((uint32_t)kSPDIF_TxFIFOEmpty |
					       (uint32_t)kSPDIF_RxFIFOFull |
					       (uint32_t)kSPDIF_UChannelReceiveRegisterFull |
					       (uint32_t)kSPDIF_QChannelReceiveRegisterFull));
}

static int spdif_config_get(const struct device *dev,
			    struct dai_config *cfg,
			    enum dai_dir dir)
{
	struct spdif_data *data = dev->data;

	memcpy(cfg, &data->cfg, sizeof(*cfg));
	return 0;
}

static int spdif_config_set(const struct device *dev,
			    const struct dai_config *cfg,
			    const void *bespoke_cfg,
			    size_t bespoke_size)
{
	const struct spdif_config *dev_cfg = dev->config;
	struct spdif_data *data = dev->data;
	SPDIF_Type *base = (SPDIF_Type *)data->regmap;
	uint32_t clock_rate = 0;
	int ret;

	if (!cfg) {
		return -EINVAL;
	}

	memcpy(&data->cfg, cfg, sizeof(data->cfg));

	ret = clock_control_get_rate(dev_cfg->clock_dev, dev_cfg->clock_subsys, &clock_rate);
	if (ret < 0) {
		LOG_ERR("Failed to query SPDIF clock root rate: %d", ret);
		return ret;
	}

	LOG_DBG("SPDIF clock root rate: %u Hz, target sample rate: %u Hz",
		clock_rate, cfg->rate);

	spdif_config_t spdif_cfg;
	SPDIF_GetDefaultConfig(&spdif_cfg);
	spdif_cfg.isTxAutoSync = false;
	spdif_cfg.isRxAutoSync = false;
	spdif_cfg.DPLLClkSource = 1; /* SPDIF0_CLK_ROOT */
	spdif_cfg.txClkSource = 1;   /* SPDIF0_CLK_ROOT */
	spdif_cfg.rxFullSelect = kSPDIF_RxFull8Samples;
	spdif_cfg.txFullSelect = kSPDIF_TxEmpty8Samples;
	spdif_cfg.uChannelSrc = kSPDIF_UChannelFromTx;
	spdif_cfg.txSource = kSPDIF_txNormal;
	spdif_cfg.validityConfig = kSPDIF_validityFlagAlwaysClear;
	spdif_cfg.gain = kSPDIF_GAIN_8;

	SPDIF_Init(base, &spdif_cfg);

	/* Set consumer audio PCM channel status (linear PCM, 48 kHz) */
	SPDIF_WriteChannelStatusHigh(base, 0x204000);
	SPDIF_WriteChannelStatusLow(base, 0x400000);

	if (cfg->rate > 0) {
		SPDIF_TxSetSampleRate(base, cfg->rate, clock_rate);
	}

	/* Start transmitting continuous zero carrier so receiver DPLL locks immediately */
	base->SCR = (base->SCR & ~(SPDIF_SCR_TXSEL_MASK | SPDIF_SCR_TXFIFO_CTRL_MASK)) |
		    SPDIF_SCR_TXSEL(kSPDIF_txNormal) |
		    SPDIF_SCR_TXFIFO_CTRL(0);
	SPDIF_TxEnable(base, true);

	data->tx_state = DAI_STATE_READY;
	data->rx_state = DAI_STATE_READY;

	return 0;
}

static int spdif_trigger(const struct device *dev,
			 enum dai_dir dir,
			 enum dai_trigger_cmd cmd)
{
	struct spdif_data *data = dev->data;
	SPDIF_Type *base = (SPDIF_Type *)data->regmap;

	switch (cmd) {
	case DAI_TRIGGER_START:
		LOG_DBG("SPDIF start dir %d", dir);
		if (dir == DAI_DIR_TX) {
			base->SIC = 0xFFFFFFFF;
			base->SCR = (base->SCR & ~(SPDIF_SCR_TXSEL_MASK | SPDIF_SCR_TXFIFO_CTRL_MASK)) |
				    SPDIF_SCR_TXSEL(kSPDIF_txNormal) |
				    SPDIF_SCR_TXFIFO_CTRL(1);
			SPDIF_TxEnable(base, true);
			SPDIF_EnableDMA(base, kSPDIF_TxDMAEnable, true);
			data->tx_state = DAI_STATE_RUNNING;
		} else if (dir == DAI_DIR_RX) {
			base->SIC = 0xFFFFFFFF;
			base->SCR |= SPDIF_SCR_RXFIFO_RST_MASK;
			base->SCR &= ~(SPDIF_SCR_RXFIFO_RST_MASK | SPDIF_SCR_RXFIFO_CTRL_MASK | SPDIF_SCR_RXFIFO_OFF_ON_MASK);
			SPDIF_EnableDMA(base, kSPDIF_RxDMAEnable, true);
			SPDIF_RxEnable(base, true);
			data->rx_state = DAI_STATE_RUNNING;
		}
		return 0;

	case DAI_TRIGGER_STOP:
	case DAI_TRIGGER_PAUSE:
		LOG_DBG("SPDIF stop/pause dir %d", dir);
		if (dir == DAI_DIR_TX) {
			/* Do NOT disable transfer clock (TxEnable) so carrier continues */
			SPDIF_EnableDMA(base, kSPDIF_TxDMAEnable, false);
			base->SCR = (base->SCR & ~SPDIF_SCR_TXFIFO_CTRL_MASK) |
				    SPDIF_SCR_TXFIFO_CTRL(0);
			base->SIC = 0xFFFFFFFF;
			data->tx_state = DAI_STATE_STOPPING;
		} else if (dir == DAI_DIR_RX) {
			SPDIF_RxEnable(base, false);
			SPDIF_EnableDMA(base, kSPDIF_RxDMAEnable, false);
			base->SIC = 0xFFFFFFFF;
			data->rx_state = DAI_STATE_STOPPING;
		}
		return 0;

	case DAI_TRIGGER_PRE_START:
	case DAI_TRIGGER_COPY:
		return 0;

	default:
		LOG_WRN("Unhandled trigger cmd %d", cmd);
		return -EINVAL;
	}
}

static const struct dai_properties *spdif_get_properties(const struct device *dev,
							 enum dai_dir dir,
							 int stream_id)
{
	const struct spdif_config *cfg = dev->config;

	switch (dir) {
	case DAI_DIR_RX:
		return cfg->rx_props;
	case DAI_DIR_TX:
		return cfg->tx_props;
	default:
		LOG_ERR("Invalid direction: %d", dir);
		return NULL;
	}
}

static int spdif_probe(const struct device *dev)
{
	return 0;
}

static int spdif_remove(const struct device *dev)
{
	return 0;
}

static DEVICE_API(dai, spdif_api) = {
	.config_set = spdif_config_set,
	.config_get = spdif_config_get,
	.trigger = spdif_trigger,
	.get_properties = spdif_get_properties,
	.probe = spdif_probe,
	.remove = spdif_remove,
};

static int spdif_clks_enable_disable(const struct device *dev, bool enable)
{
	const struct spdif_config *cfg = dev->config;

	if (enable) {
		return clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	} else {
		return clock_control_off(cfg->clock_dev, cfg->clock_subsys);
	}
}

__maybe_unused static int spdif_pm_action(const struct device *dev,
					  enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return spdif_clks_enable_disable(dev, true);
	case PM_DEVICE_ACTION_SUSPEND:
		return spdif_clks_enable_disable(dev, false);
	default:
		return 0;
	}
}

static int spdif_init(const struct device *dev)
{
	const struct spdif_config *cfg = dev->config;
	struct spdif_data *data = dev->data;
	int ret;

	device_map(&data->regmap, cfg->regmap_phys, cfg->regmap_size, K_MEM_CACHE_NONE);

	ret = spdif_clks_enable_disable(dev, true);
	if (ret < 0) {
		LOG_ERR("Failed to enable clocks: %d", ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0 && ret != -ENOENT) {
		LOG_ERR("Failed to apply default pinctrl state: %d", ret);
		return ret;
	}

	data->tx_state = DAI_STATE_NOT_READY;
	data->rx_state = DAI_STATE_NOT_READY;

	cfg->irq_config();

	return pm_device_runtime_enable(dev);
}

#define SPDIF_INIT(inst)								\
	PINCTRL_DT_INST_DEFINE(inst);							\
											\
	static const struct dai_properties spdif_tx_props_##inst = {			\
		.fifo_address = DT_INST_REG_ADDR(inst) + SPDIF_TX_FIFO_REG_OFFSET,	\
		.fifo_depth = 16 * 4,							\
		.dma_hs_id = SPDIF_TX_RX_DMA_HANDSHAKE(inst, tx),			\
	};										\
											\
	static const struct dai_properties spdif_rx_props_##inst = {			\
		.fifo_address = DT_INST_REG_ADDR(inst) + SPDIF_RX_FIFO_REG_OFFSET,	\
		.fifo_depth = 16 * 4,							\
		.dma_hs_id = SPDIF_TX_RX_DMA_HANDSHAKE(inst, rx),			\
	};										\
											\
	static void irq_config_##inst(void)						\
	{										\
		IRQ_CONNECT(DT_INST_IRQN(inst),						\
			    0,								\
			    spdif_isr,							\
			    DEVICE_DT_INST_GET(inst),					\
			    0);								\
	}										\
											\
	static const struct spdif_config spdif_config_##inst = {			\
		.regmap_phys = DT_INST_REG_ADDR(inst),					\
		.regmap_size = DT_INST_REG_SIZE(inst),					\
		.irq = DT_INST_IRQN(inst),						\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),			\
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(inst, name),\
		.tx_props = &spdif_tx_props_##inst,					\
		.rx_props = &spdif_rx_props_##inst,					\
		.irq_config = irq_config_##inst,					\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),				\
	};										\
											\
	static struct spdif_data spdif_data_##inst = {					\
		.cfg.type = DAI_IMX_SPDIF,						\
		.cfg.dai_index = DT_INST_PROP_OR(inst, dai_index, 0),			\
	};										\
											\
	PM_DEVICE_DT_INST_DEFINE(inst, spdif_pm_action);				\
											\
	DEVICE_DT_INST_DEFINE(inst, &spdif_init, PM_DEVICE_DT_INST_GET(inst),		\
			      &spdif_data_##inst, &spdif_config_##inst,			\
			      POST_KERNEL, CONFIG_DAI_INIT_PRIORITY,			\
			      &spdif_api);

DT_INST_FOREACH_STATUS_OKAY(SPDIF_INIT);

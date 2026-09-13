/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DAI_NXP_SPDIF_H_
#define ZEPHYR_DRIVERS_DAI_NXP_SPDIF_H_

#include <zephyr/drivers/dai.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <fsl_spdif.h>

#define SPDIF_TX_FIFO_REG_OFFSET 0x2C /* STL */
#define SPDIF_RX_FIFO_REG_OFFSET 0x14 /* SRL */

#define SPDIF_TX_RX_DMA_HANDSHAKE(inst, dir) \
	(((DT_INST_DMAS_CELL_BY_NAME(inst, dir, mux) & 0xFF)) | \
	 (((DT_INST_DMAS_CELL_BY_NAME(inst, dir, source) & 0xFF) << 8)))

struct spdif_config {
	mm_reg_t regmap_phys;
	size_t regmap_size;
	int irq;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	const struct pinctrl_dev_config *pincfg;
	const struct dai_properties *tx_props;
	const struct dai_properties *rx_props;
	void (*irq_config)(void);
};

struct spdif_data {
	struct dai_config cfg;
	enum dai_state tx_state;
	enum dai_state rx_state;
	mm_reg_t regmap;
};

#endif /* ZEPHYR_DRIVERS_DAI_NXP_SPDIF_H_ */

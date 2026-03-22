/*
 * Copyright (c) 2025 Liam Girdwood
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * .
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "amt2xx.h"

LOG_MODULE_REGISTER(AMT2XX, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Bus IO abstraction. AMT2xx supports SPI, SSI and UART
 */
static inline int amt2xx_bus_check(const struct device *dev)
{
	const struct amt2xx_config *cfg = dev->config;

	return cfg->bus_io->check(cfg);
}

static inline int amt2xx_cmd(const struct device *dev,
			     const struct amt2xx_cmd *cmd,
			     uint32_t *rx_data, int flags)
{
	const struct amt2xx_config *cfg = dev->config;

	return cfg->bus_io->cmd(cfg, cmd, rx_data, flags);
}

/*
 * Sample readback and commands.
 * TODO: Add reset, set zero point and read turns commands.
 */
static const struct amt2xx_cmd amt2xx_cmds[] = {
	{ .cmd = {0x00, 0x00}, .len = 2 },	/* sample readback */
	{ .cmd = {0x00, 0x60}, .len = 2 },	/* reset encoder */
	{ .cmd = {0x00, 0x70}, .len = 2 },	/* set zero point */
	{ .cmd = {0x00, 0xa0, 0x00, 0x00}, .len = 4 },	/* read turns - certain SKUs only */
};

/* enum value aligns to amt2xx_cmds index above */
enum amt2xx_cmds {
	AMT2XX_CMD_SAMPLE_READBACK = 0,
	AMT2XX_CMD_RESET_ENCODER,
	AMT2XX_CMD_SET_ZERO_POINT,
	AMT2XX_CMD_READ_TURNS,
};
struct amt2xx_data {
	struct amt2xx_reading reading;
	int io_flags;
};

int amt2xx_sample_fetch_helper(const struct device *dev,
			       enum sensor_channel chan, struct amt2xx_reading *reading)
{
	struct amt2xx_data *data = dev->data;
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

data->io_flags = AMT2XX_IO_FLAGS_SPI_CS_GPIO;

	ret = amt2xx_cmd(dev, &amt2xx_cmds[AMT2XX_CMD_SAMPLE_READBACK],
		&data->reading.abs_posn, data->io_flags);
	if (ret < 0) {
		LOG_ERR("Failed to read sample: %d", ret);
		return ret;
	}

	return 0;
}

int amt2xx_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct amt2xx_data *data = dev->data;

	return amt2xx_sample_fetch_helper(dev, chan, &data->reading);
}

/* AMT2xx uses a checksum on high 2 bits of position readback */
static int amt2xx_validdate_reading(struct amt2xx_data *data)
{
	int odd, even, k1, k0;

	/* h5 ^ h3 ^ h1 ^ l7 ^ l5 ^ l3 ^ l1 */
	odd = (((data->reading.abs_posn >> 13) & 0x1) ^
		((data->reading.abs_posn >> 11) & 0x1) ^
		((data->reading.abs_posn >> 9) & 0x1) ^
		((data->reading.abs_posn >> 7) & 0x1) ^
		((data->reading.abs_posn >> 5) & 0x1) ^
		((data->reading.abs_posn >> 3) & 0x1) ^
		((data->reading.abs_posn >> 1) & 0x1));

	/* h4 ^ h2 ^ h0 ^ l6 ^ l4 ^ l2 ^ l0 */
	even = (((data->reading.abs_posn >> 12) & 0x1) ^
		((data->reading.abs_posn >> 10) & 0x1) ^
		((data->reading.abs_posn >> 8) & 0x1) ^
		((data->reading.abs_posn >> 6) & 0x1) ^
		((data->reading.abs_posn >> 4) & 0x1) ^
		((data->reading.abs_posn >> 2) & 0x1) ^
		((data->reading.abs_posn >> 0) & 0x1));

	k1 = ((data->reading.abs_posn >> 14) & 0x1);
	k0 = ((data->reading.abs_posn >> 15) & 0x1);

	/* is k0 == !odd */
	if (k0 != (odd ^ 0x1)) {
		LOG_ERR("k0 != !odd");
		return -EINVAL;
	}
	/* is k1 == !even */
	if (k1 != (even ^ 0x1)) {
		LOG_ERR("k1 != !even");
		return -EINVAL;
	}

	return 0; /* valid */
}

/* convert raw data to degrees */
void convert_to_degrees_parts(uint16_t value, int* val1, int* val2)
{
	/* Get the full degrees value as a double */
	const double total_steps = (double)(1UL << 14);

	/* degrees = (value / total_possible_values) * degrees_in_a_circle */
	double degrees = ((double)value / total_steps) * 360.0;

	/* Extract the integer part */
	*val1 = (int)floor(degrees);

	/* Extract the fractional part */
	double fractional_part = fmod(degrees, 1.0);
	*val2 = (int)(fractional_part * 1000000);
}

static int amt2xx_channel_get(const struct device *dev,
			      enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct amt2xx_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ROTATION:
		if (amt2xx_validdate_reading(data) < 0) {
			LOG_ERR("Invalid reading");
			return -EINVAL;
		}
		/* reading is 14 bits of data and 2 parity bits */
		convert_to_degrees_parts(data->reading.abs_posn & 0x3fff,
					&val->val1, &val->val2);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static DEVICE_API(sensor, amt2xx_api_funcs) = {
	.sample_fetch = amt2xx_sample_fetch,
	.channel_get = amt2xx_channel_get,
};

static int amt2xx_chip_init(const struct device *dev)
{
	const struct amt2xx_config *cfg = dev->config;
	int err;

	err = amt2xx_bus_check(dev);
	if (err < 0) {
		LOG_DBG("bus check failed: %d", err);
		return err;
	}

#if AMT2XX_BUS_SPI
	/* The AMT2xx will immediately output data on toggle of CS and CLK and
	 * this can confuse the AMT2xx if the SoC Pinmux toggles the CS pin or
	 * toggles the CLK pin. The AMT2xx will begin to output data on MISO
	 * and get into an undefined state.
	 *
	 * We can workaround this by taking manual control of the CS pin and
	 * disabling it when the SoC is configuring the pinmux during the 1st
	 * SPI transaction. This will prevent the AMT2xx from outputting data
	 * on MISO until the SoC has configured the pinmux and the CS pin is
	 * enabled.
	 */

	/* TODO do not set CS on DT, but have manual GPIO to manage */

	/* set pin as output in high state*/
	gpio_pin_configure_dt(&cfg->cs_gpio, GPIO_OUTPUT_ACTIVE);

	/* now do 1st SPI transaction - this will setup pinmux and controller,
	 * it will fail as CS is high.
	 */
	uint32_t cmd = 0x0f0f0;
	const struct spi_buf tx_buf = {
		.buf = (uint8_t *)&cmd,
		.len = 4,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};
	spi_write_dt(&cfg->bus.spi, &tx);
	gpio_pin_set_dt(&cfg->cs_gpio, 1);

	/* Wait for the sensor to be ready */
	k_sleep(K_MSEC(200));
#endif

	/* read a sample to confirm data is valid and valid */
	err = amt2xx_sample_fetch(dev, SENSOR_CHAN_ALL);
        if (err < 0) {
                LOG_ERR("Failed to fetch sample: %d", err);
                return err;
        }
        if (amt2xx_validdate_reading(dev->data) < 0) {
                LOG_ERR("Invalid reading after init");
                return -EINVAL;
        }

	return 0;
}

/* Initializes a struct amt2xx_config for an instance on a SPI bus. */
#define AMT2XX_CONFIG_SPI(inst)				\
	{						\
		.bus.spi = SPI_DT_SPEC_INST_GET(	\
			inst, AMT2XX_SPI_OPERATION, 0),	\
		.bus_io = &amt2xx_bus_io_spi,		\
		.cs_gpio = GPIO_DT_SPEC_INST_GET(inst, cs_gpios), \
	}

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define AMT2XX_DEFINE(inst)						\
	static struct amt2xx_data amt2xx_data_##inst;			\
	static const struct amt2xx_config amt2xx_config_##inst =	\
			    AMT2XX_CONFIG_SPI(inst);			\
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			 amt2xx_chip_init,				\
			 PM_DEVICE_DT_INST_GET(inst),			\
			 &amt2xx_data_##inst,				\
			 &amt2xx_config_##inst,				\
			 POST_KERNEL,					\
			 CONFIG_SENSOR_INIT_PRIORITY,			\
			 &amt2xx_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(AMT2XX_DEFINE)

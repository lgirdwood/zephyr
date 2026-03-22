/*
 * Driver for the Melexis MLX90614 I2C 16-bit IR thermopile sensor
 *
 * This driver currently supports read back of ambient and object temperature.
 * The device only works with 100KHz I2C bus speed and has a fixed I2C address
 * of 0x5a.
 *
 * This driver currently does not support sleep mode or EEPROM write.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT melexis_mlx90614

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <math.h>

LOG_MODULE_REGISTER(MLX90614, CONFIG_SENSOR_LOG_LEVEL);

#define MLX90614_REG_RAM_BASE		0x00
#define MLX90614_REG_EEPROM_BASE	0x20

/* RO RAM register addresses with 16-bit data, MSB first */
#define MLX90614_REG_RAW1	(MLX90614_REG_RAM_BASE + 0x04)
#define MLX90614_REG_RAW2	(MLX90614_REG_RAM_BASE + 0x05)
#define MLX90614_REG_TA		(MLX90614_REG_RAM_BASE + 0x06)
#define MLX90614_REG_TOBJ1	(MLX90614_REG_RAM_BASE + 0x07)
#define MLX90614_REG_TOBJ2	(MLX90614_REG_RAM_BASE + 0x08)

/* EEPROM addresses with 16-bit data, MSB first, only support read today */
#define MLX90614_REG_EMISSIVITY	(MLX90614_REG_EEPROM_BASE + 0x04)
#define MLX90614_REG_CONFIG		(MLX90614_REG_EEPROM_BASE + 0x05)

/* Config register */
#define MLX90614_CONFIG_IIR_MASK (0x7 << 0)
#define MLX90614_CONFIG_DUAL_MASK (1 << 6)
#define MLX90614_CONFIG_FIR_MASK (0x7 << 8)
#define MLX90614_CONFIG_GAIN_MASK (0x7 << 11)

struct mlx90614_data {
	int16_t ambient_temp;
	int16_t object1_temp;
	int16_t object2_temp;
};

union mlx90614_config_reg {
	uint16_t cfg_reg;
	struct {
		uint16_t iir : 3;
		uint16_t rpt_test : 1;
		uint16_t tobj : 2;
		uint16_t dual : 1;
		uint16_t ks_sign : 1;
		uint16_t fir : 3;
		uint16_t gain : 3;
		uint16_t kt2_sign : 1;
		uint16_t test : 1;
	} __packed;
};

struct mlx90614_config {
	struct i2c_dt_spec i2c;
	union mlx90614_config_reg config;
};

static inline int mlx90614_reg_read(const struct device *dev, uint8_t reg, uint8_t *buf, uint32_t size)
{
	const struct mlx90614_config *cfg = dev->config;

	return i2c_burst_read_dt(&cfg->i2c, reg, buf, size);
}

static inline int mlx90614_ambient_temp_read(const struct device *dev)
{
	struct mlx90614_data *data = dev->data;
	uint8_t buf[2];
	int ret;

	ret = mlx90614_reg_read(dev, MLX90614_REG_TA, buf, sizeof(buf));
	if (ret) {
		LOG_ERR("Could not fetch ambient temperature [%d]", ret);
		return -EIO;
	}

	data->ambient_temp = *((int16_t *)(buf));
	return 0;
}

static inline int mlx90614_object1_temp_read(const struct device *dev)
{
	struct mlx90614_data *data = dev->data;
	uint8_t buf[2];
	int ret;

	ret = mlx90614_reg_read(dev, MLX90614_REG_TOBJ1, buf, sizeof(buf));
	if (ret) {
		LOG_ERR("Could not fetch object1 temperature [%d]", ret);
		return -EIO;
	}

	data->object1_temp = *((int16_t *)(buf));
	return 0;
}

static inline int mlx90614_object2_temp_read(const struct device *dev)
{
	struct mlx90614_data *data = dev->data;
	uint8_t buf[2];
	int ret;

	ret = mlx90614_reg_read(dev, MLX90614_REG_TOBJ2, buf, sizeof(buf));
	if (ret) {
		LOG_ERR("Could not fetch object2 temperature [%d]", ret);
		return -EIO;
	}

	data->object2_temp = *((int16_t *)(buf));
	return 0;
}

static void mlx90614_temp_to_sensor_value(int16_t temp, struct sensor_value *val)
{
	/* 0.02°K / LSB.*/
	float c = (temp - (273.15 * 50)) / 50;

	/* Integer part in degrees Celsius */
	val->val1 = c;

	/* Fractional part in micro degrees Celsius */
	if (c >= 0.0f)
		val->val2 = (c - floorf(c)) * 1000000;
	else
		val->val2 = (ceilf(c) - c) * 1000000;
}

static int mlx90614_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct mlx90614_config *cfg = dev->config;
	int ret;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		ret = mlx90614_ambient_temp_read(dev);
		break;
	case SENSOR_CHAN_OBJECT1_TEMP:
		ret = mlx90614_object1_temp_read(dev);
		break;
	case SENSOR_CHAN_OBJECT2_TEMP:
		if (cfg->config.dual) {
			ret = mlx90614_object2_temp_read(dev);
		} else {
			ret = -ENOTSUP;
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static int mlx90614_channel_get(const struct device *dev, enum sensor_channel chan,
			    struct sensor_value *val)
{
	const struct mlx90614_config *cfg = dev->config;
	struct mlx90614_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		mlx90614_temp_to_sensor_value(data->ambient_temp, val);
		return 0;
	case SENSOR_CHAN_OBJECT1_TEMP:
		mlx90614_temp_to_sensor_value(data->object1_temp, val);
		return 0;
	case SENSOR_CHAN_OBJECT2_TEMP:
		if (cfg->config.dual) {
			mlx90614_temp_to_sensor_value(data->object2_temp, val);
			return 0;
		} else {
			return -ENOTSUP;
		}
		return 0;
	default:
		return -ENOTSUP;
	}
}

static const struct sensor_driver_api mlx90614_driver_api = {
	.sample_fetch = mlx90614_sample_fetch,
	.channel_get = mlx90614_channel_get,
};

int mlx90614_init(const struct device *dev)
{
	const struct mlx90614_config *cfg = dev->config;
	int ret = 0;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C dev not ready");
		return -ENODEV;
	}

	ret = mlx90614_reg_read(dev, MLX90614_REG_CONFIG, (uint8_t*)&cfg->config, sizeof(cfg->config));
	if (ret < 0) {
		LOG_ERR("failed to write configuration (ret %d)", ret);
		return ret;
	}
	LOG_INF("Configuration: fir %d, gain %d, iir %d, dual %d",
		cfg->config.fir, cfg->config.gain, cfg->config.iir, cfg->config.dual);

	return ret;
}

#define MLX90614_INST(inst)                                             \
static struct mlx90614_data mlx90614_data_##inst;                           \
static const struct mlx90614_config mlx90614_config_##inst = {              \
	.i2c = I2C_DT_SPEC_INST_GET(inst),                          \
						\
};                                                                  \
SENSOR_DEVICE_DT_INST_DEFINE(inst, mlx90614_init, NULL, &mlx90614_data_##inst,	\
		      &mlx90614_config_##inst, POST_KERNEL,             \
		      CONFIG_SENSOR_INIT_PRIORITY, &mlx90614_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MLX90614_INST)

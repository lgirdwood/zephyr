/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_AMT2XX_AMT2XX_H_
#define ZEPHYR_DRIVERS_SENSOR_AMT2XX_AMT2XX_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>

#define DT_DRV_COMPAT samesky_amt2xx

#define AMT2XX_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

/*
 * AMTT2xx bus structure.
 * This structure is used to hold the bus-specific information as AMT2xx devices can
 * support multiple bus types (SPI, SSI and serial) depending on device SKU.
 *
 * TODO: Add support for SSI and UART IO.
 */
union amt2xx_bus {
#if AMT2XX_BUS_SPI
	struct spi_dt_spec spi;
#endif
};

struct amt2xx_cmd {
    uint8_t cmd[4];
    uint8_t len;
};

/* flags for IO */
#define AMT2XX_IO_FLAGS_SPI_CS_GPIO        BIT(0) /* manually assert SPI CS */

struct amt2xx_config;
typedef int (*amt2xx_bus_check_fn)(const struct amt2xx_config *cfg);
typedef int (*amt2xx_cmd_fn)(const struct amt2xx_config *cfg, const struct amt2xx_cmd *cmd,
    uint32_t *rx_data, int flags);

 struct amt2xx_bus_io {
     amt2xx_bus_check_fn check;
     amt2xx_cmd_fn cmd;
 };

 struct amt2xx_config {
	union amt2xx_bus bus;
	const struct amt2xx_bus_io *bus_io;
    const struct gpio_dt_spec cs_gpio;
};

#if AMT2XX_BUS_SPI
#define AMT2XX_SPI_OPERATION (SPI_WORD_SET(8) | SPI_TRANSFER_MSB )
extern const struct amt2xx_bus_io amt2xx_bus_io_spi;
#endif

struct amt2xx_reading {
    uint32_t abs_posn;  /* absolute position */
    uint32_t abs_turns; /* number of turns */
};

 #endif /* ZEPHYR_DRIVERS_SENSOR_AMT2XX_AMT2XX_H_ */
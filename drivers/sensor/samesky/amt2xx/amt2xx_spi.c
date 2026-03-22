/*
 * Copyright (c) 2025 Liam Girdwood
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-specific functionality for AMT2xx accessed via SPI.
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include "amt2xx.h"

#if AMT2XX_BUS_SPI

LOG_MODULE_DECLARE(AMT2XX, CONFIG_SENSOR_LOG_LEVEL);

static int amt2xx_bus_check_spi(const struct amt2xx_config *cfg)
{
    const union amt2xx_bus *bus = &cfg->bus;
    return spi_is_ready_dt(&bus->spi) ? 0 : -ENODEV;
}

static int amt2xx_cmd_spi(const struct amt2xx_config *cfg,
    const struct amt2xx_cmd *cmd,
    uint32_t *rx_data, int flags)
{
    const union amt2xx_bus *bus = &cfg->bus;
    char result[4];

    /* max command length is 4 bytes */
    if (cmd->len > 4) {
        LOG_ERR("cmd len too long: %d\n", cmd->len);
        return -EINVAL;
    }

    /* setup cmd to transmit over SPI */
    const struct spi_buf tx_buf[4] = {
        {
            .buf = (uint8_t *)&cmd->cmd[0],
            .len = 1,
        },
        {
            .buf = (uint8_t *)&cmd->cmd[1],
            .len = 1,
        },
        {
            .buf = (uint8_t *)&cmd->cmd[2],
            .len = 1,
        },
        {
            .buf = (uint8_t *)&cmd->cmd[3],
            .len = 1,
        },
    };
    const struct spi_buf_set tx = {
        .buffers = tx_buf,
        .count = cmd->len,
    };

    /* setup cmd response to receive over SPI */
    struct spi_buf rx_buf[4];
    rx_buf[0].buf = &result[0];
    rx_buf[0].len = 1;
    rx_buf[1].buf = &result[1];
    rx_buf[1].len = 1;
    rx_buf[2].buf = &result[2];
    rx_buf[2].len = 1;
    rx_buf[3].buf = &result[3];
    rx_buf[3].len = 1;
    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = cmd->len,
    };

    /* do we need to manually assert GPIO for CS */
    if (flags & AMT2XX_IO_FLAGS_SPI_CS_GPIO) {
        /* CS GPIO is used */
        gpio_pin_set_dt(&cfg->cs_gpio, 0);
    }

    /* send cmd */
    int ret;
    ret = spi_transceive_dt(&bus->spi, &tx, &rx);

     /* do we need to manually de-assert GPIO for CS */
    if (flags & AMT2XX_IO_FLAGS_SPI_CS_GPIO) {
        /* CS GPIO is used */
        /* sleep for 20 us */
        k_busy_wait(20);
        gpio_pin_set_dt(&cfg->cs_gpio, 1);
    }

    /* did command fail ? */
    if (ret) {
        LOG_ERR("cmd transceive failed: %d\n", ret);
        return ret;
    }

    /* copy result to rx_data */
    memcpy(rx_data, result, cmd->len);
    if (ret) {
        LOG_ERR("cmd failed: %d\n", ret);
    }
    /* and align on endianess */
    switch (cmd->len) {
    case 2:
        *rx_data = sys_be16_to_cpu(*(uint16_t *)result);
        break;
    case 4:
        *rx_data = sys_be32_to_cpu(*(uint32_t *)result);
        break;
    default:
        LOG_ERR("cmd length %d not supported.\n", cmd->len);
        return -EINVAL;
    }

    return ret;
}

const struct amt2xx_bus_io amt2xx_bus_io_spi = {
    .check = amt2xx_bus_check_spi,
    .cmd = amt2xx_cmd_spi,
};

#endif /* AMT2XX_BUS_SPI */

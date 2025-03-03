/* ST Microelectronics ISM330DHCX 6-axis IMU sensor driver
 *
 * Copyright (c) 2020 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/ism330dhcx.pdf
 */

#define DT_DRV_COMPAT st_ism330dhcx

#include <string.h>
#include "ism330dhcx.h"
#include <zephyr/logging/log.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

#define ISM330DHCX_SPI_READ (1 << 7)

LOG_MODULE_DECLARE(ISM330DHCX, CONFIG_SENSOR_LOG_LEVEL);

static int ism330dhcx_spi_read(const struct device *dev, uint8_t reg_addr, uint8_t *value,
			       uint8_t len)
{
	const struct ism330dhcx_config *cfg = dev->config;
	uint8_t buffer_tx[7] = {reg_addr | ISM330DHCX_SPI_READ, 0U, 0U, 0U, 0U, 0U, 0U};
	uint8_t buffer_rx[7] = {0U};

	const struct spi_buf spi_buf_tx = {.buf = buffer_tx, .len = len + 1};
	const struct spi_buf_set tx = {.buffers = &spi_buf_tx, .count = 1};

	const struct spi_buf spi_buf_rx = {.buf = &buffer_rx, .len = len + 1};
	const struct spi_buf_set rx = {.buffers = &spi_buf_rx, .count = 1};

	if (len > 10) {
		return -EIO;
	}

	if (spi_transceive_dt(&cfg->spi, &tx, &rx)) {
		return -EIO;
	}

	for (uint8_t i = 0; i < len; i++) {
		value[i] = buffer_rx[i + 1];
	}
	return 0;
}

static int ism330dhcx_spi_write(const struct device *dev, uint8_t reg_addr, uint8_t *value,
				uint8_t len)
{
	const struct ism330dhcx_config *cfg = dev->config;
	uint8_t cmd[6] = {reg_addr & ~ISM330DHCX_SPI_READ, 0U, 0U, 0U, 0U, 0U};

	if (len > 5) {
		return -EIO;
	}

	// copy the values to write to cmd buffer.
	// cmd[0] is the register address and shouldn't be overwritten.
	for (uint8_t i = 0; i < len; i++) {
		cmd[i+1] = value[i];
	}

	const struct spi_buf tx_buf = {.buf = cmd, .len = len + 1};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

	if (spi_write_dt(&cfg->spi, &tx)) {
		return -EIO;
	}
	return 0;
}

int ism330dhcx_spi_init(const struct device *dev)
{
	struct ism330dhcx_data *data = dev->data;
	const struct ism330dhcx_config *cfg = dev->config;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	};

	data->ctx_spi.read_reg = (stmdev_read_ptr)ism330dhcx_spi_read;
	data->ctx_spi.write_reg = (stmdev_write_ptr)ism330dhcx_spi_write;
	data->ctx_spi.mdelay = (stmdev_mdelay_ptr)stmemsc_mdelay;

	data->ctx = &data->ctx_spi;
	data->ctx->handle = (void *)dev;

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
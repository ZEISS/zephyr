/*
 * Copyright (c) 2023 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "tmc_spi.h"

LOG_MODULE_REGISTER(tmc_spi, CONFIG_TRINAMIC_LOG_LEVEL);

static void parse_tmc_spi_status(const uint8_t status_byte)
{
	if ((status_byte & BIT_MASK(0)) != 0) {
		LOG_WRN("spi dataframe: reset_flag detected");
	}
	if ((status_byte & BIT_MASK(1)) != 0) {
		LOG_WRN("spi dataframe: driver_error(1) detected");
	}
	if ((status_byte & BIT_MASK(2)) != 0) {
		LOG_WRN("spi dataframe: driver_error(2) detected");
	}
}

int tmc_spi_read_register(const struct spi_dt_spec *bus, const tmc_register_t reg, uint32_t *data)
{
	uint8_t tx_buf[5] = {0x7FU & reg, 0, 0, 0, 0};
	uint8_t rx_buf[5] = {0U};
	int ret;

	const struct spi_buf spi_buf_tx = {
		.buf = &tx_buf,
		.len = sizeof(tx_buf),
	};
	struct spi_buf_set tx = {
		.buffers = &spi_buf_tx,
		.count = 1,
	};

	struct spi_buf spi_buf_rx = {
		.buf = &rx_buf,
		.len = sizeof(rx_buf),
	};
	struct spi_buf_set rx = {
		.buffers = &spi_buf_rx,
		.count = 1,
	};

	// send read with the address byte
	ret = spi_transceive_dt(bus, &tx, &rx);
	if (ret < 0) {

		return ret;
	}
	LOG_DBG("TX [0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x]", tx_buf[0], tx_buf[1], tx_buf[2],
		tx_buf[3], tx_buf[4]);
	LOG_DBG("RX [0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x]", rx_buf[0], rx_buf[1], rx_buf[2],
		rx_buf[3], rx_buf[4]);
	parse_tmc_spi_status(rx_buf[0]);

	// read the value from the address
	ret = spi_transceive_dt(bus, &tx, &rx);
	if (ret < 0) {
		return ret;
	}

	*data = ((uint32_t)rx_buf[1] << 24) + ((uint32_t)rx_buf[2] << 16) +
			((uint32_t)rx_buf[3] << 8) + (uint32_t)rx_buf[4];

		LOG_DBG("TX [0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x]", tx_buf[0], tx_buf[1],
			tx_buf[2], tx_buf[3], tx_buf[4]);
		LOG_DBG("RX [0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x]", rx_buf[0], rx_buf[1],
			rx_buf[2], rx_buf[3], rx_buf[4]);
		parse_tmc_spi_status(rx_buf[0]);
	return ret;
}

int tmc_spi_write_register(const struct spi_dt_spec *bus, const tmc_register_t reg, const uint32_t value)
{
	uint8_t tx_buf[5] = {0x80U | reg, value >> 24, value >> 16, value >> 8, value};
	uint8_t rx_buf[5] = {0};
	int ret;

	const struct spi_buf spi_buf_tx = {
		.buf = &tx_buf,
		.len = sizeof(tx_buf),
	};
	struct spi_buf_set tx = {
		.buffers = &spi_buf_tx,
		.count = 1,
	};

	struct spi_buf spi_buf_rx = {
		.buf = &rx_buf,
		.len = sizeof(rx_buf),
	};
	struct spi_buf_set rx = {
		.buffers = &spi_buf_rx,
		.count = 1,
	};

	ret = spi_transceive_dt(bus, &tx, &rx);
	if (ret < 0) {
		return ret;
	}
	LOG_DBG("TX [0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x]", tx_buf[0], tx_buf[1], tx_buf[2],
		tx_buf[3], tx_buf[4]);
	LOG_DBG("RX [0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x]", rx_buf[0], rx_buf[1], rx_buf[2],
		rx_buf[3], rx_buf[4]);

	parse_tmc_spi_status(rx_buf[0]);

	return ret;
}
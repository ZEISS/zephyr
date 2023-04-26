/*
 * Copyright (c) 2023 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_TMC_SPI_H
#define ZEPHYR_DRIVERS_TMC_SPI_H

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/misc/trinamic/trinamic.h>

/**
 * @brief Read a register from the TMC module using the SPI Bus.
 *
 * @param bus SPI DT information of the bus.
 * @param reg Register.
 * @param data Pointer to read value.
 *
 * @return a value from spi_transceive().
 */
int tmc_spi_read_register(const struct spi_dt_spec *bus, const tmc_register_t reg, uint32_t *data);

/**
 * @brief Write into a register in the TMC module using the SPI Bus.
 *
 * @param bus SPI DT information of the bus.
 * @param reg Register.
 * @param data Value to be written in the register.
 * @return
 */
int tmc_spi_write_register(const struct spi_dt_spec *bus, const tmc_register_t reg, const uint32_t data);

#endif /* ZEPHYR_DRIVERS_TMC_SPI_H */

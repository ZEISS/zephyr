/*
 * Copyright (c) 2023 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MISC_TRINAMIC_H
#define ZEPHYR_DRIVERS_MISC_TRINAMIC_H

#include <zephyr/device.h>

/**
 * @brief Provides a type to for the trinamic registers.
 */
typedef uint8_t tmc_register_t;

/** @COND INTERNAL HIDDEN */

typedef int (*read_t)(const struct device *dev, const tmc_register_t reg_addr, uint32_t *reg_val);
typedef int (*write_t)(const struct device *dev, const tmc_register_t reg_addr, const uint32_t reg_val);

__subsystem struct trinamic_driver_api {
	read_t read;
	write_t write;
};

/** @endcond */

/**
 * @brief Read a register from the driver.
 *
 * @param dev Trinamic device instance.
 * @param reg_addr Register.
 * @param reg_val Pointer to read value.
 *
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */
static inline int trinamic_read(const struct device *dev, const tmc_register_t reg_addr, uint32_t *reg_val)
{
	const struct trinamic_driver_api *api = (const struct trinamic_driver_api *)dev->api;

	return api->read(dev, reg_addr, reg_val);
}

/**
 * @brief Write into a register in the driver.
 *
 * @param dev Trinamic device instance.
 * @param reg_addr Register.
 * @param reg_val Value to be written in the register.
 *
 *
 * @retval 0 On success.
 * @retval -errno Other negative errno in case of failure.
 */

static inline int trinamic_write(const struct device *dev, const tmc_register_t reg_addr, uint32_t reg_val)
{
	const struct trinamic_driver_api *api = (const struct trinamic_driver_api *)dev->api;

	return api->write(dev, reg_addr, reg_val);
}

#endif // ZEPHYR_DRIVERS_MISC_TRINAMIC_H

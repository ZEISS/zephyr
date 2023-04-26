/*
* Copyright (c) 2023 Carl Zeiss Meditec AG
*
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef ZEPHYR_DRIVERS_TMC_SHELL_HELPER_H
#define ZEPHYR_DRIVERS_TMC_SHELL_HELPER_H

#include <zephyr/drivers/misc/trinamic/trinamic.h>

enum tmc_register_type {
	READ,
	WRITE,
	READ_WRITE,
	READ_CLEAR
};

struct tmc_map_t {
	const char *name;
	tmc_register_t reg;
	enum tmc_register_type reg_type;
	uint8_t reg_bit_width;
};

#define TMC_SHELL_REG_MAPPING(_name, _reg, _reg_typ, reg_width)                                    \
	{                                                                                          \
		.name = _name, .reg = _reg, .reg_type = _reg_typ, .reg_bit_width = reg_width       \
	}

#endif /* ZEPHYR_DRIVERS_TMC_SHELL_HELPER_H */

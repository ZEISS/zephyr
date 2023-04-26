/*
* Copyright (c) 2023 Carl Zeiss Meditec AG
*
* SPDX-License-Identifier: Apache-2.0
*/

#include <zephyr/device.h>
#include <zephyr/drivers/misc/trinamic/trinamic.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include <stdlib.h>
#include <string.h>

#include "tmc5041.h"
#include "tmc_shell_helper.h"

static const struct tmc_map_t tmc5013_map[] = {
	TMC_SHELL_REG_MAPPING("GCONF", TMC5041_GCONF, READ_WRITE, 11),
	TMC_SHELL_REG_MAPPING("GSTAT", TMC5041_GSTAT, READ_CLEAR,  4),
	TMC_SHELL_REG_MAPPING("INPUT", TMC5041_INPUT, READ, 32),
	TMC_SHELL_REG_MAPPING("X_COMPARE", TMC5041_X_COMPARE, WRITE, 32),
	TMC_SHELL_REG_MAPPING("PWMCONF_MOTOR_1", TMC5041_PWMCONF(0), WRITE, 22),
	TMC_SHELL_REG_MAPPING("PWMCONF_MOTOR_2", TMC5041_PWMCONF(1), WRITE, 22),
	TMC_SHELL_REG_MAPPING("PWM_STATUS_MOTOR_1", TMC5041_PWM_STATUS(0), WRITE, 22),
	TMC_SHELL_REG_MAPPING("PWM_STATUS_MOTOR_2", TMC5041_PWM_STATUS(1), WRITE, 22),
	TMC_SHELL_REG_MAPPING("RAMPMODE_MOTOR_1", TMC5041_RAMPMODE(0), READ_WRITE, 2),
	TMC_SHELL_REG_MAPPING("RAMPMODE_MOTOR_2", TMC5041_RAMPMODE(1), READ_WRITE, 2),
	TMC_SHELL_REG_MAPPING("XACTUAL_MOTOR_1", TMC5041_XACTUAL(0), READ_WRITE, 32),
	TMC_SHELL_REG_MAPPING("XACTUAL_MOTOR_2", TMC5041_XACTUAL(1), READ_WRITE, 32),
	TMC_SHELL_REG_MAPPING("VACTUAL_MOTOR_1", TMC5041_VACTUAL(0), READ, 32),
	TMC_SHELL_REG_MAPPING("VACTUAL_MOTOR_2", TMC5041_VACTUAL(1), READ, 32),
	TMC_SHELL_REG_MAPPING("VACTUAL_MOTOR_1", TMC5041_VACTUAL(0), READ, 32),
	TMC_SHELL_REG_MAPPING("VACTUAL_MOTOR_2", TMC5041_VACTUAL(1), READ, 32),
	TMC_SHELL_REG_MAPPING("VSTART_MOTOR_1", TMC5041_VSTART(0), WRITE, 18),
	TMC_SHELL_REG_MAPPING("VSTART_MOTOR_2", TMC5041_VSTART(1), WRITE, 18),
	TMC_SHELL_REG_MAPPING("A1_MOTOR_1", TMC5041_A1(0), WRITE, 16),
	TMC_SHELL_REG_MAPPING("A1_MOTOR_2", TMC5041_A1(1), WRITE, 16),
	TMC_SHELL_REG_MAPPING("V1_MOTOR_1", TMC5041_V1(0), WRITE, 20),
	TMC_SHELL_REG_MAPPING("V1_MOTOR_2", TMC5041_V1(1), WRITE, 20),
	TMC_SHELL_REG_MAPPING("AMAX_MOTOR_1", TMC5041_AMAX(0), WRITE, 16),
	TMC_SHELL_REG_MAPPING("AMAX_MOTOR_2", TMC5041_AMAX(1), WRITE, 16),
	TMC_SHELL_REG_MAPPING("VMAX_MOTOR_1", TMC5041_VMAX(0), WRITE, 23),
	TMC_SHELL_REG_MAPPING("VMAX_MOTOR_2", TMC5041_VMAX(1), WRITE, 23),
	TMC_SHELL_REG_MAPPING("DMAX_MOTOR_1", TMC5041_DMAX(0), WRITE, 16),
	TMC_SHELL_REG_MAPPING("DMAX_MOTOR_2", TMC5041_DMAX(1), WRITE, 16),
	TMC_SHELL_REG_MAPPING("D1_MOTOR_1", TMC5041_D1(0), WRITE, 16),
	TMC_SHELL_REG_MAPPING("D1_MOTOR_2", TMC5041_D1(1), WRITE, 16),
	TMC_SHELL_REG_MAPPING("VSTOP_MOTOR_1", TMC5041_VSTOP(0), WRITE, 18),
	TMC_SHELL_REG_MAPPING("VSTOP_MOTOR_2", TMC5041_VSTOP(1), WRITE, 18),
	TMC_SHELL_REG_MAPPING("TZEROWAIT_MOTOR_1", TMC5041_TZEROWAIT(0), WRITE, 16),
	TMC_SHELL_REG_MAPPING("TZEROWAIT_MOTOR_2", TMC5041_TZEROWAIT(1), WRITE, 16),
	TMC_SHELL_REG_MAPPING("XTARGET_MOTOR_1", TMC5041_XTARGET(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("XTARGET_MOTOR_2", TMC5041_XTARGET(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("IHOLD_IRUN_MOTOR_1", TMC5041_IHOLD_IRUN(0), WRITE, 14),
	TMC_SHELL_REG_MAPPING("IHOLD_IRUN_MOTOR_2", TMC5041_IHOLD_IRUN(1), WRITE, 14),
	TMC_SHELL_REG_MAPPING("VCOOLTHRS_MOTOR_1", TMC5041_VCOOLTHRS(0), WRITE, 23),
	TMC_SHELL_REG_MAPPING("VCOOLTHRS_MOTOR_2", TMC5041_VCOOLTHRS(1), WRITE, 23),
	TMC_SHELL_REG_MAPPING("VHIGH_MOTOR_1", TMC5041_VHIGH(0), WRITE, 23),
	TMC_SHELL_REG_MAPPING("VHIGH_MOTOR_2", TMC5041_VHIGH(1), WRITE, 23),
	TMC_SHELL_REG_MAPPING("SW_MODE_MOTOR_1", TMC5041_SWMODE(0), WRITE, 12),
	TMC_SHELL_REG_MAPPING("SW_MODE_MOTOR_2", TMC5041_SWMODE(1), WRITE, 12),
	TMC_SHELL_REG_MAPPING("RAMP_STAT_MOTOR_1", TMC5041_RAMPSTAT(0), WRITE, 12),
	TMC_SHELL_REG_MAPPING("RAMP_STAT_MOTOR_2", TMC5041_RAMPSTAT(1), WRITE, 12),
	TMC_SHELL_REG_MAPPING("XLATCH_MOTOR_1", TMC5041_XLATCH(0), WRITE, 12),
	TMC_SHELL_REG_MAPPING("XLATCH_MOTOR_2", TMC5041_XLATCH(1), WRITE, 12),
	TMC_SHELL_REG_MAPPING("MSLUT0_MOTOR_1", TMC5041_MSLUT0(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT0_MOTOR_2", TMC5041_MSLUT0(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT1_MOTOR_1", TMC5041_MSLUT1(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT1_MOTOR_2", TMC5041_MSLUT1(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT2_MOTOR_1", TMC5041_MSLUT2(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT2_MOTOR_2", TMC5041_MSLUT2(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT3_MOTOR_1", TMC5041_MSLUT3(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT3_MOTOR_2", TMC5041_MSLUT3(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT4_MOTOR_1", TMC5041_MSLUT4(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT4_MOTOR_2", TMC5041_MSLUT4(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT5_MOTOR_1", TMC5041_MSLUT5(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT5_MOTOR_2", TMC5041_MSLUT5(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT6_MOTOR_1", TMC5041_MSLUT6(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT6_MOTOR_2", TMC5041_MSLUT6(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT7_MOTOR_1", TMC5041_MSLUT7(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUT7_MOTOR_2", TMC5041_MSLUT7(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUTSEL_MOTOR_1", TMC5041_MSLUTSEL(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUTSEL_MOTOR_2", TMC5041_MSLUTSEL(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUTSTART_MOTOR_1", TMC5041_MSLUTSTART(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSLUTSTART_MOTOR_2", TMC5041_MSLUTSTART(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("MSCNT_MOTOR_1", TMC5041_MSCNT(0), WRITE, 10),
	TMC_SHELL_REG_MAPPING("MSCNT_MOTOR_2", TMC5041_MSCNT(1), WRITE, 10),
	TMC_SHELL_REG_MAPPING("MSCURACT_MOTOR_1", TMC5041_MSCURACT(0), WRITE, 18),
	TMC_SHELL_REG_MAPPING("MSCURACT_MOTOR_2", TMC5041_MSCURACT(1), WRITE, 18),
	TMC_SHELL_REG_MAPPING("CHOPCONF_MOTOR_1", TMC5041_CHOPCONF(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("CHOPCONF_MOTOR_2", TMC5041_CHOPCONF(1), WRITE, 32),
	TMC_SHELL_REG_MAPPING("COOLCONF_MOTOR_1", TMC5041_COOLCONF(0), WRITE, 25),
	TMC_SHELL_REG_MAPPING("COOLCONF_MOTOR_2", TMC5041_COOLCONF(1), WRITE, 25),
	TMC_SHELL_REG_MAPPING("DRVSTATUS_MOTOR_1", TMC5041_DRVSTATUS(0), WRITE, 32),
	TMC_SHELL_REG_MAPPING("DRVSTATUS_MOTOR_2", TMC5041_DRVSTATUS(1), WRITE, 32),
};

static int cmd_tmc_read(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	const struct device *dev;
	tmc_register_t reg_addr;
	uint8_t reg_read_bits;

	uint32_t reg_val;
	bool reg_found = false;
	int err;
	int idx_map;

	dev = device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(sh, "Device unknown (%s)", argv[1]);
		return -ENODEV;
	}

	/* Lookup symbolic register name */
	for (idx_map = 0; idx_map < ARRAY_SIZE(tmc5013_map); idx_map++) {
		if (strcmp(argv[2], tmc5013_map[idx_map].name) == 0) {
			reg_addr = tmc5013_map[idx_map].reg;
			reg_read_bits = tmc5013_map[idx_map].reg_bit_width;
			reg_found = true;
			break;
		}
	}

	if (!reg_found)
	{
		shell_error(sh, "failed to parse register address");
		return -EINVAL;
	}

	err = trinamic_read(dev, reg_addr, &reg_val);
	if (err < 0) {
		shell_error(sh, "failed to read register (err %d)", err);
		return err;
	}
	shell_print(sh, "reg <%x> value:      %lx", reg_addr, (BIT_MASK(reg_read_bits) & reg_val));

	return 0;
}

static int cmd_tmc_write(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	const struct device *dev;
	tmc_register_t reg_addr;
	enum tmc_register_type reg_type;
	char *endptr;

	uint32_t reg_val;
	bool reg_found = false;
	int err;
	int idx_map;

	dev = device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(sh, "Device unknown (%s)", argv[1]);
		return -ENODEV;
	}

	/* Lookup symbolic register name */
	for (idx_map = 0; idx_map < ARRAY_SIZE(tmc5013_map); idx_map++) {
		if (strcmp(argv[2], tmc5013_map[idx_map].name) == 0) {
			reg_addr = tmc5013_map[idx_map].reg;
			reg_type = tmc5013_map[idx_map].reg_type;
			reg_found = true;
			break;
		}
	}

	if (!reg_found)
	{
		shell_error(sh, "failed to parse register address");
		return -EINVAL;
	}

	if ((reg_type == READ) || (reg_type == READ_CLEAR))
	{
		shell_error(sh, "error: attempting to write into a read only register");
		return -EINVAL;
	}

	reg_val = (uint32_t)strtoul(argv[3], &endptr, 10);
	if (*endptr != '\0') {
		shell_error(sh, "failed to parse write value");
		return -EINVAL;
	}

	err = trinamic_write(dev, reg_addr, reg_val);
	if (err < 0) {
		shell_error(sh, "failed to write register (err %d)", err);
		return err;
	}
	shell_print(sh, "write success: reg <%x> value:      %x", reg_addr, reg_val);

	return 0;
}

static void cmd_tmc_register(size_t idx, struct shell_static_entry *entry);

SHELL_DYNAMIC_CMD_CREATE(dcmd_tmc_register, cmd_tmc_register);

static void cmd_tmc_register(size_t idx, struct shell_static_entry *entry)
{
	if (idx < ARRAY_SIZE(tmc5013_map)) {
		entry->syntax = tmc5013_map[idx].name;

	} else {
		entry->syntax = NULL;
	}

	entry->handler = NULL;
	entry->help = "Lists the registers.";
	entry->subcmd = NULL;
}

static void cmd_tmc_device_name_register(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = &dcmd_tmc_register;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_tmc_device_name_register, cmd_tmc_device_name_register);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_trinamic_cmds,
			       SHELL_CMD_ARG(read,
					     &dsub_tmc_device_name_register,
					     "Read register values\n"
					     "Usage: trinamic read <device> <reg_addr>",
					     cmd_tmc_read,
					     3,
					     0),

			       SHELL_CMD_ARG(write,
					     &dsub_tmc_device_name_register,
					     "Write value into register\n"
					     "Usage: trinamic write <device> <reg_addr> <value>",
					     cmd_tmc_write,
					     4,
					     0),
			       SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(tmc5041, &sub_trinamic_cmds, "Trinamic motor controller commands", NULL);


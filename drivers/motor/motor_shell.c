/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include <stdlib.h>
#include <string.h>

#include "zephyr/drivers/motor.h"

#define MOTOR_CONTROLLER_ARGV_INDEX 1
#define AUXILIARY_ARG1              (MOTOR_CONTROLLER_ARGV_INDEX + 1)
#define AUXILIARY_ARG2              (AUXILIARY_ARG1 + 1)

struct motor_direction_t {
	const char *name;
	enum run_direction direction;
};

struct motor_calibrate_func_t {
	const char *name;
	motor_calibrate_func_t motor_calibrate_func;
};

struct motor_position_type_t {
	const char *name;
	enum position_type position_type;
};

int32_t zero_position_in_negative_direction(const struct device *motor, int32_t *min_pos,
					    int32_t *max_pos)
{
	bool stall_status = false;
	int32_t min = 0, max = 0;
	struct motor_run_info run_info = {.direction = POSITIVE};
	struct motor_position_info pos_actual_info = {.type = MOTOR_POSITION_ACTUAL, .position = 0};

	motor_run(motor, &run_info);
	while (stall_status == false) {
		motor_get_stall_status(motor, &stall_status);
		k_sleep(K_MSEC(100));
	}
	motor_stop(motor);
	motor_get_position(motor, &pos_actual_info);
	max = pos_actual_info.position;

	stall_status = false;
	run_info.direction = NEGATIVE;
	motor_run(motor, &run_info);
	while (stall_status == false) {
		motor_get_stall_status(motor, &stall_status);
		k_sleep(K_MSEC(100));
	}
	motor_stop(motor);
	motor_get_position(motor, &pos_actual_info);
	min = pos_actual_info.position;

	*max_pos = max - min;
	*min_pos = 0;

	return 0;
}

int32_t end_to_end_calibrate_function(const struct device *motor, int32_t *min_pos,
				      int32_t *max_pos)
{
	bool stall_status = false;
	struct motor_run_info run_info = {.direction = NEGATIVE};

	motor_run(motor, &run_info);
	while (stall_status == false) {
		motor_get_stall_status(motor, &stall_status);
		k_sleep(K_MSEC(100));
	}
	motor_stop(motor);
	struct motor_position_info pos_actual_info = {.type = MOTOR_POSITION_ACTUAL, .position = 0};

motor_get_position(motor, &pos_actual_info);
	*min_pos = pos_actual_info.position;

	stall_status = false;
	run_info.direction = POSITIVE;
	motor_run(motor, &run_info);
	while (stall_status == false) {
		motor_get_stall_status(motor, &stall_status);
		k_sleep(K_MSEC(100));
	}
	motor_stop(motor);
	motor_get_position(motor, &pos_actual_info);
	*max_pos = pos_actual_info.position;

	return 0;
}

static const struct motor_direction_t motor_direction_map[] = {
	{.name = "positive", .direction = POSITIVE}, {.name = "negative", .direction = NEGATIVE}};

static const struct motor_calibrate_func_t motor_calibrate_func_map[] = {
	{.name = "zero_position_in_neg_direction",
	 .motor_calibrate_func = zero_position_in_negative_direction},
	{.name = "end_to_end_positioning", .motor_calibrate_func = end_to_end_calibrate_function},
};

static const struct motor_position_type_t motor_position_type_map[] = {
	{.name = "min", .position_type = MOTOR_POSITION_MIN},
	{.name = "max", .position_type = MOTOR_POSITION_MAX},
	{.name = "actual", .position_type = MOTOR_POSITION_ACTUAL},
	{.name = "target", .position_type = MOTOR_POSITION_TARGET}};

static int cmd_motor_reset(const struct shell *shell_instance, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *motor_device;

	motor_device = device_get_binding(argv[MOTOR_CONTROLLER_ARGV_INDEX]);
	if (motor_device == NULL) {
		shell_error(shell_instance, "Device unknown (%s)",
			    argv[MOTOR_CONTROLLER_ARGV_INDEX]);
		return -ENODEV;
	}

	char *end_ptr;

	if (*end_ptr != '\0') {
		shell_error(shell_instance, "failed to parse value");
		return -EINVAL;
	}

	shell_print(shell_instance, "resetting %s", motor_device->name);
	motor_reset(motor_device);

	return 0;
}

static void cmd_motor_run_direction(size_t idx, struct shell_static_entry *entry)
{
	if (idx < ARRAY_SIZE(motor_direction_map)) {
		entry->syntax = motor_direction_map[idx].name;
	} else {
		entry->syntax = NULL;
	}

	entry->handler = NULL;
	entry->help = "Lists the directions.";
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dcmd_motor_run_direction, cmd_motor_run_direction);

static void cmd_run_motor_name_register(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = "List Devices";
	entry->subcmd = &dcmd_motor_run_direction;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_run_motor_name_register, cmd_run_motor_name_register);

static int cmd_motor_run(const struct shell *shell_instance, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *motor_device;

	motor_device = device_get_binding(argv[MOTOR_CONTROLLER_ARGV_INDEX]);
	if (motor_device == NULL) {
		shell_error(shell_instance, "Device unknown (%s)",
			    argv[MOTOR_CONTROLLER_ARGV_INDEX]);
		return -ENODEV;
	}
	char *end_ptr;

	if (*end_ptr != '\0') {
		shell_error(shell_instance, "failed to parse value");
		return -EINVAL;
	}

	size_t idx_map;
	struct motor_run_info run_info = {0U};

	for (idx_map = 0U; idx_map < ARRAY_SIZE(motor_direction_map); idx_map++) {
		if (strcmp(argv[AUXILIARY_ARG1], motor_direction_map[idx_map].name) == 0) {
			run_info.direction = motor_direction_map[idx_map].direction;
			shell_print(shell_instance, "freewheeling %s in %s direction",
				    motor_device->name, motor_direction_map[idx_map].name);

			motor_run(motor_device, &run_info);
		}
	}

	return 0;
}

static int cmd_motor_stop(const struct shell *shell_instance, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *motor_device;

	motor_device = device_get_binding(argv[MOTOR_CONTROLLER_ARGV_INDEX]);
	if (motor_device == NULL) {
		shell_error(shell_instance, "Device unknown (%s)",
			    argv[MOTOR_CONTROLLER_ARGV_INDEX]);
		return -ENODEV;
	}
	char *end_ptr;

	if (*end_ptr != '\0') {
		shell_error(shell_instance, "failed to parse value");
		return -EINVAL;
	}

	shell_print(shell_instance, "stopping %s", motor_device->name);

	motor_stop(motor_device);

	return 0;
}

static void cmd_motor_name_register(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = "List Devices";
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_motor_name_register, cmd_motor_name_register);

static void cmd_motor_calibrate_funcs(size_t idx, struct shell_static_entry *entry)
{
	if (idx < ARRAY_SIZE(motor_calibrate_func_map)) {
		entry->syntax = motor_calibrate_func_map[idx].name;
	} else {
		entry->syntax = NULL;
	}

	entry->handler = NULL;
	entry->help = "Lists the Calibration Functions.";
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dcmd_motor_calibrate_funcs, cmd_motor_calibrate_funcs);

static void cmd_calibrate_motor_name_register(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = "List Devices";
	entry->subcmd = &dcmd_motor_calibrate_funcs;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_calibrate_motor_name_register, cmd_calibrate_motor_name_register);

static int cmd_motor_calibrate(const struct shell *shell_instance, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *motor_device;

	motor_device = device_get_binding(argv[MOTOR_CONTROLLER_ARGV_INDEX]);
	if (motor_device == NULL) {
		shell_error(shell_instance, "Device unknown (%s)",
			    argv[MOTOR_CONTROLLER_ARGV_INDEX]);
		return -ENODEV;
	}

	char *end_ptr;

	if (*end_ptr != '\0') {
		shell_error(shell_instance, "failed to parse value");
		return -EINVAL;
	}

	size_t idx_map;

	for (idx_map = 0U; idx_map < ARRAY_SIZE(motor_calibrate_func_map); idx_map++) {
		if (strcmp(argv[AUXILIARY_ARG1], motor_calibrate_func_map[idx_map].name) == 0) {
			shell_print(shell_instance, "calibrating %s", motor_device->name);

			motor_register_calibrate_func(
				motor_device,
				motor_calibrate_func_map[idx_map].motor_calibrate_func);
			motor_calibrate(motor_device);
		}
	}

	return 0;
}

static void cmd_motor_positions(size_t idx, struct shell_static_entry *entry)
{
	if (idx < ARRAY_SIZE(motor_position_type_map)) {
		entry->syntax = motor_position_type_map[idx].name;
	} else {
		entry->syntax = NULL;
	}

	entry->handler = NULL;
	entry->help = "Lists the Stepper Motor Position Types";
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dcmd_motor_positions, cmd_motor_positions);

static void cmd_pos_motor_name_register(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = "List Devices";
	entry->subcmd = &dcmd_motor_positions;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_pos_motor_name_register, cmd_pos_motor_name_register);

static int cmd_motor_set_position(const struct shell *shell_instance, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *motor_device;

	motor_device = device_get_binding(argv[MOTOR_CONTROLLER_ARGV_INDEX]);
	if (motor_device == NULL) {
		shell_error(shell_instance, "Device unknown (%s)",
			    argv[MOTOR_CONTROLLER_ARGV_INDEX]);
		return -ENODEV;
	}

	size_t idx_map;
	struct motor_position_info pos_info = {0U};

	char *end_ptr;
	float position = (float)strtof(argv[AUXILIARY_ARG1], &end_ptr);

	if (*end_ptr != '\0') {
		shell_error(shell_instance, "failed to parse value");
		return -EINVAL;
	}

	shell_print(shell_instance, "setting %s motor %s position to %f", motor_device->name,
		    motor_position_type_map[idx_map].name, position);

	motor_set_angle(motor_device, position);

	return 0;
}

static int cmd_motor_get_position(const struct shell *shell_instance, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	const struct device *motor_device;

	motor_device = device_get_binding(argv[MOTOR_CONTROLLER_ARGV_INDEX]);
	if (motor_device == NULL) {
		shell_error(shell_instance, "Device unknown (%s)",
			    argv[MOTOR_CONTROLLER_ARGV_INDEX]);
		return -ENODEV;
	}
	char *end_ptr;

	if (*end_ptr != '\0') {
		shell_error(shell_instance, "failed to parse value");
		return -EINVAL;
	}
	size_t idx_map;
	struct motor_position_info pos_info = {0U};

	for (idx_map = 0U; idx_map < ARRAY_SIZE(motor_position_type_map); idx_map++) {
		if (strcmp(argv[AUXILIARY_ARG1], motor_position_type_map[idx_map].name) == 0) {
			pos_info.type = motor_position_type_map[idx_map].position_type;

			motor_get_position(motor_device, &pos_info);
			shell_print(shell_instance, "setting %s motor %s position to %d",
				    motor_device->name, motor_position_type_map[idx_map].name,
				    pos_info.position);
		}
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_motor_device_cmds,

	SHELL_CMD_ARG(reset, &dsub_motor_name_register, "Reset the Motor\n", cmd_motor_reset, 3, 0),
	SHELL_CMD_ARG(run, &dsub_run_motor_name_register, "Run the Motor\n", cmd_motor_run, 4, 0),
	SHELL_CMD_ARG(stop, &dsub_motor_name_register, "Stop the Motor\n", cmd_motor_stop, 3, 0),
	SHELL_CMD_ARG(calibrate, &dsub_calibrate_motor_name_register, "Calibrate the Motor\n",
		      cmd_motor_calibrate, 4, 0),
	SHELL_CMD_ARG(set_position, &dsub_pos_motor_name_register, "Move the motor with steps\n",
		      cmd_motor_set_position, 3, 0),
	SHELL_CMD_ARG(get_position, &dsub_pos_motor_name_register, "Get the motor position\n",
		      cmd_motor_get_position, 4, 0),

	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(motor, &sub_motor_device_cmds, "Stepper Motor Device Commands", NULL);

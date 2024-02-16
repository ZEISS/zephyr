/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_motor_H
#define ZEPHYR_INCLUDE_motor_H

/** \addtogroup motor
 *  @{
 */

/**
 * @file
 * @brief Public API for Motor
 *
 * The motor device API provides following functions:
 *	- Reset Motor
 *	- Set/Get Position of Motor
 *	- Run/Stop Motor
 *	- Direct Write Access to Motor Controller Specific Registers
 *	- Direct Read Access to Motor Controller Specific Registers
 */

#include <zephyr/device.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Motor Position Type required while setting or getting motor positions.
 */
enum position_type {
	/** Target Motor Position, distance or steps that the motor shall traverse */
	MOTOR_POSITION_TARGET,
	/** Actual Motor Position, this could be counted internally or fetched from controller */
	MOTOR_POSITION_ACTUAL,
	/** Minimum Traversable Motor Position, this could be calibrated using end stop */
	MOTOR_POSITION_MIN,
	/** Maximum Traversable Motor Position, this could be calibrated using end stop */
	MOTOR_POSITION_MAX,
};

enum run_direction {
	POSITIVE,
	NEGATIVE,
};

/**
 * @brief Motor Position Info required while setting or getting motor positions.
 *
 * Here is an example of how to use this struct:
 *
 *      const struct motor_position_info target_position_info = {
 *	            .format = MICRO_STEP_256, .type = MOTOR_POSITION_TARGET, .position = 51200};
 *      motor_set_position(my_motor, &target_position_info);
 *
 *      struct motor_position_info actual_position_info = {
 *	            .format = MICRO_STEP_256, .type = MOTOR_POSITION_ACTUAL};
 *      motor_get_position(my_motor, &actual_position_info);
 */
struct motor_position_info {
	/** allows the user to specify position type such as Min, Max, Actual, Target */
	enum position_type type;
	/** position value */
	int32_t position;
};

/**
 * @brief Motor Run Info required while driving Motor in Freewheeling Mode.
 *
 * Here is an example of how to use this struct:
 *
 *      const struct motor_run_info target_position_info = {
 *	            .direction = POSITIVE, .velocity = 200};
 *      motor_run(my_motor, &target_position_info);
 */
struct motor_run_info {
	enum run_direction direction;
	int32_t velocity;
};

struct motor {
	const struct device *const controller;
	const uint8_t motor_index;
};

/**
 * @typedef motor_set_angle_t
 * @brief Set the position of a given motor device
 *
 * @see motor_set_angle for more information
 */
typedef int32_t (*motor_set_angle_t)(const struct device *dev, const float angle);

typedef int32_t (*motor_rotate_t)(const struct device *dev, const float angle);

/**
 * @typedef motor_get_position_t
 * @brief Get the position of a given motor device
 *
 * @see motor_get_position for more information
 */
typedef int32_t (*motor_get_position_t)(const struct device *dev,
					struct motor_position_info *position_info);

/**
 * @typedef motor_reset_t
 * @brief Reset the motor device in a user-specific manner
 *
 * @see motor_reset for more information
 */
typedef int32_t (*motor_reset_t)(const struct device *dev);

/**
 * @typedef motor_run_t
 * @brief Run a.k.a do Freewheeling of a given motor device
 *
 * @see motor_run for more information
 */
typedef int32_t (*motor_run_t)(const struct device *dev, const struct motor_run_info *run_info);

/**
 * @typedef motor_stop_t
 * @brief Stop a given motor device
 *
 * @see motor_stop for more information
 */
typedef int32_t (*motor_stop_t)(const struct device *dev);

/**
 * @typedef motor_get_stall_status_t
 * @brief Get the stall status of a given motor device
 *
 * @see motor_get_stall_status for more information
 */
typedef int32_t (*motor_get_stall_status_t)(const struct device *dev, bool *stall_status);

/**
 * @typedef motor_calibrate_t
 * @brief Calibrate a given motor device
 *
 * @see motor_calibrate for more information
 */
typedef int32_t (*motor_calibrate_t)(const struct device *dev);

typedef int32_t (*motor_calibrate_func_t)(const struct device *dev, int32_t *min_pos,
					  int32_t *max_pos);
/**
 * @typedef motor_register_calibrate_func_t
 * @brief Register a calibration function for a given motor device
 *
 * @see motor_register_calibrate_func for more information
 */
typedef int32_t (*motor_register_calibrate_func_t)(const struct device *dev,
						   motor_calibrate_func_t);

/**
 * @typedef motor_controller_reg_write_t
 * @brief write to a certain register in motor controller directly
 *
 * @see motor_controller_write_reg for more details
 */
typedef int32_t (*motor_controller_reg_write_t)(const struct device *dev, uint8_t reg_address,
						uint32_t reg_value);

/**
 * @typedef motor_controller_reg_read_t
 * @brief read a certain register value from motor controller directly
 *
 * @see motor_read_reg for more details
 */
typedef int32_t (*motor_controller_reg_read_t)(const struct device *dev, uint8_t reg_address,
					       uint32_t *reg_value);

__subsystem struct motor_api {
	motor_reset_t motor_reset;
	motor_set_angle_t motor_set_angle;
	motor_rotate_t motor_rotate;
	motor_get_position_t motor_get_position;
	motor_get_stall_status_t motor_get_stall_status;
	motor_run_t motor_run;
	motor_stop_t motor_stop;
	motor_calibrate_t motor_calibrate;
	motor_register_calibrate_func_t motor_register_calibrate_func;

	motor_controller_reg_write_t motor_controller_write_reg;
	motor_controller_reg_read_t motor_controller_read_reg;
};

/**
 * @brief Reset motor device
 *
 * @param motor Pointer to the motor device
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t motor_reset(const struct device *dev);

static inline int32_t z_impl_motor_reset(const struct device *dev)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_reset(dev);
}

/**
 * @brief Run a.k.a freewheel a motor device
 *
 * @param dev Pointer to the motor device
 * @param run_info Pointer to the run information such as direction and velocity
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t motor_run(const struct device *dev, const struct motor_run_info *run_info);

static inline int32_t z_impl_motor_run(const struct device *dev,
				       const struct motor_run_info *run_info)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_run(dev, run_info);
}

/**
 * @brief Stop a motor device
 *
 * @param motor Pointer to the motor device
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t motor_stop(const struct device *dev);

static inline int32_t z_impl_motor_stop(const struct device *dev)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_stop(dev);
}

/**
 * @brief Set position of a motor device
 *
 * @param motor Pointer to the motor device
 * @param position_info Pointer to position information such as format, type and position
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t motor_set_angle(const struct device *dev, const float angle);

static inline int32_t z_impl_motor_set_angle(const struct device *dev, const float angle)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_set_angle(dev, angle);
}

__syscall int32_t motor_rotate(const struct device *dev, const float angle);

static inline int32_t z_impl_motor_rotate(const struct device *dev, const float angle)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_rotate(dev, angle);
}

/**
 * @brief Get position of a motor device
 *
 * @param motor Pointer to motor device
 * @param position_info Pointer to position information such as format, type and position
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t motor_get_position(const struct device *dev,
				     struct motor_position_info *position_info);

static inline int32_t z_impl_motor_get_position(const struct device *dev,
						struct motor_position_info *position_info)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_get_position(dev, position_info);
}

/**
 * @brief Get stall status of a motor device
 *
 * @param motor Pointer to motor device
 * @param stall_status Pointer to boolean signalling if the motor device is stalled or not
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t motor_get_stall_status(const struct device *dev, bool *stall_status);

static inline int32_t z_impl_motor_get_stall_status(const struct device *dev, bool *stall_status)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_get_stall_status(dev, stall_status);
}

/**
 * @brief Calibrate the Motor Device
 * @details A pre-registered calibration routine is executed
 *
 * @param motor Pointer to motor device
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t motor_calibrate(const struct device *dev);

static inline int32_t z_impl_motor_calibrate(const struct device *dev)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_calibrate(dev);
}

static inline int32_t z_impl_motor_register_calibrate_func(const struct device *dev,
							   motor_calibrate_func_t calibrate_func)
{
	if (dev == NULL) {
		return -ENODEV;
	}
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_register_calibrate_func(dev, calibrate_func);
}

/**
 * @brief Register a calibration function
 *
 * @param motor Pointer to motor device
 * @param calibrate_func Pointer to the calibration function
 *
 * @return 0 if successful, negative code if failure
 */
static inline int32_t motor_register_calibrate_func(const struct device *dev,
						    motor_calibrate_func_t calibrate_func)
{
	return z_impl_motor_register_calibrate_func(dev, calibrate_func);
}

/**
 * @brief motor_controller_write_reg
 *
 * @param dev Pointer to the device i.e. motor controller
 * @param reg_address register address to write to
 * @param reg_value value to be written to register
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t motor_controller_write_reg(const struct device *dev, uint8_t reg_address,
					     uint32_t reg_value);

static inline int32_t z_impl_motor_controller_write_reg(const struct device *dev,
							uint8_t reg_address, uint32_t reg_value)
{
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_controller_write_reg(dev, reg_address, reg_value);
}

/**
 * @brief motor_controller_read_reg
 *
 * @param dev Pointer to the device i.e. motor controller
 * @param reg_address register address to read from
 * @param reg_value Pointer holding the value of register
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t motor_controller_read_reg(const struct device *dev, uint8_t reg_address,
					    uint32_t *reg_value);

static inline int32_t z_impl_motor_controller_read_reg(const struct device *dev,
						       uint8_t reg_address, uint32_t *reg_value)
{
	const struct motor_api *api = (const struct motor_api *)dev->api;

	return api->motor_controller_read_reg(dev, reg_address, reg_value);
}

#define motor_DT_GET(label)                                                                        \
	{                                                                                          \
		.controller = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(label))),                       \
		.motor_index = DT_NODE_CHILD_IDX(DT_NODELABEL(label)),                             \
	}

#ifdef __cplusplus
}
#endif

/** @}*/

#include <syscalls/motor.h>

#endif /* ZEPHYR_INCLUDE_motor_H */

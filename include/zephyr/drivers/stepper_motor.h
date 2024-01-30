/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_STEPPER_MOTOR_H
#define ZEPHYR_INCLUDE_STEPPER_MOTOR_H

/** \addtogroup stepper_motor
 *  @{
 */

/**
 * @file
 * @brief Public API for Stepper Motor
 *
 * The stepper motor device API provides following functions:
 *	- Reset Stepper Motor
 *	- Set/Get Position of Stepper Motor
 *	- Run/Stop Stepper Motor
 *	- Direct Write Access to Stepper Motor Controller Specific Registers
 *	- Direct Read Access to Stepper Motor Controller Specific Registers
 */

#include <zephyr/device.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Stepper Motor Position Format required while setting or getting stepper motor positions.
 */
enum position_format {
	MICRO_STEP_2,
	MICRO_STEP_4,
	MICRO_STEP_8,
	MICRO_STEP_16,
	MICRO_STEP_32,
	MICRO_STEP_64,
	MICRO_STEP_128,
	MICRO_STEP_256,
	FULL_STEP,
	/** Can be calculated using gear ratio */
	RAW_IN_MM,
};

/**
 * @brief Stepper Motor Position Type required while setting or getting stepper motor positions.
 */
enum position_type {
	/** Minimum Traversable Motor Position, this could be calibrated using end stop */
	MOTOR_POSITION_MIN,
	/** Maximum Traversable Motor Position, this could be calibrated using end stop */
	MOTOR_POSITION_MAX,
	/** Actual Motor Position, this could be counted internally or fetched from controller */
	MOTOR_POSITION_ACTUAL,
	/** Target Motor Position, distance or steps that the motor shall traverse */
	MOTOR_POSITION_TARGET,
};

enum run_direction {
	POSITIVE,
	NEGATIVE,
};

/**
 * @brief Stepper Motor Position Info required while setting or getting stepper motor positions.
 *
 * Here is an example of how to use this struct:
 *
 *      const struct stepper_motor_position_info target_position_info = {
 *	            .format = MICRO_STEP_256, .type = MOTOR_POSITION_TARGET, .position = 51200};
 *      stepper_motor_set_position(my_motor, &target_position_info);
 *
 *      struct stepper_motor_position_info actual_position_info = {
 *	            .format = MICRO_STEP_256, .type = MOTOR_POSITION_ACTUAL};
 *      stepper_motor_get_position(my_motor, &actual_position_info);
 */
struct stepper_motor_position_info {
	/** allows the user to specify if the desired position is to be in raw or mircrosteps */
	enum position_format format;
	/** allows the user to specify position type such as Min, Max, Actual, Target */
	enum position_type type;
	/** position value */
	int32_t position;
};

/**
 * @brief Stepper Motor Run Info required while driving Motor in Freewheeling Mode.
 *
 * Here is an example of how to use this struct:
 *
 *      const struct stepper_motor_run_info target_position_info = {
 *	            .direction = POSITIVE, .velocity = 200};
 *      stepper_motor_run(my_motor, &target_position_info);
 */
struct stepper_motor_run_info {
	enum run_direction direction;
	int32_t velocity;
};

struct stepper_motor {
	const struct device *const controller;
	const uint8_t motor_index;
};

/**
 * @typedef stepper_motor_set_position_t
 * @brief Set the position of a given stepper motor device
 *
 * @see stepper_motor_set_position for more information
 */
typedef int32_t (*stepper_motor_set_position_t)(
	const struct stepper_motor *motor,
	const struct stepper_motor_position_info *position_info);

/**
 * @typedef stepper_motor_get_position_t
 * @brief Get the position of a given stepper motor device
 *
 * @see stepper_motor_get_position for more information
 */
typedef int32_t (*stepper_motor_get_position_t)(const struct stepper_motor *motor,
						struct stepper_motor_position_info *position_info);

/**
 * @typedef stepper_motor_reset_t
 * @brief Reset the stepper motor device in a user-specific manner
 *
 * @see stepper_motor_reset for more information
 */
typedef int32_t (*stepper_motor_reset_t)(const struct stepper_motor *motor);

/**
 * @typedef stepper_motor_run_t
 * @brief Run a.k.a do Freewheeling of a given stepper motor device
 *
 * @see stepper_motor_run for more information
 */
typedef int32_t (*stepper_motor_run_t)(const struct stepper_motor *motor,
				       const struct stepper_motor_run_info *run_info);

/**
 * @typedef stepper_motor_stop_t
 * @brief Stop a given stepper motor device
 *
 * @see stepper_motor_stop for more information
 */
typedef int32_t (*stepper_motor_stop_t)(const struct stepper_motor *motor);

/**
 * @typedef stepper_motor_get_stall_status_t
 * @brief Get the stall status of a given stepper motor device
 *
 * @see stepper_motor_get_stall_status for more information
 */
typedef int32_t (*stepper_motor_get_stall_status_t)(const struct stepper_motor *motor,
						    bool *stall_status);

/**
 * @typedef stepper_motor_calibrate_t
 * @brief Calibrate a given stepper motor device
 *
 * @see stepper_motor_calibrate for more information
 */
typedef int32_t (*stepper_motor_calibrate_t)(const struct stepper_motor *motor);

typedef int32_t (*stepper_motor_calibrate_func_t)(const struct stepper_motor *motor,
						  int32_t *min_pos, int32_t *max_pos);
/**
 * @typedef stepper_motor_register_calibrate_func_t
 * @brief Register a calibration function for a given stepper motor device
 *
 * @see stepper_motor_register_calibrate_func for more information
 */
typedef int32_t (*stepper_motor_register_calibrate_func_t)(
	const struct stepper_motor *motor, stepper_motor_calibrate_func_t);

/**
 * @typedef stepper_motor_controller_reg_write_t
 * @brief write to a certain register in motor controller directly
 *
 * @see stepper_motor_controller_write_reg for more details
 */
typedef int32_t (*stepper_motor_controller_reg_write_t)(const struct device *dev,
							uint8_t reg_address, uint32_t reg_value);

/**
 * @typedef stepper_motor_controller_reg_read_t
 * @brief read a certain register value from motor controller directly
 *
 * @see stepper_motor_read_reg for more details
 */
typedef int32_t (*stepper_motor_controller_reg_read_t)(const struct device *dev,
						       uint8_t reg_address, uint32_t *reg_value);

__subsystem struct stepper_motor_api {
	stepper_motor_reset_t stepper_motor_reset;
	stepper_motor_set_position_t stepper_motor_set_position;
	stepper_motor_get_position_t stepper_motor_get_position;
	stepper_motor_get_stall_status_t stepper_motor_get_stall_status;
	stepper_motor_run_t stepper_motor_run;
	stepper_motor_stop_t stepper_motor_stop;
	stepper_motor_calibrate_t stepper_motor_calibrate;
	stepper_motor_register_calibrate_func_t stepper_motor_register_calibrate_func;

	stepper_motor_controller_reg_write_t stepper_motor_controller_write_reg;
	stepper_motor_controller_reg_read_t stepper_motor_controller_read_reg;
};

/**
 * @brief Reset stepper motor device
 *
 * @param motor Pointer to the stepper motor device
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t stepper_motor_reset(const struct stepper_motor *motor);

static inline int32_t z_impl_stepper_motor_reset(const struct stepper_motor *motor)
{
	if (motor->controller == NULL) {
		return -ENODEV;
	}
	const struct stepper_motor_api *api =
		(const struct stepper_motor_api *)motor->controller->api;

	return api->stepper_motor_reset(motor);
}

/**
 * @brief Run a.k.a freewheel a stepper motor device
 *
 * @param motor Pointer to the stepper motor device
 * @param run_info Pointer to the run information such as direction and velocity
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t stepper_motor_run(const struct stepper_motor *motor,
				    const struct stepper_motor_run_info *run_info);

static inline int32_t z_impl_stepper_motor_run(const struct stepper_motor *motor,
					       const struct stepper_motor_run_info *run_info)
{
	if (motor->controller == NULL) {
		return -ENODEV;
	}
	const struct stepper_motor_api *api =
		(const struct stepper_motor_api *)motor->controller->api;

	return api->stepper_motor_run(motor, run_info);
}

/**
 * @brief Stop a stepper motor device
 *
 * @param motor Pointer to the stepper motor device
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t stepper_motor_stop(const struct stepper_motor *motor);

static inline int32_t z_impl_stepper_motor_stop(const struct stepper_motor *motor)
{
	if (motor->controller == NULL) {
		return -ENODEV;
	}
	const struct stepper_motor_api *api =
		(const struct stepper_motor_api *)motor->controller->api;

	return api->stepper_motor_stop(motor);
}

/**
 * @brief Set position of a stepper motor device
 *
 * @param motor Pointer to the stepper motor device
 * @param position_info Pointer to position information such as format, type and position
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t stepper_motor_set_position(const struct stepper_motor *motor,
			   const struct stepper_motor_position_info *position_info);

static inline int32_t
z_impl_stepper_motor_set_position(const struct stepper_motor *motor,
				  const struct stepper_motor_position_info *position_info)
{
	if (motor->controller == NULL) {
		return -ENODEV;
	}
	const struct stepper_motor_api *api =
		(const struct stepper_motor_api *)motor->controller->api;

	return api->stepper_motor_set_position(motor, position_info);
}

/**
 * @brief Get position of a stepper motor device
 *
 * @param motor Pointer to stepper motor device
 * @param position_info Pointer to position information such as format, type and position
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t stepper_motor_get_position(const struct stepper_motor *motor,
					     struct stepper_motor_position_info *position_info);

static inline int32_t
z_impl_stepper_motor_get_position(const struct stepper_motor *motor,
				  struct stepper_motor_position_info *position_info)
{
	if (motor->controller == NULL) {
		return -ENODEV;
	}
	const struct stepper_motor_api *api =
		(const struct stepper_motor_api *)motor->controller->api;

	return api->stepper_motor_get_position(motor, position_info);
}

/**
 * @brief Get stall status of a stepper motor device
 *
 * @param motor Pointer to stepper motor device
 * @param stall_status Pointer to boolean signalling if the stepper motor device is stalled or not
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t stepper_motor_get_stall_status(const struct stepper_motor *motor,
						 bool *stall_status);

static inline int32_t
z_impl_stepper_motor_get_stall_status(const struct stepper_motor *motor, bool *stall_status)
{
	if (motor->controller == NULL) {
		return -ENODEV;
	}
	const struct stepper_motor_api *api =
		(const struct stepper_motor_api *)motor->controller->api;

	return api->stepper_motor_get_stall_status(motor, stall_status);
}

/**
 * @brief Calibrate the Stepper Motor Device
 * @details A pre-registered calibration routine is executed
 *
 * @param motor Pointer to stepper motor device
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t stepper_motor_calibrate(const struct stepper_motor *motor);

static inline int32_t z_impl_stepper_motor_calibrate(const struct stepper_motor *motor)
{
	if (motor->controller == NULL) {
		return -ENODEV;
	}
	const struct stepper_motor_api *api =
		(const struct stepper_motor_api *)motor->controller->api;

	return api->stepper_motor_calibrate(motor);
}

static inline int32_t
z_impl_stepper_motor_register_calibrate_func(const struct stepper_motor *motor,
					     stepper_motor_calibrate_func_t calibrate_func)
{
	if (motor->controller == NULL) {
		return -ENODEV;
	}
	const struct stepper_motor_api *api =
		(const struct stepper_motor_api *)motor->controller->api;

	return api->stepper_motor_register_calibrate_func(motor, calibrate_func);
}

/**
 * @brief Register a calibration function
 *
 * @param motor Pointer to stepper motor device
 * @param calibrate_func Pointer to the calibration function
 *
 * @return 0 if successful, negative code if failure
 */
static inline int32_t
stepper_motor_register_calibrate_func(const struct stepper_motor *motor,
				      stepper_motor_calibrate_func_t calibrate_func)
{
	return z_impl_stepper_motor_register_calibrate_func(motor, calibrate_func);
}

/**
 * @brief stepper_motor_controller_write_reg
 *
 * @param dev Pointer to the device i.e. motor controller
 * @param reg_address register address to write to
 * @param reg_value value to be written to register
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t stepper_motor_controller_write_reg(const struct device *dev, uint8_t reg_address,
						     uint32_t reg_value);

static inline int32_t z_impl_stepper_motor_controller_write_reg(const struct device *dev,
								uint8_t reg_address,
								uint32_t reg_value)
{
	const struct stepper_motor_api *api = (const struct stepper_motor_api *)dev->api;

	return api->stepper_motor_controller_write_reg(dev, reg_address, reg_value);
}

/**
 * @brief stepper_motor_controller_read_reg
 *
 * @param dev Pointer to the device i.e. motor controller
 * @param reg_address register address to read from
 * @param reg_value Pointer holding the value of register
 *
 * @return 0 if successful, negative code if failure
 */
__syscall int32_t stepper_motor_controller_read_reg(const struct device *dev, uint8_t reg_address,
						    uint32_t *reg_value);

static inline int32_t z_impl_stepper_motor_controller_read_reg(const struct device *dev,
							       uint8_t reg_address,
							       uint32_t *reg_value)
{
	const struct stepper_motor_api *api = (const struct stepper_motor_api *)dev->api;

	return api->stepper_motor_controller_read_reg(dev, reg_address, reg_value);
}

#define STEPPER_MOTOR_DT_GET(label)                                                                \
	{                                                                                          \
		.controller = DEVICE_DT_GET(DT_PARENT(DT_NODELABEL(label))),                       \
		.motor_index = DT_NODE_CHILD_IDX(DT_NODELABEL(label)),                             \
	}

#ifdef __cplusplus
}
#endif

/** @}*/

#include <syscalls/stepper_motor.h>

#endif /* ZEPHYR_INCLUDE_STEPPER_MOTOR_H */

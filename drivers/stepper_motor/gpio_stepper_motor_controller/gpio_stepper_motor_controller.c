/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT gpio_stepper_motor_controller

#include "zephyr/drivers/gpio.h"
#include <zephyr/kernel.h>
#include "zephyr/drivers/stepper_motor.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gpio_stepper_motor_controller, CONFIG_STEPPER_MOTOR_LOG_LEVEL);

struct gpio_stepper_motor_controller_data {
	int32_t min_position;
	int32_t max_position;
	int32_t actual_position;
	int32_t target_position;
	uint32_t actual_speed;
};

struct gpio_stepper_motor_controller_config {
	const uint16_t motor_steps_per_rev;
	const double motor_gear_ratio;
	const struct gpio_dt_spec *gpio_in_pins;
	const uint8_t number_in_pins;
};

static int32_t stepper_motor_move_with_4_gpios(const struct gpio_dt_spec *gpio_spec,
					       int step_number)
{
	switch (abs(step_number) % 4) {
	case 0:
		LOG_DBG("Step 1010 %d %d %d %d", gpio_spec[0].pin, gpio_spec[1].pin,
			gpio_spec[2].pin, gpio_spec[3].pin);
		gpio_pin_configure_dt(&gpio_spec[0], GPIO_OUTPUT_HIGH);
		gpio_pin_configure_dt(&gpio_spec[1], GPIO_OUTPUT_LOW);
		gpio_pin_configure_dt(&gpio_spec[2], GPIO_OUTPUT_HIGH);
		gpio_pin_configure_dt(&gpio_spec[3], GPIO_OUTPUT_LOW);
		break;
	case 1:
		LOG_DBG("Step 0110");
		gpio_pin_configure_dt(&gpio_spec[0], GPIO_OUTPUT_LOW);
		gpio_pin_configure_dt(&gpio_spec[1], GPIO_OUTPUT_HIGH);
		gpio_pin_configure_dt(&gpio_spec[2], GPIO_OUTPUT_HIGH);
		gpio_pin_configure_dt(&gpio_spec[3], GPIO_OUTPUT_LOW);
		break;
	case 2:
		LOG_DBG("Step 0101");
		gpio_pin_configure_dt(&gpio_spec[0], GPIO_OUTPUT_LOW);
		gpio_pin_configure_dt(&gpio_spec[1], GPIO_OUTPUT_HIGH);
		gpio_pin_configure_dt(&gpio_spec[2], GPIO_OUTPUT_LOW);
		gpio_pin_configure_dt(&gpio_spec[3], GPIO_OUTPUT_HIGH);
		break;
	case 3:
		LOG_DBG("Step 1001");
		gpio_pin_configure_dt(&gpio_spec[0], GPIO_OUTPUT_HIGH);
		gpio_pin_configure_dt(&gpio_spec[1], GPIO_OUTPUT_LOW);
		gpio_pin_configure_dt(&gpio_spec[2], GPIO_OUTPUT_LOW);
		gpio_pin_configure_dt(&gpio_spec[3], GPIO_OUTPUT_HIGH);
		break;
	}
	return 0;
}

static int32_t stepper_motor_move(const struct stepper_motor *motor)
{
	struct gpio_stepper_motor_controller_data *motor_data =
		(struct gpio_stepper_motor_controller_data *)motor->controller->data;
	const struct gpio_stepper_motor_controller_config *motor_config =
		(const struct gpio_stepper_motor_controller_config *)motor->controller->config;
	if (motor_data->actual_speed == 0) {
		motor_data->actual_speed = 10;
		LOG_INF("Running Motor with default speed");
	}
	uint32_t step_delay = 60L * 1000L * 1000L /
			      (motor_config->motor_steps_per_rev * motor_config->motor_gear_ratio) /
			      motor_data->actual_speed;
	while (motor_data->target_position != motor_data->actual_position) {

		int32_t remaining_steps = motor_data->target_position - motor_data->actual_position;

		LOG_DBG("Actual Position %d", motor_data->actual_position);
		switch (motor_config->number_in_pins) {
		case 4: {
			stepper_motor_move_with_4_gpios(motor_config->gpio_in_pins,
							motor_data->actual_position);
			break;
		}
		default: {
			return -ENOTSUP;
		}
		}
		if (remaining_steps > 0) {
			motor_data->actual_position++;
		} else {
			motor_data->actual_position--;
		}
		k_sleep(K_USEC(step_delay));
	}
	LOG_INF("Motor is at Target Position");
	return 0;
}

static int32_t gpio_stepper_motor_set_position(const struct stepper_motor *motor,
					       const struct stepper_motor_position_info *pos_info)
{
	struct gpio_stepper_motor_controller_data *motor_data =
		(struct gpio_stepper_motor_controller_data *)motor->controller->data;

	switch (pos_info->type) {
	case MOTOR_POSITION_MIN: {
		motor_data->min_position = pos_info->position;
		break;
	}
	case MOTOR_POSITION_MAX: {
		motor_data->max_position = pos_info->position;
		break;
	}
	case MOTOR_POSITION_ACTUAL: {
		motor_data->actual_position = pos_info->position;
		break;
	}
	case MOTOR_POSITION_TARGET: {
		motor_data->target_position = pos_info->position;
		stepper_motor_move(motor);
		break;
	}
	}

	return 0;
}

static int32_t gpio_stepper_motor_get_position(const struct stepper_motor *motor,
					       struct stepper_motor_position_info *pos_info)
{
	struct gpio_stepper_motor_controller_data *motor_data =
		(struct gpio_stepper_motor_controller_data *)motor->controller->data;
	switch (pos_info->type) {
	case MOTOR_POSITION_MIN: {
		pos_info->position = motor_data->min_position;
		break;
	}
	case MOTOR_POSITION_MAX: {
		pos_info->position = motor_data->max_position;
		break;
	}
	case MOTOR_POSITION_ACTUAL: {
		pos_info->position = motor_data->actual_position;
		break;
	}
	case MOTOR_POSITION_TARGET: {
		pos_info->position = motor_data->target_position;
		break;
	}
	}
	return 0;
}

static int gpio_stepper_motor_controller_init(const struct device *dev)
{
	const struct gpio_stepper_motor_controller_config *config =
		(const struct gpio_stepper_motor_controller_config *)dev->config;
	LOG_INF("Initializing %s gpio_stepper_motor_controller with pins %d for motor wit steps "
		"per rev:%d, "
		"gear ratio:%f",
		dev->name, config->number_in_pins, config->motor_steps_per_rev,
		config->motor_gear_ratio);

	for (uint8_t n_pin = 0; n_pin < config->number_in_pins; n_pin++) {
		gpio_pin_configure_dt(&config->gpio_in_pins[n_pin], GPIO_OUTPUT_LOW);
	}

	k_sleep(K_SECONDS(5));
	return 0;
}

static const struct stepper_motor_api gpio_stepper_motor_controller_stepper_motor_controller_api = {
	.stepper_motor_set_position = gpio_stepper_motor_set_position,
	.stepper_motor_get_position = gpio_stepper_motor_get_position,
};

#define GPIO_STEPPER_MOTOR_CONTROLLER_DEFINE(inst)                                                 \
	static const struct gpio_dt_spec gpio_stepper_motor_in_pins_##inst[] = {                   \
		DT_INST_FOREACH_PROP_ELEM_SEP(inst, gpios, GPIO_DT_SPEC_GET_BY_IDX, (,)),         \
	};                                                                                         \
	static const struct gpio_stepper_motor_controller_config                                   \
		gpio_stepper_motor_controller_config_##inst = {                                    \
			.motor_steps_per_rev =                                                     \
				DT_PROP(DT_INST_CHILD(inst, motor), steps_per_revolution),         \
			.motor_gear_ratio =                                                        \
				DT_STRING_UNQUOTED(DT_INST_CHILD(inst, motor), gear_ratio),        \
			.gpio_in_pins = gpio_stepper_motor_in_pins_##inst,                         \
			.number_in_pins = ARRAY_SIZE(gpio_stepper_motor_in_pins_##inst),           \
	};                                                                                         \
	static struct gpio_stepper_motor_controller_data                                           \
		gpio_stepper_motor_controller_data_##inst = {.actual_position = 0u};               \
	DEVICE_DT_INST_DEFINE(inst, gpio_stepper_motor_controller_init, NULL,                      \
			      &gpio_stepper_motor_controller_data_##inst,                          \
			      &gpio_stepper_motor_controller_config_##inst, POST_KERNEL,           \
			      CONFIG_APPLICATION_INIT_PRIORITY,                                    \
			      &gpio_stepper_motor_controller_stepper_motor_controller_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_STEPPER_MOTOR_CONTROLLER_DEFINE)

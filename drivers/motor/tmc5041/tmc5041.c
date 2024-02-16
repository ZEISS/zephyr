/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT trinamic_tmc5041

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/stepper_motor/tmc5041_reg.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_TMC5041_SPI
#include "tmc5041_spi.h"
#endif

LOG_MODULE_REGISTER(tmc5041, CONFIG_STEPPER_MOTOR_LOG_LEVEL);

struct tmc5041_stepper_motor_data {
	struct k_mutex calibration_mutex;

	int32_t min_position;
	int32_t max_position;
	stepper_motor_calibrate_func_t stepper_motor_calibrate_func;
	uint16_t micro_step_resolution;
};

struct tmc5041_controller_data {
	/** Mutex to prevent further access  on the SPI Bus when a read is underway. */
	struct k_mutex mutex;
	/** INT pin GPIO callback. */
	struct gpio_callback int_cb;

	struct tmc5041_stepper_motor_data *motors_data;
};

struct tmc5041_stepper_motor_config {
	/** addr refers to the index of the motor */
	const int32_t index;
	const int8_t stall_guard_setting;
	const double gear_ratio;
	const uint8_t steps_per_revolution;
};

struct tmc5041_controller_config {
	/** SPI instance. */
	struct spi_dt_spec spi;
	const struct tmc5041_stepper_motor_config *motors;
	const uint8_t n_motors;
#ifdef CONFIG_TMC5041_INT
	/** INT pin input (optional). */
	struct gpio_dt_spec int_pin;
#endif
};

static inline int tmc5041_write(const struct device *dev, const uint8_t reg_addr,
				const uint32_t reg_val)
{
	const struct tmc5041_controller_config *config = dev->config;
	struct tmc5041_controller_data *data = dev->data;
	const struct spi_dt_spec bus = config->spi;
	int status;

	(void)k_mutex_lock(&data->mutex, K_FOREVER);
	status = tmc_spi_write_register(&bus, reg_addr, reg_val);
	(void)k_mutex_unlock(&data->mutex);
	return status;
}

static inline int tmc5041_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val)
{
	const struct tmc5041_controller_config *config = dev->config;
	struct tmc5041_controller_data *data = dev->data;
	const struct spi_dt_spec bus = config->spi;
	int status;

	(void)k_mutex_lock(&data->mutex, K_FOREVER);
	status = tmc_spi_read_register(&bus, reg_addr, reg_val);
	(void)k_mutex_unlock(&data->mutex);
	return status;
}

static int32_t tmc5041_motor_reset(const struct stepper_motor *motor)
{
	const struct tmc5041_controller_config *tmc5041_config =
		(const struct tmc5041_controller_config *)motor->controller->config;
	const struct tmc5041_stepper_motor_config *config =
		(const struct tmc5041_stepper_motor_config
			 *)(&tmc5041_config->motors[motor->motor_index]);

	int32_t register_address, register_value;

	register_address = TMC5041_GCONF;
	register_value = 0x8;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_XACTUAL(config->index);
	register_value = 0;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_XTARGET(config->index);
	register_value = 0;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_CHOPCONF(config->index);
	register_value = 0x100C5;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_TZEROWAIT(config->index);
	register_value = 100;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_PWMCONF(config->index);
	register_value = 0x401C8;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_VHIGH(config->index);
	register_value = 180000;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_VCOOLTHRS(config->index);
	register_value = 150000;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_A1(config->index);
	register_value = 0xFE80;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_V1(config->index);
	register_value = 0xC350;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_AMAX(config->index);
	register_value = 0x1f000;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_DMAX(config->index);
	register_value = 0x2BCF;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_VMAX(config->index);
	register_value = 270000;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_D1(config->index);
	register_value = 0x578;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_VSTOP(config->index);
	register_value = 0x10;
	tmc5041_write(motor->controller, register_address, register_value);

	register_address = TMC5041_RAMPMODE(config->index);
	register_value = 0x00;
	tmc5041_write(motor->controller, register_address, register_value);

	LOG_INF("Setting stall guard to %d", config->stall_guard_setting);
	register_address = TMC5041_COOLCONF(config->index);
	tmc5041_write(motor->controller, register_address,
		      config->stall_guard_setting << TMC5041_COOLCONF_SG2_THRESHOLD_VALUE_SHIFT);

	register_address = TMC5041_IHOLD_IRUN(config->index);
	register_value = (TMC5041_IRUN(16) | TMC5041_IHOLD(1) | TMC5041_IHOLDDELAY(1));
	tmc5041_write(motor->controller, register_address, register_value);

	/*Set Micro Stepping*/
	tmc5041_write(motor->controller, TMC5041_MSLUT0(config->index), 0xAAAAB554);
	tmc5041_write(motor->controller, TMC5041_MSLUT1(config->index), 0x4A9554AA);
	tmc5041_write(motor->controller, TMC5041_MSLUT2(config->index), 0x24492929);
	tmc5041_write(motor->controller, TMC5041_MSLUT3(config->index), 0x10104222);
	tmc5041_write(motor->controller, TMC5041_MSLUT4(config->index), 0xFBFFFFFF);
	tmc5041_write(motor->controller, TMC5041_MSLUT5(config->index), 0xB5BB777D);
	tmc5041_write(motor->controller, TMC5041_MSLUT6(config->index), 0x49295556);
	tmc5041_write(motor->controller, TMC5041_MSLUT7(config->index), 0x00404222);

	tmc5041_write(motor->controller, TMC5041_MSLUTSEL(config->index), 0xFFFF8056);
	tmc5041_write(motor->controller, TMC5041_MSLUTSTART(config->index), 0x00F70000);

	/* Disable to release motor after stop event */
	tmc5041_write(motor->controller, TMC5041_SWMODE(config->index), 0);

	return 0;
}

static int32_t tmc5041_motor_run(const struct stepper_motor *motor,
				 const struct stepper_motor_run_info *run_info)
{
	LOG_INF("RUnning Motor %s", motor->controller->name);
	const struct tmc5041_controller_config *tmc5041_config =
		(const struct tmc5041_controller_config *)motor->controller->config;
	const struct tmc5041_stepper_motor_config *config =
		(const struct tmc5041_stepper_motor_config
			 *)(&tmc5041_config->motors[motor->motor_index]);
	struct tmc5041_controller_data *tmc5041_data =
		(struct tmc5041_controller_data *)motor->controller->data;
	struct tmc5041_stepper_motor_data *data =
		(struct tmc5041_stepper_motor_data *)(&tmc5041_data
							       ->motors_data[motor->motor_index]);

	if ((0U == data->min_position) && (0U == data->max_position)) {
		LOG_WRN("trying to run %s motor in an uncalibrated state", motor->controller->name);
	}
	int32_t ret = 0;

	(void)k_mutex_lock(&data->calibration_mutex, K_FOREVER);
	LOG_INF("Writing to Controller for motor %d", config->index);
	tmc5041_write(motor->controller, TMC5041_SWMODE(config->index),
		      TMC5041_SW_MODE_SG_STOP_DISABLE);
	switch (run_info->direction) {
	case POSITIVE: {
		tmc5041_write(motor->controller, TMC5041_RAMPMODE(config->index),
			      TMC5041_RAMPMODE_POSITIVE_VELOCITY_MODE);
		break;
	}

	case NEGATIVE: {
		tmc5041_write(motor->controller, TMC5041_RAMPMODE(config->index),
			      TMC5041_RAMPMODE_NEGATIVE_VELOCITY_MODE);
		break;
	}

	default: {
		ret = -EINVAL;
		break;
	}
	}
	(void)k_mutex_unlock(&data->calibration_mutex);

	/**
	 * Do not enable during motor spin-up, wait until the motor velocity exceeds a certain
	 * value, where StallGuard2 delivers a stable result
	 */
	k_sleep(K_MSEC(100));
	tmc5041_write(motor->controller, TMC5041_SWMODE(config->index),
		      TMC5041_SW_MODE_SG_STOP_ENABLE);
	return ret;
}

static int32_t tmc5041_motor_stop(const struct stepper_motor *motor)
{
	const struct tmc5041_controller_config *tmc5041_config =
		(const struct tmc5041_controller_config *)motor->controller->config;
	const struct tmc5041_stepper_motor_config *config =
		(const struct tmc5041_stepper_motor_config
			 *)(&tmc5041_config->motors[motor->motor_index]);

	tmc5041_write(motor->controller, TMC5041_RAMPMODE(config->index),
		      TMC5041_RAMPMODE_HOLD_MODE);
	return 0;
}

static int32_t tmc5041_motor_set_position(const struct stepper_motor *motor,
					  const struct stepper_motor_position_info *position_info)
{
	const struct tmc5041_controller_config *tmc5041_config =
		(const struct tmc5041_controller_config *)motor->controller->config;
	const struct tmc5041_stepper_motor_config *config =
		(const struct tmc5041_stepper_motor_config
			 *)(&tmc5041_config->motors[motor->motor_index]);
	struct tmc5041_controller_data *tmc5041_data =
		(struct tmc5041_controller_data *)motor->controller->data;
	struct tmc5041_stepper_motor_data *data =
		(struct tmc5041_stepper_motor_data *)(&tmc5041_data
							       ->motors_data[motor->motor_index]);

	int32_t ret = 0;

	if ((0U == data->min_position) && (0U == data->max_position)) {
		LOG_WRN("%s motor is running in an uncalibrated state", motor->controller->name);
	}

	(void)k_mutex_lock(&data->calibration_mutex, K_FOREVER);

	switch (position_info->type) {
	case MOTOR_POSITION_MIN: {
		data->min_position = position_info->position;
		break;
	}
	case MOTOR_POSITION_MAX: {
		data->max_position = position_info->position;
		break;
	}
	case MOTOR_POSITION_TARGET: {
		tmc5041_write(motor->controller, TMC5041_SWMODE(config->index),
			      TMC5041_SW_MODE_SG_STOP_DISABLE);
		tmc5041_write(motor->controller, TMC5041_RAMPMODE(config->index),
			      TMC5041_RAMPMODE_POSITIONING_MODE);

		tmc5041_write(motor->controller, TMC5041_XTARGET(config->index),
			      position_info->position);

		k_sleep(K_MSEC(100));
		tmc5041_write(motor->controller, TMC5041_SWMODE(config->index),
			      TMC5041_SW_MODE_SG_STOP_ENABLE);
		break;
	}
	case MOTOR_POSITION_ACTUAL: {
		tmc5041_write(motor->controller, TMC5041_RAMPMODE(config->index),
			      TMC5041_RAMPMODE_HOLD_MODE);
		tmc5041_write(motor->controller, TMC5041_XACTUAL(config->index),
			      position_info->position);
		break;
	}
	default: {
		ret = -EINVAL;
		break;
	}
	}

	(void)k_mutex_unlock(&data->calibration_mutex);

	return ret;
}

static int32_t tmc5041_motor_get_position(const struct stepper_motor *motor,
					  struct stepper_motor_position_info *position_info)
{
	const struct tmc5041_controller_config *tmc5041_config =
		(const struct tmc5041_controller_config *)motor->controller->config;
	const struct tmc5041_stepper_motor_config *config =
		(const struct tmc5041_stepper_motor_config
			 *)(&tmc5041_config->motors[motor->motor_index]);
	struct tmc5041_controller_data *tmc5041_data =
		(struct tmc5041_controller_data *)motor->controller->data;
	struct tmc5041_stepper_motor_data *data =
		(struct tmc5041_stepper_motor_data *)(&tmc5041_data
							       ->motors_data[motor->motor_index]);

	int32_t ret = 0;

	switch (position_info->type) {
	case MOTOR_POSITION_MIN: {
		position_info->position = data->min_position;
		break;
	}
	case MOTOR_POSITION_MAX: {
		position_info->position = data->max_position;
		break;
	}
	case MOTOR_POSITION_ACTUAL: {
		tmc5041_read(motor->controller, TMC5041_XACTUAL(config->index),
			     &position_info->position);
		break;
	}
	case MOTOR_POSITION_TARGET: {
		tmc5041_read(motor->controller, TMC5041_XTARGET(config->index),
			     &position_info->position);
		break;
	}
	default: {
		ret = -EINVAL;
		break;
	}
	}
	return ret;
}

static int32_t tmc5041_motor_get_stall_status(const struct stepper_motor *motor, bool *stall_status)
{
	const struct tmc5041_controller_config *tmc5041_config =
		(const struct tmc5041_controller_config *)motor->controller->config;
	const struct tmc5041_stepper_motor_config *config =
		(const struct tmc5041_stepper_motor_config
			 *)(&tmc5041_config->motors[motor->motor_index]);

	int32_t sg_result = 0;

	tmc5041_read(motor->controller, TMC5041_DRVSTATUS(config->index), &sg_result);

	LOG_DBG("Detecting Load for motor %s %ld %ld", motor->controller->name,
		sg_result & TMC5041_DRV_STATUS_SG_RESULT_MASK,
		(sg_result & TMC5041_DRV_STATUS_SG_STATUS_MASK) >>
			TMC5041_DRV_STATUS_SG_STATUS_SHIFT);

	if (0U == (sg_result & TMC5041_DRV_STATUS_SG_RESULT_MASK)) {
		*stall_status = true;
	} else {
		*stall_status = false;
	}
	return 0;
}

static int32_t tmc5041_motor_calibrate(const struct stepper_motor *motor)
{
	struct tmc5041_controller_data *tmc5041_data =
		(struct tmc5041_controller_data *)motor->controller->data;
	struct tmc5041_stepper_motor_data *data =
		(struct tmc5041_stepper_motor_data *)(&tmc5041_data
							       ->motors_data[motor->motor_index]);

	int32_t ret = 0;

	if (data->stepper_motor_calibrate_func) {
		(void)k_mutex_lock(&data->calibration_mutex, K_FOREVER);
		data->stepper_motor_calibrate_func(motor, &data->min_position, &data->max_position);
		(void)k_mutex_unlock(&data->calibration_mutex);

		LOG_INF("Min Pos:%d Max Pos:%d", data->min_position, data->max_position);
	} else {
		LOG_ERR("%s motor cannot be calibrated without calibration function",
			motor->controller->name);
		ret = -ENOTSUP;
	}
	return ret;
}

static int32_t
tmc5041_motor_register_calibrate_func(const struct stepper_motor *motor,
				      int32_t (*calibrate_func)(const struct stepper_motor *motor,
								int32_t *min_pos, int32_t *max_pos))
{
	struct tmc5041_controller_data *tmc5041_data =
		(struct tmc5041_controller_data *)motor->controller->data;
	struct tmc5041_stepper_motor_data *data =
		(struct tmc5041_stepper_motor_data *)(&tmc5041_data
							       ->motors_data[motor->motor_index]);
	(void)k_mutex_lock(&data->calibration_mutex, K_FOREVER);
	if (calibrate_func) {
		data->stepper_motor_calibrate_func = calibrate_func;
	}
	(void)k_mutex_unlock(&data->calibration_mutex);
	return 0;
}

#ifdef CONFIG_TMC5041_INT
static void tmc5041_int_pin_callback_handler(const struct device *port, struct gpio_callback *cb,
					     gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	/* for now, a message is shown */
	LOG_INF("INT pin interrupt detected");
}
#endif /* CONFIG_TMC5041_INT */

static int tmc5041_init(const struct device *dev)
{
	const struct tmc5041_controller_config *config = dev->config;
	struct tmc5041_controller_data *data = dev->data;
	int status = 0;

	for (int i = 0; i < config->n_motors; i++) {
	}

	(void)k_mutex_init(&data->mutex);

	/* configure SPI */
	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

#ifdef CONFIG_TMC5041_INT
	/* configure int GPIO */
	if (config->int_pin.port != NULL) {
		if (!device_is_ready(config->int_pin.port)) {
			LOG_ERR("INT GPIO controller not ready");
			return -ENODEV;
		}

		status = gpio_pin_configure_dt(&config->int_pin, GPIO_INPUT);
		if (status < 0) {
			LOG_ERR("Could not configure INT GPIO (%d)", status);
			return status;
		}

		gpio_init_callback(&data->int_cb, tmc5041_int_pin_callback_handler,
				   BIT(config->int_pin.pin));

		status = gpio_add_callback(config->int_pin.port, &data->int_cb);
		if (status < 0) {
			LOG_ERR("Could not add INT pin GPIO callback (%d)", status);
			return status;
		}

		status = gpio_pin_interrupt_configure_dt(&config->int_pin, GPIO_INT_EDGE_RISING);
		if (status < 0) {
			LOG_ERR("failed to configure INT interrupt (err %d)", status);
			return -EINVAL;
		}
	}
#endif /* CONFIG_TMC5041_INT */

	/* Read GSTAT register values to clear any errors SPI Datagram. */
	uint32_t gstat_value;
	(void)tmc5041_read(dev, TMC5041_GSTAT, &gstat_value);
	LOG_INF("Device %s initialized", dev->name);
	return status;
}

static const struct stepper_motor_api tmc5041_stepper_motor_controller_api = {
	.stepper_motor_reset = tmc5041_motor_reset,
	.stepper_motor_run = tmc5041_motor_run,
	.stepper_motor_stop = tmc5041_motor_stop,
	.stepper_motor_get_stall_status = tmc5041_motor_get_stall_status,
	.stepper_motor_get_position = tmc5041_motor_get_position,
	.stepper_motor_set_position = tmc5041_motor_set_position,
	.stepper_motor_calibrate = tmc5041_motor_calibrate,
	.stepper_motor_register_calibrate_func = tmc5041_motor_register_calibrate_func,
	.stepper_motor_controller_write_reg = tmc5041_write,
	.stepper_motor_controller_read_reg = tmc5041_read,
};

#define tmc5041_stepper_motor_config_INIT(child)                                                   \
	{.index = DT_PROP(child, reg),                                                             \
	 .stall_guard_setting = DT_PROP(child, stall_guard_setting),                               \
	 .gear_ratio = DT_STRING_UNQUOTED(child, gear_ratio),                                      \
	 .steps_per_revolution = DT_PROP(child, steps_per_revolution)},

#define STEPPER_MOTOR_DATA_INIT(child)                                                             \
	{.micro_step_resolution = DT_PROP_OR(child, micro_step_res, 256)},

#define TMC5041_DEFINE(inst)                                                                       \
	static const struct tmc5041_stepper_motor_config tmc5041_stepper_motor_config_##inst[] = { \
		DT_INST_FOREACH_CHILD(inst, tmc5041_stepper_motor_config_INIT)};                   \
	static struct tmc5041_stepper_motor_data tmc5041_stepper_motor_data_##inst[] = {           \
		DT_INST_FOREACH_CHILD(inst, STEPPER_MOTOR_DATA_INIT)};                             \
	static struct tmc5041_controller_data tmc5041_controller_data_##inst = {                   \
		.motors_data = tmc5041_stepper_motor_data_##inst};                                 \
	static const struct tmc5041_controller_config tmc5041_config_##inst = {                    \
		.spi = SPI_DT_SPEC_INST_GET(inst,                                                  \
					    SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |                \
						    SPI_MODE_CPOL | SPI_MODE_CPHA |                \
						    SPI_WORD_SET(8),                               \
					    0),                                                    \
		.motors = tmc5041_stepper_motor_config_##inst,                                     \
		.n_motors = ARRAY_SIZE(tmc5041_stepper_motor_config_##inst),                       \
		IF_ENABLED(CONFIG_TMC5041_INT,                                                     \
			   (.int_pin = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {})))};           \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, tmc5041_init, NULL, &tmc5041_controller_data_##inst,           \
			      &tmc5041_config_##inst, POST_KERNEL,                                 \
			      CONFIG_APPLICATION_INIT_PRIORITY,                                    \
			      &tmc5041_stepper_motor_controller_api);

DT_INST_FOREACH_STATUS_OKAY(TMC5041_DEFINE)

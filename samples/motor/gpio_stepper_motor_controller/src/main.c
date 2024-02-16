/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/drivers/motor.h"

const struct device *motor_x = DEVICE_DT_GET(DT_NODELABEL(motor_x));
const struct device *motor_y = DEVICE_DT_GET(DT_NODELABEL(motor_y));
const struct device *motor_z = DEVICE_DT_GET(DT_NODELABEL(motor_z));

static void motor_x_thread(void)
{
	for (;;) {

		motor_set_angle(motor_x, 225);
		k_sleep(K_SECONDS(1));
		motor_rotate(motor_x, -450);
		k_sleep(K_SECONDS(1));
	}
}

static void motor_y_thread(void)
{
	for (;;) {
		motor_set_angle(motor_y, 225);
		k_sleep(K_SECONDS(1));
		motor_rotate(motor_y, -450);
		k_sleep(K_SECONDS(1));
	}
}

static void motor_z_thread(void)
{
	for (;;) {
		motor_set_angle(motor_z, 225);
		k_sleep(K_SECONDS(1));
		motor_rotate(motor_z, -450);
		k_sleep(K_SECONDS(1));
	}
}

K_THREAD_DEFINE(motor_x_thread_id, 1024 * 2, motor_x_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(motor_y_thread_id, 1024 * 2, motor_y_thread, NULL, NULL, NULL, 5, 0, 0);

K_THREAD_DEFINE(motor_z_thread_id, 1024 * 2, motor_z_thread, NULL, NULL, NULL, 5, 0, 0);

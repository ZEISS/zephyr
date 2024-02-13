/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/drivers/stepper_motor.h"

const struct stepper_motor motor = STEPPER_MOTOR_DT_GET(motor);

int main(void)
{
	for (;;) {
		struct stepper_motor_position_info position_info = {
			.type = MOTOR_POSITION_TARGET,
			.position = 2480,
		};
		stepper_motor_set_position(&motor, &position_info);

		position_info.position = -2480;

		stepper_motor_set_position(&motor, &position_info);
	}

	return 0;
}

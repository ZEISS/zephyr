# SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_GPIO_STEPPER_MOTOR_CONTROLLER gpio_stepper_motor_controller.c)

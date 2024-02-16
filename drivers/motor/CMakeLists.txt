# SPDX-FileCopyrightText: Copyright (c) 2023 Carl Zeiss Meditec AG
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_GPIO_STEPPER_MOTOR_CONTROLLER gpio_stepper_motor_controller)
add_subdirectory_ifdef(CONFIG_TMC5041 tmc5041)

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/motor.h)

zephyr_library()
zephyr_library_property(ALLOW_EMPTY TRUE)

zephyr_library_sources_ifdef(CONFIG_STEPPER_MOTOR_SHELL motor_shell.c)

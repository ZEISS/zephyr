# Copyright (c) 2023 Carl Zeiss Meditec AG
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_TMC_SPI                     tmc_spi.c)
zephyr_library_sources_ifdef(CONFIG_TRINAMIC_SHELL              tmc_shell.c)
zephyr_library_sources_ifdef(CONFIG_TRINAMIC_TMC5041                  tmc5041.c)
zephyr_library_sources_ifdef(CONFIG_TRINAMIC_TMC5041_SHELL      tmc5041_shell.c)


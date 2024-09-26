/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_TRINAMIC_H_
#define ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_TRINAMIC_H_

#include <stdint.h>
#include <zephyr/drivers/stepper.h>

#ifdef __cplusplus
extern "C" {
#endif

struct tmc_ramp_generator_data {
	uint32_t vstart;
	uint32_t a1;
	uint32_t v1;
	uint32_t d1;
	uint32_t amax;
	uint32_t vmax;
	uint32_t dmax;
	uint32_t vstop;
	uint32_t tzerowait;
	uint32_t vcoolthrs;
	uint32_t vhigh;
	uint32_t iholdrun;
};

/**
 * @brief Get Trinamic Stepper Ramp Generator data from DT
 *
 * @param node DT node identifier
 *
 * @return struct tmc_ramp_generator_data
 */
#define TMC_RAMP_DT_SPEC_GET(node)							\
	{										\
		.vstart = DT_PROP_OR(node, vstart, 0),					\
		.a1 = DT_PROP_OR(node, a1, 0),						\
		.v1 = DT_PROP_OR(node, v1, 0),						\
		.d1 = DT_PROP_OR(node, d1, 0),						\
		.amax = DT_PROP_OR(node, amax, 0),					\
		.vmax = DT_PROP_OR(node, vmax, 0),					\
		.dmax = DT_PROP_OR(node, dmax, 0),					\
		.vstop = DT_PROP_OR(node, vstop, 0),					\
		.tzerowait = DT_PROP_OR(node, tzerowait, 0),				\
		.vcoolthrs = DT_PROP_OR(node, vcoolthrs, 0),				\
		.vhigh = DT_PROP_OR(node, vhigh, 0),					\
		.iholdrun = (TMC5041_IRUN(DT_PROP_OR(node, irun, 0)) |			\
			     TMC5041_IHOLD(DT_PROP_OR(node, ihold, 0)) |		\
			     TMC5041_IHOLDDELAY(DT_PROP_OR(node, iholddelay, 0))),	\
	}

/**
 * @brief Configure Trinamic Stepper Ramp Generator
 *
 * @param dev Pointer to the stepper motor controller instance
 * @param ramp_data Pointer to a struct containing the required ramp parameters
 *
 * @retval -EIO General input / output error
 * @retval -ENOSYS If not implemented by device driver
 * @retval 0 Success
 */
int tmc5041_stepper_set_ramp(const struct device *dev,
			     const struct tmc_ramp_generator_data *ramp_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_TRINAMIC_H_ */

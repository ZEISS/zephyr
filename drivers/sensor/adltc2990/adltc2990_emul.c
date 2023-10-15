/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adltc2990

#include <zephyr/device.h>
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/emul_sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "adltc2990.h"
#include "adltc2990_reg.h"
#include "adltc2990_emul.h"

LOG_MODULE_DECLARE(adltc2990, CONFIG_SENSOR_LOG_LEVEL);

#define ADLTC2990_NUM_REGS ADLTC2990_REG_VCC_LSB

struct adltc2990_emul_data {
	uint8_t reg[ADLTC2990_NUM_REGS];
};

struct adltc2990_emul_cfg {
};

void adltc2990_emul_set_reg(const struct emul *target, uint8_t reg_addr, const uint8_t *val)
{
	struct adltc2990_emul_data *data = target->data;

	__ASSERT_NO_MSG(reg_addr <= ADLTC2990_NUM_REGS);
	memcpy(data->reg + reg_addr, val, 1);
}

void adltc2990_emul_get_reg(const struct emul *target, uint8_t reg_addr, uint8_t *val)
{
	struct adltc2990_emul_data *data = target->data;

	__ASSERT_NO_MSG(reg_addr <= ADLTC2990_NUM_REGS);
	memcpy(val, data->reg + reg_addr, 1);
}

void adltc2990_emul_reset(const struct emul *target)
{
	struct adltc2990_emul_data *data = target->data;

	memset(data->reg, 0, ADLTC2990_NUM_REGS);
}

static int adltc2990_emul_handle_write(const struct emul *target, uint8_t regn, uint8_t value)
{
	struct adltc2990_emul_data *data = target->data;

	switch (regn) {
	case ADLTC2990_REG_CONTROL:
		data->reg[ADLTC2990_REG_CONTROL] = value;
		break;

	case ADLTC2990_REG_TRIGGER:
		data->reg[ADLTC2990_REG_TRIGGER] = value;
		break;

	default:
		break;
	}
	return 0;
}

static int adltc2990_emul_transfer_i2c(const struct emul *target, struct i2c_msg *msgs,
				       int num_msgs, int addr)
{
	struct adltc2990_emul_data *data = target->data;

	i2c_dump_msgs_rw(target->dev, msgs, num_msgs, addr, false);

	if (num_msgs < 1) {
		printk("Invalid number of messages: %d", num_msgs);
		return -EIO;
	}
	if (FIELD_GET(I2C_MSG_READ, msgs->flags)) {
		printk("Unexpected read");
		return -EIO;
	}
	if (msgs->len < 1) {
		printk("Unexpected msg0 length %s", msgs->len);
		return -EIO;
	}

	uint8_t regn = msgs->buf[0];
	bool is_read = FIELD_GET(I2C_MSG_READ, msgs->flags) == 1;
	bool is_stop = FIELD_GET(I2C_MSG_STOP, msgs->flags) == 1;

	if (!is_stop && !is_read) {
		/* First message was a write with the register number, check next message */
		msgs++;
		is_read = FIELD_GET(I2C_MSG_READ, msgs->flags) == 1;
		is_stop = FIELD_GET(I2C_MSG_STOP, msgs->flags) == 1;
	}

	if (is_read) {
		for (int i = 0; i < msgs->len; ++i) {
			msgs->buf[i] = data->reg[regn + i];
		}
	} else {
		int rc = adltc2990_emul_handle_write(target, regn, msgs->buf[1]);

		if (rc != 0) {
			return rc;
		}
	}
	return 0;
};

static int adltc2990_emul_init(const struct emul *target, const struct device *parent)
{
	ARG_UNUSED(parent);
	adltc2990_emul_reset(target);

	return 0;
}

static const struct i2c_emul_api adltc2990_emul_api_i2c = {
	.transfer = adltc2990_emul_transfer_i2c,
};

static int16_t clamped_voltage_single_ended_value(q31_t value)
{
	int16_t reg_value =
		(value * 100 / (int32_t)ADLTC2990_VOLTAGE_SINGLEENDED_CONVERSION_FACTOR);
	if (reg_value < ((ADLTC2990_VOLTAGE_SINGLE_ENDED_MIN_DATA_REG_VALUE) << 17) >> 17) {
		reg_value = ADLTC2990_VOLTAGE_SINGLE_ENDED_MIN_DATA_REG_VALUE;
	} else if (reg_value > ((ADLTC2990_VOLTAGE_SINGLE_ENDED_MAX_DATA_REG_VALUE) << 17) >> 17) {
		reg_value = ADLTC2990_VOLTAGE_SINGLE_ENDED_MAX_DATA_REG_VALUE;
	}
	return reg_value;
}

static int16_t clamped_voltage_differential_value(q31_t value)
{
	int16_t reg_value =
		(value * 100 / (int32_t)ADLTC2990_VOLTAGE_DIFFERENTIAL_CONVERSION_FACTOR);
	if (reg_value < ((ADLTC2990_VOLTAGE_SINGLE_ENDED_MIN_DATA_REG_VALUE) << 17) >> 17) {
		reg_value = ADLTC2990_VOLTAGE_SINGLE_ENDED_MIN_DATA_REG_VALUE;
	} else if (reg_value > ((ADLTC2990_VOLTAGE_SINGLE_ENDED_MAX_DATA_REG_VALUE) << 17) >> 17) {
		reg_value = ADLTC2990_VOLTAGE_SINGLE_ENDED_MAX_DATA_REG_VALUE;
	}
	return reg_value;
}

static int16_t clamped_current_value(q31_t current_value, uint32_t resistor)
{
	double voltage_value = (current_value / (double)1000000) * resistor;

	voltage_value = (voltage_value * 100) / 1942;
	int16_t reg_value = voltage_value;

	LOG_INF("reg value is 0x%x", reg_value);

	return reg_value;
}

static int16_t clamped_reg_temperature_value(const struct emul *target, q31_t value)
{
	const struct adltc2990_config *dev_config = target->dev->config;
	int16_t reg_value = (value / (int32_t)ADLTC2990_MICRO_CELCIUS_PER_BIT);

	switch (dev_config->temp_format) {
	case ADLTC2990_TEMPERATURE_FORMAT_CELSIUS:
		if (reg_value < (ADLTC2990_CELCIUS_MIN_DATA_REG_VALUE << 19) >> 19) {
			reg_value = ADLTC2990_CELCIUS_MIN_DATA_REG_VALUE;
			LOG_DBG("clamping reg value to lower end: %d", reg_value);
		} else if (reg_value > (ADLTC2990_CELCIUS_MAX_DATA_REG_VALUE << 19) >> 19) {
			reg_value = ADLTC2990_CELCIUS_MAX_DATA_REG_VALUE;
			LOG_DBG("clamping reg value to upper end: %d", reg_value);
		}
		break;
	case ADLTC2990_TEMPERATURE_FORMAT_KELVIN:
		if (reg_value < ADLTC2990_KELVIN_MIN_DATA_REG_VALUE) {
			reg_value = ADLTC2990_KELVIN_MIN_DATA_REG_VALUE;
		} else if (reg_value > ADLTC2990_KELVIN_MAX_DATA_REG_VALUE) {
			reg_value = ADLTC2990_KELVIN_MAX_DATA_REG_VALUE;
		}
		break;
	default:
		break;
	}
	return reg_value;
}

static int set_channel(const struct emul *target, enum sensor_channel ch, q31_t value, int8_t shift)
{
	const struct adltc2990_config *dev_config = target->dev->config;
	struct adltc2990_emul_data *data = target->data;

	uint8_t msb_reg[5] = {0}, lsb_reg[5] = {0}, msb_value[5] = {0}, lsb_value[5] = {0};
	int16_t reg_value[5] = {0};
	uint8_t total_reg_count = 0;

	switch (ch) {
	case SENSOR_CHAN_DIE_TEMP: {
		reg_value[total_reg_count] = clamped_reg_temperature_value(target, value);

		msb_reg[total_reg_count] = ADLTC2990_REG_INTERNAL_TEMP_MSB;
		lsb_reg[total_reg_count] = ADLTC2990_REG_INTERNAL_TEMP_LSB;

		break;
	}
	case SENSOR_CHAN_AMBIENT_TEMP: {
		if (adltc2990_get_v1_v2_measurement_modes(dev_config->measurement_mode[1],
							  dev_config->measurement_mode[0]) ==
		    TEMPERATURE) {
			reg_value[total_reg_count] = clamped_reg_temperature_value(target, value);

			msb_reg[total_reg_count] = ADLTC2990_REG_V1_MSB;
			lsb_reg[total_reg_count] = ADLTC2990_REG_V1_LSB;
			total_reg_count += 1;
		}
		if (adltc2990_get_v3_v4_measurement_modes(dev_config->measurement_mode[1],
							  dev_config->measurement_mode[0]) ==
		    TEMPERATURE) {
			reg_value[total_reg_count] = clamped_reg_temperature_value(target, value);

			msb_reg[total_reg_count] = ADLTC2990_REG_V3_MSB;
			lsb_reg[total_reg_count] = ADLTC2990_REG_V3_LSB;
		}
		break;
	}
	case SENSOR_CHAN_CURRENT: {
		if (adltc2990_get_v1_v2_measurement_modes(dev_config->measurement_mode[1],
							  dev_config->measurement_mode[0]) ==
		    VOLTAGE_DIFFERENTIAL) {
			reg_value[total_reg_count] = clamped_current_value(
				value, dev_config->pins_v1_v2.pins_current_resistor);
			msb_reg[total_reg_count] = ADLTC2990_REG_V1_MSB;
			lsb_reg[total_reg_count] = ADLTC2990_REG_V1_LSB;
		}
		break;
	}
	case SENSOR_CHAN_VOLTAGE: {
		if (adltc2990_get_v1_v2_measurement_modes(dev_config->measurement_mode[1],
							  dev_config->measurement_mode[0]) ==
		    VOLTAGE_DIFFERENTIAL) {
			reg_value[total_reg_count] = clamped_voltage_differential_value(value);
			msb_reg[total_reg_count] = ADLTC2990_REG_V1_MSB;
			lsb_reg[total_reg_count] = ADLTC2990_REG_V1_LSB;
			total_reg_count += 1;
		} else if (adltc2990_get_v1_v2_measurement_modes(dev_config->measurement_mode[1],
								 dev_config->measurement_mode[0]) ==
			   VOLTAGE_SINGLEENDED) {
			reg_value[total_reg_count] = clamped_voltage_single_ended_value(value);
			msb_reg[total_reg_count] = ADLTC2990_REG_V1_MSB;
			lsb_reg[total_reg_count] = ADLTC2990_REG_V1_LSB;
			total_reg_count += 1;

			reg_value[total_reg_count] = clamped_voltage_single_ended_value(value);
			msb_reg[total_reg_count] = ADLTC2990_REG_V2_MSB;
			lsb_reg[total_reg_count] = ADLTC2990_REG_V2_LSB;
			total_reg_count += 1;
		}

		if (adltc2990_get_v3_v4_measurement_modes(dev_config->measurement_mode[1],
							  dev_config->measurement_mode[0]) ==
		    VOLTAGE_DIFFERENTIAL) {
			reg_value[total_reg_count] = clamped_voltage_differential_value(value);
			msb_reg[total_reg_count] = ADLTC2990_REG_V3_MSB;
			lsb_reg[total_reg_count] = ADLTC2990_REG_V3_LSB;
			total_reg_count += 1;
		} else if (adltc2990_get_v3_v4_measurement_modes(dev_config->measurement_mode[1],
								 dev_config->measurement_mode[0]) ==
			   VOLTAGE_SINGLEENDED) {
			reg_value[total_reg_count] = clamped_voltage_single_ended_value(value);
			msb_reg[total_reg_count] = ADLTC2990_REG_V3_MSB;
			lsb_reg[total_reg_count] = ADLTC2990_REG_V3_LSB;
			total_reg_count += 1;

			reg_value[total_reg_count] = clamped_voltage_single_ended_value(value);
			msb_reg[total_reg_count] = ADLTC2990_REG_V4_MSB;
			lsb_reg[total_reg_count] = ADLTC2990_REG_V4_LSB;
			total_reg_count += 1;
		}

		msb_reg[total_reg_count] = ADLTC2990_REG_VCC_MSB;
		lsb_reg[total_reg_count] = ADLTC2990_REG_VCC_LSB;
		break;
	}

	default: {
		break;
	}
	}

	for (int i = 0; i <= total_reg_count; i++) {
		msb_value[i] = reg_value[i] >> 8;
		lsb_value[i] = reg_value[i] & 0x00ff;
		adltc2990_emul_set_reg(target, msb_reg[i], &msb_value[i]);
		adltc2990_emul_set_reg(target, lsb_reg[i], &lsb_value[i]);
	}
	return 0;
}

static int get_sample_range(const struct emul *target, enum sensor_channel ch, q31_t *lower,
			    q31_t *upper, q31_t *epsilon, int8_t *shift)
{
	const struct adltc2990_config *dev_config = target->dev->config;

	switch (ch) {
	case SENSOR_CHAN_DIE_TEMP:
	case SENSOR_CHAN_AMBIENT_TEMP: {
		if (dev_config->temp_format == ADLTC2990_TEMPERATURE_FORMAT_KELVIN) {
			*lower = ADLTC2990_KELVIN_MIN_DATA_REG_VALUE *
				 ADLTC2990_TEMPERATURE_CONVERSION_FACTOR;
			*upper = ADLTC2990_KELVIN_MAX_DATA_REG_VALUE *
				 ADLTC2990_TEMPERATURE_CONVERSION_FACTOR;
		} else {
			*lower = ((ADLTC2990_CELCIUS_MIN_DATA_REG_VALUE << 19) >> 19) *
				 ADLTC2990_TEMPERATURE_CONVERSION_FACTOR;
			*upper = ADLTC2990_CELCIUS_MAX_DATA_REG_VALUE *
				 ADLTC2990_TEMPERATURE_CONVERSION_FACTOR;
		}
		*epsilon = ADLTC2990_TEMPERATURE_CONVERSION_FACTOR;
		break;
	}
	case SENSOR_CHAN_CURRENT: {
		break;
	}
	case SENSOR_CHAN_VOLTAGE: {
		if (adltc2990_get_v1_v2_measurement_modes(dev_config->measurement_mode[1],
							  dev_config->measurement_mode[0]) ==
		    VOLTAGE_DIFFERENTIAL) {
			*lower = ((ADLTC2990_VOLTAGE_SINGLE_ENDED_MIN_DATA_REG_VALUE << 17) >> 17) *
				 ADLTC2990_VOLTAGE_DIFFERENTIAL_CONVERSION_FACTOR;
			*lower /= (int32_t)100;
			*upper = ADLTC2990_VOLTAGE_SINGLE_ENDED_MAX_DATA_REG_VALUE *
				 ADLTC2990_VOLTAGE_DIFFERENTIAL_CONVERSION_FACTOR / 100;
			*epsilon = (ADLTC2990_VOLTAGE_DIFFERENTIAL_CONVERSION_FACTOR) / 100;
		} else {
			*lower = ((ADLTC2990_VOLTAGE_SINGLE_ENDED_MIN_DATA_REG_VALUE << 17) >> 17) *
				 ADLTC2990_VOLTAGE_SINGLEENDED_CONVERSION_FACTOR;
			*lower /= (int32_t)100;
			*upper = ADLTC2990_VOLTAGE_SINGLE_ENDED_MAX_DATA_REG_VALUE *
				 ADLTC2990_VOLTAGE_SINGLEENDED_CONVERSION_FACTOR / 100;
			*epsilon = (ADLTC2990_VOLTAGE_SINGLEENDED_CONVERSION_FACTOR) / 100;
		}

		break;
	}
	default: {
		return -ENOTSUP;
	}
	}
	return 0;
}

static const struct emul_sensor_backend_api backend_api = {
	.set_channel = set_channel,
	.get_sample_range = get_sample_range,
};

#define ADLTC2990_EMUL(n)                                                                          \
	const struct adltc2990_emul_cfg adltc2990_emul_cfg_##n;                                    \
	struct adltc2990_emul_data adltc2990_emul_data_##n;                                        \
	EMUL_DT_INST_DEFINE(n, adltc2990_emul_init, &adltc2990_emul_data_##n,                      \
			    &adltc2990_emul_cfg_##n, &adltc2990_emul_api_i2c, &backend_api)

DT_INST_FOREACH_STATUS_OKAY(ADLTC2990_EMUL)

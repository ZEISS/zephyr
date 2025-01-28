/*
 * Copyright 2023 Google LLC
 * SPDX-FileCopyrightText: Copyright (c) 2025 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_tcs3430

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/tcs3430.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tcs3430, CONFIG_SENSOR_LOG_LEVEL);

#define TCS3430_ENABLE_REG  0x80   /* general on/off settings */
#define TCS3430_ENABLE_WEN  BIT(3) /* enables or disables the wait feature */
#define TCS3430_ENABLE_AEN  BIT(1) /* activates the ALS function */
#define TCS3430_ENABLE_PON  BIT(0) /* activates the internal oscillator to allow operation */

#define TCS3430_ATIME_REG 0x81 /* controls integration time of ALS ADCs */
#define TCS3430_WTIME_REG 0x83 /* controls wait timer */
#define TCS3430_AILTL_REG 0x84 /* low byte of low interrupt sensor threshold */
#define TCS3430_AILTH_REG 0x85 /* high byte of low interrupt sensor threshold */
#define TCS3430_AIHTL_REG 0x86 /* low byte of high interrupt sensor threshold */
#define TCS3430_AIHTH_REG 0x87 /* high byte of high interrupt sensor threshold */

#define TCS3430_PERS_REG 0x8C /* interrupt persistence filter */

#define TCS3430_CFG0_REG   0x8D   /* long wait control */
#define TCS3430_CFG0_WLONG BIT(2) /* increases wait cycle by 12x if asserted */

#define TCS3430_CFG1_REG   0x90   /* gain & channel 3 control */
#define TCS3430_CFG1_AMUX  BIT(3) /* configure X (0) or IR2 (1) as third channel */

#define TCS3430_REVID_REG   0x91 /* revision number identification */

#define TCS3430_ID_REG 0x92
#define TCS3430_ID     0xDC /* part number TCS3430 */

#define TCS3430_STATUS_REG  0x93   /* ALS status register */
#define TCS3430_STATUS_ASAT BIT(7) /* ALS saturation */
#define TCS3430_STATUS_AINT BIT(4) /* ALS interrupt */

#define TCS3430_CH0DATAL_REG 0x94 /* low byte of CH0 ADC data (Z) */
#define TCS3430_CH0DATAH_REG 0x95 /* high byte of CH0 ADC data (Z) */
#define TCS3430_CH1DATAL_REG 0x96 /* low byte of CH1 ADC data (Y) */
#define TCS3430_CH1DATAH_REG 0x97 /* high byte of CH1 ADC data (Y) */
#define TCS3430_CH2DATAL_REG 0x98 /* low byte of CH2 ADC data (IR1) */
#define TCS3430_CH2DATAH_REG 0x99 /* high byte of CH2 ADC data (IR1) */
#define TCS3430_CH3DATAL_REG 0x9A /* low byte of CH3 ADC data (X/IR2 depending on CFG1_AMUX) */
#define TCS3430_CH3DATAH_REG 0x9B /* high byte of CH3 ADC data (X/IR2 depending on CFG1_AMUX) */

#define TCS3430_CFG2_REG   0x9F   /* high gain control */
#define TCS3430_CFG2_HGAIN BIT(4) /* high gain mode: 128x gain if CFG1_AGAIN=0x11 */

#define TCS3430_CFG3_REG            0xAB   /* interrupt-related config */
#define TCS3430_CFG3_INT_READ_CLEAR BIT(7) /* clear status register on read */
#define TCS3430_CFG3_SAI            BIT(4) /* sleep after interrupt */

#define TCS3430_AZ_CONFIG_REG     0xD6   /* auto-zero configuration */
#define TCS3430_AZ_CONFIG_AZ_MODE BIT(7) /* auto-zero mode */

#define TCS3430_INTENAB_REG     0xDD   /* interrupt enable */
#define TCS3430_INTENAB_ASIEN   BIT(5) /* enable ASAT (sensor saturation) interrupt */
#define TCS3430_INTENAB_AIEN    BIT(4) /* enable ALS interrupt */

/* Default values */
#define TCS3430_DEFAULT_ENABLE    (TCS3430_ENABLE_AEN | TCS3430_ENABLE_PON)
#define TCS3430_DEFAULT_ATIME     0xFF /* maximum integration time */
#define TCS3430_DEFAULT_PERS      0x00 /* interrupt persistence filter off */
#define TCS3430_DEFAULT_CFG0      0x80 /* must be set to 0x80 according to data sheet */
#define TCS3430_DEFAULT_CFG1      0x00 /* CH3 to X, gain to 1x */
#define TCS3430_DEFAULT_CFG2      0x04 /* high gain mode off */
#define TCS3430_DEFAULT_CFG3      0x0C /* sleep after interrupt OFF */
#define TCS3430_DEFAULT_AZ_CONFIG 0x7F /* auto-zero at first ALS cycle */
#define TCS3430_DEFAULT_INTENAB   0x00 /* interrupts off */

struct tcs3430_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec int_gpio;
	uint16_t integration_cycles;
	uint8_t gain;
};

struct tcs3430_data {
	struct gpio_callback gpio_cb;
	const struct device *dev;

	uint16_t sample_zycx[4];

	struct k_sem data_sem;
};

static int tcs3430_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct tcs3430_config *cfg = dev->config;
	struct tcs3430_data *data = dev->data;
	int ret;
	uint8_t status;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_VISION_X &&
	    chan != SENSOR_CHAN_VISION_Y && chan != SENSOR_CHAN_VISION_Z &&
	    chan != SENSOR_CHAN_IR) {
		LOG_ERR("Unsupported sensor channel");
		return -ENOTSUP;
	}

	ret = i2c_reg_read_byte_dt(&cfg->i2c, TCS3430_STATUS_REG, &status);
	if (ret) {
		return ret;
	}

	if (status & TCS3430_STATUS_ASAT) {
		LOG_ERR("ALS saturated!");
		ret = i2c_reg_write_byte_dt(&cfg->i2c, TCS3430_STATUS_REG, TCS3430_STATUS_ASAT);
		if (ret != 0) {
			LOG_ERR("Couldn't clear ASAT: %d", ret);
		}
	}

	ret = i2c_burst_read_dt(&cfg->i2c, TCS3430_CH0DATAL_REG,
				(uint8_t *)&data->sample_zycx, sizeof(data->sample_zycx));
	if (ret) {
		return ret;
	}

	return 0;
}

static int tcs3430_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct tcs3430_data *data = dev->data;
	switch (chan) {
	case SENSOR_CHAN_VISION_Y: {
		val->val1 = sys_le16_to_cpu(data->sample_zycx[1]);
		val->val2 = 0;
		break;
	}
	case SENSOR_CHAN_VISION_X: {
		val->val1 = sys_le16_to_cpu(data->sample_zycx[3]);
		val->val2 = 0;
		break;
	}
	case SENSOR_CHAN_VISION_Z: {
		val->val1 = sys_le16_to_cpu(data->sample_zycx[0]);
		val->val2 = 0;
		break;
	}
	case SENSOR_CHAN_IR: {
		val->val1 = sys_le16_to_cpu(data->sample_zycx[2]);
		val->val2 = 0;
		break;
	}
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int tcs3430_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct tcs3430_config *cfg = dev->config;
	int ret;
	uint8_t reg_val;

	switch ((enum sensor_attribute_tcs3430)attr) {
	case SENSOR_ATTR_TCS3430_INTEGRATION_CYCLES:
		if (!IN_RANGE(val->val1, 1, 256)) {
			return -EINVAL;
		}
		reg_val = (uint8_t)(val->val1-1);
		ret = i2c_reg_write_byte_dt(&cfg->i2c, TCS3430_ATIME_REG, reg_val);
		if (ret) {
			return ret;
		}
		break;
	case SENSOR_ATTR_TCS3430_GAIN:
		if (!IN_RANGE(val->val1, 0, 3)) {
			return -EINVAL;
		}
		reg_val = (uint8_t)val->val1;
		ret = i2c_reg_write_byte_dt(&cfg->i2c, TCS3430_CFG1_REG, reg_val);
		if (ret) {
			return ret;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int tcs3430_sensor_setup(const struct device *dev)
{
	const struct tcs3430_config *cfg = dev->config;
	uint8_t chip_id;
	int ret;
	if (!IN_RANGE(cfg->integration_cycles, 1, 256)) {
		LOG_ERR("Invalid integration cycles: %d", cfg->integration_cycles);
		return -EINVAL;
	}
	uint8_t atime_default = (uint8_t)(cfg->integration_cycles-1);
	struct {
		uint8_t reg_addr;
		uint8_t value;
	} reset_regs[] = {
		{TCS3430_ATIME_REG, atime_default},
		{TCS3430_PERS_REG, TCS3430_DEFAULT_PERS},
		{TCS3430_CFG0_REG, TCS3430_DEFAULT_CFG0},
		{TCS3430_INTENAB_REG, TCS3430_DEFAULT_INTENAB},
		{TCS3430_CFG1_REG, cfg->gain},
		{TCS3430_CFG2_REG, TCS3430_DEFAULT_CFG2},
		{TCS3430_CFG3_REG, TCS3430_DEFAULT_CFG3},
		{TCS3430_AZ_CONFIG_REG, TCS3430_DEFAULT_AZ_CONFIG},
		{TCS3430_ENABLE_REG, TCS3430_DEFAULT_ENABLE},
	};

	ret = i2c_reg_read_byte_dt(&cfg->i2c, TCS3430_ID_REG, &chip_id);
	if (ret) {
		LOG_ERR("Failed to read chip id: %d", ret);
		return ret;
	}

	if (chip_id != TCS3430_ID) {
		LOG_ERR("Invalid chip id: %02x", chip_id);
		return -EIO;
	}

	LOG_INF("chip id: 0x%x", chip_id);

	for (size_t i = 0; i < ARRAY_SIZE(reset_regs); i++) {
		ret = i2c_reg_write_byte_dt(&cfg->i2c, reset_regs[i].reg_addr, reset_regs[i].value);
		if (ret) {
			LOG_ERR("Failed to set default register: %02x", reset_regs[i].reg_addr);
			return ret;
		}
	}

	return 0;
}

static const struct sensor_driver_api tcs3430_api = {
	.sample_fetch = tcs3430_sample_fetch,
	.channel_get = tcs3430_channel_get,
	.attr_set = tcs3430_attr_set,
};

static int tcs3430_init(const struct device *dev)
{
	const struct tcs3430_config *cfg = dev->config;
	struct tcs3430_data *data = dev->data;
	int ret;

	k_sem_init(&data->data_sem, 0, K_SEM_MAX_LIMIT);
	data->dev = dev;

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}

	ret = tcs3430_sensor_setup(dev);
	if (ret < 0) {
		LOG_ERR("Failed to setup device: %d", ret);
		return ret;
	}

	return 0;
}

#define TCS3430_INIT(n)                                                                            \
	static struct tcs3430_data tcs3430_data_##n;                                               \
	static const struct tcs3430_config tcs3430_config_##n = {                                  \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
		.int_gpio = GPIO_DT_SPEC_INST_GET(n, int_gpios),                                   \
		.integration_cycles = DT_INST_PROP(n, integration_cycles),                        \
		.gain = DT_INST_PROP(n, gain),                                          \
	};                                                                                         \
	SENSOR_DEVICE_DT_INST_DEFINE(n, &tcs3430_init, NULL, &tcs3430_data_##n,                    \
				     &tcs3430_config_##n, POST_KERNEL,                             \
				     CONFIG_SENSOR_INIT_PRIORITY, &tcs3430_api);

DT_INST_FOREACH_STATUS_OKAY(TCS3430_INIT)

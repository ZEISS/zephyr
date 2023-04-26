/*
 * Copyright (c) 2023 Carl Zeiss Meditec AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT trinamic_tmc5041

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/misc/trinamic/trinamic.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_TMC_SPI
#include "tmc_spi.h"
#endif

LOG_MODULE_REGISTER(tmc5041, CONFIG_TRINAMIC_LOG_LEVEL);

struct tmc5041_data {
	/** Mutex to prevent further access  on the SPi Bus when a read is underway. */
	struct k_mutex mutex;
	/** INT pin GPIO callback. */
	struct gpio_callback int_cb;
};

struct tmc5041_config {
	/** SPI instance. */
	struct spi_dt_spec spi;
	/** Instance API. */
	struct trinamic_driver_api api;
#ifdef CONFIG_TMC5041_INT
	/** INT pin input (optional). */
	struct gpio_dt_spec int_pin;
#endif
};

#ifdef CONFIG_TMC5041_INT
static void tmc5041_int_pin_callback_handler(const struct device *port, struct gpio_callback *cb,
					     gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	/* for now, a message is shown */
	LOG_INF("INT pin interrupt detected");
}
#endif /* CONFIG_TMC5041_INT*/

static int tmc5041_init(const struct device *dev)
{
	const struct tmc5041_config *config = dev->config;
	struct tmc5041_data *data = dev->data;
	int ret = 0;

	k_mutex_init(&data->mutex);

	/* configure SPI */
	if (!spi_is_ready(&config->spi)) {
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

		ret = gpio_pin_configure_dt(&config->int_pin, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Could not configure INT GPIO (%d)", ret);
			return ret;
		}

		gpio_init_callback(&data->int_cb, tmc5041_int_pin_callback_handler,
				   BIT(config->int_pin.pin));

		ret = gpio_add_callback(config->int_pin.port, &data->int_cb);
		if (ret < 0) {
			LOG_ERR("Could not add INT pin GPIO callback (%d)", ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&config->int_pin, GPIO_INT_EDGE_BOTH);
		if (ret) {
			LOG_ERR("failed to configure INT interrupt (err %d)", ret);
			return -EINVAL;
		}
	}
#endif /* CONFIG_TMC5041_INT */
	LOG_INF("Device %s initialized", dev->name);
	return ret;
}

static inline int tmc5041_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val)
{
	const struct tmc5041_config *config = dev->config;
	struct tmc5041_data *data = dev->data;
	const struct spi_dt_spec bus = config->spi;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);
	ret = tmc_spi_read_register(&bus, reg_addr, reg_val);
	k_mutex_unlock(&data->mutex);
	return ret;
}

static inline int tmc5041_write(const struct device *dev, const uint8_t reg_addr,
				const uint32_t reg_val)
{
	const struct tmc5041_config *config = dev->config;
	struct tmc5041_data *data = dev->data;
	const struct spi_dt_spec bus = config->spi;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);
	ret = tmc_spi_write_register(&bus, reg_addr, reg_val);
	k_mutex_unlock(&data->mutex);
	return ret;
}

static const struct trinamic_driver_api tmc5041_api = {.read = tmc5041_read,
						       .write = tmc5041_write};

#define TMC5041_DEFINE(inst)                                                                       \
	static struct tmc5041_data tmc5041_data_##inst;                                            \
                                                                                                   \
	static const struct tmc5041_config tmc5041_config_##inst = {                               \
		.spi = SPI_DT_SPEC_INST_GET(inst,                                                  \
					    SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |                \
						    SPI_MODE_CPOL | SPI_MODE_CPHA |                \
						    SPI_WORD_SET(8),                               \
					    0),                                                    \
		IF_ENABLED(CONFIG_TMC5041_INT,                                                     \
			   (.int_pin = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {}), ))};         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, tmc5041_init, NULL, &tmc5041_data_##inst,                      \
			      &tmc5041_config_##inst, POST_KERNEL,                                 \
			      CONFIG_APPLICATION_INIT_PRIORITY, &tmc5041_api);

DT_INST_FOREACH_STATUS_OKAY(TMC5041_DEFINE)

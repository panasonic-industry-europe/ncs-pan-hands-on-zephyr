/*
 * Copyright (c) 2022 Panasonic Industrial Devices Europe GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT panasonic_sngcja5

// #include <string.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/i2c.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/sys/byteorder.h>
// #include <zephyr/sys/util.h>
// #include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
// #include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "sngcja5.h"

LOG_MODULE_REGISTER(SNGCJA5, CONFIG_SENSOR_LOG_LEVEL);

static int read_bytes(const struct i2c_dt_spec *dev, uint8_t addr, uint8_t *data, uint32_t num_bytes)
{
	struct i2c_msg msgs[2];

	msgs[0].buf = &addr;
	msgs[0].len = 1U;
	msgs[0].flags = I2C_MSG_WRITE;

	msgs[1].buf = data;
	msgs[1].len = num_bytes;
	msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer_dt(dev, &msgs[0], 2);
}

static int read_register_4(const struct i2c_dt_spec *dev, uint8_t addr, uint32_t *value)
{
	uint8_t buf[4];
	int ret;

	ret = read_bytes(dev, addr, &buf[0], 4);
	if (ret)
	{
		LOG_ERR("read_bytes() @ i2c failed");
		return -EIO;
	}

	*value = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

	return 0;
}

static int read_register_2(const struct i2c_dt_spec *dev, uint8_t addr, uint16_t *value)
{
	uint8_t buf[2];
	int ret;

	ret = read_bytes(dev, addr, &buf[0], 2);
	if (ret)
	{
		LOG_ERR("read_bytes() @ i2c failed");
		return -EIO;
	}

	*value = (buf[1] << 8) | buf[0];

	return 0;
}

static int sngcja5_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct sngcja5_data *device_data = dev->data;
	const struct sngcja5_config *device_config = dev->config;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	// Read out all relevant sensor value registers
	if(!read_register_4(&device_config->i2c, SNGCJA5_PM10_LL, &(device_data->pm1_0))){
		return -EIO;
	}
	if(!read_register_4(&device_config->i2c, SNGCJA5_PM25_LL, &(device_data->pm2_5))){
		return -EIO;
	}
	if(!read_register_4(&device_config->i2c, SNGCJA5_PM100_LL, &(device_data->pm10_0))){
		return -EIO;
	}
	if(!read_register_2(&device_config->i2c, SNGCJA5_05_L, &(device_data->pc0_5))){
		return -EIO;
	}
	if(!read_register_2(&device_config->i2c, SNGCJA5_10_L, &(device_data->pc1_0))){
		return -EIO;
	}
	if(!read_register_2(&device_config->i2c, SNGCJA5_25_L, &(device_data->pc2_5))){
		return -EIO;
	}
	if(!read_register_2(&device_config->i2c, SNGCJA5_50_L, &(device_data->pc5_0))){
		return -EIO;
	}
	if(!read_register_2(&device_config->i2c, SNGCJA5_75_L, &(device_data->pc7_5))){
		return -EIO;
	}
	if(!read_register_2(&device_config->i2c, SNGCJA5_100_L, &(device_data->pc10_0))){
		return -EIO;
	}

	return 0;
}

static int sngcja5_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct sngcja5_data *device_data = dev->data;
	int32_t uval;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	val[0].val1 = device_data->pc0_5;
	val[0].val2 = 0;
	val[1].val1 = device_data->pc1_0;
	val[1].val2 = 0;
	val[2].val1 = device_data->pc2_5;
	val[2].val2 = 0;
	val[3].val1 = device_data->pc5_0;
	val[3].val2 = 0;
	val[4].val1 = device_data->pc7_5;
	val[4].val2 = 0;
	val[5].val1 = device_data->pc10_0;
	val[5].val2 = 0;
	uval = (int32_t)device_data->pm1_0 * SNGCJA5_SCALE_FACTOR;
	val[6].val1 = uval / 1000000;
	val[6].val2 = uval % 1000000;
	uval = (int32_t)device_data->pm2_5 * SNGCJA5_SCALE_FACTOR;
	val[7].val1 = uval / 1000000;
	val[7].val2 = uval % 1000000;
	uval = (int32_t)device_data->pm10_0 * SNGCJA5_SCALE_FACTOR;
	val[8].val1 = uval / 1000000;
	val[8].val2 = uval % 1000000;

	return 0;
}

int sngcja5_init(const struct device *dev)
{
	const struct sngcja5_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device is not ready");
		return -ENODEV;
	}

	if (!i2c_configure(config->i2c.bus, I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER))
	{
		LOG_ERR("i2c_configure() failed");
		return -EIO;
	}

	return 0;
}

static const struct sensor_driver_api sngcja5_driver_api = {
	.sample_fetch = sngcja5_sample_fetch,
	.channel_get = sngcja5_channel_get,
};

#define SNGCJA5_DEFINE(inst)									\
	static struct sngcja5_data sngcja5_data_##inst;						\
												\
	static const struct sngcja5_config sngcja5_config_##inst = {				\
		.i2c = I2C_DT_SPEC_INST_GET(inst),						\
	};											\
												\
	DEVICE_DT_INST_DEFINE(inst, sngcja5_init, NULL,						\
			      &sngcja5_data_##inst, &sngcja5_config_##inst, POST_KERNEL,	\
			      CONFIG_SENSOR_INIT_PRIORITY, &sngcja5_driver_api);		\

DT_INST_FOREACH_STATUS_OKAY(SNGCJA5_DEFINE)

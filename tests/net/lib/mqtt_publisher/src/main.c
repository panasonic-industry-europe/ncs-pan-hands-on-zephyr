/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

extern void test_mqtt_init(void);
extern void test_mqtt_connect(void);
extern void test_mqtt_pingreq(void);
extern void test_mqtt_publish(void);
extern void test_mqtt_disconnect(void);

void test_main(void)
{
	ztest_test_suite(mqtt_test,
			ztest_unit_test(test_mqtt_connect),
			ztest_unit_test(test_mqtt_pingreq),
			ztest_unit_test(test_mqtt_publish),
			ztest_unit_test(test_mqtt_disconnect));
	ztest_run_test_suite(mqtt_test);
}

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>

extern void test_slist(void);
extern void test_sflist(void);
extern void test_dlist(void);
extern void test_dlist2(void);

void test_main(void)
{
	ztest_test_suite(dlist,
			 ztest_unit_test(test_dlist),
			 ztest_unit_test(test_dlist2),
			 ztest_unit_test(test_slist),
			 ztest_unit_test(test_sflist)
			 );

	ztest_run_test_suite(dlist);
}

/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/ztest.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(capt_test, LOG_LEVEL_DBG);

#define CAPTOUCH_NODE DT_NODELABEL(capt)

#if !DT_NODE_EXISTS(CAPTOUCH_NODE)
#error "Data tree node 'capt' is not defined!"
#endif

static void capt_event_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type == INPUT_EV_KEY && evt->code >= INPUT_BTN_0) {
		LOG_INF("Pin %d: %s", evt->code - INPUT_BTN_0, evt->value ? "Pressed" : "Released");
	}
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(CAPTOUCH_NODE), capt_event_cb, NULL);

ZTEST(capt_driver_tests, test_capt_init)
{
	const struct device *dev = DEVICE_DT_GET(CAPTOUCH_NODE);

	zassert_true(device_is_ready(dev), "CAPT device not ready!");

	k_msleep(200);
}

ZTEST_SUITE(capt_driver_tests, NULL, NULL, NULL, NULL, NULL);

/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>

LOG_MODULE_REGISTER(sample, LOG_LEVEL_INF);

#define REFRESH_RATE 100

static const struct device *const touch_dev = DEVICE_DT_GET(DT_NODELABEL(tsc20030));

static struct k_sem sync;

static struct {
	size_t x;
	size_t y;
	bool pressed;
} touch_point;

static void touch_event_callback(struct input_event *evt, void *user_data)
{
	if (evt->code == INPUT_ABS_X) {
		touch_point.x = evt->value;
	}
	if (evt->code == INPUT_ABS_Y) {
		touch_point.y = evt->value;
	}
	if (evt->code == INPUT_BTN_TOUCH) {
		touch_point.pressed = evt->value;
	}
	if (evt->sync) {
		k_sem_give(&sync);
	}
}

INPUT_CALLBACK_DEFINE(touch_dev, touch_event_callback, NULL);

int main(void)
{
	LOG_INF("Touch sample for touchscreen: %s", touch_dev->name);

	if (!device_is_ready(touch_dev)) {
		LOG_ERR("Device %s not found. Aborting sample.", touch_dev->name);
		return 0;
	}

	k_sem_init(&sync, 0, 1);

	while (1) {
		k_msleep(REFRESH_RATE);
		k_sem_take(&sync, K_FOREVER);
		LOG_INF("TOUCH %s X, Y: (%d, %d)", touch_point.pressed ? "PRESS" : "RELEASE",
			touch_point.x, touch_point.y);
	}

	return 0;
}

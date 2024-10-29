/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tsc2003

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tsc2003, CONFIG_INPUT_LOG_LEVEL);

/* TSC2003 used registers */
#define CMD_MEASURE_X 0xC0
#define CMD_MEASURE_Y 0xD0
#define CMD_MEASURE_Z1 0xB0
#define CMD_MEASURE_Z2 0xC0

/** TSC2003 configuration (DT). */
struct tsc2003_config {
    /** I2C bus. */
    struct i2c_dt_spec bus;
    struct gpio_dt_spec reset_gpio;

	uint16_t screen_width;
	uint16_t screen_height;

    int raw_x_min;
    int raw_y_min;
    uint16_t raw_x_max;
    uint16_t raw_y_max;

    bool inverted_x;
    bool inverted_y;
	bool swapped_x_y;
//#ifdef CONFIG_INPUT_TSC2003_INTERRUPT
    /** Interrupt GPIO information. */
    struct gpio_dt_spec int_gpio;
//#endif
};

/** TSC2003 data. */
struct tsc2003_data {
    /** Device pointer. */
    const struct device *dev;
    /** Work queue (for deferred read). */
    struct k_work work;
//#ifdef CONFIG_INPUT_TSC2003_INTERRUPT
    /** Interrupt GPIO callback. */
    struct gpio_callback int_gpio_cb;
//#else
    /** Timer (polling mode). */
//    struct k_timer timer;
//#endif
    /** Last pressed state. */
    bool pressed_old;
};

static int tsc2003_read_register(const struct device *dev, uint8_t cmd, uint16_t *data)
{
    const struct tsc2003_config *config = dev->config;
    uint8_t buf[2];
    int ret;

    ret = i2c_write_dt(&config->bus, &cmd, 1);
    if (ret < 0) {
        return ret;
    }

    ret = i2c_read_dt(&config->bus, buf, 2);
    if (ret < 0) {
        return ret;
    }

	LOG_DBG("buf[0]: %d, buf[1]: %d", buf[0], buf[1]);
    *data = (buf[0] << 4) | (buf[1] >> 4);  // Convert 12-bit value
	LOG_DBG("data: %d", *data);
    return 0;
}

static int tsc2003_process(const struct device *dev)
{
    struct tsc2003_data *data = dev->data;
    const struct tsc2003_config *config = dev->config;
    uint16_t raw_x, raw_y, z1, z2;
    int ret;

    /* Read X, Y, Z1, Z2 positions */
    ret = tsc2003_read_register(dev, CMD_MEASURE_X, &raw_x);
    if (ret < 0) {
        return ret;
    }

    ret = tsc2003_read_register(dev, CMD_MEASURE_Y, &raw_y);
    if (ret < 0) {
        return ret;
    }

    ret = tsc2003_read_register(dev, CMD_MEASURE_Z1, &z1);
    if (ret < 0) {
        return ret;
    }

    ret = tsc2003_read_register(dev, CMD_MEASURE_Z2, &z2);
    if (ret < 0) {
        return ret;
    }

    bool pressed = (z1 != 0 && z2 != 0);

    if (pressed) {
        int x = raw_x;
        int y = raw_y;

        if (config->screen_width > 0 && config->screen_height > 0) {
            x = (((int)raw_x - config->raw_x_min) * config->screen_width) /
                (config->raw_x_max - config->raw_x_min);
            y = (((int)raw_y - config->raw_y_min) * config->screen_height) /
                (config->raw_y_max - config->raw_y_min);

            x = CLAMP(x, 0, config->screen_width);
            y = CLAMP(y, 0, config->screen_height);
        }

        x = config->screen_width - x;
        y = config->screen_height - y;

        if (config->inverted_x) {
            x = config->screen_width - x;
        }
        if (config->inverted_y) {
            y = config->screen_height - y;
        }
        input_report_abs(dev, INPUT_ABS_X, x, false, K_FOREVER);
        input_report_abs(dev, INPUT_ABS_Y, y, false, K_FOREVER);
        input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);
    } else if (data->pressed_old && !pressed) {
        input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
    }

    data->pressed_old = pressed;

    return 0;
}

static void tsc2003_work_handler(struct k_work *work)
{
    struct tsc2003_data *data = CONTAINER_OF(work, struct tsc2003_data, work);
    const struct tsc2003_config *config = data->dev->config;

    //LOG_WRN("TEST");
    gpio_remove_callback(config->int_gpio.port, &data->int_gpio_cb);
    tsc2003_process(data->dev);
    gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
}

//#ifdef CONFIG_INPUT_TSC2003_INTERRUPT
static void tsc2003_isr_handler(const struct device *dev,
                   struct gpio_callback *cb, uint32_t pins)
{
    struct tsc2003_data *data = CONTAINER_OF(cb, struct tsc2003_data, int_gpio_cb);

    k_work_submit(&data->work);
}
//#else
//static void tsc2003_timer_handler(struct k_timer *timer)
//{
//    struct tsc2003_data *data = CONTAINER_OF(timer, struct tsc2003_data, timer);
//
//    k_work_submit(&data->work);
//}
//#endif

static int tsc2003_init(const struct device *dev)
{
    const struct tsc2003_config *config = dev->config;
    struct tsc2003_data *data = dev->data;
    int r;

    if (!device_is_ready(config->bus.bus)) {
        LOG_ERR("I2C controller device not ready");
        return -ENODEV;
    }

    data->dev = dev;

    k_work_init(&data->work, tsc2003_work_handler);

    if (config->reset_gpio.port != NULL) {
        /* Enable reset GPIO and assert reset */
        r = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);
        if (r < 0) {
            LOG_ERR("Could not enable reset GPIO");
            return r;
        }
        /*
         * Datasheet requires reset be held low 1 ms, or
         * 1 ms + 100us if powering on controller. Hold low for
         * 5 ms to be safe.
         */
        k_sleep(K_MSEC(5));
        /* Pull reset pin high to complete reset sequence */
        r = gpio_pin_set_dt(&config->reset_gpio, 0);
        if (r < 0) {
            return r;
        }
    }

//#ifdef CONFIG_INPUT_TSC2003_INTERRUPT
    if (!gpio_is_ready_dt(&config->int_gpio)) {
        LOG_ERR("Interrupt GPIO controller device not ready");
        return -ENODEV;
    }

    r = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
    if (r < 0) {
        LOG_ERR("Could not configure interrupt GPIO pin");
        return r;
    }

    r = gpio_pin_interrupt_configure_dt(&config->int_gpio,
                        GPIO_INT_EDGE_FALLING);
    if (r < 0) {
        LOG_ERR("Could not configure interrupt GPIO interrupt.");
        return r;
    }

    gpio_init_callback(&data->int_gpio_cb, tsc2003_isr_handler,
               BIT(config->int_gpio.pin));
    r = gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
    if (r < 0) {
        LOG_ERR("Could not set gpio callback");
        return r;
    }

    /* Read X, Y, Z1, Z2 positions */
    uint16_t value;
    r = tsc2003_read_register(dev, CMD_MEASURE_X, &value);
    if (r < 0) {
        return r;
    }
    LOG_INF("FAIRY: [%d]", value);
//#else
//    k_timer_init(&data->timer, tsc2003_timer_handler, NULL);
//    k_timer_start(&data->timer, K_MSEC(CONFIG_INPUT_TSC2003_PERIOD),
//              K_MSEC(CONFIG_INPUT_TSC2003_PERIOD));
//#endif

    return 0;
}

#define TSC2003_INIT(n)                                                    \
    static const struct tsc2003_config tsc2003_config_##n = {               \
        .bus = I2C_DT_SPEC_INST_GET(n),                                     \
        .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),         \
        .screen_width = DT_INST_PROP(n, screen_width),                           \
        .screen_height = DT_INST_PROP(n, screen_height),                         \
        .raw_x_min = DT_INST_PROP(n, raw_x_min),                            \
        .raw_y_min = DT_INST_PROP(n, raw_y_min),                            \
        .raw_x_max = DT_INST_PROP(n, raw_x_max),                            \
        .raw_y_max = DT_INST_PROP(n, raw_y_max),                            \
        .inverted_x = DT_NODE_HAS_PROP(n, inverted_x),                      \
        .inverted_y = DT_NODE_HAS_PROP(n, inverted_y),                      \
        .swapped_x_y = DT_NODE_HAS_PROP(n, swapped_x_y),                    \
        .int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),                 \
    };                                                                      \
    static struct tsc2003_data tsc2003_data_##n;                            \
    DEVICE_DT_INST_DEFINE(n, tsc2003_init, NULL,                            \
                          &tsc2003_data_##n, &tsc2003_config_##n,           \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TSC2003_INIT)

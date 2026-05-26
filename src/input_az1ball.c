/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT az1ball_az1ball

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(input_az1ball, CONFIG_ZMK_INPUT_AZ1BALL_LOG_LEVEL);

struct az1ball_config {
    struct i2c_dt_spec i2c;
    uint32_t polling_interval_ms;
    uint8_t count_type;
    bool invert_x;
    bool invert_y;
    bool swap_xy;
    uint8_t scale_x;
    uint8_t scale_y;
};

struct az1ball_data {
    struct k_work_delayable work;
    const struct device *dev;
    bool button_state;
};

/*
 * NOTE: az1uball's 0x90/0x91 "count-type" init write is intentionally REMOVED.
 * The palette-system aztouch trackpad uses a different command set
 * (0x30 / 0x40..0x45 / 0x4C) and a foreign byte upsets its I2C state.
 */

static int az1ball_read_report(const struct device *dev) {
    const struct az1ball_config *cfg = dev->config;
    struct az1ball_data *data = dev->data;
    uint8_t buf[5];

    int ret = i2c_read_dt(&cfg->i2c, buf, sizeof(buf));
    if (ret < 0) {
        LOG_WRN("I2C read failed: %d", ret);
        return ret;
    }

    LOG_DBG("raw %02x %02x %02x %02x %02x", buf[0], buf[1], buf[2], buf[3], buf[4]);

    int16_t left = buf[0];
    int16_t right = buf[1];
    int16_t up = buf[2];
    int16_t down = buf[3];

    int16_t dx = right - left;
    int16_t dy = down - up; // positive is down (Linux input convention)

    if (cfg->swap_xy) {
        int16_t tmp = dx;
        dx = dy;
        dy = tmp;
    }

    if (cfg->invert_x) {
        dx = -dx;
    }
    if (cfg->invert_y) {
        dy = -dy;
    }

    if (cfg->scale_x > 1) {
        dx *= cfg->scale_x;
    }
    if (cfg->scale_y > 1) {
        dy *= cfg->scale_y;
    }

    if (dx != 0 || dy != 0) {
        input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
    }

    bool pressed = (buf[4] & 0x80) != 0;
    if (pressed != data->button_state) {
        input_report_key(dev, INPUT_BTN_0, pressed, true, K_FOREVER);
        data->button_state = pressed;
    }

    return 0;
}

static void az1ball_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct az1ball_data *data = CONTAINER_OF(dwork, struct az1ball_data, work);
    const struct device *dev = data->dev;
    const struct az1ball_config *cfg = dev->config;

    az1ball_read_report(dev);

    k_work_reschedule(&data->work, K_MSEC(cfg->polling_interval_ms));
}

static int az1ball_init(const struct device *dev) {
    const struct az1ball_config *cfg = dev->config;
    struct az1ball_data *data = dev->data;

    if (!device_is_ready(cfg->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* Diagnostic: scan the I2C bus so the log shows exactly what ACKs. */
    for (uint16_t a = 0x08; a < 0x78; a++) {
        struct i2c_dt_spec probe = cfg->i2c;
        probe.addr = a;
        uint8_t b;
        if (i2c_read_dt(&probe, &b, 1) == 0) {
            LOG_INF("I2C scan: ACK at 0x%02x", a);
        }
    }
    LOG_INF("I2C scan done (configured addr 0x%02x)", cfg->i2c.addr);

    data->dev = dev;
    data->button_state = false;

    k_work_init_delayable(&data->work, az1ball_work_handler);
    k_work_schedule(&data->work, K_MSEC(cfg->polling_interval_ms));

    return 0;
}

#define AZ1BALL_INIT(inst)                                                                         \
    static struct az1ball_data az1ball_data_##inst;                                                 \
    static const struct az1ball_config az1ball_config_##inst = {                                    \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                          \
        .polling_interval_ms = DT_INST_PROP(inst, polling_interval_ms),                             \
        .count_type = DT_INST_PROP(inst, count_type),                                               \
        .invert_x = DT_INST_PROP(inst, invert_x),                                                   \
        .invert_y = DT_INST_PROP(inst, invert_y),                                                   \
        .swap_xy = DT_INST_PROP(inst, swap_xy),                                                     \
        .scale_x = DT_INST_PROP(inst, scale_x),                                                     \
        .scale_y = DT_INST_PROP(inst, scale_y),                                                     \
    };                                                                                              \
    DEVICE_DT_INST_DEFINE(inst, az1ball_init, NULL, &az1ball_data_##inst,                            \
                          &az1ball_config_##inst, POST_KERNEL,                                      \
                          CONFIG_ZMK_INPUT_AZ1BALL_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(AZ1BALL_INIT)

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

#ifdef CONFIG_BT
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#endif

#ifdef CONFIG_SETTINGS
#include <zephyr/settings/settings.h>
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(board_cdc_acm_uart))
#define AZ1BALL_USB_SERIAL 1
#include <zephyr/drivers/uart.h>
#include <stdlib.h>
#elif DT_NODE_EXISTS(DT_NODELABEL(cdc_acm_uart0))
#define AZ1BALL_USB_SERIAL 1
#include <zephyr/drivers/uart.h>
#include <stdlib.h>
#endif

LOG_MODULE_REGISTER(input_az1ball, CONFIG_ZMK_INPUT_AZ1BALL_LOG_LEVEL);

static uint8_t az1ball_speed;

static void az1ball_save_speed(void) {
#ifdef CONFIG_SETTINGS
    settings_save_one("az1ball/speed", &az1ball_speed, sizeof(az1ball_speed));
#endif
}

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

/* --- Settings persistence --- */

#ifdef CONFIG_SETTINGS
static int az1ball_settings_set(const char *name, size_t len,
                                settings_read_cb read_cb, void *cb_arg) {
    if (!strcmp(name, "speed")) {
        if (len != sizeof(az1ball_speed)) {
            return -EINVAL;
        }
        read_cb(cb_arg, &az1ball_speed, sizeof(az1ball_speed));
        LOG_INF("loaded speed=%u from settings", az1ball_speed);
        return 0;
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(az1ball, "az1ball", NULL,
                               az1ball_settings_set, NULL, NULL);
#endif /* CONFIG_SETTINGS */

/* --- BLE GATT configuration service --- */

#ifdef CONFIG_BT

#define AZ1BALL_SVC_UUID_VAL \
    BT_UUID_128_ENCODE(0xa0e3d001, 0xe8b0, 0x4e16, 0xb70b, 0x4535e8981884)
#define AZ1BALL_SPEED_UUID_VAL \
    BT_UUID_128_ENCODE(0xa0e3d002, 0xe8b0, 0x4e16, 0xb70b, 0x4535e8981884)

static struct bt_uuid_128 az1ball_svc_uuid =
    BT_UUID_INIT_128(AZ1BALL_SVC_UUID_VAL);
static struct bt_uuid_128 az1ball_speed_uuid =
    BT_UUID_INIT_128(AZ1BALL_SPEED_UUID_VAL);

static ssize_t speed_read_cb(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &az1ball_speed, sizeof(az1ball_speed));
}

static ssize_t speed_write_cb(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf, uint16_t len,
                              uint16_t offset, uint8_t flags) {
    if (len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    uint8_t val = ((const uint8_t *)buf)[0];
    if (val < 1 || val > 10) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    az1ball_speed = val;
    LOG_INF("speed set to %u via BLE", val);
    az1ball_save_speed();
    return len;
}

BT_GATT_SERVICE_DEFINE(az1ball_config_svc,
    BT_GATT_PRIMARY_SERVICE(&az1ball_svc_uuid),
    BT_GATT_CHARACTERISTIC(&az1ball_speed_uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
        speed_read_cb, speed_write_cb, NULL),
);

#endif /* CONFIG_BT */

/* --- USB Serial configuration interface --- */

#ifdef AZ1BALL_USB_SERIAL

#if DT_NODE_EXISTS(DT_NODELABEL(board_cdc_acm_uart))
static const struct device *const serial_dev =
    DEVICE_DT_GET(DT_NODELABEL(board_cdc_acm_uart));
#else
static const struct device *const serial_dev =
    DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));
#endif
static struct k_work_delayable serial_work;
static uint8_t serial_buf[16];
static uint8_t serial_buf_pos;

static void serial_send(const char *s) {
    for (; *s; s++) {
        uart_poll_out(serial_dev, *s);
    }
}

static void serial_process_cmd(void) {
    serial_buf[serial_buf_pos] = '\0';
    char *cmd = (char *)serial_buf;
    char resp[16];

    if (cmd[0] == '?') {
        snprintf(resp, sizeof(resp), "SPD:%u\n", az1ball_speed);
        serial_send(resp);
    } else {
        int val = atoi(cmd);
        if (val >= 1 && val <= 10) {
            az1ball_speed = (uint8_t)val;
            az1ball_save_speed();
            snprintf(resp, sizeof(resp), "OK:%u\n", az1ball_speed);
            serial_send(resp);
            LOG_INF("speed set to %u via serial", az1ball_speed);
        } else {
            serial_send("ERR\n");
        }
    }
    serial_buf_pos = 0;
}

static void serial_work_handler(struct k_work *work) {
    if (!device_is_ready(serial_dev)) {
        k_work_reschedule(&serial_work, K_MSEC(500));
        return;
    }

    uint8_t c;
    while (uart_poll_in(serial_dev, &c) == 0) {
        if (c == '\n' || c == '\r') {
            if (serial_buf_pos > 0) {
                serial_process_cmd();
            }
        } else if (serial_buf_pos < sizeof(serial_buf) - 1) {
            serial_buf[serial_buf_pos++] = c;
        }
    }
    k_work_reschedule(&serial_work, K_MSEC(100));
}

static void az1ball_serial_init(void) {
    if (!device_is_ready(serial_dev)) {
        LOG_WRN("CDC ACM UART not ready");
        return;
    }
    k_work_init_delayable(&serial_work, serial_work_handler);
    k_work_schedule(&serial_work, K_MSEC(1000));
    LOG_INF("USB serial config ready");
}

#endif /* AZ1BALL_USB_SERIAL */

/* --- Sensor polling --- */

static int az1ball_read_report(const struct device *dev) {
    const struct az1ball_config *cfg = dev->config;
    struct az1ball_data *data = dev->data;
    uint8_t buf[5];

    int ret = i2c_read_dt(&cfg->i2c, buf, sizeof(buf));
    if (ret < 0) {
        LOG_WRN("I2C read failed: %d", ret);
        return ret;
    }

    LOG_DBG("raw %02x %02x %02x %02x %02x",
            buf[0], buf[1], buf[2], buf[3], buf[4]);

    int16_t left = buf[0];
    int16_t right = buf[1];
    int16_t up = buf[2];
    int16_t down = buf[3];

    int16_t dx = right - left;
    int16_t dy = down - up;

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

    uint8_t s = az1ball_speed > 0 ? az1ball_speed : cfg->scale_x;
    if (s > 1) {
        dx *= s;
        dy *= s;
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

    if (az1ball_speed == 0) {
        az1ball_speed = cfg->scale_x > 0 ? cfg->scale_x : 1;
    }

    {
        uint8_t cmd = 0x20;
        int wret = i2c_write_dt(&cfg->i2c, &cmd, 1);
        LOG_INF("aztouch set-mode 0x20 -> %d (%s)", wret,
                wret == 0 ? "ACK / chip alive" : "NO-ACK");
    }

    data->dev = dev;
    data->button_state = false;

    k_work_init_delayable(&data->work, az1ball_work_handler);
    k_work_schedule(&data->work, K_MSEC(cfg->polling_interval_ms));

#ifdef AZ1BALL_USB_SERIAL
    az1ball_serial_init();
#endif

    return 0;
}

#define AZ1BALL_INIT(inst)                                                     \
    static struct az1ball_data az1ball_data_##inst;                             \
    static const struct az1ball_config az1ball_config_##inst = {                \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                    \
        .polling_interval_ms = DT_INST_PROP(inst, polling_interval_ms),        \
        .count_type = DT_INST_PROP(inst, count_type),                          \
        .invert_x = DT_INST_PROP(inst, invert_x),                             \
        .invert_y = DT_INST_PROP(inst, invert_y),                             \
        .swap_xy = DT_INST_PROP(inst, swap_xy),                               \
        .scale_x = DT_INST_PROP(inst, scale_x),                               \
        .scale_y = DT_INST_PROP(inst, scale_y),                               \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(inst, az1ball_init, NULL, &az1ball_data_##inst,       \
                          &az1ball_config_##inst, POST_KERNEL,                 \
                          CONFIG_ZMK_INPUT_AZ1BALL_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(AZ1BALL_INIT)

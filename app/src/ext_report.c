/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/init.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/ext_report.h>
#include <drivers/ext_report.h>

static const struct device *ext_report = NULL;

static int zmk_ext_init(const struct device *dev) {
    ARG_UNUSED(dev);

    ext_report = device_get_binding("EXT_REPORT");
    if (ext_report == NULL) {
        LOG_ERR("Failed to get ext_report device binding");
        return -ENODEV;
    }

    return 0;
}

bool zmk_ext_is_connected() {
    bool connected = false;

    if (ext_report == NULL) {
        return false;
    }

    ext_report_is_connected(ext_report, &connected);

    return connected;
}

int zmk_ext_send_keyboard_report(struct zmk_hid_keyboard_report_body *body) {
    if (ext_report == NULL) {
        return -ENODEV;
    }

    return ext_report_send_keyboard_report(ext_report, body);
}

int zmk_ext_send_consumer_report(struct zmk_hid_consumer_report_body *body) {
    if (ext_report == NULL) {
        return -ENODEV;
    }

    return ext_report_send_consumer_report(ext_report, body);
}

SYS_INIT(zmk_ext_init, APPLICATION, CONFIG_ZMK_EXT_REPORT_INIT_PRIORITY);

/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>

#include <zmk/hid.h>

#ifdef __cplusplus
extern "C" {
#endif

enum ble_ext_state {
    BLE_EXT_STATE_IDLE = 0,
    BLE_EXT_STATE_PAIRING,
    BLE_EXT_STATE_CONNECTED,
};

typedef void (*ble_ext_state_callback_t)(const struct device *dev, uint8_t profile,
                                         enum ble_ext_state state);

typedef int (*ble_ext_select_profile_t)(const struct device *dev, uint8_t profile);

typedef int (*ble_ext_pair_t)(const struct device *dev);

typedef int (*ble_ext_connect_t)(const struct device *dev);

typedef int (*ble_ext_get_state_t)(const struct device *dev, enum ble_ext_state *state);

typedef int (*ble_ext_set_state_callback_t)(const struct device *dev,
                                            ble_ext_state_callback_t callback);

typedef int (*ble_ext_send_keyboard_report_t)(const struct device *dev,
                                              struct zmk_hid_keyboard_report_body *body);

typedef int (*ble_ext_send_consumer_report_t)(const struct device *dev,
                                              struct zmk_hid_consumer_report_body *body);

struct ble_ext_driver_api {
    ble_ext_select_profile_t select_profile;
    ble_ext_pair_t pair;
    ble_ext_connect_t connect;
    ble_ext_get_state_t get_state;
    ble_ext_set_state_callback_t set_state_callback;
    ble_ext_send_keyboard_report_t send_keyboard_report;
    ble_ext_send_consumer_report_t send_consumer_report;
};

static inline int ble_ext_select_profile(const struct device *dev, uint8_t profile) {
    const struct ble_ext_driver_api *api = (const struct ble_ext_driver_api *)dev->api;

    if (api->select_profile == NULL) {
        return -ENOTSUP;
    }

    return api->select_profile(dev, profile);
}

static inline int ble_ext_pair(const struct device *dev) {
    const struct ble_ext_driver_api *api = (const struct ble_ext_driver_api *)dev->api;

    if (api->pair == NULL) {
        return -ENOTSUP;
    }

    return api->pair(dev);
}

static inline int ble_ext_connect(const struct device *dev) {
    const struct ble_ext_driver_api *api = (const struct ble_ext_driver_api *)dev->api;

    if (api->connect == NULL) {
        return -ENOTSUP;
    }

    return api->connect(dev);
}

static inline int ble_ext_get_state(const struct device *dev, enum ble_ext_state *state) {
    const struct ble_ext_driver_api *api = (const struct ble_ext_driver_api *)dev->api;

    if (api->get_state == NULL) {
        return -ENOTSUP;
    }

    return api->get_state(dev, state);
}

static inline int ble_ext_set_state_callback(const struct device *dev,
                                             ble_ext_state_callback_t callback) {
    const struct ble_ext_driver_api *api = (const struct ble_ext_driver_api *)dev->api;

    if (api->set_state_callback == NULL) {
        return -ENOTSUP;
    }

    return api->set_state_callback(dev, callback);
}

static inline int ble_ext_send_keyboard_report(const struct device *dev,
                                               struct zmk_hid_keyboard_report_body *body) {
    const struct ble_ext_driver_api *api = (const struct ble_ext_driver_api *)dev->api;

    if (api->send_keyboard_report == NULL) {
        return -ENOTSUP;
    }

    return api->send_keyboard_report(dev, body);
}

static inline int ble_ext_send_consumer_report(const struct device *dev,
                                               struct zmk_hid_consumer_report_body *body) {
    const struct ble_ext_driver_api *api = (const struct ble_ext_driver_api *)dev->api;

    if (api->send_consumer_report == NULL) {
        return -ENOTSUP;
    }

    return api->send_consumer_report(dev, body);
}

#ifdef __cplusplus
}
#endif

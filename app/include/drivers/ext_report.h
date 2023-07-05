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

typedef int (*ext_report_profile_get_count_t)(const struct device *dev, uint8_t *count);

typedef int (*ext_report_profile_select_t)(const struct device *dev, uint8_t index);

typedef int (*ext_report_profile_get_selected_t)(const struct device *dev, uint8_t *index);

typedef int (*ext_report_clear_bonds_t)(const struct device *dev);

typedef int (*ext_report_is_connected_t)(const struct device *dev, bool *connected);

typedef int (*ext_report_send_keyboard_report_t)(const struct device *dev,
                                                 struct zmk_hid_keyboard_report_body *body);

typedef int (*ext_report_send_consumer_report_t)(const struct device *dev,
                                                 struct zmk_hid_consumer_report_body *body);

struct ext_report_driver_api {
    ext_report_profile_get_count_t profile_get_count;
    ext_report_profile_select_t profile_select;
    ext_report_profile_get_selected_t profile_get_selected;
    ext_report_clear_bonds_t clear_bonds;
    ext_report_is_connected_t is_connected;
    ext_report_send_keyboard_report_t send_keyboard_report;
    ext_report_send_consumer_report_t send_consumer_report;
};

static inline int ext_report_profile_get_count(const struct device *dev, uint8_t *count) {
    const struct ext_report_driver_api *api = (const struct ext_report_driver_api *)dev->api;

    if (api->profile_get_count == NULL) {
        return -ENOTSUP;
    }

    return api->profile_get_count(dev, count);
}

static inline int ext_report_profile_select(const struct device *dev, uint8_t index) {
    const struct ext_report_driver_api *api = (const struct ext_report_driver_api *)dev->api;

    if (api->profile_select == NULL) {
        return -ENOTSUP;
    }

    return api->profile_select(dev, index);
}

static inline int ext_report_profile_get_selected(const struct device *dev, uint8_t *index) {
    const struct ext_report_driver_api *api = (const struct ext_report_driver_api *)dev->api;

    if (api->profile_get_selected == NULL) {
        return -ENOTSUP;
    }

    return api->profile_get_selected(dev, index);
}

static inline int ext_report_clear_bonds(const struct device *dev) {
    const struct ext_report_driver_api *api = (const struct ext_report_driver_api *)dev->api;

    if (api->clear_bonds == NULL) {
        return -ENOTSUP;
    }

    return api->clear_bonds(dev);
}

static inline int ext_report_is_connected(const struct device *dev, bool *connected) {
    const struct ext_report_driver_api *api = (const struct ext_report_driver_api *)dev->api;

    if (api->is_connected == NULL) {
        return -ENOTSUP;
    }

    return api->is_connected(dev, connected);
}

static inline int ext_report_send_keyboard_report(const struct device *dev,
                                                  struct zmk_hid_keyboard_report_body *body) {
    const struct ext_report_driver_api *api = (const struct ext_report_driver_api *)dev->api;

    if (api->send_keyboard_report == NULL) {
        return -ENOTSUP;
    }

    return api->send_keyboard_report(dev, body);
}

static inline int ext_report_send_consumer_report(const struct device *dev,
                                                  struct zmk_hid_consumer_report_body *body) {
    const struct ext_report_driver_api *api = (const struct ext_report_driver_api *)dev->api;

    if (api->send_consumer_report == NULL) {
        return -ENOTSUP;
    }

    return api->send_consumer_report(dev, body);
}

#ifdef __cplusplus
}
#endif

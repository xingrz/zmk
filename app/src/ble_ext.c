/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/init.h>
#include <zephyr/device.h>

#if defined(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <stdlib.h>

#include <zmk/ble.h>
#include <zmk/hog.h>
#include <zmk/event_manager.h>
#include <zmk/events/ble_active_profile_changed.h>

#include <drivers/ble_ext.h>

#define ZMK_BLE_EXT_NODE DT_CHOSEN(zmk_ble_ext)
#define ZMK_BLE_EXT_PROFILE_COUNT DT_PROP(ZMK_BLE_EXT_NODE, profile_count)

static const struct device *ble_ext = DEVICE_DT_GET_OR_NULL(ZMK_BLE_EXT_NODE);

static struct zmk_ble_profile profiles[ZMK_BLE_EXT_PROFILE_COUNT];
static uint8_t active_profile;

static void raise_profile_changed_event() {
    ZMK_EVENT_RAISE(new_zmk_ble_active_profile_changed((struct zmk_ble_active_profile_changed){
        .index = active_profile, .profile = &profiles[active_profile]}));
}

static void raise_profile_changed_event_callback(struct k_work *work) {
    raise_profile_changed_event();
}

K_WORK_DEFINE(raise_profile_changed_event_work, raise_profile_changed_event_callback);

#if defined(CONFIG_SETTINGS)
static int ble_profiles_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg);

static struct settings_handler ble_profiles_handler = {
    .name = "ble",
    .h_set = ble_profiles_handle_set,
};

static struct k_work_delayable ble_save_work;

static int ble_profiles_handle_set(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg) {
    int ret;
    const char *next;

    LOG_DBG("Setting BLE value %s", name);

    if (settings_name_steq(name, "active_profile", &next) && !next) {
        if (len != sizeof(active_profile)) {
            LOG_ERR("Invalid active profile size (got %d expected %d)", len,
                    sizeof(active_profile));
            return -EINVAL;
        }

        ret = read_cb(cb_arg, &active_profile, sizeof(active_profile));
        if (ret <= 0) {
            LOG_ERR("Failed to read active profile from settings (err %d)", ret);
            return ret;
        }
    }

    return 0;
}

static void ble_save_profile_work(struct k_work *work) {
    settings_save_one("ble/active_profile", &active_profile, sizeof(active_profile));
}
#endif /* CONFIG_SETTINGS */

static int ble_save_profile() {
#if defined(CONFIG_SETTINGS)
    return k_work_reschedule(&ble_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
#else
    return 0;
#endif
}

static int ble_set_bonded(uint8_t index, bool bonded) {
    return k_work_submit(&raise_profile_changed_event_work);
}

int zmk_ble_clear_bonds() {
    ble_set_bonded(active_profile, false);
    return ble_ext_pair(ble_ext);
}

int zmk_ble_prof_next() {
    if (ble_ext == NULL) {
        return -ENODEV;
    }

    return ble_ext_select_profile(ble_ext, (active_profile + 1) % ZMK_BLE_EXT_PROFILE_COUNT);
}

int zmk_ble_prof_prev() {
    if (ble_ext == NULL) {
        return -ENODEV;
    }

    return ble_ext_select_profile(ble_ext, (active_profile + ZMK_BLE_EXT_PROFILE_COUNT - 1) %
                                               ZMK_BLE_EXT_PROFILE_COUNT);
}

int zmk_ble_prof_select(uint8_t index) {
    int ret;

    if (ble_ext == NULL) {
        return -ENODEV;
    }

    LOG_DBG("Selecting profile %d", index);

    ret = ble_ext_select_profile(ble_ext, index);
    if (ret < 0) {
        return ret;
    }

    active_profile = index;
    ble_save_profile();

    return ble_ext_connect(ble_ext);
}

int zmk_ble_active_profile_index() { return active_profile; }

bt_addr_le_t *zmk_ble_active_profile_addr() { return NULL; }

bool zmk_ble_active_profile_is_open() {
    int ret;
    enum ble_ext_state state;

    if (ble_ext == NULL) {
        return false;
    }

    ret = ble_ext_get_state(ble_ext, &state);
    if (ret < 0) {
        return false;
    }

    return state == BLE_EXT_STATE_PAIRING;
}

bool zmk_ble_active_profile_is_connected() {
    int ret;
    enum ble_ext_state state;

    if (ble_ext == NULL) {
        return false;
    }

    ret = ble_ext_get_state(ble_ext, &state);
    if (ret < 0) {
        return false;
    }

    return state == BLE_EXT_STATE_CONNECTED;
}

char *zmk_ble_active_profile_name() { return ""; }

int zmk_hog_send_keyboard_report(struct zmk_hid_keyboard_report_body *body) {
    if (ble_ext == NULL) {
        return -ENODEV;
    }

    return ble_ext_send_keyboard_report(ble_ext, body);
}

int zmk_hog_send_consumer_report(struct zmk_hid_consumer_report_body *body) {
    if (ble_ext == NULL) {
        return -ENODEV;
    }

    return ble_ext_send_consumer_report(ble_ext, body);
}

static void zmk_ble_ext_state_callback(const struct device *dev, uint8_t profile,
                                       enum ble_ext_state state) {
    ARG_UNUSED(dev);

    LOG_DBG("BLE state changed to %d", state);

    if (state == BLE_EXT_STATE_CONNECTED) {
        ble_set_bonded(profile, true);
    } else if (state == BLE_EXT_STATE_IDLE) {
        ble_ext_pair(ble_ext);
    }
}

int zmk_ble_ext_init(const struct device *dev) {
    ARG_UNUSED(dev);
    int ret;

    if (ble_ext == NULL) {
        LOG_ERR("Failed to get ble_ext device");
        return -ENODEV;
    }

    ble_ext_set_state_callback(ble_ext, zmk_ble_ext_state_callback);

#if defined(CONFIG_SETTINGS)
    settings_subsys_init();

    ret = settings_register(&ble_profiles_handler);
    if (ret < 0) {
        LOG_ERR("Failed to register settings handler (err %d)", ret);
        return ret;
    }

    k_work_init_delayable(&ble_save_work, ble_save_profile_work);

    settings_load_subtree("ble");
#endif /* CONFIG_SETTINGS */

    return 0;
}

SYS_INIT(zmk_ble_ext_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);

/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zmk/hid.h>

bool zmk_ext_is_connected();

int zmk_ext_send_keyboard_report(struct zmk_hid_keyboard_report_body *body);

int zmk_ext_send_consumer_report(struct zmk_hid_consumer_report_body *body);

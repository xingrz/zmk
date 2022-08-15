# Copyright (c) 2022 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_sources_ifdef(CONFIG_WS2812_STRIP_PWM ws2812_pwm_stm32.c)

# Copyright (c) 2022 The ZMK Contributors
# SPDX-License-Identifier: MIT

board_runner_args(pyocd "--target=nrf52840" "--frequency=4000000")

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)

# Copyright (c) 2022 The ZMK Contributors
# SPDX-License-Identifier: MIT

set_ifndef(BOARD_FLASH_RUNNER pyocd)

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)

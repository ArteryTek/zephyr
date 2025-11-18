# SPDX-License-Identifier: Apache-2.0

# keep first
board_runner_args(pyocd "--target=at32f407vgt7")
board_runner_args(jlink "--device=AT32F407VGT7" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)

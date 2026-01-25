# boards/arm/oceanpad/board.cmake

set(BOARD_FLASH_RUNNER openocd)
set(BOARD_DEBUG_RUNNER openocd)



board_runner_args(openocd --config "${BOARD_DIR}/support/openocd.cfg")
board_runner_args(openocd --cmd-erase "nrf5 mass_erase")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)

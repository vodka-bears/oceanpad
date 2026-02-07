#pragma once

#include <zephyr/types.h>

#include "GamepadState.hpp"

struct NativeButtons {
    uint8_t lb : 1;
    uint8_t rb : 1;
    uint8_t ls : 1;
    uint8_t rs : 1;

    uint8_t start : 1;
    uint8_t back  : 1;
    uint8_t home  : 1;
    uint8_t xd_switch   : 1;
};

struct __packed ExpanderButtons {
    uint8_t dpad_left   : 1;
    uint8_t dpad_down   : 1;
    uint8_t dpad_up     : 1;
    uint8_t pad1        : 1;
    uint8_t dpad_right  : 1;
    uint8_t mode        : 1;
    uint8_t vibr        : 1;
    uint8_t pad2        : 1;

    uint8_t a           : 1;
    uint8_t x           : 1;
    uint8_t y           : 1;
    uint8_t b           : 1;
    uint8_t pad3        : 4;
};

struct RawAxes {
    uint16_t raw_lx;
    uint16_t raw_ly;
    uint16_t raw_rx;
    uint16_t raw_ry;
    uint16_t raw_lt;
    uint16_t raw_rt;
};

struct RawData {
    NativeButtons native_buttons;
    ExpanderButtons expander_buttons;
    RawAxes raw_axes;
    IMUData raw_imu;
};

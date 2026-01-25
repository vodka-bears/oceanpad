#pragma once

#include <zephyr/types.h>

struct AnalogAxes {
    int16_t stick_lx, stick_ly;
    int16_t stick_rx, stick_ry;
    uint16_t trigger_lt, trigger_rt;
};

struct DigitalButtons {
    uint16_t a : 1;
    uint16_t b : 1;
    uint16_t x : 1;
    uint16_t y : 1;

    uint16_t lb : 1;
    uint16_t rb : 1;

    uint16_t ls : 1;
    uint16_t rs : 1;

    uint16_t start : 1;
    uint16_t back  : 1;
    uint16_t home  : 1;

    uint16_t mode  : 1;
    uint16_t vibr  : 1;

    uint16_t switch_xd : 1;
};

enum class DPadState : uint8_t {
    Up = 0,
    UpRight,
    Right,
    DownRight,
    Down,
    DownLeft,
    Left,
    UpLeft,
    Centered,
};

struct IMUData {
    int16_t gyro[3];
    int16_t accel[3];
};

struct GamepadState {
    AnalogAxes axes;
    DigitalButtons buttons;
    DPadState dpad;
    IMUData imu_data;
    uint8_t battery_percent;
};

struct VibrationData {
    uint8_t big_motor;
    uint8_t small_motor;
};

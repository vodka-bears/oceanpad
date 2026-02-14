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

    uint16_t xd_switch : 1;

    uint16_t pad : 2;
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

struct __packed VibrationDataXbox {
    uint8_t enable_right_motor : 1;
    uint8_t enable_left_motor : 1;
    uint8_t enable_right_trigger : 1;
    uint8_t enable_left_trigger : 1;
    uint8_t pad_enable       : 4;

    uint8_t magnitude_left_trigger;
    uint8_t magnitude_right_trigger;
    uint8_t magnitude_left_motor;
    uint8_t magnitude_right_motor;

    uint8_t duration;

    uint8_t start_delay;

    uint8_t loop_count;
};

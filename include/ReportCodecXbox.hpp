#pragma once
#include "ReportCodec.hpp"
#include "GamepadState.hpp"

class ReportCodecXbox : public ReportCodec<GamepadState, VibrationData> {
public:
    virtual uint8_t encode_input(uint8_t report_id, const GamepadState& state, uint8_t* buffer, uint16_t buffer_len) const override;
    virtual VibrationData decode_output(uint8_t report_id, const uint8_t* report_buffer, uint16_t report_len) const override;
};

struct __packed InputReportXbox {
    uint16_t lx;
    uint16_t ly;
    uint16_t rx;
    uint16_t ry;

    uint16_t lt : 10;
    uint16_t pad_lt : 6;

    uint16_t rt : 10;
    uint16_t pad_rt : 6;

    uint8_t  hat  : 4;
    uint8_t  pad_hat : 4;

    uint16_t a      : 1; // 0x01
    uint16_t b      : 1; // 0x02
    uint16_t pad_b1 : 1; // 0x04 (skip)
    uint16_t x      : 1; // 0x08
    uint16_t y      : 1; // 0x10
    uint16_t pad_b2 : 1; // 0x20 (skip)
    uint16_t lb     : 1; // 0x40
    uint16_t rb     : 1; // 0x80
    uint16_t pad_b3 : 2; // 0x100, 0x200 (skip)
    uint16_t back   : 1; // 0x400
    uint16_t start  : 1; // 0x800
    uint16_t guide  : 1; // 0x1000
    uint16_t ls     : 1; // 0x2000
    uint16_t rs     : 1; // 0x4000
    uint16_t pad_b4 : 1; // 0x8000

    uint8_t  share     : 1; // 0x01
    uint8_t  pad_share : 7;
};

struct __packed OutputReportXbox {
    uint8_t enable_actuators : 4;
    uint8_t pad_enable       : 4;

    uint8_t motor_magnitude[4];

    uint8_t duration;

    uint8_t start_delay;

    uint8_t loop_count;
};

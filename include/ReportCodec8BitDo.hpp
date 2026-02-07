#pragma once
#include "ReportCodec.hpp"
#include "GamepadState.hpp"

class ReportCodec8BitDo : public ReportCodec<GamepadState, VibrationDataXbox> {
public:
    virtual uint8_t encode_input(uint8_t report_id, const GamepadState& data, uint8_t* report_buffer, uint16_t report_len) const override;
    virtual uint8_t decode_output(uint8_t report_id, VibrationDataXbox& data, const uint8_t* report_buffer, uint16_t report_len) const override;
};

struct __packed InputReport8BitDo {
    uint8_t  dpad      : 4;
    uint8_t  pad_dpad  : 4;

    uint8_t  lx;
    uint8_t  ly;
    uint8_t  rx;
    uint8_t  ry;

    uint8_t  rt;
    uint8_t  lt;

    uint8_t  a  : 1;
    uint8_t  b  : 1;
    uint8_t  pl : 1;
    uint8_t  x  : 1;
    uint8_t  y  : 1;
    uint8_t  pr : 1;
    uint8_t  lb : 1;
    uint8_t  rb : 1;

    uint8_t  lt_btn : 1;
    uint8_t  rt_btn : 1;
    uint8_t  view   : 1;
    uint8_t  menu   : 1;
    uint8_t  home   : 1;
    uint8_t  ls     : 1;
    uint8_t  rs     : 1;
    uint8_t  pad_b1 : 1;

    uint8_t  l4     : 1;
    uint8_t  r4     : 1;
    uint8_t  pad_b2 : 6;

    uint8_t  vendor_zero;
    uint8_t  const12;
    uint8_t  const60;
    uint8_t  battery;

    int16_t  accel[3];
    int16_t  gyro[3];

    uint8_t  vendor_padding[7];
};

struct __packed OutputReport8BitDo {
    uint8_t motor_big;
    uint8_t motor_small;
    uint8_t padding[2];
};

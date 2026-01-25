#include <string.h>
#include "ReportCodec8BitDo.hpp"
#include "GamepadState.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ReportCodec8BitDo, LOG_LEVEL_DBG);

uint8_t ReportCodec8BitDo::encode_input(uint8_t report_id, const GamepadState& state, uint8_t* buffer, uint16_t buffer_len) const {
    if (report_id != 1) {
        LOG_WRN("Only report id 1 is supported but %d is requested, ignoring", report_id);
        return 0;
    }
    if (buffer_len < 33)
    {
        LOG_WRN("Buffer size of at least 33 is required but %d is provided, ignoring", buffer_len);
        return 0;
    }
    auto* report = reinterpret_cast<InputReport8BitDo*>(buffer);

    memset(report, 0, sizeof(InputReport8BitDo));

    report->dpad = state.dpad == DPadState::Centered ? 0xF : static_cast<uint8_t>(state.dpad);

    auto conv_stick = [](int32_t stick_in) -> uint8_t {
        int32_t val = (stick_in <= -32767) ? -32768 : stick_in;
        return static_cast<uint8_t>((val >> 8) + 128);
    };

    report->lx  = conv_stick(state.axes.stick_lx);
    report->ly  = conv_stick(state.axes.stick_ly);
    report->rx  = conv_stick(state.axes.stick_rx);
    report->ry  = conv_stick(state.axes.stick_ry);

    report->lt = (uint8_t)(state.axes.trigger_lt >> 8);
    report->rt = (uint8_t)(state.axes.trigger_rt >> 8);



    report->a = state.buttons.a;
    report->b = state.buttons.b;
    report->pl = state.buttons.vibr;
    report->x = state.buttons.x;
    report->y = state.buttons.y;
    report->lb = state.buttons.lb;
    report->rb = state.buttons.rb;

    report->lt_btn = report->lt >= 0x4e;
    report->rt_btn = report->rt >= 0x4e;
    report->view = state.buttons.back;
    report->menu = state.buttons.start;
    report->home = state.buttons.home;
    report->ls = state.buttons.ls;
    report->rs = state.buttons.rs;

    report->vendor_zero = 0;
    report->const12 = 0x12;
    report->const60 = 0x60;
    report->battery = (uint8_t)((uint16_t)state.battery_percent * 255 / 100);

    report->accel[0] = state.imu_data.accel[0];
    report->accel[1] = -state.imu_data.accel[1];
    report->accel[2] = -state.imu_data.accel[2];
    report->gyro[0] = state.imu_data.gyro[0];
    report->gyro[1] = -state.imu_data.gyro[1];
    report->gyro[2] = -state.imu_data.gyro[2];

    return sizeof(InputReport8BitDo);
}

VibrationData ReportCodec8BitDo::decode_output(uint8_t report_id, const uint8_t* report_buffer, uint16_t report_len) const {
    if (report_id != 5) {
        LOG_WRN("Only report id 5 is supported but %d is requested, ignoring", report_id);
        return { 0, 0 };
    }
    if (report_len != 4) {
        LOG_WRN("Output report id 5 must be 4 bytes long but %d is provided, ignoring", report_len);
        return { 0, 0 };
    }
    LOG_HEXDUMP_DBG(report_buffer, report_len, "Output report 5: ");
    auto* report = reinterpret_cast<const OutputReport8BitDo*>(report_buffer);
    VibrationData ret_data;
    ret_data.big_motor = (uint8_t)((uint16_t)(report->motor_big) * 100 / 255);
    ret_data.small_motor = (uint8_t)((uint16_t)(report->motor_small) * 100 / 255);
    LOG_DBG("Big motor %3d Small Motor %3d", ret_data.big_motor, ret_data.small_motor);
    return ret_data;
}

#include <string.h>
#include "ReportCodecXbox.hpp"
#include "GamepadState.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ReportCodecXbox, LOG_LEVEL_WRN);

uint8_t ReportCodecXbox::encode_input(uint8_t report_id, const GamepadState& state, uint8_t* buffer, uint16_t buffer_len) const {
    if (report_id != 1) {
        LOG_WRN("Only report id 1 is supported but %d is requested, ignoring", report_id);
        return 0;
    }
    if (buffer_len < 16)
    {
        LOG_WRN("Buffer size of at least 16 is required but %d is provided, ignoring", buffer_len);
        return 0;
    }
    auto* report = reinterpret_cast<InputReportXbox*>(buffer);

    memset(report, 0, sizeof(InputReportXbox));

    auto conv_stick = [](int32_t stick_in) -> uint16_t {
        int32_t val = (stick_in <= -32767) ? -32768 : stick_in;
        return static_cast<uint16_t>(val + 32768);
    };

    report->lx  = conv_stick(state.axes.stick_lx);
    report->ly  = conv_stick(state.axes.stick_ly);
    report->rx  = conv_stick(state.axes.stick_rx);
    report->ry  = conv_stick(state.axes.stick_ry);

    report->lt = (state.axes.trigger_lt >> 6) & 0x03FF;
    report->rt = (state.axes.trigger_rt >> 6) & 0x03FF;

    report->hat = state.dpad == DPadState::Centered ? 0 : static_cast<uint8_t>(state.dpad) + 1;

    report->a = state.buttons.a;
    report->b = state.buttons.b;
    report->x = state.buttons.x;
    report->y = state.buttons.y;
    report->lb = state.buttons.lb;
    report->rb = state.buttons.rb;
    report->ls = state.buttons.ls;
    report->rs = state.buttons.rs;

    report->back  = state.buttons.back;
    report->start = state.buttons.start;
    report->guide = state.buttons.home;
    report->share = state.buttons.vibr;

    return sizeof(InputReportXbox);
}

VibrationData ReportCodecXbox::decode_output(uint8_t report_id, const uint8_t* report_buffer, uint16_t report_len) const {
    if (report_id != 3) {
        LOG_WRN("Only report id 3 is supported but %d is requested, ignoring", report_id);
        return { 0, 0 };
    }
    if (report_len != 8) {
        LOG_WRN("Output report id 3 must be 8 bytes long but %d is provided, ignoring", report_len);
        return { 0, 0 };
    }
    auto* report = reinterpret_cast<const OutputReportXbox*>(report_buffer);
    LOG_DBG("en_act: %01X mot1: %02X mot2: %02X mot3: %02X mot4: %02X dur %02X dly %02X cnt %02X",
        report->enable_actuators,
        report->motor_magnitude[0],
        report->motor_magnitude[1],
        report->motor_magnitude[2],
        report->motor_magnitude[3],
        report->duration,
        report->start_delay,
        report->loop_count
    );
    VibrationData ret_data;
    ret_data.big_motor = (report->enable_actuators & (1 << 0)) ? report->motor_magnitude[2] : 0;
    ret_data.big_motor = (report->enable_actuators & (1 << 1)) ? report->motor_magnitude[3] : 0;
    return ret_data;
}

#include <zephyr/logging/log.h>

#include "GamepadState.hpp"
#include "ReportCodec/ReportCodecXbox.hpp"

LOG_MODULE_REGISTER(ReportCodecXbox, LOG_LEVEL_WRN);

uint8_t ReportCodecXbox::encode_input(uint8_t report_id, const GamepadState& data, uint8_t* report_buffer, uint16_t report_len) const {
    if (report_id != 1) {
        LOG_WRN("Only report id 1 is supported but %d is requested, ignoring", report_id);
        return 0;
    }
    if (report_len < 16)
    {
        LOG_WRN("Buffer size of at least 16 is required but %d is provided, ignoring", report_len);
        return 0;
    }
    auto* report = reinterpret_cast<InputReportXbox*>(report_buffer);

    memset(report, 0, sizeof(InputReportXbox));

    auto conv_stick = [](int32_t stick_in) -> uint16_t {
        int32_t val = (stick_in <= -32767) ? -32768 : stick_in;
        return static_cast<uint16_t>(val + 32768);
    };

    report->lx  = conv_stick(data.axes.stick_lx);
    report->ly  = conv_stick(data.axes.stick_ly);
    report->rx  = conv_stick(data.axes.stick_rx);
    report->ry  = conv_stick(data.axes.stick_ry);

    report->lt = (data.axes.trigger_lt >> 6) & 0x03FF;
    report->rt = (data.axes.trigger_rt >> 6) & 0x03FF;

    report->hat = data.dpad == DPadState::Centered ? 0 : static_cast<uint8_t>(data.dpad) + 1;

    report->a = data.buttons.a;
    report->b = data.buttons.b;
    report->x = data.buttons.x;
    report->y = data.buttons.y;
    report->lb = data.buttons.lb;
    report->rb = data.buttons.rb;
    report->ls = data.buttons.ls;
    report->rs = data.buttons.rs;

    report->back  = data.buttons.back;
    report->start = data.buttons.start;
    report->guide = data.buttons.home;
    report->share = data.buttons.vibr;

    return sizeof(InputReportXbox);
}

uint8_t ReportCodecXbox::decode_output(uint8_t report_id, VibrationDataXbox& data, const uint8_t* report_buffer, uint16_t report_len) const {
    if (report_id != 3) {
        LOG_WRN("Only report id 3 is supported but %d is requested, ignoring", report_id);
        return 0;
    }
    if (report_len != 8) {
        LOG_WRN("Output report id 3 must be 8 bytes long but %d is provided, ignoring", report_len);
        return 0;
    }
    auto* report = reinterpret_cast<const VibrationDataXbox*>(report_buffer);
    data = *report;
    return sizeof(VibrationDataXbox);
}

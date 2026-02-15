#include <string.h>
#include <zephyr/logging/log.h>

#include "GamepadState.hpp"
#include "ReportCodec/ReportCodec8BitDo.hpp"

LOG_MODULE_REGISTER(ReportCodec8BitDo, LOG_LEVEL_WRN);

int ReportCodec8BitDo::encode_input(uint8_t& report_id, const GamepadState& data, uint8_t* report_buffer, uint16_t buffer_len) const {
    if (buffer_len < 33)
    {
        LOG_WRN("Buffer size of at least 33 is required but %d is provided, ignoring", buffer_len);
        return -ENOMEM;
    }

    report_id = 1;

    auto* report = reinterpret_cast<InputReport8BitDo*>(report_buffer);
    memset(report, 0, sizeof(InputReport8BitDo));

    report->dpad = data.dpad == DPadState::Centered ? 0xF : static_cast<uint8_t>(data.dpad);

    auto conv_stick = [](int32_t stick_in) -> uint8_t {
        int32_t val = (stick_in <= -32767) ? -32768 : stick_in;
        return static_cast<uint8_t>((val >> 8) + 128);
    };

    report->lx  = conv_stick(data.axes.stick_lx);
    report->ly  = conv_stick(data.axes.stick_ly);
    report->rx  = conv_stick(data.axes.stick_rx);
    report->ry  = conv_stick(data.axes.stick_ry);

    report->lt = (uint8_t)(data.axes.trigger_lt >> 8);
    report->rt = (uint8_t)(data.axes.trigger_rt >> 8);

    report->a = data.buttons.a;
    report->b = data.buttons.b;
    report->pl = data.buttons.vibr;
    report->x = data.buttons.x;
    report->y = data.buttons.y;
    report->lb = data.buttons.lb;
    report->rb = data.buttons.rb;

    report->lt_btn = report->lt >= 0x4e;
    report->rt_btn = report->rt >= 0x4e;
    report->view = data.buttons.back;
    report->menu = data.buttons.start;
    report->home = data.buttons.home;
    report->ls = data.buttons.ls;
    report->rs = data.buttons.rs;

    report->vendor_zero = 0;
    report->const12 = 0x12;
    report->const60 = 0x60;
    report->battery = (uint8_t)((uint16_t)data.battery_percent * 255 / 100);

    report->accel[0] = data.imu_data.accel[0];
    report->accel[1] = -data.imu_data.accel[1];
    report->accel[2] = -data.imu_data.accel[2];
    report->gyro[0] = data.imu_data.gyro[0];
    report->gyro[1] = -data.imu_data.gyro[1];
    report->gyro[2] = -data.imu_data.gyro[2];

    return sizeof(InputReport8BitDo);
}

int ReportCodec8BitDo::decode_output(uint8_t report_id, VibrationDataXbox& data, const uint8_t* report_buffer, uint16_t report_len) const {
    if (report_id != 5) {
        LOG_WRN("Only report id 5 is supported but %d is requested, ignoring", report_id);
        return -ENOTSUP;
    }
    if (report_len != 4) {
        LOG_WRN("Output report id 5 must be 4 bytes long but %d is provided, ignoring", report_len);
        return -EBADMSG;
    }
    auto* report = reinterpret_cast<const OutputReport8BitDo*>(report_buffer);
    data.enable_right_motor = 1;
    data.enable_left_motor = 1;
    data.enable_right_trigger = 0;
    data.enable_left_trigger = 0;
    data.magnitude_left_trigger = 0;
    data.magnitude_right_trigger = 0;
    data.magnitude_left_motor = (uint8_t)((uint16_t)(report->motor_big) * 100 / 255);
    data.magnitude_right_motor = (uint8_t)((uint16_t)(report->motor_small) * 100 / 255);
    data.duration = 0xFF;
    data.start_delay = 0x0;
    data.loop_count = 0xFF;
    return sizeof(OutputReport8BitDo);;
}

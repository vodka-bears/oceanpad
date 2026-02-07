#include "HardwareManager/InputProcessor/InputProcessor.hpp"

int InputProcessor::init() {
    int err = 0;
    err = axes_calibrator.init();
    if (err) {
        return err;
    }
    return 0;
}

void InputProcessor::process_raw_data(GamepadState& gamepad_state, const RawData& raw_data) {
    button_processor.process_raw_buttons(gamepad_state.buttons, gamepad_state.dpad, raw_data.native_buttons, raw_data.expander_buttons);
    axes_calibrator.process_raw_axes(gamepad_state.axes, raw_data.raw_axes);
    if (axes_calibrator.is_calibration()) {
        if (raw_data.native_buttons.home)
        {
            axes_calibrator.save_calibration();
            reboot_after_calibration_flag = true;
        } else if (raw_data.native_buttons.back) {
            reboot_after_calibration_flag = true;
        }
    }
    if (raw_data.native_buttons.xd_switch) {
        imu_calibrator.process_raw_imu(gamepad_state.imu_data, raw_data.raw_imu);
    }
}

bool InputProcessor::is_axes_calibration() {
    return axes_calibrator.is_calibration();
}

void InputProcessor::start_axes_calibration() {
    axes_calibrator.start_calibration();
}

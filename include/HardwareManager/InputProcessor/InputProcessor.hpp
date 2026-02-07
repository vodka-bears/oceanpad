#pragma once

#include "GamepadState.hpp"
#include "HardwareManager/HardwareTypes.hpp"
#include "HardwareManager/InputProcessor/ButtonProcessor.hpp"
#include "HardwareManager/InputProcessor/AxesCalibrator.hpp"
#include "HardwareManager/InputProcessor/IMUCalibrator.hpp"

class InputProcessor final {
public:
    int init();
    void process_raw_data(GamepadState& gamepad_state, const RawData& raw_data);

    bool is_axes_calibration();
    void start_axes_calibration();
    bool is_ready_to_reboot_after_calibration() const { return reboot_after_calibration_flag; }
private:
    ButtonProcessor button_processor;
    AxesCalibrator axes_calibrator;
    IMUCalibrator imu_calibrator;

    bool reboot_after_calibration_flag{ false };
};

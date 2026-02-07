#pragma once

#include "GamepadState.hpp"

class IMUCalibrator final {
public:
    void process_raw_imu(IMUData& imu_data, const IMUData& raw_imu);
};

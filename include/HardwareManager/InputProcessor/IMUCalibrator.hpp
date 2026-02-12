#pragma once

#include "GamepadState.hpp"
#include <zephyr/kernel.h>
#include <stdlib.h>

class IMUCalibrator final {
public:
    void process_raw_imu(IMUData& imu_data, const IMUData& raw_imu);
    void start_calibration();
    bool is_calibration() const { return calib_count > 0; }

private:
    int32_t gyro_bias[3] = {0, 0, 0};
    uint16_t calib_count = 0;
    int32_t temp_sums[3] = {0, 0, 0};

    static constexpr int16_t STABILITY_THRESHOLD = 25;
    static constexpr uint16_t CALIB_SAMPLES = 256;
};

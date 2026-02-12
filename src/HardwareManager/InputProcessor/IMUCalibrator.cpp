#include "HardwareManager/InputProcessor/IMUCalibrator.hpp"

void IMUCalibrator::process_raw_imu(IMUData& imu_data, const IMUData& raw_imu) {
    if (calib_count > 0) {
        uint16_t samples_collected = CALIB_SAMPLES - calib_count;

        if (samples_collected > 10) {
            bool motion_detected = false;
            for (int i = 0; i < 3; i++) {
                int32_t current_avg = temp_sums[i] / samples_collected;
                if (abs(raw_imu.gyro[i] - (int16_t)current_avg) > STABILITY_THRESHOLD) {
                    motion_detected = true;
                    break;
                }
            }

            if (motion_detected) {
                /* Restart calibration window if device moved */
                start_calibration();
                return;
            }
        }

        /* Accumulate samples */
        for (int i = 0; i < 3; i++) {
            temp_sums[i] += raw_imu.gyro[i];
        }

        calib_count--;

        if (calib_count == 0) {
            /* Calibration complete: finalize bias using bit shift */
            for (int i = 0; i < 3; i++) {
                gyro_bias[i] = temp_sums[i] >> 8;
            }
        }

        /* Report zero gyro while calibrating, but keep raw accel */
        imu_data = raw_imu;
        for (int i = 0; i < 3; i++) {
            imu_data.gyro[i] = 0;
        }
    } else {
        /* Normal operation: apply bias to gyro, copy accel as-is */
        for (int i = 0; i < 3; i++) {
            imu_data.gyro[i] = raw_imu.gyro[i] - (int16_t)gyro_bias[i];
            imu_data.accel[i] = raw_imu.accel[i];
        }
    }
}

void IMUCalibrator::start_calibration() {
    for (int i = 0; i < 3; i++) {
        temp_sums[i] = 0;
    }
    calib_count = CALIB_SAMPLES;
}

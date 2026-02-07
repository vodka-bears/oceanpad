#include "HardwareManager/InputProcessor/IMUCalibrator.hpp"

void IMUCalibrator::process_raw_imu(IMUData& imu_data, const IMUData& raw_imu) {
    imu_data = raw_imu; //stub for now
}

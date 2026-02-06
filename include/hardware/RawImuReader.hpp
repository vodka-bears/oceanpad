#pragma once

#include <zephyr/drivers/i2c.h>

#include "hardware/HardwareTypes.hpp"

class RawImuReader final {
public:
    int init();
    int deinit();
    int getRawData(IMUData & raw_imu);
private:
    static constexpr i2c_dt_spec imu_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(imu));
};

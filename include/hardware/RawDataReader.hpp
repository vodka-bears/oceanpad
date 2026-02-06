#pragma once

#include <zephyr/device.h>

#include "hardware/HardwareTypes.hpp"
#include "hardware/NativeButtonsReader.hpp"
#include "hardware/ExpanderButtonsReader.hpp"
#include "hardware/RawAxesReader.hpp"
#include "hardware/RawImuReader.hpp"

class RawDataReader final {
public:
    int init();
    int deinit();
    int getRawData(RawData & raw_data);
private:
    bool imu_enabled{ false };
    NativeButtonsReader native_buttons_reader;
    ExpanderButtonsReader expander_buttons_reader;
    RawAxesReader raw_axes_reader;
    RawImuReader imu_reader;
};

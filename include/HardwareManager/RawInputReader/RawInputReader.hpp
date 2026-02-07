#pragma once

#include <zephyr/device.h>

#include "HardwareManager/HardwareTypes.hpp"
#include "HardwareManager/RawInputReader/NativeButtonsReader.hpp"
#include "HardwareManager/RawInputReader/ExpanderButtonsReader.hpp"
#include "HardwareManager/RawInputReader/RawAxesReader.hpp"
#include "HardwareManager/RawInputReader/RawImuReader.hpp"

class RawInputReader final {
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

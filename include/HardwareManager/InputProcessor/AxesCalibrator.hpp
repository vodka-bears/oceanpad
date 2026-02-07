#pragma once

#include <zephyr/settings/settings.h>

#include "GamepadState.hpp"
#include "HardwareManager/HardwareTypes.hpp"

struct StickCalib {
    uint16_t min;
    uint16_t max;
    uint16_t center;
};

struct TriggerCalib {
    uint16_t min;
    uint16_t max;
};

struct CalibData {
    StickCalib lx;
    StickCalib ly;
    StickCalib rx;
    StickCalib ry;

    TriggerCalib lt;
    TriggerCalib rt;
};

class AxesCalibrator final {
public:
    AxesCalibrator();
    int init();
    void process_raw_axes(AnalogAxes& analog_axes, const RawAxes& raw_axes);
    static int settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg);

    bool is_calibration() const { return calibration; }
    void start_calibration();
    void save_calibration();
private:
    void process_calibration(const RawAxes& raw_axes);

    static void get_calibrated_axes(AnalogAxes& axes, const RawAxes& raw_axes, const CalibData& calib_data);
    static int16_t get_calibrated_stick(const uint16_t raw_stick, const StickCalib& stick_calib, uint16_t center_dz, uint16_t edge_dz, bool is_inverted);
    static uint16_t get_calibrated_trigger(const uint16_t raw_trigger, const TriggerCalib& trigger_calib, uint16_t edge_dz);

    CalibData calib_data;
    bool calibration{ false };
    bool capture_center_flag{ false };

    static inline const uint16_t CENTER_DEADZONE = 65;
    static inline const uint16_t EDGE_DEADZONE = 40;
};

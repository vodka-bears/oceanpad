#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/settings/settings.h>

#include "GamepadState.hpp"
#include "HardwareManager/RawInputReader/RawInputReader.hpp"
#include "HardwareManager/LedBlinker.hpp"
#include "HardwareManager/BatteryGauge.hpp"

class HardwareManager {
public:
    HardwareManager();

    int init();
    int update();

    GamepadState get_state_copy();

    void set_vibration(VibrationData vibr_d);

    void set_led(bool is_on);
    void set_led(const LedPwmParams& led_params);

    uint8_t get_battery_percent();

    void start_calibration();
    bool is_calibration();

    void restart();
    void sleep();

    static int settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg);

private:
    void copy_expander();
    void copy_native_buttons();
    void copy_analog();

    void calibration_routine();
    void save_calibration();

    static AnalogAxes get_calibrated_axes(const RawAxes& raw_axes, const CalibData& calib_data);
    static int16_t get_calibrated_stick(const uint16_t raw_stick, const StickCalib& stick_calib, uint16_t center_dz, uint16_t edge_dz, bool is_inverted);
    static uint16_t get_calibrated_trigger(const uint16_t raw_trigger, const TriggerCalib& trigger_calib, uint16_t edge_dz);

    static DPadState get_dpad_state(uint8_t mask);

    static constexpr struct pwm_dt_spec motor_big = PWM_DT_SPEC_GET(DT_NODELABEL(motor_big));
    static constexpr struct pwm_dt_spec motor_small = PWM_DT_SPEC_GET(DT_NODELABEL(motor_small));

    static constexpr struct gpio_dt_spec axes_pwr = GPIO_DT_SPEC_GET(DT_NODELABEL(axes_pwr), gpios);

    struct k_mutex data_mutex;

    BatteryGauge battery_gauge;

    RawInputReader raw_input_reader;

    LedBlinker led_blinker;
    bool ignore_led{ false };

    GamepadState input_state;

    RawData raw_data;

    CalibData calib_data;

    CalibState calib_state;

    static inline const uint8_t MIN_MOTOR_PERCENT = 50;
    static inline const uint16_t CENTER_DEADZONE = 65;
    static inline const uint16_t EDGE_DEADZONE = 40;

    static HardwareManager* instance;
};




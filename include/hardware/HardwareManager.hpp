#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/settings/settings.h>

#include "GamepadState.hpp"
#include "hardware/RawDataReader.hpp"

class HardwareManager {
public:
    HardwareManager();

    int init();
    int update();

    GamepadState get_state_copy();

    void set_vibration(VibrationData vibr_d);

    void set_led(bool is_on);

    uint8_t get_battery_percent();

    void start_calibration();
    bool is_calibration();

    void restart();
    void sleep();

    static int settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg);

    void debug_print_raw();

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
    static constexpr struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(DT_NODELABEL(status_led), gpios);

    static constexpr adc_dt_spec vbat = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 6);

    struct k_mutex data_mutex;

    RawDataReader raw_data_reader;

    GamepadState input_state;

    RawData raw_data;

    CalibData calib_data;

    CalibState calib_state;

    /* Device handles */
    const struct device* adc_dev;

    static inline const uint8_t MIN_MOTOR_PERCENT = 50;
    static inline const uint16_t CENTER_DEADZONE = 65;
    static inline const uint16_t EDGE_DEADZONE = 40;

    static inline const uint32_t VBAT_SCALE_MUL = 201;
    static inline const uint32_t VBAT_SCALE_DIV = 100;
    static inline const uint32_t BAT_MV_100 = 2900;
    static inline const uint32_t BAT_MV_0 = 2200;

    static HardwareManager* instance;
};




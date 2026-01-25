#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/settings/settings.h>
#include "GamepadState.hpp"

struct NativeButtons {
    uint8_t lb : 1;
    uint8_t rb : 1;
    uint8_t ls : 1;
    uint8_t rs : 1;

    uint8_t start : 1;
    uint8_t back  : 1;
    uint8_t home  : 1;
    uint8_t switch_xd   : 1;
};

struct __packed ExpanderButtons {
    uint8_t dpad_left   : 1;
    uint8_t dpad_down   : 1;
    uint8_t dpad_up     : 1;
    uint8_t pad1        : 1;
    uint8_t dpad_right  : 1;
    uint8_t mode        : 1;
    uint8_t vibr        : 1;
    uint8_t pad2        : 1;

    uint8_t a           : 1;
    uint8_t x           : 1;
    uint8_t y           : 1;
    uint8_t b           : 1;
    uint8_t pad3        : 4;
};

struct RawAxes {
    uint16_t raw_lx;
    uint16_t raw_ly;
    uint16_t raw_rx;
    uint16_t raw_ry;
    uint16_t raw_lt;
    uint16_t raw_rt;
};

struct RawData {
    NativeButtons native_buttons;
    ExpanderButtons expander_buttons;
    RawAxes raw_axes;
    IMUData raw_imu;
};

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

enum class CalibState : uint8_t {
    Idle,
    WaitingForCenter,
    MeasuringLimits,
    Saving
};

class HardwareManager {
public:
    HardwareManager();

    int init();
    void update_phys();
    void update_imu();

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
    int setup_expander();
    int setup_imu();

    void update_expander();
    void update_native_buttons();
    void update_analog();

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
    static constexpr gpio_dt_spec exp_reset = GPIO_DT_SPEC_GET(DT_NODELABEL(expander_reset), gpios);

    struct NatvieButtonsPins {
         gpio_dt_spec lb ;
         gpio_dt_spec rb;
         gpio_dt_spec ls;
         gpio_dt_spec rs;
         gpio_dt_spec start;
         gpio_dt_spec back;
         gpio_dt_spec home;
    };

    static constexpr NatvieButtonsPins native_pins {
        .lb = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_lb), gpios),
        .rb = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_rb), gpios),
        .ls = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_ls), gpios),
        .rs = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_rs), gpios),
        .start = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_start), gpios),
        .back = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_back), gpios),
        .home = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_home), gpios),
    };

    static constexpr gpio_dt_spec xd_switch = GPIO_DT_SPEC_GET(DT_NODELABEL(mode_switch), gpios);

    struct AxesChannels {
        adc_dt_spec lx;
        adc_dt_spec ly;
        adc_dt_spec rx;
        adc_dt_spec ry;
        adc_dt_spec lt;
        adc_dt_spec rt;
        adc_dt_spec vbat;
    };

    static constexpr AxesChannels axes_channels {
        .lx = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0),
        .ly = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1),
        .rx = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2),
        .ry = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3),
        .lt = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 4),
        .rt = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 5),
        .vbat = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 6)
    };

    struct k_mutex data_mutex;

    GamepadState input_state;

    RawData raw_data;

    AnalogAxes axes;

    CalibData calib_data;

    CalibState calib_state;

    /* Device handles */
    const struct device* i2c_bus;
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




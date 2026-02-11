#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "GamepadState.hpp"
#include "HardwareManager/RawInputReader/RawInputReader.hpp"
#include "HardwareManager/InputProcessor/InputProcessor.hpp"
#include "HardwareManager/LedBlinker.hpp"
#include "HardwareManager/BatteryGauge.hpp"
#include "HardwareManager/MotorVibrator.hpp"

enum class LedPattern : uint8_t {
    Connected,
    AdvertisingUndiscoverable,
    AdvertisingDiscoverable,
};

class HardwareManager final {
public:
    HardwareManager();

    int init();
    int update();

    void get_state(GamepadState& state);

    void set_vibration(const VibrationDataXbox& vibr_d);

    void set_led(LedPattern led_pattern);

    uint8_t get_battery_percent();

    void start_calibration();
    bool is_calibration();

    void restart();
    void sleep();

private:
    struct k_mutex data_mutex;

    static constexpr struct gpio_dt_spec axes_pwr = GPIO_DT_SPEC_GET(DT_NODELABEL(axes_pwr), gpios);

    BatteryGauge battery_gauge;

    InputProcessor input_processor;
    RawInputReader raw_input_reader;

    MotorVibrator motor_vibrator;
    LedBlinker led_blinker;
    bool ignore_led{ false };
    bool shutdown{ false };

    GamepadState gamepad_state;
    RawData raw_data;

    static inline const uint16_t MIN_VOLTAGE = 2150;

    static constexpr LedPwmParams LED_SEQ_CALIB {
        .min_brightness = 0,
        .max_brightness = 255,
        .rise_ms = 0,
        .hold_ms = 0,
        .fall_ms = 333,
        .pulse_delay_ms = 0,
        .burst_delay_ms = 500,
        .pulses_per_burst = 3,
    };

    static constexpr LedPwmParams LED_SEQ_ADV_DISCO {
        .min_brightness = 0,
        .max_brightness = 255,
        .rise_ms = 200,
        .hold_ms = 0,
        .fall_ms = 200,
        .pulse_delay_ms = 0,
        .burst_delay_ms = 0,
        .pulses_per_burst = 1,
    };

    static constexpr LedPwmParams LED_SEQ_ADV_UNDISCO {
        .min_brightness = 0,
        .max_brightness = 255,
        .rise_ms = 200,
        .hold_ms = 0,
        .fall_ms = 200,
        .pulse_delay_ms = 0,
        .burst_delay_ms = 300,
        .pulses_per_burst = 2,
    };
};




#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>

#include "GamepadState.hpp"

struct MotorSequenceParams {
    uint8_t magnitude; //0 - 100
    uint8_t duration; // in 10ms units
    uint8_t delay; // in 10ms units, applied first
    uint8_t count;  // add one when running! 0 means do once
};

class SingleMotorVibrator final {
public:
    int init(const pwm_dt_spec* pwm_ptr);
    void start_sequence(const MotorSequenceParams& params);
    void stop();
private:
    const pwm_dt_spec* pwm_channel = nullptr;
    MotorSequenceParams current_params;
    uint8_t sequence_counter = 0;
    bool is_running{ false };

    struct k_work_delayable work;
    static void work_handler(struct k_work* work_ptr);
    void process();
    void apply_pwm(uint8_t percent);

    static inline const uint8_t MIN_MOTOR_PERCENT = 50;
};

class MotorVibrator final {
public:
    int init();
    void apply_vibration(const VibrationDataXbox& vibration_data);
    void stop();
private:
    static constexpr struct pwm_dt_spec motor_big = PWM_DT_SPEC_GET(DT_NODELABEL(motor_big));
    static constexpr struct pwm_dt_spec motor_small = PWM_DT_SPEC_GET(DT_NODELABEL(motor_small));

    SingleMotorVibrator left_motor_vibrator;
    SingleMotorVibrator right_motor_vibrator;

    bool is_init{ false };
};

#include "HardwareManager/MotorVibrator.hpp"

int SingleMotorVibrator::init(const pwm_dt_spec* pwm_ptr) {
    if (!pwm_is_ready_dt(pwm_ptr))
    {
        return -ENOMEM;
    }
    pwm_channel = pwm_ptr;
    k_work_init_delayable(&work, work_handler);
    apply_pwm(0);
    return 0;
}

void SingleMotorVibrator::start_sequence(const MotorSequenceParams& params) {
    stop();
    current_params = params;
    if (params.magnitude == 0) {
        stop();
    }
    else
    {
        k_work_reschedule(&work, K_MSEC(current_params.delay  * 10));
    }
}

void SingleMotorVibrator::stop() {
    sequence_counter = 0;
    is_running = false;
    k_work_cancel_delayable(&work);
    apply_pwm(0);
}

void SingleMotorVibrator::work_handler(struct k_work* work_ptr) {
    auto* dwork = k_work_delayable_from_work(work_ptr);
    auto* obj = CONTAINER_OF(dwork, SingleMotorVibrator, work);
    obj->process();
}

void SingleMotorVibrator::process() {
    if (!is_running)
    {
        apply_pwm(MIN(current_params.magnitude, 100));
        is_running = true;
        k_work_reschedule(&work, K_MSEC(current_params.duration  * 10));
    } else {
        apply_pwm(0);
        is_running = false;
        if (sequence_counter == current_params.count) {
            k_work_cancel_delayable(&work);
        } else {
            sequence_counter++;
            k_work_reschedule(&work, K_MSEC(current_params.delay  * 10));
        }
    }
}

void SingleMotorVibrator::apply_pwm(uint8_t percent) {
    if (percent == 0)
    {
        pwm_set_pulse_dt(pwm_channel, 0);
        return;
    }

    const uint8_t pct = MIN(percent, 100);

    uint32_t scaled_pct = MIN_MOTOR_PERCENT + (static_cast<uint32_t>(pct) * (100 - MIN_MOTOR_PERCENT) / 100);

    const uint32_t pulse = pwm_channel->period * scaled_pct / 100;
    pwm_set_pulse_dt(pwm_channel, pulse);
}

int MotorVibrator::init() {
    int err = 0;
    err = left_motor_vibrator.init(&motor_big);
    if (err) {
        return err;
    }
    err = right_motor_vibrator.init(&motor_small);
    if (err) {
        return err;
    }
    return 0;
}

void MotorVibrator::apply_vibration(const VibrationDataXbox& vibration_data) {
    if (vibration_data.enable_left_motor) {
        left_motor_vibrator.start_sequence({
            vibration_data.magnitude_left_motor,
            vibration_data.duration,
            vibration_data.start_delay,
            vibration_data.loop_count,
        });
    }
    if (vibration_data.enable_right_motor) {
        right_motor_vibrator.start_sequence({
            vibration_data.magnitude_right_motor,
            vibration_data.duration,
            vibration_data.start_delay,
            vibration_data.loop_count,
        });
    }
}

void MotorVibrator::stop() {
    left_motor_vibrator.stop();
    right_motor_vibrator.stop();
}

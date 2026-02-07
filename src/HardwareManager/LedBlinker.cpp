#include "HardwareManager/LedBlinker.hpp"

// Static LUT for 8-bit perceptual brightness correction
static const uint8_t brightness_lut[256] = {
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,
      1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,
      3,   3,   3,   3,   3,   4,   4,   4,   4,   5,   5,   5,   5,   6,   6,   6,
      6,   7,   7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  11,  11,  11,  12,
     12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,
     20,  20,  21,  22,  22,  23,  23,  24,  25,  25,  26,  26,  27,  28,  28,  29,
     30,  30,  31,  32,  33,  33,  34,  35,  35,  36,  37,  38,  39,  39,  40,  41,
     42,  43,  43,  44,  45,  46,  47,  48,  49,  49,  50,  51,  52,  53,  54,  55,
     56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,
     73,  74,  75,  76,  77,  78,  79,  81,  82,  83,  84,  85,  87,  88,  89,  90,
     91,  93,  94,  95,  97,  98,  99, 100, 102, 103, 105, 106, 107, 109, 110, 111,
    113, 114, 116, 117, 119, 120, 121, 123, 124, 126, 127, 129, 130, 132, 133, 135,
    137, 138, 140, 141, 143, 145, 146, 148, 149, 151, 153, 154, 156, 158, 159, 161,
    163, 165, 166, 168, 170, 172, 173, 175, 177, 179, 181, 182, 184, 186, 188, 190,
    192, 194, 196, 197, 199, 201, 203, 205, 207, 209, 211, 213, 215, 217, 219, 221,
    223, 225, 227, 229, 231, 234, 236, 238, 240, 242, 244, 246, 248, 251, 253, 255,
};


int LedBlinker::init() {
    if (!pwm_is_ready_dt(&led_pwm))
    {
        return -ENOMEM;
    }
    k_work_init_delayable(&work, work_handler);
    apply_pwm(0);
    return 0;
}

void LedBlinker::start_sequence(const LedPwmParams& params) {
    current_params = params;
    pulse_counter = 0;
    transition_to(LedState::Rising);
}

void LedBlinker::set_brightness(uint8_t value) {
    current_state = LedState::Idle;
    k_work_cancel_delayable(&work);
    apply_pwm(value);
}

void LedBlinker::work_handler(struct k_work* work_ptr) {
    auto* dwork = k_work_delayable_from_work(work_ptr);
    auto* obj = CONTAINER_OF(dwork, LedBlinker, work);
    obj->process();
}

void LedBlinker::apply_pwm(uint8_t value) {
    const uint32_t pulse = led_pwm.period * brightness_lut[value] / 255;
    pwm_set_pulse_dt(&led_pwm, pulse);
}

void LedBlinker::transition_to(LedState next_state) {
    current_state = next_state;
    state_start_ms = k_uptime_get_32();

    uint32_t delay_ms = 0;
    uint8_t static_brightness = 0;
    bool is_static = false;

    switch (current_state) {
        case LedState::Rising:
            if (current_params.rise_ms > 0) {
                delay_ms = UPDATE_INTERVAL_MS;
                apply_pwm(current_params.min_brightness);
            }
            break;

        case LedState::Holding:
            is_static = true;
            delay_ms = current_params.hold_ms;
            static_brightness = current_params.max_brightness;
            break;

        case LedState::Falling:
            if (current_params.fall_ms > 0) {
                delay_ms = UPDATE_INTERVAL_MS;
                apply_pwm(current_params.max_brightness);
            }
            break;

        case LedState::PulseGap:
            is_static = true;
            delay_ms = current_params.pulse_delay_ms;
            static_brightness = current_params.min_brightness;
            break;

        case LedState::BurstGap:
            is_static = true;
            delay_ms = current_params.burst_delay_ms;
            static_brightness = current_params.min_brightness;
            break;

        default: return;
    }

    // If it's a static phase (Hold or Gap), set brightness once and sleep until the end
    if (is_static) {
        apply_pwm(static_brightness);
    }

    // Optimization: If delay is 0, don't reschedule, just jump to the next state immediately
    if (delay_ms == 0) {
        process(); // This will trigger the next state logic
    } else {
        k_work_reschedule(&work, K_MSEC(delay_ms));
    }
}

uint8_t LedBlinker::calculate_fading(uint32_t elapsed, uint32_t duration, bool rising) {
    if (duration == 0) return rising ? current_params.max_brightness : current_params.min_brightness;

    int32_t diff = current_params.max_brightness - current_params.min_brightness;
    uint32_t progress = (elapsed * diff) / duration;

    if (rising) {
        return (uint8_t)MIN(current_params.min_brightness + progress, current_params.max_brightness);
    } else {
        return (uint8_t)MAX((int32_t)current_params.max_brightness - (int32_t)progress, (int32_t)current_params.min_brightness);
    }
}

void LedBlinker::process() {
    if (current_state == LedState::Idle) {
        return;
    }

    uint32_t now = k_uptime_get_32();
    uint32_t elapsed = now - state_start_ms;
    bool phase_finished = false;

    // 1. Handle dynamic Fading
    if (current_state == LedState::Rising || current_state == LedState::Falling) {
        uint32_t duration = (current_state == LedState::Rising) ?
                             current_params.rise_ms : current_params.fall_ms;

        if (elapsed >= duration) {
            phase_finished = true;
        } else {
            uint8_t brightness = calculate_fading(elapsed, duration, current_state == LedState::Rising);
            apply_pwm(brightness);
            k_work_reschedule(&work, K_MSEC(UPDATE_INTERVAL_MS));
            return; // Stay in this state
        }
    }
    // 2. Handle static phases (they only trigger process() when the timer expires)
    else {
        phase_finished = true;
    }

    // 3. State Machine Transitions
    if (phase_finished) {
        switch (current_state) {
            case LedState::Rising:
                transition_to(LedState::Holding);
                break;
            case LedState::Holding:
                transition_to(LedState::Falling);
                break;
            case LedState::Falling:
                if (++pulse_counter < current_params.pulses_per_burst) {
                    transition_to(LedState::PulseGap);
                } else {
                    pulse_counter = 0;
                    transition_to(LedState::BurstGap);
                }
                break;
            case LedState::PulseGap:
            case LedState::BurstGap:
                transition_to(LedState::Rising);
                break;
            default:
                set_brightness(0);
                break;
        }
    }
}

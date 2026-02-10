#include <zephyr/logging/log.h>

#include "HardwareManager/LedBlinker.hpp"

LOG_MODULE_REGISTER(LedBlinker, LOG_LEVEL_WRN);
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
    apply_brightness(0);
    return 0;
}

void LedBlinker::start_sequence(const LedPwmParams& params) {
    current_params = params;
    pulse_counter = 0;
    transition_to(LedState::Rising);
}

void LedBlinker::set_brightness(uint8_t value) {
    if (value == 0) {
        transition_to(LedState::Off);
    }
    current_params.max_brightness = value;
    current_params.min_brightness = value;
    transition_to(LedState::Static);
}

void LedBlinker::set_vbat(uint16_t new_vbat) {
    if (new_vbat < VOLTAGE_MIN) {
        LOG_DBG("Too low voltage %d, ignoring", new_vbat);
        return;
    }
    vbat = new_vbat;
}

void LedBlinker::apply_brightness(uint8_t value) {
    if (value == 0) {
        pwm_set_pulse_dt(&led_pwm, 0);
        return;
    }
    const uint16_t numerator = (VOLTAGE_MAX - VOLTAGE_MIN);
    const uint16_t denominator = (vbat - VOLTAGE_MIN);
    //const uint16_t numerator = 1;
    //const uint16_t denominator = 1;
    const uint64_t intermediate = (uint64_t)led_pwm.period * brightness_lut[value] * numerator;
    uint32_t pulse = (uint32_t)(intermediate / denominator / 255);
    LOG_DBG("Voltage: %4d, pulse: %5d", vbat, pulse);
    if (pulse > led_pwm.period) {
        pulse = led_pwm.period;
    }
    pwm_set_pulse_dt(&led_pwm, pulse);
}

void LedBlinker::work_handler(struct k_work* work_ptr) {
    auto* dwork = k_work_delayable_from_work(work_ptr);
    auto* obj = CONTAINER_OF(dwork, LedBlinker, work);
    obj->process();
}

void LedBlinker::process() {
    if (current_state == LedState::Off) {
        return;
    }

    uint32_t elapsed = k_uptime_get_32() - state_start_ms;

    // 1. Intra-phase logic
    switch (current_state) {
    case LedState::Static:
        apply_brightness(current_params.max_brightness);
        k_work_reschedule(&work, K_MSEC(UPDATE_INTERVAL_MS)); // 50Hz for Vbat compensation
        return;

    case LedState::Rising:
        if (current_params.rise_ms > 0 && elapsed < current_params.rise_ms) {
            apply_brightness(calculate_fading(elapsed, current_params.rise_ms, true));
            k_work_reschedule(&work, K_MSEC(UPDATE_INTERVAL_MS));
            return;
        }
        break;

    case LedState::Holding:
        if (current_params.hold_ms > 0 && elapsed < current_params.hold_ms) {
            apply_brightness(current_params.max_brightness);
            k_work_reschedule(&work, K_MSEC(UPDATE_INTERVAL_MS));
            return;
        }
        break;

    case LedState::Falling:
        if (current_params.fall_ms > 0 && elapsed < current_params.fall_ms) {
            apply_brightness(calculate_fading(elapsed, current_params.fall_ms, false));
            k_work_reschedule(&work, K_MSEC(UPDATE_INTERVAL_MS));
            return;
        }
        break;

    case LedState::PulseGap:
    case LedState::BurstGap:
    {        uint32_t gap_duration = (current_state == LedState::PulseGap) ?
            current_params.pulse_delay_ms : current_params.burst_delay_ms;

        if (gap_duration > 0 && elapsed < gap_duration) {
            apply_brightness(current_params.min_brightness);

            // Optimization: sleep until end of gap if LED is off (min_brightness == 0)
            if (current_params.min_brightness == 0) {
                k_work_reschedule(&work, K_MSEC(gap_duration - elapsed));
            } else {
                k_work_reschedule(&work, K_MSEC(UPDATE_INTERVAL_MS));
            }
            return;
        }
        break;
    }
    default: //off
        break;
    }

    // 2. Phase transition logic (triggered when current phase is complete or duration is 0)
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
        apply_brightness(0);
        break;
    }
}

void LedBlinker::transition_to(LedState next_state) {
    current_state = next_state;
    state_start_ms = k_uptime_get_32();

    uint8_t initial_br = 0;
    uint32_t next_delay_ms = UPDATE_INTERVAL_MS;
    uint32_t phase_duration = 0;

    switch (current_state) {
        case LedState::Static:
            initial_br = current_params.max_brightness;
            phase_duration = 1; // Prevent zero-length skip for Static
            break;

        case LedState::Rising:
            initial_br = current_params.min_brightness;
            phase_duration = current_params.rise_ms;
            break;

        case LedState::Holding:
            initial_br = current_params.max_brightness;
            phase_duration = current_params.hold_ms;
            break;

        case LedState::Falling:
            initial_br = current_params.max_brightness;
            phase_duration = current_params.fall_ms;
            break;

        case LedState::PulseGap:
        case LedState::BurstGap:
            initial_br = current_params.min_brightness;
            phase_duration = (current_state == LedState::PulseGap) ?
                              current_params.pulse_delay_ms : current_params.burst_delay_ms;

            // Optimization for zero brightness gaps
            if (current_params.min_brightness == 0) {
                next_delay_ms = phase_duration;
            }
            break;

        case LedState::Off:
            apply_brightness(0);
            k_work_cancel_delayable(&work);
            return;
    }

    // Apply starting brightness of the new phase immediately
    apply_brightness(initial_br);

    // Optimization: if phase duration is 0, skip directly to the next state transition
    if (phase_duration == 0) {
        process();
    } else {
        k_work_reschedule(&work, K_MSEC(next_delay_ms));
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

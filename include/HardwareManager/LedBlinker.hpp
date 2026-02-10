#pragma once

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

struct LedPwmParams {
    uint8_t  min_brightness;   // Usually 0
    uint8_t  max_brightness;   // Max PWM value (e.g., 255)
    uint16_t rise_ms;          // Time to fade in
    uint16_t hold_ms;          // Time to stay at max brightness
    uint16_t fall_ms;          // Time to fade out
    uint16_t pulse_delay_ms;   // Gap between pulses in a burst
    uint16_t burst_delay_ms;   // Gap between series of bursts
    uint8_t  pulses_per_burst; // Number of "peaks" before burst_delay
};

class LedBlinker {
public:
    enum class LedState : uint8_t { Off, Static, Rising, Holding, Falling, PulseGap, BurstGap };

    int init();
    void start_sequence(const LedPwmParams& params);
    void set_brightness(uint8_t value); // Manual override
    void set_vbat(uint16_t new_vbat);
private:
    static inline const uint32_t UPDATE_INTERVAL_MS = 20; // 50Hz update rate for smooth fading
    static inline const uint32_t VOLTAGE_MAX = 1800;
    static inline const uint32_t VOLTAGE_MIN = 1200;

    uint16_t vbat = 3000;

    struct k_work_delayable work;
    LedPwmParams current_params;

    uint32_t state_start_ms = 0;
    LedState current_state = LedState::Off;
    uint8_t  pulse_counter = 0;

    static constexpr const struct pwm_dt_spec led_pwm = PWM_DT_SPEC_GET(DT_NODELABEL(status_pwm_led));

    void apply_brightness(uint8_t value);

    static void work_handler(struct k_work* work_ptr);
    void process();
    void transition_to(LedState next_state);
    uint8_t calculate_fading(uint32_t elapsed, uint32_t duration, bool rising);
};

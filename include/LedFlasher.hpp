#pragma once
#include <zephyr/kernel.h>

/**
 * @brief Parameters for LED flashing patterns.
 */
struct LedParams {
    uint8_t  count;          // Number of flashes in a series
    uint16_t on_time_ms;     // Duration of each flash
    uint16_t off_time_ms;    // Pause between flashes within a series
    uint16_t delay_time_ms;  // Pause between series
};

/**
 * @brief Function pointer type for hardware control.
 * @param state true to turn LED on, false to turn it off.
 * @param context User-provided pointer (usually an instance of a manager class).
 */
typedef void (*LedSetStateCallback)(bool state, void* context);

class LedFlasher {
public:
    LedFlasher();

    /**
     * @brief Connects the flasher to the hardware.
     */
    void set_handler(LedSetStateCallback cb, void* context);

    /**
     * @brief Starts a flashing pattern.
     */
    void start(const LedParams& params);

    /**
     * @brief Stops flashing and turns the LED off.
     */
    void stop();

    /**
     * @brief Manual overrides (stops active flashing).
     */
    void turn_on();
    void turn_off();

private:
    struct k_work_delayable work;
    LedParams current_params;

    LedSetStateCallback handler_cb = nullptr;
    void* handler_context = nullptr;

    uint8_t current_count = 0;
    bool is_active = false;
    bool led_is_on = false;

    static void work_handler(struct k_work* work_ptr);
    void process();
    void notify(bool state);
};

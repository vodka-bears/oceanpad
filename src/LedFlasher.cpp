#include "LedFlasher.hpp"

LedFlasher::LedFlasher() {
    // Initialize the delayed work structure
    k_work_init_delayable(&work, work_handler);
}

void LedFlasher::set_handler(LedSetStateCallback cb, void* context) {
    handler_cb = cb;
    handler_context = context;
}

void LedFlasher::notify(bool state) {
    if (handler_cb) {
        handler_cb(state, handler_context);
    }
}

void LedFlasher::turn_on() {
    stop();
    notify(true);
}

void LedFlasher::turn_off() {
    stop();
    notify(false);
}

void LedFlasher::start(const LedParams& params) {
    current_params = params;
    current_count = 0;
    led_is_on = false;
    is_active = true;
    // Start the process immediately
    k_work_reschedule(&work, K_NO_WAIT);
}

void LedFlasher::stop() {
    is_active = false;
    k_work_cancel_delayable(&work);
    notify(false);
}

void LedFlasher::work_handler(struct k_work* work_ptr) {
    // Recover the class instance pointer using CONTAINER_OF
    struct k_work_delayable* dwork = k_work_delayable_from_work(work_ptr);
    LedFlasher* obj = CONTAINER_OF(dwork, LedFlasher, work);
    obj->process();
}

void LedFlasher::process() {
    if (!is_active) return;

    if (!led_is_on) {
        // Toggle ON
        notify(true);
        led_is_on = true;
        k_work_reschedule(&work, K_MSEC(current_params.on_time_ms));
    } else {
        // Toggle OFF
        notify(false);
        led_is_on = false;
        current_count++;

        if (current_count < current_params.count) {
            // Wait before next flash in the series
            k_work_reschedule(&work, K_MSEC(current_params.off_time_ms));
        } else {
            current_count = 0;
            // Wait for the long delay before repeating
            k_work_reschedule(&work, K_MSEC(current_params.delay_time_ms));
        }
    }
}

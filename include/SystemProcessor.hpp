#pragma once

#include "HardwareManager/HardwareManager.hpp"
#include "BleService.hpp"

class SystemProcessor final {
public:
    void init(HardwareManager* hw_ptr, BleService* ble_ptr);
    void process_input_logic(const GamepadState& gamepad_state);
    void process_system_logic(const GamepadState& gamepad_state);
    void on_connected();
    void on_disconnected(bool graceful);
    void on_adv_timeout(bool discoverable);
private:
    HardwareManager* hardware_manager{ nullptr };
    BleService* ble_service{ nullptr };

    void start_advertising();
    void on_advertising_discoverable_timeout();
    static bool is_state_idle(const GamepadState& gamepad_state);

    uint8_t identity_idx{ 0 };

    int64_t system_press_start = 0;
    int64_t home_press_start = 0;

    int64_t next_battery_update_time = 0;

    int64_t last_non_idle_time = 0;

    static inline const int32_t LONG_PRESS_TIMEOUT_MS = 3'000;
    static inline const int32_t BATTERY_UPDATE_PERIOD_MS = 30'000;
    static inline const int64_t IDLE_TIMEOUT_MS = 900'000;
    static inline const uint16_t AXIS_ACTIVITY_THRESHOLD = 2048;
};

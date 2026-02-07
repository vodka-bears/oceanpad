#pragma once
#include "HardwareManager/HardwareManager.hpp"
#include "ReportCodecXbox.hpp"
#include "ReportCodec8BitDo.hpp"
#include "BleService.hpp"

class OceanPadApp {
public:
    void run();

    void handle_vibration(uint8_t report_id, const uint8_t* data, uint16_t len);
private:
    static void input_thread_fn(void *arg1, void *arg2, void *arg3);
    static void system_thread_fn(void *arg1, void *arg2, void *arg3);

    void input_loop();
    void system_loop();

    void handle_system_logic();

    void start_advertising();

    void on_advertising_discoverable_timeout();

    static bool is_state_idle(const GamepadState& gp_state);

    HardwareManager hw;

    using ReportCodecOceanPad = ReportCodec<GamepadState, VibrationDataXbox>;

    const ReportCodecOceanPad* current_codec = nullptr;

    static const ReportCodecXbox xbox_codec;
    static const ReportCodec8BitDo abitdo_codec;

    BleService ble_service;

    bool xd_switch_was_on{ false };

    int64_t system_press_start = 0;
    int64_t home_press_start = 0;

    int64_t next_battery_update_time = 0;

    int64_t last_non_idle_time = 0;

    uint64_t interval_us = 7500;

    GamepadState gamepad_state;

    static inline const int32_t LONG_PRESS_TIMEOUT_MS = 3'000;
    static inline const int32_t BATTERY_UPDATE_PERIOD_MS = 30'000;
    static inline const int64_t IDLE_TIMEOUT_MS = 900'000;
    static inline const uint16_t AXIS_ACTIVITY_THRESHOLD = 2048;

    struct k_thread input_thread_data;
    struct k_thread system_thread_data;
    K_KERNEL_STACK_MEMBER(input_stack, 1024);
    K_KERNEL_STACK_MEMBER(system_stack, 2048);
};

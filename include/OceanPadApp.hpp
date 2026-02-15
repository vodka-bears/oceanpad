#pragma once
#include "HardwareManager/HardwareManager.hpp"
#include "BleService.hpp"
#include "SystemProcessor.hpp"
#include "DeviceConfig.hpp"

class OceanPadApp {
public:
    void run();

private:
    static void input_thread_fn(void *arg1, void *arg2, void *arg3);
    static void system_thread_fn(void *arg1, void *arg2, void *arg3);

    void input_loop();
    void system_loop();

    void handle_incoming_report(uint8_t report_id, const uint8_t* data, uint16_t len);

    HardwareManager hardware_manager;

    using ReportCodecOceanPad = ReportCodec<GamepadState, VibrationDataXbox>;

    const ReportCodecOceanPad* current_codec = nullptr;

    SystemProcessor system_processor;

    static BleService ble_service;
    uint64_t interval_us = 7500;

    uint8_t input_report_buffer[34];

    GamepadState gamepad_state;

    static inline const uint16_t SYSTEM_LOOP_PERIOD_MS = 250;

    struct k_thread input_thread_data;
    struct k_thread system_thread_data;
    K_KERNEL_STACK_MEMBER(input_stack, 1024);
    K_KERNEL_STACK_MEMBER(system_stack, 2048);

    class OceanPadHidCallbacks : public HidServiceCallbacks {
    public:
        virtual void on_incoming_report(uint8_t report_id, const uint8_t* data, uint16_t len) const override;
        virtual void on_connected() const override;
        virtual void on_disconnected(bool graceful) const override;
        virtual void on_adv_timeout(bool discoverable) const override;
        void set_ptr(OceanPadApp* new_ptr) { app_ptr = new_ptr; }
    private:
        OceanPadApp* app_ptr{ nullptr };
    } hid_callbacks;
};

#include "OceanPadApp.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(OceanPadApp, LOG_LEVEL_DBG);

BleService OceanPadApp::ble_service;

void OceanPadApp::run() {
    int err = 0;

    err = settings_subsys_init();
    if (err) {
        LOG_ERR("Settings subsys init failed (err %d)", err);
        k_sleep(K_FOREVER);
    }

    err = hardware_manager.init();
    if (err != 0)
    {
        LOG_ERR("Error hardware init");
        k_sleep(K_FOREVER);
    }

    hardware_manager.get_state(gamepad_state);
    uint8_t identity_idx = hardware_manager.get_identity_idx();

    //LOG_DBG("Identity index: %d", identity_idx);

    current_codec = oceanpad_config_variants[identity_idx].report_codec;
    interval_us = oceanpad_config_variants[identity_idx].interval_us;

    err = ble_service.init(&oceanpad_config_variants[identity_idx]);
    if (err) {
        LOG_ERR("BLE init failed (err %d)", err);
        k_sleep(K_FOREVER);
    }

    hid_callbacks.set_ptr(this);
    ble_service.set_callbacks(&hid_callbacks);
    err = settings_load();
    if (err) {
        LOG_ERR("Settings load failed (err %d)", err);
        k_sleep(K_FOREVER);
    }

    err = ble_service.set_identity(identity_idx);
    if (err) {
        LOG_ERR("Set identity failed (err %d)", err);
        k_sleep(K_FOREVER);
    }

    system_processor.init(&hardware_manager, &ble_service);

    k_thread_create(&input_thread_data, input_stack, K_THREAD_STACK_SIZEOF(input_stack),
                    input_thread_fn, this, NULL, NULL,
                    K_PRIO_COOP(2), 0, K_NO_WAIT);

    k_thread_create(&system_thread_data, system_stack, K_THREAD_STACK_SIZEOF(system_stack),
                    system_thread_fn, this, NULL, NULL,
                    K_PRIO_PREEMPT(10), 0, K_NO_WAIT);

    k_sleep(K_FOREVER);
}

void OceanPadApp::input_thread_fn(void *arg1, void *arg2, void *arg3) {
    auto *app = static_cast<OceanPadApp *>(arg1);

    uint64_t next_tick = k_uptime_ticks();
    const uint64_t interval_ticks = k_us_to_ticks_near64(app->interval_us);

    while (true) {
        uint64_t now = k_uptime_ticks();
        next_tick += interval_ticks;
        app->input_loop();
        now = k_uptime_ticks();
        if (next_tick <= now) {
            //LOG_DBG("late by %lld", now - next_tick);
            next_tick = now + interval_ticks;
        }
        k_sleep(K_TIMEOUT_ABS_TICKS(next_tick));
    }
}

void OceanPadApp::system_thread_fn(void *arg1, void *arg2, void *arg3) {
    auto *app = static_cast<OceanPadApp *>(arg1);
    while (true) {
        app->system_loop();
        k_msleep(SYSTEM_LOOP_PERIOD_MS);
    }
}

void OceanPadApp::input_loop() {
    hardware_manager.update();
    if (hardware_manager.is_calibration()) {
        return;
    }
    hardware_manager.get_state(gamepad_state);
    system_processor.process_input_logic(gamepad_state);

    uint8_t report_id = 0;
    int report_len = current_codec->encode_input(report_id, gamepad_state, input_report_buffer, sizeof(input_report_buffer));
    if (report_len < 0) {
        LOG_ERR("Report encoding failure, err: %d", report_len);
    } else if (report_len == 0) {
        //do nothing
    } else if (ble_service.get_state() >= BleServiceState::Connected)
    {
        ble_service.update_report(report_id, input_report_buffer, report_len);
    }
}

void OceanPadApp::system_loop() {
    GamepadState gamepad_state_copy;
    hardware_manager.get_state(gamepad_state_copy);
    system_processor.process_system_logic(gamepad_state_copy);
}

void OceanPadApp::handle_incoming_report(uint8_t report_id, const uint8_t* data, uint16_t len) {
    VibrationDataXbox vibr_data;
    if (hardware_manager.is_calibration())
    {
        return;
    }
    int data_len = current_codec->decode_output(report_id, vibr_data, data, len);
    if (data_len == 0) {
        LOG_WRN("Failed to decode output report!");
    }
    else {
        hardware_manager.set_vibration(vibr_data);
    }
}

void OceanPadApp::OceanPadHidCallbacks::on_incoming_report(uint8_t report_id, const uint8_t* data, uint16_t len) const {
    if (!app_ptr) {
        return;
    }
    app_ptr->handle_incoming_report(report_id, data, len);
}

void OceanPadApp::OceanPadHidCallbacks::on_connected() const {
    if (!app_ptr) {
        return;
    }
    app_ptr->system_processor.on_connected();
}

void OceanPadApp::OceanPadHidCallbacks::on_disconnected(bool graceful) const {
    if (!app_ptr) {
        return;
    }
    app_ptr->system_processor.on_disconnected(graceful);
}

void OceanPadApp::OceanPadHidCallbacks::on_adv_timeout(bool discoverable) const {
    if (!app_ptr) {
        return;
    }
    app_ptr->system_processor.on_adv_timeout(discoverable);
}

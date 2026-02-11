#include "OceanPadApp.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(oceanpad_app, LOG_LEVEL_DBG);

const ReportCodecXbox OceanPadApp::xbox_codec;
const ReportCodec8BitDo OceanPadApp::abitdo_codec;
BleService OceanPadApp::ble_service;


void OceanPadApp::run() {
    int err = 0;

    err = settings_subsys_init();
    if (err) {
        LOG_ERR("Settings subsys init failed (err %d)", err);
        k_sleep(K_FOREVER);
    }

    err = hw.init();
    if (err != 0)
    {
        LOG_ERR("Error hardware init");
        k_sleep(K_FOREVER);
    }


    hw.get_state(gamepad_state);
    identity_idx = hw.get_identity_idx();

    LOG_DBG("Identity index: %d", identity_idx);

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

    start_advertising();

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
        k_msleep(500);
    }

}

void OceanPadApp::input_loop() {
    hw.update();
    if (!hw.is_calibration()) {
        hw.get_state(gamepad_state);
        if (!is_state_idle(gamepad_state))
        {
            last_non_idle_time = k_uptime_get();
        }
        uint8_t report_len = current_codec->encode_input(1, gamepad_state, input_report_buffer, sizeof(input_report_buffer));

        if (ble_service.get_state() >= BleServiceState::Connected)
        {
            ble_service.update_report(1, input_report_buffer, report_len);
        }
    }
}

void OceanPadApp::system_loop() {
    hw.get_state(gamepad_state);
    handle_system_logic();
    int64_t current_time = k_uptime_get();
    if (ble_service.get_state() >= BleServiceState::Connected && current_time > next_battery_update_time)
    {
        ble_service.update_battery_level(hw.get_battery_percent());
        next_battery_update_time = current_time + BATTERY_UPDATE_PERIOD_MS;
    }
    if (current_time > last_non_idle_time + IDLE_TIMEOUT_MS)
    {
        LOG_DBG("Idle timeout, shutting down");
        hw.sleep();
    }
    if (hw.get_identity_idx() != identity_idx)
    {
        ble_service.do_disconnect();
        LOG_DBG("Identity switched, restarting");
        hw.restart();
    }
}

void OceanPadApp::handle_system_logic() {
    if (gamepad_state.buttons.mode) {
        if (system_press_start == 0) {
            system_press_start = k_uptime_get();
        } else if (system_press_start > 0 && (k_uptime_get() - system_press_start >= LONG_PRESS_TIMEOUT_MS)) {
            if (gamepad_state.buttons.b) {
                LOG_DBG("Mode+B: Calibration Mode Initialized");
                hw.start_calibration();
            } else if (gamepad_state.buttons.y) {
                LOG_DBG("Mode+Y: Clearing bonded peers");
                ble_service.clear_bonded_peers();
                hw.restart();
            } else {
                BleServiceState ble_state = ble_service.get_state();
                if (ble_state == BleServiceState::AdvertisingDiscoverable || ble_state == BleServiceState::ConnectedAdvertising) {
                    LOG_DBG("Mode Long Press: Stop advertsing discoverable");
                    ble_service.stop_advertising();
                    if (ble_service.get_state() == BleServiceState::AdvertisingUndiscoverable)
                    {
                        hw.set_led(LedPattern::AdvertisingDiscoverable);
                    }
                    else if (ble_service.get_state() == BleServiceState::Connected)
                    {
                        hw.set_led(LedPattern::Connected);
                    }
                } else {
                    LOG_DBG("Mode Long Press: Advertsing discoverable");
                    ble_service.start_advertising(true);
                    hw.set_led(LedPattern::AdvertisingDiscoverable);
                }
            }
            system_press_start = -1;
        }
    } else {
        system_press_start = 0;
    }

    if (gamepad_state.buttons.home) {
        if (home_press_start == 0) {
            home_press_start = k_uptime_get();
        } else if (home_press_start > 0 && (k_uptime_get() - home_press_start >= LONG_PRESS_TIMEOUT_MS)) {
            LOG_DBG("HOME Long Press: System OFF");
            ble_service.do_disconnect();
            hw.sleep();
            home_press_start = -1;
        }
    } else {
        home_press_start = 0;
    }
}


void OceanPadApp::start_advertising() {
    if (ble_service.has_bonded_peer())
    {
        hw.set_led(LedPattern::AdvertisingUndiscoverable);
        ble_service.start_advertising(false);
    }
    else
    {
        hw.set_led(LedPattern::AdvertisingDiscoverable);
        ble_service.start_advertising(true);
    }
}

void OceanPadApp::on_advertising_discoverable_timeout() {
    if (ble_service.get_state() >= BleServiceState::Connected)
    {
        hw.set_led(LedPattern::Connected);
    }
    else if (ble_service.has_bonded_peer()) {
        hw.set_led(LedPattern::AdvertisingUndiscoverable);
        ble_service.start_advertising(false);
    }
    else
    {
        hw.sleep();
    }
}

bool OceanPadApp::is_state_idle(const GamepadState& gp_state) {
    DigitalButtons btns_copy = gp_state.buttons;
    btns_copy.xd_switch = 0;
    if (*(uint16_t*)(&btns_copy))
    {
        //LOG_DBG("Buttons not idle");
        return false;
    }
    if (gp_state.dpad != DPadState::Centered)
    {
        //LOG_DBG("Dpad not idle");
        return false;
    }
    auto abs = [](int16_t inp) -> uint16_t {
        if (inp < 0) return -inp;
        return inp;
    };
    if (abs(gp_state.axes.stick_lx) > AXIS_ACTIVITY_THRESHOLD ||
        abs(gp_state.axes.stick_ly) > AXIS_ACTIVITY_THRESHOLD ||
        abs(gp_state.axes.stick_rx) > AXIS_ACTIVITY_THRESHOLD ||
        abs(gp_state.axes.stick_ry) > AXIS_ACTIVITY_THRESHOLD
    )
    {
        //LOG_DBG("Sticks not idle");
        return false;
    }
    if (gp_state.axes.trigger_lt > AXIS_ACTIVITY_THRESHOLD ||
        gp_state.axes.trigger_rt > AXIS_ACTIVITY_THRESHOLD
    )
    {
        //LOG_DBG("Triggers not idle");
        return false;
    }
    return true;
}

void OceanPadApp::handle_incoming_report(uint8_t report_id, const uint8_t* data, uint16_t len) {
    VibrationDataXbox vibr_data;
    if (hw.is_calibration())
    {
        return;
    }
    int data_len = current_codec->decode_output(report_id, vibr_data, data, len);
    if (data_len == 0) {
        LOG_WRN("Failed to decode output report!");
    }
    else {
        hw.set_vibration(vibr_data);
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
    app_ptr->hw.set_led(LedPattern::Connected);
}

void OceanPadApp::OceanPadHidCallbacks::on_disconnected(bool graceful) const {
    if (!app_ptr) {
        return;
    }
    if (graceful) {
        app_ptr->hw.sleep();
    } else {
        app_ptr->start_advertising();
    }
}

void OceanPadApp::OceanPadHidCallbacks::on_adv_timeout(bool discoverable) const {
    if (!app_ptr) {
        return;
    }
    if (discoverable) {
        app_ptr->on_advertising_discoverable_timeout();
    } else {
        app_ptr->hw.sleep();
    }
}

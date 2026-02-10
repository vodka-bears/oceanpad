#include "OceanPadApp.hpp"
#include "hid_report_maps.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(oceanpad_app, LOG_LEVEL_DBG);

static OceanPadApp* g_app_instance = nullptr;
const ReportCodecXbox OceanPadApp::xbox_codec;
const ReportCodec8BitDo OceanPadApp::abitdo_codec;

void vibration_callback_wrapper(uint8_t report_id, const uint8_t* data, uint16_t len) {
    if (g_app_instance) {
        g_app_instance->handle_vibration(report_id, data, len);
    }
}

void OceanPadApp::run() {

    g_app_instance = this;

    int ret = 0;

    ret = settings_subsys_init();
    if (ret) {
        LOG_ERR("Settings subsys init failed (err %d)", ret);
    }

    ret = hw.init();
    if (ret != 0)
    {
        LOG_ERR("Error hardware init");
    }


    hw.get_state(gamepad_state);
    xd_switch_was_on = gamepad_state.buttons.xd_switch;

    LOG_DBG("X-D switch is %s", xd_switch_was_on ? "on" : "off");

    //led_flasher.set_handler(led_bridge, &hw);

    if (xd_switch_was_on) {
        current_codec = &abitdo_codec;
        interval_us = 8333;
    } else {
        current_codec = &xbox_codec;
        interval_us = 7500;
    }

    static const DeviceInfo xbox_device_info {
        .name = "Xbox Wireless Controller",
        .appearance = 964,
    };

    static const DisConfig xbox_dis = {
        .pnp_id = {
            .src = 2,
            .vid = 0x045E,
            .pid = 0x0B13,
            .ver = 0x0523,
        },
        .manufacturer = "Microsoft",
        .model = NULL,
        .serial = "18062023",
        .fw_rev = "OceanPad",
    };

    static const ReportRef xbox_reports[2] = {
        { 0x01, HidReportType::Input  },
        { 0x03, HidReportType::Output  },
    };

    static const HidConfig xbox_hid = {
        .report_map = report_map_xbox,
        .report_map_size = ARRAY_SIZE(report_map_xbox),
        .report_buf = xbox_reports,
        .report_count = ARRAY_SIZE(xbox_reports),
    };

    static const DeviceInfo abitdo_device_info {
        .name = "OceanPad",
        .appearance = 964,
    };

    static const DisConfig abitdo_dis = {
        .pnp_id = {
            .src = 2,
            .vid = 0x2DC8,
            .pid = 0x6012,
            .ver = 0x0001,
        },
        .manufacturer = "Vodka Bears",
        .model = NULL,
        .serial = "18062023",
        .fw_rev = "OceanPad",
    };

    static const ReportRef abitdo_reports[2] = {
        { 0x01, HidReportType::Input  },
        { 0x05, HidReportType::Output  },
    };

    static const HidConfig abitdo_hid = {
        .report_map = report_map_8bitdo,
        .report_map_size = ARRAY_SIZE(report_map_8bitdo),
        .report_buf = abitdo_reports,
        .report_count = ARRAY_SIZE(abitdo_reports),
    };

    if (xd_switch_was_on) {
        ret = ble_service.init(&abitdo_device_info, &abitdo_dis, &abitdo_hid);
    } else {
        ret = ble_service.init(&xbox_device_info, &xbox_dis, &xbox_hid);
    }
    LOG_DBG("ble_service.init returned %d", ret);
    if (ret) {
        LOG_ERR("BLE init failed (err %d)", ret);
    }
    ble_service.set_output_report_callback(vibration_callback_wrapper);
    ble_service.set_status_callbacks(
        []() { g_app_instance->hw.set_led(LedPattern::Connected); },
        []() { g_app_instance->hw.sleep(); },
        []() { /*g_app_instance->start_advertising();*/ },
        []() { g_app_instance->on_advertising_discoverable_timeout(); },
        []() { g_app_instance->hw.sleep(); }
    );
    ret = settings_load();
    if (ret) {
        LOG_ERR("Settings load failed (err %d)", ret);
    }

    if (xd_switch_was_on) {
        ret = ble_service.set_identity(1);
    } else {

    }

    start_advertising();


    k_thread_create(&input_thread_data, input_stack, K_THREAD_STACK_SIZEOF(input_stack),
                    input_thread_fn, this, NULL, NULL,
                    K_PRIO_COOP(2), 0, K_NO_WAIT);

    k_thread_create(&system_thread_data, system_stack, K_THREAD_STACK_SIZEOF(system_stack),
                    system_thread_fn, this, NULL, NULL,
                    K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
}

void OceanPadApp::handle_vibration(uint8_t report_id, const uint8_t* data, uint16_t len) {
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
            LOG_DBG("late by %lld", now - next_tick);
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
        uint8_t input_report[34];
        uint8_t report_len = current_codec->encode_input(1, gamepad_state, input_report, sizeof(input_report));

        if (ble_service.get_state() >= BleServiceState::Connected)
        {
            ble_service.send_input_report(1, input_report, report_len);
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
    if (gamepad_state.buttons.xd_switch != xd_switch_was_on)
    {
        LOG_DBG("X-D Switch flipped, restarting");
        hw.restart();
    }
}

void OceanPadApp::handle_system_logic() {
    if (gamepad_state.buttons.mode) {
        if (system_press_start == 0) {
            system_press_start = k_uptime_get();
        } else if (system_press_start > 0 && (k_uptime_get() - system_press_start >= LONG_PRESS_TIMEOUT_MS)) {
            if (gamepad_state.buttons.b) {
                LOG_DBG("System+B: Calibration Mode Initialized");
                hw.start_calibration();
            } else if (gamepad_state.buttons.y) {
                LOG_DBG("System+Y: Clearing bonded peers");
                ble_service.clear_bonded_peers();
                hw.restart();
            } else {
                LOG_DBG("System Long Press: BT Pairing Mode");
                ble_service.start_advertising_discoverable();
                hw.set_led(LedPattern::AdvertisingDiscoverable);
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
        ble_service.start_advertising_undiscoverable();
    }
    else
    {
        hw.set_led(LedPattern::AdvertisingDiscoverable);
        ble_service.start_advertising_discoverable();
    }
}

void OceanPadApp::on_advertising_discoverable_timeout() {
    if (ble_service.get_state() >= BleServiceState::Connected)
    {
        hw.set_led(LedPattern::Connected);
    }
    else if (ble_service.has_bonded_peer()) {
        hw.set_led(LedPattern::AdvertisingUndiscoverable);
        ble_service.start_advertising_undiscoverable();
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
        return false;
    }
    if (gp_state.dpad != DPadState::Centered)
    {
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
        return false;
    }
    if (gp_state.axes.trigger_lt > AXIS_ACTIVITY_THRESHOLD ||
        gp_state.axes.trigger_rt > AXIS_ACTIVITY_THRESHOLD
    )
    {
        return false;
    }
    return true;
}

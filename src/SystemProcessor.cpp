#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "SystemProcessor.hpp"

LOG_MODULE_REGISTER(SystemProcessor, LOG_LEVEL_DBG);

void SystemProcessor::init(HardwareManager* hardware_manager_ptr, BleService* ble_ptr) {
    hardware_manager = hardware_manager_ptr;
    ble_service = ble_ptr;
    identity_idx = hardware_manager->get_identity_idx();
    next_battery_update_time = k_uptime_get();
    start_advertising();
}

void SystemProcessor::process_input_logic(const GamepadState& gamepad_state) {
    if (!is_state_idle(gamepad_state))
    {
        last_non_idle_time = k_uptime_get();
    }
}

void SystemProcessor::process_system_logic(const GamepadState& gamepad_state) {
    int64_t current_time = k_uptime_get();
    if (current_time > next_battery_update_time)
    {
        ble_service->update_battery_level(hardware_manager->get_battery_percent());
        next_battery_update_time = current_time + BATTERY_UPDATE_PERIOD_MS;
    }
    if (current_time > last_non_idle_time + IDLE_TIMEOUT_MS)
    {
        LOG_DBG("Idle timeout, shutting down");
        hardware_manager->sleep();
    }
    if (hardware_manager->get_identity_idx() != identity_idx)
    {
        ble_service->do_disconnect();
        LOG_DBG("Identity switched, restarting");
        hardware_manager->restart();
    }
    if (gamepad_state.buttons.mode) {
        if (system_press_start == 0) {
            system_press_start = current_time;
        } else if (system_press_start > 0 && (current_time - system_press_start >= LONG_PRESS_TIMEOUT_MS)) {
            if (gamepad_state.buttons.b && !gamepad_state.buttons.y && !gamepad_state.buttons.x) {
                LOG_DBG("Mode+B: Axes calibration");
                hardware_manager->start_calibration(0);
            } else if (!gamepad_state.buttons.b && gamepad_state.buttons.y && !gamepad_state.buttons.x) {
                LOG_DBG("Mode+Y: Clearing bonded peers");
                ble_service->clear_bonded_peers();
                hardware_manager->restart();
            } else if (!gamepad_state.buttons.b && !gamepad_state.buttons.y && gamepad_state.buttons.x) {
                if (hardware_manager->get_identity_idx() == 1) {
                    LOG_DBG("Mode+X: Gyro calibration");
                    hardware_manager->start_calibration(1);
                }
                else {
                    LOG_DBG("Mode+X: Gyro calibration only available for IMU enabled mode");
                }
            } else if (!gamepad_state.buttons.b && !gamepad_state.buttons.y && !gamepad_state.buttons.x) {
                BleServiceState ble_state = ble_service->get_state();
                if (ble_state == BleServiceState::AdvertisingDiscoverable || ble_state == BleServiceState::ConnectedAdvertising) {
                    LOG_DBG("Mode Long Press: Stop advertsing discoverable");
                    ble_service->stop_advertising();
                    if (ble_service->get_state() == BleServiceState::AdvertisingUndiscoverable)
                    {
                        hardware_manager->set_led(LedPattern::AdvertisingUndiscoverable);
                    }
                    else if (ble_service->get_state() == BleServiceState::Connected)
                    {
                        hardware_manager->set_led(LedPattern::Connected);
                    }
                } else {
                    LOG_DBG("Mode Long Press: Advertsing discoverable");
                    ble_service->start_advertising(true);
                    hardware_manager->set_led(LedPattern::AdvertisingDiscoverable);
                }
            }
            system_press_start = -1;
        }
    } else {
        system_press_start = 0;
    }

    if (gamepad_state.buttons.home) {
        if (home_press_start == 0) {
            home_press_start = current_time;
        } else if (home_press_start > 0 && (current_time - home_press_start >= LONG_PRESS_TIMEOUT_MS)) {
            LOG_DBG("HOME Long Press: System OFF");
            ble_service->do_disconnect();
            hardware_manager->sleep();
            home_press_start = -1;
        }
    } else {
        home_press_start = 0;
    }
}

void SystemProcessor::on_connected() {
    hardware_manager->set_led(LedPattern::Connected);
    next_battery_update_time = k_uptime_get() + 500;
}

void SystemProcessor::on_disconnected(bool graceful) {
    if (graceful) {
        hardware_manager->sleep();
    } else {
        start_advertising();
    }
}

void SystemProcessor::on_adv_timeout(bool discoverable) {
    if (discoverable) {
        on_advertising_discoverable_timeout();
    } else {
        hardware_manager->sleep();
    }
}

void SystemProcessor::start_advertising() {
    if (ble_service->has_bonded_peer())
    {
        hardware_manager->set_led(LedPattern::AdvertisingUndiscoverable);
        ble_service->start_advertising(false);
    }
    else
    {
        hardware_manager->set_led(LedPattern::AdvertisingDiscoverable);
        ble_service->start_advertising(true);
    }
}

void SystemProcessor::on_advertising_discoverable_timeout() {
    if (ble_service->get_state() >= BleServiceState::Connected)
    {
        hardware_manager->set_led(LedPattern::Connected);
    }
    else if (ble_service->has_bonded_peer()) {
        hardware_manager->set_led(LedPattern::AdvertisingUndiscoverable);
        ble_service->start_advertising(false);
    }
    else
    {
        hardware_manager->sleep();
    }
}

bool SystemProcessor::is_state_idle(const GamepadState& gamepad_state) {
    DigitalButtons btns_copy = gamepad_state.buttons;
    btns_copy.xd_switch = 0;
    if (*(uint16_t*)(&btns_copy))
    {
        return false;
    }
    if (gamepad_state.dpad != DPadState::Centered)
    {
        return false;
    }
    auto abs = [](int16_t inp) -> uint16_t {
        if (inp < 0) return -inp;
        return inp;
    };
    if (abs(gamepad_state.axes.stick_lx) > AXIS_ACTIVITY_THRESHOLD ||
        abs(gamepad_state.axes.stick_ly) > AXIS_ACTIVITY_THRESHOLD ||
        abs(gamepad_state.axes.stick_rx) > AXIS_ACTIVITY_THRESHOLD ||
        abs(gamepad_state.axes.stick_ry) > AXIS_ACTIVITY_THRESHOLD
    )
    {
        return false;
    }
    if (gamepad_state.axes.trigger_lt > AXIS_ACTIVITY_THRESHOLD ||
        gamepad_state.axes.trigger_rt > AXIS_ACTIVITY_THRESHOLD
    )
    {
        return false;
    }
    return true;
}

#include "HardwareManager/HardwareManager.hpp"
#include <zephyr/logging/log.h>
#include <hal/nrf_saadc.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_power.h>


LOG_MODULE_REGISTER(HardwareManager, LOG_LEVEL_INF);

HardwareManager::HardwareManager() {
    k_mutex_init(&data_mutex);
    memset(&gamepad_state, 0, sizeof(gamepad_state));
}

int HardwareManager::init() {
    int err = 0;

    err = gpio_pin_configure_dt(&axes_pwr, GPIO_OUTPUT_ACTIVE);
    if (err) {
        LOG_ERR("Failed to configure axes power gpio, err %d", err);
        return err;
    }
    err = gpio_pin_set_dt(&axes_pwr, 1);
    if (err) {
        LOG_ERR("Failed to enable axes power gpio, err %d", err);
        return err;
    }

    err = battery_gauge.init();
    if (err) {
        LOG_ERR("Failed to init battery_gauge, err %d", err);
        return err;
    }

    err = raw_input_reader.init();
    if (err)
    {
        LOG_ERR("Failed to setup raw input reader, err %d", err);
        return err;
    }

    err = input_processor.init();
    if (err)
    {
        LOG_ERR("Failed to setup input processor, err %d", err);
        return err;
    }

    err = motor_vibrator.init();
    if (err)
    {
        LOG_ERR("Failed to setup motor vibrator, err %d", err);
        return err;
    }

    err = led_blinker.init();
    if (err)
    {
        LOG_ERR("Failed to setup led blinker, err %d", err);
        return err;
    }

    err = raw_input_reader.getRawData(raw_data);
    if (err)
    {
        LOG_ERR("Failed to read raw data, err %d", err);
        return err;
    }

    k_mutex_lock(&data_mutex, K_FOREVER);
    input_processor.process_raw_data(gamepad_state, raw_data);
    k_mutex_unlock(&data_mutex);
    return 0;
}

int HardwareManager::update() {
    if (shutdown) {
        return 0;
    }
    int ret = raw_input_reader.getRawData(raw_data);
    if (ret) {
        return ret;
    }
    GamepadState new_state;
    input_processor.process_raw_data(new_state, raw_data);
    if (is_calibration()) {
        if (input_processor.is_ready_to_reboot_after_calibration()) {
            restart();
        }
        return 0;
    }
    k_mutex_lock(&data_mutex, K_FOREVER);
    gamepad_state = new_state;
    k_mutex_unlock(&data_mutex);
    return 0;
}

void HardwareManager::get_state(GamepadState& state) {
    k_mutex_lock(&data_mutex, K_FOREVER);
    state = gamepad_state;
    k_mutex_unlock(&data_mutex);
}

void HardwareManager::set_vibration(const VibrationDataXbox& vibr_d) {
    motor_vibrator.apply_vibration(vibr_d);
}

void HardwareManager::set_led(LedPattern led_pattern) {
    switch (led_pattern) {
    case LedPattern::Connected:
        led_blinker.set_brightness(255);
        break;
    case LedPattern::AdvertisingUndiscoverable:
        led_blinker.start_sequence(LED_SEQ_ADV_UNDISCO);
        break;
    case LedPattern::AdvertisingDiscoverable:
        led_blinker.start_sequence(LED_SEQ_ADV_DISCO);
        break;
    }
}

uint8_t HardwareManager::get_battery_percent() {
    gamepad_state.battery_percent = battery_gauge.get_battery_percent();;
    return gamepad_state.battery_percent;
}

void HardwareManager::start_calibration() {
    input_processor.start_axes_calibration();
    led_blinker.start_sequence(LED_SEQ_CALIB);
}

bool HardwareManager::is_calibration() {
    return input_processor.is_axes_calibration();
}

void HardwareManager::restart() {
    shutdown = true;
    int err = raw_input_reader.deinit();
    if (err) {
        LOG_ERR("RawDataReader deinit failed, err: %d", err);
    }
    ignore_led = true;
    led_blinker.set_brightness(0);
    motor_vibrator.stop();
    shutdown = true;
    LOG_DBG("Restarting");
    k_msleep(100);
    sys_reboot(SYS_REBOOT_COLD);
}

void HardwareManager::sleep() {
    shutdown = true;
    gpio_pin_set_dt(&axes_pwr, 0);
    int err = raw_input_reader.deinit();
    if (err) {
        LOG_ERR("RawDataReader deinit failed, err: %d", err);
    }
    ignore_led = true;
    led_blinker.set_brightness(0);
    motor_vibrator.stop();
    LOG_DBG("Going to sleep");
    k_msleep(100);
    nrf_power_system_off(NRF_POWER);
}


#include "hardware/HardwareManager.hpp"
#include <zephyr/logging/log.h>
#include <hal/nrf_saadc.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_power.h>


LOG_MODULE_REGISTER(HardwareManager, LOG_LEVEL_INF);

HardwareManager* HardwareManager::instance = nullptr;

static struct settings_handler input_conf = {
    .name = "oceanpad",
    .h_set = HardwareManager::settings_set
};

HardwareManager::HardwareManager() {
    instance = this;
    k_mutex_init(&data_mutex);
    memset(&input_state, 0, sizeof(input_state));
    i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    // Default values

    calib_data.lx.center = calib_data.ly.center = calib_data.rx.center = calib_data.ry.center = 1800;
    calib_data.lx.min = calib_data.ly.min = calib_data.rx.min = calib_data.ry.min = 400;
    calib_data.lx.max = calib_data.ly.max = calib_data.rx.max = calib_data.ry.max = 3200;

    calib_data.lt.min = calib_data.rt.min = 1200;
    calib_data.lt.max = calib_data.rt.max = 2200;
}

int HardwareManager::init() {
    int err = adc_channel_setup_dt(&vbat);
    if (err < 0) {
        LOG_ERR("Failed to setup channel,  err %d", err);
        return err;
    }

    gpio_pin_configure_dt(&axes_pwr, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_ACTIVE);

    gpio_pin_set_dt(&axes_pwr, 1);
    gpio_pin_set_dt(&status_led, 0);

    if (!pwm_is_ready_dt(&motor_big) || !pwm_is_ready_dt(&motor_small)) {
        LOG_ERR("Motor PWM not init!");
        return -ENODEV;
    }
    raw_data_reader.init();
    err = raw_data_reader.getRawData(raw_data);
    k_mutex_lock(&data_mutex, K_FOREVER);
    copy_native_buttons();
    k_mutex_unlock(&data_mutex);
    settings_register(&input_conf);
    return 0;
}

int HardwareManager::update() {
    int ret = raw_data_reader.getRawData(raw_data);
    if (ret) {
        return ret;
    }
    if (is_calibration()) {
        calibration_routine();
        return 0;
    }
    k_mutex_lock(&data_mutex, K_FOREVER);
    copy_expander();
    copy_native_buttons();
    copy_analog();
    if (raw_data.native_buttons.xd_switch) {
        input_state.imu_data = raw_data.raw_imu; //for now
    }
    k_mutex_unlock(&data_mutex);
    return 0;
}
/*
void HardwareManager::update_phys() {
    update_expander();
    update_native_buttons();
    update_analog();
    if (is_calibration()) {
        calibration_routine();
        return;
    }
    k_mutex_lock(&data_mutex, K_FOREVER);
    copy_expander();
    copy_native_buttons();
    copy_analog();
    k_mutex_unlock(&data_mutex);
}

void HardwareManager::update_imu() {
    uint8_t raw[12];

    if (i2c_burst_read(i2c_bus, IMU_ADDR, REG_ACCEL_DATA_X1, raw, 12) == 0) {
        raw_data.raw_imu.accel[0] = (int16_t)((raw[0] << 8) | raw[1]);
        raw_data.raw_imu.accel[1] = (int16_t)((raw[2] << 8) | raw[3]);
        raw_data.raw_imu.accel[2] = (int16_t)((raw[4] << 8) | raw[5]);

        raw_data.raw_imu.gyro[0] = (int16_t)((raw[6] << 8) | raw[7]);
        raw_data.raw_imu.gyro[1] = (int16_t)((raw[8] << 8) | raw[9]);
        raw_data.raw_imu.gyro[2] = (int16_t)((raw[10] << 8) | raw[11]);
    }
    k_mutex_lock(&data_mutex, K_FOREVER);
    input_state.imu_data = raw_data.raw_imu; //for now
    k_mutex_unlock(&data_mutex);
}
*/

GamepadState HardwareManager::get_state_copy() {
    GamepadState ret;
    k_mutex_lock(&data_mutex, K_FOREVER);
    ret = input_state;
    k_mutex_unlock(&data_mutex);
    return ret;
}

void HardwareManager::set_vibration(VibrationData vibr_d) {
    auto calculate_pulse = [this](const struct pwm_dt_spec *spec, uint8_t pct) -> uint32_t {
        if (pct == 0) return 0;

        if (pct > 100) pct = 100;

        uint32_t scaled_pct = MIN_MOTOR_PERCENT + (static_cast<uint32_t>(pct) * (100 - MIN_MOTOR_PERCENT) / 100);

        return (spec->period * scaled_pct) / 100;
    };

    uint32_t pulse_big = calculate_pulse(&motor_big, vibr_d.big_motor);
    uint32_t pulse_small = calculate_pulse(&motor_small, vibr_d.small_motor);

    LOG_DBG("Vibration scaling: H %d%% -> pulse %d | G %d%% -> pulse %d",
            vibr_d.big_motor, pulse_big, vibr_d.small_motor, pulse_small);

    pwm_set_pulse_dt(&motor_big, pulse_big);
    pwm_set_pulse_dt(&motor_small, pulse_small);
}

void HardwareManager::set_led(bool is_on) {
    gpio_pin_set_dt(&status_led, is_on);
}

uint8_t HardwareManager::get_battery_percent() {
    uint16_t vbat_raw;
    struct adc_sequence seq = {
        .channels    = BIT(6),
        .buffer      = &vbat_raw,
        .buffer_size = sizeof(vbat_raw),
        .resolution  = 12,
        .oversampling = 4,
    };

    int err = adc_read(adc_dev, &seq);
    if (err) {
        LOG_ERR("ADC vbat read err: %d", err);
        return 0;
    }

    LOG_DBG("Vbat raw: %u", vbat_raw);

    int32_t batt_mv_unscaled = vbat_raw;

    adc_raw_to_millivolts_dt(&vbat, &batt_mv_unscaled);

    LOG_DBG("Vbat mV unscaled: %u", batt_mv_unscaled);

    uint32_t batt_mv_scaled = VBAT_SCALE_MUL * batt_mv_unscaled / VBAT_SCALE_DIV;

    LOG_DBG("Vbat mV scaled: %u", batt_mv_scaled);

    uint32_t batt_percent;

    if (batt_mv_scaled < BAT_MV_0) {
        batt_percent = 0;
    } else if (batt_mv_scaled > BAT_MV_100) {
        batt_percent = 100;
    } else {
        batt_percent = 100 * (batt_mv_scaled - BAT_MV_0) / (BAT_MV_100 - BAT_MV_0);
    }

    LOG_DBG("Batt %%: %u", batt_percent);

    input_state.battery_percent = batt_percent;
    return (uint8_t) batt_percent;
}

void HardwareManager::start_calibration() {
    calib_state = CalibState::WaitingForCenter;
}

bool HardwareManager::is_calibration() {
    return calib_state != CalibState::Idle;
}

void HardwareManager::restart() {
    //if (i2c_reg_write_byte(i2c_bus, IMU_ADDR, REG_PWR_MGMT0, PWR_MODE_SLEEP) != 0) {
    //    LOG_WRN("IMU: Failed to set sleep mode");
    //} //in case we won't need the imu after restart
    int err = raw_data_reader.deinit();
    if (err) {
        LOG_ERR("RawDataReader deinit failed, err: %d", err);
    }
    LOG_DBG("Restarting");
    k_msleep(100);
    sys_reboot(SYS_REBOOT_COLD);
}

void HardwareManager::sleep() {
    set_led(false);
    //if (i2c_reg_write_byte(i2c_bus, IMU_ADDR, REG_PWR_MGMT0, PWR_MODE_SLEEP) != 0) {
    //    LOG_WRN("IMU: Failed to set sleep mode");
    //}
    gpio_pin_set_dt(&axes_pwr, 0);
    //gpio_pin_configure_dt(&xd_switch, GPIO_DISCONNECTED);
    //gpio_pin_set_dt(&exp_reset, 1); //prob stays latched
    //gpio_pin_interrupt_configure_dt(&native_pins.home, GPIO_INT_LEVEL_ACTIVE);
    int err = raw_data_reader.deinit();
    if (err) {
        LOG_ERR("RawDataReader deinit failed, err: %d", err);
    }
    LOG_DBG("Going to sleep");
    k_msleep(100);
    nrf_power_system_off(NRF_POWER);
}

int HardwareManager::settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    LOG_DBG("Settings set callback called, name: %s", name);
    const char *next;
    if (settings_name_steq(name, "cal", &next) && !next) {
        if (len != sizeof(calib_data)) return -EINVAL;

        if (instance) {
            int ret = read_cb(cb_arg, &instance->calib_data, sizeof(instance->calib_data));
            LOG_DBG("Calib LX min: %4d, center: %4d, max: %4d", instance->calib_data.lx.min, instance->calib_data.lx.center, instance->calib_data.lx.max);
            LOG_DBG("Calib LY min: %4d, center: %4d, max: %4d", instance->calib_data.ly.min, instance->calib_data.ly.center, instance->calib_data.ly.max);
            LOG_DBG("Calib RX min: %4d, center: %4d, max: %4d", instance->calib_data.rx.min, instance->calib_data.rx.center, instance->calib_data.rx.max);
            LOG_DBG("Calib RY min: %4d, center: %4d, max: %4d", instance->calib_data.ry.min, instance->calib_data.rx.center, instance->calib_data.rx.max);

            LOG_DBG("Calib LT min: %4d, max: %4d", instance->calib_data.lt.min, instance->calib_data.lt.max);
            LOG_DBG("Calib RT min: %4d, max: %4d", instance->calib_data.rt.min, instance->calib_data.rt.max);

            if (ret >= 0)
            {
                return 0;
            }

            return ret;
        }
    }

    return -ENOENT;
}

void HardwareManager::debug_print_raw() {
    LOG_DBG("LX %4d LY %4d RX %4d RY %4d LT %4d RT %4d",
        raw_data.raw_axes.raw_lx, raw_data.raw_axes.raw_ly,
        raw_data.raw_axes.raw_rx, raw_data.raw_axes.raw_ry,
        raw_data.raw_axes.raw_lt, raw_data.raw_axes.raw_rt
    );
}

/*
int HardwareManager::setup_expander() {
    // Enable Pull-up/down resistors
    i2c_reg_write_byte(i2c_bus, EXP_ADDR, REG_PUD_EN0, 0xFF);
    i2c_reg_write_byte(i2c_bus, EXP_ADDR, REG_PUD_EN1, 0xFF);

    // Select Pull-DOWN (0x00) for all pins
    i2c_reg_write_byte(i2c_bus, EXP_ADDR, REG_PUD_SEL0, 0x00);
    i2c_reg_write_byte(i2c_bus, EXP_ADDR, REG_PUD_SEL1, 0x00);

    return 0;
}

int HardwareManager::setup_imu() {
    uint8_t chip_id = 0;

    if (i2c_reg_read_byte(i2c_bus, IMU_ADDR, REG_WHO_AM_I, &chip_id) != 0) {
        LOG_ERR("IMU: I2C read failed (WhoAmI)");
        return false;
    }

    if (chip_id != WHO_AM_I_EXPECTED) {
        LOG_ERR("IMU: Wrong chip ID: 0x%02X (Expected 0x47)", chip_id);
        return false;
    }

    i2c_reg_write_byte(i2c_bus, IMU_ADDR, REG_DEVICE_CONFIG, BIT_SOFT_RESET);
    k_msleep(2);

    i2c_reg_write_byte(i2c_bus, IMU_ADDR, REG_PWR_MGMT0, PWR_LN_ALL_ON);

    k_msleep(30);

    i2c_reg_write_byte(i2c_bus, IMU_ADDR, REG_GYRO_CONFIG0, IMU_GYRO_2000DPS_1KHZ);

    i2c_reg_write_byte(i2c_bus, IMU_ADDR, REG_ACCEL_CONFIG0, IMU_ACCEL_8G_1KHZ);

    LOG_INF("IMU: Configured for 2000dps, 8G @ 1kHz");
    return 0;
}

void HardwareManager::update_expander() {
    if (i2c_burst_read(i2c_bus, EXP_ADDR, REG_INPUT_P0, reinterpret_cast<uint8_t*>(&raw_data.expander_buttons), 2) < 0) {
        LOG_WRN("Expander read failed");
        return;
    }
}

void HardwareManager::update_native_buttons() {
    raw_data.native_buttons.lb = gpio_pin_get_dt(&native_pins.lb);
    raw_data.native_buttons.rb = gpio_pin_get_dt(&native_pins.rb);

    raw_data.native_buttons.ls = gpio_pin_get_dt(&native_pins.ls);
    raw_data.native_buttons.rs = gpio_pin_get_dt(&native_pins.rs);

    raw_data.native_buttons.start = gpio_pin_get_dt(&native_pins.start);
    raw_data.native_buttons.back = gpio_pin_get_dt(&native_pins.back);
    raw_data.native_buttons.home = gpio_pin_get_dt(&native_pins.home);

    raw_data.native_buttons.switch_xd = gpio_pin_get_dt(&xd_switch);
}

void HardwareManager::update_analog() {
    struct adc_sequence seq = {
        .channels       = 0b00111111,
        .buffer         = &raw_data.raw_axes,
        .buffer_size    = sizeof(raw_data.raw_axes),
        .resolution     = 12,
    };
    adc_read(adc_dev, &seq);
}

*/

void HardwareManager::copy_expander() {
    if (data_mutex.owner != k_current_get())
    {
        LOG_WRN("Mutex is not locked!");
    }

    input_state.buttons.a = raw_data.expander_buttons.a;
    input_state.buttons.b = raw_data.expander_buttons.b;
    input_state.buttons.x = raw_data.expander_buttons.x;
    input_state.buttons.y = raw_data.expander_buttons.y;

    input_state.buttons.mode = raw_data.expander_buttons.mode;
    input_state.buttons.vibr = raw_data.expander_buttons.vibr;

    input_state.dpad = get_dpad_state(
        raw_data.expander_buttons.dpad_left << 3 |
        raw_data.expander_buttons.dpad_down << 2 |
        raw_data.expander_buttons.dpad_right << 1 |
        raw_data.expander_buttons.dpad_up << 0
    );
}

void HardwareManager::copy_native_buttons() {
    if (data_mutex.owner != k_current_get())
    {
        LOG_WRN("Mutex is not locked!");
    }

    input_state.buttons.lb = raw_data.native_buttons.lb;
    input_state.buttons.rb = raw_data.native_buttons.rb;

    input_state.buttons.ls = raw_data.native_buttons.ls;
    input_state.buttons.rs = raw_data.native_buttons.rs;

    input_state.buttons.start = raw_data.native_buttons.start;
    input_state.buttons.back = raw_data.native_buttons.back;
    input_state.buttons.home = raw_data.native_buttons.home;

    input_state.buttons.switch_xd = raw_data.native_buttons.xd_switch;
}

void HardwareManager::copy_analog() {
    if (data_mutex.owner != k_current_get())
    {
        LOG_WRN("Mutex is not locked!");
    }
    input_state.axes = get_calibrated_axes(raw_data.raw_axes, calib_data);
}

void HardwareManager::calibration_routine() {
    static bool home_was_pressed = false;
    if (home_was_pressed && raw_data.native_buttons.home)
    {
        return;
    }
    home_was_pressed = false;
    switch (calib_state) {
        case CalibState::WaitingForCenter:
            if (raw_data.native_buttons.home) {
                calib_data.lx.center = calib_data.lx.min = calib_data.lx.max = raw_data.raw_axes.raw_lx;

                calib_data.ly.center = calib_data.ly.min = calib_data.ly.max = raw_data.raw_axes.raw_ly;

                calib_data.rx.center = calib_data.rx.min = calib_data.rx.max = raw_data.raw_axes.raw_rx;

                calib_data.ry.center = calib_data.ry.min = calib_data.ry.max = raw_data.raw_axes.raw_ry;

                calib_data.lt.min = calib_data.lt.max = raw_data.raw_axes.raw_lt;

                calib_data.rt.min = calib_data.rt.max = raw_data.raw_axes.raw_rt;

                LOG_INF("Center Captured. Rotate sticks, then press HOME again.");
                calib_state = CalibState::MeasuringLimits;
                home_was_pressed = true;
            }
            break;

        case CalibState::MeasuringLimits:
            calib_data.lx.min = MIN(calib_data.lx.min, raw_data.raw_axes.raw_lx);
            calib_data.lx.max = MAX(calib_data.lx.max, raw_data.raw_axes.raw_lx);

            calib_data.ly.min = MIN(calib_data.ly.min, raw_data.raw_axes.raw_ly);
            calib_data.ly.max = MAX(calib_data.ly.max, raw_data.raw_axes.raw_ly);

            calib_data.rx.min = MIN(calib_data.rx.min, raw_data.raw_axes.raw_rx);
            calib_data.rx.max = MAX(calib_data.rx.max, raw_data.raw_axes.raw_rx);

            calib_data.ry.min = MIN(calib_data.rx.min, raw_data.raw_axes.raw_rx);
            calib_data.ry.max = MAX(calib_data.rx.max, raw_data.raw_axes.raw_rx);

            calib_data.lt.min = MIN(calib_data.lt.min, raw_data.raw_axes.raw_lt);
            calib_data.lt.max = MAX(calib_data.lt.max, raw_data.raw_axes.raw_lt);

            calib_data.rt.min = MIN(calib_data.rt.min, raw_data.raw_axes.raw_rt);
            calib_data.rt.max = MAX(calib_data.rt.max, raw_data.raw_axes.raw_rt);

            if (raw_data.native_buttons.home) {
                LOG_INF("Limits Captured. Saving...");
                calib_state = CalibState::Saving;
                home_was_pressed = true;
            }
            break;

        case CalibState::Saving:
            save_calibration();
            k_msleep(100);
            LOG_INF("Done. Rebooting...");
            restart();
            break;

        default:
            break;
    }
}

void HardwareManager::save_calibration() {
    settings_save_one("oceanpad/cal", &calib_data, sizeof(calib_data));
}

DPadState HardwareManager::get_dpad_state(uint8_t mask) {
    switch (mask)
    {
        case 0b0001:
            return DPadState::Up;
        case 0b0011:
            return DPadState::UpRight;
        case 0b0010:
            return DPadState::Right;
        case 0b0110:
            return DPadState::DownRight;
        case 0b0100:
            return DPadState::Down;
        case 0b1100:
            return DPadState::DownLeft;
        case 0b1000:
            return DPadState::Left;
        case 0b1001:
            return DPadState::UpLeft;
    }
    return DPadState::Centered;
}

AnalogAxes HardwareManager::get_calibrated_axes(const RawAxes& raw_axes, const CalibData& calib_data) {
    AnalogAxes ret_axes;
    ret_axes.stick_lx = get_calibrated_stick(raw_axes.raw_lx, calib_data.lx, CENTER_DEADZONE, EDGE_DEADZONE, false);
    ret_axes.stick_ly = get_calibrated_stick(raw_axes.raw_ly, calib_data.ly, CENTER_DEADZONE, EDGE_DEADZONE, true);
    ret_axes.stick_rx = get_calibrated_stick(raw_axes.raw_rx, calib_data.rx, CENTER_DEADZONE, EDGE_DEADZONE, false);
    ret_axes.stick_ry = get_calibrated_stick(raw_axes.raw_ry, calib_data.ry, CENTER_DEADZONE, EDGE_DEADZONE, true);

    ret_axes.trigger_lt = get_calibrated_trigger(raw_axes.raw_lt, calib_data.lt, EDGE_DEADZONE);
    ret_axes.trigger_rt = get_calibrated_trigger(raw_axes.raw_rt, calib_data.rt, EDGE_DEADZONE);
    return ret_axes;
}

int16_t HardwareManager::get_calibrated_stick(const uint16_t raw_stick, const StickCalib& stick_calib, uint16_t center_dz, uint16_t edge_dz, bool is_inverted) {
    int32_t out = 0;

    if (raw_stick > (stick_calib.center - center_dz) && raw_stick < (stick_calib.center + center_dz)) {
        return 0;
    }

    if (raw_stick < stick_calib.center) {
        int32_t range = (stick_calib.center - center_dz) - (stick_calib.min + edge_dz);
        if (range <= 0) return 0;
        out = ((int32_t)raw_stick - (stick_calib.center - center_dz)) * 32767 / range;
    } else {
        int32_t range = (stick_calib.max - edge_dz) - (stick_calib.center + center_dz);
        if (range <= 0) return 0;
        out = ((int32_t)raw_stick - (stick_calib.center + center_dz)) * 32767 / range;
    }

    if (is_inverted) {
        out = -out;
    }
    if (out > 32767) out = 32767;
    if (out < -32767) out = -32767;

    return (int16_t)out;
}

uint16_t HardwareManager::get_calibrated_trigger(const uint16_t raw_trigger, const TriggerCalib& trigger_calib, uint16_t edge_dz) {
    int32_t eff_min = (int32_t)trigger_calib.min + edge_dz;
    int32_t eff_max = (int32_t)trigger_calib.max - edge_dz;
    int32_t eff_range = eff_max - eff_min;

    if (eff_range <= 0) return 0;

    if (raw_trigger <= eff_min) return 65535;
    if (raw_trigger >= eff_max) return 0;

    int32_t out = ((eff_max - (int32_t)raw_trigger) * 65535) / eff_range;

    return (uint16_t)out;
}


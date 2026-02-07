#include <zephyr/kernel.h>

#include "HardwareManager/InputProcessor/AxesCalibrator.hpp"

static struct settings_handler calib_conf = {
    .name = "oceanpad",
    .h_set = AxesCalibrator::settings_set
};

AxesCalibrator* instance = nullptr;

AxesCalibrator::AxesCalibrator() {
    instance = this;
    calib_data.lx.center = calib_data.ly.center = calib_data.rx.center = calib_data.ry.center = 1800;
    calib_data.lx.min = calib_data.ly.min = calib_data.rx.min = calib_data.ry.min = 400;
    calib_data.lx.max = calib_data.ly.max = calib_data.rx.max = calib_data.ry.max = 3200;
    calib_data.lt.min = calib_data.rt.min = 1200;
    calib_data.lt.max = calib_data.rt.max = 2200;
}

int AxesCalibrator::init() {
    return settings_register(&calib_conf);
}

void AxesCalibrator::process_raw_axes(AnalogAxes& analog_axes, const RawAxes& raw_axes) {
    if (calibration)
    {
        process_calibration(raw_axes);
        return;
    }
    get_calibrated_axes(analog_axes, raw_axes, calib_data);
}

void AxesCalibrator::save_calibration() {
    settings_save_one("oceanpad/cal", &calib_data, sizeof(calib_data));
}

void AxesCalibrator::start_calibration() {
    calibration = true;
    capture_center_flag = true;
}

int AxesCalibrator::settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    if (settings_name_steq(name, "cal", &next) && !next) {
        if (len != sizeof(calib_data)) {
            return -EINVAL;
        }

        if (instance) {
            int ret = read_cb(cb_arg, &instance->calib_data, sizeof(instance->calib_data));
            if (ret >= 0)
            {
                return 0;
            }
            return ret;
        }
    }
    return -ENOENT;
}

void AxesCalibrator::process_calibration(const RawAxes& raw_axes) {
    if (capture_center_flag) {
        calib_data.lx.center = calib_data.lx.min = calib_data.lx.max = raw_axes.raw_lx;
        calib_data.ly.center = calib_data.ly.min = calib_data.ly.max = raw_axes.raw_ly;
        calib_data.rx.center = calib_data.rx.min = calib_data.rx.max = raw_axes.raw_rx;
        calib_data.ry.center = calib_data.ry.min = calib_data.ry.max = raw_axes.raw_ry;
        calib_data.lt.min = calib_data.lt.max = raw_axes.raw_lt;
        calib_data.rt.min = calib_data.rt.max = raw_axes.raw_rt;
        capture_center_flag = false;
        return;
    }
    calib_data.lx.min = MIN(calib_data.lx.min, raw_axes.raw_lx);
    calib_data.lx.max = MAX(calib_data.lx.max, raw_axes.raw_lx);

    calib_data.ly.min = MIN(calib_data.ly.min, raw_axes.raw_ly);
    calib_data.ly.max = MAX(calib_data.ly.max, raw_axes.raw_ly);

    calib_data.rx.min = MIN(calib_data.rx.min, raw_axes.raw_rx);
    calib_data.rx.max = MAX(calib_data.rx.max, raw_axes.raw_rx);

    calib_data.ry.min = MIN(calib_data.ry.min, raw_axes.raw_ry);
    calib_data.ry.max = MAX(calib_data.ry.max, raw_axes.raw_ry);

    calib_data.lt.min = MIN(calib_data.lt.min, raw_axes.raw_lt);
    calib_data.lt.max = MAX(calib_data.lt.max, raw_axes.raw_lt);

    calib_data.rt.min = MIN(calib_data.rt.min, raw_axes.raw_rt);
    calib_data.rt.max = MAX(calib_data.rt.max, raw_axes.raw_rt);
}

void AxesCalibrator::get_calibrated_axes(AnalogAxes& axes, const RawAxes& raw_axes, const CalibData& calib_data) {
    axes.stick_lx = get_calibrated_stick(raw_axes.raw_lx, calib_data.lx, CENTER_DEADZONE, EDGE_DEADZONE, false);
    axes.stick_ly = get_calibrated_stick(raw_axes.raw_ly, calib_data.ly, CENTER_DEADZONE, EDGE_DEADZONE, true);
    axes.stick_rx = get_calibrated_stick(raw_axes.raw_rx, calib_data.rx, CENTER_DEADZONE, EDGE_DEADZONE, false);
    axes.stick_ry = get_calibrated_stick(raw_axes.raw_ry, calib_data.ry, CENTER_DEADZONE, EDGE_DEADZONE, true);

    axes.trigger_lt = get_calibrated_trigger(raw_axes.raw_lt, calib_data.lt, EDGE_DEADZONE);
    axes.trigger_rt = get_calibrated_trigger(raw_axes.raw_rt, calib_data.rt, EDGE_DEADZONE);
}

int16_t AxesCalibrator::get_calibrated_stick(const uint16_t raw_stick, const StickCalib& stick_calib, uint16_t center_dz, uint16_t edge_dz, bool is_inverted) {
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

uint16_t AxesCalibrator::get_calibrated_trigger(const uint16_t raw_trigger, const TriggerCalib& trigger_calib, uint16_t edge_dz) {
    int32_t eff_min = (int32_t)trigger_calib.min + edge_dz;
    int32_t eff_max = (int32_t)trigger_calib.max - edge_dz;
    int32_t eff_range = eff_max - eff_min;

    if (eff_range <= 0) return 0;

    if (raw_trigger <= eff_min) return 65535;
    if (raw_trigger >= eff_max) return 0;

    int32_t out = ((eff_max - (int32_t)raw_trigger) * 65535) / eff_range;

    return (uint16_t)out;
}

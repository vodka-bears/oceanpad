#include <zephyr/logging/log.h>

#include "HardwareManager/RawInputReader/RawInputReader.hpp"

LOG_MODULE_REGISTER(RawInputReader, LOG_LEVEL_WRN);

int RawInputReader::init() {
    int err = 0;

    err = native_buttons_reader.init();
    if (err)
    {
        LOG_ERR("Failed to initialize native buttons reader, err: %d", err);
        return err;
    }

    NativeButtons init_buttons;
    err = native_buttons_reader.getRawData(init_buttons);
    if (err)
    {
        LOG_ERR("Failed to read initial native buttons, err: %d", err);
        return err;
    }
    imu_enabled = init_buttons.xd_switch;

    err = expander_buttons_reader.init();
    if (err)
    {
        LOG_ERR("Failed to initialize expander buttons reader, err: %d", err);
        return err;
    }

    err = raw_axes_reader.init();
    if (err)
    {
        LOG_ERR("Failed to initialize raw axes reader, err: %d", err);
        return err;
    }

    if (imu_enabled) {
        err = imu_reader.init();
        if (err)
        {
            LOG_ERR("Failed to initialize IMU reader, err: %d", err);
            return err;
        }
    }
    return 0;
}

int RawInputReader::deinit() {
    int err = 0;

    err = expander_buttons_reader.deinit();
    if (err)
    {
        LOG_ERR("Failed to deinitialize expander buttons reader, err: %d", err);
        return err;
    }

    err = raw_axes_reader.deinit();
    if (err)
    {
        LOG_ERR("Failed to deinitialize raw axes reader, err: %d", err);
        return err;
    }

    if (imu_enabled) {
        err = imu_reader.deinit();
        if (err)
        {
            LOG_ERR("Failed to deinitialize IMU reader, err: %d", err);
            return err;
        }
    }

    err = native_buttons_reader.deinit(); //last because of interrupt on home
    if (err)
    {
        LOG_ERR("Failed to deinitialize native buttons reader, err: %d", err);
        return err;
    }

    return 0;
}

int RawInputReader::getRawData(RawData & raw_data) {
    int err = 0;

    err = native_buttons_reader.getRawData(raw_data.native_buttons);
    if (err)
    {
        LOG_ERR("Failed to read native buttons, err: %d", err);
        return err;
    }

    err = expander_buttons_reader.getRawData(raw_data.expander_buttons);
    if (err)
    {
        LOG_ERR("Failed to read expander buttons, err: %d", err);
        return err;
    }

    err = raw_axes_reader.getRawData(raw_data.raw_axes);
    if (err)
    {
        LOG_ERR("Failed to read raw axes, err: %d", err);
        return err;
    }

    if (imu_enabled) {
        err = imu_reader.getRawData(raw_data.raw_imu);
        if (err)
        {
            LOG_ERR("Failed to read imu, err: %d", err);
            return err;
        }
    }
    return 0;
}

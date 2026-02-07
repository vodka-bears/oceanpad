#pragma once

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

#include "HardwareManager/HardwareTypes.hpp"

class ExpanderButtonsReader final {
public:
    int init();
    int deinit();
    int getRawData(ExpanderButtons & expander_buttons);
private:
    static constexpr i2c_dt_spec expander_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(expander));
    static constexpr gpio_dt_spec exp_reset = GPIO_DT_SPEC_GET(DT_NODELABEL(expander_reset), gpios);
};

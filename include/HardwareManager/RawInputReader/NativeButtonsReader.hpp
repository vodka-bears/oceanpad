#pragma once

#include <zephyr/drivers/gpio.h>

#include "HardwareManager/HardwareTypes.hpp"

class NativeButtonsReader final {
public:
    int init();
    int deinit();
    int getRawData(NativeButtons & native_buttons);
private:
    static constexpr gpio_dt_spec lb = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_lb), gpios);
    static constexpr gpio_dt_spec rb = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_rb), gpios);
    static constexpr gpio_dt_spec ls = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_ls), gpios);
    static constexpr gpio_dt_spec rs = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_rs), gpios);
    static constexpr gpio_dt_spec start = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_start), gpios);
    static constexpr gpio_dt_spec back = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_back), gpios);
    static constexpr gpio_dt_spec home = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_home), gpios);
    static constexpr gpio_dt_spec xd_switch = GPIO_DT_SPEC_GET(DT_NODELABEL(mode_switch), gpios);

    static constexpr const gpio_dt_spec* pins_ptrs[] = { &lb, &rb, &ls, &rs, &start, &back, &home, &xd_switch };
};

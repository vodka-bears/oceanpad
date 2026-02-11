#include "HardwareManager/RawInputReader/NativeButtonsReader.hpp"

int NativeButtonsReader::init() {
    int ret = 0;
    for (const gpio_dt_spec* pin : pins_ptrs) {
        if (!gpio_is_ready_dt(pin))
        {
            return -ENODEV;
        }
        ret = gpio_pin_configure_dt(pin, GPIO_INPUT);
        if (ret) {
            return ret;
        }
    }
    return 0;
}

int NativeButtonsReader::deinit() {
    int ret = 0;
    for (const gpio_dt_spec* pin : pins_ptrs) {
        if (!gpio_is_ready_dt(pin))
        {
            return -ENODEV;
        }
        if (pin == &home) {
            ret = gpio_pin_configure_dt(pin, GPIO_INPUT);
            ret = gpio_pin_interrupt_configure_dt(pin, GPIO_INT_LEVEL_ACTIVE);
        } else {
            ret = gpio_pin_configure_dt(pin, GPIO_DISCONNECTED);
        }
        if (ret)
        {
            return ret;
        }
    }
    return 0;
}

int NativeButtonsReader::getRawData(NativeButtons & native_buttons) {
    native_buttons.lb = gpio_pin_get_dt(&lb);
    native_buttons.rb = gpio_pin_get_dt(&rb);

    native_buttons.ls = gpio_pin_get_dt(&ls);
    native_buttons.rs = gpio_pin_get_dt(&rs);

    native_buttons.start = gpio_pin_get_dt(&start);
    native_buttons.back = gpio_pin_get_dt(&back);
    native_buttons.home = gpio_pin_get_dt(&home);

    native_buttons.xd_switch = gpio_pin_get_dt(&xd_switch);

    return 0;
}

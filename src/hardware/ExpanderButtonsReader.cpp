#include "hardware/ExpanderButtonsReader.hpp"

/* PI4IOE5V6416 I2C Address and Registers */
#define EXP_ADDR        0x20
#define REG_INPUT_P0    0x00
#define REG_INPUT_P1    0x01
#define REG_PUD_EN0     0x46
#define REG_PUD_EN1     0x47
#define REG_PUD_SEL0    0x48
#define REG_PUD_SEL1    0x49

int ExpanderButtonsReader::init() {
    if (!gpio_is_ready_dt(&exp_reset))
    {
        return -ENODEV;
    }
    if (!i2c_is_ready_dt(&expander_i2c))
    {
        return -ENODEV;
    }

    int err = 0;

    err = gpio_pin_configure_dt(&exp_reset, GPIO_INPUT);
    if (err) {
        return err;
    }

    err = gpio_pin_set_dt(&exp_reset, 0);
    if (err) {
        return err;
    }

    /* Enable Pull-up/down resistors */
    err = i2c_reg_write_byte_dt(&expander_i2c, REG_PUD_EN0, 0xFF);
    if (err) {
        return err;
    }

    err = i2c_reg_write_byte_dt(&expander_i2c, REG_PUD_EN1, 0xFF);
    if (err) {
        return err;
    }

    /* Select Pull-DOWN (0x00) for all pins */
    err = i2c_reg_write_byte_dt(&expander_i2c, REG_PUD_SEL0, 0x00);
    if (err) {
        return err;
    }

    err = i2c_reg_write_byte_dt(&expander_i2c, REG_PUD_SEL1, 0x00);
    if (err) {
        return err;
    }

    return 0;
}


int ExpanderButtonsReader::deinit() {
    int err = gpio_pin_set_dt(&exp_reset, 1); //resetting gpio expander pulling the reset line low
    if (err) {
        return err;
    }
    return 0;
}

int ExpanderButtonsReader::getRawData(ExpanderButtons & expander_buttons) {
    int err = i2c_burst_read_dt(&expander_i2c, REG_INPUT_P0, reinterpret_cast<uint8_t*>(&expander_buttons), 2) < 0;
    if (err) {
        return err;
    }
    return 0;
}

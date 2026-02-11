#pragma once

#include <zephyr/drivers/adc.h>

class BatteryGauge final {
public:
    int init();
    int update();
    uint8_t get_battery_percent();
    uint16_t get_vbat_mv_last();
private:
    static inline const size_t VBAT_HISTORY_LEN = 64;
    uint8_t last_history_idx = 0;
    uint16_t voltage_history[VBAT_HISTORY_LEN];

    static constexpr adc_dt_spec vbat_adc_chan = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 6);

    static inline const uint32_t VBAT_SCALE_MUL = 201;
    static inline const uint32_t VBAT_SCALE_DIV = 100;

    struct BatteryLevel {
        uint16_t voltage_mv;
        uint8_t percentage;
    };

    //mixed chemistry, alkaline leaning
    static constexpr const BatteryLevel BATT_LUT[] = {
        {3100, 100},
        {3000, 95},
        {2900, 90},
        {2800, 80},
        {2700, 70},
        {2650, 60},
        {2600, 50},
        {2550, 40},
        {2500, 30},
        {2450, 20},
        {2400, 12},
        {2350, 6},
        {2300, 3},
        {2250, 1},
        {2200, 0}     // functional cutoff
    };

    static constexpr const size_t BATT_LUT_SIZE = sizeof(BATT_LUT) / sizeof(BATT_LUT[0]);
};

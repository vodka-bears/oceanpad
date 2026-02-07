#include <zephyr/logging/log.h>

#include "HardwareManager/BatteryGauge.hpp"

LOG_MODULE_REGISTER(BatteryGauge, LOG_LEVEL_WRN);

int BatteryGauge::init() {
    return adc_channel_setup_dt(&vbat_adc_chan);
}

uint8_t BatteryGauge::get_battery_percent() {
    uint16_t vbat_raw;
    struct adc_sequence seq = {
        .channels    = BIT(6),
        .buffer      = &vbat_raw,
        .buffer_size = sizeof(vbat_raw),
        .resolution  = 12,
        .oversampling = 4,
    };

    int err = adc_read_dt(&vbat_adc_chan, &seq);
    if (err) {
        LOG_ERR("ADC vbat read err: %d", err);
        return 0;
    }

    int32_t batt_mv_unscaled = vbat_raw;

    adc_raw_to_millivolts_dt(&vbat_adc_chan, &batt_mv_unscaled);

    LOG_DBG("Vbat mV unscaled: %u", batt_mv_unscaled);

    uint32_t batt_mv_scaled = VBAT_SCALE_MUL * batt_mv_unscaled / VBAT_SCALE_DIV;

    LOG_DBG("Vbat mV scaled: %u", batt_mv_scaled);

    if (batt_mv_scaled >= BATT_LUT[0].voltage_mv) {
        return 100;
    }
    if (batt_mv_scaled <= BATT_LUT[BATT_LUT_SIZE - 1].voltage_mv) {
        return 0;
    }

    for (size_t i = 0; i < BATT_LUT_SIZE - 1; i++) {
        if (batt_mv_scaled <= BATT_LUT[i].voltage_mv && batt_mv_scaled > BATT_LUT[i+1].voltage_mv) {
            uint32_t volt_high = BATT_LUT[i].voltage_mv;
            uint32_t volt_low  = BATT_LUT[i+1].voltage_mv;
            uint8_t perc_high  = BATT_LUT[i].percentage;
            uint8_t perc_low   = BATT_LUT[i+1].percentage;

            uint32_t offset = batt_mv_scaled - volt_low;
            uint32_t volt_range = volt_high - volt_low;
            uint32_t perc_range = perc_high - perc_low;

            uint8_t result = (uint8_t)(perc_low + (offset * perc_range) / volt_range);
            LOG_DBG("Batt %%: %u", result);
            return result;
        }
    }

    return 0;
}

#include <zephyr/logging/log.h>

#include "HardwareManager/BatteryGauge.hpp"

LOG_MODULE_REGISTER(BatteryGauge, LOG_LEVEL_DBG);

int BatteryGauge::init() {
    int err = adc_channel_setup_dt(&vbat_adc_chan);
    if (err) {
        return err;
    }
    update();
    uint16_t vbat_on_start = voltage_history[last_history_idx];
    //LOG_DBG("last vbat is %d", vbat_on_start);
    for (last_history_idx = 0; last_history_idx < VBAT_HISTORY_LEN; last_history_idx++) {
        voltage_history[last_history_idx] = vbat_on_start;
    }
    last_history_idx = 0;
    return 0;
}

int BatteryGauge::update() {
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
        return err;
    }

    int32_t batt_mv_unscaled = vbat_raw;

    adc_raw_to_millivolts_dt(&vbat_adc_chan, &batt_mv_unscaled);

    //LOG_DBG("Vbat mV unscaled: %u", batt_mv_unscaled);

    last_history_idx += 1;
    if (last_history_idx >= VBAT_HISTORY_LEN)
    {
        last_history_idx = 0;
    }

    voltage_history[last_history_idx] = VBAT_SCALE_MUL * batt_mv_unscaled / VBAT_SCALE_DIV;

    //LOG_DBG("Vbat mV scaled: %u", voltage_history[last_history_idx]);

    return 0;
}

uint8_t BatteryGauge::get_battery_percent() {
    uint32_t voltage_sum = 0;
    for (int it : voltage_history) {
        voltage_sum += it;
    }
    const uint32_t avg_vbat = voltage_sum / VBAT_HISTORY_LEN;
    //LOG_DBG("voltage_sum = %d", voltage_sum);
    //LOG_DBG("avg_vbat = %d", avg_vbat);
    if (avg_vbat >= BATT_LUT[0].voltage_mv) {
        return 100;
    }
    if (avg_vbat <= BATT_LUT[BATT_LUT_SIZE - 1].voltage_mv) {
        return 0;
    }
    for (size_t i = 0; i < BATT_LUT_SIZE - 1; i++) {
        if (avg_vbat <= BATT_LUT[i].voltage_mv && avg_vbat > BATT_LUT[i+1].voltage_mv) {
            uint32_t volt_high = BATT_LUT[i].voltage_mv;
            uint32_t volt_low  = BATT_LUT[i+1].voltage_mv;
            uint8_t perc_high  = BATT_LUT[i].percentage;
            uint8_t perc_low   = BATT_LUT[i+1].percentage;

            uint32_t offset = avg_vbat - volt_low;
            uint32_t volt_range = volt_high - volt_low;
            uint32_t perc_range = perc_high - perc_low;

            uint8_t result = (uint8_t)(perc_low + (offset * perc_range) / volt_range);
            //LOG_DBG("Batt %%: %u", result);
            return result;
        }
    }
    return 0;
}

uint16_t BatteryGauge::get_vbat_mv_last() {
    return voltage_history[last_history_idx];
}

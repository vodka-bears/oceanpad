#pragma once

#include <zephyr/drivers/adc.h>

#include "HardwareManager/HardwareTypes.hpp"

class RawAxesReader final {
public:
    int init();
    int deinit();
    int getRawData(RawAxes & raw_axes);
private:
    static constexpr const device* adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));

    static constexpr const adc_dt_spec lx = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
    static constexpr const adc_dt_spec ly = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
    static constexpr const adc_dt_spec rx = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2);
    static constexpr const adc_dt_spec ry = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3);
    static constexpr const adc_dt_spec lt = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 4);
    static constexpr const adc_dt_spec rt = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 5);

    static constexpr const adc_dt_spec* chan_ptrs[] = { &lx, &ly, &rx, &ry, &lt, &rt };
};

#include "hardware/RawAxesReader.hpp"

int RawAxesReader::init() {
    if (!device_is_ready(adc_dev)) {
        return -ENODEV;
    }
    int err = 0;
    for (const adc_dt_spec*  chan : chan_ptrs) {
        err = adc_channel_setup_dt(chan);
        if (err) {
            return err;
        }
    }
    return 0;
}

int RawAxesReader::deinit() {
    return 0;
}

int RawAxesReader::getRawData(RawAxes & raw_axes) {
    struct adc_sequence seq = {
        .channels       = 0b00111111,
        .buffer         = &raw_axes,
        .buffer_size    = sizeof(raw_axes),
        .resolution     = 12,
    };
    return adc_read(adc_dev, &seq);
}

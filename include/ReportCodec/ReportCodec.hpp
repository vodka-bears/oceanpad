#pragma once
#include <zephyr/types.h>

template <typename TInput, typename TOutput/*, typename TFeature*/>
class ReportCodec {
public:
    virtual ~ReportCodec() = default;

    virtual int encode_input(uint8_t& report_id, const TInput& data, uint8_t* report_buffer, uint16_t buffer_len) const = 0;
    virtual int decode_output(uint8_t report_id, TOutput& data, const uint8_t* report_buffer, uint16_t report_len) const = 0;

    //virtual int encode_feature(uint8_t report_id, const TFeature& data, uint8_t* report_buffer, uint16_t report_len) const = 0;
    //virtual int decode_feature(uint8_t report_id, TFeature& data, const uint8_t* report_buffer, uint16_t report_len) const = 0;
};

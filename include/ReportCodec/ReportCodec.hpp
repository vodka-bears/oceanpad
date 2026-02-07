#pragma once
#include <zephyr/types.h>

template <typename TInput, typename TOutput>
class ReportCodec {
public:
    virtual ~ReportCodec() = default;

    virtual uint8_t encode_input(uint8_t report_id, const TInput& data, uint8_t* report_buffer, uint16_t report_len) const = 0;
    virtual uint8_t decode_output(uint8_t report_id, TOutput& data, const uint8_t* report_buffer, uint16_t report_len) const = 0;
};

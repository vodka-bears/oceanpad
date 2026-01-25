#pragma once
#include <zephyr/types.h>

template <typename Tinput, typename TOutput>
class ReportCodec {
public:
    virtual ~ReportCodec() = default;

    virtual uint8_t encode_input(uint8_t report_id, const Tinput& data, uint8_t* buffer, uint16_t buffer_len) const = 0;
    virtual TOutput decode_output(uint8_t report_id, const uint8_t* report_buffer, uint16_t report_len) const = 0;
};

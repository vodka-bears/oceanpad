#pragma once

#include <zephyr/types.h>

#include "ReportCodec/ReportCodecXbox.hpp"
#include "ReportCodec/ReportCodec8BitDo.hpp"
#include "hid_report_maps.hpp"

using ReportCodecOceanPad = ReportCodec<GamepadState, VibrationDataXbox>;

struct PnpID {
    uint8_t  src;
    uint16_t vid;
    uint16_t pid;
    uint16_t ver;
}  __packed;

struct DisConfig {
    PnpID pnp_id;
    const char* manufacturer;
    const char* model;
    const char* serial;
    const char* fw_rev;
};

enum class HidReportType : uint8_t {
    Input = 0x01,
    Output = 0x02,
    Feature = 0x03,
};

struct ReportRef {
    uint8_t id;
    HidReportType type;
} __packed;

struct HidConfig {
    const uint8_t* report_map;
    size_t report_map_size;
    const ReportRef* reports;
    uint8_t report_count;
};

struct DeviceConfig {
    const char* name;
    uint16_t appearance;
    DisConfig dis_config;
    HidConfig hid_config;
    const ReportCodecOceanPad* report_codec;
    uint32_t interval_us;
};

static inline const ReportCodecXbox xbox_codec;
static inline const ReportCodec8BitDo abitdo_codec;

static inline const ReportRef xbox_reports[] = {
    { 0x01, HidReportType::Input },
    { 0x03, HidReportType::Output }
};

static inline const ReportRef abitdo_reports[] = {
    { 0x01, HidReportType::Input },
    { 0x05, HidReportType::Output }
};

static inline const DeviceConfig oceanpad_config_variants[] = {
    {
        .name = "Xbox Wireless Controller",
        .appearance = 964,
        .dis_config = {
            .pnp_id = {
                .src = 2,
                .vid = 0x045E,
                .pid = 0x0B13,
                .ver = 0x0523,
            },
            .manufacturer = "Microsoft",
            .model = nullptr,
            .serial = "18062023",
            .fw_rev = "OceanPad",
        },
        .hid_config = {
            .report_map = report_map_xbox,
            .report_map_size = ARRAY_SIZE(report_map_xbox),
            .reports = xbox_reports,
            .report_count = ARRAY_SIZE(xbox_reports),
        },
        .report_codec = &xbox_codec,
        .interval_us = 7500,
    },
    {
        .name = "OceanPad",
        .appearance = 964,
        .dis_config = {
            .pnp_id = {
                .src = 2,
                .vid = 0x2DC8,
                .pid = 0x6012,
                .ver = 0x0001,
            },
            .manufacturer = "Vodka Bears",
            .model = nullptr,
            .serial = "18062023",
            .fw_rev = "OceanPad",
        },
        .hid_config = {
            .report_map = report_map_8bitdo,
            .report_map_size = ARRAY_SIZE(report_map_8bitdo),
            .reports = abitdo_reports,
            .report_count = ARRAY_SIZE(abitdo_reports),
        },
        .report_codec = &abitdo_codec,
        .interval_us = 8333,
    },
};

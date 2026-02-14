#pragma once

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>

#include "DeviceConfig.hpp"

#define MAX_REPORTS 8
#define MAX_REPORT_LEN CONFIG_BT_L2CAP_TX_MTU - 3

enum class BleServiceState : uint8_t {
    Idle,
    AdvertisingDiscoverable,
    AdvertisingUndiscoverable,
    Connected,
    ConnectedAdvertising,
};

struct ReportTableEntry {
    ReportRef ref;
    uint8_t attr_idx;
    uint8_t report_cache[MAX_REPORT_LEN];
    uint8_t report_cache_len;
};

class HidServiceCallbacks {
public:
    virtual void on_incoming_report(uint8_t report_id, const uint8_t* data, uint16_t len) const {}
    virtual void on_connected() const {}
    virtual void on_disconnected(bool graceful) const {}
    virtual void on_adv_timeout(bool discoverable) const {}
};

class BleService {
public:
    int init(const DeviceConfig* device_config);
    int set_identity(uint8_t bt_id);

    void set_callbacks(const HidServiceCallbacks* new_callbacks) { hid_callbacks = new_callbacks; }

    int start_advertising(bool discoverable);
    int stop_advertising();
    int update_report(uint8_t report_id, const uint8_t* data, uint16_t len);
    int update_battery_level(uint8_t level);

    BleServiceState get_state() const { return current_state; }
    void do_disconnect();
    bool has_bonded_peer();
    void clear_bonded_peers();

    // BAS Callbacks
    static ssize_t read_battery_level_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len, uint16_t offset);
    static void battery_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
    // DIS Callbacks
    static ssize_t read_dis_pnp_id(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
    static ssize_t read_dis_string(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset);
    // HID Callbacks
    static ssize_t read_info_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset);
    static ssize_t write_ctrl_point_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                       const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
    static ssize_t read_report_map_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset);
    static ssize_t read_report_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset);
    static ssize_t write_report_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
    static ssize_t read_report_ref_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset);
private:
    static void add_characteristic(
        struct bt_gatt_attr *attr_array, uint8_t &a_idx,
        struct bt_gatt_chrc *chrc_pool, uint8_t &c_idx,
        const struct bt_uuid *uuid, uint8_t props, uint16_t perm,
        bt_gatt_attr_read_func_t read, bt_gatt_attr_write_func_t write, void *user_data
    );
    int init_dis(const DisConfig* dis);
    int init_hid(const HidConfig* hid);
    void init_advertising();
    void restart_advertising();

    uint8_t last_battery_level;

    uint8_t current_bt_id;

    bt_gatt_exchange_params exchange_params;

    const DisConfig* dis_ptr;

    struct bt_gatt_attr dis_attrs[20];
    struct bt_gatt_chrc dis_chrcs[9];
    struct bt_gatt_service dis_svc;

    struct bt_gatt_attr hids_attrs[8 + (MAX_REPORTS * 4)];
    struct bt_gatt_chrc hids_chrcs[MAX_REPORTS + 3];
    struct bt_gatt_ccc_managed_user_data hids_ccc_pool[MAX_REPORTS];
    struct bt_gatt_service hids_svc;

    ReportTableEntry report_storage[MAX_REPORTS];
    uint8_t report_count;

    const uint8_t* current_report_map;
    uint16_t current_report_map_size;

    uint16_t appearance;
    static inline const uint8_t uuids[] = {
        BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
        BT_UUID_16_ENCODE(BT_UUID_DIS_VAL),
        BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)
    };
    struct bt_data ad_discoverable[3] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA(BT_DATA_UUID16_ALL, uuids, sizeof(uuids)),
        BT_DATA(0, 0, 0), //to be filled in the code
    };
    struct bt_data sd_discoverable[1]= {
        BT_DATA(0, 0, 0), //to be filled in the code
    };
    static inline const struct bt_data ad_undiscoverable[1] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    };

    struct bt_le_adv_param adv_param_discoverable  = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY,
        BT_GAP_ADV_FAST_INT_MIN_1,
        BT_GAP_ADV_FAST_INT_MAX_1,
        nullptr
    );
    struct bt_le_adv_param adv_param_undiscoverable = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_FILTER_SCAN_REQ,
        BT_GAP_ADV_FAST_INT_MIN_1,
        BT_GAP_ADV_FAST_INT_MAX_1,
        nullptr
    );

    BleServiceState current_state;
    struct bt_conn *current_conn;

    struct k_work_delayable adv_timeout_work;
    static void on_adv_timeout(struct k_work *work);

    struct k_mutex report_mutex;
    uint8_t input_report_cache[MAX_REPORT_LEN];
    uint16_t input_report_cache_len;

    const HidServiceCallbacks* hid_callbacks{ nullptr };

    bool mtu_negotiated{ false };
    bool battery_notify_enable{ false };

    static inline const uint16_t ADV_TIMEOUT_S = 60;
};

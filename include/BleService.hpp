#pragma once

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>

#define MAX_REPORTS 8
#define MAX_REPORT_LEN CONFIG_BT_L2CAP_TX_MTU - 3

enum class BleServiceState : uint8_t {
    Idle,
    AdvertisingDiscoverable,
    AdvertisingUndiscoverable,
    Connected,
    ConnectedAdvertising,
};

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

struct ReportTableEntry {
    ReportRef ref;
    uint8_t attr_idx;
    uint8_t report_cache[MAX_REPORT_LEN];
    uint8_t report_cache_len;
};

struct HidConfig {
    const uint8_t* report_map;
    size_t report_map_size;
    const ReportRef* report_buf;
    uint8_t report_count;
};

struct DeviceInfo {
    const char* name;
    uint16_t appearance;
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
    int init(const DeviceInfo* device_info, const DisConfig* dis, const HidConfig* hid);
    int set_identity(uint8_t bt_id);

    void set_callbacks(const HidServiceCallbacks* new_callbacks) { hid_callbacks = new_callbacks; }

    int start_advertising(bool discoverable);
    int update_report(uint8_t report_id, const uint8_t* data, uint16_t len);
    int update_battery_level(uint8_t level);

    BleServiceState get_state() const { return current_state; }
    void do_disconnect();
    bool has_bonded_peer();
    void clear_bonded_peers();

    // BAS Callbacks
    static ssize_t read_battery_level_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len, uint16_t offset);
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
    struct bt_data ad_discoverable[3];
    struct bt_data sd[1];

    BleServiceState current_state;
    struct bt_conn *current_conn;

    struct k_work_delayable adv_timeout_work;
    static void on_adv_timeout(struct k_work *work);

    struct k_mutex report_mutex;
    uint8_t input_report_cache[MAX_REPORT_LEN];
    uint16_t input_report_cache_len;

    const HidServiceCallbacks* hid_callbacks{ nullptr };

    static inline const uint16_t ADV_TIMEOUT_S = 60;
};

#pragma once

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/mutex.h>

#define MAX_REPORTS 2
#define MAX_REPORT_LEN 64
#define MAX_REPORT_MAP_SIZE 512

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

class BleService {
public:
    using OutputReportCallback = void (*)(uint8_t report_id, const uint8_t* data, uint16_t len);
    using SimpleCallback = void (*)();

    int init(const DeviceInfo* device_info, const DisConfig* dis, const HidConfig* hid);
    int set_identity(uint8_t bt_id);
    void set_output_report_callback(OutputReportCallback cb) { output_cb = cb; }
    void set_status_callbacks(SimpleCallback on_connected,
                              SimpleCallback on_disconnected_graceful,
                              SimpleCallback on_connection_lost,
                              SimpleCallback on_advertising_discoverable_end,
                              SimpleCallback on_advertising_undiscoverable_end) {
        connected_app_cb = on_connected;
        disconnected_graceful_app_cb = on_disconnected_graceful;
        connection_lost_app_cb = on_connection_lost;
        advertising_discoverable_end_cb = on_advertising_discoverable_end;
        advertising_undiscoverable_end_cb = on_advertising_undiscoverable_end;
    }

    int start_advertising_discoverable();
    int start_advertising_undiscoverable();
    int send_input_report(uint8_t report_id, const uint8_t* data, uint16_t len);
    int update_battery_level(uint8_t level);

    BleServiceState get_state() const { return current_state; }
    void do_disconnect();
    bool has_bonded_peer();
    void clear_bonded_peers();

    static OutputReportCallback output_cb;
    static uint8_t last_battery_level;

    // GATT Callbacks
    static ssize_t read_input_report_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset);
    static ssize_t read_report_map_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset);
    static ssize_t read_info_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset);
    static ssize_t write_output_report_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
    static ssize_t read_report_ref_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset);
    static ssize_t write_ctrl_point_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                       const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
    static ssize_t read_battery_level_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len, uint16_t offset);
    static ssize_t read_dis_pnp_id(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
    static ssize_t read_dis_string(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset);

private:
    int init_services(const DisConfig* dis, const HidConfig* hid);
    static void restart_advertising();

    static uint8_t current_bt_id;

    static DisConfig saved_dis;

    static struct bt_gatt_attr dis_attrs[20];
    static struct bt_gatt_attr hids_attrs[8 + (MAX_REPORTS * 4)];
    static struct bt_gatt_chrc dis_chrcs[9];
    static struct bt_gatt_chrc hids_chrcs[MAX_REPORTS + 3];
    static struct bt_gatt_ccc_managed_user_data hids_ccc_pool[MAX_REPORTS];
    static struct bt_gatt_service dis_svc;
    static struct bt_gatt_service hids_svc;

    static ReportTableEntry report_storage[MAX_REPORTS];
    static uint8_t report_count;

    static uint8_t current_report_map[MAX_REPORT_MAP_SIZE];
    static uint16_t current_report_map_size;

    static uint16_t appearance;
    static struct bt_data ad_discoverable[3];
    static struct bt_data sd[1];

    static BleServiceState current_state;
    static struct bt_conn *current_conn;

    static struct k_work_delayable adv_timeout_work;
    static void on_adv_timeout(struct k_work *work);

    static struct sys_mutex report_mutex;
    static uint8_t input_report_cache[MAX_REPORT_LEN];
    static uint16_t input_report_cache_len;

    static SimpleCallback connected_app_cb;
    static SimpleCallback disconnected_graceful_app_cb;
    static SimpleCallback connection_lost_app_cb;
    static SimpleCallback advertising_discoverable_end_cb;
    static SimpleCallback advertising_undiscoverable_end_cb;

    static inline const uint16_t ADV_TIMEOUT_S = 60;
};

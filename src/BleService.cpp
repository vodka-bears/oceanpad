#include "BleService.hpp"
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_service, LOG_LEVEL_DBG);

// --- Static HID Data ---
static const uint8_t hid_info[4] = { 0x01, 0x01, 0x00, 0x01 };

uint8_t BleService::current_bt_id = 0;

DisConfig BleService::saved_dis;

struct bt_gatt_attr BleService::dis_attrs[20];
struct bt_gatt_attr BleService::hids_attrs[8 + (MAX_REPORTS * 4)];
struct bt_gatt_chrc BleService::dis_chrcs[9];
struct bt_gatt_chrc BleService::hids_chrcs[MAX_REPORTS + 3];
struct bt_gatt_ccc_managed_user_data BleService::hids_ccc_pool[MAX_REPORTS];
struct bt_gatt_service BleService::dis_svc;
struct bt_gatt_service BleService::hids_svc;

ReportTableEntry BleService::report_storage[MAX_REPORTS];
uint8_t BleService::report_count;

uint8_t BleService::current_report_map[MAX_REPORT_MAP_SIZE];
uint16_t BleService::current_report_map_size = 0;

uint16_t BleService::appearance;
struct bt_data BleService::ad_discoverable[3];
struct bt_data BleService::sd[1];

struct bt_conn* BleService::current_conn = nullptr;
BleServiceState BleService::current_state = BleServiceState::Idle;
BleService::OutputReportCallback BleService::output_cb = nullptr;
struct sys_mutex BleService::report_mutex;

uint8_t BleService::last_battery_level = 0xFF;
struct k_work_delayable BleService::adv_timeout_work;

// Инициализация коллбеков приложения
BleService::SimpleCallback BleService::connected_app_cb = nullptr;
BleService::SimpleCallback BleService::disconnected_graceful_app_cb = nullptr;
BleService::SimpleCallback BleService::connection_lost_app_cb = nullptr;
BleService::SimpleCallback BleService::advertising_discoverable_end_cb = nullptr;
BleService::SimpleCallback BleService::advertising_undiscoverable_end_cb = nullptr;

// --- GATT Callbacks Implementation ---

ssize_t BleService::read_report_map_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, current_report_map, current_report_map_size);
}

ssize_t BleService::read_info_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, hid_info, sizeof(hid_info));
}

ssize_t BleService::read_input_report_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len, uint16_t offset) {

    ReportTableEntry* entry = static_cast<ReportTableEntry*>(attr->user_data);
    if (!entry) {
        LOG_ERR("Filed to read report %d", entry->ref.id);
        return -EINVAL;
    }

    ssize_t read_len;
    sys_mutex_lock(&report_mutex, K_FOREVER);

    // Читаем из кэша конкретного репорта
    read_len = bt_gatt_attr_read(conn, attr, buf, len, offset,
                                 entry->report_cache, entry->report_cache_len);

    sys_mutex_unlock(&report_mutex);
    return read_len;
}

ssize_t BleService::write_output_report_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
   ReportTableEntry* entry = static_cast<ReportTableEntry*>(attr->user_data);

    // 1. Базовые проверки
    if (!entry) return -EINVAL;
    if (offset + len > MAX_REPORT_LEN) return -BT_ATT_ERR_INVALID_OFFSET;

    sys_mutex_lock(&report_mutex, K_FOREVER);

    memcpy(entry->report_cache + offset, buf, len);
    entry->report_cache_len = offset + len;

    sys_mutex_unlock(&report_mutex);

     if (output_cb) {
        output_cb(entry->ref.id, (const uint8_t*)buf, len);
    }

    return len;
}

ssize_t BleService::read_report_ref_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, 2);
}

ssize_t BleService::write_ctrl_point_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                        const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    return len;
}

ssize_t BleService::read_battery_level_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len, uint16_t offset) {
    uint8_t lvl = last_battery_level;
    if (lvl == 0xFF) lvl = 50;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &lvl, sizeof(lvl));
}


ssize_t BleService::read_dis_pnp_id(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    if (!attr->user_data) {
        return 0;
    }
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(PnpID));
}

ssize_t BleService::read_dis_string(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, strlen((char*)attr->user_data));
}

BT_GATT_SERVICE_DEFINE(bas_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
    BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           BleService::read_battery_level_cb, NULL, &BleService::last_battery_level),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);


static const struct bt_data ad_undiscoverable[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, 0),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL)),
};



int BleService::init(const DeviceInfo* device_info, const DisConfig* dis, const HidConfig* hid) {
    init_services(dis, hid);

    int err = bt_enable(NULL);
    if (err) return err;

    bt_set_appearance(device_info->appearance);

    if (device_info->name)
    {
        err = bt_set_name(device_info->name);
        if (err)
        {
            LOG_WRN("Setting name failed, err %d", err);
        }
    }

    sys_mutex_init(&report_mutex);

    static struct bt_conn_cb conn_callbacks = {
        .connected = [](struct bt_conn *conn, uint8_t err) {
            if (err) {
                LOG_ERR("Connection failed (err %u)", err);
                return;
            }

            bt_conn_ref(conn);
            LOG_DBG("Connected (ref captured). Waiting for security...");
        },

        .disconnected = [](struct bt_conn *conn, uint8_t reason) {
            LOG_DBG("Disconnected (reason %u)", reason);

            // Если это было наше активное соединение
            if (current_conn == conn) {
                bt_conn_unref(current_conn);
                current_conn = nullptr;
                current_state = BleServiceState::Idle;

                if (reason == BT_HCI_ERR_REMOTE_USER_TERM_CONN) {
                    if (disconnected_graceful_app_cb) disconnected_graceful_app_cb();
                } else {
                    if (connection_lost_app_cb) connection_lost_app_cb();
                }
            } else {
                bt_conn_unref(conn);
                restart_advertising();
            }
        },

        .security_changed = [](struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
            if (err) {
                LOG_ERR("Security failed (err %d). Disconnecting...", err);
                bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
                return;
            }

            LOG_DBG("Security changed: level %u", level);

            if (level >= BT_SECURITY_L2) {
                // Если уже есть кто-то другой — выгоняем старого
                if (current_conn != nullptr && current_conn != conn) {
                    LOG_DBG("Replacing old connection with new bonded peer");
                    bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                    bt_conn_unref(current_conn);
                }

                current_conn = conn;
                current_state = BleServiceState::Connected;

                k_work_cancel_delayable(&adv_timeout_work);
                if (connected_app_cb) connected_app_cb();
            }
        },
    };
    bt_conn_cb_register(&conn_callbacks);
    static struct bt_conn_auth_info_cb auth_info_callbacks = {
        .pairing_complete = [](struct bt_conn *conn, bool bonded) {
            if (!bonded) {
                return;
            } //now remove all unneded bonds
            const bt_addr_le_t *curr_addr = bt_conn_get_dst(conn);
            bt_foreach_bond(current_bt_id, [](const struct bt_bond_info *info, void *user_data) {
                const bt_addr_le_t *current = static_cast<const bt_addr_le_t *>(user_data);
                if (bt_addr_le_cmp(&info->addr, current) != 0) {
                    bt_unpair(current_bt_id, &info->addr);
                }
            }, const_cast<bt_addr_le_t *>(curr_addr));
        },
    };
    bt_conn_auth_info_cb_register(&auth_info_callbacks);
    k_work_init_delayable(&adv_timeout_work, on_adv_timeout);
    LOG_DBG("BLE init finished");
    return 0;
}

int BleService::set_identity(uint8_t bt_id) {
    // --- Логика управления MAC адресом через Identity ---
    current_bt_id = bt_id;
    if (current_bt_id > 0) {
        bt_addr_le_t base_addr;
        size_t count = 1;

        // Получаем адрес по умолчанию (Identity 0)
        bt_id_get(&base_addr, &count);

        LOG_HEXDUMP_DBG(base_addr.a.val, 6, "Current mac: ");

        // Увеличиваем MAC на значение bt_id
        // Простая арифметика для младшего байта (Little Endian)
        uint32_t carry = current_bt_id;
        for (int i = 0; i < 6; i++) {
            uint32_t val = base_addr.a.val[i] + carry;
            base_addr.a.val[i] = (uint8_t)val;
            carry = val >> 8;
            if (carry == 0) break;
        }

        base_addr.type = BT_ADDR_LE_RANDOM;
        base_addr.a.val[5] |= 0xC0;

        LOG_HEXDUMP_DBG(base_addr.a.val, 6, "New mac: ");

        int id_ret = bt_id_create(&base_addr, NULL);
        if (id_ret < 0) {
            if (id_ret == -EALREADY) {
                LOG_DBG("Identity %d already exists, resetting", current_bt_id);
                return bt_id_reset(current_bt_id, &base_addr, NULL);
            } else {
                LOG_ERR("Failed to create identity %d (err %d). Check CONFIG_BT_ID_MAX.", current_bt_id, id_ret);
                return id_ret;
            }
        } else {
            if (id_ret != current_bt_id) {
                LOG_WRN("Identity created with index %d, but %d was requested", id_ret, current_bt_id);
                current_bt_id = (uint8_t)id_ret;
            }
        }
        return 0;
    }
    return 0;
}

int BleService::init_services(const DisConfig* dis, const HidConfig* hid) {
    if (!dis || !hid) {
        LOG_ERR("dis or hid is NULL!");
        return -EINVAL;
    }
    if (hid->report_map_size > MAX_REPORT_MAP_SIZE)
    {
        LOG_ERR("Report map of size %d provided while only %d is supported, error", hid->report_map_size, MAX_REPORTS);
        return -EINVAL;
    }

    saved_dis = *dis;

    auto add_characteristic = [](struct bt_gatt_attr *attr_array, uint8_t &a_idx,
                                struct bt_gatt_chrc *chrc_pool, uint8_t &c_idx,
                                const struct bt_uuid *uuid, uint8_t props,
                                bt_gatt_attr_read_func_t read,
                                bt_gatt_attr_write_func_t write, void *user_data) {

        chrc_pool[c_idx].uuid = uuid;
        chrc_pool[c_idx].value_handle = 0U;
        chrc_pool[c_idx].properties = props;

        attr_array[a_idx++] = BT_GATT_ATTRIBUTE(BT_UUID_GATT_CHRC,
                                BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
                                &chrc_pool[c_idx]);

        attr_array[a_idx++] = BT_GATT_ATTRIBUTE(uuid,
                                (uint16_t)((read ? BT_GATT_PERM_READ : 0) | (write ? BT_GATT_PERM_WRITE : 0)),
                                read, write, user_data);

        c_idx++;
    };
    memset(dis_attrs, 0, sizeof(dis_attrs));
    memset(dis_chrcs, 0, sizeof(dis_chrcs));
    memset(hids_attrs, 0, sizeof(hids_attrs));
    memset(hids_chrcs, 0, sizeof(hids_chrcs));
    uint8_t d_idx = 0;
    uint8_t d_c_idx = 0;
    dis_attrs[d_idx++] = BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS);
    add_characteristic(dis_attrs, d_idx, dis_chrcs, d_c_idx,
                       BT_UUID_DIS_PNP_ID, BT_GATT_CHRC_READ,
                       read_dis_pnp_id, NULL, (void*)(&saved_dis.pnp_id));
    char uuid_str[32];
    bt_uuid_to_str(dis_attrs[0].uuid, uuid_str, 32);
    bt_uuid_to_str(dis_attrs[1].uuid, uuid_str, 32);
    bt_uuid_to_str(dis_attrs[2].uuid, uuid_str, 32);
    if (saved_dis.manufacturer) {
        add_characteristic(dis_attrs, d_idx, dis_chrcs, d_c_idx,
                           BT_UUID_DIS_MANUFACTURER_NAME, BT_GATT_CHRC_READ,
                           read_dis_string, NULL, (void*)saved_dis.manufacturer);
    }
    if (saved_dis.model) {
        add_characteristic(dis_attrs, d_idx, dis_chrcs, d_c_idx,
                           BT_UUID_DIS_MODEL_NUMBER, BT_GATT_CHRC_READ,
                           read_dis_string, NULL, (void*)saved_dis.model);
    }
    if (saved_dis.serial) {
        add_characteristic(dis_attrs, d_idx, dis_chrcs, d_c_idx,
                           BT_UUID_DIS_SERIAL_NUMBER, BT_GATT_CHRC_READ,
                           read_dis_string, NULL, (void*)saved_dis.serial);
    }
    if (saved_dis.fw_rev) {
        add_characteristic(dis_attrs, d_idx, dis_chrcs, d_c_idx,
                           BT_UUID_DIS_FIRMWARE_REVISION, BT_GATT_CHRC_READ,
                           read_dis_string, NULL, (void*)saved_dis.fw_rev);
    }
    dis_svc.attrs = dis_attrs;
    dis_svc.attr_count = d_idx;
    bt_gatt_service_register(&dis_svc);



    uint8_t h_idx = 0;
    uint8_t h_c_idx = 0;
    hids_attrs[h_idx++] = BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS);
    add_characteristic(hids_attrs, h_idx, hids_chrcs, h_c_idx,
                       BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,
                       read_info_cb, NULL, NULL);
    add_characteristic(hids_attrs, h_idx, hids_chrcs, h_c_idx,
                       BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                       NULL, write_ctrl_point_cb, NULL);
    add_characteristic(hids_attrs, h_idx, hids_chrcs, h_c_idx,
                       BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
                       read_report_map_cb, NULL, NULL);
    report_count = hid->report_count;
    if (report_count > MAX_REPORTS) {
        LOG_WRN("Too many reports, truncating to %d", MAX_REPORTS);
        report_count = MAX_REPORTS;
    }

    uint8_t ccc_idx = 0;
    for (uint8_t i = 0; i < report_count; i++) {
        report_storage[i].ref = hid->report_buf[i];
        report_storage[i].report_cache_len = 0;

        if (report_storage[i].ref.type == HidReportType::Input) {
            add_characteristic(hids_attrs, h_idx, hids_chrcs, h_c_idx,
                               BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                               read_input_report_cb, NULL, &report_storage[i]);

            report_storage[i].attr_idx = h_idx - 1;
            hids_ccc_pool[ccc_idx] = BT_GATT_CCC_MANAGED_USER_DATA_INIT(NULL, NULL, NULL);
            hids_attrs[h_idx++] = BT_GATT_CCC_MANAGED(&hids_ccc_pool[ccc_idx++], BT_GATT_PERM_READ | BT_GATT_PERM_WRITE);
            ccc_idx++;
        }
        else {
            add_characteristic(hids_attrs, h_idx, hids_chrcs, h_c_idx,
                               BT_UUID_HIDS_REPORT,
                               BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                               read_input_report_cb, write_output_report_cb, &report_storage[i]);

            report_storage[i].attr_idx = h_idx - 1;
        }

        hids_attrs[h_idx++] = BT_GATT_ATTRIBUTE(BT_UUID_HIDS_REPORT_REF,
                                BT_GATT_PERM_READ, read_report_ref_cb, NULL,
                                &report_storage[i].ref);
    }
    hids_svc.attrs = hids_attrs;
    hids_svc.attr_count = h_idx;
    bt_gatt_service_register(&hids_svc);
    memcpy(current_report_map, hid->report_map, hid->report_map_size);
    current_report_map_size = hid->report_map_size;
    return 0;
}

int BleService::start_advertising_discoverable() {
    //if (current_state == BleServiceState::Connected) {
    //    do_disconnect();
    //    k_msleep(100);
    //}
    bt_le_adv_stop();


    static const uint8_t adv_flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
    static const uint8_t uuids[] = {
        BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
        BT_UUID_16_ENCODE(BT_UUID_DIS_VAL),
        BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)
    };

    ad_discoverable[0].type = BT_DATA_FLAGS;
    ad_discoverable[0].data_len = 1;
    ad_discoverable[0].data = &adv_flags;

    appearance = bt_get_appearance();
    ad_discoverable[1].type = BT_DATA_GAP_APPEARANCE;
    ad_discoverable[1].data_len = sizeof(appearance);
    ad_discoverable[1].data = (const uint8_t *)&appearance;

    ad_discoverable[2].type = BT_DATA_UUID16_ALL;
    ad_discoverable[2].data_len = sizeof(uuids);
    ad_discoverable[2].data = uuids;

    // Scan Response (Имя)
    const char* name = bt_get_name();
    sd[0].type = BT_DATA_NAME_COMPLETE;
    sd[0].data_len = (uint8_t)strlen(name);
    sd[0].data = (const uint8_t *)name;

    struct bt_le_adv_param adv_param = *BT_LE_ADV_CONN_FAST_1;
    adv_param.id = current_bt_id;

    LOG_DBG("Advertising discoverable");
    int ret = bt_le_adv_start(&adv_param, ad_discoverable, ARRAY_SIZE(ad_discoverable), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Failed to begin adverising, err %d", ret);
    } else {
        current_state = current_conn ? BleServiceState::ConnectedAdvertising : BleServiceState::AdvertisingDiscoverable;
        k_work_reschedule(&adv_timeout_work, K_SECONDS(ADV_TIMEOUT_S));
    }
    return ret;
}

int BleService::start_advertising_undiscoverable() {
    //if (current_state == BleServiceState::Connected) {
    //    do_disconnect();
    //    k_msleep(100);
    //}
    bt_le_adv_stop();
    bt_le_filter_accept_list_clear();
    bt_foreach_bond(current_bt_id, [](const struct bt_bond_info *info, void *) {
        bt_le_filter_accept_list_add(&info->addr);
    }, nullptr);

    struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_FILTER_SCAN_REQ,
        BT_GAP_ADV_FAST_INT_MIN_1,
        BT_GAP_ADV_FAST_INT_MAX_1,
        nullptr
    );
    adv_param.id = current_bt_id;
    LOG_DBG("Advertising undiscoverable");
    int ret = bt_le_adv_start(&adv_param, ad_undiscoverable, ARRAY_SIZE(ad_undiscoverable), nullptr, 0);
    if (ret) {
        LOG_ERR("Failed to begin adverising, err %d", ret);
    } else {
        current_state = BleServiceState::AdvertisingUndiscoverable;
        k_work_reschedule(&adv_timeout_work, K_SECONDS(ADV_TIMEOUT_S));
    }
    return ret;
}

int BleService::send_input_report(uint8_t report_id, const uint8_t* data, uint16_t len) {
    if (current_state < BleServiceState::Connected || !current_conn) return -ENOTCONN;

    if (len > MAX_REPORT_LEN)
    {
        LOG_WRN("Report of len %d provided while maximum %d is supported, ignoring", len, MAX_REPORT_LEN);
        return -EINVAL;
    }

    ReportTableEntry* report_entry = nullptr;

    for (int i = 0; i < report_count; i++) {
        if (report_storage[i].ref.id == report_id) {
            if (report_storage[i].ref.type != HidReportType::Input) {
                LOG_WRN("Report id %d is not input, ignoring", report_id);
                return -EINVAL;
            }
            report_entry = &report_storage[i];
            break;
        }
    }
    if (report_entry == nullptr) {
        LOG_WRN("Report id %d is not registered, ignoring", report_id);
        return -EINVAL;
    }

    sys_mutex_lock(&report_mutex, K_FOREVER);
    if (report_entry->report_cache_len == len && memcmp(data, report_entry->report_cache, len) == 0)
    {
        sys_mutex_unlock(&report_mutex);
        return 0;
    }

    report_entry->report_cache_len = len;

    memcpy(report_entry->report_cache, data, report_entry->report_cache_len);
    sys_mutex_unlock(&report_mutex);
    return bt_gatt_notify(current_conn, &hids_svc.attrs[report_entry->attr_idx], data, len);
}

int BleService::update_battery_level(uint8_t level) {
    if (level > 100) level = 100;

    if (level == last_battery_level) {
        return 0;
    }

    last_battery_level = level;

    if (current_state != BleServiceState::Connected || !current_conn) {
        return -ENOTCONN;
    }

    int err = bt_gatt_notify(current_conn, &bas_svc.attrs[1], &last_battery_level, sizeof(last_battery_level));
    //int err = 0;

    if (!err) {
        LOG_DBG("Battery level updated: %u%%", level);
    }

    return err;
}

void BleService::do_disconnect() {
    if (current_conn) {
        LOG_DBG("Initiating purposeful disconnect...");

        int err = bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        if (err) {
            LOG_ERR("Failed to disconnect (err %d)", err);
        } else {
            LOG_DBG("Disconnect command sent successfully");
        }
    } else {
        LOG_WRN("Disconnect requested, but no active connection");
    }
}

bool BleService::has_bonded_peer() {
    uint8_t count = 0;
    bt_foreach_bond(current_bt_id, [](const struct bt_bond_info *info, void *user_data) {
        *static_cast<uint8_t*>(user_data) = 1;
    }, &count);
    return count > 0;
}

void BleService::clear_bonded_peers() {
    bt_le_adv_stop();
    int err = bt_unpair(current_bt_id, BT_ADDR_LE_ANY);
    if (err) LOG_ERR("Clear peers failed: %d", err);
}

void BleService::on_adv_timeout(struct k_work *work) {
    LOG_DBG("Advertising timeout reached, current_state: %d", static_cast<uint8_t>(current_state));
    bt_le_adv_stop();
    BleServiceState old_state = current_state;
    current_state = current_conn ? BleServiceState::Connected : BleServiceState::Idle;
    if ((old_state == BleServiceState::AdvertisingDiscoverable || old_state == BleServiceState::ConnectedAdvertising) && advertising_discoverable_end_cb) {
        advertising_discoverable_end_cb();
    }
    else if (old_state == BleServiceState::AdvertisingUndiscoverable && advertising_undiscoverable_end_cb) {
        advertising_undiscoverable_end_cb();
    }
}

void BleService::restart_advertising() {
    bt_le_adv_stop();
    int ret;
    if (current_state == BleServiceState::AdvertisingDiscoverable || current_state == BleServiceState::ConnectedAdvertising) {
        LOG_DBG("Restarting discoverable adverising");
        struct bt_le_adv_param adv_param = *BT_LE_ADV_CONN_FAST_1;
        adv_param.id = current_bt_id;
        ret = bt_le_adv_start(&adv_param, ad_discoverable, ARRAY_SIZE(ad_discoverable), sd, ARRAY_SIZE(sd));
        if (ret)
        {
            LOG_ERR("Failed to restart discoverable advertising, err %d", ret);
        }
    } else if (current_state == BleServiceState::AdvertisingUndiscoverable) {
        LOG_DBG("Restarting undiscoverable adverising");
        struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
            BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_FILTER_SCAN_REQ,
            BT_GAP_ADV_FAST_INT_MIN_1,
            BT_GAP_ADV_FAST_INT_MAX_1,
            NULL
        );
        adv_param.id = current_bt_id;
        ret = bt_le_adv_start(&adv_param, ad_undiscoverable, ARRAY_SIZE(ad_undiscoverable), nullptr, 0);
        if (ret)
        {
            LOG_ERR("Failed to restart undiscoverable advertising, err %d", ret);
        }
    }
}

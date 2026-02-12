#include "BleService.hpp"
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_service, LOG_LEVEL_DBG);

static const uint8_t hid_info[4] = { 0x01, 0x01, 0x00, 0x01 };
static BleService* instance = nullptr;

ssize_t BleService::read_battery_level_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len, uint16_t offset) {
    uint8_t lvl = instance->last_battery_level;
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

ssize_t BleService::read_info_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, hid_info, sizeof(hid_info));
}

ssize_t BleService::write_ctrl_point_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                        const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    LOG_HEXDUMP_DBG(buf, len, "Control point write:");
    return len;
}

ssize_t BleService::read_report_map_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, instance->current_report_map, instance->current_report_map_size);
}



ssize_t BleService::read_report_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len, uint16_t offset) {

    ReportTableEntry* entry = static_cast<ReportTableEntry*>(attr->user_data);
    if (!entry) {
        LOG_ERR("Filed to read report %d", entry->ref.id);
        return -EINVAL;
    }

    ssize_t read_len;
    k_mutex_lock(&instance->report_mutex, K_FOREVER);

    read_len = bt_gatt_attr_read(conn, attr, buf, len, offset,
                                 entry->report_cache, entry->report_cache_len);

    k_mutex_unlock(&instance->report_mutex);
    return read_len;
}

ssize_t BleService::write_report_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
   ReportTableEntry* entry = static_cast<ReportTableEntry*>(attr->user_data);

    if (!entry) return -EINVAL;
    if (offset + len > MAX_REPORT_LEN) return -BT_ATT_ERR_INVALID_OFFSET;

    k_mutex_lock(&instance->report_mutex, K_FOREVER);

    memcpy(entry->report_cache + offset, buf, len);
    entry->report_cache_len = offset + len;

    k_mutex_unlock(&instance->report_mutex);

    instance->hid_callbacks->on_incoming_report(entry->ref.id, (const uint8_t*)buf, len);

    return len;
}

ssize_t BleService::read_report_ref_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, 2);
}


BT_GATT_SERVICE_DEFINE(bas_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
    BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           BleService::read_battery_level_cb, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);


static const struct bt_data ad_undiscoverable[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, 0),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL)),
};


int BleService::init(const DeviceConfig* device_config) {
    if (instance != nullptr) {
        LOG_ERR("BleService already initialized!");
        return -EALREADY;
    }
    instance = this;
    int err = 0;
    err = init_dis(&device_config->dis_config);
    if (err) {
        LOG_ERR("Init DIS failed, err: %d", err);
        return err;
    }
    err = init_hid(&device_config->hid_config);
    if (err) {
        LOG_ERR("Init HID failed, err: %d", err);
        return err;
    }

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("BLE enable failed, err: %d", err);
        return err;
    }

    err = bt_set_appearance(device_config->appearance);
    if (err) {
        LOG_ERR("Set appearance failed, err: %d", err);
        return err;
    }

    if (device_config->name)
    {
        err = bt_set_name(device_config->name);
        if (err)
        {
            LOG_ERR("Set name failed, err: %d", err);
            return err;
        }
    }

    k_mutex_init(&report_mutex);

    static struct bt_conn_cb conn_callbacks = {
        .connected = [](struct bt_conn *conn, uint8_t err) {
            if (err) {
                LOG_ERR("Connection failed (err %u)", err);
                return;
            }
            bt_conn_ref(conn);
            //LOG_DBG("Connected (ref captured). Waiting for security...");
        },

        .disconnected = [](struct bt_conn *conn, uint8_t reason) {
            LOG_DBG("Disconnected (reason %u)", reason);

            if (instance->current_conn == conn) {
                bt_conn_unref(instance->current_conn);
                instance->current_conn = nullptr;
                instance->current_state = BleServiceState::Idle;

                if (reason == BT_HCI_ERR_REMOTE_USER_TERM_CONN) { //graceful disconnect
                    instance->hid_callbacks->on_disconnected(true);
                } else if (reason == BT_HCI_ERR_LOCALHOST_TERM_CONN) { //disonnect by device, usually before poweroff
                    //thus do nothing
                } else { //any other reason
                    instance->hid_callbacks->on_disconnected(false);
                }
            } else {
                bt_conn_unref(conn);
                instance->restart_advertising();
            }
        },

        .security_changed = [](struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
            if (err) {
                LOG_ERR("Security failed (err %d). Disconnecting...", err);
                bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
                return;
            }
            instance->exchange_params = {
                [](struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)  {
                    //LOG_DBG("MTU exchange %s ", err == 0 ? "successful" : "failed");
                    //LOG_DBG("MTU size is: %d", bt_gatt_get_mtu(conn));
                },
            };
            //LOG_DBG("MTU size is: %d", bt_gatt_get_mtu(conn));
            int mtu_err = bt_gatt_exchange_mtu(conn, &instance->exchange_params);
            if (mtu_err) {
                LOG_ERR("MTU exchange failed (err %d)", mtu_err);
            } else {
                k_sleep(K_MSEC(50)); //otherwise <wrn> bt_att: No ATT channel for MTU 36
                //LOG_DBG("MTU exchange pending");
            }

            if (level >= BT_SECURITY_L2) {
                if (instance->current_conn != nullptr && instance->current_conn != conn) {
                    LOG_DBG("Replacing old connection with new bonded peer");
                    bt_conn_disconnect(instance->current_conn, BT_HCI_ERR_AUTH_FAIL);
                    bt_conn_unref(instance->current_conn);
                }

                instance->current_conn = conn;
                instance->current_state = BleServiceState::Connected;

                k_work_cancel_delayable(&instance->adv_timeout_work);
                instance->hid_callbacks->on_connected();
            }


            struct bt_conn_info info;
            if (bt_conn_get_info(conn, &info) == 0) {
                char addr_str[BT_ADDR_LE_STR_LEN];
                bt_addr_le_to_str(info.le.dst, addr_str, sizeof(addr_str));
                LOG_DBG("Connected and authentificated: %s", addr_str);
            } else {
                LOG_ERR("Failed to get connection info");
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
            bt_foreach_bond(instance->current_bt_id, [](const struct bt_bond_info *info, void *user_data) {
                const bt_addr_le_t *current = static_cast<const bt_addr_le_t *>(user_data);
                if (bt_addr_le_cmp(&info->addr, current) != 0) {
                    bt_unpair(instance->current_bt_id, &info->addr);
                }
            }, const_cast<bt_addr_le_t *>(curr_addr));
        },
    };
    bt_conn_auth_info_cb_register(&auth_info_callbacks);
    k_work_init_delayable(&adv_timeout_work, on_adv_timeout);
    init_advertising();
    //LOG_DBG("BLE init finished");
    return 0;
}

int BleService::set_identity(uint8_t bt_id) {
    current_bt_id = bt_id;
    if (current_bt_id > 0) {
        bt_addr_le_t base_addr;
        size_t count = 1;

        bt_id_get(&base_addr, &count);

        //LOG_HEXDUMP_DBG(base_addr.a.val, 6, "Current mac: ");

        uint32_t carry = current_bt_id;
        for (int i = 0; i < 6; i++) {
            uint32_t val = base_addr.a.val[i] + carry;
            base_addr.a.val[i] = (uint8_t)val;
            carry = val >> 8;
            if (carry == 0) break;
        }

        base_addr.type = BT_ADDR_LE_RANDOM;
        base_addr.a.val[5] |= 0xC0;

        //LOG_HEXDUMP_DBG(base_addr.a.val, 6, "New mac: ");

        int id_ret = bt_id_create(&base_addr, NULL);
        if (id_ret < 0) {
            if (id_ret == -EALREADY) {
                return 0;
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

void BleService::add_characteristic(struct bt_gatt_attr *attr_array, uint8_t &a_idx,
                        struct bt_gatt_chrc *chrc_pool, uint8_t &c_idx,
                        const struct bt_uuid *uuid, uint8_t props, uint16_t perm,
                        bt_gatt_attr_read_func_t read, bt_gatt_attr_write_func_t write, void *user_data) {
    chrc_pool[c_idx].uuid = uuid;
    chrc_pool[c_idx].value_handle = 0U;
    chrc_pool[c_idx].properties = props;

    attr_array[a_idx++] = BT_GATT_ATTRIBUTE(BT_UUID_GATT_CHRC,
                            BT_GATT_PERM_READ, bt_gatt_attr_read_chrc, NULL,
                            &chrc_pool[c_idx]);

    attr_array[a_idx++] = BT_GATT_ATTRIBUTE(uuid, perm, read, write, user_data);

    c_idx++;
}

int BleService::init_dis(const DisConfig* dis) {
    if (!dis) {
        LOG_ERR("dis is NULL!");
        return -EINVAL;
    }
    dis_ptr = dis;
    memset(dis_attrs, 0, sizeof(dis_attrs));
    memset(dis_chrcs, 0, sizeof(dis_chrcs));
    uint8_t d_idx = 0;
    uint8_t d_c_idx = 0;
    dis_attrs[d_idx++] = BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS);
    add_characteristic(dis_attrs, d_idx, dis_chrcs, d_c_idx,
                       BT_UUID_DIS_PNP_ID, BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
                       read_dis_pnp_id, NULL, (void*)(&dis_ptr->pnp_id));
    if (dis_ptr->manufacturer) {
        add_characteristic(dis_attrs, d_idx, dis_chrcs, d_c_idx,
                           BT_UUID_DIS_MANUFACTURER_NAME, BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
                           read_dis_string, NULL, (void*)dis_ptr->manufacturer);
    }
    if (dis_ptr->model) {
        add_characteristic(dis_attrs, d_idx, dis_chrcs, d_c_idx,
                           BT_UUID_DIS_MODEL_NUMBER, BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
                           read_dis_string, NULL, (void*)dis_ptr->model);
    }
    if (dis_ptr->serial) {
        add_characteristic(dis_attrs, d_idx, dis_chrcs, d_c_idx,
                           BT_UUID_DIS_SERIAL_NUMBER, BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
                           read_dis_string, NULL, (void*)dis_ptr->serial);
    }
    if (dis_ptr->fw_rev) {
        add_characteristic(dis_attrs, d_idx, dis_chrcs, d_c_idx,
                           BT_UUID_DIS_FIRMWARE_REVISION, BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
                           read_dis_string, NULL, (void*)dis_ptr->fw_rev);
    }
    dis_svc.attrs = dis_attrs;
    dis_svc.attr_count = d_idx;
    return bt_gatt_service_register(&dis_svc);
}

int BleService::init_hid(const HidConfig* hid) {
    if (!hid) {
        LOG_ERR("hid is NULL!");
        return -EINVAL;
    }
    uint8_t h_idx = 0;
    uint8_t h_c_idx = 0;
    hids_attrs[h_idx++] = BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS);
    add_characteristic(hids_attrs, h_idx, hids_chrcs, h_c_idx,
                       BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ, BT_GATT_PERM_READ,
                       read_info_cb, NULL, NULL);
    add_characteristic(hids_attrs, h_idx, hids_chrcs, h_c_idx,
                       BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE_LESC,
                       NULL, write_ctrl_point_cb, NULL);
    add_characteristic(hids_attrs, h_idx, hids_chrcs, h_c_idx,
                       BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ, BT_GATT_PERM_READ_LESC,
                       read_report_map_cb, NULL, this);
    report_count = hid->report_count;
    if (report_count > MAX_REPORTS) {
        LOG_WRN("Too many reports, truncating to %d", MAX_REPORTS);
        report_count = MAX_REPORTS;
    }

    uint8_t ccc_idx = 0;
    for (uint8_t i = 0; i < report_count; i++) {
        report_storage[i].ref = hid->reports[i];
        report_storage[i].report_cache_len = 0;

        if (report_storage[i].ref.type == HidReportType::Input) {
            add_characteristic(hids_attrs, h_idx, hids_chrcs, h_c_idx,
                               BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ_LESC,
                               read_report_cb, NULL, &report_storage[i]);

            report_storage[i].attr_idx = h_idx - 1;
            hids_ccc_pool[ccc_idx] = BT_GATT_CCC_MANAGED_USER_DATA_INIT(NULL, NULL, NULL);
            hids_attrs[h_idx++] = BT_GATT_CCC_MANAGED(&hids_ccc_pool[ccc_idx++], BT_GATT_PERM_READ_LESC | BT_GATT_PERM_WRITE_LESC);
        }
        else {
            add_characteristic(hids_attrs, h_idx, hids_chrcs, h_c_idx,
                               BT_UUID_HIDS_REPORT,
                               BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                               BT_GATT_PERM_READ_LESC | BT_GATT_PERM_WRITE_LESC,
                               read_report_cb, write_report_cb, &report_storage[i]);

            report_storage[i].attr_idx = h_idx - 1;
        }

        hids_attrs[h_idx++] = BT_GATT_ATTRIBUTE(BT_UUID_HIDS_REPORT_REF,
                                BT_GATT_PERM_READ, read_report_ref_cb, NULL,
                                &report_storage[i].ref);
    }
    hids_svc.attrs = hids_attrs;
    hids_svc.attr_count = h_idx;
    current_report_map = hid->report_map;
    current_report_map_size = hid->report_map_size;
    return bt_gatt_service_register(&hids_svc);
}

void BleService::init_advertising() {
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

    const char* name = bt_get_name();
    sd[0].type = BT_DATA_NAME_COMPLETE;
    sd[0].data_len = (uint8_t)strlen(name);
    sd[0].data = (const uint8_t *)name;
}

int BleService::start_advertising(bool discoverable) {
    bt_le_adv_stop();
    int err = 0;
    if (discoverable) {
        struct bt_le_adv_param adv_param = *BT_LE_ADV_CONN_FAST_1;
        adv_param.id = current_bt_id;

        LOG_DBG("Advertising discoverable");
        current_state = current_conn ? BleServiceState::ConnectedAdvertising : BleServiceState::AdvertisingDiscoverable;
        err = bt_le_adv_start(&adv_param, ad_discoverable, ARRAY_SIZE(ad_discoverable), sd, ARRAY_SIZE(sd));
    } else {
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
        current_state = BleServiceState::AdvertisingUndiscoverable;
        err = bt_le_adv_start(&adv_param, ad_undiscoverable, ARRAY_SIZE(ad_undiscoverable), nullptr, 0);
    }
    if (err) {
        LOG_ERR("Failed to begin adverising, err %d", err);
    } else {

        k_work_reschedule(&adv_timeout_work, K_SECONDS(ADV_TIMEOUT_S));
    }
    return err;
}

int BleService::stop_advertising() {
    int err = 0;
    if (current_state == BleServiceState::AdvertisingDiscoverable) {
        LOG_DBG("Aborting discoverable adverising while disconnected");
        err = start_advertising(false);
    } else if (current_state == BleServiceState::ConnectedAdvertising) {
        LOG_DBG("Aborting discoverable adverising while connected");
        err = bt_le_adv_stop();
        current_state = BleServiceState::Connected;
    } else {
        LOG_WRN("Not adverising discoverable but function called");
        err = -EINVAL;
    }
    return err;
}

int BleService::update_report(uint8_t report_id, const uint8_t* data, uint16_t len) {
    if (current_state < BleServiceState::Connected || !current_conn) return -ENOTCONN;

    if (len > MAX_REPORT_LEN)
    {
        LOG_WRN("Report of len %d provided while maximum %d is supported, ignoring", len, MAX_REPORT_LEN);
        return -EINVAL;
    }

    ReportTableEntry* report_entry = nullptr;

    for (int i = 0; i < report_count; i++) {
        if (report_storage[i].ref.id == report_id) {
            if (report_storage[i].ref.type == HidReportType::Output) {
                LOG_WRN("Report id %d is output, ignoring", report_id);
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

    k_mutex_lock(&report_mutex, K_FOREVER);
    if (report_entry->report_cache_len == len && memcmp(data, report_entry->report_cache, len) == 0)
    {
        k_mutex_unlock(&report_mutex);
        return 0;
    }

    report_entry->report_cache_len = len;

    memcpy(report_entry->report_cache, data, report_entry->report_cache_len);
    k_mutex_unlock(&report_mutex);
    if (report_entry->ref.type == HidReportType::Input) {
        return bt_gatt_notify(current_conn, &hids_svc.attrs[report_entry->attr_idx], data, len);
    }
    return 0;
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

    if (!err) {
        LOG_DBG("Battery level updated: %u%%", level);
    }
    else {
        LOG_ERR("Battery level update err: %d", err);
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
    LOG_DBG("Advertising timeout reached, current_state: %d", static_cast<uint8_t>(instance->current_state));
    bt_le_adv_stop();
    BleServiceState old_state = instance->current_state;
    instance->current_state = instance->current_conn ? BleServiceState::Connected : BleServiceState::Idle;
    if ((old_state == BleServiceState::AdvertisingDiscoverable || old_state == BleServiceState::ConnectedAdvertising)) {
        instance->hid_callbacks->on_adv_timeout(true);
    }
    else if (old_state == BleServiceState::AdvertisingUndiscoverable) {
        instance->hid_callbacks->on_adv_timeout(false);
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

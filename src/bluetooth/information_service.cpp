/**
 * @file information_service.cpp
 * @author
 * @brief
 * @version 2.4.1
 * @date 2025-05-12
 *
 * @copyright Botz Innovation 2025
 *
 */

#define MODULE bluetooth

#include <errors.hpp>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>

#include <cstring>

#include "hardware_data.pb.h"
#include <ble_services.hpp>
#include <util.hpp>

LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

static uint32_t error_bitfield_data = 0;
static uint32_t previous_error_bitfield_data = 0; // Store the previous value of error_bitfield_data
static bool init_status_bt_update = false;        // A flag to force the first update

static uint8_t charge_status = 0;

static bool status_subscribed = true;
static bool meta_status_subscribed = true;
static uint32_t log_available_meta = 0;

static char meta_req_id[util::max_path_length] = {0};

static uint8_t ct[10];
static uint8_t ct_update = 0;

// FWD Declarations
// clang-format off

//Current Time Characteristic
static ssize_t jis_read_current_time(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static ssize_t jis_write_current_time(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buf, uint16_t len, uint16_t offset, uint8_t flags);
static void jis_current_time_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
// Status Characteristic
static ssize_t jis_status_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_status_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);

// Meta Log available Characteristic
static ssize_t jis_log_available_meta_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_log_available_meta_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
// Battery Status Characteristic
static ssize_t jis_charge_status_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_charge_status_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
//Req ID
static ssize_t jis_req_id_hw_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len,
    uint16_t offset);
static void jis_req_id_hw_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
static ssize_t jis_req_id_meta_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len,
    uint16_t offset);
static void jis_req_id_meta_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);

// UUID Defines
static struct bt_uuid_128 SENSING_INFO_SERVICE_UUID =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eaa, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

static struct bt_uuid_128 STATUS_UUID =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eab, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));


static struct bt_uuid_128 log_available_meta_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eac, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

static struct bt_uuid_128 CHARGE_STATUS_UUID =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ead, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));


static struct bt_uuid_128 req_id_meta_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eae, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));


BT_GATT_SERVICE_DEFINE(
    info_service, BT_GATT_PRIMARY_SERVICE(&SENSING_INFO_SERVICE_UUID),
    // Current Ttime Characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_read_current_time, jis_write_current_time,
                            ct),
    BT_GATT_CCC(jis_current_time_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Status Characteristic
    BT_GATT_CHARACTERISTIC(&STATUS_UUID.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_status_read, nullptr,
                            static_cast<void *>(&error_bitfield_data)),
    BT_GATT_CCC(jis_status_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),


 // Meta Logs available Characteristic
    BT_GATT_CHARACTERISTIC(&log_available_meta_uuid.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_log_available_meta_read, nullptr,
                             static_cast<void *>(&log_available_meta)),
    BT_GATT_CCC(jis_log_available_meta_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Battery Status Characteristic
    BT_GATT_CHARACTERISTIC(&CHARGE_STATUS_UUID.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_charge_status_read, nullptr,
                            static_cast<void *>(&charge_status)),
    BT_GATT_CCC(jis_charge_status_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),


// Meta Req Id Characteristic
    BT_GATT_CHARACTERISTIC(&req_id_meta_uuid.uuid,
                            BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_req_id_meta_read, nullptr,
                            meta_req_id),
    BT_GATT_CCC(jis_req_id_meta_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT));

// clang-format on
/**
 * @brief
 *
 * @param status
 *
 */
void jis_set_err_status_notify(err_t new_device_status)
{
    switch (new_device_status)
    {

        case err_t::FILE_SYSTEM_ERROR:
            //   set Fault Storage
            error_bitfield_data |= ErrorStatus_storage_persistent_fault_bit;
            LOG_INF("set storage fault");
            break;

        case err_t::BATTERY_FAULT:
            //   set Fault Battery
            error_bitfield_data |= ErrorStatus_battery_adc_persistent_fault_bit;
            LOG_INF("set battery/charger fault");
            break;

        case err_t::BATTERY_DISCONNECTION_ERROR:
            // set Battery disconnection fault
            error_bitfield_data |= ErrorStatus_battery_disconnection_persistent_fault_bit;
            LOG_INF("set battery disconnection fault");
            break;

        case err_t::BLUETOOTH_ERROR:
            //  set bluetooth error
            error_bitfield_data |= ErrorStatus_bt_persistent_fault_bit;
            LOG_INF("set BT fault");
            break;

        case err_t::DFU_ERROR:
            //  set DFU error
            error_bitfield_data |= ErrorStatus_dfu_persistent_fault_bit;
            LOG_INF("set dfu fault");
            break;

        case err_t::FLASH_FAILURE:
            //  set flash error
            error_bitfield_data |= ErrorStatus_flash_persistent_fault_bit;
            LOG_INF("Set Flash read/write failure");
            break;

        default:
            LOG_WRN("UNKNOWN device fault status reported: %d", (int)new_device_status);
            break;
    }

    // Check if error_bitfield_data has changed, update BT, only if there is difference in value
    if (((!init_status_bt_update) && (error_bitfield_data != 0)) ||
        ((error_bitfield_data != previous_error_bitfield_data) && (error_bitfield_data != 0)))
    {
        if (status_subscribed)
        {
            auto *status_gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &STATUS_UUID.uuid);
            bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&error_bitfield_data),
                           sizeof(error_bitfield_data));
            LOG_DBG(" Set Fault status: status user data: 0x%X\n", error_bitfield_data);
        }

        // Update the previous value, after reporting the error
        previous_error_bitfield_data = error_bitfield_data;
        init_status_bt_update = true;
    }

    else
    {
        LOG_WRN("Errors reported but are disabled due to FCT Login");
    }
}

/**
 * @brief clears the error status
 *
 * @param new_device_status
 *
 */
void jis_clear_err_status_notify(err_t new_device_status)
{

    switch (new_device_status)
    {

        case err_t::FILE_SYSTEM_ERROR:
            //   Clear Fault Storage
            error_bitfield_data &= ~ErrorStatus_storage_persistent_fault_bit;
            LOG_DBG("Clear storage fault");
            break;

        case err_t::BATTERY_FAULT:
            // Clear Fault Battery
            error_bitfield_data &= ~ErrorStatus_battery_adc_persistent_fault_bit;
            LOG_DBG("Clear battery/charger fault");
            break;

        case err_t::BATTERY_DISCONNECTION_ERROR:
            // Clear Battery disconnection
            error_bitfield_data &= ~ErrorStatus_battery_disconnection_persistent_fault_bit;
            LOG_DBG("Clear battery disconnection fault");
            break;

        case err_t::BLUETOOTH_ERROR:
            //  Clear Fault Bluetooth
            error_bitfield_data &= ~ErrorStatus_bt_persistent_fault_bit;
            LOG_DBG("clear BT error");
            break;

        case err_t::DFU_ERROR:
            //  set load cell error
            error_bitfield_data &= ~ErrorStatus_dfu_persistent_fault_bit;
            LOG_DBG("clear dfu fault");
            break;

        case err_t::FLASH_FAILURE:
            //  set flash error
            error_bitfield_data &= ~ErrorStatus_flash_persistent_fault_bit;
            LOG_INF("Clear Flash read/write failure");
            break;

        default:
            LOG_WRN("UNKNOWN device clear status reported: %d", (int)new_device_status);
            break;
    }
    // Check if error_bitfield_data has changed or update it if its the first time
    if (!init_status_bt_update || error_bitfield_data != previous_error_bitfield_data)
    {
        if (status_subscribed)
        {
            auto *status_gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &STATUS_UUID.uuid);
            bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&error_bitfield_data),
                           sizeof(error_bitfield_data));
            LOG_INF("Clear Bit field Data: 0x%X\n", error_bitfield_data);
            // create the critical error event and fill in the status
        }

        previous_error_bitfield_data = error_bitfield_data; // Update the previous value

        // All monitored bits are cleared, allow system reset to be cleared
        // Don't clear it at the first time, the static variable is cleared in the warm reboot.
        if ((error_bitfield_data == 0) && (init_status_bt_update))
        {
            LOG_WRN("error_bitfield_data %d, previous_error_bitfield_data: %d init_status_bt_update: %d",
                    error_bitfield_data, previous_error_bitfield_data, init_status_bt_update);
        }
        init_status_bt_update = true;
    }
}

/**
 * @brief
 *
 * @param attr
 * @param value
 *
 */
static void jis_status_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    status_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Status CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

/**
 * @brief
 *
 * @param attr
 * @param value
 *
 */
static void jis_current_time_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    /* TODO: Handle value */
    LOG_DBG("Current TIme CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_read_current_time(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                     uint16_t offset)
{
    // To develop

    const char *value = static_cast<const char *>(attr->user_data);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(ct));
}

static ssize_t jis_write_current_time(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                      uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(flags);
    uint8_t *value = static_cast<uint8_t *>(attr->user_data);

    if (offset + len > sizeof(ct))
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    std::memcpy(value + offset, buf, len);
    ct_update = 1U;

    return len;
}
static ssize_t jis_status_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                               uint16_t offset)
{
    auto *value = static_cast<uint16_t *>(attr->user_data);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(error_bitfield_data));
}

/**
 * @brief
 *
 *
 */
void cts_notify(void)
{ /* Current Time Service updates only when time is changed */
    if (!ct_update)
    {
        return;
    }

    ct_update = 0U;
    bt_gatt_notify(NULL, &info_service.attrs[1], &ct, sizeof(ct));
}

/**
 * @brief
 *
 * @param new_charge_status
 *
 */
void jis_charge_status_notify(uint8_t new_charge_status)
{
    LOG_DBG("charge status: %d", new_charge_status);

    charge_status = new_charge_status;

    bt_gatt_notify_uuid(nullptr, &CHARGE_STATUS_UUID.uuid, info_service.attrs, static_cast<void *>(&charge_status),
                        sizeof(charge_status));
}

/**
 * @brief
 *
 * @param attr
 * @param value
 *
 */
static void jis_charge_status_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    ARG_UNUSED(value);
    // status_subscribed = value == BT_GATT_CCC_NOTIFY;
}

/**
 * @brief
 *
 * @param conn
 * @param attr
 * @param buf
 * @param len
 * @param offset
 * @return ssize_t
 *
 */
static ssize_t jis_charge_status_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                      uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, static_cast<void *>(&charge_status), sizeof(charge_status));
}

void cs_log_available_meta_notify(uint32_t stu)
{
    log_available_meta = stu;
    LOG_DBG("Meta_ID: %u, status Notified: %s", log_available_meta, meta_status_subscribed ? "true" : "false");
    if (meta_status_subscribed)
    {
        auto *status_gatt =
            bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &log_available_meta_uuid.uuid);
        bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&log_available_meta), sizeof(log_available_meta));
    }
}

static void jis_log_available_meta_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    meta_status_subscribed = value == BT_GATT_CCC_NOTIFY;
}

static ssize_t jis_log_available_meta_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                           uint16_t len, uint16_t offset)
{
    auto *value = static_cast<uint8_t *>(attr->user_data);
    LOG_DBG("log_available_meta_read : %d", log_available_meta);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(log_available_meta));
}

/**
 * @brief Notify the subscribed clients of the change in the request ID metadata.
 *
 * This function copies the provided `stu` string into the `meta_req_id` buffer. If there is a
 * subscription to the request ID metadata notification (i.e., `meta_status_subscribed` is true),
 * it looks up the GATT attribute by UUID and sends a notification to subscribed clients with the
 * updated `meta_req_id`.
 *
 * @param stu The string containing the new request ID metadata. It is copied into the `meta_req_id` buffer.
 *
 */
void cs_req_id_meta_notify(const char *stu)
{
    // clear buffer before writing the new value
    std::memset(meta_req_id, 0, sizeof(meta_req_id));
    std::strcpy(meta_req_id, stu);
    if (meta_status_subscribed)
    {
        auto *status_gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &req_id_meta_uuid.uuid);
        bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&meta_req_id), sizeof(meta_req_id));
    }
}

static void jis_req_id_meta_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    meta_status_subscribed = value == BT_GATT_CCC_NOTIFY;
}

static ssize_t jis_req_id_meta_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                    uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(meta_req_id));
}

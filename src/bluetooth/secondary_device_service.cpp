/**
 * @file secondary_device_service.cpp
 * @author Botz Innovation
 * @brief Secondary Device Service - Information about connected secondary device
 * @version 1.0
 * @date 2025-01
 *
 * @copyright Copyright (c) 2025 Botz Innovation
 *
 * This service provides information about the connected secondary device
 * and relays its data. Only available on primary device.
 */

#define MODULE secondary_device_service

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <app.hpp>

LOG_MODULE_REGISTER(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)

// Service UUID: 4fd5b6a0-9d89-4061-92aa-319ca786baae (increment pattern)
static struct bt_uuid_128 SECONDARY_DEVICE_SERVICE_UUID = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a0, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Characteristic UUIDs (using same pattern as information service)
// Device Info: ...a1 to ...a5
static struct bt_uuid_128 secondary_manufacturer_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a1, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

static struct bt_uuid_128 secondary_model_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a2, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

static struct bt_uuid_128 secondary_serial_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a3, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

static struct bt_uuid_128 secondary_hw_rev_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a4, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

static struct bt_uuid_128 secondary_fw_rev_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a5, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// FOTA Progress: ...a6
static struct bt_uuid_128 secondary_fota_progress_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a6, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// File Management: ...a7 to ...ac
static struct bt_uuid_128 secondary_foot_log_available_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a7, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

static struct bt_uuid_128 secondary_foot_log_path_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a8, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

static struct bt_uuid_128 secondary_bhi360_log_available_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a9, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

static struct bt_uuid_128 secondary_bhi360_log_path_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6aa, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

static struct bt_uuid_128 secondary_activity_log_available_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6ab, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

static struct bt_uuid_128 secondary_activity_log_path_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6ac, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Weight Measurement: ...ad
static struct bt_uuid_128 secondary_weight_measurement_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6ad, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Static data storage
static char secondary_manufacturer[32] = "Not Connected";
static char secondary_model[32] = "Not Connected";
static char secondary_serial[32] = "Not Connected";
static char secondary_hw_rev[16] = "Not Connected";
static char secondary_fw_rev[16] = "Not Connected";

static fota_progress_msg_t secondary_fota_progress_value = {0};
static uint8_t secondary_foot_log_available = 0;
static char secondary_foot_log_path[64] = "";
static uint8_t secondary_bhi360_log_available = 0;
static char secondary_bhi360_log_path[64] = "";
static uint8_t secondary_activity_log_available = 0;
static char secondary_activity_log_path[64] = "";
static uint16_t secondary_weight_kg_x10 = 0;

// CCC storage
static bool secondary_fota_progress_notify_enabled = false;
static bool secondary_foot_log_available_notify_enabled = false;
static bool secondary_foot_log_path_notify_enabled = false;
static bool secondary_bhi360_log_available_notify_enabled = false;
static bool secondary_bhi360_log_path_notify_enabled = false;
static bool secondary_activity_log_available_notify_enabled = false;
static bool secondary_activity_log_path_notify_enabled = false;
static bool secondary_weight_notify_enabled = false;

// Connection tracking
static struct bt_conn *current_conn = NULL;

// Forward declarations
static ssize_t sds_secondary_info_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len, uint16_t offset);

static ssize_t sds_secondary_fota_progress_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                void *buf, uint16_t len, uint16_t offset);
static void sds_secondary_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t sds_secondary_foot_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                     void *buf, uint16_t len, uint16_t offset);
static void sds_secondary_foot_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t sds_secondary_foot_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                void *buf, uint16_t len, uint16_t offset);
static void sds_secondary_foot_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t sds_secondary_bhi360_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                       void *buf, uint16_t len, uint16_t offset);
static void sds_secondary_bhi360_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t sds_secondary_bhi360_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                  void *buf, uint16_t len, uint16_t offset);
static void sds_secondary_bhi360_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t sds_secondary_activity_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                         void *buf, uint16_t len, uint16_t offset);
static void sds_secondary_activity_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t sds_secondary_activity_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                    void *buf, uint16_t len, uint16_t offset);
static void sds_secondary_activity_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t sds_secondary_weight_measurement_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                    void *buf, uint16_t len, uint16_t offset);
static void sds_secondary_weight_measurement_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

// Service definition
BT_GATT_SERVICE_DEFINE(
    secondary_device_service,
    BT_GATT_PRIMARY_SERVICE(&SECONDARY_DEVICE_SERVICE_UUID),
    
    // Secondary Device Information
    BT_GATT_CHARACTERISTIC(&secondary_manufacturer_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_info_read, nullptr,
        secondary_manufacturer),
    
    BT_GATT_CHARACTERISTIC(&secondary_model_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_info_read, nullptr,
        secondary_model),
    
    BT_GATT_CHARACTERISTIC(&secondary_serial_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_info_read, nullptr,
        secondary_serial),
    
    BT_GATT_CHARACTERISTIC(&secondary_hw_rev_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_info_read, nullptr,
        secondary_hw_rev),
    
    BT_GATT_CHARACTERISTIC(&secondary_fw_rev_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_info_read, nullptr,
        secondary_fw_rev),
    
    // Secondary FOTA Progress
    BT_GATT_CHARACTERISTIC(&secondary_fota_progress_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_fota_progress_read, nullptr,
        static_cast<void *>(&secondary_fota_progress_value)),
    BT_GATT_CCC(sds_secondary_fota_progress_ccc_cfg_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Secondary Foot Log Available
    BT_GATT_CHARACTERISTIC(&secondary_foot_log_available_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_foot_log_available_read, nullptr,
        static_cast<void *>(&secondary_foot_log_available)),
    BT_GATT_CCC(sds_secondary_foot_log_available_ccc_cfg_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Secondary Foot Log Path
    BT_GATT_CHARACTERISTIC(&secondary_foot_log_path_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_foot_log_path_read, nullptr,
        secondary_foot_log_path),
    BT_GATT_CCC(sds_secondary_foot_log_path_ccc_cfg_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Secondary BHI360 Log Available
    BT_GATT_CHARACTERISTIC(&secondary_bhi360_log_available_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_bhi360_log_available_read, nullptr,
        static_cast<void *>(&secondary_bhi360_log_available)),
    BT_GATT_CCC(sds_secondary_bhi360_log_available_ccc_cfg_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Secondary BHI360 Log Path
    BT_GATT_CHARACTERISTIC(&secondary_bhi360_log_path_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_bhi360_log_path_read, nullptr,
        secondary_bhi360_log_path),
    BT_GATT_CCC(sds_secondary_bhi360_log_path_ccc_cfg_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Secondary Activity Log Available
    BT_GATT_CHARACTERISTIC(&secondary_activity_log_available_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_activity_log_available_read, nullptr,
        static_cast<void *>(&secondary_activity_log_available)),
    BT_GATT_CCC(sds_secondary_activity_log_available_ccc_cfg_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Secondary Activity Log Path
    BT_GATT_CHARACTERISTIC(&secondary_activity_log_path_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_activity_log_path_read, nullptr,
        secondary_activity_log_path),
    BT_GATT_CCC(sds_secondary_activity_log_path_ccc_cfg_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Secondary Weight Measurement
    BT_GATT_CHARACTERISTIC(&secondary_weight_measurement_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        sds_secondary_weight_measurement_read, nullptr,
        static_cast<void *>(&secondary_weight_kg_x10)),
    BT_GATT_CCC(sds_secondary_weight_measurement_ccc_cfg_changed,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT)
);

// --- Handler Implementations ---

// Secondary device info read handler
static ssize_t sds_secondary_info_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset)
{
    const char *value = (const char *)attr->user_data;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

// Secondary FOTA Progress handlers
static void sds_secondary_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("sds_secondary_fota_progress_ccc_cfg_changed: attr is NULL");
        return;
    }
    
    secondary_fota_progress_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Secondary FOTA progress notifications %s",
            secondary_fota_progress_notify_enabled ? "enabled" : "disabled");
}

static ssize_t sds_secondary_fota_progress_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            &secondary_fota_progress_value, sizeof(secondary_fota_progress_value));
}

// Secondary Foot Log Available handlers
static void sds_secondary_foot_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("sds_secondary_foot_log_available_ccc_cfg_changed: attr is NULL");
        return;
    }
    
    secondary_foot_log_available_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Secondary foot log available notifications %s",
            secondary_foot_log_available_notify_enabled ? "enabled" : "disabled");
}

static ssize_t sds_secondary_foot_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                    void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            &secondary_foot_log_available, sizeof(secondary_foot_log_available));
}

// Secondary Foot Log Path handlers
static void sds_secondary_foot_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("sds_secondary_foot_log_path_ccc_cfg_changed: attr is NULL");
        return;
    }
    
    secondary_foot_log_path_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Secondary foot log path notifications %s",
            secondary_foot_log_path_notify_enabled ? "enabled" : "disabled");
}

static ssize_t sds_secondary_foot_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            secondary_foot_log_path, strlen(secondary_foot_log_path));
}

// Secondary BHI360 Log Available handlers
static void sds_secondary_bhi360_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("sds_secondary_bhi360_log_available_ccc_cfg_changed: attr is NULL");
        return;
    }
    
    secondary_bhi360_log_available_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Secondary BHI360 log available notifications %s",
            secondary_bhi360_log_available_notify_enabled ? "enabled" : "disabled");
}

static ssize_t sds_secondary_bhi360_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                      void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            &secondary_bhi360_log_available, sizeof(secondary_bhi360_log_available));
}

// Secondary BHI360 Log Path handlers
static void sds_secondary_bhi360_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("sds_secondary_bhi360_log_path_ccc_cfg_changed: attr is NULL");
        return;
    }
    
    secondary_bhi360_log_path_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Secondary BHI360 log path notifications %s",
            secondary_bhi360_log_path_notify_enabled ? "enabled" : "disabled");
}

static ssize_t sds_secondary_bhi360_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                 void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            secondary_bhi360_log_path, strlen(secondary_bhi360_log_path));
}

// Secondary Activity Log Available handlers
static void sds_secondary_activity_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("sds_secondary_activity_log_available_ccc_cfg_changed: attr is NULL");
        return;
    }
    
    secondary_activity_log_available_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Secondary activity log available notifications %s",
            secondary_activity_log_available_notify_enabled ? "enabled" : "disabled");
}

static ssize_t sds_secondary_activity_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                        void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            &secondary_activity_log_available, sizeof(secondary_activity_log_available));
}

// Secondary Activity Log Path handlers
static void sds_secondary_activity_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("sds_secondary_activity_log_path_ccc_cfg_changed: attr is NULL");
        return;
    }
    
    secondary_activity_log_path_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Secondary activity log path notifications %s",
            secondary_activity_log_path_notify_enabled ? "enabled" : "disabled");
}

static ssize_t sds_secondary_activity_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                   void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            secondary_activity_log_path, strlen(secondary_activity_log_path));
}

// Secondary Weight Measurement handlers
static void sds_secondary_weight_measurement_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("sds_secondary_weight_measurement_ccc_cfg_changed: attr is NULL");
        return;
    }
    
    secondary_weight_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Secondary weight measurement notifications %s",
            secondary_weight_notify_enabled ? "enabled" : "disabled");
}

static ssize_t sds_secondary_weight_measurement_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                   void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            &secondary_weight_kg_x10, sizeof(secondary_weight_kg_x10));
}

// --- Public API Functions (called from bluetooth thread only) ---

void sds_update_secondary_device_info(const char *manufacturer, const char *model,
                                     const char *serial, const char *hw_rev, const char *fw_rev)
{
    if (manufacturer) {
        strncpy(secondary_manufacturer, manufacturer, sizeof(secondary_manufacturer) - 1);
        secondary_manufacturer[sizeof(secondary_manufacturer) - 1] = '\0';
    }
    if (model) {
        strncpy(secondary_model, model, sizeof(secondary_model) - 1);
        secondary_model[sizeof(secondary_model) - 1] = '\0';
    }
    if (serial) {
        strncpy(secondary_serial, serial, sizeof(secondary_serial) - 1);
        secondary_serial[sizeof(secondary_serial) - 1] = '\0';
    }
    if (hw_rev) {
        strncpy(secondary_hw_rev, hw_rev, sizeof(secondary_hw_rev) - 1);
        secondary_hw_rev[sizeof(secondary_hw_rev) - 1] = '\0';
    }
    if (fw_rev) {
        strncpy(secondary_fw_rev, fw_rev, sizeof(secondary_fw_rev) - 1);
        secondary_fw_rev[sizeof(secondary_fw_rev) - 1] = '\0';
    }
    
    LOG_INF("Secondary device info updated: %s %s", manufacturer, model);
}

void sds_clear_secondary_device_info(void)
{
    strcpy(secondary_manufacturer, "Not Connected");
    strcpy(secondary_model, "Not Connected");
    strcpy(secondary_serial, "Not Connected");
    strcpy(secondary_hw_rev, "Not Connected");
    strcpy(secondary_fw_rev, "Not Connected");
    
    LOG_INF("Secondary device info cleared");
}

void sds_secondary_fota_progress_notify(const fota_progress_msg_t *progress)
{
    if (!progress) {
        return;
    }
    
    memcpy(&secondary_fota_progress_value, progress, sizeof(fota_progress_msg_t));
    
    if (secondary_fota_progress_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &secondary_device_service.attrs[12],
                                &secondary_fota_progress_value,
                                sizeof(secondary_fota_progress_value));
        if (err) {
            LOG_WRN("Failed to send secondary FOTA progress notification: %d", err);
        }
    }
}

void sds_secondary_foot_log_available_notify(uint8_t log_id)
{
    secondary_foot_log_available = log_id;
    
    if (secondary_foot_log_available_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &secondary_device_service.attrs[15],
                                &secondary_foot_log_available,
                                sizeof(secondary_foot_log_available));
        if (err) {
            LOG_WRN("Failed to send secondary foot log available notification: %d", err);
        }
    }
}

void sds_secondary_foot_log_path_notify(const char *path)
{
    if (!path) {
        return;
    }
    
    strncpy(secondary_foot_log_path, path, sizeof(secondary_foot_log_path) - 1);
    secondary_foot_log_path[sizeof(secondary_foot_log_path) - 1] = '\0';
    
    if (secondary_foot_log_path_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &secondary_device_service.attrs[18],
                                secondary_foot_log_path,
                                strlen(secondary_foot_log_path));
        if (err) {
            LOG_WRN("Failed to send secondary foot log path notification: %d", err);
        }
    }
}

void sds_secondary_bhi360_log_available_notify(uint8_t log_id)
{
    secondary_bhi360_log_available = log_id;
    
    if (secondary_bhi360_log_available_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &secondary_device_service.attrs[21],
                                &secondary_bhi360_log_available,
                                sizeof(secondary_bhi360_log_available));
        if (err) {
            LOG_WRN("Failed to send secondary BHI360 log available notification: %d", err);
        }
    }
}

void sds_secondary_bhi360_log_path_notify(const char *path)
{
    if (!path) {
        return;
    }
    
    strncpy(secondary_bhi360_log_path, path, sizeof(secondary_bhi360_log_path) - 1);
    secondary_bhi360_log_path[sizeof(secondary_bhi360_log_path) - 1] = '\0';
    
    if (secondary_bhi360_log_path_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &secondary_device_service.attrs[24],
                                secondary_bhi360_log_path,
                                strlen(secondary_bhi360_log_path));
        if (err) {
            LOG_WRN("Failed to send secondary BHI360 log path notification: %d", err);
        }
    }
}

void sds_secondary_activity_log_available_notify(uint8_t log_id)
{
    secondary_activity_log_available = log_id;
    
    if (secondary_activity_log_available_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &secondary_device_service.attrs[27],
                                &secondary_activity_log_available,
                                sizeof(secondary_activity_log_available));
        if (err) {
            LOG_WRN("Failed to send secondary activity log available notification: %d", err);
        }
    }
}

void sds_secondary_activity_log_path_notify(const char *path)
{
    if (!path) {
        return;
    }
    
    strncpy(secondary_activity_log_path, path, sizeof(secondary_activity_log_path) - 1);
    secondary_activity_log_path[sizeof(secondary_activity_log_path) - 1] = '\0';
    
    if (secondary_activity_log_path_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &secondary_device_service.attrs[30],
                                secondary_activity_log_path,
                                strlen(secondary_activity_log_path));
        if (err) {
            LOG_WRN("Failed to send secondary activity log path notification: %d", err);
        }
    }
}

void sds_secondary_weight_measurement_notify(float weight_kg)
{
    secondary_weight_kg_x10 = (uint16_t)(weight_kg * 10.0f);
    
    if (secondary_weight_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &secondary_device_service.attrs[33],
                                &secondary_weight_kg_x10,
                                sizeof(secondary_weight_kg_x10));
        if (err) {
            LOG_WRN("Failed to send secondary weight measurement notification: %d", err);
        }
    }
}

void sds_set_connection(struct bt_conn *conn)
{
    current_conn = conn;
    
    if (!conn) {
        // Connection lost, reset notification states
        secondary_fota_progress_notify_enabled = false;
        secondary_foot_log_available_notify_enabled = false;
        secondary_foot_log_path_notify_enabled = false;
        secondary_bhi360_log_available_notify_enabled = false;
        secondary_bhi360_log_path_notify_enabled = false;
        secondary_activity_log_available_notify_enabled = false;
        secondary_activity_log_path_notify_enabled = false;
        secondary_weight_notify_enabled = false;
    }
}

#endif // CONFIG_PRIMARY_DEVICE
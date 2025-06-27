/**
 * @file ble_d2d_tx_service.cpp
 * @brief D2D TX GATT Service for Secondary Device
 * 
 * This service allows the secondary device to send sensor data to the primary device
 */

#include <cstddef>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "ble_d2d_tx_service.hpp"
#include <app.hpp>
#include <app_fixed_point.hpp>
#include <ble_services.hpp>
#include "ccc_callback_fix.hpp"

LOG_MODULE_REGISTER(d2d_tx_svc, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Service UUID: 75ad68d6-200c-437d-98b5-061862076c5f
static struct bt_uuid_128 d2d_tx_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x75ad68d6, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Characteristic UUIDs
static struct bt_uuid_128 d2d_foot_sensor_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68dc, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_bhi360_data1_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68dd, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_bhi360_data2_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68de, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_bhi360_data3_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68df, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_status_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d6, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_charge_status_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d8, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_foot_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d7, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_bhi360_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68da, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_device_info_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e1, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// FOTA progress UUID
static struct bt_uuid_128 d2d_fota_progress_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e3, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Path UUIDs
static struct bt_uuid_128 d2d_foot_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d9, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_bhi360_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68db, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_activity_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e4, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_activity_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e5, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Activity step count UUID (new)
static struct bt_uuid_128 d2d_activity_step_count_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e6, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Connection tracking
static struct bt_conn *primary_conn = NULL;

// CCC (Client Characteristic Configuration) states
static bool foot_sensor_notify_enabled = false;
static bool bhi360_data1_notify_enabled = false;
static bool bhi360_data2_notify_enabled = false;
static bool bhi360_data3_notify_enabled = false;
static bool status_notify_enabled = false;
static bool charge_status_notify_enabled = false;
static bool foot_log_notify_enabled = false;
static bool bhi360_log_notify_enabled = false;
static bool device_info_notify_enabled = false;
static bool fota_progress_notify_enabled = false;
static bool foot_log_path_notify_enabled = false;
static bool bhi360_log_path_notify_enabled = false;
static bool activity_log_notify_enabled = false;
static bool activity_log_path_notify_enabled = false;
static bool activity_step_count_notify_enabled = false;
static foot_samples_t foot_sensor_char_value = {0};

// Data buffers - using fixed-point versions for BLE transmission
static foot_samples_t foot_sensor_data;
static bhi360_3d_mapping_fixed_t bhi360_data1_fixed;
static bhi360_step_count_fixed_t bhi360_data2_fixed;
static bhi360_linear_accel_fixed_t bhi360_data3_fixed;
static uint32_t device_status = 0;
static uint8_t charge_status = 0;
static uint8_t foot_log_available = 0;
static uint8_t bhi360_log_available = 0;
static device_info_msg_t device_info_data = {0};
static fota_progress_msg_t fota_progress_data = {0};
static char foot_log_path[256] = {0};
static char bhi360_log_path[256] = {0};
static uint8_t activity_log_available = 0;
static char activity_log_path[256] = {0};
static bhi360_step_count_fixed_t activity_step_count_fixed = {0, 0};

// CCC changed callbacks
static void foot_sensor_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("foot_sensor_ccc_changed: attr is NULL");
        return;
    }
    bool was_enabled = foot_sensor_notify_enabled;
    foot_sensor_notify_enabled = value == BT_GATT_CCC_NOTIFY;
    LOG_INF("Foot sensor CCC changed: value=0x%04x, notifications now %s", 
            value, foot_sensor_notify_enabled ? "ENABLED" : "DISABLED");
    
    // Log state change
    if (!was_enabled && foot_sensor_notify_enabled) {
        LOG_INF("Foot sensor notifications ACTIVATED by primary device");
    } else if (was_enabled && !foot_sensor_notify_enabled) {
        LOG_INF("Foot sensor notifications DEACTIVATED by primary device");
    }
}

static void bhi360_data1_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("bhi360_data1_ccc_changed: attr is NULL");
        return;
    }
    bhi360_data1_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 data1 notifications %s", bhi360_data1_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_data2_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("bhi360_data2_ccc_changed: attr is NULL");
        return;
    }
    bhi360_data2_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 data2 notifications %s", bhi360_data2_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_data3_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("bhi360_data3_ccc_changed: attr is NULL");
        return;
    }
    bhi360_data3_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 data3 notifications %s", bhi360_data3_notify_enabled ? "enabled" : "disabled");
}

static void status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("status_ccc_changed: attr is NULL");
        return;
    }
    status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Status notifications %s", status_notify_enabled ? "enabled" : "disabled");
}

static void charge_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("charge_status_ccc_changed: attr is NULL");
        return;
    }
    charge_status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Charge status notifications %s", charge_status_notify_enabled ? "enabled" : "disabled");
}

static void foot_log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("foot_log_ccc_changed: attr is NULL");
        return;
    }
    foot_log_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Foot log notifications %s", foot_log_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("bhi360_log_ccc_changed: attr is NULL");
        return;
    }
    bhi360_log_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 log notifications %s", bhi360_log_notify_enabled ? "enabled" : "disabled");
}

static void device_info_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("device_info_ccc_changed: attr is NULL");
        return;
    }
    device_info_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Device info notifications %s", device_info_notify_enabled ? "enabled" : "disabled");
}

static void fota_progress_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("fota_progress_ccc_changed: attr is NULL");
        return;
    }
    fota_progress_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("FOTA progress notifications %s", fota_progress_notify_enabled ? "enabled" : "disabled");
}

static void foot_log_path_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("foot_log_path_ccc_changed: attr is NULL");
        return;
    }
    foot_log_path_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Foot log path notifications %s", foot_log_path_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_log_path_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("bhi360_log_path_ccc_changed: attr is NULL");
        return;
    }
    bhi360_log_path_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 log path notifications %s", bhi360_log_path_notify_enabled ? "enabled" : "disabled");
}

static void activity_log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("activity_log_ccc_changed: attr is NULL");
        return;
    }
    activity_log_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Activity log notifications %s", activity_log_notify_enabled ? "enabled" : "disabled");
}

static void activity_log_path_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("activity_log_path_ccc_changed: attr is NULL");
        return;
    }
    activity_log_path_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Activity log path notifications %s", activity_log_path_notify_enabled ? "enabled" : "disabled");
}

static void activity_step_count_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("activity_step_count_ccc_changed: attr is NULL");
        return;
    }
    activity_step_count_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Activity step count notifications %s", activity_step_count_notify_enabled ? "enabled" : "disabled");
}

// Read callbacks (optional - for debugging)
static ssize_t read_foot_sensor(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &foot_sensor_data, sizeof(foot_sensor_data));
}

static ssize_t read_status(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len, uint16_t offset)
{
    const foot_samples_t *value_to_read = static_cast<const foot_samples_t *>(attr->user_data);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value_to_read, sizeof(foot_samples_t));
}

// GATT Service Definition - Only for secondary device
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
BT_GATT_SERVICE_DEFINE(d2d_tx_svc,
    BT_GATT_PRIMARY_SERVICE(&d2d_tx_service_uuid),
    
    // Foot sensor data characteristic (notify)
    BT_GATT_CHARACTERISTIC(&d2d_foot_sensor_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_foot_sensor, NULL, static_cast<void *>(&foot_sensor_char_value)),
    BT_GATT_CCC(foot_sensor_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // BHI360 3D mapping data (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_data1_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(bhi360_data1_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // BHI360 step count data (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_data2_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(bhi360_data2_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // BHI360 linear acceleration data (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_data3_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(bhi360_data3_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Device status (notify)
    BT_GATT_CHARACTERISTIC(&d2d_status_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_status, NULL, NULL),
    BT_GATT_CCC(status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Charge status (notify)
    BT_GATT_CHARACTERISTIC(&d2d_charge_status_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(charge_status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Foot log available (notify)
    BT_GATT_CHARACTERISTIC(&d2d_foot_log_available_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(foot_log_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // BHI360 log available (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_log_available_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(bhi360_log_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Device info (notify)
    BT_GATT_CHARACTERISTIC(&d2d_device_info_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(device_info_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // FOTA progress (notify)
    BT_GATT_CHARACTERISTIC(&d2d_fota_progress_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(fota_progress_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Foot log path (notify)
    BT_GATT_CHARACTERISTIC(&d2d_foot_log_path_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(foot_log_path_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // BHI360 log path (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_log_path_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(bhi360_log_path_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Activity log available (notify)
    BT_GATT_CHARACTERISTIC(&d2d_activity_log_available_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(activity_log_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Activity log path (notify)
    BT_GATT_CHARACTERISTIC(&d2d_activity_log_path_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(activity_log_path_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Activity step count (notify)
    BT_GATT_CHARACTERISTIC(&d2d_activity_step_count_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(activity_step_count_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);
#endif // !CONFIG_PRIMARY_DEVICE

// Public functions to send notifications
int d2d_tx_notify_foot_sensor_data(const foot_samples_t *samples)
{
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    foot_samples_t temp_foot_data;
    // 1. Get the current epoch timestamp from your RTC module
    // uint32_t current_epoch = get_current_epoch_time(); // TODO: Use this when timestamp is needed

     if (!primary_conn) {
        LOG_WRN("D2D TX: Cannot send foot sensor data - no connection");
        return -ENOTCONN;
    }
    
    if (!foot_sensor_notify_enabled) {
        // Force enable since we know subscriptions are working
        foot_sensor_notify_enabled = true;
    }
    
   
    memcpy(temp_foot_data.values, samples->values, sizeof(samples->values));
    
        
    memcpy(&foot_sensor_char_value, &temp_foot_data, sizeof(foot_samples_t));

    
    // The foot sensor characteristic is at a fixed index in the service definition
    // Index 0: Primary service
    // Index 1: Foot sensor characteristic declaration
    // Index 2: Foot sensor CCC
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[1];
    
    // Get connection info for debugging
    struct bt_conn_info info;
    if (bt_conn_get_info(primary_conn, &info) == 0) {
        LOG_DBG("Connection state before notify: type=%u, role=%u, state=%u", 
                info.type, info.role, info.state);
    }
    
        
    int err = bt_gatt_notify(primary_conn, char_attr, static_cast<void *>(&foot_sensor_char_value), sizeof(foot_sensor_char_value));
    if (err) {
        LOG_ERR("Failed to send foot sensor notification: %d", err);
        if (err == -ENOTCONN) {
            LOG_ERR("Connection lost");
        } else if (err == -EINVAL) {
            LOG_ERR("Invalid parameters");
        } else if (err == -ENOTSUP) {
            LOG_ERR("Notifications not supported or not enabled");
        }
    }
    
    return err;
#else
    // Primary device doesn't have this service
    return -ENOTSUP;
#endif
}

int d2d_tx_notify_bhi360_data1(const bhi360_3d_mapping_t *data)
{
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    LOG_DBG("D2D TX: BHI360 data1 notify called");
    
    if (!primary_conn) {
        LOG_WRN("D2D TX: Cannot send BHI360 data1 - no connection");
        return -ENOTCONN;
    }
    
    if (!bhi360_data1_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send BHI360 data1 - notifications not enabled");
        return -ENOTCONN;
    }
    
    // Convert float data to fixed-point for BLE transmission
    convert_3d_mapping_to_fixed(*data, bhi360_data1_fixed);
    
    // BHI360 data1 characteristic is at index 4
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[4];
    int err = bt_gatt_notify(primary_conn, char_attr, &bhi360_data1_fixed, sizeof(bhi360_data1_fixed));
    if (err) {
        LOG_ERR("Failed to send BHI360 data1 notification: %d", err);
    }
    
    return err;
#else
    return -ENOTSUP;
#endif
}

int d2d_tx_notify_bhi360_data2(const bhi360_step_count_t *data)
{
    LOG_DBG("D2D TX: BHI360 data2 (step count) notify called - steps=%u", data->step_count);
    
    if (!primary_conn || !bhi360_data2_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send BHI360 data2 - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    // Step count data is already integers, just copy
    bhi360_data2_fixed.step_count = data->step_count;
    bhi360_data2_fixed.activity_duration_s = data->activity_duration_s;
    
    // BHI360 data2 characteristic is at index 7
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[7];
    int err = bt_gatt_notify(primary_conn, char_attr, &bhi360_data2_fixed, sizeof(bhi360_data2_fixed));
    if (err) {
        LOG_ERR("Failed to send BHI360 data2 notification: %d", err);
    }
    
    return err;
}

int d2d_tx_notify_bhi360_data3(const bhi360_linear_accel_t *data)
{
    LOG_DBG("D2D TX: BHI360 data3 (linear accel) notify called");
    
    if (!primary_conn || !bhi360_data3_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send BHI360 data3 - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    // Convert float data to fixed-point for BLE transmission
    convert_linear_accel_to_fixed(*data, bhi360_data3_fixed);
    
    // BHI360 data3 characteristic is at index 10
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[10];
    int err = bt_gatt_notify(primary_conn, char_attr, &bhi360_data3_fixed, sizeof(bhi360_data3_fixed));
    if (err) {
        LOG_ERR("Failed to send BHI360 data3 notification: %d", err);
    }
    
    return err;
}

int d2d_tx_notify_status(uint32_t status)
{
    LOG_DBG("D2D TX: Status notify called with status=0x%08x", status);
    
    if (!primary_conn) {
        LOG_WRN("D2D TX: Cannot send status - no connection");
        return -ENOTCONN;
    }
    
    if (!status_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send status - notifications not enabled");
        return -ENOTCONN;
    }
    
    device_status = status;
    
    // Status characteristic is at index 13
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[13];
    int err = bt_gatt_notify(primary_conn, char_attr, &status, sizeof(status));
    if (err) {
        LOG_ERR("Failed to send status notification: %d", err);
    } else {
        LOG_DBG("Status notification sent successfully");
    }
    
    return err;
}

int d2d_tx_notify_charge_status(uint8_t status)
{
    if (!primary_conn || !charge_status_notify_enabled) {
        return -ENOTCONN;
    }
    
    charge_status = status;
    
    // Charge status characteristic is at index 16
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[16];
    int err = bt_gatt_notify(primary_conn, char_attr, &status, sizeof(status));
    if (err) {
        LOG_ERR("Failed to send charge status notification: %d", err);
    } 
    
    return err;
}

int d2d_tx_notify_foot_log_available(uint8_t log_id)
{
    if (!primary_conn || !foot_log_notify_enabled) {
        return -ENOTCONN;
    }
    
    foot_log_available = log_id;
    
    // Foot log characteristic is at index 19
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[19];
    int err = bt_gatt_notify(primary_conn, char_attr, &log_id, sizeof(log_id));
    if (err) {
        LOG_ERR("Failed to send foot log notification: %d", err);
    } 
    
    return err;
}

int d2d_tx_notify_bhi360_log_available(uint8_t log_id)
{
    if (!primary_conn || !bhi360_log_notify_enabled) {
        return -ENOTCONN;
    }
    
    bhi360_log_available = log_id;
    
    // BHI360 log characteristic is at index 22
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[22];
    int err = bt_gatt_notify(primary_conn, char_attr, &log_id, sizeof(log_id));
    if (err) {
        LOG_ERR("Failed to send BHI360 log notification: %d", err);
    } 
    
    return err;
}

int d2d_tx_notify_device_info(const device_info_msg_t *info)
{
    if (!primary_conn || !device_info_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send device info - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    // Copy the device info data
    memcpy(&device_info_data, info, sizeof(device_info_msg_t));
    
    // Device info characteristic is at index 25
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[25];
    int err = bt_gatt_notify(primary_conn, char_attr, &device_info_data, sizeof(device_info_data));
    if (err) {
        LOG_ERR("Failed to send device info notification: %d", err);
    }
    
    return err;
}

int d2d_tx_notify_fota_progress(const fota_progress_msg_t *progress)
{
    if (!primary_conn || !fota_progress_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send FOTA progress - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    // Copy the FOTA progress data
    memcpy(&fota_progress_data, progress, sizeof(fota_progress_msg_t));
    
    // FOTA progress characteristic is at index 28 (after device info + CCC)
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[28];
    int err = bt_gatt_notify(primary_conn, char_attr, &fota_progress_data, sizeof(fota_progress_data));
    if (err) {
        LOG_ERR("Failed to send FOTA progress notification: %d", err);
    } else {
        LOG_DBG("D2D FOTA progress sent: active=%d, status=%d, percent=%d%%",
                progress->is_active, progress->status, progress->percent_complete);
    }
    
    return err;
}

int d2d_tx_notify_foot_log_path(const char *path)
{
    if (!primary_conn || !foot_log_path_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send foot log path - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    size_t len = strlen(path) + 1; // Include null terminator
    if (len > sizeof(foot_log_path)) {
        len = sizeof(foot_log_path);
    }
    
    memcpy(foot_log_path, path, len);
    foot_log_path[sizeof(foot_log_path) - 1] = '\0'; // Ensure null termination
    
    // Foot log path characteristic is at index 31
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[31];
    int err = bt_gatt_notify(primary_conn, char_attr, foot_log_path, len);
    if (err) {
        LOG_ERR("Failed to send foot log path notification: %d", err);
    } else {
        LOG_DBG("D2D foot log path sent: %s", foot_log_path);
    }
    
    return err;
}

int d2d_tx_notify_bhi360_log_path(const char *path)
{
    if (!primary_conn || !bhi360_log_path_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send BHI360 log path - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    size_t len = strlen(path) + 1;
    if (len > sizeof(bhi360_log_path)) {
        len = sizeof(bhi360_log_path);
    }
    
    memcpy(bhi360_log_path, path, len);
    bhi360_log_path[sizeof(bhi360_log_path) - 1] = '\0';
    
    // BHI360 log path characteristic is at index 34
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[34];
    int err = bt_gatt_notify(primary_conn, char_attr, bhi360_log_path, len);
    if (err) {
        LOG_ERR("Failed to send BHI360 log path notification: %d", err);
    } else {
        LOG_DBG("D2D BHI360 log path sent: %s", bhi360_log_path);
    }
    
    return err;
}

int d2d_tx_notify_activity_log_available(uint8_t log_id)
{
    if (!primary_conn || !activity_log_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send activity log available - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    activity_log_available = log_id;
    
    // Activity log available characteristic is at index 37
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[37];
    int err = bt_gatt_notify(primary_conn, char_attr, &log_id, sizeof(log_id));
    if (err) {
        LOG_ERR("Failed to send activity log notification: %d", err);
    } else {
        LOG_DBG("D2D activity log available sent: ID=%u", log_id);
    }
    
    return err;
}

int d2d_tx_notify_activity_log_path(const char *path)
{
    if (!primary_conn || !activity_log_path_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send activity log path - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    size_t len = strlen(path) + 1;
    if (len > sizeof(activity_log_path)) {
        len = sizeof(activity_log_path);
    }
    
    memcpy(activity_log_path, path, len);
    activity_log_path[sizeof(activity_log_path) - 1] = '\0';
    
    // Activity log path characteristic is at index 40
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[40];
    int err = bt_gatt_notify(primary_conn, char_attr, activity_log_path, len);
    if (err) {
        LOG_ERR("Failed to send activity log path notification: %d", err);
    } else {
        LOG_DBG("D2D activity log path sent: %s", activity_log_path);
    }
    
    return err;
}

int d2d_tx_notify_activity_step_count(const bhi360_step_count_t *data)
{
    LOG_DBG("D2D TX: Activity step count notify called - steps=%u", data->step_count);
    
    if (!primary_conn || !activity_step_count_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send activity step count - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    // Step count data is already integers, just copy
    activity_step_count_fixed.step_count = data->step_count;
    activity_step_count_fixed.activity_duration_s = 0;  // Always 0 - deprecated
    
    // Activity step count characteristic is at index 43 (after activity log path + CCC)
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[43];
    int err = bt_gatt_notify(primary_conn, char_attr, &activity_step_count_fixed, sizeof(activity_step_count_fixed));
    if (err) {
        LOG_ERR("Failed to send activity step count notification: %d", err);
    } else {
        LOG_DBG("D2D activity step count sent: %u steps", data->step_count);
    }
    
    return err;
}

void d2d_tx_service_set_connection(struct bt_conn *conn)
{
    primary_conn = conn;
    if (conn) {
        LOG_INF("D2D TX service: Primary device connected (conn=%p)", (void*)conn);
        // Get connection info for debugging
        struct bt_conn_info info;
        if (bt_conn_get_info(conn, &info) == 0) {
            LOG_INF("  Connection type: %u, role: %u, id: %u", info.type, info.role, info.id);
        }
    } else {
        LOG_INF("D2D TX service: Primary device disconnected");
        // Reset all notification states
        foot_sensor_notify_enabled = false;
        bhi360_data1_notify_enabled = false;
        bhi360_data2_notify_enabled = false;
        bhi360_data3_notify_enabled = false;
        status_notify_enabled = false;
        charge_status_notify_enabled = false;
        foot_log_notify_enabled = false;
        bhi360_log_notify_enabled = false;
        device_info_notify_enabled = false;
    }
}

void d2d_tx_service_init(void)
{
    LOG_INF("D2D TX GATT service initialized");
    LOG_INF("Service attributes at %p, count=%d", d2d_tx_svc.attrs, d2d_tx_svc.attr_count);
}
/**
 * @file ble_d2d_tx_service.cpp
 * @brief D2D TX GATT Service for Secondary Device
 * 
 * This service allows the secondary device to send sensor data to the primary device
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "ble_d2d_tx_service.hpp"
#include <app.hpp>
#include <app_fixed_point.hpp>

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

// Data buffers - using fixed-point versions for BLE transmission
static foot_samples_t foot_sensor_data;
static bhi360_3d_mapping_fixed_t bhi360_data1_fixed;
static bhi360_step_count_fixed_t bhi360_data2_fixed;
static bhi360_linear_accel_fixed_t bhi360_data3_fixed;
static uint32_t device_status = 0;
static uint8_t charge_status = 0;
static uint8_t foot_log_available = 0;
static uint8_t bhi360_log_available = 0;

// CCC changed callbacks
static void foot_sensor_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    foot_sensor_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Foot sensor notifications %s", foot_sensor_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_data1_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bhi360_data1_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 data1 notifications %s", bhi360_data1_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_data2_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bhi360_data2_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 data2 notifications %s", bhi360_data2_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_data3_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bhi360_data3_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 data3 notifications %s", bhi360_data3_notify_enabled ? "enabled" : "disabled");
}

static void status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Status notifications %s", status_notify_enabled ? "enabled" : "disabled");
}

static void charge_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    charge_status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Charge status notifications %s", charge_status_notify_enabled ? "enabled" : "disabled");
}

static void foot_log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    foot_log_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Foot log notifications %s", foot_log_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bhi360_log_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 log notifications %s", bhi360_log_notify_enabled ? "enabled" : "disabled");
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
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &device_status, sizeof(device_status));
}

// GATT Service Definition
BT_GATT_SERVICE_DEFINE(d2d_tx_svc,
    BT_GATT_PRIMARY_SERVICE(&d2d_tx_service_uuid),
    
    // Foot sensor data characteristic (notify)
    BT_GATT_CHARACTERISTIC(&d2d_foot_sensor_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_foot_sensor, NULL, NULL),
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
);

// Public functions to send notifications
int d2d_tx_notify_foot_sensor_data(const foot_samples_t *samples)
{
    static uint32_t notify_count = 0;
    notify_count++;
    
    LOG_INF("D2D TX: Attempting to send foot sensor data #%u - conn=%s, enabled=%d", 
            notify_count, primary_conn ? "yes" : "no", foot_sensor_notify_enabled);
    
    if (!primary_conn || !foot_sensor_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send foot sensor data - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    memcpy(&foot_sensor_data, samples, sizeof(foot_samples_t));
    
    // Foot sensor characteristic declaration is at index 1
    int err = bt_gatt_notify(primary_conn, &d2d_tx_svc.attrs[1], samples, sizeof(foot_samples_t));
    if (err) {
        LOG_ERR("Failed to send foot sensor notification #%u: %d", notify_count, err);
    } else {
        LOG_INF("D2D TX: Sent foot sensor data notification #%u to primary", notify_count);
    }
    
    return err;
}

int d2d_tx_notify_bhi360_data1(const bhi360_3d_mapping_t *data)
{
    LOG_INF("D2D TX: Attempting to send BHI360 3D mapping - conn=%s, enabled=%d", 
            primary_conn ? "yes" : "no", bhi360_data1_notify_enabled);
    
    if (!primary_conn || !bhi360_data1_notify_enabled) {
        LOG_WRN("D2D TX: Cannot send BHI360 data1 - no connection or notifications disabled");
        return -ENOTCONN;
    }
    
    // Convert float data to fixed-point for BLE transmission
    convert_3d_mapping_to_fixed(*data, bhi360_data1_fixed);
    
    // BHI360 data1 characteristic is at index 4
    int err = bt_gatt_notify(primary_conn, &d2d_tx_svc.attrs[4], &bhi360_data1_fixed, sizeof(bhi360_data1_fixed));
    if (err) {
        LOG_ERR("Failed to send BHI360 data1 notification: %d", err);
    } else {
        LOG_DBG("Sent BHI360 3D mapping notification (fixed-point)");
    }
    
    return err;
}

int d2d_tx_notify_bhi360_data2(const bhi360_step_count_t *data)
{
    if (!primary_conn || !bhi360_data2_notify_enabled) {
        return -ENOTCONN;
    }
    
    // Step count data is already integers, just copy
    bhi360_data2_fixed.step_count = data->step_count;
    bhi360_data2_fixed.activity_duration_s = data->activity_duration_s;
    
    // BHI360 data2 characteristic is at index 7
    int err = bt_gatt_notify(primary_conn, &d2d_tx_svc.attrs[7], &bhi360_data2_fixed, sizeof(bhi360_data2_fixed));
    if (err) {
        LOG_ERR("Failed to send BHI360 data2 notification: %d", err);
    } else {
        LOG_DBG("Sent BHI360 step count notification");
    }
    
    return err;
}

int d2d_tx_notify_bhi360_data3(const bhi360_linear_accel_t *data)
{
    if (!primary_conn || !bhi360_data3_notify_enabled) {
        return -ENOTCONN;
    }
    
    // Convert float data to fixed-point for BLE transmission
    convert_linear_accel_to_fixed(*data, bhi360_data3_fixed);
    
    // BHI360 data3 characteristic is at index 10
    int err = bt_gatt_notify(primary_conn, &d2d_tx_svc.attrs[10], &bhi360_data3_fixed, sizeof(bhi360_data3_fixed));
    if (err) {
        LOG_ERR("Failed to send BHI360 data3 notification: %d", err);
    } else {
        LOG_DBG("Sent BHI360 linear accel notification (fixed-point)");
    }
    
    return err;
}

int d2d_tx_notify_status(uint32_t status)
{
    if (!primary_conn || !status_notify_enabled) {
        return -ENOTCONN;
    }
    
    device_status = status;
    
    // Status characteristic is at index 13
    int err = bt_gatt_notify(primary_conn, &d2d_tx_svc.attrs[13], &status, sizeof(status));
    if (err) {
        LOG_ERR("Failed to send status notification: %d", err);
    } else {
        LOG_DBG("Sent status notification: 0x%08x", status);
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
    int err = bt_gatt_notify(primary_conn, &d2d_tx_svc.attrs[16], &status, sizeof(status));
    if (err) {
        LOG_ERR("Failed to send charge status notification: %d", err);
    } else {
        LOG_DBG("Sent charge status notification: %u", status);
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
    int err = bt_gatt_notify(primary_conn, &d2d_tx_svc.attrs[19], &log_id, sizeof(log_id));
    if (err) {
        LOG_ERR("Failed to send foot log notification: %d", err);
    } else {
        LOG_DBG("Sent foot log available notification: %u", log_id);
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
    int err = bt_gatt_notify(primary_conn, &d2d_tx_svc.attrs[22], &log_id, sizeof(log_id));
    if (err) {
        LOG_ERR("Failed to send BHI360 log notification: %d", err);
    } else {
        LOG_DBG("Sent BHI360 log available notification: %u", log_id);
    }
    
    return err;
}

void d2d_tx_service_set_connection(struct bt_conn *conn)
{
    primary_conn = conn;
    if (conn) {
        LOG_INF("D2D TX service: Primary device connected");
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
    }
}

void d2d_tx_service_init(void)
{
    LOG_INF("D2D TX GATT service initialized");
}
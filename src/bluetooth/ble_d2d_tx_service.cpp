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
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble_d2d_tx_service.hpp"
#include "ccc_callback_fix.hpp"
#include <app.hpp>
#include <app_fixed_point.hpp>
#include <ble_services.hpp>

LOG_MODULE_REGISTER(d2d_tx_svc, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Define the globals that were previously incorrectly defined as static in header
d2d_pending_sample_t pending_sample {};
d2d_sample_batch_t d2d_batch_buffer = {};

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

// Weight measurement UUID
static struct bt_uuid_128 d2d_weight_measurement_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e7, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// D2D batch characteristic UUID (new)
static struct bt_uuid_128 d2d_batch_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e8, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Battery level UUID
static struct bt_uuid_128 d2d_battery_level_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e9, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// D2D Metrics UUID (for parameter-only mode)
static struct bt_uuid_128 d2d_metrics_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68ea, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

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
static bool weight_measurement_notify_enabled = false;
static bool d2d_batch_notify_enabled = false;
static bool battery_level_notify_enabled = false;
static bool metrics_notify_enabled = false;
// Remove unnecessary static buffers - we'll send data directly
static foot_samples_t last_foot_sensor_data = {0};  // Keep last sent data for read callback
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
static uint16_t weight_kg_x10 = 0; // Weight in kg * 10 (for 0.1kg precision)
static uint8_t battery_level = 0; // Battery level percentage (0-100)
static d2d_metrics_packet_t metrics_data = {0}; // D2D metrics packet

// CCC changed callbacks
static void foot_sensor_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("foot_sensor_ccc_changed: attr is NULL");
        return;
    }
    bool was_enabled = foot_sensor_notify_enabled;
    foot_sensor_notify_enabled = value == BT_GATT_CCC_NOTIFY;
    LOG_INF("Foot sensor CCC changed: value=0x%04x, notifications now %s", value,
            foot_sensor_notify_enabled ? "ENABLED" : "DISABLED");

    // Log state change
    if (!was_enabled && foot_sensor_notify_enabled)
    {
        LOG_INF("Foot sensor notifications ACTIVATED by primary device");
    }
    else if (was_enabled && !foot_sensor_notify_enabled)
    {
        LOG_INF("Foot sensor notifications DEACTIVATED by primary device");
    }
}

static void bhi360_data1_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("bhi360_data1_ccc_changed: attr is NULL");
        return;
    }
    bhi360_data1_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 data1 notifications %s", bhi360_data1_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_data2_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("bhi360_data2_ccc_changed: attr is NULL");
        return;
    }
    bhi360_data2_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 data2 notifications %s", bhi360_data2_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_data3_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("bhi360_data3_ccc_changed: attr is NULL");
        return;
    }
    bhi360_data3_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 data3 notifications %s", bhi360_data3_notify_enabled ? "enabled" : "disabled");
}

static void status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("status_ccc_changed: attr is NULL");
        return;
    }
    status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Status notifications %s", status_notify_enabled ? "enabled" : "disabled");
}

static void charge_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("charge_status_ccc_changed: attr is NULL");
        return;
    }
    charge_status_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Charge status notifications %s", charge_status_notify_enabled ? "enabled" : "disabled");
}

static void foot_log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("foot_log_ccc_changed: attr is NULL");
        return;
    }
    foot_log_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Foot log notifications %s", foot_log_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("bhi360_log_ccc_changed: attr is NULL");
        return;
    }
    bhi360_log_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 log notifications %s", bhi360_log_notify_enabled ? "enabled" : "disabled");
}

static void device_info_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("device_info_ccc_changed: attr is NULL");
        return;
    }
    device_info_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Device info notifications %s", device_info_notify_enabled ? "enabled" : "disabled");
}

static void fota_progress_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("fota_progress_ccc_changed: attr is NULL");
        return;
    }
    fota_progress_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("FOTA progress notifications %s", fota_progress_notify_enabled ? "enabled" : "disabled");
}

static void foot_log_path_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("foot_log_path_ccc_changed: attr is NULL");
        return;
    }
    foot_log_path_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Foot log path notifications %s", foot_log_path_notify_enabled ? "enabled" : "disabled");
}

static void bhi360_log_path_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("bhi360_log_path_ccc_changed: attr is NULL");
        return;
    }
    bhi360_log_path_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("BHI360 log path notifications %s", bhi360_log_path_notify_enabled ? "enabled" : "disabled");
}

static void activity_log_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("activity_log_ccc_changed: attr is NULL");
        return;
    }
    activity_log_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Activity log notifications %s", activity_log_notify_enabled ? "enabled" : "disabled");
}

static void activity_log_path_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("activity_log_path_ccc_changed: attr is NULL");
        return;
    }
    activity_log_path_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Activity log path notifications %s", activity_log_path_notify_enabled ? "enabled" : "disabled");
}

static void activity_step_count_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("activity_step_count_ccc_changed: attr is NULL");
        return;
    }
    activity_step_count_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Activity step count notifications %s", activity_step_count_notify_enabled ? "enabled" : "disabled");
}

static void weight_measurement_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("weight_measurement_ccc_changed: attr is NULL");
        return;
    }
    weight_measurement_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Weight measurement notifications %s", weight_measurement_notify_enabled ? "enabled" : "disabled");
}

static void d2d_batch_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("d2d_batch_ccc_changed: attr is NULL");
        return;
    }
    d2d_batch_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("D2D batch notifications %s", d2d_batch_notify_enabled ? "enabled" : "disabled");
}

static void battery_level_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("battery_level_ccc_changed: attr is NULL");
        return;
    }
    battery_level_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Battery level notifications %s", battery_level_notify_enabled ? "enabled" : "disabled");
}

static void metrics_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("metrics_ccc_changed: attr is NULL");
        return;
    }
    metrics_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("D2D Metrics notifications %s", metrics_notify_enabled ? "enabled" : "disabled");
}
// Read callback for D2D batch characteristic
static ssize_t read_d2d_batch(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                              uint16_t offset)
{
    // Return the most recent batch buffer (from header)
    extern d2d_sample_batch_t d2d_batch_buffer;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &d2d_batch_buffer, sizeof(d2d_sample_batch_t));
}

// Read callbacks (optional - for debugging)
static ssize_t read_foot_sensor(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                uint16_t offset)
{
    // Return the last sent foot sensor data
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &last_foot_sensor_data, sizeof(last_foot_sensor_data));
}

static ssize_t read_status(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                           uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &device_status, sizeof(device_status));
}

// GATT Service Definition - Only for secondary device
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
BT_GATT_SERVICE_DEFINE(
    d2d_tx_svc, BT_GATT_PRIMARY_SERVICE(&d2d_tx_service_uuid),

    // Foot sensor data characteristic (notify)
    BT_GATT_CHARACTERISTIC(&d2d_foot_sensor_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
                           read_foot_sensor, NULL, NULL),
    BT_GATT_CCC(foot_sensor_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // BHI360 3D mapping data (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_data1_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(bhi360_data1_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // BHI360 step count data (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_data2_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(bhi360_data2_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // BHI360 linear acceleration data (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_data3_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(bhi360_data3_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Device status (notify)
    BT_GATT_CHARACTERISTIC(&d2d_status_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
                           read_status, NULL, NULL),
    BT_GATT_CCC(status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Charge status (notify)
    BT_GATT_CHARACTERISTIC(&d2d_charge_status_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(charge_status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Foot log available (notify)
    BT_GATT_CHARACTERISTIC(&d2d_foot_log_available_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(foot_log_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // BHI360 log available (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_log_available_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL,
                           NULL),
    BT_GATT_CCC(bhi360_log_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Device info (notify)
    BT_GATT_CHARACTERISTIC(&d2d_device_info_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(device_info_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // FOTA progress (notify)
    BT_GATT_CHARACTERISTIC(&d2d_fota_progress_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(fota_progress_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Foot log path (notify)
    BT_GATT_CHARACTERISTIC(&d2d_foot_log_path_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(foot_log_path_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // BHI360 log path (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_log_path_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(bhi360_log_path_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Activity log available (notify)
    BT_GATT_CHARACTERISTIC(&d2d_activity_log_available_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL,
                           NULL),
    BT_GATT_CCC(activity_log_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Activity log path (notify)
    BT_GATT_CHARACTERISTIC(&d2d_activity_log_path_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(activity_log_path_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Activity step count (notify)
    BT_GATT_CHARACTERISTIC(&d2d_activity_step_count_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL,
                           NULL),
    BT_GATT_CCC(activity_step_count_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Weight measurement (notify)
    BT_GATT_CHARACTERISTIC(&d2d_weight_measurement_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(weight_measurement_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // D2D batch (notify)
    BT_GATT_CHARACTERISTIC(&d2d_batch_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_READ,
                           read_d2d_batch, NULL, NULL),
    BT_GATT_CCC(d2d_batch_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Battery level (notify)
    BT_GATT_CHARACTERISTIC(&d2d_battery_level_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(battery_level_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // D2D Metrics (notify) - for parameter-only mode
    BT_GATT_CHARACTERISTIC(&d2d_metrics_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(metrics_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );
#endif // !CONFIG_PRIMARY_DEVICE


// --- D2D batching logic (fixed-point for BLE optimization) ---
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)

// Helper: Convert foot_samples_t to fixed-point foot_samples_t (if not already fixed-point)
static void convert_foot_samples_to_fixed(const foot_samples_t *src, foot_samples_t *dst)
{
    // If foot_samples_t is already fixed-point, this is a memcpy
    memcpy(dst, src, sizeof(foot_samples_t));
}

// Helper: Convert bhi360_log_record_t to fixed-point bhi360_log_record_fixed_t
static void convert_bhi360_log_record_to_fixed(const bhi360_log_record_t *src, bhi360_log_record_fixed_t *dst)
{
    // Convert each float field to fixed-point (Q15 or Q8 as appropriate)
    // Use scale factors as defined in FixedPoint namespace (see Fixed_Point_Conversion_Guide.md)
    dst->lacc_x = float_to_fixed16(src->lacc_x, FixedPoint::ACCEL_SCALE);
    dst->lacc_y = float_to_fixed16(src->lacc_y, FixedPoint::ACCEL_SCALE);
    dst->lacc_z = float_to_fixed16(src->lacc_z, FixedPoint::ACCEL_SCALE);
    dst->gyro_x = float_to_fixed16(src->gyro_x, FixedPoint::GYRO_SCALE);
    dst->gyro_y = float_to_fixed16(src->gyro_y, FixedPoint::GYRO_SCALE);
    dst->gyro_z = float_to_fixed16(src->gyro_z, FixedPoint::GYRO_SCALE);
    // Add more fields if needed, using the correct scale for each
    // Add more fields if needed
}

static void try_combine_and_buffer(void)
{
    if (pending_sample.foot_ready && pending_sample.imu_ready)
    {
        d2d_batch_buffer.timestamp[0] = pending_sample.timestamp;
        // Convert and store foot sample as fixed-point
        convert_foot_samples_to_fixed(&pending_sample.foot, &d2d_batch_buffer.foot[0]);
        // Convert and store IMU sample as fixed-point (cast to correct type)
        convert_bhi360_log_record_to_fixed(&pending_sample.imu, (bhi360_log_record_fixed_t *)&d2d_batch_buffer.imu[0]);

        // Reset pending
        pending_sample.foot_ready = false;
        pending_sample.imu_ready = false;

        // Send the single-sample batch
        if (primary_conn && d2d_batch_notify_enabled)
        {
            const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[d2d_tx_svc.attr_count - 2];
            int err = bt_gatt_notify(primary_conn, char_attr, &d2d_batch_buffer, sizeof(d2d_sample_batch_t));
            if (err)
            {
                LOG_ERR("Failed to send D2D batch notification: %d", err);
            }
            else
            {
                LOG_DBG("D2D batch notification sent");
            }
        }
    }
}

int d2d_tx_notify_foot_sensor_data(const foot_samples_t *samples)
{
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!primary_conn)
    {
        LOG_WRN("D2D TX: Cannot send foot sensor data - no connection");
        return -ENOTCONN;
    }
    
    if (!foot_sensor_notify_enabled)
    {
        // Force enable since we know subscriptions are working
        foot_sensor_notify_enabled = true;
        LOG_INF("D2D TX: Force-enabled foot sensor notifications");
    }
    
    // Save the data for read callbacks
    memcpy(&last_foot_sensor_data, samples, sizeof(foot_samples_t));
    
    // Log the values we're about to send
    LOG_INF("D2D TX: Sending foot sensor data - values[0-7]: %u %u %u %u %u %u %u %u",
            samples->values[0], samples->values[1],
            samples->values[2], samples->values[3],
            samples->values[4], samples->values[5],
            samples->values[6], samples->values[7]);
    
    // Simply send the raw data directly without any copying or conversion
    // The foot sensor characteristic is at a fixed index in the service definition
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[1];
    
    // Send the data directly from the input parameter
    int err = bt_gatt_notify(primary_conn, char_attr, samples, sizeof(foot_samples_t));
    
    if (err)
    {
        LOG_ERR("Failed to send foot sensor notification: %d", err);
        if (err == -ENOTCONN)
        {
            LOG_ERR("Connection lost");
        }
        else if (err == -EINVAL)
        {
            LOG_ERR("Invalid parameters");
        }
        else if (err == -ENOTSUP)
        {
            LOG_ERR("Notifications not supported or not enabled");
        }
    }
    else
    {
        LOG_DBG("D2D TX: Foot sensor data sent successfully");
    }
    
    // Remove the batching logic since we won't need it after legacy removal
    // Just keep it simple - send raw data directly
    
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
    if (!primary_conn)
    {
        LOG_WRN("D2D TX: Cannot send BHI360 data1 - no connection");
        return -ENOTCONN;
    }
    if (!bhi360_data1_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send BHI360 data1 - notifications not enabled");
        return -ENOTCONN;
    }
    // Convert float data to fixed-point for BLE transmission
    convert_3d_mapping_to_fixed(*data, bhi360_data1_fixed);
    // BHI360 data1 characteristic is at index 4
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[4];
    int err = bt_gatt_notify(primary_conn, char_attr, &bhi360_data1_fixed, sizeof(bhi360_data1_fixed));
    if (err)
    {
        LOG_ERR("Failed to send BHI360 data1 notification: %d", err);
    }
    // --- Batch buffering logic ---
    memset(&pending_sample.imu, 0, sizeof(pending_sample.imu));
    // Map incoming 3D mapping data to bhi360_log_record_t fields
    pending_sample.imu.lacc_x = data->accel_x;
    pending_sample.imu.lacc_y = data->accel_y;
    pending_sample.imu.lacc_z = data->accel_z;
    pending_sample.imu.gyro_x = data->gyro_x;
    pending_sample.imu.gyro_y = data->gyro_y;
    pending_sample.imu.gyro_z = data->gyro_z;
    // If you have quaternion data, map it here as well if available in data
    pending_sample.imu_ready = true;
    try_combine_and_buffer();
    return err;
#else
    return -ENOTSUP;
#endif
}

int d2d_tx_notify_bhi360_data3(const bhi360_linear_accel_t *data)
{
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    LOG_DBG("D2D TX: BHI360 data3 (linear accel) notify called");
    if (!primary_conn || !bhi360_data3_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send BHI360 data3 - no connection or notifications disabled");
        return -ENOTCONN;
    }
    // Convert float data to fixed-point for BLE transmission
    convert_linear_accel_to_fixed(*data, bhi360_data3_fixed);
    // BHI360 data3 characteristic is at index 10
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[10];
    int err = bt_gatt_notify(primary_conn, char_attr, &bhi360_data3_fixed, sizeof(bhi360_data3_fixed));
    if (err)
    {
        LOG_ERR("Failed to send BHI360 data3 notification: %d", err);
    }
    // --- Batch buffering logic ---
    memset(&pending_sample.imu, 0, sizeof(pending_sample.imu));
    // Only fill linear acceleration fields for batch
    pending_sample.imu.lacc_x = data->x;
    pending_sample.imu.lacc_y = data->y;
    pending_sample.imu.lacc_z = data->z;
    pending_sample.imu_ready = true;
    try_combine_and_buffer();
    return err;
#else
    return -ENOTSUP;
#endif
}
#else
int d2d_tx_notify_foot_sensor_data(const foot_samples_t *samples)
{
    return -ENOTSUP;
}
int d2d_tx_notify_bhi360_data1(const bhi360_3d_mapping_t *data)
{
    return -ENOTSUP;
}
int d2d_tx_notify_bhi360_data3(const bhi360_linear_accel_t *data)
{
    return -ENOTSUP;
}
#endif

int d2d_tx_notify_bhi360_data2(const bhi360_step_count_t *data)
{
    LOG_DBG("D2D TX: BHI360 data2 (step count) notify called - steps=%u", data->step_count);

    if (!primary_conn || !bhi360_data2_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send BHI360 data2 - no connection or notifications disabled");
        return -ENOTCONN;
    }

    // Step count data is already integers, just copy
    bhi360_data2_fixed.step_count = data->step_count;
    bhi360_data2_fixed.activity_duration_s = data->activity_duration_s;

    // BHI360 data2 characteristic is at index 7
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[7];
    int err = bt_gatt_notify(primary_conn, char_attr, &bhi360_data2_fixed, sizeof(bhi360_data2_fixed));
    if (err)
    {
        LOG_ERR("Failed to send BHI360 data2 notification: %d", err);
    }

    return err;
}

int d2d_tx_notify_status(uint32_t status)
{
    LOG_DBG("D2D TX: Status notify called with status=0x%08x", status);

    if (!primary_conn)
    {
        LOG_WRN("D2D TX: Cannot send status - no connection");
        return -ENOTCONN;
    }

    if (!status_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send status - notifications not enabled");
        return -ENOTCONN;
    }

    device_status = status;

    // Status characteristic is at index 13
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[13];
    int err = bt_gatt_notify(primary_conn, char_attr, &status, sizeof(status));
    if (err)
    {
        LOG_ERR("Failed to send status notification: %d", err);
    }
    else
    {
        LOG_DBG("Status notification sent successfully");
    }

    return err;
}

int d2d_tx_notify_charge_status(uint8_t status)
{
    if (!primary_conn || !charge_status_notify_enabled)
    {
        return -ENOTCONN;
    }

    charge_status = status;

    // Charge status characteristic is at index 16
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[16];
    int err = bt_gatt_notify(primary_conn, char_attr, &status, sizeof(status));
    if (err)
    {
        LOG_ERR("Failed to send charge status notification: %d", err);
    }

    return err;
}

int d2d_tx_notify_foot_log_available(uint8_t log_id)
{
    if (!primary_conn || !foot_log_notify_enabled)
    {
        return -ENOTCONN;
    }

    foot_log_available = log_id;

    // Foot log characteristic is at index 19
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[19];
    int err = bt_gatt_notify(primary_conn, char_attr, &log_id, sizeof(log_id));
    if (err)
    {
        LOG_ERR("Failed to send foot log notification: %d", err);
    }

    return err;
}

int d2d_tx_notify_bhi360_log_available(uint8_t log_id)
{
    if (!primary_conn || !bhi360_log_notify_enabled)
    {
        return -ENOTCONN;
    }

    bhi360_log_available = log_id;

    // BHI360 log characteristic is at index 22
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[22];
    int err = bt_gatt_notify(primary_conn, char_attr, &log_id, sizeof(log_id));
    if (err)
    {
        LOG_ERR("Failed to send BHI360 log notification: %d", err);
    }

    return err;
}

int d2d_tx_notify_device_info(const device_info_msg_t *info)
{
    if (!primary_conn || !device_info_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send device info - no connection or notifications disabled");
        return -ENOTCONN;
    }

    // Copy the device info data
    memcpy(&device_info_data, info, sizeof(device_info_msg_t));

    // Device info characteristic is at index 25
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[25];
    int err = bt_gatt_notify(primary_conn, char_attr, &device_info_data, sizeof(device_info_data));
    if (err)
    {
        LOG_ERR("Failed to send device info notification: %d", err);
    }

    return err;
}

int d2d_tx_notify_fota_progress(const fota_progress_msg_t *progress)
{
    if (!primary_conn || !fota_progress_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send FOTA progress - no connection or notifications disabled");
        return -ENOTCONN;
    }

    // Copy the FOTA progress data
    memcpy(&fota_progress_data, progress, sizeof(fota_progress_msg_t));

    // FOTA progress characteristic is at index 28 (after device info + CCC)
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[28];
    int err = bt_gatt_notify(primary_conn, char_attr, &fota_progress_data, sizeof(fota_progress_data));
    if (err)
    {
        LOG_ERR("Failed to send FOTA progress notification: %d", err);
    }
    else
    {
        LOG_DBG("D2D FOTA progress sent: active=%d, status=%d, percent=%d%%", progress->is_active, progress->status,
                progress->percent_complete);
    }

    return err;
}

int d2d_tx_notify_foot_log_path(const char *path)
{
    if (!primary_conn || !foot_log_path_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send foot log path - no connection or notifications disabled");
        return -ENOTCONN;
    }

    size_t len = strlen(path) + 1; // Include null terminator
    if (len > sizeof(foot_log_path))
    {
        len = sizeof(foot_log_path);
    }

    memcpy(foot_log_path, path, len);
    foot_log_path[sizeof(foot_log_path) - 1] = '\0'; // Ensure null termination

    // Foot log path characteristic is at index 31
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[31];
    int err = bt_gatt_notify(primary_conn, char_attr, foot_log_path, len);
    if (err)
    {
        LOG_ERR("Failed to send foot log path notification: %d", err);
    }
    else
    {
        LOG_DBG("D2D foot log path sent: %s", foot_log_path);
    }

    return err;
}

int d2d_tx_notify_bhi360_log_path(const char *path)
{
    if (!primary_conn || !bhi360_log_path_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send BHI360 log path - no connection or notifications disabled");
        return -ENOTCONN;
    }

    size_t len = strlen(path) + 1;
    if (len > sizeof(bhi360_log_path))
    {
        len = sizeof(bhi360_log_path);
    }

    memcpy(bhi360_log_path, path, len);
    bhi360_log_path[sizeof(bhi360_log_path) - 1] = '\0';

    // BHI360 log path characteristic is at index 34
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[34];
    int err = bt_gatt_notify(primary_conn, char_attr, bhi360_log_path, len);
    if (err)
    {
        LOG_ERR("Failed to send BHI360 log path notification: %d", err);
    }
    else
    {
        LOG_DBG("D2D BHI360 log path sent: %s", bhi360_log_path);
    }

    return err;
}

int d2d_tx_notify_activity_log_available(uint8_t log_id)
{
    if (!primary_conn || !activity_log_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send activity log available - no connection or notifications disabled");
        return -ENOTCONN;
    }

    activity_log_available = log_id;

    // Activity log available characteristic is at index 37
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[37];
    int err = bt_gatt_notify(primary_conn, char_attr, &log_id, sizeof(log_id));
    if (err)
    {
        LOG_ERR("Failed to send activity log notification: %d", err);
    }
    else
    {
        LOG_DBG("D2D activity log available sent: ID=%u", log_id);
    }

    return err;
}

int d2d_tx_notify_activity_log_path(const char *path)
{
    if (!primary_conn || !activity_log_path_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send activity log path - no connection or notifications disabled");
        return -ENOTCONN;
    }

    size_t len = strlen(path) + 1;
    if (len > sizeof(activity_log_path))
    {
        len = sizeof(activity_log_path);
    }

    memcpy(activity_log_path, path, len);
    activity_log_path[sizeof(activity_log_path) - 1] = '\0';

    // Activity log path characteristic is at index 40
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[40];
    int err = bt_gatt_notify(primary_conn, char_attr, activity_log_path, len);
    if (err)
    {
        LOG_ERR("Failed to send activity log path notification: %d", err);
    }
    else
    {
        LOG_DBG("D2D activity log path sent: %s", activity_log_path);
    }

    return err;
}

int d2d_tx_notify_activity_step_count(const bhi360_step_count_t *data)
{
    LOG_DBG("D2D TX: Activity step count notify called - steps=%u", data->step_count);

    if (!primary_conn || !activity_step_count_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send activity step count - no connection or notifications disabled");
        return -ENOTCONN;
    }

    // Step count data is already integers, just copy
    activity_step_count_fixed.step_count = data->step_count;
    activity_step_count_fixed.activity_duration_s = 0; // Always 0 - deprecated

    // Activity step count characteristic is at index 43 (after activity log path + CCC)
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[43];
    int err = bt_gatt_notify(primary_conn, char_attr, &activity_step_count_fixed, sizeof(activity_step_count_fixed));
    if (err)
    {
        LOG_ERR("Failed to send activity step count notification: %d", err);
    }
    else
    {
        LOG_DBG("D2D activity step count sent: %u steps", data->step_count);
    }

    return err;
}

int d2d_tx_notify_weight_measurement(float weight_kg)
{
    LOG_INF("D2D TX: Weight measurement notify called - weight=%.1f kg", weight_kg);

    if (!primary_conn || !weight_measurement_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send weight measurement - no connection or notifications disabled");
        return -ENOTCONN;
    }

    // Convert to uint16_t with 0.1kg precision
    weight_kg_x10 = (uint16_t)(weight_kg * 10);

    // Weight measurement characteristic is at index 46 (after activity step count + CCC)
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[46];
    int err = bt_gatt_notify(primary_conn, char_attr, &weight_kg_x10, sizeof(weight_kg_x10));
    if (err)
    {
        LOG_ERR("Failed to send weight measurement notification: %d", err);
    }
    else
    {
        LOG_INF("D2D weight measurement sent: %.1f kg (raw=%u)", weight_kg, weight_kg_x10);
    }

    return err;
}

void d2d_tx_service_set_connection(struct bt_conn *conn)
{
    primary_conn = conn;
    if (conn)
    {
        LOG_INF("D2D TX service: Primary device connected (conn=%p)", (void *)conn);
        // Get connection info for debugging
        struct bt_conn_info info;
        if (bt_conn_get_info(conn, &info) == 0)
        {
            LOG_INF("  Connection type: %u, role: %u, id: %u", info.type, info.role, info.id);
        }
    }
    else
    {
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
        fota_progress_notify_enabled = false;
        foot_log_path_notify_enabled = false;
        bhi360_log_path_notify_enabled = false;
        activity_log_notify_enabled = false;
        activity_log_path_notify_enabled = false;
        activity_step_count_notify_enabled = false;
        weight_measurement_notify_enabled = false;
        d2d_batch_notify_enabled = false;
        battery_level_notify_enabled = false;
        metrics_notify_enabled = false;
    }
}
// Send D2D batch notification
int d2d_tx_notify_d2d_batch(const d2d_sample_batch_t *batch)
{
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!primary_conn || !d2d_batch_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send D2D batch - no connection or notifications disabled");
        return -ENOTCONN;
    }

    // D2D batch characteristic is at the last index (after weight measurement + CCC)
    // Find the correct index: count all attrs, batch char is last two (char+ccc)
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[d2d_tx_svc.attr_count - 2];
    int err = bt_gatt_notify(primary_conn, char_attr, batch, sizeof(d2d_sample_batch_t));
    if (err)
    {
        LOG_ERR("Failed to send D2D batch notification: %d", err);
    }
    else
    {
        //empty
    }
    return err;
#else
    return -ENOTSUP;
#endif
}

int d2d_tx_notify_battery_level(uint8_t level)
{
    LOG_INF("D2D TX: Battery level notify called - level=%u%%", level);

    if (!primary_conn || !battery_level_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send battery level - no connection or notifications disabled");
        return -ENOTCONN;
    }

    battery_level = level;

    // Battery level characteristic is at index 51 (after D2D batch + CCC)
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[51];
    int err = bt_gatt_notify(primary_conn, char_attr, &battery_level, sizeof(battery_level));
    if (err)
    {
        LOG_ERR("Failed to send battery level notification: %d", err);
    }
    else
    {
        LOG_INF("D2D battery level sent: %u%%", level);
    }

    return err;
}

int d2d_tx_notify_metrics(const d2d_metrics_packet_t *metrics)
{
    LOG_INF("D2D TX: Metrics notify called - seq=%u, status=%u",
            metrics->sequence_num, metrics->calculation_status);

    if (!primary_conn || !metrics_notify_enabled)
    {
        LOG_WRN("D2D TX: Cannot send metrics - no connection or notifications disabled");
        return -ENOTCONN;
    }

    // Copy the metrics data
    memcpy(&metrics_data, metrics, sizeof(d2d_metrics_packet_t));

    // D2D Metrics characteristic is at the last index (after battery level + CCC)
    const struct bt_gatt_attr *char_attr = &d2d_tx_svc.attrs[d2d_tx_svc.attr_count - 2];
    int err = bt_gatt_notify(primary_conn, char_attr, &metrics_data, sizeof(metrics_data));
    if (err)
    {
        LOG_ERR("Failed to send D2D metrics notification: %d", err);
    }
    else
    {
        // Count valid bits in the valid_mask array
        int valid_count = 0;
        for (int i = 0; i < sizeof(metrics->valid_mask); i++) {
            valid_count += __builtin_popcount(metrics->valid_mask[i]);
        }
        LOG_INF("D2D metrics sent: seq=%u, status=%u, valid_count=%d",
                metrics->sequence_num, metrics->calculation_status, valid_count);
    }

    return err;
}

void d2d_tx_service_init(void)
{
    LOG_INF("D2D TX GATT service initialized");
    LOG_INF("Service attributes at %p, count=%d", d2d_tx_svc.attrs, d2d_tx_svc.attr_count);
}

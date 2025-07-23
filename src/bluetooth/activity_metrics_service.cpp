/**
 * @file activity_metrics_service.cpp
 * @author Botz Innovation
 * @brief Activity Metrics Service - Real-time running metrics for mobile apps
 * @version 1.0
 * @date 2025-01
 *
 * @copyright Copyright (c) 2025 Botz Innovation
 *
 * This service provides real-time activity metrics calculated by the realtime_metrics
 * module. Data is sent at 1Hz with packed formats for efficiency.
 */

#define MODULE activity_metrics_service

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <app.hpp>
#include <app_fixed_point.hpp>
#include "../realtime_metrics/realtime_metrics.h"

LOG_MODULE_REGISTER(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Service UUID: 4fd5b690-9d89-4061-92aa-319ca786baae
static struct bt_uuid_128 ACTIVITY_METRICS_SERVICE_UUID = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b690, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Characteristic UUIDs (using same pattern as information service)
// Real-time Metrics: ...b691
static struct bt_uuid_128 realtime_metrics_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b691, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Asymmetry Metrics: ...b692
static struct bt_uuid_128 asymmetry_metrics_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b692, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Biomechanics Extended: ...b693
static struct bt_uuid_128 biomechanics_extended_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b693, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Session Summary: ...b694
static struct bt_uuid_128 session_summary_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b694, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// GPS Data (Write): ...b695
static struct bt_uuid_128 gps_data_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b695, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Total Step Count (moved from Information Service): ...b696
static struct bt_uuid_128 total_step_count_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b696, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Activity Step Count (moved from Information Service): ...b697
static struct bt_uuid_128 activity_step_count_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b697, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// BLE data structures (packed for efficiency)
typedef struct __attribute__((packed)) {
    uint16_t cadence_spm;        // 0-1: Steps per minute
    uint16_t pace_sec_km;        // 2-3: Seconds per kilometer
    uint32_t distance_m;         // 4-7: Distance in meters
    uint8_t  form_score;         // 8: Form score 0-100
    int8_t   balance_lr_pct;     // 9: Balance -50 to +50
    uint16_t ground_contact_ms;  // 10-11: Average ground contact time
    uint16_t flight_time_ms;     // 12-13: Average flight time
    uint8_t  efficiency_score;   // 14: Efficiency 0-100
    uint8_t  alerts;            // 15: Alert flags
    uint32_t reserved;          // 16-19: Reserved for future use
} realtime_metrics_ble_t;

typedef struct __attribute__((packed)) {
    uint8_t contact_time_asym;   // 0: Contact time asymmetry %
    uint8_t flight_time_asym;    // 1: Flight time asymmetry %
    uint8_t force_asym;          // 2: Force asymmetry %
    uint8_t pronation_asym;      // 3: Pronation asymmetry %
    uint8_t strike_left;         // 4: Left strike pattern (0=heel,1=mid,2=fore)
    uint8_t strike_right;        // 5: Right strike pattern
    uint16_t reserved;           // 6-7: Reserved
} asymmetry_metrics_ble_t;

typedef struct __attribute__((packed)) {
    int8_t  pronation_left;      // 0: Left pronation degrees
    int8_t  pronation_right;     // 1: Right pronation degrees
    uint16_t loading_rate_left;  // 2-3: Left loading rate N/s
    uint16_t loading_rate_right; // 4-5: Right loading rate N/s
    uint8_t arch_collapse_left;  // 6: Left arch collapse 0-100
    uint8_t arch_collapse_right; // 7: Right arch collapse 0-100
    uint32_t reserved;           // 8-11: Reserved
} biomechanics_extended_ble_t;

typedef struct __attribute__((packed)) {
    uint32_t total_distance_m;   // 0-3: Total distance meters
    uint16_t avg_pace_sec_km;    // 4-5: Average pace
    uint16_t avg_cadence_spm;    // 6-7: Average cadence
    uint32_t total_steps;        // 8-11: Total steps
    uint16_t calories_kcal;      // 12-13: Calories burned
    uint8_t  avg_form_score;     // 14: Average form score
    uint8_t  reserved;           // 15: Reserved
    uint32_t duration_sec;       // 16-19: Session duration seconds
} session_summary_ble_t;

typedef struct __attribute__((packed)) {
    int32_t  latitude_e7;        // 0-3: Latitude * 10^7
    int32_t  longitude_e7;       // 4-7: Longitude * 10^7
    uint16_t distance_delta_m;   // 8-9: Distance since last update
    uint8_t  accuracy_m;         // 10: GPS accuracy in meters
    int16_t  elevation_change_m; // 11-12: Elevation change
    uint8_t  gps_mode;          // 13: GPS mode (0=off,1=calib,2=precise,3=race)
    uint16_t reserved;           // 14-15: Reserved
} gps_data_ble_t;

// Static data storage
static realtime_metrics_ble_t realtime_metrics_value = {0};
static asymmetry_metrics_ble_t asymmetry_metrics_value = {0};
static biomechanics_extended_ble_t biomechanics_extended_value = {0};
static session_summary_ble_t session_summary_value = {0};
static gps_data_ble_t gps_data_value = {
    .latitude_e7 = 0,
    .longitude_e7 = 0,
    .distance_delta_m = 0,
    .accuracy_m = 0,
    .elevation_change_m = 0,
    .gps_mode = 0,
    .reserved = 0
};

// Step count storage (using same format as Information Service)
static bhi360_step_count_fixed_t total_step_count_value = {0, 0};
static bhi360_step_count_fixed_t activity_step_count_value = {0, 0};

// CCC (Client Characteristic Configuration) storage
static bool realtime_metrics_notify_enabled = false;
static bool asymmetry_metrics_notify_enabled = false;
static bool biomechanics_extended_notify_enabled = false;
static bool session_summary_notify_enabled = false;
static bool total_step_count_notify_enabled = false;
static bool activity_step_count_notify_enabled = false;

// Connection tracking
static struct bt_conn *current_conn = NULL;

// Forward declarations
static ssize_t ams_realtime_metrics_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
void *buf, uint16_t len, uint16_t offset);
static void ams_realtime_metrics_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t ams_asymmetry_metrics_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
void *buf, uint16_t len, uint16_t offset);
static void ams_asymmetry_metrics_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t ams_biomechanics_extended_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
void *buf, uint16_t len, uint16_t offset);
static void ams_biomechanics_extended_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t ams_session_summary_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
void *buf, uint16_t len, uint16_t offset);
static void ams_session_summary_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t ams_gps_data_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

// Step count characteristics
static ssize_t ams_total_step_count_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
void *buf, uint16_t len, uint16_t offset);
static void ams_total_step_count_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

static ssize_t ams_activity_step_count_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
void *buf, uint16_t len, uint16_t offset);
static void ams_activity_step_count_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

// Service definition (following information_service.cpp template)
BT_GATT_SERVICE_DEFINE(
    activity_metrics_service, 
    BT_GATT_PRIMARY_SERVICE(&ACTIVITY_METRICS_SERVICE_UUID),
    
    // Real-time Metrics Characteristic (20 bytes, Notify, 1Hz)
    BT_GATT_CHARACTERISTIC(&realtime_metrics_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ_ENCRYPT,
                          ams_realtime_metrics_read, nullptr,
                          static_cast<void *>(&realtime_metrics_value)),
    BT_GATT_CCC(ams_realtime_metrics_ccc_cfg_changed, 
                BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Asymmetry Metrics Characteristic (8 bytes, Notify, 1Hz)
    BT_GATT_CHARACTERISTIC(&asymmetry_metrics_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ_ENCRYPT,
                          ams_asymmetry_metrics_read, nullptr,
                          static_cast<void *>(&asymmetry_metrics_value)),
    BT_GATT_CCC(ams_asymmetry_metrics_ccc_cfg_changed,
                BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Biomechanics Extended Characteristic (12 bytes, Read/Notify, On-demand)
    BT_GATT_CHARACTERISTIC(&biomechanics_extended_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ_ENCRYPT,
                          ams_biomechanics_extended_read, nullptr,
                          static_cast<void *>(&biomechanics_extended_value)),
    BT_GATT_CCC(ams_biomechanics_extended_ccc_cfg_changed,
                BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Session Summary Characteristic (20 bytes, Read/Notify, End of session)
    BT_GATT_CHARACTERISTIC(&session_summary_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ_ENCRYPT,
                          ams_session_summary_read, nullptr,
                          static_cast<void *>(&session_summary_value)),
    BT_GATT_CCC(ams_session_summary_ccc_cfg_changed,
                BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // GPS Data Characteristic (16 bytes, Write, From phone)
    BT_GATT_CHARACTERISTIC(&gps_data_uuid.uuid,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE_ENCRYPT,
                          nullptr, ams_gps_data_write,
                          static_cast<void *>(&gps_data_value)),
    
    // Total Step Count Characteristic (aggregated from both feet)
    BT_GATT_CHARACTERISTIC(&total_step_count_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ_ENCRYPT,
                          ams_total_step_count_read, nullptr,
                          static_cast<void *>(&total_step_count_value)),
    BT_GATT_CCC(ams_total_step_count_ccc_cfg_changed,
                BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    
    // Activity Step Count Characteristic (steps during current activity)
    BT_GATT_CHARACTERISTIC(&activity_step_count_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ_ENCRYPT,
                          ams_activity_step_count_read, nullptr,
                          static_cast<void *>(&activity_step_count_value)),
    BT_GATT_CCC(ams_activity_step_count_ccc_cfg_changed,
                BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT)
);

// --- Real-time Metrics Handlers ---
static void ams_realtime_metrics_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("ams_realtime_metrics_ccc_cfg_changed: attr is NULL");
        return;
    }

    realtime_metrics_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Real-time metrics notifications %s", 
            realtime_metrics_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_realtime_metrics_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                        void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, 
                            &realtime_metrics_value, sizeof(realtime_metrics_value));
}

// --- Asymmetry Metrics Handlers ---
static void ams_asymmetry_metrics_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("ams_asymmetry_metrics_ccc_cfg_changed: attr is NULL");
        return;
    }

    asymmetry_metrics_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Asymmetry metrics notifications %s",
            asymmetry_metrics_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_asymmetry_metrics_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            &asymmetry_metrics_value, sizeof(asymmetry_metrics_value));
}

// --- Biomechanics Extended Handlers ---
static void ams_biomechanics_extended_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("ams_biomechanics_extended_ccc_cfg_changed: attr is NULL");
        return;
    }

    biomechanics_extended_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Biomechanics extended notifications %s",
            biomechanics_extended_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_biomechanics_extended_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                             void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            &biomechanics_extended_value, sizeof(biomechanics_extended_value));
}

// --- Session Summary Handlers ---
static void ams_session_summary_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("ams_session_summary_ccc_cfg_changed: attr is NULL");
        return;
    }

    session_summary_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Session summary notifications %s",
            session_summary_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_session_summary_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            &session_summary_value, sizeof(session_summary_value));
}

// --- GPS Data Handler ---
static ssize_t ams_gps_data_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    
    if (offset + len > sizeof(gps_data_ble_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy((uint8_t *)&gps_data_value + offset, buf, len);

    if (offset + len == sizeof(gps_data_ble_t)) {
        // Complete GPS data received
        LOG_INF("GPS data received: lat=%d, lon=%d, accuracy=%dm",
                gps_data_value.latitude_e7, gps_data_value.longitude_e7,
                gps_data_value.accuracy_m);
        
        // TODO: Send to activity_metrics module for processing
        // For now, just log the data
    }

    return len;
}

// --- Total Step Count Handlers ---
static void ams_total_step_count_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("ams_total_step_count_ccc_cfg_changed: attr is NULL");
        return;
    }

    total_step_count_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Total step count notifications %s",
            total_step_count_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_total_step_count_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                        void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            &total_step_count_value, sizeof(total_step_count_value));
}

// --- Activity Step Count Handlers ---
static void ams_activity_step_count_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("ams_activity_step_count_ccc_cfg_changed: attr is NULL");
        return;
    }

    activity_step_count_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Activity step count notifications %s",
            activity_step_count_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_activity_step_count_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                            &activity_step_count_value, sizeof(activity_step_count_value));
}

// --- Public API Functions ---

/**
 * @brief Update real-time metrics from realtime_metrics module
 * Note: This should only be called from the bluetooth thread after receiving
 * a message via the bluetooth message queue
 * 
 * @param metrics Pointer to metrics data
 */
void ams_update_realtime_metrics(const realtime_metrics_t *metrics)
{
    if (!metrics) {
        return;
    }

    // Update the BLE structure
    realtime_metrics_value.cadence_spm = metrics->cadence_spm;
    realtime_metrics_value.pace_sec_km = metrics->pace_sec_km;
    realtime_metrics_value.distance_m = metrics->distance_m;
    realtime_metrics_value.form_score = metrics->form_score;
    realtime_metrics_value.balance_lr_pct = metrics->balance_lr_pct;
    realtime_metrics_value.ground_contact_ms = metrics->ground_contact_ms;
    realtime_metrics_value.flight_time_ms = metrics->flight_time_ms;
    realtime_metrics_value.efficiency_score = metrics->efficiency_score;
    realtime_metrics_value.alerts = metrics->alerts;

    // Send notification if enabled and connected
    if (realtime_metrics_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn, 
                                &activity_metrics_service.attrs[2],
                                &realtime_metrics_value, 
                                sizeof(realtime_metrics_value));
        if (err) {
            LOG_WRN("Failed to send real-time metrics notification: %d", err);
        }
    }

    // Also update asymmetry metrics
    asymmetry_metrics_value.contact_time_asym = metrics->contact_time_asymmetry;
    asymmetry_metrics_value.flight_time_asym = metrics->flight_time_asymmetry;
    asymmetry_metrics_value.force_asym = metrics->force_asymmetry;
    asymmetry_metrics_value.pronation_asym = metrics->pronation_asymmetry;
    asymmetry_metrics_value.strike_left = metrics->left_strike_pattern;
    asymmetry_metrics_value.strike_right = metrics->right_strike_pattern;

    // Send asymmetry notification if enabled
    if (asymmetry_metrics_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &activity_metrics_service.attrs[5],
                                &asymmetry_metrics_value,
                                sizeof(asymmetry_metrics_value));
        if (err) {
            LOG_WRN("Failed to send asymmetry metrics notification: %d", err);
        }
    }
}

/**
 * @brief Update biomechanics extended data
 * Note: This should only be called from the bluetooth thread
 * 
 * @param data Pointer to biomechanics data
 */
void ams_update_biomechanics_extended(const biomechanics_extended_ble_t *data)
{
    if (!data) {
        return;
    }

    memcpy(&biomechanics_extended_value, data, sizeof(biomechanics_extended_ble_t));

    // Send notification if enabled
    if (biomechanics_extended_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &activity_metrics_service.attrs[8],
                                &biomechanics_extended_value,
                                sizeof(biomechanics_extended_value));
        if (err) {
            LOG_WRN("Failed to send biomechanics extended notification: %d", err);
        }
    }
}

/**
 * @brief Update session summary (typically at end of activity)
 * Note: This should only be called from the bluetooth thread
 * 
 * @param summary Pointer to session summary data
 */
void ams_update_session_summary(const session_summary_ble_t *summary)
{
    if (!summary) {
        return;
    }

    memcpy(&session_summary_value, summary, sizeof(session_summary_ble_t));

    // Send notification if enabled
    if (session_summary_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &activity_metrics_service.attrs[11],
                                &session_summary_value,
                                sizeof(session_summary_value));
        if (err) {
            LOG_WRN("Failed to send session summary notification: %d", err);
        }
    }
}

/**
 * @brief Set the current BLE connection for notifications
 * Note: This should only be called from the bluetooth thread
 * 
 * @param conn The BLE connection handle
 */
void ams_set_connection(struct bt_conn *conn)
{
    current_conn = conn;
    
    if (!conn) {
        // Connection lost, reset notification states
        realtime_metrics_notify_enabled = false;
        asymmetry_metrics_notify_enabled = false;
        biomechanics_extended_notify_enabled = false;
        session_summary_notify_enabled = false;
        total_step_count_notify_enabled = false;
        activity_step_count_notify_enabled = false;
    }
}

/**
 * @brief Update total step count (aggregated from both feet)
 * Note: This should only be called from the bluetooth thread
 * 
 * @param total_steps Total step count
 */
void ams_update_total_step_count(uint32_t total_steps)
{
    total_step_count_value.step_count = total_steps;
    total_step_count_value.activity_duration_s = 0;  // Deprecated - always 0

    // Send notification if enabled and connected
    if (total_step_count_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &activity_metrics_service.attrs[15],  // Attribute index for total step count
                                &total_step_count_value,
                                sizeof(total_step_count_value));
        if (err) {
            LOG_WRN("Failed to send total step count notification: %d", err);
        }
    }
}

void ams_update_activity_step_count(uint32_t activity_steps)
{
    activity_step_count_value.step_count = activity_steps;
    activity_step_count_value.activity_duration_s = 0;  // Deprecated - always 0

    // Send notification if enabled and connected
    if (activity_step_count_notify_enabled && current_conn) {
        int err = bt_gatt_notify(current_conn,
                                &activity_metrics_service.attrs[18],  // Attribute index for activity step count
                                &activity_step_count_value,
                                sizeof(activity_step_count_value));
        if (err) {
            LOG_WRN("Failed to send activity step count notification: %d", err);
        }
    }
}

/**
 * @brief Get the last GPS data received from phone
 * Note: This should only be called from the bluetooth thread
 * 
 * @param data Output buffer for GPS data
 * @return true if GPS data is available
 */
bool ams_get_gps_data(gps_data_ble_t *data)
{
    if (!data) {
        return false;
    }

    // Check if we have valid GPS data
    if (gps_data_value.latitude_e7 == 0 && gps_data_value.longitude_e7 == 0) {
        return false;
    }

    memcpy(data, &gps_data_value, sizeof(gps_data_ble_t));
    return true;
}
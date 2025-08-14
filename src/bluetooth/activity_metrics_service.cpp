/**
 * @file activity_metrics_service.cpp
 * @author Botz Innovation
 * @brief Activity Metrics Service - Real-time running metrics for mobile apps
 * @version 1.0
 * @date 2025-01
 *
 * @copyright Copyright (c) 2025 Botz Innovation
 *
 * This service provides real-time activity metrics calculated by the
 * realtime_metrics module. Data is sent at 1Hz with packed formats for
 * efficiency.
 */

#define MODULE activity_metrics_service

#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

#include "../realtime_metrics/realtime_metrics.h"
#include <app.hpp>
#include <app_fixed_point.hpp>

LOG_MODULE_REGISTER(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Forward declarations

static ssize_t ams_stride_duration_ms_read(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset);
static void
ams_stride_duration_ms_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value);
static ssize_t ams_stride_duration_asym_read(struct bt_conn *conn,
                                             const struct bt_gatt_attr *attr,
                                             void *buf, uint16_t len,
                                             uint16_t offset);
static void
ams_stride_duration_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                         uint16_t value);
static ssize_t ams_stride_length_cm_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset);
static void
ams_stride_length_cm_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value);
static ssize_t ams_stride_length_asym_read(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset);
static void
ams_stride_length_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value);

static ssize_t ams_stride_length_avg_mm_read(struct bt_conn *conn,
                                             const struct bt_gatt_attr *attr,
                                             void *buf, uint16_t len,
                                             uint16_t offset);
static void
ams_stride_length_avg_mm_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                         uint16_t value);

static ssize_t ams_run_speed_cms_read(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len,
                                      uint16_t offset);
static void
ams_run_speed_cms_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                  uint16_t value);

// Service UUID: 4fd5b690-9d89-4061-92aa-319ca786baae
static struct bt_uuid_128 ACTIVITY_METRICS_SERVICE_UUID = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b690, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Characteristic UUIDs for individual metrics (using same pattern as
// information service) Real-time Metrics individual UUIDs
static struct bt_uuid_128 cadence_spm_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b691, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 pace_sec_km_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b698, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 distance_m_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b699, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 form_score_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b69a, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 balance_lr_pct_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b69b, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 ground_contact_ms_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b69c, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 flight_time_ms_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b69d, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 efficiency_score_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b69e, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 alerts_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b69f, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Asymmetry Metrics individual UUIDs
static struct bt_uuid_128 contact_time_asym_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a0, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 flight_time_asym_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a1, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 force_asym_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a2, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 pronation_asym_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a3, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 strike_left_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a4, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 strike_right_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a5, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Biomechanics Extended individual UUIDs
static struct bt_uuid_128 pronation_left_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a6, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 pronation_right_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a7, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 loading_rate_left_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a8, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 loading_rate_right_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6a9, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 arch_collapse_left_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6aa, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 arch_collapse_right_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6ab, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Session Summary individual UUIDs
static struct bt_uuid_128 total_distance_m_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6ac, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 avg_pace_sec_km_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6ad, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 avg_cadence_spm_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6ae, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 total_steps_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6af, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 calories_kcal_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6b0, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 avg_form_score_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6b1, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 duration_sec_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6b2, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Stride Metrics individual UUIDs
static struct bt_uuid_128 stride_duration_ms_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6b3, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 stride_duration_asym_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6b4, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 stride_length_cm_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6b5, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 stride_length_asym_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6b6, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// New stride metrics - Stride Length Average and Run Speed
static struct bt_uuid_128 stride_length_avg_mm_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6b7, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
static struct bt_uuid_128 run_speed_cms_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b6b8, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// GPS Data (Write): ...b695
static struct bt_uuid_128 gps_data_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b695, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Total Step Count (moved from Information Service): ...b696
static struct bt_uuid_128 total_step_count_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b696, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Activity Step Count (moved from Information Service): ...b697
static struct bt_uuid_128 activity_step_count_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b697, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
// Packed structures for legacy update functions and GPS write
struct gps_data_ble_t {
  int32_t latitude_e7;
  int32_t longitude_e7;
  uint16_t distance_delta_m;
  uint8_t accuracy_m;
  int16_t elevation_change_m;
  uint8_t gps_mode;
} __packed;

static gps_data_ble_t gps_data_value = {0};

struct biomechanics_extended_ble_t {
  int8_t pronation_left;
  int8_t pronation_right;
  uint16_t loading_rate_left;
  uint16_t loading_rate_right;
  uint8_t arch_collapse_left;
  uint8_t arch_collapse_right;
} __packed;

struct session_summary_ble_t {
  uint32_t total_distance_m;
  uint16_t avg_pace_sec_km;
  uint16_t avg_cadence_spm;
  uint32_t total_steps;
  uint16_t calories_kcal;
  uint8_t avg_form_score;
  uint32_t duration_sec;
} __packed;

// Stride Metrics individual variables
static uint16_t stride_duration_ms_value = 0;
static uint8_t stride_duration_asym_value = 0;
static uint16_t stride_length_cm_value = 0;
static uint8_t stride_length_asym_value = 0;
static uint16_t stride_length_avg_mm_value = 0; // Average stride length in millimeters
static uint16_t run_speed_cms_value = 0; // Run speed in cm/s (m/s × 100)

// CCC for stride metrics
static bool stride_duration_ms_notify_enabled = false;
static bool stride_duration_asym_notify_enabled = false;
static bool stride_length_cm_notify_enabled = false;
static bool stride_length_asym_notify_enabled = false;
static bool stride_length_avg_mm_notify_enabled = false;
static bool run_speed_cms_notify_enabled = false;

// Individual metric variables for BLE characteristics
// Real-time Metrics individual variables
static uint16_t cadence_spm_value = 0;       // Steps per minute
static uint16_t pace_sec_km_value = 0;       // Seconds per kilometer
static uint32_t distance_m_value = 0;        // Distance in meters
static uint8_t form_score_value = 0;         // Form score 0-100
static int8_t balance_lr_pct_value = 0;      // Balance -50 to +50
static uint16_t ground_contact_ms_value = 0; // Average ground contact time
static uint16_t flight_time_ms_value = 0;    // Average flight time
static uint8_t efficiency_score_value = 0;   // Efficiency 0-100
static uint8_t alerts_value = 0;             // Alert flags

// Asymmetry Metrics individual variables
static uint8_t contact_time_asym_value = 0; // Contact time asymmetry %
static uint8_t flight_time_asym_value = 0;  // Flight time asymmetry %
static uint8_t force_asym_value = 0;        // Force asymmetry %
static uint8_t pronation_asym_value = 0;    // Pronation asymmetry %
static uint8_t strike_left_value =
    0; // Left strike pattern (0=heel,1=mid,2=fore)
static uint8_t strike_right_value = 0; // Right strike pattern

// Biomechanics Extended individual variables
static int8_t pronation_left_value = 0;       // Left pronation degrees
static int8_t pronation_right_value = 0;      // Right pronation degrees
static uint16_t loading_rate_left_value = 0;  // Left loading rate N/s
static uint16_t loading_rate_right_value = 0; // Right loading rate N/s
static uint8_t arch_collapse_left_value = 0;  // Left arch collapse 0-100
static uint8_t arch_collapse_right_value = 0; // Right arch collapse 0-100

// Session Summary individual variables
static uint32_t total_distance_m_value = 0; // Total distance meters
static uint16_t avg_pace_sec_km_value = 0;  // Average pace
static uint16_t avg_cadence_spm_value = 0;  // Average cadence
static uint32_t total_steps_value = 0;      // Total steps
static uint16_t calories_kcal_value = 0;    // Calories burned
static uint8_t avg_form_score_value = 0;    // Average form score
static uint32_t duration_sec_value = 0;     // Session duration seconds

// GPS Data individual variables
static int32_t latitude_e7_value = 0;        // Latitude * 10^7
static int32_t longitude_e7_value = 0;       // Longitude * 10^7
static uint16_t distance_delta_m_value = 0;  // Distance since last update
static uint8_t accuracy_m_value = 0;         // GPS accuracy in meters
static int16_t elevation_change_m_value = 0; // Elevation change
static uint8_t gps_mode_value = 0; // GPS mode (0=off,1=calib,2=precise,3=race)

// Static data storage for step counts

// Step count storage (using same format as Information Service)
static bhi360_step_count_fixed_t total_step_count_value = {0, 0};
static bhi360_step_count_fixed_t activity_step_count_value = {0, 0};

// CCC (Client Characteristic Configuration) storage for individual
// characteristics
static bool cadence_spm_notify_enabled = false;
static bool pace_sec_km_notify_enabled = false;
static bool distance_m_notify_enabled = false;
static bool form_score_notify_enabled = false;
static bool balance_lr_pct_notify_enabled = false;
static bool ground_contact_ms_notify_enabled = false;
static bool flight_time_ms_notify_enabled = false;
static bool efficiency_score_notify_enabled = false;
static bool alerts_notify_enabled = false;

static bool contact_time_asym_notify_enabled = false;
static bool flight_time_asym_notify_enabled = false;
static bool force_asym_notify_enabled = false;
static bool pronation_asym_notify_enabled = false;
static bool strike_left_notify_enabled = false;
static bool strike_right_notify_enabled = false;

static bool pronation_left_notify_enabled = false;
static bool pronation_right_notify_enabled = false;
static bool loading_rate_left_notify_enabled = false;
static bool loading_rate_right_notify_enabled = false;
static bool arch_collapse_left_notify_enabled = false;
static bool arch_collapse_right_notify_enabled = false;

static bool total_distance_m_notify_enabled = false;
static bool avg_pace_sec_km_notify_enabled = false;
static bool avg_cadence_spm_notify_enabled = false;
static bool total_steps_notify_enabled = false;
static bool calories_kcal_notify_enabled = false;
static bool avg_form_score_notify_enabled = false;
static bool duration_sec_notify_enabled = false;

static bool total_step_count_notify_enabled = false;
static bool activity_step_count_notify_enabled = false;

// Connection tracking
static struct bt_conn *current_conn = NULL;

// Forward declarations for individual characteristics
// Real-time Metrics
static ssize_t ams_cadence_spm_read(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, void *buf,
                                    uint16_t len, uint16_t offset);
static void ams_cadence_spm_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                            uint16_t value);

static ssize_t ams_pace_sec_km_read(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, void *buf,
                                    uint16_t len, uint16_t offset);
static void ams_pace_sec_km_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                            uint16_t value);

static ssize_t ams_distance_m_read(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr, void *buf,
                                   uint16_t len, uint16_t offset);
static void ams_distance_m_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                           uint16_t value);

static ssize_t ams_form_score_read(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr, void *buf,
                                   uint16_t len, uint16_t offset);
static void ams_form_score_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                           uint16_t value);

static ssize_t ams_balance_lr_pct_read(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len,
                                       uint16_t offset);
static void ams_balance_lr_pct_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                               uint16_t value);

static ssize_t ams_ground_contact_ms_read(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len,
                                          uint16_t offset);
static void
ams_ground_contact_ms_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                      uint16_t value);

static ssize_t ams_flight_time_ms_read(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len,
                                       uint16_t offset);
static void ams_flight_time_ms_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                               uint16_t value);

static ssize_t ams_efficiency_score_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset);
static void
ams_efficiency_score_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value);

static ssize_t ams_alerts_read(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr, void *buf,
                               uint16_t len, uint16_t offset);
static void ams_alerts_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value);

// Asymmetry Metrics
static ssize_t ams_contact_time_asym_read(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len,
                                          uint16_t offset);
static void
ams_contact_time_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                      uint16_t value);

static ssize_t ams_flight_time_asym_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset);
static void
ams_flight_time_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value);

static ssize_t ams_force_asym_read(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr, void *buf,
                                   uint16_t len, uint16_t offset);
static void ams_force_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                           uint16_t value);

static ssize_t ams_pronation_asym_read(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len,
                                       uint16_t offset);
static void ams_pronation_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                               uint16_t value);

static ssize_t ams_strike_left_read(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, void *buf,
                                    uint16_t len, uint16_t offset);
static void ams_strike_left_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                            uint16_t value);

static ssize_t ams_strike_right_read(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr, void *buf,
                                     uint16_t len, uint16_t offset);
static void ams_strike_right_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                             uint16_t value);

// Biomechanics Extended
static ssize_t ams_pronation_left_read(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len,
                                       uint16_t offset);
static void ams_pronation_left_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                               uint16_t value);

static ssize_t ams_pronation_right_read(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        void *buf, uint16_t len,
                                        uint16_t offset);
static void ams_pronation_right_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                                uint16_t value);

static ssize_t ams_loading_rate_left_read(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len,
                                          uint16_t offset);
static void
ams_loading_rate_left_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                      uint16_t value);

static ssize_t ams_loading_rate_right_read(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset);
static void
ams_loading_rate_right_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value);

static ssize_t ams_arch_collapse_left_read(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset);
static void
ams_arch_collapse_left_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value);

static ssize_t ams_arch_collapse_right_read(struct bt_conn *conn,
                                            const struct bt_gatt_attr *attr,
                                            void *buf, uint16_t len,
                                            uint16_t offset);
static void
ams_arch_collapse_right_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                        uint16_t value);

// Session Summary
static ssize_t ams_total_distance_m_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset);
static void
ams_total_distance_m_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value);

static ssize_t ams_avg_pace_sec_km_read(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        void *buf, uint16_t len,
                                        uint16_t offset);
static void ams_avg_pace_sec_km_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                                uint16_t value);

static ssize_t ams_avg_cadence_spm_read(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        void *buf, uint16_t len,
                                        uint16_t offset);
static void ams_avg_cadence_spm_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                                uint16_t value);

static ssize_t ams_total_steps_read(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, void *buf,
                                    uint16_t len, uint16_t offset);
static void ams_total_steps_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                            uint16_t value);

static ssize_t ams_calories_kcal_read(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset);

static void ams_calories_kcal_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                              uint16_t value);

static ssize_t ams_avg_form_score_read(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len,
                                       uint16_t offset);
static void ams_avg_form_score_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                               uint16_t value);

static ssize_t ams_duration_sec_read(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr, void *buf,
                                     uint16_t len, uint16_t offset);
static void ams_duration_sec_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                             uint16_t value);

// GPS Data write handler
static ssize_t ams_gps_data_write(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len,
                                  uint16_t offset, uint8_t flags);

// Step count characteristics
static ssize_t ams_total_step_count_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset);
static void
ams_total_step_count_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value);

static ssize_t ams_activity_step_count_read(struct bt_conn *conn,
                                            const struct bt_gatt_attr *attr,
                                            void *buf, uint16_t len,
                                            uint16_t offset);
static void
ams_activity_step_count_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                        uint16_t value);

// Service definition with individual characteristics for each metric
BT_GATT_SERVICE_DEFINE(
    activity_metrics_service,
    BT_GATT_PRIMARY_SERVICE(&ACTIVITY_METRICS_SERVICE_UUID),

    // Real-time Metrics Individual Characteristics (Notify, 1Hz)
    BT_GATT_CHARACTERISTIC(&cadence_spm_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_cadence_spm_read,
                           nullptr, static_cast<void *>(&cadence_spm_value)),
    BT_GATT_CCC(ams_cadence_spm_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&pace_sec_km_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_pace_sec_km_read,
                           nullptr, static_cast<void *>(&pace_sec_km_value)),
    BT_GATT_CCC(ams_pace_sec_km_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&distance_m_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_distance_m_read,
                           nullptr, static_cast<void *>(&distance_m_value)),
    BT_GATT_CCC(ams_distance_m_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&form_score_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_form_score_read,
                           nullptr, static_cast<void *>(&form_score_value)),
    BT_GATT_CCC(ams_form_score_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&balance_lr_pct_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_balance_lr_pct_read,
                           nullptr, static_cast<void *>(&balance_lr_pct_value)),
    BT_GATT_CCC(ams_balance_lr_pct_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&ground_contact_ms_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_ground_contact_ms_read, nullptr,
                           static_cast<void *>(&ground_contact_ms_value)),
    BT_GATT_CCC(ams_ground_contact_ms_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&flight_time_ms_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_flight_time_ms_read,
                           nullptr, static_cast<void *>(&flight_time_ms_value)),
    BT_GATT_CCC(ams_flight_time_ms_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&efficiency_score_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_efficiency_score_read,
                           nullptr,
                           static_cast<void *>(&efficiency_score_value)),
    BT_GATT_CCC(ams_efficiency_score_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&alerts_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_alerts_read, nullptr,
                           static_cast<void *>(&alerts_value)),
    BT_GATT_CCC(ams_alerts_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Asymmetry Metrics Individual Characteristics (Notify, 1Hz)
    BT_GATT_CHARACTERISTIC(&contact_time_asym_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_contact_time_asym_read, nullptr,
                           static_cast<void *>(&contact_time_asym_value)),
    BT_GATT_CCC(ams_contact_time_asym_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&flight_time_asym_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_flight_time_asym_read,
                           nullptr,
                           static_cast<void *>(&flight_time_asym_value)),
    BT_GATT_CCC(ams_flight_time_asym_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&force_asym_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_force_asym_read,
                           nullptr, static_cast<void *>(&force_asym_value)),
    BT_GATT_CCC(ams_force_asym_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&pronation_asym_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_pronation_asym_read,
                           nullptr, static_cast<void *>(&pronation_asym_value)),
    BT_GATT_CCC(ams_pronation_asym_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&strike_left_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_strike_left_read,
                           nullptr, static_cast<void *>(&strike_left_value)),
    BT_GATT_CCC(ams_strike_left_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&strike_right_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_strike_right_read,
                           nullptr, static_cast<void *>(&strike_right_value)),
    BT_GATT_CCC(ams_strike_right_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Biomechanics Extended Individual Characteristics (Read/Notify, On-demand)
    BT_GATT_CHARACTERISTIC(&pronation_left_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_pronation_left_read,
                           nullptr, static_cast<void *>(&pronation_left_value)),
    BT_GATT_CCC(ams_pronation_left_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&pronation_right_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_pronation_right_read,
                           nullptr,
                           static_cast<void *>(&pronation_right_value)),
    BT_GATT_CCC(ams_pronation_right_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&loading_rate_left_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_loading_rate_left_read, nullptr,
                           static_cast<void *>(&loading_rate_left_value)),
    BT_GATT_CCC(ams_loading_rate_left_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&loading_rate_right_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_loading_rate_right_read, nullptr,
                           static_cast<void *>(&loading_rate_right_value)),
    BT_GATT_CCC(ams_loading_rate_right_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&arch_collapse_left_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_arch_collapse_left_read, nullptr,
                           static_cast<void *>(&arch_collapse_left_value)),
    BT_GATT_CCC(ams_arch_collapse_left_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&arch_collapse_right_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_arch_collapse_right_read, nullptr,
                           static_cast<void *>(&arch_collapse_right_value)),
    BT_GATT_CCC(ams_arch_collapse_right_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Session Summary Individual Characteristics (Read/Notify, End of session)
    BT_GATT_CHARACTERISTIC(&total_distance_m_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_total_distance_m_read,
                           nullptr,
                           static_cast<void *>(&total_distance_m_value)),
    BT_GATT_CCC(ams_total_distance_m_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&avg_pace_sec_km_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_avg_pace_sec_km_read,
                           nullptr,
                           static_cast<void *>(&avg_pace_sec_km_value)),
    BT_GATT_CCC(ams_avg_pace_sec_km_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&avg_cadence_spm_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_avg_cadence_spm_read,
                           nullptr,
                           static_cast<void *>(&avg_cadence_spm_value)),
    BT_GATT_CCC(ams_avg_cadence_spm_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&total_steps_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_total_steps_read,
                           nullptr, static_cast<void *>(&total_steps_value)),
    BT_GATT_CCC(ams_total_steps_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&calories_kcal_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_calories_kcal_read,
                           nullptr, static_cast<void *>(&calories_kcal_value)),
    BT_GATT_CCC(ams_calories_kcal_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&avg_form_score_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_avg_form_score_read,
                           nullptr, static_cast<void *>(&avg_form_score_value)),
    BT_GATT_CCC(ams_avg_form_score_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&duration_sec_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_duration_sec_read,
                           nullptr, static_cast<void *>(&duration_sec_value)),
    BT_GATT_CCC(ams_duration_sec_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // GPS Data Characteristic (Write, From phone)
    BT_GATT_CHARACTERISTIC(&gps_data_uuid.uuid, BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE, nullptr,
                           ams_gps_data_write, nullptr),

    // Stride Metrics Individual Characteristics (Notify, 1Hz)
    BT_GATT_CHARACTERISTIC(&stride_duration_ms_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_stride_duration_ms_read, nullptr,
                           static_cast<void *>(&stride_duration_ms_value)),
    BT_GATT_CCC(ams_stride_duration_ms_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&stride_duration_asym_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_stride_duration_asym_read, nullptr,
                           static_cast<void *>(&stride_duration_asym_value)),
    BT_GATT_CCC(ams_stride_duration_asym_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&stride_length_cm_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_stride_length_cm_read,
                           nullptr,
                           static_cast<void *>(&stride_length_cm_value)),
    BT_GATT_CCC(ams_stride_length_cm_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    BT_GATT_CHARACTERISTIC(&stride_length_asym_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_stride_length_asym_read, nullptr,
                           static_cast<void *>(&stride_length_asym_value)),
    BT_GATT_CCC(ams_stride_length_asym_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Stride Length Average (mm)
    BT_GATT_CHARACTERISTIC(&stride_length_avg_mm_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_stride_length_avg_mm_read, nullptr,
                           static_cast<void *>(&stride_length_avg_mm_value)),
    BT_GATT_CCC(ams_stride_length_avg_mm_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Run Speed (cm/s = m/s × 100)
    BT_GATT_CHARACTERISTIC(&run_speed_cms_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_run_speed_cms_read, nullptr,
                           static_cast<void *>(&run_speed_cms_value)),
    BT_GATT_CCC(ams_run_speed_cms_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Total Step Count Characteristic (aggregated from both feet)
    BT_GATT_CHARACTERISTIC(&total_step_count_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, ams_total_step_count_read,
                           nullptr,
                           static_cast<void *>(&total_step_count_value)),
    BT_GATT_CCC(ams_total_step_count_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Activity Step Count Characteristic (steps during current activity)
    BT_GATT_CHARACTERISTIC(&activity_step_count_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           ams_activity_step_count_read, nullptr,
                           static_cast<void *>(&activity_step_count_value)),
    BT_GATT_CCC(ams_activity_step_count_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

// --- Real-time Metrics Individual Handlers ---
static void ams_cadence_spm_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                            uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_cadence_spm_ccc_cfg_changed: attr is NULL");
    return;
  }
  cadence_spm_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Cadence SPM notifications %s",
          cadence_spm_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_cadence_spm_read(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, void *buf,
                                    uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &cadence_spm_value,
                           sizeof(cadence_spm_value));
}

static void ams_pace_sec_km_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                            uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_pace_sec_km_ccc_cfg_changed: attr is NULL");
    return;
  }
  pace_sec_km_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Pace Sec/Km notifications %s",
          pace_sec_km_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_pace_sec_km_read(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, void *buf,
                                    uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &pace_sec_km_value,
                           sizeof(pace_sec_km_value));
}

static void ams_distance_m_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                           uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_distance_m_ccc_cfg_changed: attr is NULL");
    return;
  }
  distance_m_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Distance M notifications %s",
          distance_m_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_distance_m_read(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr, void *buf,
                                   uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &distance_m_value,
                           sizeof(distance_m_value));
}

static void ams_form_score_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                           uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_form_score_ccc_cfg_changed: attr is NULL");
    return;
  }
  form_score_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Form Score notifications %s",
          form_score_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_form_score_read(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr, void *buf,
                                   uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &form_score_value,
                           sizeof(form_score_value));
}

static void ams_balance_lr_pct_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                               uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_balance_lr_pct_ccc_cfg_changed: attr is NULL");
    return;
  }
  balance_lr_pct_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Balance L/R Pct notifications %s",
          balance_lr_pct_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_balance_lr_pct_read(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len,
                                       uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &balance_lr_pct_value,
                           sizeof(balance_lr_pct_value));
}

static void
ams_ground_contact_ms_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                      uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_ground_contact_ms_ccc_cfg_changed: attr is NULL");
    return;
  }
  ground_contact_ms_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Ground Contact MS notifications %s",
          ground_contact_ms_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_ground_contact_ms_read(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len,
                                          uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &ground_contact_ms_value,
                           sizeof(ground_contact_ms_value));
}

static void ams_flight_time_ms_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                               uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_flight_time_ms_ccc_cfg_changed: attr is NULL");
    return;
  }
  flight_time_ms_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Flight Time MS notifications %s",
          flight_time_ms_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_flight_time_ms_read(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len,
                                       uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &flight_time_ms_value,
                           sizeof(flight_time_ms_value));
}

static void
ams_efficiency_score_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_efficiency_score_ccc_cfg_changed: attr is NULL");
    return;
  }
  efficiency_score_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Efficiency Score notifications %s",
          efficiency_score_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_efficiency_score_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &efficiency_score_value,
                           sizeof(efficiency_score_value));
}

static void ams_alerts_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_alerts_ccc_cfg_changed: attr is NULL");
    return;
  }
  alerts_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Alerts notifications %s",
          alerts_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_alerts_read(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr, void *buf,
                               uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &alerts_value,
                           sizeof(alerts_value));
}

// --- Asymmetry Metrics Individual Handlers ---
static void
ams_contact_time_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                      uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_contact_time_asym_ccc_cfg_changed: attr is NULL");
    return;
  }
  contact_time_asym_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Contact Time Asymmetry notifications %s",
          contact_time_asym_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_contact_time_asym_read(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len,
                                          uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &contact_time_asym_value,
                           sizeof(contact_time_asym_value));
}

static void
ams_flight_time_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_flight_time_asym_ccc_cfg_changed: attr is NULL");
    return;
  }
  flight_time_asym_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Flight Time Asymmetry notifications %s",
          flight_time_asym_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_flight_time_asym_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &flight_time_asym_value,
                           sizeof(flight_time_asym_value));
}

static void ams_force_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                           uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_force_asym_ccc_cfg_changed: attr is NULL");
    return;
  }
  force_asym_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Force Asymmetry notifications %s",
          force_asym_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_force_asym_read(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr, void *buf,
                                   uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &force_asym_value,
                           sizeof(force_asym_value));
}

static void ams_pronation_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                               uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_pronation_asym_ccc_cfg_changed: attr is NULL");
    return;
  }
  pronation_asym_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Pronation Asymmetry notifications %s",
          pronation_asym_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_pronation_asym_read(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len,
                                       uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &pronation_asym_value,
                           sizeof(pronation_asym_value));
}

static void ams_strike_left_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                            uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_strike_left_ccc_cfg_changed: attr is NULL");
    return;
  }
  strike_left_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Strike Left notifications %s",
          strike_left_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_strike_left_read(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, void *buf,
                                    uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &strike_left_value,
                           sizeof(strike_left_value));
}

static void ams_strike_right_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                             uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_strike_right_ccc_cfg_changed: attr is NULL");
    return;
  }
  strike_right_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Strike Right notifications %s",
          strike_right_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_strike_right_read(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr, void *buf,
                                     uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &strike_right_value,
                           sizeof(strike_right_value));
}

// --- Biomechanics Extended Individual Handlers ---
static void ams_pronation_left_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                               uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_pronation_left_ccc_cfg_changed: attr is NULL");
    return;
  }
  pronation_left_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Pronation Left notifications %s",
          pronation_left_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_pronation_left_read(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len,
                                       uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &pronation_left_value,
                           sizeof(pronation_left_value));
}

static void ams_pronation_right_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                                uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_pronation_right_ccc_cfg_changed: attr is NULL");
    return;
  }
  pronation_right_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Pronation Right notifications %s",
          pronation_right_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_pronation_right_read(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        void *buf, uint16_t len,
                                        uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &pronation_right_value,
                           sizeof(pronation_right_value));
}

static void
ams_loading_rate_left_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                      uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_loading_rate_left_ccc_cfg_changed: attr is NULL");
    return;
  }
  loading_rate_left_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Loading Rate Left notifications %s",
          loading_rate_left_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_loading_rate_left_read(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          void *buf, uint16_t len,
                                          uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &loading_rate_left_value,
                           sizeof(loading_rate_left_value));
}

static void
ams_loading_rate_right_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_loading_rate_right_ccc_cfg_changed: attr is NULL");
    return;
  }
  loading_rate_right_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Loading Rate Right notifications %s",
          loading_rate_right_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_loading_rate_right_read(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &loading_rate_right_value,
                           sizeof(loading_rate_right_value));
}

static void
ams_arch_collapse_left_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_arch_collapse_left_ccc_cfg_changed: attr is NULL");
    return;
  }
  arch_collapse_left_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Arch Collapse Left notifications %s",
          arch_collapse_left_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_arch_collapse_left_read(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &arch_collapse_left_value,
                           sizeof(arch_collapse_left_value));
}

static void
ams_arch_collapse_right_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                        uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_arch_collapse_right_ccc_cfg_changed: attr is NULL");
    return;
  }
  arch_collapse_right_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Arch Collapse Right notifications %s",
          arch_collapse_right_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_arch_collapse_right_read(struct bt_conn *conn,
                                            const struct bt_gatt_attr *attr,
                                            void *buf, uint16_t len,
                                            uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &arch_collapse_right_value,
                           sizeof(arch_collapse_right_value));
}

// --- Session Summary Individual Handlers ---
static void
ams_total_distance_m_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_total_distance_m_ccc_cfg_changed: attr is NULL");
    return;
  }
  total_distance_m_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Total Distance M notifications %s",
          total_distance_m_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_total_distance_m_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &total_distance_m_value,
                           sizeof(total_distance_m_value));
}

static void ams_avg_pace_sec_km_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                                uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_avg_pace_sec_km_ccc_cfg_changed: attr is NULL");
    return;
  }
  avg_pace_sec_km_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Avg Pace Sec/Km notifications %s",
          avg_pace_sec_km_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_avg_pace_sec_km_read(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        void *buf, uint16_t len,
                                        uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &avg_pace_sec_km_value,
                           sizeof(avg_pace_sec_km_value));
}

static void ams_avg_cadence_spm_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                                uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_avg_cadence_spm_ccc_cfg_changed: attr is NULL");
    return;
  }
  avg_cadence_spm_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Avg Cadence SPM notifications %s",
          avg_cadence_spm_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_avg_cadence_spm_read(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        void *buf, uint16_t len,
                                        uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &avg_cadence_spm_value,
                           sizeof(avg_cadence_spm_value));
}

static void ams_total_steps_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                            uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_total_steps_ccc_cfg_changed: attr is NULL");
    return;
  }
  total_steps_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Total Steps notifications %s",
          total_steps_notify_enabled ? "enabled" : "disabled");
}

// --- Stride Metrics Individual Handlers ---
static void
ams_stride_duration_ms_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_stride_duration_ms_ccc_cfg_changed: attr is NULL");
    return;
  }
  stride_duration_ms_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Stride Duration MS notifications %s",
          stride_duration_ms_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_stride_duration_ms_read(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &stride_duration_ms_value,
                           sizeof(stride_duration_ms_value));
}

static void
ams_stride_duration_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                         uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_stride_duration_asym_ccc_cfg_changed: attr is NULL");
    return;
  }
  stride_duration_asym_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Stride Duration Asymmetry notifications %s",
          stride_duration_asym_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_stride_duration_asym_read(struct bt_conn *conn,
                                             const struct bt_gatt_attr *attr,
                                             void *buf, uint16_t len,
                                             uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &stride_duration_asym_value,
                           sizeof(stride_duration_asym_value));
}

static void
ams_stride_length_cm_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_stride_length_cm_ccc_cfg_changed: attr is NULL");
    return;
  }
  stride_length_cm_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Stride Length CM notifications %s",
          stride_length_cm_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_stride_length_cm_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &stride_length_cm_value,
                           sizeof(stride_length_cm_value));
}

static void
ams_stride_length_asym_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_stride_length_asym_ccc_cfg_changed: attr is NULL");
    return;
  }
  stride_length_asym_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Stride Length Asymmetry notifications %s",
          stride_length_asym_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_stride_length_asym_read(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &stride_length_asym_value,
                           sizeof(stride_length_asym_value));
}

// --- Stride Length Average Handler ---
static void
ams_stride_length_avg_mm_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                         uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_stride_length_avg_mm_ccc_cfg_changed: attr is NULL");
    return;
  }
  stride_length_avg_mm_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Stride Length Average MM notifications %s",
          stride_length_avg_mm_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_stride_length_avg_mm_read(struct bt_conn *conn,
                                             const struct bt_gatt_attr *attr,
                                             void *buf, uint16_t len,
                                             uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &stride_length_avg_mm_value,
                           sizeof(stride_length_avg_mm_value));
}

// --- Run Speed Handler ---
static void
ams_run_speed_cms_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                  uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_run_speed_cms_ccc_cfg_changed: attr is NULL");
    return;
  }
  run_speed_cms_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Run Speed CMS notifications %s",
          run_speed_cms_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_run_speed_cms_read(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len,
                                      uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &run_speed_cms_value,
                           sizeof(run_speed_cms_value));
}

static ssize_t ams_total_steps_read(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr, void *buf,
                                    uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &total_steps_value,
                           sizeof(total_steps_value));
}

static void ams_calories_kcal_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                              uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_calories_kcal_ccc_cfg_changed: attr is NULL");
    return;
  }
  calories_kcal_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Calories Kcal notifications %s",
          calories_kcal_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_calories_kcal_read(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len,
                                      uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &calories_kcal_value,
                           sizeof(calories_kcal_value));
}

static void ams_avg_form_score_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                               uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_avg_form_score_ccc_cfg_changed: attr is NULL");
    return;
  }
  avg_form_score_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Avg Form Score notifications %s",
          avg_form_score_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_avg_form_score_read(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       void *buf, uint16_t len,
                                       uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &avg_form_score_value,
                           sizeof(avg_form_score_value));
}

static void ams_duration_sec_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                             uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_duration_sec_ccc_cfg_changed: attr is NULL");
    return;
  }
  duration_sec_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Duration Sec notifications %s",
          duration_sec_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_duration_sec_read(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr, void *buf,
                                     uint16_t len, uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &duration_sec_value,
                           sizeof(duration_sec_value));
}

// --- GPS Data Handler ---
static ssize_t ams_gps_data_write(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len,
                                  uint16_t offset, uint8_t flags) {
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
static void
ams_total_step_count_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                     uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_total_step_count_ccc_cfg_changed: attr is NULL");
    return;
  }

  total_step_count_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Total step count notifications %s",
          total_step_count_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_total_step_count_read(struct bt_conn *conn,
                                         const struct bt_gatt_attr *attr,
                                         void *buf, uint16_t len,
                                         uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &total_step_count_value,
                           sizeof(total_step_count_value));
}

// --- Activity Step Count Handlers ---
static void
ams_activity_step_count_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                        uint16_t value) {
  if (!attr) {
    LOG_ERR("ams_activity_step_count_ccc_cfg_changed: attr is NULL");
    return;
  }

  activity_step_count_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Activity step count notifications %s",
          activity_step_count_notify_enabled ? "enabled" : "disabled");
}

static ssize_t ams_activity_step_count_read(struct bt_conn *conn,
                                            const struct bt_gatt_attr *attr,
                                            void *buf, uint16_t len,
                                            uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &activity_step_count_value,
                           sizeof(activity_step_count_value));
}

// --- Public API Functions ---

/**
 * @brief Update real-time metrics from realtime_metrics module
 * Note: This should only be called from the bluetooth thread after receiving
 * a message via the bluetooth message queue
 *
 * @param metrics Pointer to metrics data
 */
void ams_update_realtime_metrics(const realtime_metrics_t *metrics) {
  
  /*  if (!metrics) {
      return;
    } */ //To implement it properly

#if IS_ENABLED(CONFIG_TEST_RANDOM_GENERATOR) // This is for testing only!!!!!!
  cadence_spm_value = (uint16_t)(sys_rand32_get() % 5000) / 100.0f +
                      20.0f; // Random 20.0 to 70.0
  contact_time_asym_value = (uint8_t)((sys_rand32_get() % 101));    // 0 to 100
  stride_duration_asym_value = (uint8_t)((sys_rand32_get() % 101)); // 0 to 100
  stride_length_asym_value = (uint8_t)((sys_rand32_get() % 101));   // 0 to 100
  ground_contact_ms_value = (uint16_t)(sys_rand32_get() % 1001);    // 0 to 1000
  flight_time_ms_value = (uint16_t)(sys_rand32_get() % 501);        // 0 to 500
  stride_duration_ms_value =
      (uint16_t)(sys_rand32_get() % 501); // 0 to 500  stride average
  pace_sec_km_value =
      (uint32_t)(180 + (sys_rand32_get() % 1321)); // 180 to 1500 (inclusive)
  distance_m_value =
      (uint32_t)(2000 + (sys_rand32_get() % 3001));             // 2000–5000m)
  form_score_value = (uint8_t)((sys_rand32_get() % 101));       // 0 to 100
  balance_lr_pct_value = (uint8_t)((sys_rand32_get() % 101));   // 0 to 100
  efficiency_score_value = (uint8_t)((sys_rand32_get() % 101)); // 0 to 100
  alerts_value = (uint8_t)((sys_rand32_get() % 101));           // 0 to 100
  
  // New metrics for testing
  stride_length_avg_mm_value = (uint16_t)(1200 + (sys_rand32_get() % 801)); // 1200-2000mm (1.2-2.0m)
  // Calculate run speed based on cadence and stride length
  // Formula: speed = cadence(spm) * stride_length(m) / 120
  // Convert to cm/s: multiply by 100
  uint32_t speed_calc = (cadence_spm_value * stride_length_avg_mm_value) / 1200; // Result in cm/s
  run_speed_cms_value = (uint16_t)(speed_calc > 65535 ? 65535 : speed_calc); // Cap at uint16_t max

  if (current_conn) {
    if (cadence_spm_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &cadence_spm_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &cadence_spm_value,
                               sizeof(cadence_spm_value));
      if (err)
        LOG_WRN("Failed to send Cadence SPM notification: %d", err);
    }

    if (contact_time_asym_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &contact_time_asym_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &contact_time_asym_value,
                         sizeof(contact_time_asym_value));
      if (err)
        LOG_WRN("Failed to send Contact Time Asymmetry notification: %d", err);
    }

    if (stride_duration_asym_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &stride_duration_asym_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &stride_duration_asym_value,
                         sizeof(stride_duration_asym_value));
      if (err)
        LOG_WRN("Failed to send Stride Duration Asymmetry notification: %d",
                err);
    }

    if (stride_length_asym_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &stride_length_asym_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &stride_length_asym_value,
                         sizeof(stride_length_asym_value));
      if (err)
        LOG_WRN("Failed to send Stride Length Asymmetry notification: %d", err);
    }

    if (ground_contact_ms_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &ground_contact_ms_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &ground_contact_ms_value,
                         sizeof(ground_contact_ms_value));
      if (err)
        LOG_WRN("Failed to send Ground Contact MS notification: %d", err);
    }

    if (flight_time_ms_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &flight_time_ms_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &flight_time_ms_value,
                               sizeof(flight_time_ms_value));
      if (err)
        LOG_WRN("Failed to send Flight Time MS notification: %d", err);
    }

    if (stride_duration_ms_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &stride_duration_ms_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &stride_duration_ms_value,
                         sizeof(stride_duration_ms_value));
      if (err)
        LOG_WRN("Failed to send Stride Duration MS notification: %d", err);
    }

    if (pace_sec_km_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &avg_pace_sec_km_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &pace_sec_km_value,
                               sizeof(pace_sec_km_value));
      if (err)
        LOG_WRN("Failed to send Pace Sec/Km notification: %d", err);
    }

    if (distance_m_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &total_distance_m_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &distance_m_value,
                               sizeof(distance_m_value));
      if (err)
        LOG_WRN("Failed to send Distance M notification: %d", err);
    }

    if (form_score_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &form_score_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &form_score_value,
                               sizeof(form_score_value));
      if (err)
        LOG_WRN("Failed to send Form Score notification: %d", err);
    }

    if (balance_lr_pct_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &balance_lr_pct_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &balance_lr_pct_value,
                               sizeof(balance_lr_pct_value));
      if (err)
        LOG_WRN("Failed to send Balance L/R Pct notification: %d", err);
    }

    if (efficiency_score_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &efficiency_score_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &efficiency_score_value,
                         sizeof(efficiency_score_value));
      if (err)
        LOG_WRN("Failed to send Efficiency Score notification: %d", err);
    }

    if (alerts_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &alerts_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &alerts_value,
                               sizeof(alerts_value));
      if (err)
        LOG_WRN("Failed to send Alerts notification: %d", err);
    }

    if (stride_length_avg_mm_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &stride_length_avg_mm_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &stride_length_avg_mm_value,
                               sizeof(stride_length_avg_mm_value));
      if (err)
        LOG_WRN("Failed to send Stride Length Average MM notification: %d", err);
    }

    if (run_speed_cms_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &run_speed_cms_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &run_speed_cms_value,
                               sizeof(run_speed_cms_value));
      if (err)
        LOG_WRN("Failed to send Run Speed CMS notification: %d", err);
    }
  }

  return;

#else

  // Update individual real-time metrics values
  cadence_spm_value = metrics->cadence_spm;
  pace_sec_km_value = metrics->pace_sec_km;
  distance_m_value = metrics->distance_m;
  form_score_value = metrics->form_score;
  balance_lr_pct_value = metrics->balance_lr_pct;
  ground_contact_ms_value = metrics->ground_contact_ms;
  flight_time_ms_value = metrics->flight_time_ms;
  efficiency_score_value = metrics->efficiency_score;
  alerts_value = metrics->alerts;

  // Send notifications for individual real-time metrics if enabled and
  // connected
  if (current_conn) {
    if (cadence_spm_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &cadence_spm_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &cadence_spm_value,
                               sizeof(cadence_spm_value));
      if (err)
        LOG_WRN("Failed to send Cadence SPM notification: %d", err);
    }
    if (pace_sec_km_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &avg_pace_sec_km_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &pace_sec_km_value,
                               sizeof(pace_sec_km_value));
      if (err)
        LOG_WRN("Failed to send Pace Sec/Km notification: %d", err);
    }
    if (distance_m_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &total_distance_m_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &distance_m_value,
                               sizeof(distance_m_value));
      if (err)
        LOG_WRN("Failed to send Distance M notification: %d", err);
    }
    if (form_score_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &form_score_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &form_score_value,
                               sizeof(form_score_value));
      if (err)
        LOG_WRN("Failed to send Form Score notification: %d", err);
    }
    if (balance_lr_pct_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &balance_lr_pct_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &balance_lr_pct_value,
                               sizeof(balance_lr_pct_value));
      if (err)
        LOG_WRN("Failed to send Balance L/R Pct notification: %d", err);
    }
    if (ground_contact_ms_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &ground_contact_ms_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &ground_contact_ms_value,
                         sizeof(ground_contact_ms_value));
      if (err)
        LOG_WRN("Failed to send Ground Contact MS notification: %d", err);
    }
    if (flight_time_ms_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &flight_time_ms_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &flight_time_ms_value,
                               sizeof(flight_time_ms_value));
      if (err)
        LOG_WRN("Failed to send Flight Time MS notification: %d", err);
    }
    if (efficiency_score_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &efficiency_score_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &efficiency_score_value,
                         sizeof(efficiency_score_value));
      if (err)
        LOG_WRN("Failed to send Efficiency Score notification: %d", err);
    }

    if (alerts_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &alerts_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &alerts_value,
                               sizeof(alerts_value));
      if (err)
        LOG_WRN("Failed to send Alerts notification: %d", err);
    }
  }

  // Update individual asymmetry metrics values
  contact_time_asym_value = metrics->contact_time_asymmetry;
  flight_time_asym_value = metrics->flight_time_asymmetry;
  force_asym_value = metrics->force_asymmetry;
  pronation_asym_value = metrics->pronation_asymmetry;
  strike_left_value = metrics->left_strike_pattern;
  strike_right_value = metrics->right_strike_pattern;

  // Send notifications for individual asymmetry metrics if enabled and
  // connected
  if (current_conn) {
    if (contact_time_asym_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &contact_time_asym_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &contact_time_asym_value,
                         sizeof(contact_time_asym_value));
      if (err)
        LOG_WRN("Failed to send Contact Time Asymmetry notification: %d", err);
    }
    if (flight_time_asym_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &flight_time_asym_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &flight_time_asym_value,
                         sizeof(flight_time_asym_value));
      if (err)
        LOG_WRN("Failed to send Flight Time Asymmetry notification: %d", err);
    }
    if (force_asym_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &force_asym_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &force_asym_value,
                               sizeof(force_asym_value));
      if (err)
        LOG_WRN("Failed to send Force Asymmetry notification: %d", err);
    }

    if (pronation_asym_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &pronation_asym_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &pronation_asym_value,
                               sizeof(pronation_asym_value));
      if (err)
        LOG_WRN("Failed to send Pronation Asymmetry notification: %d", err);
    }

    if (strike_left_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &strike_left_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &strike_left_value,
                               sizeof(strike_left_value));
      if (err)
        LOG_WRN("Failed to send Strike Left notification: %d", err);
    }
    if (strike_right_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &strike_right_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &strike_right_value,
                               sizeof(strike_right_value));
      if (err)
        LOG_WRN("Failed to send Strike Right notification: %d", err);
    }
  }

  // Update stride metrics values
  stride_duration_ms_value = metrics->stride_duration_ms;
  stride_duration_asym_value = metrics->stride_duration_asymmetry;
  stride_length_cm_value = metrics->stride_length_cm;
  stride_length_asym_value = metrics->stride_length_asymmetry;
  
  // Calculate stride length average in mm (mean of left and right)
  // Assuming stride_length_cm is already the average, convert to mm
  stride_length_avg_mm_value = metrics->stride_length_cm * 10;
  
  // Calculate run speed: speed = cadence(spm) * stride_length(m) / 120
  // Result in cm/s (m/s × 100)
  uint32_t speed_calc = (metrics->cadence_spm * stride_length_avg_mm_value) / 1200;
  run_speed_cms_value = (uint16_t)(speed_calc > 65535 ? 65535 : speed_calc);

  // Send notifications for stride metrics if enabled and connected
  if (current_conn) {
    if (stride_duration_ms_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &stride_duration_ms_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &stride_duration_ms_value,
                         sizeof(stride_duration_ms_value));
      if (err)
        LOG_WRN("Failed to send Stride Duration MS notification: %d", err);
    }

    if (stride_duration_asym_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &stride_duration_asym_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &stride_duration_asym_value,
                         sizeof(stride_duration_asym_value));
      if (err)
        LOG_WRN("Failed to send Stride Duration Asymmetry notification: %d",
                err);
    }

    if (stride_length_cm_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &stride_length_cm_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &stride_length_cm_value,
                         sizeof(stride_length_cm_value));
      if (err)
        LOG_WRN("Failed to send Stride Length CM notification: %d", err);
    }

    if (stride_length_asym_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &stride_length_asym_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &stride_length_asym_value,
                         sizeof(stride_length_asym_value));
      if (err)
        LOG_WRN("Failed to send Stride Length Asymmetry notification: %d", err);
    }

    if (stride_length_avg_mm_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &stride_length_avg_mm_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &stride_length_avg_mm_value,
                               sizeof(stride_length_avg_mm_value));
      if (err)
        LOG_WRN("Failed to send Stride Length Average MM notification: %d", err);
    }

    if (run_speed_cms_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &run_speed_cms_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &run_speed_cms_value,
                               sizeof(run_speed_cms_value));
      if (err)
        LOG_WRN("Failed to send Run Speed CMS notification: %d", err);
    }
  }
#endif
}

/**
 * @brief Update biomechanics extended data
 * Note: This should only be called from the bluetooth thread
 *
 * @param data Pointer to biomechanics data
 */
void ams_update_biomechanics_extended(const biomechanics_extended_ble_t *data) {
  if (!data) {
    return;
  }

  // Update individual biomechanics extended values
  pronation_left_value = data->pronation_left;
  pronation_right_value = data->pronation_right;
  loading_rate_left_value = data->loading_rate_left;
  loading_rate_right_value = data->loading_rate_right;
  arch_collapse_left_value = data->arch_collapse_left;
  arch_collapse_right_value = data->arch_collapse_right;

  // Send notifications for individual biomechanics extended metrics if enabled
  // and connected
  if (current_conn) {
    if (pronation_left_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &pronation_left_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &pronation_left_value,
                               sizeof(pronation_left_value));
      if (err)
        LOG_WRN("Failed to send Pronation Left notification: %d", err);
    }

    if (pronation_right_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &pronation_right_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &pronation_right_value,
                         sizeof(pronation_right_value));
      if (err)
        LOG_WRN("Failed to send Pronation Right notification: %d", err);
    }

    if (loading_rate_left_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &loading_rate_left_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &loading_rate_left_value,
                         sizeof(loading_rate_left_value));
      if (err)
        LOG_WRN("Failed to send Loading Rate Left notification: %d", err);
    }

    if (loading_rate_right_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &loading_rate_right_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &loading_rate_right_value,
                         sizeof(loading_rate_right_value));
      if (err)
        LOG_WRN("Failed to send Loading Rate Right notification: %d", err);
    }

    if (arch_collapse_left_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &arch_collapse_left_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &arch_collapse_left_value,
                         sizeof(arch_collapse_left_value));
      if (err)
        LOG_WRN("Failed to send Arch Collapse Left notification: %d", err);
    }

    if (arch_collapse_right_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &arch_collapse_right_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &arch_collapse_right_value,
                         sizeof(arch_collapse_right_value));
      if (err)
        LOG_WRN("Failed to send Arch Collapse Right notification: %d", err);
    }
  }
}

/**
 * @brief Update session summary (typically at end of activity)
 * Note: This should only be called from the bluetooth thread
 *
 * @param summary Pointer to session summary data
 */
void ams_update_session_summary(const session_summary_ble_t *summary) {
  if (!summary) {
    return;
  }

  // Update individual session summary values
  total_distance_m_value = summary->total_distance_m;
  avg_pace_sec_km_value = summary->avg_pace_sec_km;
  avg_cadence_spm_value = summary->avg_cadence_spm;
  total_steps_value = summary->total_steps;
  calories_kcal_value = summary->calories_kcal;
  avg_form_score_value = summary->avg_form_score;
  duration_sec_value = summary->duration_sec;

  // Send notifications for individual session summary metrics if enabled and
  // connected
  if (current_conn) {
    if (total_distance_m_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &total_distance_m_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &total_distance_m_value,
                         sizeof(total_distance_m_value));
      if (err)
        LOG_WRN("Failed to send Total Distance M notification: %d", err);
    }

    if (avg_pace_sec_km_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &avg_pace_sec_km_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &avg_pace_sec_km_value,
                         sizeof(avg_pace_sec_km_value));
      if (err)
        LOG_WRN("Failed to send Avg Pace Sec/Km notification: %d", err);
    }

    if (avg_cadence_spm_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &avg_cadence_spm_uuid.uuid);
      int err =
          bt_gatt_notify(current_conn, status_gatt, &avg_cadence_spm_value,
                         sizeof(avg_cadence_spm_value));
      if (err)
        LOG_WRN("Failed to send Avg Cadence SPM notification: %d", err);
    }

    if (total_steps_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &total_steps_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &total_steps_value,
                               sizeof(total_steps_value));
      if (err)
        LOG_WRN("Failed to send Total Steps notification: %d", err);
    }

    if (calories_kcal_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &calories_kcal_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &calories_kcal_value,
                               sizeof(calories_kcal_value));
      if (err)
        LOG_WRN("Failed to send Calories Kcal notification: %d", err);
    }

    if (avg_form_score_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &avg_form_score_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &avg_form_score_value,
                               sizeof(avg_form_score_value));
      if (err)
        LOG_WRN("Failed to send Avg Form Score notification: %d", err);
    }

    if (duration_sec_notify_enabled) {
      auto *status_gatt = bt_gatt_find_by_uuid(
          activity_metrics_service.attrs, activity_metrics_service.attr_count,
          &duration_sec_uuid.uuid);
      int err = bt_gatt_notify(current_conn, status_gatt, &duration_sec_value,
                               sizeof(duration_sec_value));
      if (err)
        LOG_WRN("Failed to send Duration Sec notification: %d", err);
    }
  }
}

/**
 * @brief Set the current BLE connection for notifications
 * Note: This should only be called from the bluetooth thread
 *
 * @param conn The BLE connection handle
 */
void ams_set_connection(struct bt_conn *conn) {
  current_conn = conn;

  if (!conn) {
    // Connection lost, reset notification states for all individual
    // characteristics
    cadence_spm_notify_enabled = false;
    pace_sec_km_notify_enabled = false;
    distance_m_notify_enabled = false;
    form_score_notify_enabled = false;
    balance_lr_pct_notify_enabled = false;
    ground_contact_ms_notify_enabled = false;
    flight_time_ms_notify_enabled = false;
    efficiency_score_notify_enabled = false;
    alerts_notify_enabled = false;

    contact_time_asym_notify_enabled = false;
    flight_time_asym_notify_enabled = false;
    force_asym_notify_enabled = false;
    pronation_asym_notify_enabled = false;
    strike_left_notify_enabled = false;
    strike_right_notify_enabled = false;

    pronation_left_notify_enabled = false;
    pronation_right_notify_enabled = false;
    loading_rate_left_notify_enabled = false;
    loading_rate_right_notify_enabled = false;
    arch_collapse_left_notify_enabled = false;
    arch_collapse_right_notify_enabled = false;

    total_distance_m_notify_enabled = false;
    avg_pace_sec_km_notify_enabled = false;
    avg_cadence_spm_notify_enabled = false;
    total_steps_notify_enabled = false;
    calories_kcal_notify_enabled = false;
    avg_form_score_notify_enabled = false;
    duration_sec_notify_enabled = false;

    total_step_count_notify_enabled = false;
    activity_step_count_notify_enabled = false;

    stride_duration_ms_notify_enabled = false;
    stride_duration_asym_notify_enabled = false;
    stride_length_cm_notify_enabled = false;
    stride_length_asym_notify_enabled = false;
    stride_length_avg_mm_notify_enabled = false;
    run_speed_cms_notify_enabled = false;
  }
}

/**
 * @brief Update total step count (aggregated from both feet)
 * Note: This should only be called from the bluetooth thread
 *
 * @param total_steps Total step count
 */
void ams_update_total_step_count(uint32_t total_steps) {
  total_step_count_value.step_count = total_steps;
  total_step_count_value.activity_duration_s = 0; // Deprecated - always 0

  // Send notification if enabled and connected
  if (total_step_count_notify_enabled && current_conn) {
    auto *status_gatt = bt_gatt_find_by_uuid(
        activity_metrics_service.attrs, activity_metrics_service.attr_count,
        &total_step_count_uuid.uuid);
    int err =
        bt_gatt_notify(current_conn,
                       status_gatt, // Attribute index for total step count
                       &total_step_count_value, sizeof(total_step_count_value));
    if (err) {
      LOG_WRN("Failed to send total step count notification: %d", err);
    }
  }
}

void ams_update_activity_step_count(uint32_t activity_steps) {
  activity_step_count_value.step_count = activity_steps;
  activity_step_count_value.activity_duration_s = 0; // Deprecated - always 0

  // Send notification if enabled and connected
  if (activity_step_count_notify_enabled && current_conn) {
    auto *status_gatt = bt_gatt_find_by_uuid(
        activity_metrics_service.attrs, activity_metrics_service.attr_count,
        &activity_step_count_uuid.uuid);
    int err = bt_gatt_notify(
        current_conn,
        status_gatt, // Attribute index for activity step count
        &activity_step_count_value, sizeof(activity_step_count_value));
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
bool ams_get_gps_data(gps_data_ble_t *data) {
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
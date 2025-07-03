/**
 * @file activity_metrics_service.h
 * @brief Activity Metrics Service API
 * @version 1.0
 * @date 2025-01
 *
 * @copyright Copyright (c) 2025 Botz Innovation
 */

#ifndef ACTIVITY_METRICS_SERVICE_H
#define ACTIVITY_METRICS_SERVICE_H

#include <zephyr/bluetooth/conn.h>
#include "../realtime_metrics/realtime_metrics.h"

// Forward declarations of BLE structures from the .cpp file
typedef struct __attribute__((packed)) {
    int8_t  pronation_left;
    int8_t  pronation_right;
    uint16_t loading_rate_left;
    uint16_t loading_rate_right;
    uint8_t arch_collapse_left;
    uint8_t arch_collapse_right;
    uint32_t reserved;
} biomechanics_extended_ble_t;

typedef struct __attribute__((packed)) {
    uint32_t total_distance_m;
    uint16_t avg_pace_sec_km;
    uint16_t avg_cadence_spm;
    uint32_t total_steps;
    uint16_t calories_kcal;
    uint8_t  avg_form_score;
    uint8_t  reserved;
    uint32_t duration_sec;
} session_summary_ble_t;

typedef struct __attribute__((packed)) {
    int32_t  latitude_e7;
    int32_t  longitude_e7;
    uint16_t distance_delta_m;
    uint8_t  accuracy_m;
    int16_t  elevation_change_m;
    uint8_t  gps_mode;
    uint16_t reserved;
} gps_data_ble_t;

// Internal API functions - only called from bluetooth thread
void ams_update_realtime_metrics(const realtime_metrics_t *metrics);
void ams_update_biomechanics_extended(const biomechanics_extended_ble_t *data);
void ams_update_session_summary(const session_summary_ble_t *summary);
void ams_set_connection(struct bt_conn *conn);
bool ams_get_gps_data(gps_data_ble_t *data);
void ams_update_total_step_count(uint32_t total_steps);
void ams_update_activity_step_count(uint32_t activity_steps);

#endif // ACTIVITY_METRICS_SERVICE_H
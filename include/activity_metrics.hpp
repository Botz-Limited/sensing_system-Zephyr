/**
 * @file activity_metrics.hpp
 * @brief Simple activity metrics calculation interface
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#ifndef APP_INCLUDE_ACTIVITY_METRICS_HEADER_
#define APP_INCLUDE_ACTIVITY_METRICS_HEADER_

#include <stdint.h>
#include <activity_session.hpp>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize activity metrics module

// Process pressure data from foot sensors
void activity_metrics_process_pressure(uint8_t foot, float pressure_values[8], uint32_t timestamp);

// Update BHI360 data
void activity_metrics_update_bhi360(float quat_w, float quat_x, float quat_y, float quat_z,
                                   float lacc_x, float lacc_y, float lacc_z,
                                   uint32_t step_count);

// Get current activity metrics for BLE transmission
void activity_metrics_get_current(RealtimeMetricsPacket* packet);

#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_ACTIVITY_METRICS_HEADER_
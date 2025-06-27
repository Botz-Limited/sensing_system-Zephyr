/**
 * @file ble_conn_params.hpp
 * @brief BLE connection parameter management for background execution support
 * 
 * This module allows the mobile app to request different connection profiles
 * to optimize power consumption during background execution.
 */

#ifndef BLE_CONN_PARAMS_HPP
#define BLE_CONN_PARAMS_HPP

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Connection profile types
 * 
 * The mobile app can request these profiles based on its execution state:
 * - FOREGROUND: App is active, user viewing data (15-30ms interval)
 * - BACKGROUND: App in background but active (50-100ms interval)  
 * - BACKGROUND_IDLE: App in background, minimal activity (200-500ms interval)
 */
typedef enum {
    CONN_PROFILE_FOREGROUND = 0,      // High performance, real-time data
    CONN_PROFILE_BACKGROUND = 1,      // Power saving, app in background
    CONN_PROFILE_BACKGROUND_IDLE = 2, // Maximum power saving
    CONN_PROFILE_MAX
} conn_profile_t;

/**
 * @brief Connection parameter configuration
 */
typedef struct {
    uint16_t interval_min;  // Minimum connection interval (1.25ms units)
    uint16_t interval_max;  // Maximum connection interval (1.25ms units)
    uint16_t latency;       // Slave latency (number of events)
    uint16_t timeout;       // Supervision timeout (10ms units)
    const char *name;       // Profile name for logging
} conn_param_config_t;

/**
 * @brief Data rate configuration per profile
 */
typedef struct {
    uint32_t foot_sensor_interval_ms;   // Foot sensor sampling interval
    uint32_t motion_sensor_interval_ms; // Motion sensor sampling interval
    bool aggregate_data;                // Enable data aggregation
    uint8_t aggregation_count;          // Number of samples to aggregate
} data_rate_config_t;

/**
 * @brief Initialize connection parameter manager
 * @return 0 on success, negative errno on failure
 */
int ble_conn_params_init(void);

/**
 * @brief Update connection parameters for a specific profile
 * @param conn BLE connection handle
 * @param profile Requested connection profile
 * @return 0 on success, negative errno on failure
 */
int ble_conn_params_update(struct bt_conn *conn, conn_profile_t profile);

/**
 * @brief Get current connection profile
 * @return Current active profile
 */
conn_profile_t ble_conn_params_get_profile(void);

/**
 * @brief Update sensor data rates based on connection profile
 * @param profile Connection profile to configure for
 */
void ble_conn_params_update_data_rates(conn_profile_t profile);

/**
 * @brief Check if data aggregation is enabled
 * @return true if aggregation is enabled for current profile
 */
bool ble_conn_params_is_aggregation_enabled(void);

/**
 * @brief Get aggregation count for current profile
 * @return Number of samples to aggregate before sending
 */
uint8_t ble_conn_params_get_aggregation_count(void);

/**
 * @brief Connection parameter update callback
 * @param conn Connection that was updated
 * @param interval New connection interval
 * @param latency New slave latency
 * @param timeout New supervision timeout
 */
void ble_conn_params_on_updated(struct bt_conn *conn, uint16_t interval,
                                uint16_t latency, uint16_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CONN_PARAMS_HPP */
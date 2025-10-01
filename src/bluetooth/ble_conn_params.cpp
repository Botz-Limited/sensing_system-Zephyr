/**
 * @file ble_conn_params.cpp
 * @brief BLE connection parameter management implementation
 */

#include "ble_conn_params.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(ble_conn_params, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// TODO: Replace these with actual sensor control functions when available
static void foot_sensor_set_sample_interval(uint32_t interval_ms) {
    LOG_INF("TODO: Set foot sensor interval to %d ms", interval_ms);
}

static void motion_sensor_set_sample_interval(uint32_t interval_ms) {
    LOG_INF("TODO: Set motion sensor interval to %d ms", interval_ms);
}

// Connection parameter configurations for each profile
static const conn_param_config_t conn_param_configs[CONN_PROFILE_MAX] = {
    [CONN_PROFILE_FOREGROUND] = {
        .interval_min = 12,   // 15ms (12 * 1.25ms)
        .interval_max = 24,   // 30ms (24 * 1.25ms)
        .latency = 0,         // No latency for real-time
        .timeout = 400,       // 4s supervision timeout
        .name = "FOREGROUND"
    },
    [CONN_PROFILE_BACKGROUND] = {
        .interval_min = 40,   // 50ms (40 * 1.25ms)
        .interval_max = 80,   // 100ms (80 * 1.25ms)
        .latency = 4,         // Allow 4 missed intervals
        .timeout = 600,       // 6s supervision timeout
        .name = "BACKGROUND"
    },
    [CONN_PROFILE_BACKGROUND_IDLE] = {
        .interval_min = 160,  // 200ms (160 * 1.25ms)
        .interval_max = 400,  // 500ms (400 * 1.25ms)
        .latency = 10,        // Allow 10 missed intervals
        .timeout = 1000,      // 10s supervision timeout
        .name = "BACKGROUND_IDLE"
    }
};

// Data rate configurations for each profile
static const data_rate_config_t data_rate_configs[CONN_PROFILE_MAX] = {
    [CONN_PROFILE_FOREGROUND] = {
        .foot_sensor_interval_ms = 15,
        .motion_sensor_interval_ms = 30,
        .aggregate_data = false,
        .aggregation_count = 1
    },
    [CONN_PROFILE_BACKGROUND] = {
        .foot_sensor_interval_ms = 50,
        .motion_sensor_interval_ms = 100,
        .aggregate_data = true,
        .aggregation_count = 3
    },
    [CONN_PROFILE_BACKGROUND_IDLE] = {
        .foot_sensor_interval_ms = 200,
        .motion_sensor_interval_ms = 500,
        .aggregate_data = true,
        .aggregation_count = 5
    }
};

// Current active profile
static conn_profile_t current_profile = CONN_PROFILE_FOREGROUND;
static struct bt_conn *current_conn = NULL;

// Mutex for thread safety
static K_MUTEX_DEFINE(conn_params_mutex);

int ble_conn_params_init(void)
{
    LOG_INF("BLE connection parameter manager initialized");
    current_profile = CONN_PROFILE_FOREGROUND;
    return 0;
}

int ble_conn_params_update(struct bt_conn *conn, conn_profile_t profile)
{

    if (!conn) {
        LOG_ERR("Invalid connection handle");
        return -EINVAL;
    }

    if (profile >= CONN_PROFILE_MAX) {
        LOG_ERR("Invalid profile: %d", profile);
        return -EINVAL;
    }

    k_mutex_lock(&conn_params_mutex, K_FOREVER);

    const conn_param_config_t *config = &conn_param_configs[profile];
    
    struct bt_le_conn_param params = {
        .interval_min = config->interval_min,
        .interval_max = config->interval_max,
        .latency = config->latency,
        .timeout = config->timeout
    };

    LOG_INF("Requesting connection parameter update to %s profile:", config->name);
    LOG_INF("  Interval: %.1f-%.1f ms (units: %d-%d)",
    config->interval_min * 1.25, config->interval_max * 1.25,
    config->interval_min, config->interval_max);
    LOG_INF("  Latency: %d, Timeout: %d ms", 
            config->latency, config->timeout * 10);

    int err = bt_conn_le_param_update(conn, &params);
    if (err) {
        LOG_ERR("Failed to request connection parameter update: %d", err);
        k_mutex_unlock(&conn_params_mutex);
        return err;
    }

    // Store the requested profile (will be confirmed in callback)
    current_profile = profile;
    current_conn = conn;

    // Update data rates immediately
    ble_conn_params_update_data_rates(profile);

    k_mutex_unlock(&conn_params_mutex);

    LOG_INF("Connection parameter update requested successfully");
    return 0;
}

conn_profile_t ble_conn_params_get_profile(void)
{
    conn_profile_t profile;
    
    k_mutex_lock(&conn_params_mutex, K_FOREVER);
    profile = current_profile;
    k_mutex_unlock(&conn_params_mutex);
    
    return profile;
}

void ble_conn_params_update_data_rates(conn_profile_t profile)
{
    if (profile >= CONN_PROFILE_MAX) {
        LOG_ERR("Invalid profile for data rate update: %d", profile);
        return;
    }

    const data_rate_config_t *config = &data_rate_configs[profile];
    
    LOG_INF("Updating sensor data rates for %s profile:", 
            conn_param_configs[profile].name);
    LOG_INF("  Foot sensor: %d ms", config->foot_sensor_interval_ms);
    LOG_INF("  Motion sensor: %d ms", config->motion_sensor_interval_ms);
    LOG_INF("  Data aggregation: %s (count: %d)", 
            config->aggregate_data ? "enabled" : "disabled",
            config->aggregation_count);

    // Update foot sensor sampling rate
    #if IS_ENABLED(CONFIG_FOOT_SENSOR)
    foot_sensor_set_sample_interval(config->foot_sensor_interval_ms);
    #endif

    // Update motion sensor sampling rate
    #if IS_ENABLED(CONFIG_MOTION_SENSOR)
    motion_sensor_set_sample_interval(config->motion_sensor_interval_ms);
    #endif
}

bool ble_conn_params_is_aggregation_enabled(void)
{
    bool enabled;
    
    k_mutex_lock(&conn_params_mutex, K_FOREVER);
    enabled = data_rate_configs[current_profile].aggregate_data;
    k_mutex_unlock(&conn_params_mutex);
    
    return enabled;
}

uint8_t ble_conn_params_get_aggregation_count(void)
{
    uint8_t count;
    
    k_mutex_lock(&conn_params_mutex, K_FOREVER);
    count = data_rate_configs[current_profile].aggregation_count;
    k_mutex_unlock(&conn_params_mutex);
    
    return count;
}

void ble_conn_params_on_updated(struct bt_conn *conn, uint16_t interval,
                                uint16_t latency, uint16_t timeout)
{
    ARG_UNUSED(conn);
    LOG_INF("Connection parameters updated:");
    LOG_INF("  Interval: %.2f ms (units: %d)", interval * 1.25, interval);
    LOG_INF("  Latency: %d", latency);
    LOG_INF("  Timeout: %d ms", timeout * 10);

    // Determine which profile these parameters match
    conn_profile_t detected_profile = CONN_PROFILE_FOREGROUND;
    
    if (interval >= 160) {  // >= 200ms
        detected_profile = CONN_PROFILE_BACKGROUND_IDLE;
    } else if (interval >= 40) {  // >= 50ms
        detected_profile = CONN_PROFILE_BACKGROUND;
    }

    k_mutex_lock(&conn_params_mutex, K_FOREVER);
    
    if (detected_profile != current_profile) {
        LOG_WRN("Detected profile %s differs from requested %s",
                conn_param_configs[detected_profile].name,
                conn_param_configs[current_profile].name);
        
        // Update to detected profile
        current_profile = detected_profile;
        ble_conn_params_update_data_rates(detected_profile);
    }
    
    k_mutex_unlock(&conn_params_mutex);
}
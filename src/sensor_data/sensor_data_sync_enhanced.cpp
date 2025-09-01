#include "sensor_data_sync_enhanced.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app.hpp>  

LOG_MODULE_REGISTER(sensor_sync_enhanced, CONFIG_SENSOR_DATA_MODULE_LOG_LEVEL);

// Sync timeout in milliseconds
#define SYNC_TIMEOUT_MS 50

// Maximum time difference for "good" sync quality
#define SYNC_EXCELLENT_DIFF_MS 5
#define SYNC_GOOD_DIFF_MS 10
#define SYNC_FAIR_DIFF_MS 20

// Enhanced sync state
static enhanced_sync_state_t sync_state;


// Calculate sync quality based on time difference
static uint8_t calculate_sync_quality(uint32_t time_diff_ms)
{
    if (time_diff_ms <= SYNC_EXCELLENT_DIFF_MS)
    {
        return 100; // Excellent sync
    }
    else if (time_diff_ms <= SYNC_GOOD_DIFF_MS)
    {
        return 80; // Good sync
    }
    else if (time_diff_ms <= SYNC_FAIR_DIFF_MS)
    {
        return 60; // Fair sync
    }
    else if (time_diff_ms <= SYNC_TIMEOUT_MS)
    {
        return 40; // Poor sync
    }
    else
    {
        return 0; // No sync
    }
}

// Try to create full synchronized data if all components available
static void try_full_sync(void)
{
    // Check if we have all required data
    if (!sync_state.primary_foot_pending || !sync_state.secondary_foot_pending) {
        return;  // Missing foot data
    }
    
    // Calculate time differences
    uint32_t foot_time_diff = (sync_state.primary_foot_time > sync_state.secondary_foot_time) ?
                              (sync_state.primary_foot_time - sync_state.secondary_foot_time) :
                              (sync_state.secondary_foot_time - sync_state.primary_foot_time);
    
    // Check if foot data is synchronized within threshold
    if (foot_time_diff > SYNC_TIMEOUT_MS) {
        // Data too old, clear the older one
        if (sync_state.primary_foot_time < sync_state.secondary_foot_time) {
            sync_state.primary_foot_pending = false;
        } else {
            sync_state.secondary_foot_pending = false;
        }
        sync_state.sync_timeouts++;
        return;
    }
    
    // Create full synchronized data
    full_synchronized_data_t full_sync;
    memset(&full_sync, 0, sizeof(full_sync));
    
    // Copy foot data
    memcpy(&full_sync.left_foot, &sync_state.secondary_foot, sizeof(foot_samples_t));
    memcpy(&full_sync.right_foot, &sync_state.primary_foot, sizeof(foot_samples_t));
    
    // Copy IMU data if available
    if (sync_state.primary_imu_pending) {
        memcpy(full_sync.right_imu.quaternion, sync_state.primary_imu.quaternion, sizeof(full_sync.right_imu.quaternion));
        memcpy(full_sync.right_imu.linear_acc, sync_state.primary_imu.linear_acc, sizeof(full_sync.right_imu.linear_acc));
        memcpy(full_sync.right_imu.gyro, sync_state.primary_imu.gyro, sizeof(full_sync.right_imu.gyro));
        full_sync.right_imu.step_count = sync_state.primary_imu.step_count;
        full_sync.right_imu.valid = true;
    }
    
    if (sync_state.secondary_imu_pending) {
        memcpy(full_sync.left_imu.quaternion, sync_state.secondary_imu.quaternion, sizeof(full_sync.left_imu.quaternion));
        memcpy(full_sync.left_imu.linear_acc, sync_state.secondary_imu.linear_acc, sizeof(full_sync.left_imu.linear_acc));
        memcpy(full_sync.left_imu.gyro, sync_state.secondary_imu.gyro, sizeof(full_sync.left_imu.gyro));
        full_sync.left_imu.step_count = sync_state.secondary_imu.step_count;
        full_sync.left_imu.valid = true;
    }
    
    // Set timing info
    full_sync.sync_time = MAX(sync_state.primary_foot_time, sync_state.secondary_foot_time);
    full_sync.sync_quality = calculate_sync_quality(foot_time_diff);
    full_sync.delta_time_ms = 10;  // Assuming 100Hz
    
    // Send directly to consumers - NO intermediate queue!
    extern struct k_msgq analytics_queue;
    extern struct k_msgq realtime_queue;
    extern struct k_msgq activity_metrics_msgq;
    
    // Create message
    generic_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.sender = SENDER_SENSOR_DATA;
    msg.type = MSG_TYPE_FULL_SYNC_DATA;
    memcpy(&msg.data.full_sync_data, &full_sync, sizeof(full_synchronized_data_t));
    
    // Push to all consumers
    if (k_msgq_put(&analytics_queue, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Analytics queue full, dropping full sync data");
    }
    if (k_msgq_put(&realtime_queue, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Realtime queue full, dropping full sync data");
    }
    if (k_msgq_put(&activity_metrics_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Activity metrics queue full, dropping full sync data");
    }
    
    // Log success based on what we synchronized
    if (full_sync.left_imu.valid && full_sync.right_imu.valid) {
        sync_state.full_sync_matched++;
        LOG_DBG("Full sync with bilateral IMU data sent (quality: %u%%)", full_sync.sync_quality);
    } else {
        sync_state.partial_sync_matched++;
        LOG_DBG("Partial sync - foot data only sent (quality: %u%%)", full_sync.sync_quality);
    }
    
    // Clear pending flags
    sync_state.primary_foot_pending = false;
    sync_state.secondary_foot_pending = false;
    
    // Clear IMU pending flags if they were used
    if (full_sync.right_imu.valid) {
        sync_state.primary_imu_pending = false;
    }
    if (full_sync.left_imu.valid) {
        sync_state.secondary_imu_pending = false;
    }
}

void sensor_data_sync_enhanced_init(void)
{
    memset(&sync_state, 0, sizeof(sync_state));
    LOG_INF("Enhanced synchronization initialized");
}

void sensor_data_sync_enhanced_process_foot(const foot_samples_t *data, bool is_secondary)
{
    if (!data)
        return;

    uint32_t current_time = k_uptime_get_32();

    if (is_secondary)
    {
        // Secondary foot data (left foot)
        memcpy(&sync_state.secondary_foot, data, sizeof(foot_samples_t));
        sync_state.secondary_foot_time = current_time;
        sync_state.secondary_foot_pending = true;
        LOG_DBG("Received secondary foot data at %u", current_time);
    }
    else
    {
        // Primary foot data (right foot)
        memcpy(&sync_state.primary_foot, data, sizeof(foot_samples_t));
        sync_state.primary_foot_time = current_time;
        sync_state.primary_foot_pending = true;
        LOG_DBG("Received primary foot data at %u", current_time);
    }

    // Try to create full synchronized data
    try_full_sync();
}

void sensor_data_sync_enhanced_process_imu(const bhi360_log_record_t *data, bool is_secondary)
{
    if (!data)
        return;

    uint32_t current_time = k_uptime_get_32();

    if (is_secondary)
    {
        // Secondary IMU data
        sync_state.secondary_imu.quaternion[0] = data->quat_w;
        sync_state.secondary_imu.quaternion[1] = data->quat_x;
        sync_state.secondary_imu.quaternion[2] = data->quat_y;
        sync_state.secondary_imu.quaternion[3] = data->quat_z;
        sync_state.secondary_imu.linear_acc[0] = data->lacc_x;
        sync_state.secondary_imu.linear_acc[1] = data->lacc_y;
        sync_state.secondary_imu.linear_acc[2] = data->lacc_z;
        sync_state.secondary_imu.gyro[0] = data->gyro_x;
        sync_state.secondary_imu.gyro[1] = data->gyro_y;
        sync_state.secondary_imu.gyro[2] = data->gyro_z;
        sync_state.secondary_imu.step_count = data->step_count;
        sync_state.secondary_imu_time = current_time;
        sync_state.secondary_imu_pending = true;
        LOG_DBG("Received secondary IMU data at %u", current_time);
    }
    else
    {
        // Primary IMU data
        sync_state.primary_imu.quaternion[0] = data->quat_w;
        sync_state.primary_imu.quaternion[1] = data->quat_x;
        sync_state.primary_imu.quaternion[2] = data->quat_y;
        sync_state.primary_imu.quaternion[3] = data->quat_z;
        sync_state.primary_imu.linear_acc[0] = data->lacc_x;
        sync_state.primary_imu.linear_acc[1] = data->lacc_y;
        sync_state.primary_imu.linear_acc[2] = data->lacc_z;
        sync_state.primary_imu.gyro[0] = data->gyro_x;
        sync_state.primary_imu.gyro[1] = data->gyro_y;
        sync_state.primary_imu.gyro[2] = data->gyro_z;
        sync_state.primary_imu.step_count = data->step_count;
        sync_state.primary_imu_time = current_time;
        sync_state.primary_imu_pending = true;
        LOG_DBG("Received primary IMU data at %u", current_time);
    }

    // Try to create full synchronized data
    try_full_sync();
}

void sensor_data_sync_enhanced_get_stats(uint32_t *full_sync, uint32_t *partial_sync, uint32_t *timeouts)
{
    if (full_sync)
        *full_sync = sync_state.full_sync_matched;
    if (partial_sync)
        *partial_sync = sync_state.partial_sync_matched;
    if (timeouts)
        *timeouts = sync_state.sync_timeouts;
}
#ifndef SENSOR_DATA_SYNC_ENHANCED_H
#define SENSOR_DATA_SYNC_ENHANCED_H

#include <stdint.h>
#include <stdbool.h>
#include <app.hpp>

#ifdef __cplusplus
extern "C" {
#endif

// Enhanced sync state with IMU data
typedef struct {
    // Foot data buffers
    foot_samples_t primary_foot;
    foot_samples_t secondary_foot;
    bool primary_foot_pending;
    bool secondary_foot_pending;
    uint32_t primary_foot_time;
    uint32_t secondary_foot_time;
    
    // IMU data buffers
    struct {
        float quaternion[4];
        float linear_acc[3];
        float gyro[3];
        uint32_t step_count;
    } primary_imu;
    struct {
        float quaternion[4];
        float linear_acc[3];
        float gyro[3];
        uint32_t step_count;
    } secondary_imu;
    bool primary_imu_pending;
    bool secondary_imu_pending;
    uint32_t primary_imu_time;
    uint32_t secondary_imu_time;
    
    // Statistics
    uint32_t full_sync_matched;
    uint32_t partial_sync_matched;
    uint32_t sync_timeouts;
} enhanced_sync_state_t;

// Initialize enhanced synchronization
void sensor_data_sync_enhanced_init(void);

// Process foot data with enhanced sync
void sensor_data_sync_enhanced_process_foot(const foot_samples_t *data, bool is_secondary);

// Process IMU data
void sensor_data_sync_enhanced_process_imu(const bhi360_log_record_t *data, bool is_secondary);

// Get enhanced statistics
void sensor_data_sync_enhanced_get_stats(uint32_t *full_sync, uint32_t *partial_sync, uint32_t *timeouts);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_DATA_SYNC_ENHANCED_H
/**
 * @file ble_d2d_quaternion.cpp
 * @brief D2D quaternion transmission for secondary device
 * @version 1.0
 * @date 2024-12-20
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app.hpp>
#include "ble_d2d_tx.hpp"
#include "orientation_3d_service.hpp"

LOG_MODULE_DECLARE(bluetooth, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)

// Rate limiting for D2D quaternion transmission
static uint32_t last_d2d_quat_time = 0;
static const uint32_t D2D_QUAT_INTERVAL_MS = 50; // 20Hz

/**
 * @brief Send quaternion data from secondary to primary via D2D
 * 
 * This is called from motion sensor at 50Hz, we downsample to 20Hz
 */
int ble_d2d_tx_send_quaternion_3d(float quat_w, float quat_x, float quat_y, float quat_z)
{
    uint32_t current_time = k_uptime_get_32();
    
    // Rate limit to 20Hz
    if (current_time - last_d2d_quat_time < D2D_QUAT_INTERVAL_MS) {
        return 0; // Skip this update
    }
    
    // Create quaternion message
    quaternion_3d_t quat_data = {
        .quat_w = quat_w,
        .quat_x = quat_x,
        .quat_y = quat_y,
        .quat_z = quat_z,
        .timestamp_ms = current_time
    };
    
    // Send via existing D2D infrastructure
    // We can reuse the bhi360_data1 channel since it has similar data
    bhi360_3d_mapping_t mapping_data = {
        .accel_x = quat_x,
        .accel_y = quat_y,
        .accel_z = quat_z,
        .gyro_x = 0.0f,  // Not used for 3D viz
        .gyro_y = 0.0f,
        .gyro_z = 0.0f,
        .quat_w = quat_w
    };
    
    int ret = ble_d2d_tx_send_bhi360_data1(&mapping_data);
    if (ret == 0) {
        last_d2d_quat_time = current_time;
    }
    
    return ret;
}

#endif // !CONFIG_PRIMARY_DEVICE

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)

/**
 * @brief Handle received quaternion data from secondary device
 * 
 * This is called when primary receives D2D quaternion data
 */
void d2d_handle_quaternion_3d(const quaternion_3d_t* quat_data)
{
    if (!quat_data) {
        return;
    }
    
    // Forward to 3D orientation service
    orientation_3d_update_secondary(quat_data->quat_w, 
                                   quat_data->quat_x, 
                                   quat_data->quat_y, 
                                   quat_data->quat_z);
}

#endif // CONFIG_PRIMARY_DEVICE
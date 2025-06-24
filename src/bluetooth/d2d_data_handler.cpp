/**
 * @file d2d_data_handler.cpp
 * @brief Implementation of D2D data handler for primary device
 */

#include "d2d_data_handler.hpp"
#include <zephyr/logging/log.h>
#include <ble_services.hpp>
#include <app.hpp>
#include "orientation_3d_service.hpp"

LOG_MODULE_REGISTER(d2d_data_handler, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// TODO: This is a placeholder implementation
// The actual implementation should forward data to the Information Service
// and potentially aggregate data from both feet

int d2d_data_handler_init(void)
{
    LOG_INF("D2D data handler initialized");
    return 0;
}

int d2d_data_handler_process_foot_samples(const foot_samples_t *samples)
{
    if (!samples) {
        return -EINVAL;
    }
    
    LOG_DBG("Processing foot samples from secondary device");
    
    // TODO: Implement actual processing
    // 1. Check if we should aggregate with primary foot data
    // 2. Forward to Information Service for phone notification
    // 3. Log to file system if logging is active
    
    // For now, just log the first few values
    LOG_INF("Secondary foot samples: [%u, %u, %u, ...]", 
            samples->values[0], samples->values[1], samples->values[2]);
    
    return 0;
}

int d2d_data_handler_process_bhi360_3d_mapping(const bhi360_3d_mapping_t *data)
{
    if (!data) {
        return -EINVAL;
    }
    
    LOG_DBG("Processing BHI360 3D mapping from secondary device");
    
    if (data->gyro_x == 0.0f && data->gyro_y == 0.0f && data->gyro_z == 0.0f) {
        // This is quaternion-only data for 3D visualization
        // Forward to 3D orientation service (secondary is left shoe)
        orientation_3d_update_secondary(data->quat_w, 
                                       data->accel_x,  // Actually quat_x
                                       data->accel_y,  // Actually quat_y
                                       data->accel_z); // Actually quat_z
        LOG_DBG("Updated 3D orientation from secondary: w=%.3f, x=%.3f, y=%.3f, z=%.3f",
                (double)data->quat_w, (double)data->accel_x, 
                (double)data->accel_y, (double)data->accel_z);
    } else {
        // This is full sensor data for logging
        // TODO: Forward to Information Service for regular data logging
        LOG_DBG("Full sensor data from secondary (not just quaternion)");
    }
    
    return 0;
}

int d2d_data_handler_process_bhi360_step_count(const bhi360_step_count_t *data)
{
    if (!data) {
        return -EINVAL;
    }
    
    LOG_DBG("Processing BHI360 step count from secondary: %u steps", 
            data->step_count);
    
    // TODO: Aggregate with primary step count
    // Total steps = primary + secondary
    
    return 0;
}

int d2d_data_handler_process_bhi360_linear_accel(const bhi360_linear_accel_t *data)
{
    if (!data) {
        return -EINVAL;
    }
    
    LOG_DBG("Processing BHI360 linear accel from secondary device");
    
    // TODO: Forward to Information Service
    
    return 0;
}

int d2d_data_handler_process_file_path(uint8_t log_id, uint8_t file_type, const char *path)
{
    if (!path) {
        return -EINVAL;
    }
    
    LOG_INF("Secondary device file path - ID: %u, Type: %u, Path: %s", 
            log_id, file_type, path);
    
    // TODO: Update Information Service with secondary device file paths
    // This would require adding secondary file path characteristics
    // to the Information Service
    
    return 0;
}

int d2d_data_handler_process_log_available(uint8_t log_id, uint8_t file_type)
{
    LOG_INF("Secondary device log available - ID: %u, Type: %u", 
            log_id, file_type);
    
    // TODO: Update Information Service to notify phone
    // The phone should then use the File Proxy to retrieve the file
    
    return 0;
}

int d2d_data_handler_process_status(uint32_t status)
{
    LOG_DBG("Secondary device status: 0x%08x", status);
    
    // TODO: Merge with primary status
    // Combined status = primary_status | (secondary_status << 16)
    // Or use separate characteristic for secondary status
    
    return 0;
}

int d2d_data_handler_process_charge_status(uint8_t charge_status)
{
    LOG_INF("Secondary device charge status: %u%%", charge_status);

    // Forward to Information Service to notify phone
    // Note: This updates the primary's charge status characteristic with secondary's value
    // In a real implementation, you might want to have separate characteristics for each device
    jis_charge_status_notify(charge_status);

    return 0;
}
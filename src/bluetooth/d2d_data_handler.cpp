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
    
    LOG_INF("Received foot sensor data from secondary");
    LOG_DBG("Processing foot samples from secondary device");
    
    // Forward to Information Service for phone notification
    // The jis_foot_sensor_notify function will add sequence numbers
    jis_foot_sensor_notify(samples);
    
    LOG_INF("Secondary foot samples forwarded to phone: [%u, %u, %u, ...]", 
            samples->values[0], samples->values[1], samples->values[2]);
    
    // TODO: Log to file system if logging is active
    
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
    }
    
    // Always forward to Information Service for phone notification
    // The jis_bhi360_data1_notify function will add sequence numbers
    jis_bhi360_data1_notify(data);
    LOG_DBG("Secondary BHI360 3D mapping data forwarded to phone");
    
    return 0;
}

int d2d_data_handler_process_bhi360_step_count(const bhi360_step_count_t *data)
{
    if (!data) {
        return -EINVAL;
    }
    
    LOG_DBG("Processing BHI360 step count from secondary: %u steps", 
            data->step_count);
    
    // Don't forward individual step counts to phone anymore
    // The bluetooth module will handle aggregation when it receives
    // step counts from both primary (via message queue) and secondary
    // jis_bhi360_data2_notify(data);
    LOG_DBG("Secondary BHI360 step count received for aggregation");
    
    // The bluetooth module will handle aggregation when it receives
    // step counts from both primary (via message queue) and secondary (via this notification)
    
    return 0;
}

int d2d_data_handler_process_bhi360_linear_accel(const bhi360_linear_accel_t *data)
{
    if (!data) {
        return -EINVAL;
    }
    
    LOG_DBG("Processing BHI360 linear accel from secondary device");
    
    // Forward to Information Service for phone notification
    // The jis_bhi360_data3_notify function will add sequence numbers
    jis_bhi360_data3_notify(data);
    LOG_DBG("Secondary BHI360 linear accel data forwarded to phone");
    
    return 0;
}

int d2d_data_handler_process_file_path(uint8_t log_id, uint8_t file_type, const char *path)
{
    if (!path) {
        return -EINVAL;
    }
    
    LOG_INF("Secondary device file path - ID: %u, Type: %u, Path: %s", 
            log_id, file_type, path);
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Forward declarations
    extern void jis_secondary_foot_log_path_notify(const char* path);
    extern void jis_secondary_bhi360_log_path_notify(const char* path);
    extern void jis_secondary_activity_log_path_notify(const char* path);
    
    // Forward to Information Service based on file type
    switch (file_type) {
        case 0: // Foot sensor log
            jis_secondary_foot_log_path_notify(path);
            break;
        case 1: // BHI360 log
            jis_secondary_bhi360_log_path_notify(path);
            break;
        case 2: // Activity log
            jis_secondary_activity_log_path_notify(path);
            break;
        default:
            LOG_WRN("Unknown file type %u", file_type);
            break;
    }
#endif
    
    return 0;
}

int d2d_data_handler_process_log_available(uint8_t log_id, uint8_t file_type)
{
    LOG_INF("Secondary device log available - ID: %u, Type: %u", 
            log_id, file_type);
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Forward declarations
    extern void jis_secondary_foot_log_available_notify(uint8_t log_id);
    extern void jis_secondary_bhi360_log_available_notify(uint8_t log_id);
    extern void jis_secondary_activity_log_available_notify(uint8_t log_id);
    
    // Forward to Information Service based on file type
    switch (file_type) {
        case 0: // Foot sensor log
            jis_secondary_foot_log_available_notify(log_id);
            break;
        case 1: // BHI360 log
            jis_secondary_bhi360_log_available_notify(log_id);
            break;
        case 2: // Activity log
            jis_secondary_activity_log_available_notify(log_id);
            break;
        default:
            LOG_WRN("Unknown file type %u", file_type);
            break;
    }
#endif
    
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

int d2d_data_handler_process_activity_step_count(const bhi360_step_count_t *data)
{
    if (!data) {
        return -EINVAL;
    }
    
    LOG_DBG("Processing activity step count from secondary: %u steps", 
            data->step_count);
    
    // Send to bluetooth module for aggregation
    // Use a different sender to indicate this is activity steps from secondary
    generic_message_t msg = {};
    msg.sender = SENDER_D2D_SECONDARY;
    msg.type = MSG_TYPE_ACTIVITY_STEP_COUNT;  // Use activity step count message type
    msg.data.bhi360_step_count = *data;
    
    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) == 0) {
        LOG_DBG("Sent secondary activity step count to bluetooth module");
    } else {
        LOG_ERR("Failed to send secondary activity step count to bluetooth module");
    }
    
    return 0;
}
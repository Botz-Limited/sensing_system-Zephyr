/**
 * @file d2d_data_handler.cpp
 * @brief Implementation of D2D data handler for primary device
 */

#include "d2d_data_handler.hpp"
#include <zephyr/logging/log.h>
#include <ble_services.hpp>
#include <app.hpp>

LOG_MODULE_REGISTER(d2d_data_handler, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Message queues defined in app.cpp
extern struct k_msgq sensor_data_msgq;
extern struct k_msgq activity_metrics_msgq;
extern struct k_msgq bluetooth_msgq;

int d2d_data_handler_init(void)
{
    LOG_INF("D2D data handler initialized");
    return 0;
}

// Handle D2D batch data from secondary device
int d2d_data_handler_process_batch(const d2d_sample_batch_t *batch)
{
    if (!batch || batch->count == 0) {
        LOG_WRN("Received empty or null D2D batch");
        return -EINVAL;
    }

    LOG_INF("Processing D2D batch: count=%u, timestamp[0]=%u", batch->count, batch->timestamp[0]);

    //To implement

    return 0;
}

int d2d_data_handler_process_foot_samples(const foot_samples_t *samples)
{
    if (!samples) {
        return -EINVAL;
    }
    
    LOG_INF("Received foot sensor data from secondary");
    
    // DO NOT forward directly to phone via jis_foot_sensor_notify()
    // That function is for PRIMARY device's foot data only!
    // Instead, forward to bluetooth module which will handle it properly
    
    // Send to bluetooth module for proper handling
    generic_message_t ble_msg;
    ble_msg.sender = SENDER_D2D_SECONDARY;
    ble_msg.type = MSG_TYPE_FOOT_SAMPLES;
    memcpy(&ble_msg.data.foot_samples, samples, sizeof(foot_samples_t));
    
    if (k_msgq_put(&bluetooth_msgq, &ble_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary foot data to bluetooth module");
    } else {
        LOG_DBG("Secondary foot data forwarded to bluetooth_msgq for proper routing");
    }
    
    // Also forward to sensor_data_msgq for consolidation
    generic_message_t sensor_msg;
    sensor_msg.sender = SENDER_D2D_SECONDARY;
    sensor_msg.type = MSG_TYPE_FOOT_SAMPLES;
    memcpy(&sensor_msg.data.foot_samples, samples, sizeof(foot_samples_t));
    
    if (k_msgq_put(&sensor_data_msgq, &sensor_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary foot data to sensor data module");
    } else {
        LOG_DBG("Secondary foot data forwarded to sensor_data_msgq");
    }
    
    return 0;
}

int d2d_data_handler_process_bhi360_3d_mapping(const bhi360_3d_mapping_t *data)
{
    if (!data) {
        return -EINVAL;
    }
    
    LOG_DBG("Processing BHI360 3D mapping from secondary device");
    
    // DO NOT forward directly to phone via jis_bhi360_data1_notify()
    // That function is for PRIMARY device's BHI360 data only!
    // Instead, forward to bluetooth module which will handle it properly
    
    // Send to bluetooth module for proper handling
    generic_message_t ble_msg;
    ble_msg.sender = SENDER_D2D_SECONDARY;
    ble_msg.type = MSG_TYPE_BHI360_3D_MAPPING;
    memcpy(&ble_msg.data.bhi360_3d_mapping, data, sizeof(bhi360_3d_mapping_t));
    
    if (k_msgq_put(&bluetooth_msgq, &ble_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary BHI360 3D data to bluetooth module");
    } else {
        LOG_DBG("Secondary BHI360 3D data forwarded to bluetooth_msgq for proper routing");
    }
    
    // Also forward to sensor_data_msgq as BHI360_LOG_RECORD
    generic_message_t sensor_msg;
    sensor_msg.sender = SENDER_D2D_SECONDARY;
    sensor_msg.type = MSG_TYPE_BHI360_LOG_RECORD;
    
    // Convert 3D mapping to log record format
    // Note: In 3D mapping, accel_x/y/z contain quaternion x/y/z
    sensor_msg.data.bhi360_log_record.quat_x = data->accel_x;
    sensor_msg.data.bhi360_log_record.quat_y = data->accel_y;
    sensor_msg.data.bhi360_log_record.quat_z = data->accel_z;
    sensor_msg.data.bhi360_log_record.quat_w = data->quat_w;
    sensor_msg.data.bhi360_log_record.gyro_x = data->gyro_x;
    sensor_msg.data.bhi360_log_record.gyro_y = data->gyro_y;
    sensor_msg.data.bhi360_log_record.gyro_z = data->gyro_z;
    // Linear acceleration not available in 3D mapping
    sensor_msg.data.bhi360_log_record.lacc_x = 0;
    sensor_msg.data.bhi360_log_record.lacc_y = 0;
    sensor_msg.data.bhi360_log_record.lacc_z = 0;
    
    if (k_msgq_put(&sensor_data_msgq, &sensor_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary BHI360 3D data to sensor data module");
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
    
    // Forward to activity metrics for aggregation
    generic_message_t msg;
    msg.sender = SENDER_D2D_SECONDARY;
    msg.type = MSG_TYPE_BHI360_STEP_COUNT;
    memcpy(&msg.data.bhi360_step_count, data, sizeof(bhi360_step_count_t));
    
    if (k_msgq_put(&activity_metrics_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary step count to activity metrics module");
    } else {
        LOG_DBG("Secondary step count forwarded to activity_metrics_msgq");
    }
    
    return 0;
}

int d2d_data_handler_process_bhi360_linear_accel(const bhi360_linear_accel_t *data)
{
    if (!data) {
        return -EINVAL;
    }
    
    LOG_DBG("Processing BHI360 linear accel from secondary device");
    
    // DO NOT forward directly to phone via jis_bhi360_data3_notify()
    // That function is for PRIMARY device's BHI360 data only!
    
    // Send to bluetooth module for proper handling
    generic_message_t ble_msg;
    ble_msg.sender = SENDER_D2D_SECONDARY;
    ble_msg.type = MSG_TYPE_BHI360_LINEAR_ACCEL;
    memcpy(&ble_msg.data.bhi360_linear_accel, data, sizeof(bhi360_linear_accel_t));
    
    if (k_msgq_put(&bluetooth_msgq, &ble_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary BHI360 linear accel to bluetooth module");
    } else {
        LOG_DBG("Secondary BHI360 linear accel forwarded to bluetooth_msgq for proper routing");
    }
    
    // Also forward to sensor_data_msgq
    generic_message_t sensor_msg;
    sensor_msg.sender = SENDER_D2D_SECONDARY;
    sensor_msg.type = MSG_TYPE_BHI360_LINEAR_ACCEL;
    memcpy(&sensor_msg.data.bhi360_linear_accel, data, sizeof(bhi360_linear_accel_t));
    
    if (k_msgq_put(&sensor_data_msgq, &sensor_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary linear accel to sensor data module");
    }
    
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
    
    // Forward to Information Service based on file type
    switch (file_type) {
        case 0: // Foot sensor log
            jis_secondary_foot_log_path_notify(path);
            break;
        case 1: // BHI360 log
            jis_secondary_bhi360_log_path_notify(path);
            break;
        case 2: // Activity log
            // NOTE: Not forwarding activity logs - each device maintains its own
            LOG_DBG("Secondary activity log path received but not forwarded (separate logs)");
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
    
    // Forward to Information Service based on file type
    switch (file_type) {
        case 0: // Foot sensor log
            jis_secondary_foot_log_available_notify(log_id);
            break;
        case 1: // BHI360 log
            jis_secondary_bhi360_log_available_notify(log_id);
            break;
        case 2: // Activity log
            // NOTE: Not forwarding activity logs - each device maintains its own
            LOG_DBG("Secondary activity log available but not forwarded (separate logs)");
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
    
    // Forward to bluetooth module for aggregation
    generic_message_t msg;
    msg.sender = SENDER_D2D_SECONDARY;
    msg.type = MSG_TYPE_ACTIVITY_STEP_COUNT;  // Use activity-specific type
    memcpy(&msg.data.bhi360_step_count, data, sizeof(bhi360_step_count_t));
    
    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary activity step count to bluetooth module");
    } else {
        LOG_DBG("Secondary activity step count forwarded to bluetooth_msgq");
    }
    
    return 0;
}
/**
 * @file d2d_data_handler.cpp
 * @brief Implementation of D2D data handler for primary device
 */

#include "d2d_data_handler.hpp"
#include <zephyr/logging/log.h>
#include <ble_services.hpp>
#include <app.hpp>
#include "../data_dispatcher/data_dispatcher.h"
#include <d2d_metrics.h>

LOG_MODULE_REGISTER(d2d_data_handler, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Message queues defined in app.cpp
extern struct k_msgq sensor_data_msgq;
extern struct k_msgq activity_metrics_msgq;
extern struct k_msgq bluetooth_msgq;

// Global storage for secondary data (for aggregation with primary)
foot_samples_t g_secondary_foot_data = {0};
d2d_quaternion_fixed_t g_secondary_quat_data = {0};
uint32_t g_secondary_last_timestamp = 0;
bool g_secondary_data_valid = false;

int d2d_data_handler_init(void)
{
    LOG_INF("D2D data handler initialized");
    return 0;
}

// Handle D2D batch data from secondary device (simplified: quaternion only)
int d2d_data_handler_process_batch(const d2d_sample_batch_t *batch)
{
    if (!batch) {
        LOG_WRN("Received null D2D batch");
        return -EINVAL;
    }

    LOG_INF("Processing D2D batch (quaternion-only): timestamp[0]=%u", batch->timestamp[0]);

    // Store secondary data for aggregation (D2D_BATCH_SIZE = 1 currently)
    if (D2D_BATCH_SIZE > 0) {
        // Store foot sample (direct copy, no conversion needed)
        memcpy(&g_secondary_foot_data, &batch->foot[0], sizeof(foot_samples_t));
        
        // Store quaternion data (already in fixed-point format)
        memcpy(&g_secondary_quat_data, &batch->quat[0], sizeof(d2d_quaternion_fixed_t));
        
        // Update timestamp and validity
        g_secondary_last_timestamp = batch->timestamp[0];
        g_secondary_data_valid = true;
        
        LOG_DBG("Secondary data stored for aggregation: foot + quaternion");
        LOG_DBG("Foot values[0-3]: %u %u %u %u",
                g_secondary_foot_data.values[0], g_secondary_foot_data.values[1],
                g_secondary_foot_data.values[2], g_secondary_foot_data.values[3]);
        LOG_DBG("Quaternion (fixed): x=%d y=%d z=%d w=%d",
                g_secondary_quat_data.quat_x, g_secondary_quat_data.quat_y,
                g_secondary_quat_data.quat_z, g_secondary_quat_data.quat_w);
        
        // Still forward to central hub for logging/analysis
        extern struct k_msgq central_data_hub_msgq;
        
        // Foot data to central hub
        tagged_message_t foot_msg;
        foot_msg.msg.sender = SENDER_D2D_SECONDARY;
        foot_msg.msg.type = MSG_TYPE_FOOT_SAMPLES;
        foot_msg.tag = SECONDARY_FOOT;
        memcpy(&foot_msg.msg.data.foot_samples, &batch->foot[0], sizeof(foot_samples_t));
        if (k_msgq_put(&central_data_hub_msgq, &foot_msg, K_NO_WAIT) != 0) {
            LOG_ERR("Failed to queue foot sample to central hub");
        } else {
            LOG_DBG("Foot sample queued to central hub with tag SECONDARY_FOOT");
        }

        // Convert quaternion for central hub (needs float format)
        bhi360_log_record_t imu;
        imu.quat_x = fixed16_to_float(batch->quat[0].quat_x, FixedPoint::QUAT_SCALE);
        imu.quat_y = fixed16_to_float(batch->quat[0].quat_y, FixedPoint::QUAT_SCALE);
        imu.quat_z = fixed16_to_float(batch->quat[0].quat_z, FixedPoint::QUAT_SCALE);
        imu.quat_w = fixed16_to_float(batch->quat[0].quat_w, FixedPoint::QUAT_SCALE);
        imu.gyro_x = 0.0f;
        imu.gyro_y = 0.0f;
        imu.gyro_z = 0.0f;
        imu.lacc_x = 0.0f;
        imu.lacc_y = 0.0f;
        imu.lacc_z = 0.0f;
        imu.quat_accuracy = 0.0f;
        imu.step_count = 0;
        imu.timestamp = batch->timestamp[0];
        
        // Quaternion data to central hub
        tagged_message_t imu_msg;
        imu_msg.msg.sender = SENDER_D2D_SECONDARY;
        imu_msg.msg.type = MSG_TYPE_BHI360_LOG_RECORD;
        imu_msg.tag = SECONDARY_IMU;
        memcpy(&imu_msg.msg.data.bhi360_log_record, &imu, sizeof(bhi360_log_record_t));
        if (k_msgq_put(&central_data_hub_msgq, &imu_msg, K_NO_WAIT) != 0) {
            LOG_ERR("Failed to queue quaternion sample to central hub");
        } else {
            LOG_DBG("Quaternion sample queued to central hub (gyro/accel zeroed)");
        }
    }

    LOG_INF("D2D batch processed and stored: 28 bytes (foot + quaternion only)");
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
    
    // Forward to central_data_hub_msgq with tagging for new architecture
  //  extern struct k_msgq central_data_hub_msgq;
  //  tagged_message_t tagged_msg;
  //  tagged_msg.msg.sender = SENDER_D2D_SECONDARY;
   // tagged_msg.msg.type = MSG_TYPE_FOOT_SAMPLES;
    //tagged_msg.tag = SECONDARY_FOOT;
   // memcpy(&tagged_msg.msg.data.foot_samples, samples, sizeof(foot_samples_t));
    
   // if (k_msgq_put(&central_data_hub_msgq, &tagged_msg, K_NO_WAIT) != 0) {
     //   LOG_WRN("Failed to forward secondary foot data to central data hub");
  //  } else {
       // LOG_DBG("Secondary foot data forwarded to central_data_hub_msgq with tag SECONDARY_FOOT");
  //  }
    
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
    
    // Forward to central_data_hub_msgq with tagging for new architecture
    extern struct k_msgq central_data_hub_msgq;
    tagged_message_t tagged_msg_imu;
    tagged_msg_imu.msg.sender = SENDER_D2D_SECONDARY;
    tagged_msg_imu.msg.type = MSG_TYPE_BHI360_LOG_RECORD;
    tagged_msg_imu.tag = SECONDARY_IMU;
    
    // Convert 3D mapping to log record format
    tagged_msg_imu.msg.data.bhi360_log_record.quat_x = data->accel_x;
    tagged_msg_imu.msg.data.bhi360_log_record.quat_y = data->accel_y;
    tagged_msg_imu.msg.data.bhi360_log_record.quat_z = data->accel_z;
    tagged_msg_imu.msg.data.bhi360_log_record.quat_w = data->quat_w;
    tagged_msg_imu.msg.data.bhi360_log_record.gyro_x = data->gyro_x;
    tagged_msg_imu.msg.data.bhi360_log_record.gyro_y = data->gyro_y;
    tagged_msg_imu.msg.data.bhi360_log_record.gyro_z = data->gyro_z;
    tagged_msg_imu.msg.data.bhi360_log_record.lacc_x = 0;
    tagged_msg_imu.msg.data.bhi360_log_record.lacc_y = 0;
    tagged_msg_imu.msg.data.bhi360_log_record.lacc_z = 0;
    
    if (k_msgq_put(&central_data_hub_msgq, &tagged_msg_imu, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary BHI360 3D data to central data hub");
    } else {
        LOG_DBG("Secondary BHI360 3D data forwarded to central_data_hub_msgq with tag SECONDARY_IMU");
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
    
    // Also forward to central_data_hub_msgq with tagging
    extern struct k_msgq central_data_hub_msgq;
    tagged_message_t tagged_msg_step;
    tagged_msg_step.msg.sender = SENDER_D2D_SECONDARY;
    tagged_msg_step.msg.type = MSG_TYPE_BHI360_STEP_COUNT;
    tagged_msg_step.tag = ACTIVITY_DATA;
    memcpy(&tagged_msg_step.msg.data.bhi360_step_count, data, sizeof(bhi360_step_count_t));
    
    if (k_msgq_put(&central_data_hub_msgq, &tagged_msg_step, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary step count to central data hub");
    } else {
        LOG_DBG("Secondary step count forwarded to central_data_hub_msgq with tag ACTIVITY_DATA");
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
    
    // Forward to central_data_hub_msgq with tagging
    extern struct k_msgq central_data_hub_msgq;
    tagged_message_t tagged_msg_accel;
    tagged_msg_accel.msg.sender = SENDER_D2D_SECONDARY;
    tagged_msg_accel.msg.type = MSG_TYPE_BHI360_LINEAR_ACCEL;
    tagged_msg_accel.tag = SECONDARY_IMU;
    memcpy(&tagged_msg_accel.msg.data.bhi360_linear_accel, data, sizeof(bhi360_linear_accel_t));
    
    if (k_msgq_put(&central_data_hub_msgq, &tagged_msg_accel, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward secondary linear accel to central data hub");
    } else {
        LOG_DBG("Secondary linear accel forwarded to central_data_hub_msgq with tag SECONDARY_IMU");
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

int d2d_data_handler_process_metrics(const d2d_metrics_packet_t *metrics)
{
    if (!metrics) {
        LOG_WRN("Received null D2D metrics packet");
        return -EINVAL;
    }

    LOG_INF("Processing D2D metrics packet: timestamp=%u, sequence=%u, status=%u",
            metrics->timestamp, metrics->sequence_num, metrics->calculation_status);

    // Forward metrics to sensor_data module for bilateral processing
    generic_message_t msg;
    msg.sender = SENDER_D2D_SECONDARY;
    msg.type = MSG_TYPE_D2D_METRICS;
    memcpy(&msg.data.d2d_metrics, metrics, sizeof(d2d_metrics_packet_t));
    
    if (k_msgq_put(&sensor_data_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward D2D metrics to sensor_data module");
        return -ENOSPC;
    } else {
        LOG_DBG("D2D metrics forwarded to sensor_data_msgq for bilateral processing");
    }

    // Also forward to bilateral_metrics module if enabled
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    extern struct k_msgq analytics_msgq;
    generic_message_t analytics_msg;
    analytics_msg.sender = SENDER_D2D_SECONDARY;
    analytics_msg.type = MSG_TYPE_D2D_METRICS_RECEIVED;
    memcpy(&analytics_msg.data.d2d_metrics, metrics, sizeof(d2d_metrics_packet_t));
    
    if (k_msgq_put(&analytics_msgq, &analytics_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to forward D2D metrics to analytics module");
    } else {
        LOG_DBG("D2D metrics forwarded to analytics_msgq for bilateral calculations");
    }
#endif

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
#include "ble_seq_manager.hpp"
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_seq_manager, LOG_LEVEL_WRN);

// Singleton constructor
BleSequenceManager::BleSequenceManager() 
    : foot_seq(0), bhi360_3d_seq(0), bhi360_accel_seq(0),
      recovery_mode(false), disconnect_timestamp(0) {
}

// Singleton instance
BleSequenceManager& BleSequenceManager::getInstance() {
    static BleSequenceManager instance;
    return instance;
}

// Helper to increment sequence number
uint8_t BleSequenceManager::nextSeq(uint8_t current) {
    // Increment and wrap at 127 to avoid 0xFF (recovery marker)
    return (current + 1) & 0x7F;
}

void BleSequenceManager::addFootSample(const foot_samples_t* data, foot_samples_ble_t* ble_data) {
    if (!data || !ble_data) {
        return;
    }

    // Add sequence number
    ble_data->seq_num = foot_seq;
    
    // Copy sensor data
    memcpy(ble_data->values, data->values, sizeof(data->values));
    
    // Store in recovery buffer if not in recovery mode
    if (!recovery_mode) {
        foot_buffer.push(*ble_data);
    }
    
    // Increment sequence for next packet
    foot_seq = nextSeq(foot_seq);
    
    LOG_DBG("Foot sample seq=%d, buffer_size=%d", ble_data->seq_num, foot_buffer.size());
}

void BleSequenceManager::addBhi3603D(const bhi360_3d_mapping_t* data, bhi360_3d_mapping_ble_t* ble_data) {
    if (!data || !ble_data) {
        return;
    }

    // Add sequence number
    ble_data->seq_num = bhi360_3d_seq;
    
    // Convert to fixed-point format
    // Note: In bhi360_3d_mapping_t, accel_x/y/z are actually used for quaternion x/y/z
    ble_data->quat_x = static_cast<int16_t>(data->accel_x * 10000);
    ble_data->quat_y = static_cast<int16_t>(data->accel_y * 10000);
    ble_data->quat_z = static_cast<int16_t>(data->accel_z * 10000);
    ble_data->quat_w = static_cast<int16_t>(data->quat_w * 10000);
    ble_data->gyro_x = static_cast<int16_t>(data->gyro_x * 10000);
    ble_data->gyro_y = static_cast<int16_t>(data->gyro_y * 10000);
    ble_data->gyro_z = static_cast<int16_t>(data->gyro_z * 10000);
    ble_data->quat_accuracy = 100; // Default accuracy (1.0 * 100)
    
    // Store in recovery buffer if not in recovery mode
    if (!recovery_mode) {
        bhi360_3d_buffer.push(*ble_data);
    }
    
    // Increment sequence for next packet
    bhi360_3d_seq = nextSeq(bhi360_3d_seq);
    
    LOG_DBG("BHI360 3D seq=%d, buffer_size=%d", ble_data->seq_num, bhi360_3d_buffer.size());
}

void BleSequenceManager::addBhi360Accel(const bhi360_linear_accel_t* data, bhi360_linear_accel_ble_t* ble_data) {
    if (!data || !ble_data) {
        return;
    }

    // Add sequence number
    ble_data->seq_num = bhi360_accel_seq;
    
    // Convert to fixed-point format (mm/s²)
    ble_data->x = static_cast<int16_t>(data->x * 1000);
    ble_data->y = static_cast<int16_t>(data->y * 1000);
    ble_data->z = static_cast<int16_t>(data->z * 1000);
    
    // Store in recovery buffer if not in recovery mode
    if (!recovery_mode) {
        bhi360_accel_buffer.push(*ble_data);
    }
    
    // Increment sequence for next packet
    bhi360_accel_seq = nextSeq(bhi360_accel_seq);
    
    LOG_DBG("BHI360 accel seq=%d, buffer_size=%d", ble_data->seq_num, bhi360_accel_buffer.size());
}

void BleSequenceManager::onDisconnect() {
    // Mark disconnection time
    disconnect_timestamp = k_uptime_get_32();
    LOG_INF("BLE disconnected, timestamp=%u", disconnect_timestamp);
}

void BleSequenceManager::onReconnect() {
    uint32_t current_time = k_uptime_get_32();
    uint32_t time_diff = current_time - disconnect_timestamp;
    
    // Only enter recovery if disconnection was recent
    if (disconnect_timestamp > 0 && time_diff < RECOVERY_TIMEOUT_MS) {
        recovery_mode = true;
        LOG_INF("Entering recovery mode, disconnect duration=%u ms", time_diff);
        LOG_INF("Buffer sizes - foot:%d, 3D:%d, accel:%d", 
                foot_buffer.size(), bhi360_3d_buffer.size(), bhi360_accel_buffer.size());
    } else {
        LOG_INF("No recovery needed, disconnect duration=%u ms", time_diff);
        // Clear old buffers if timeout exceeded
        if (time_diff >= RECOVERY_TIMEOUT_MS) {
            clearBuffers();
        }
    }
}

uint8_t BleSequenceManager::getFootBuffer(foot_samples_ble_t* out_buffer, uint8_t max_items) {
    return foot_buffer.getAll(out_buffer, max_items);
}

uint8_t BleSequenceManager::getBhi3603DBuffer(bhi360_3d_mapping_ble_t* out_buffer, uint8_t max_items) {
    return bhi360_3d_buffer.getAll(out_buffer, max_items);
}

uint8_t BleSequenceManager::getBhi360AccelBuffer(bhi360_linear_accel_ble_t* out_buffer, uint8_t max_items) {
    return bhi360_accel_buffer.getAll(out_buffer, max_items);
}

void BleSequenceManager::clearBuffers() {
    foot_buffer.clear();
    bhi360_3d_buffer.clear();
    bhi360_accel_buffer.clear();
    LOG_INF("All recovery buffers cleared");
}
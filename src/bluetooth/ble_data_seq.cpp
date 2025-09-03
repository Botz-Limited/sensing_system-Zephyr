/**
 * @file ble_data_seq.cpp
 * @brief Implementation of BLE sequence number management
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#include "ble_data_seq.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_data_seq, LOG_LEVEL_WRN);

void ble_seq_init(ble_seq_manager_t *manager)
{
    if (!manager) {
        LOG_ERR("NULL manager pointer");
        return;
    }
    
    manager->foot_seq = 0;
    manager->bhi360_3d_seq = 0;
    manager->bhi360_accel_seq = 0;
    
    LOG_INF("BLE sequence manager initialized");
}

seq_num_t ble_seq_next(seq_num_t current)
{
    // Simple increment with rollover at 255
    if (current == SEQ_NUM_MAX) {
        return 0;
    }
    return current + 1;
}

uint8_t ble_seq_gap(seq_num_t last_received, seq_num_t current)
{
    // Handle sequence number rollover
    if (current >= last_received) {
        // Normal case: no rollover
        return current - last_received - 1;
    } else {
        // Rollover case: current < last_received
        // Gap = (255 - last_received) + current
        return (SEQ_NUM_MAX - last_received) + current;
    }
}

bool ble_seq_recoverable(seq_num_t last_sent, seq_num_t requested)
{
    // Calculate how far back the requested sequence is
    uint8_t distance;
    
    if (last_sent >= requested) {
        // Normal case
        distance = last_sent - requested;
    } else {
        // Rollover case
        distance = (SEQ_NUM_MAX - requested) + last_sent + 1;
    }
    
    // Check if it's within our buffer size
    return distance <= BLE_RECOVERY_BUFFER_SIZE;
}
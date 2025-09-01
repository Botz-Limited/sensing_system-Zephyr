/**
 * @file sensor_data_ring_buffer.h
 * @brief Ring buffer implementation for sensor data module
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#ifndef SENSOR_DATA_RING_BUFFER_HPP
#define SENSOR_DATA_RING_BUFFER_HPP

#include <zephyr/kernel.h>
#include <app.hpp>

// Ring buffer sizes - must be power of 2 for efficient masking
#define FOOT_DATA_RING_SIZE     8   // Can buffer 80ms of foot data at 100Hz
#define IMU_DATA_RING_SIZE      8   // Can buffer 80ms of IMU data at 100Hz
#define COMMAND_RING_SIZE       4   // Commands are less frequent

// Foot data ring buffer
typedef struct {
    foot_samples_t data[FOOT_DATA_RING_SIZE];
    uint8_t foot_id[FOOT_DATA_RING_SIZE];  // Which foot (0=left, 1=right)
    uint8_t write_idx;
    uint8_t read_idx;
    atomic_t count;
} foot_data_ring_t;

// IMU data ring buffer
typedef struct {
    bhi360_log_record_t data[IMU_DATA_RING_SIZE];
    uint8_t write_idx;
    uint8_t read_idx;
    atomic_t count;
} imu_data_ring_t;

// Command ring buffer
typedef struct {
    char data[COMMAND_RING_SIZE][MAX_COMMAND_STRING_LEN];
    uint8_t write_idx;
    uint8_t read_idx;
    atomic_t count;
} command_ring_t;

// Ring buffer operations
static inline bool foot_ring_put(foot_data_ring_t *ring, 
                                 const foot_samples_t *data, 
                                 uint8_t foot_id)
{
    if (atomic_get(&ring->count) >= FOOT_DATA_RING_SIZE) {
        return false;  // Buffer full
    }
    
    memcpy(&ring->data[ring->write_idx], data, sizeof(foot_samples_t));
    ring->foot_id[ring->write_idx] = foot_id;
    
    ring->write_idx = (ring->write_idx + 1) & (FOOT_DATA_RING_SIZE - 1);
    atomic_inc(&ring->count);
    
    return true;
}

static inline bool foot_ring_get(foot_data_ring_t *ring, 
                                 foot_samples_t *data, 
                                 uint8_t *foot_id)
{
    if (atomic_get(&ring->count) == 0) {
        return false;  // Buffer empty
    }
    
    memcpy(data, &ring->data[ring->read_idx], sizeof(foot_samples_t));
    *foot_id = ring->foot_id[ring->read_idx];
    
    ring->read_idx = (ring->read_idx + 1) & (FOOT_DATA_RING_SIZE - 1);
    atomic_dec(&ring->count);
    
    return true;
}

static inline bool imu_ring_put(imu_data_ring_t *ring, 
                                const bhi360_log_record_t *data)
{
    if (atomic_get(&ring->count) >= IMU_DATA_RING_SIZE) {
        return false;  // Buffer full
    }
    
    memcpy(&ring->data[ring->write_idx], data, sizeof(bhi360_log_record_t));
    
    ring->write_idx = (ring->write_idx + 1) & (IMU_DATA_RING_SIZE - 1);
    atomic_inc(&ring->count);
    
    return true;
}

static inline bool imu_ring_get(imu_data_ring_t *ring, 
                                bhi360_log_record_t *data)
{
    if (atomic_get(&ring->count) == 0) {
        return false;  // Buffer empty
    }
    
    memcpy(data, &ring->data[ring->read_idx], sizeof(bhi360_log_record_t));
    
    ring->read_idx = (ring->read_idx + 1) & (IMU_DATA_RING_SIZE - 1);
    atomic_dec(&ring->count);
    
    return true;
}

static inline bool command_ring_put(command_ring_t *ring, 
                                    const char *command)
{
    if (atomic_get(&ring->count) >= COMMAND_RING_SIZE) {
        return false;  // Buffer full
    }
    
    strncpy(ring->data[ring->write_idx], command, MAX_COMMAND_STRING_LEN - 1);
    ring->data[ring->write_idx][MAX_COMMAND_STRING_LEN - 1] = '\0';
    
    ring->write_idx = (ring->write_idx + 1) & (COMMAND_RING_SIZE - 1);
    atomic_inc(&ring->count);
    
    return true;
}

static inline bool command_ring_get(command_ring_t *ring, 
                                    char *command)
{
    if (atomic_get(&ring->count) == 0) {
        return false;  // Buffer empty
    }
    
    strncpy(command, ring->data[ring->read_idx], MAX_COMMAND_STRING_LEN);
    
    ring->read_idx = (ring->read_idx + 1) & (COMMAND_RING_SIZE - 1);
    atomic_dec(&ring->count);
    
    return true;
}

// Statistics tracking
typedef struct {
    uint32_t foot_data_received;
    uint32_t foot_data_dropped;
    uint32_t imu_data_received;
    uint32_t imu_data_dropped;
    uint32_t commands_received;
    uint32_t commands_dropped;
    uint32_t max_foot_ring_depth;
    uint32_t max_imu_ring_depth;
} sensor_data_stats_t;

#endif // SENSOR_DATA_RING_BUFFER_HPP
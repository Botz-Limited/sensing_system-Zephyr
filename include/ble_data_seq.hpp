/**
 * @file ble_data_seq.hpp
 * @brief BLE data structures with sequence numbers for packet loss detection
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 *
 * This file defines BLE-specific data structures that include sequence numbers
 * for detecting packet loss during transmission. These structures are used
 * ONLY for BLE transmission, not for file logging.
 */

#ifndef BLE_DATA_SEQ_HPP
#define BLE_DATA_SEQ_HPP

#include <cstdint>
#include "app.hpp"
#include "app_fixed_point.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// Sequence number type - 8-bit rolling counter (0-255)
typedef uint8_t seq_num_t;

// Maximum sequence number before rollover
#define SEQ_NUM_MAX 255

// Buffer size for recovery mechanism (number of packets to keep)
#define BLE_RECOVERY_BUFFER_SIZE 10  // ~500ms at 20Hz for foot data

/**
 * @brief Foot sensor data with sequence number for BLE transmission
 * 
 * This structure adds a sequence number to foot sensor data for
 * packet loss detection during BLE transmission.
 */
typedef struct __attribute__((packed)) {
    seq_num_t seq_num;                          // 1 byte: Rolling sequence number
    uint16_t values[NUM_FOOT_SENSOR_CHANNELS];  // 16 bytes: 8 channels
} foot_samples_ble_t;  // Total: 17 bytes

/**
 * @brief BHI360 3D mapping data with sequence number for BLE transmission
 * 
 * Uses fixed-point format for efficient BLE transmission
 */
typedef struct __attribute__((packed)) {
    seq_num_t seq_num;         // 1 byte: Rolling sequence number
    int16_t quat_x;           // 2 bytes: Quaternion X × 10000
    int16_t quat_y;           // 2 bytes: Quaternion Y × 10000
    int16_t quat_z;           // 2 bytes: Quaternion Z × 10000
    int16_t quat_w;           // 2 bytes: Quaternion W × 10000
    int16_t gyro_x;           // 2 bytes: Gyroscope X × 10000 (rad/s)
    int16_t gyro_y;           // 2 bytes: Gyroscope Y × 10000 (rad/s)
    int16_t gyro_z;           // 2 bytes: Gyroscope Z × 10000 (rad/s)
    uint8_t quat_accuracy;    // 1 byte: Accuracy × 100 (0-300)
} bhi360_3d_mapping_ble_t;  // Total: 16 bytes

/**
 * @brief BHI360 linear acceleration with sequence number for BLE transmission
 */
typedef struct __attribute__((packed)) {
    seq_num_t seq_num;  // 1 byte: Rolling sequence number
    int16_t x;         // 2 bytes: Acceleration X × 1000 (mm/s²)
    int16_t y;         // 2 bytes: Acceleration Y × 1000 (mm/s²)
    int16_t z;         // 2 bytes: Acceleration Z × 1000 (mm/s²)
} bhi360_linear_accel_ble_t;  // Total: 7 bytes

/**
 * @brief BHI360 step count - no sequence number needed (low rate)
 * 
 * Step count updates are infrequent enough that packet loss
 * is not a significant concern
 */
typedef bhi360_step_count_fixed_t bhi360_step_count_ble_t;

/**
 * @brief Recovery request structure
 * 
 * Used by mobile app to request missing packets after detecting gaps
 */
typedef struct __attribute__((packed)) {
    uint8_t data_type;    // 0=foot, 1=BHI360_3D, 2=BHI360_accel
    seq_num_t start_seq;  // First missing sequence number
    seq_num_t end_seq;    // Last missing sequence number
} ble_recovery_request_t;

/**
 * @brief Recovery response header
 * 
 * Sent before retransmitting requested packets
 */
typedef struct __attribute__((packed)) {
    uint8_t data_type;     // Same as request
    seq_num_t start_seq;   // First sequence in response
    uint8_t packet_count;  // Number of packets following
} ble_recovery_response_t;

/**
 * @brief Sequence number manager
 * 
 * Tracks current sequence numbers for each data type
 */
typedef struct {
    seq_num_t foot_seq;
    seq_num_t bhi360_3d_seq;
    seq_num_t bhi360_accel_seq;
} ble_seq_manager_t;

/**
 * @brief Initialize sequence number manager
 */
void ble_seq_init(ble_seq_manager_t *manager);

/**
 * @brief Get next sequence number (with rollover)
 */
seq_num_t ble_seq_next(seq_num_t current);

/**
 * @brief Calculate number of missed packets between two sequence numbers
 * 
 * Handles rollover correctly
 */
uint8_t ble_seq_gap(seq_num_t last_received, seq_num_t current);

/**
 * @brief Check if a sequence number is within recovery buffer range
 */
bool ble_seq_recoverable(seq_num_t last_sent, seq_num_t requested);

#ifdef __cplusplus
}
#endif

#endif /* BLE_DATA_SEQ_HPP */
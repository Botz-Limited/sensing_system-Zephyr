/**
 * @file app.hpp
 * @brief
 * @version 1.0
 * @date 15/5/2025
 *
 * @copyright Botz Innovation 2025
 *
 */
#ifndef APP_INCLUDE_APP_HEADER_
#define APP_INCLUDE_APP_HEADER_

#include <cstdint>
#include <events/bluetooth_state_event.h>
#include <events/app_state_event.h>
#include <events/data_event.h>
#include <zephyr/kernel.h>
#include <time.h>

void app_log(const char *fmt, ...);

#define MAX_FILE_PATH_LEN 64


// Define a structure for foot sensor samples
#define NUM_FOOT_SENSOR_CHANNELS 8
struct foot_samples_t {
    uint16_t values[NUM_FOOT_SENSOR_CHANNELS]; // This array will now hold 8 uint16_t values
    // ... any other fields ...
};

// Structures for BHI360 data (unchanged)
typedef struct
{
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;

} bhi360_3d_mapping_t; // Size: 6 * 4 bytes (floats) + 8 bytes (uint64_t) = 24 + 8 = 32 bytes

typedef struct
{
    float x, y, z;
} bhi360_linear_accel_t; // Size: 12 bytes

typedef struct
{
    uint32_t step_count;
    uint32_t activity_duration_s;
} bhi360_step_count_t; // Size: 8 bytes

typedef struct
{
    float quat_x, quat_y, quat_z, quat_w, quat_accuracy;
    float lacc_x, lacc_y, lacc_z;
    uint32_t step_count;
    uint64_t timestamp;
} bhi360_log_record_t;

// If you send commands, define a max size for the command string
#define MAX_COMMAND_STRING_LEN 32 // Example max length for a command string
typedef char command_data_t[MAX_COMMAND_STRING_LEN];

typedef struct {
    uint8_t file_sequence_id;
    char file_path[MAX_FILE_PATH_LEN];
} new_log_info_msg_t;

typedef struct {
    record_type_t type; // e.g., RECORD_HARDWARE_FOOT_SENSOR
    uint8_t id;        // The sequence ID of the file to delete
} delete_log_command_t;

// Generic message wrapper with Union ---
// This struct will now directly hold the data payload using a union.
// The size of this struct will be the size of its largest member in the union.
typedef struct
{
    sender_type_t sender;
    msg_type_t type;
    union {
        foot_samples_t foot_samples;
        bhi360_3d_mapping_t bhi360_3d_mapping;
        bhi360_linear_accel_t bhi360_linear_accel;
        bhi360_step_count_t bhi360_step_count;
        bhi360_log_record_t bhi360_log_record;
        new_log_info_msg_t new_hardware_log_file;
        command_data_t command_str; // For command messages
        delete_log_command_t delete_cmd;
    } data;                         // All actual data payloads will be stored here
} generic_message_t;

// Define the message queue parameters based on the new generic_message_t size
// Ensure CONFIG_MSG_Q_MAX_SIZE_BYTES in Kconfig is at least sizeof(generic_message_t)
#define MSG_QUEUE_MESSAGE_SIZE sizeof(generic_message_t)
#define MSG_QUEUE_DEPTH 10 // Adjust if necessary

// --- MESSAGE QUEUE DECLARATION (unchanged) ---
extern struct k_msgq bluetooth_msgq;
extern struct k_msgq data_msgq;

const char *get_sender_name(sender_type_t sender);

#endif // APP_INCLUDE_APP_HEADER_

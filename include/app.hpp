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

#include <events/bluetooth_state_event.h>
#include <events/data_event.h>
#include <zephyr/kernel.h>

void app_log(const char *fmt, ...);

// Define message types (unchanged)
typedef enum
{
    MSG_TYPE_FOOT_SAMPLES,
    MSG_TYPE_BHI360_3D_MAPPING,
    MSG_TYPE_BHI360_STEP_COUNT,
    MSG_TYPE_COMMAND, // If you send commands directly too
} msg_type_t;

// Define sender types (unchanged)
typedef enum
{
    SENDER_NONE,
    SENDER_FOOT_SENSOR_THREAD,
    SENDER_BHI360_THREAD,
    SENDER_UI_THREAD,
} sender_type_t;

// Define a structure for foot sensor samples (unchanged)
#define NUM_FOOT_SENSOR_CHANNELS 8
typedef struct
{
    uint16_t values[NUM_FOOT_SENSOR_CHANNELS];
} foot_samples_t; // Size: 16 bytes

// Structures for BHI360 data (unchanged)
typedef struct
{
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    uint64_t timestamp_us;
} bhi360_3d_mapping_t; // Size: 6 * 4 bytes (floats) + 8 bytes (uint64_t) = 24 + 8 = 32 bytes

typedef struct
{
    uint32_t step_count;
    uint32_t activity_duration_s;
} bhi360_step_count_t; // Size: 8 bytes

// If you send commands, define a max size for the command string
#define MAX_COMMAND_STRING_LEN 32 // Example max length for a command string
typedef char command_data_t[MAX_COMMAND_STRING_LEN];

// --- NEW: Generic message wrapper with Union ---
// This struct will now directly hold the data payload using a union.
// The size of this struct will be the size of its largest member in the union.
typedef struct
{
    sender_type_t sender;
    msg_type_t type;
    union {
        foot_samples_t foot_samples;
        bhi360_3d_mapping_t bhi360_3d_mapping;
        bhi360_step_count_t bhi360_step_count;
        command_data_t command_str; // For command messages
    } data;                         // All actual data payloads will be stored here
} generic_message_t;

// Define the message queue parameters based on the new generic_message_t size
// Ensure CONFIG_MSG_Q_MAX_SIZE_BYTES in Kconfig is at least sizeof(generic_message_t)
#define MSG_QUEUE_MESSAGE_SIZE sizeof(generic_message_t)
#define MSG_QUEUE_DEPTH 10 // Adjust if necessary

// --- MESSAGE QUEUE DECLARATION (unchanged) ---
extern struct k_msgq bluetooth_msgq;

#endif // APP_INCLUDE_APP_HEADER_

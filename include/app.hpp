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
#include <errors.hpp>

void app_log(const char *fmt, ...);

static constexpr uint8_t MAX_FILE_PATH_LEN = 64;


// Define a structure for foot sensor samples
static constexpr uint8_t NUM_FOOT_SENSOR_CHANNELS = 8;
struct foot_samples_t {
    uint16_t values[NUM_FOOT_SENSOR_CHANNELS]; // This array will now hold 8 uint16_t values
    // ... any other fields ...
};

// Structures for BHI360 data
typedef struct
{
    float accel_x, accel_y, accel_z;  // Currently used for quaternion x,y,z
    float gyro_x, gyro_y, gyro_z;     // Gyroscope angular velocity
    float quat_w;                     // Quaternion W component (added)
} bhi360_3d_mapping_t; // Size: 7 * 4 bytes (floats) = 28 bytes

// High-rate quaternion data for 3D visualization
typedef struct
{
    float quat_w, quat_x, quat_y, quat_z;
    uint32_t timestamp_ms;
} quaternion_3d_t; // Size: 20 bytes

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
    float gyro_x, gyro_y, gyro_z;  // Added gyroscope data
    uint32_t step_count;
    uint64_t timestamp;
} bhi360_log_record_t;

// If you send commands, define a max size for the command string
static constexpr uint8_t MAX_COMMAND_STRING_LEN = 32; // Example max length for a command string
typedef char command_data_t[MAX_COMMAND_STRING_LEN];

typedef struct {
    uint8_t file_sequence_id;
    char file_path[MAX_FILE_PATH_LEN];
} new_log_info_msg_t;

typedef struct {
    record_type_t type; // e.g., RECORD_HARDWARE_FOOT_SENSOR
    uint8_t id;        // The sequence ID of the file to delete
} delete_log_command_t;

// FOTA progress structure
typedef struct {
    bool is_active;
    uint8_t status; // 0=idle, 1=in_progress, 2=pending, 3=confirmed, 4=error
    uint8_t percent_complete;
    uint32_t bytes_received;
    uint32_t total_size;
    int32_t error_code;
} fota_progress_msg_t;

// Error status structure for reporting module errors
typedef struct {
    err_t error_code;  // The error code from errors.hpp
    bool is_set;       // true = set error, false = clear error
} error_status_msg_t;

// BHI360 calibration data structure
typedef struct {
    uint8_t sensor_type;  // 0=accel, 1=gyro, 2=mag
    uint16_t profile_size;
    uint8_t *profile_data;  // Pointer to calibration data (dynamically allocated)
} bhi360_calibration_data_t;

// Weight calibration data structure
typedef struct {
    float scale_factor;     // kg per ADC unit
    float nonlinear_a;      // Nonlinear coefficient A
    float nonlinear_b;      // Nonlinear coefficient B  
    float nonlinear_c;      // Nonlinear coefficient C
    float temp_coeff;       // Temperature coefficient (%/°C)
    float temp_ref;         // Reference temperature (°C)
    bool is_calibrated;     // Flag to indicate if calibration is valid
} weight_calibration_data_t;

// Weight calibration step data (for calibration procedure)
typedef struct {
    float known_weight_kg;  // User's actual weight for calibration
} weight_calibration_step_t;

// Activity step count data for activity file
typedef struct {
    uint16_t left_step_count;   // Left foot step count
    uint16_t right_step_count;  // Right foot step count
} activity_step_count_t;

// Device information structure for D2D sharing
typedef struct {
    char manufacturer[32];
    char model[16];
    char serial[16];
    char hw_rev[16];
    char fw_rev[16];
} device_info_msg_t;

// Weight measurement result
typedef struct {
    float weight_kg;
    uint32_t timestamp;
} weight_measurement_msg_t;

// Generic message wrapper with Union ---
// This struct will now directly hold the data payload using a union.
// The size of this struct will be the size of its largest member in the union.
typedef struct
{
    sender_type_t sender;
    msg_type_t type;
    // Metadata for command messages (ignored for sensor data)
    char fw_version[32];
    uint32_t sampling_frequency;
    union {
        foot_samples_t foot_samples;
        bhi360_3d_mapping_t bhi360_3d_mapping;
        bhi360_linear_accel_t bhi360_linear_accel;
        bhi360_step_count_t bhi360_step_count;
        bhi360_log_record_t bhi360_log_record;
        quaternion_3d_t quaternion_3d;
        new_log_info_msg_t new_hardware_log_file;
        command_data_t command_str; // For command messages
        delete_log_command_t delete_cmd;
        fota_progress_msg_t fota_progress;
        error_status_msg_t error_status;
        bhi360_calibration_data_t bhi360_calibration;
        activity_step_count_t activity_step_count;
        device_info_msg_t device_info;
        weight_calibration_data_t weight_calibration;
        weight_calibration_step_t weight_calibration_step;
        weight_measurement_msg_t weight_measurement;
    } data;                         // All actual data payloads will be stored here
} generic_message_t;

// Define the message queue parameters based on the new generic_message_t size
// Ensure CONFIG_MSG_Q_MAX_SIZE_BYTES in Kconfig is at least sizeof(generic_message_t)
#define MSG_QUEUE_MESSAGE_SIZE sizeof(generic_message_t)
static constexpr uint8_t MSG_QUEUE_DEPTH = 20; // Reduced to save RAM - was causing 252KB usage

// --- MESSAGE QUEUE DECLARATION (unchanged) ---
extern struct k_msgq bluetooth_msgq;
extern struct k_msgq data_msgq;
extern struct k_msgq motion_sensor_msgq;
extern struct k_msgq activity_metrics_msgq;
extern struct k_msgq sensor_data_msgq;  // New multi-thread architecture queue
#if defined(CONFIG_WIFI_MODULE)
extern struct k_msgq wifi_msgq;
#endif

const char *get_sender_name(sender_type_t sender);

#endif // APP_INCLUDE_APP_HEADER_

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

#include "../src/realtime_metrics/realtime_metrics.h"      // For realtime_metrics_t
#include "../src/sensor_data/sensor_data_consolidated.hpp" // For sensor_data_consolidated_t
#include <activity_session.hpp>                            // For GPSUpdateCommand
#include <cstdint>
#include <d2d_metrics.h> // For d2d_metrics_packet_t
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/bluetooth_state_event.h>
#include <events/data_event.h>
#include <time.h>
#include <zephyr/kernel.h>

void app_log(const char *fmt, ...);

static constexpr uint8_t MAX_FILE_PATH_LEN = 34;

// Define a structure for foot sensor samples
static constexpr uint8_t NUM_FOOT_SENSOR_CHANNELS = 8;
struct foot_samples_t
{
    uint16_t values[NUM_FOOT_SENSOR_CHANNELS]; // This array will now hold 8 uint16_t values
    // ... any other fields ...
};

// Aggregated foot samples for primary + secondary (36 bytes)
struct foot_samples_aggregated_t
{
    uint32_t timestamp;                           // 4 bytes - uptime in ms
    uint16_t primary[NUM_FOOT_SENSOR_CHANNELS];   // 16 bytes - primary foot channels
    uint16_t secondary[NUM_FOOT_SENSOR_CHANNELS]; // 16 bytes - secondary foot channels
} __attribute__((packed));

// Aggregated quaternion for primary + secondary (20 bytes)
struct quaternion_aggregated_t
{
    uint32_t timestamp;  // 4 bytes - uptime in ms
    int16_t primary_x;   // 2 bytes
    int16_t primary_y;   // 2 bytes
    int16_t primary_z;   // 2 bytes
    int16_t primary_w;   // 2 bytes
    int16_t secondary_x; // 2 bytes
    int16_t secondary_y; // 2 bytes
    int16_t secondary_z; // 2 bytes
    int16_t secondary_w; // 2 bytes
} __attribute__((packed));

// Structures for BHI360 data
typedef struct
{
    float accel_x, accel_y, accel_z; // Currently used for quaternion x,y,z
    float gyro_x, gyro_y, gyro_z;    // Gyroscope angular velocity
    float quat_w;                    // Quaternion W component (added)
} bhi360_3d_mapping_t;               // Size: 7 * 4 bytes (floats) = 28 bytes

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
    float gyro_x, gyro_y, gyro_z; // Added gyroscope data
    uint32_t step_count;
    uint64_t timestamp;
} bhi360_log_record_t;

// If you send commands, define a max size for the command string
static constexpr uint8_t MAX_COMMAND_STRING_LEN = 32; // Example max length for a command string
typedef char command_data_t[MAX_COMMAND_STRING_LEN];

typedef struct
{
    uint8_t file_sequence_id;
    char file_path[MAX_FILE_PATH_LEN];
} new_log_info_msg_t;

typedef struct
{
    record_type_t type; // e.g., RECORD_HARDWARE_FOOT_SENSOR
    uint8_t id;         // The sequence ID of the file to delete
} delete_log_command_t;

// FOTA progress structure
typedef struct
{
    bool is_active;
    uint8_t status; // 0=idle, 1=in_progress, 2=pending, 3=confirmed, 4=error
    uint8_t percent_complete;
    uint32_t bytes_received;
    uint32_t total_size;
    int32_t error_code;
} fota_progress_msg_t;

// Error status structure for reporting module errors
typedef struct
{
    err_t error_code; // The error code from errors.hpp
    bool is_set;      // true = set error, false = clear error
} error_status_msg_t;

// BHI360 calibration data structure
typedef struct
{
    uint8_t sensor_type; // 0=accel, 1=gyro, 2=mag
    uint16_t profile_size;
    uint8_t *profile_data; // Pointer to calibration data (dynamically allocated)
} bhi360_calibration_data_t;

// Weight calibration data structure
typedef struct
{
    float scale_factor; // kg per ADC unit
    float nonlinear_a;  // Nonlinear coefficient A
    float nonlinear_b;  // Nonlinear coefficient B
    float nonlinear_c;  // Nonlinear coefficient C
    float temp_coeff;   // Temperature coefficient (%/°C)
    float temp_ref;     // Reference temperature (°C)
    bool is_calibrated; // Flag to indicate if calibration is valid
} weight_calibration_data_t;

// Foot weight map calibration data structure
typedef struct
{
    uint16_t sensor_map[NUM_FOOT_SENSOR_CHANNELS];       // Average sensor values during standing
    uint32_t total_weight;                               // Total weight across all sensors
    float weight_distribution[NUM_FOOT_SENSOR_CHANNELS]; // Percentage distribution per sensor
    uint32_t timestamp_ms;                               // When calibration was performed
    uint16_t sample_count;                               // Number of samples averaged
    bool is_valid;                                       // Flag to indicate if calibration is valid
} foot_weight_map_data_t;

typedef struct __attribute__((packed))
{
    uint16_t voltage_mV; // Battery voltage in millivolts
    int8_t percentage;   // Estimated charge percentage (-1 if unknown)
    uint8_t status;      // Charging status (0=discharging, 1=charging, 2=full)
} battery_info_t;

typedef struct
{
    uint8_t user_height_cm;
    uint8_t user_weight_kg;
    uint8_t user_age_years;
    uint8_t user_sex;
} user_config_t;

struct ActivityFileHeaderV3
{
    char magic[4];
    uint8_t version;
    uint32_t start_time;
    uint32_t sample_rate;
    char fw_version[16];
    uint8_t user_height_cm;
    uint8_t user_weight_kg;
    uint8_t user_age_years;
    uint8_t user_sex;
    battery_info_t battery;
    uint8_t service_uuid[16];
};

typedef struct
{
    uint32_t packet_number;
    uint32_t timestamp;
    uint16_t cadence_spm;
    uint16_t pace_sec_km;
    uint16_t speed_kmh_x10; // Speed in km/h * 10 (fixed point, 1 decimal)
    int8_t balance_lr_pct;
    uint16_t ground_contact_ms;
    uint16_t flight_time_ms;
    int8_t contact_time_asymmetry;
    int8_t force_asymmetry;
    int8_t pronation_asymmetry;
    uint8_t left_strike_pattern;
    uint8_t right_strike_pattern;
    int8_t avg_pronation_deg;
    uint8_t vertical_ratio;
} __attribute__((packed)) activity_metrics_binary_t;

struct ActivityFileFooterV3
{
    uint32_t end_time;      // end epoch time
    uint32_t record_count;  // Total records written
    uint32_t packet_count;  // Total packets processed
    uint32_t file_crc;      // CRC-32 placeholder
    battery_info_t battery; // Battery at session end
};

// Weight calibration step data (for calibration procedure)
typedef struct
{
    float known_weight_kg; // User's actual weight for calibration
} weight_calibration_step_t;

// Activity step count data for activity file
typedef struct
{
    uint16_t left_step_count;  // Left foot step count
    uint16_t right_step_count; // Right foot step count
} activity_step_count_t;

// Device information structure for D2D sharing
typedef struct
{
    char manufacturer[32];
    char model[16];
    char serial[16];
    char hw_rev[16];
    char fw_rev[16];
} device_info_msg_t;

// Weight measurement result
typedef struct
{
    float weight_kg;
    uint32_t timestamp;
} weight_measurement_msg_t;

// Battery level message
typedef struct
{
    uint8_t level; // Battery level percentage (0-100)
} battery_level_msg_t;

// Synchronized foot data structure
typedef struct
{
    foot_samples_t left;  // Secondary device (left foot)
    foot_samples_t right; // Primary device (right foot)
    uint32_t sync_time;   // Optional: k_uptime_get_32() when paired
} synchronized_foot_data_t;

// Enhanced synchronized data with IMU information
typedef struct
{
    // Foot pressure data
    foot_samples_t left_foot;
    foot_samples_t right_foot;

    // IMU data from each device
    struct
    {
        float quaternion[4];
        float linear_acc[3];
        float gyro[3];
        uint32_t step_count;
        bool valid; // Indicates if IMU data is available
    } left_imu;     // From secondary device

    struct
    {
        float quaternion[4];
        float linear_acc[3];
        float gyro[3];
        uint32_t step_count;
        bool valid; // Indicates if IMU data is available
    } right_imu;    // From primary device

    // Timing
    uint32_t sync_time;
    uint16_t delta_time_ms;

    // Metadata
    uint8_t sync_quality; // 0-100%, based on time difference
} full_synchronized_data_t;

// Analytics results structure for passing complex calculations
typedef struct
{
    float running_efficiency; // 0-100%
    float fatigue_index;      // 0-100
    float injury_risk;        // 0-100
    float stride_length;      // meters
    float pronation_angle;    // degrees
    float vertical_stiffness; // N/m
    uint32_t timestamp_ms;    // When calculated
} analytics_results_t;

// Biomechanics extended data for BLE transmission
typedef struct
{
    int8_t pronation_left;
    int8_t pronation_right;
    uint16_t loading_rate_left;
    uint16_t loading_rate_right;
    uint8_t arch_collapse_left;
    uint8_t arch_collapse_right;
} biomechanics_extended_msg_t;

// Session summary data for BLE transmission
typedef struct
{
    uint32_t total_distance_m;
    uint16_t avg_pace_sec_km;
    uint16_t avg_cadence_spm;
    uint32_t total_steps;
    uint16_t calories_kcal;
    uint8_t avg_form_score;
    uint32_t duration_sec;
} session_summary_msg_t;

// File copy to SD card message (LAB_VERSION only)
typedef struct
{
    char source_path[64];   // Path to file in internal flash
    char dest_filename[64]; // Destination filename on SD card
} copy_file_to_sd_msg_t;

// Default sampling frequency for activity logging (in Hz)
#define DEFAULT_ACTIVITY_SAMPLING_FREQUENCY_HZ 100

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
    uint8_t service_uuid[16]; // BLE service UUID for activity logging
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
        GPSUpdateCommand gps_update;                       // GPS update data
        sensor_data_consolidated_t sensor_consolidated;    // Consolidated sensor data
        synchronized_foot_data_t sync_foot_data;           // Add this
        full_synchronized_data_t full_sync_data;           // Full sync data
        realtime_metrics_t realtime_metrics;               // Real-time metrics data
        analytics_results_t analytics_results;             // Analytics calculation results
        battery_level_msg_t battery_level;                 // Battery level from secondary device
        d2d_metrics_packet_t d2d_metrics;                  // D2D calculated metrics packet
        biomechanics_extended_msg_t biomechanics_extended; // Biomechanics extended data
        session_summary_msg_t session_summary;             // Session summary data
        copy_file_to_sd_msg_t copy_to_sd;                  // File copy request (LAB_VERSION only)
        foot_weight_map_data_t foot_weight_map;            // Foot weight map calibration data
        ActivityFileHeaderV3 activity_header;
        activity_metrics_binary_t activity_metrics;
        ActivityFileFooterV3 activity_footer;
    } data; // All actual data payloads will be stored here
} generic_message_t;

// Define the message queue parameters based on the new generic_message_t size
// Ensure CONFIG_MSG_Q_MAX_SIZE_BYTES in Kconfig is at least sizeof(generic_message_t)
#define MSG_QUEUE_MESSAGE_SIZE sizeof(generic_message_t)
static constexpr uint8_t MSG_QUEUE_DEPTH = 40;

// --- MESSAGE QUEUE DECLARATION (unchanged) ---
extern struct k_msgq bluetooth_msgq;
extern struct k_msgq data_msgq;
extern struct k_msgq data_sd_msgq;
extern struct k_msgq motion_sensor_msgq;
extern struct k_msgq activity_metrics_msgq;
extern struct k_msgq sensor_data_msgq; // New multi-thread architecture queue
extern struct k_msgq sync_pair_msgq;   // synchronized pairs
extern struct k_msgq full_sync_msgq;
#if defined(CONFIG_WIFI_MODULE)
extern struct k_msgq wifi_msgq;
#endif

const char *get_sender_name(sender_type_t sender);

#endif // APP_INCLUDE_APP_HEADER_

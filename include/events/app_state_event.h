#ifndef APP_INCLUDE_APP_EVENT_H_
#define APP_INCLUDE_APP_EVENT_H_

#include <app_event_manager.h> // Required for APP_EVENT_TYPE_DECLARE

#ifdef __cplusplus
extern "C"
{
#endif

// Define an enum for various application states
typedef enum {
    APP_STATE_UNKNOWN, // Default or uninitialized state
    APP_STATE_INIT,    // Application is initializing
    APP_STATE_READY,   // Application is fully operational
    APP_STATE_SHUTDOWN_PREPARING, // Application is preparing for shutdown/restart
    APP_STATE_SHUTDOWN, // Application has shut down
    APP_STATE_ERROR    // Application is in an error state
} app_state_t;

struct app_state_event {
    struct app_event_header header;

    /* Custom data fields. */
    app_state_t state;     // Changed from const char* to app_state_t enum
    app_state_t exit_state; // Changed from const char* to app_state_t enum (optional, depends on usage)
};

// Declare the event type for the application state event manager
APP_EVENT_TYPE_DECLARE(app_state_event);

typedef enum {
    RECORD_NONE = 0,
    RECORD_METADATA = 1,
    RECORD_HARDWARE_FOOT_SENSOR = 2,
    RECORD_HARDWARE_BHI360 = 3,
    RECORD_HARDWARE_ACTIVITY = 4,
    // Add other record types as needed
} record_type_t;


// Define message types
typedef enum
{
    MSG_TYPE_NONE,
    MSG_TYPE_STATUS,
    MSG_TYPE_FOOT_SAMPLES,
    MSG_TYPE_BHI360_3D_MAPPING,
    MSG_TYPE_BHI360_LINEAR_ACCEL,
    MSG_TYPE_BHI360_STEP_COUNT,
    MSG_TYPE_BHI360_LOG_RECORD,
    MSG_TYPE_QUATERNION_3D,  // High-rate quaternion for 3D visualization
    MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE,
    MSG_TYPE_NEW_BHI360_LOG_FILE,
    MSG_TYPE_NEW_ACTIVITY_LOG_FILE,
    MSG_TYPE_ACTIVITY_STEP_COUNT, // Step count data for activity file
    MSG_TYPE_COMMAND, // Generic command string
    MSG_TYPE_DELETE_FOOT_LOG, // Specific message type for deleting foot sensor logs
    MSG_TYPE_DELETE_BHI360_LOG, // Specific message type for deleting BHI360 logs
    MSG_TYPE_DELETE_ACTIVITY_LOG, // Specific message type for deleting activity logs
    MSG_TYPE_FOTA_PROGRESS, // FOTA progress updates
    MSG_TYPE_ERROR_STATUS, // Error status updates from modules
    MSG_TYPE_SAVE_BHI360_CALIBRATION, // Save BHI360 calibration data
    MSG_TYPE_LOAD_BHI360_CALIBRATION, // Load BHI360 calibration data
    MSG_TYPE_TRIGGER_BHI360_CALIBRATION, // Trigger BHI360 calibration
    MSG_TYPE_REQUEST_BHI360_CALIBRATION, // Request calibration data from data module
    MSG_TYPE_BHI360_CALIBRATION_DATA, // Calibration data response from data module
    MSG_TYPE_DEVICE_INFO, // Device information for D2D sharing
    MSG_TYPE_SENSOR_DATA_CONSOLIDATED, // Consolidated sensor data from sensor_data module
    MSG_TYPE_REALTIME_METRICS, // Real-time metrics from realtime_metrics module
    MSG_TYPE_ANALYTICS_RESULTS, // Analytics results from analytics module
    MSG_TYPE_ACTIVITY_METRICS_BLE, // Activity metrics for BLE transmission
    MSG_TYPE_SAVE_WEIGHT_CALIBRATION, // Save weight calibration data to storage
    MSG_TYPE_REQUEST_WEIGHT_CALIBRATION, // Request weight calibration from storage
    MSG_TYPE_WEIGHT_CALIBRATION_DATA, // Weight calibration data response
    MSG_TYPE_START_WEIGHT_CALIBRATION, // Start weight calibration procedure
    MSG_TYPE_WEIGHT_MEASUREMENT, // Weight measurement result
    MSG_TYPE_GPS_UPDATE, // GPS update from mobile app
    MSG_TYPE_FULL_SYNC_DATA,
    MSG_TYPE_SYNC_FOOT_DATA,
    MSG_TYPE_BATTERY_LEVEL_PRIMARY, // Battery level notification
    MSG_TYPE_BATTERY_LEVEL_SECONDARY, // Battery level notification
} msg_type_t;

// Define sender types
typedef enum
{
    SENDER_NONE,
    SENDER_FOOT_SENSOR_THREAD,
    SENDER_BHI360_THREAD,
    SENDER_BTH, // Bluetooth
    SENDER_DATA,
    SENDER_D2D_SECONDARY, // D2D data from secondary device
    SENDER_MOTION_SENSOR, // Motion sensor module
    SENDER_WIFI, // WiFi module
    SENDER_ACTIVITY_METRICS, // Activity metrics module
    SENDER_SENSOR_DATA, // Sensor data consolidation module
    SENDER_REALTIME_METRICS, // Real-time metrics module
    SENDER_ANALYTICS, // Analytics module
    SENDER_BATTTERY, // Analytics module
} sender_type_t;
#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_APP_EVENT_H_

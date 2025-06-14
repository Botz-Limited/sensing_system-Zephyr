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
    MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE,
    MSG_TYPE_NEW_BHI360_LOG_FILE,
    MSG_TYPE_COMMAND, // Generic command string
    MSG_TYPE_DELETE_FOOT_LOG, // Specific message type for deleting foot sensor logs
    MSG_TYPE_DELETE_BHI360_LOG, // Specific message type for deleting BHI360 logs
} msg_type_t;

// Define sender types
typedef enum
{
    SENDER_NONE,
    SENDER_FOOT_SENSOR_THREAD,
    SENDER_BHI360_THREAD,
    SENDER_BTH, // Bluetooth
    SENDER_DATA,
} sender_type_t;
#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_APP_EVENT_H_

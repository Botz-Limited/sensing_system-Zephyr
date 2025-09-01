#ifndef APP_INCLUDE_STREAMING_CONTROL_EVENT_H_
#define APP_INCLUDE_STREAMING_CONTROL_EVENT_H_

#include <app_event_manager.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Streaming control event for enabling/disabling BLE data streaming
struct streaming_control_event
{
    struct app_event_header header;
    bool foot_sensor_streaming_enabled;
    bool quaternion_streaming_enabled;
};

APP_EVENT_TYPE_DECLARE(streaming_control_event);

#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_STREAMING_CONTROL_EVENT_H_
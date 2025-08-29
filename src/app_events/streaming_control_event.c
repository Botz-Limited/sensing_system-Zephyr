#include <events/streaming_control_event.h>

static void log_streaming_control_event(const struct app_event_header *aeh)
{
    struct streaming_control_event *event = cast_streaming_control_event(aeh);
    
    APP_EVENT_MANAGER_LOG(aeh, "foot_streaming=%d, quat_streaming=%d", 
                          event->foot_sensor_streaming_enabled,
                          event->quaternion_streaming_enabled);
}

APP_EVENT_TYPE_DEFINE(streaming_control_event,
                      log_streaming_control_event,
                      NULL,
                      APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE));
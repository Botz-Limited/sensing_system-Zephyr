#include <events/foot_sensor_event.h>

static void log_foot_sensor_stop_activity_event(const struct app_event_header *aeh)
{
    /* No additional data to log for stop activity event */
    APP_EVENT_MANAGER_LOG(aeh, "Foot sensor stop activity trigger");
}

APP_EVENT_TYPE_DEFINE(foot_sensor_stop_activity_event,                                       /* Unique event name. */
                      log_foot_sensor_stop_activity_event,                                   /* Function logging event data. */
                      NULL,                                                                  /* No event info provided. */
                      APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE));         /* Flags managing event type. */

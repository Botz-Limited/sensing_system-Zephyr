#include <events/foot_sensor_event.h>

const char *foot_sensor_state_to_str[] = {
                                        [FOOT_SENSOR_STATE_POWER_OFF] = "FootSensor power off"
};

static void log_foot_sensor_state_event(const struct app_event_header *aeh)
{
    struct foot_sensor_state_event *event = cast_foot_sensor_state_event(aeh);

    APP_EVENT_MANAGER_LOG(aeh, "state=%s", foot_sensor_state_to_str[event->state]);
}

APP_EVENT_TYPE_DEFINE(foot_sensor_state_event,                                       /* Unique event name. */
                      log_foot_sensor_state_event,                                   /* Function logging event data. */
                      NULL,                                                          /* No event info provided. */
                      APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE)); /* Flags managing event type. */

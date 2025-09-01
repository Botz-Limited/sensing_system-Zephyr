#ifndef APP_INCLUDE_FOOT_SENSOR_STATE_EVENT_H_
#define APP_INCLUDE_FOOT_SENSOR_STATE_EVENT_H_

#include <app_event_manager.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern const char *foot_sensor_state_to_str[];

enum foot_sensor_state_t
{
    FOOT_SENSOR_STATE_POWER_OFF,
};

struct foot_sensor_state_event
{
    struct app_event_header header;

    enum foot_sensor_state_t state;
};

APP_EVENT_TYPE_DECLARE(foot_sensor_state_event);

// Event for foot sensor start activity trigger
struct foot_sensor_start_activity_event {
    struct app_event_header header;
};
APP_EVENT_TYPE_DECLARE(foot_sensor_start_activity_event);

// Event for foot sensor stop activity trigger
struct foot_sensor_stop_activity_event {
    struct app_event_header header;
};
APP_EVENT_TYPE_DECLARE(foot_sensor_stop_activity_event);

#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_FOOT_SENSOR_STATE_EVENT_H_

#ifndef APP_INCLUDE_MOTION_SENSOR_STATE_EVENT_H_
#define APP_INCLUDE_MOTION_SENSOR_STATE_EVENT_H_

#include <app_event_manager.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern const char *motion_sensor_state_to_str[];

enum motion_sensor_state_t
{
    MOTION_SENSOR_STATE_POWER_OFF,
};

struct motion_sensor_state_event
{
    struct app_event_header header;

    enum motion_sensor_state_t state;
};

APP_EVENT_TYPE_DECLARE(motion_sensor_state_event);

// Event for motion sensor start activity trigger
struct motion_sensor_start_activity_event {
    struct app_event_header header;
};
APP_EVENT_TYPE_DECLARE(motion_sensor_start_activity_event);

// Event for motion sensor stop activity trigger
struct motion_sensor_stop_activity_event {
    struct app_event_header header;
};
APP_EVENT_TYPE_DECLARE(motion_sensor_stop_activity_event);

#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_MOTION_SENSOR_STATE_EVENT_H_

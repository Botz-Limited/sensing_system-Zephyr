#ifndef APP_INCLUDE_IDLE_EVENT_H_
#define APP_INCLUDE_IDLE_EVENT_H_

#include <app_event_manager.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Idle state event - signals system entering/exiting idle state
 */
struct idle_state_event {
    struct app_event_header header;
    bool entering_idle;  // true = entering idle, false = exiting idle
};
APP_EVENT_TYPE_DECLARE(idle_state_event);

/**
 * @brief Sleep state event - signals system entering/exiting sleep state
 */
struct sleep_state_event {
    struct app_event_header header;
    bool entering_sleep;  // true = entering sleep, false = waking up
};
APP_EVENT_TYPE_DECLARE(sleep_state_event);

/**
 * @brief Motion activity detected event - signals BHI360 activity
 */
struct motion_activity_event {
    struct app_event_header header;
    uint32_t timestamp_ms;  // Timestamp of last activity
};
APP_EVENT_TYPE_DECLARE(motion_activity_event);

#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_IDLE_EVENT_H_
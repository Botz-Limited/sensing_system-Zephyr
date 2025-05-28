#ifndef APP_INCLUDE_APP_EVENT_H_
#define APP_INCLUDE_APP_EVENT_H_

#include <app_event_manager.h>

#ifdef __cplusplus
extern "C"
{
#endif

struct app_state_event {
    struct app_event_header header;

    /* Custom data fields. */
    const char *state;
    const char *exit_state;
};

APP_EVENT_TYPE_DECLARE(app_state_event);

#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_APP_EVENT_H_
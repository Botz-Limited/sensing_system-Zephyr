#include <events/app_state_event.h>

static void log_app_state_event(const struct app_event_header *aeh)
{
    struct app_state_event *event = cast_app_state_event(aeh);

    APP_EVENT_MANAGER_LOG(aeh, "entering state = %s, exiting state = %s", event->state, event->exit_state);
}

APP_EVENT_TYPE_DEFINE(app_state_event,                                                /* Unique event name. */
                      log_app_state_event,                                            /* Function logging event data. */
                      NULL,                                                           /* No event info provided. */
                      APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE)); /* Flags managing event type. */
#include <events/app_state_event.h>

const char *app_state_to_str[] = {[APP_STATE_UNKNOWN] = "Advertising failed",
                                        [APP_STATE_INIT] = "Advertising",
                                        [APP_STATE_READY] = "Connected",
                                        [APP_STATE_SHUTDOWN_PREPARING] = "Timed Out",
                                        [APP_STATE_SHUTDOWN] = "Disconnected",
                                        [APP_STATE_ERROR] = "Bluetooth paired"
};

static void log_app_state_event(const struct app_event_header *aeh)
{
    struct app_state_event *event = cast_app_state_event(aeh);

    APP_EVENT_MANAGER_LOG(aeh, "state = %s", app_state_to_str[event->state]);
}

APP_EVENT_TYPE_DEFINE(app_state_event,                                                /* Unique event name. */
                      log_app_state_event,                                            /* Function logging event data. */
                      NULL,                                                           /* No event info provided. */
                      APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE)); /* Flags managing event type. */
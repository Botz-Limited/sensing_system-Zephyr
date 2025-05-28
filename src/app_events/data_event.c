#include <events/data_event.h>

const char *data_to_str[] = {
    [DATA_ERROR] = "Data Module Error",
    [PROTO_ERROR] = "Protobuf Error",
    [DATA_NO_ERROR] = "File System no error",
    [DATA_LOG_COMPLETE] = "File System closed logging",
    [DATA_FACTORY_RESET_COMPLETE] = "Data has erased filesystem",
};

static void log_data_event(const struct app_event_header *aeh)
{
    struct data_event *event = cast_data_event(aeh);

    APP_EVENT_MANAGER_LOG(aeh, "state=%s", data_to_str[event->data_state]);
}

APP_EVENT_TYPE_DEFINE(data_event,                                                    /* Unique event name. */
                      log_data_event,                                                /* Function logging event data. */
                      NULL,                                                          /* No event info provided. */
                      APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE)); /* Flags managing event type. */


const char *calibration_to_str[] = {
    [CALIBRATION_NO_ERROR] = "Calibration no Error",
    [CALIBRATION_ERROR] = "Calibration Error",
};

static void log_calibration_event(const struct app_event_header *aeh)
{
    struct calibration_event *event = cast_calibration_event(aeh);

    APP_EVENT_MANAGER_LOG(aeh, "state=%s", calibration_to_str[event->state]);
}

APP_EVENT_TYPE_DEFINE(calibration_event,                                                    /* Unique event name. */
                      log_calibration_event,                                                /* Function logging event data. */
                      NULL,                                                          /* No event info provided. */
                      APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE)); /* Flags managing event type. */

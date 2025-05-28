#ifndef APP_INCLUDE_DATA_EVENT_H_
#define APP_INCLUDE_DATA_EVENT_H_

#include <app_event_manager.h>

#ifdef __cplusplus
extern "C"
{
#endif

    enum data_state_t
    {
        DATA_ERROR,
        PROTO_ERROR,
        DATA_NO_ERROR,
        DATA_LOG_COMPLETE,
        DATA_FACTORY_RESET_COMPLETE,
    };

    extern const char *data_to_str[];

    struct data_event
    {
        struct app_event_header header;

        enum data_state_t data_state;
    };

    APP_EVENT_TYPE_DECLARE(data_event);

    enum calibration_state_t
    {
        CALIBRATION_NO_ERROR,
        CALIBRATION_ERROR,
    };

    extern const char *calibration_to_str[];

    struct calibration_event
    {
        struct app_event_header header;
        enum calibration_state_t state;
    };

    APP_EVENT_TYPE_DECLARE(calibration_event);

#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_DATA_EVENT_H_

#ifndef APP_INCLUDE_RECORD_ID_EVENT_H_
#define APP_INCLUDE_RECORD_ID_EVENT_H_

#include <app_event_manager.h>



#ifdef __cplusplus
extern "C"
{
#endif

extern const char *record_type_to_str[];

enum record_type_t
{
    RECORD_METADATA = 0,
    RECORD_HARDWARE,
};

struct record_id_event
{
    struct app_event_header header;
    uint32_t id;
    char path[CONFIG_MAX_PATH_LEN];
    enum record_type_t record_type;
};

APP_EVENT_TYPE_DECLARE(record_id_event);

#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_RECORD_ID_EVENT_H_

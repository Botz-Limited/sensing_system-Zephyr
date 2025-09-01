#ifndef APP_INCLUDE_BLUETOOTH_STATE_EVENT_H_
#define APP_INCLUDE_BLUETOOTH_STATE_EVENT_H_

#include <app_event_manager.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern const char *bluetooth_state_to_str[];


enum bluetooth_state_t
{
    BLE_STATE_FAILED_ADVERTISING = 0,
    BLE_STATE_ADVERTISING,
    BLE_STATE_CONNECTED,
    BLE_STATE_TIMEOUT,
    BLE_STATE_DISCONNECTED,
    BLE_SATE_PAIRED,
    BLE_STATE_NOT_PAIRED,
    BLE_STATE_POWER_OFF,
};

struct bluetooth_state_event
{
    struct app_event_header header;

    enum bluetooth_state_t state;
};

APP_EVENT_TYPE_DECLARE(bluetooth_state_event);

#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_BLUETOOTH_STATE_EVENT_H_

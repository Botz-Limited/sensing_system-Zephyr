#include <events/bluetooth_state_event.h>

const char *bluetooth_state_to_str[] = {[BLE_STATE_FAILED_ADVERTISING] = "Advertising failed",
                                        [BLE_STATE_ADVERTISING] = "Advertising",
                                        [BLE_STATE_CONNECTED] = "Connected",
                                        [BLE_STATE_TIMEOUT] = "Timed Out",
                                        [BLE_STATE_DISCONNECTED] = "Disconnected",
                                        [BLE_SATE_PAIRED] = "Bluetooth paired",
                                        [BLE_STATE_NOT_PAIRED] = "Bluetooth not paired",
                                        [BLE_STATE_POWER_OFF] = "Bluetooth power off"
};

static void log_bluetooth_state_event(const struct app_event_header *aeh)
{
    struct bluetooth_state_event *event = cast_bluetooth_state_event(aeh);

    APP_EVENT_MANAGER_LOG(aeh, "state=%s", bluetooth_state_to_str[event->state]);
}

APP_EVENT_TYPE_DEFINE(bluetooth_state_event,                                             /* Unique event name. */
                      log_bluetooth_state_event,                                         /* Function logging event data. */
                      NULL,                                                          /* No event info provided. */
                      APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE)); /* Flags managing event type. */

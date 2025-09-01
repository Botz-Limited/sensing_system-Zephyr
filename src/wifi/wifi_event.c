/**
 * @file wifi_event.c
 * @brief WiFi event implementation
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 *
 */

#include <events/wifi_event.h>

APP_EVENT_TYPE_DEFINE(wifi_connected_event,
                     NULL,
                     NULL,
                     APP_EVENT_FLAGS_CREATE());

APP_EVENT_TYPE_DEFINE(wifi_disconnected_event,
                     NULL,
                     NULL,
                     APP_EVENT_FLAGS_CREATE());
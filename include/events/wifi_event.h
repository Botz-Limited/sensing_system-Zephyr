/**
 * @file wifi_event.h
 * @brief WiFi module events
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 *
 */

#ifndef WIFI_EVENT_H_
#define WIFI_EVENT_H_

#include <app_event_manager.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi connected event
 * 
 * This event is sent when WiFi (nRF7002) is connected and ready to receive data
 */
struct wifi_connected_event {
    struct app_event_header header;
};

/**
 * @brief WiFi disconnected event
 * 
 * This event is sent when WiFi (nRF7002) is disconnected
 */
struct wifi_disconnected_event {
    struct app_event_header header;
};

APP_EVENT_TYPE_DECLARE(wifi_connected_event);
APP_EVENT_TYPE_DECLARE(wifi_disconnected_event);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_EVENT_H_ */
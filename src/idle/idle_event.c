/**
 * @file idle_event.c
 * @brief Implementation of idle/sleep power management events
 * @version 1.0
 * @date August 2025
 *
 * @copyright Botz Innovation 2025
 */

#include <events/idle_event.h>

APP_EVENT_TYPE_DEFINE(idle_state_event,
                     NULL,
                     NULL,
                     APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE));

APP_EVENT_TYPE_DEFINE(sleep_state_event,
                     NULL,
                     NULL,
                     APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE));

APP_EVENT_TYPE_DEFINE(motion_activity_event,
                     NULL,
                     NULL,
                     APP_EVENT_FLAGS_CREATE(APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE));
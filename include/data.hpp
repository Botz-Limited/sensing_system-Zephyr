/**
 * @file data.hpp
 * @brief
 * @version 1.0
 * @date 15/5/2025
 *
 * @copyright Botz Innovation 2025
 *
 */
#ifndef APP_INCLUDE_DATA_HEADER_
#define APP_INCLUDE_DATA_HEADER_

#include <stdint.h>
#include <zephyr/kernel.h>

#include <events/data_event.h>

#define data_wait_timer 2000

static inline void set_data_event(data_state_t new_evt)
{
    static data_state_t prev_evt = DATA_NO_ERROR;

    if(prev_evt != new_evt)
    {
        auto *event = new_data_event();
        event->data_state = new_evt;
        prev_evt = new_evt;
        APP_EVENT_SUBMIT(event);
    }
}

#endif // APP_INCLUDE_DATA_HEADER_

/**
 * @file app.hpp
 * @brief
 * @version 1.0
 * @date 15/5/2025
 *
 * @copyright Botz Innovation 2025
 *
 */
#ifndef APP_INCLUDE_APP_HEADER_
#define APP_INCLUDE_APP_HEADER_

#include <zephyr/kernel.h>
#include <events/data_event.h>
#include <events/bluetooth_state_event.h>
#include <events/data_event.h>

void app_log(const char *fmt, ...);

#endif // APP_INCLUDE_APP_HEADER_

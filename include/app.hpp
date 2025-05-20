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

#include <stdint.h>
#include <zephyr/kernel.h>

#define app_wait_timer 2000

#ifdef __cplusplus
extern "C"
{
#endif

int app_init(void);

#ifdef __cplusplus
}
#endif

#endif // APP_INCLUDE_APP_HEADER_

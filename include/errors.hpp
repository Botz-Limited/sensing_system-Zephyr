/**
 * @file errors.hpp
 * @author Nancy Jayakumar (nancy.jayakumar@chiaro.co.uk)
 * @brief
 * @version 2.8.4
 * @date 2024-11-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef APP_INCLUDE_ERRORS_H_
#define APP_INCLUDE_ERRORS_H_

#include <stdint.h>

enum class err_t
{
    NO_ERROR = 0,
    BATTERY_FAULT = -1,
    BLUETOOTH_ERROR = -5,
    HARDWARE = -15,
    DATA_ERROR = -16,
    DFU_ERROR = -17,
    ADC_ERROR = -19,
    I2C_ERROR = -20,
    BATTERY_DISCONNECTION_ERROR = -21,
    MOTION_ERROR = -22,
    RTC_ERROR = -23,

    // File related errors continue from here
    FILE_SYSTEM_ERROR = -24,
    PROTO_ENCODE_ERROR = -25,
    FILE_SYSTEM_NO_FILES = -26,
    FILE_SYSTEM_STORAGE_FULL = -27,
    FLASH_FAILURE = -28,
    INVALID_PARAMETER = -29,
    QUEUE_FULL = -30,
    WIFI_ERROR = -31,
};

#endif // APP_INCLUDE_ERRORS_H_

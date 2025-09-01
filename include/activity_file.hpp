/**
 * @file activity_file.hpp
 * @brief Activity file management for calculated metrics during activity sessions
 * @version 1.0
 * @date 2025-06-21
 *
 * @copyright Botz Innovation 2025
 *
 */
#ifndef APP_INCLUDE_ACTIVITY_FILE_HEADER_
#define APP_INCLUDE_ACTIVITY_FILE_HEADER_

#include <stdint.h>
#include <zephyr/kernel.h>

/**
 * @brief Activity file header structure
 * Written once at the beginning of each activity file
 */
typedef struct __packed {
    uint32_t timestamp;         // Unix epoch time when file was created
    uint8_t  file_version;      // File format version (1)
    uint8_t  firmware_version[3]; // Major.Minor.Patch
    uint16_t reserved;          // Reserved for future use
} activity_file_header_t;

/**
 * @brief Activity sample structure
 * Written periodically during activity
 */
typedef struct __packed {
    uint16_t delta_time_ms;     // Time since last sample (milliseconds)
    uint16_t left_step_count;   // Left foot step count
    uint16_t right_step_count;  // Right foot step count
    uint16_t reserved;          // Reserved for future metrics
} activity_sample_t;

/**
 * @brief Activity file constants
 */
static constexpr uint8_t ACTIVITY_FILE_VERSION = 1;
static constexpr uint16_t ACTIVITY_SAMPLE_SIZE = sizeof(activity_sample_t);
static constexpr uint16_t ACTIVITY_HEADER_SIZE = sizeof(activity_file_header_t);

#endif // APP_INCLUDE_ACTIVITY_FILE_HEADER_
/**
 * @file app_step_count.hpp
 * @brief Step count data structures without time information
 * @version 1.0
 * @date 2025-01-15
 *
 * @copyright Botz Innovation 2025
 *
 * This file defines step count structures that only contain step counts,
 * without any time or duration information. Time-based metrics should be
 * calculated using separate time characteristics.
 */

#ifndef APP_INCLUDE_APP_STEP_COUNT_HEADER_
#define APP_INCLUDE_APP_STEP_COUNT_HEADER_

#include <cstdint>

// Simple step count structure (no time information)
typedef struct
{
    uint32_t step_count;
} step_count_only_t;  // Size: 4 bytes

// Activity step count with left/right separation
typedef struct
{
    uint32_t left_step_count;   // Left foot step count
    uint32_t right_step_count;  // Right foot step count
} activity_step_count_detailed_t;  // Size: 8 bytes

// Global vs Activity step tracking
typedef struct
{
    uint32_t global_step_count;    // Total steps since device boot
    uint32_t activity_step_count;  // Steps during current activity
} step_count_tracking_t;  // Size: 8 bytes

#endif // APP_INCLUDE_APP_STEP_COUNT_HEADER_
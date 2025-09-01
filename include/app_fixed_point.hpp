/**
 * @file app_fixed_point.hpp
 * @brief Fixed-point data structures for efficient BLE transmission
 * @version 1.0
 * @date 2025-01-15
 *
 * @copyright Botz Innovation 2025
 *
 * This file defines fixed-point integer versions of sensor data structures
 * to avoid sending floats over BLE, which is inefficient and not portable.
 */

#ifndef APP_INCLUDE_APP_FIXED_POINT_HEADER_
#define APP_INCLUDE_APP_FIXED_POINT_HEADER_

#include <cstdint>
#include "app.hpp"  // For bhi360_3d_mapping_t, bhi360_linear_accel_t, etc.

// Scaling factors for fixed-point conversion
namespace FixedPoint {
    // Quaternion components are normalized [-1, 1], scale by 10000 for 4 decimal places
    static constexpr int32_t QUAT_SCALE = 10000;
    
    // Linear acceleration in m/s², scale by 1000 for 3 decimal places (mm/s²)
    static constexpr int32_t ACCEL_SCALE = 1000;
    
    // Gyroscope in rad/s, scale by 10000 for 4 decimal places
    static constexpr int32_t GYRO_SCALE = 10000;
    
    // Quaternion accuracy [0-3], scale by 100 for percentage
    static constexpr int32_t ACCURACY_SCALE = 100;
}

// Fixed-point structures for BLE transmission
typedef struct
{
    // Quaternion components scaled by 10000 (range: -10000 to 10000)
    int16_t quat_x;  // Real range: -1.0000 to 1.0000
    int16_t quat_y;
    int16_t quat_z;
    int16_t quat_w;
    
    // Gyroscope scaled by 10000 (typical range: -20000 to 20000 for ±2 rad/s)
    int16_t gyro_x;  // Real range: ±2.0000 rad/s typical
    int16_t gyro_y;
    int16_t gyro_z;
    
    // Quaternion accuracy scaled by 100 (range: 0 to 300)
    uint8_t quat_accuracy;  // Real range: 0.00 to 3.00
} bhi360_3d_mapping_fixed_t;  // Size: 15 bytes (vs 28 bytes for float version)

typedef struct
{
    // Linear acceleration scaled by 1000 (typical range: -20000 to 20000 for ±20 m/s²)
    int16_t x;  // Real range: ±20.000 m/s² typical
    int16_t y;
    int16_t z;
} bhi360_linear_accel_fixed_t;  // Size: 6 bytes (vs 12 bytes for float version)

// Step count remains the same (already using integers)
typedef struct
{
    uint32_t step_count;
    uint32_t activity_duration_s;
} bhi360_step_count_fixed_t;  // Size: 8 bytes (same as original)

// For logging, we might want higher precision
typedef struct
{
    // Quaternion with higher precision for logging (scaled by 100000)
    int32_t quat_x;  // 4 bytes each, 5 decimal places
    int32_t quat_y;
    int32_t quat_z;
    int32_t quat_w;
    int16_t quat_accuracy;  // Scaled by 100
    
    // Linear acceleration scaled by 10000 for logging (4 decimal places)
    int32_t lacc_x;  // Real range: ±20.0000 m/s²
    int32_t lacc_y;
    int32_t lacc_z;
    
    // Gyroscope scaled by 100000 for logging (5 decimal places)
    int32_t gyro_x;  // Real range: ±2.00000 rad/s
    int32_t gyro_y;
    int32_t gyro_z;
    
    uint32_t step_count;
    uint64_t timestamp;
} bhi360_log_record_fixed_t;  // Size: 58 bytes (vs 60 bytes for float version)

// Conversion functions
inline int16_t float_to_fixed16(float value, int32_t scale) {
    return static_cast<int16_t>(value * scale);
}

inline int32_t float_to_fixed32(float value, int32_t scale) {
    return static_cast<int32_t>(value * scale);
}

inline float fixed16_to_float(int16_t value, int32_t scale) {
    return static_cast<float>(value) / scale;
}

inline float fixed32_to_float(int32_t value, int32_t scale) {
    return static_cast<float>(value) / scale;
}

// Conversion helpers for complete structures
inline void convert_3d_mapping_to_fixed(const bhi360_3d_mapping_t& src, bhi360_3d_mapping_fixed_t& dst) {
    // Note: src.accel_x/y/z are actually quaternion x/y/z in current implementation
    dst.quat_x = float_to_fixed16(src.accel_x, FixedPoint::QUAT_SCALE);
    dst.quat_y = float_to_fixed16(src.accel_y, FixedPoint::QUAT_SCALE);
    dst.quat_z = float_to_fixed16(src.accel_z, FixedPoint::QUAT_SCALE);
    dst.quat_w = float_to_fixed16(src.quat_w, FixedPoint::QUAT_SCALE);
    dst.gyro_x = float_to_fixed16(src.gyro_x, FixedPoint::GYRO_SCALE);
    dst.gyro_y = float_to_fixed16(src.gyro_y, FixedPoint::GYRO_SCALE);
    dst.gyro_z = float_to_fixed16(src.gyro_z, FixedPoint::GYRO_SCALE);
    // Accuracy is not available in the current structure, set to 0 for now
    // This would need to be passed separately if needed
    dst.quat_accuracy = 0;
}

// Overloaded version that accepts accuracy
inline void convert_3d_mapping_to_fixed(const bhi360_3d_mapping_t& src, float quat_accuracy, bhi360_3d_mapping_fixed_t& dst) {
    convert_3d_mapping_to_fixed(src, dst);
    dst.quat_accuracy = static_cast<uint8_t>(quat_accuracy * FixedPoint::ACCURACY_SCALE);
}

inline void convert_linear_accel_to_fixed(const bhi360_linear_accel_t& src, bhi360_linear_accel_fixed_t& dst) {
    dst.x = float_to_fixed16(src.x, FixedPoint::ACCEL_SCALE);
    dst.y = float_to_fixed16(src.y, FixedPoint::ACCEL_SCALE);
    dst.z = float_to_fixed16(src.z, FixedPoint::ACCEL_SCALE);
}

#endif // APP_INCLUDE_APP_FIXED_POINT_HEADER_
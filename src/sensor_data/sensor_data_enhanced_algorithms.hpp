/**
 * @file sensor_data_enhanced_algorithms.h
 * @brief Enhanced algorithms that combine IMU and pressure data for better accuracy
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 * 
 * These algorithms use sensor fusion to provide more accurate metrics
 */

#ifndef SENSOR_DATA_ENHANCED_ALGORITHMS_HPP
#define SENSOR_DATA_ENHANCED_ALGORITHMS_HPP

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "sensor_data_fast_processing.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// Sensor layout for 8-channel pressure (typical insole configuration)
// 0: Heel lateral, 1: Heel medial
// 2: Midfoot lateral, 3: Midfoot arch, 4: Midfoot medial
// 5: Forefoot lateral, 6: Forefoot central, 7: Big toe/medial

#define SENSOR_HEEL_LATERAL 0
#define SENSOR_HEEL_MEDIAL 1
#define SENSOR_MID_LATERAL 2
#define SENSOR_MID_ARCH 3
#define SENSOR_MID_MEDIAL 4
#define SENSOR_FORE_LATERAL 5
#define SENSOR_FORE_CENTRAL 6
#define SENSOR_FORE_MEDIAL 7

/**
 * @brief Enhanced pronation calculation using IMU + pressure validation
 * 
 * According to the Activity Session spec, pronation should be validated with pressure data
 * for better accuracy. This combines IMU roll angle with medial/lateral pressure shift.
 * 
 * @param quaternion IMU quaternion [w, x, y, z]
 * @param pressure Array of 8 pressure sensor values
 * @param in_contact Whether foot is in contact with ground
 * @return Pronation angle in degrees (-45 to +45), positive = overpronation
 */
ALWAYS_INLINE int8_t calculate_pronation_enhanced(const float quaternion[4], 
                                                 const uint16_t pressure[8],
                                                 bool in_contact)
{
    // First get basic pronation from IMU
    int8_t imu_pronation = quick_pronation_check(quaternion);
    
    // If not in contact, return IMU-only estimate
    if (!in_contact) {
        return imu_pronation;
    }
    
    // Calculate medial vs lateral pressure distribution
    uint32_t lateral_pressure = pressure[SENSOR_HEEL_LATERAL] + 
                               pressure[SENSOR_MID_LATERAL] + 
                               pressure[SENSOR_FORE_LATERAL];
    
    uint32_t medial_pressure = pressure[SENSOR_HEEL_MEDIAL] + 
                              pressure[SENSOR_MID_MEDIAL] + 
                              pressure[SENSOR_FORE_MEDIAL];
    
    uint32_t total_pressure = lateral_pressure + medial_pressure;
    
    if (total_pressure == 0) {
        return imu_pronation;
    }
    
    // Calculate pressure ratio (0.0 = all lateral, 1.0 = all medial)
    float medial_ratio = (float)medial_pressure / (float)total_pressure;
    
    // Validate and adjust pronation based on pressure
    // Normal: 40-60% medial pressure
    // Overpronation: >60% medial pressure
    // Underpronation: <40% medial pressure
    
    int8_t pressure_adjustment = 0;
    
    if (medial_ratio > 0.6f) {
        // Pressure confirms overpronation
        pressure_adjustment = (int8_t)((medial_ratio - 0.6f) * 50.0f); // Up to +20°
        if (imu_pronation > 0) {
            // IMU and pressure agree on overpronation
            imu_pronation = (imu_pronation + pressure_adjustment > 45) ? 45 : 
                           (imu_pronation + pressure_adjustment);
        }
    } else if (medial_ratio < 0.4f) {
        // Pressure indicates underpronation
        pressure_adjustment = (int8_t)((0.4f - medial_ratio) * -50.0f); // Up to -20°
        if (imu_pronation < 0) {
            // IMU and pressure agree on underpronation
            imu_pronation = (imu_pronation + pressure_adjustment < -45) ? -45 : 
                           (imu_pronation + pressure_adjustment);
        }
    }
    
    return imu_pronation;
}

/**
 * @brief Calculate Center of Pressure (COP) coordinates
 * 
 * @param pressure Array of 8 pressure sensor values
 * @param cop_x Output: COP X coordinate in mm (lateral-medial, 0 = center)
 * @param cop_y Output: COP Y coordinate in mm (posterior-anterior, 0 = midfoot)
 * @return true if COP calculated successfully
 */
ALWAYS_INLINE bool calculate_center_of_pressure(const uint16_t pressure[8],
                                               int16_t *cop_x,
                                               int16_t *cop_y)
{
    // Sensor positions in mm (typical insole layout)
    // X: negative = lateral, positive = medial
    // Y: negative = heel, positive = forefoot
    static const int16_t sensor_x[8] = {-20, 20, -25, 0, 25, -20, 0, 20};
    static const int16_t sensor_y[8] = {-80, -80, -30, -30, -30, 40, 60, 80};
    
    uint32_t total_pressure = 0;
    int32_t weighted_x = 0;
    int32_t weighted_y = 0;
    
    // Calculate weighted average position
    for (int i = 0; i < 8; i++) {
        total_pressure += pressure[i];
        weighted_x += pressure[i] * sensor_x[i];
        weighted_y += pressure[i] * sensor_y[i];
    }
    
    if (total_pressure == 0) {
        *cop_x = 0;
        *cop_y = 0;
        return false;
    }
    
    *cop_x = (int16_t)(weighted_x / total_pressure);
    *cop_y = (int16_t)(weighted_y / total_pressure);
    
    return true;
}

/**
 * @brief Calculate loading rate using pressure and time
 * 
 * @param current_force Current total force
 * @param previous_force Previous total force
 * @param delta_time_ms Time between samples in milliseconds
 * @return Loading rate in N/s
 */
ALWAYS_INLINE uint16_t calculate_loading_rate(uint16_t current_force,
                                             uint16_t previous_force,
                                             uint16_t delta_time_ms)
{
    if (delta_time_ms == 0 || current_force <= previous_force) {
        return 0;
    }
    
    // Calculate rate: (force_change / time) * 1000 to get N/s
    uint32_t force_change = current_force - previous_force;
    uint32_t rate = (force_change * 1000) / delta_time_ms;
    
    return (rate > UINT16_MAX) ? UINT16_MAX : (uint16_t)rate;
}

/**
 * @brief Detect true flight time (both feet off ground)
 * 
 * @param left_contact Left foot contact state
 * @param right_contact Right foot contact state
 * @param delta_time_ms Time to add if both feet are off ground
 * @param accumulated_flight_time Running total of flight time
 * @return Updated flight time
 */
ALWAYS_INLINE uint16_t update_true_flight_time(bool left_contact,
                                               bool right_contact,
                                               uint16_t delta_time_ms,
                                               uint16_t accumulated_flight_time)
{
    if (!left_contact && !right_contact) {
        // Both feet off ground - true flight
        uint32_t new_time = accumulated_flight_time + delta_time_ms;
        return (new_time > UINT16_MAX) ? UINT16_MAX : (uint16_t)new_time;
    }
    
    // At least one foot on ground - reset
    return 0;
}

/**
 * @brief Calculate double support time (both feet on ground)
 * 
 * @param left_contact Left foot contact state
 * @param right_contact Right foot contact state
 * @param delta_time_ms Time to add if both feet are on ground
 * @param accumulated_double_time Running total of double support time
 * @return Updated double support time
 */
ALWAYS_INLINE uint16_t update_double_support_time(bool left_contact,
                                                  bool right_contact,
                                                  uint16_t delta_time_ms,
                                                  uint16_t accumulated_double_time)
{
    if (left_contact && right_contact) {
        // Both feet on ground - double support
        uint32_t new_time = accumulated_double_time + delta_time_ms;
        return (new_time > UINT16_MAX) ? UINT16_MAX : (uint16_t)new_time;
    }
    
    // At least one foot off ground - reset
    return 0;
}

/**
 * @brief Enhanced foot strike angle using IMU quaternion and pressure timing
 * 
 * @param quaternion IMU quaternion at moment of contact
 * @param heel_force Heel pressure at contact
 * @param fore_force Forefoot pressure at contact
 * @return Foot strike angle in degrees (positive = heel strike, negative = forefoot)
 */
ALWAYS_INLINE int8_t calculate_foot_strike_angle(const float quaternion[4],
                                                uint16_t heel_force,
                                                uint16_t fore_force)
{
    // Extract pitch angle from quaternion
    // Pitch = atan2(2*(w*y - x*z), 1 - 2*(y*y + z*z))
    float w = quaternion[0];
    float x = quaternion[1];
    float y = quaternion[2];
    float z = quaternion[3];
    
    // Simplified pitch calculation
    float pitch_rad = 2.0f * (w * y - x * z);
    int8_t pitch_deg = (int8_t)(pitch_rad * 57.2958f);
    
    // Validate with pressure - if forefoot hits first, angle should be negative
    if (fore_force > heel_force * 2) {
        // Clear forefoot strike
        if (pitch_deg > 0) pitch_deg = -pitch_deg;
    } else if (heel_force > fore_force * 2) {
        // Clear heel strike
        if (pitch_deg < 0) pitch_deg = -pitch_deg;
    }
    
    // Clamp to reasonable range
    if (pitch_deg > 30) return 30;
    if (pitch_deg < -30) return -30;
    
    return pitch_deg;
}

/**
 * @brief Calculate push-off power estimate
 * 
 * @param force Peak force during push-off
 * @param contact_time Contact time in milliseconds
 * @param body_weight User body weight in kg
 * @return Push-off power index (0-100)
 */
ALWAYS_INLINE uint8_t calculate_pushoff_power(uint16_t force,
                                             uint16_t contact_time,
                                             uint16_t body_weight)
{
    if (contact_time == 0 || body_weight == 0) {
        return 0;
    }
    
    // Normalize force by body weight
    uint32_t normalized_force = (force * 100) / (body_weight * 10); // Assuming force in N
    
    // Power index based on force and inverse of contact time
    // Shorter contact with higher force = more power
    uint32_t power_index = (normalized_force * 1000) / contact_time;
    
    // Scale to 0-100
    if (power_index > 200) power_index = 200;
    return (uint8_t)(power_index / 2);
}

/**
 * @brief Detect arch collapse using midfoot pressure
 * 
 * @param pressure Array of 8 pressure sensor values
 * @param phase Current contact phase
 * @return Arch collapse index (0-100, higher = more collapse)
 */
ALWAYS_INLINE uint8_t detect_arch_collapse(const uint16_t pressure[8],
                                          contact_phase_t phase)
{
    // Only meaningful during midstance
    if (phase != PHASE_MIDSTANCE && phase != PHASE_LOADING) {
        return 0;
    }
    
    // Check arch sensor pressure relative to total midfoot
    uint32_t arch_pressure = pressure[SENSOR_MID_ARCH];
    uint32_t total_midfoot = pressure[SENSOR_MID_LATERAL] + 
                            pressure[SENSOR_MID_ARCH] + 
                            pressure[SENSOR_MID_MEDIAL];
    
    if (total_midfoot == 0) {
        return 0;
    }
    
    // High arch pressure indicates collapse
    uint8_t arch_ratio = (uint8_t)((arch_pressure * 100) / total_midfoot);
    
    // Normal arch: 20-40% of midfoot pressure
    // Collapsed: >60% of midfoot pressure
    if (arch_ratio < 20) {
        return 0;  // High arch, no collapse
    } else if (arch_ratio > 60) {
        return 100;  // Severe collapse
    } else {
        // Linear scale from 20-60%
        return (uint8_t)((arch_ratio - 20) * 2.5f);
    }
}

#ifdef __cplusplus
}
#endif

#endif // SENSOR_DATA_ENHANCED_ALGORITHMS_HPP
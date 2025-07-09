/**
 * @file sensor_data_fast_processing.h
 * @brief Fast algorithms for 100Hz sensor data processing
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 * 
 * These algorithms are optimized for speed (<0.1ms each) to maintain 100Hz processing
 */

#ifndef SENSOR_DATA_FAST_PROCESSING_H
#define SENSOR_DATA_FAST_PROCESSING_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// Performance optimization hints
#ifndef ALWAYS_INLINE
#define ALWAYS_INLINE __attribute__((always_inline)) static inline
#endif
#define HOT_FUNCTION __attribute__((hot))
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

// Constants
#define CONTACT_THRESHOLD_N 50      // 50 Newtons minimum for ground contact
#define CONTACT_HYSTERESIS_N 10     // 10N hysteresis to prevent bouncing
#define NUM_PRESSURE_SENSORS 8      // 8 sensors per foot

// Contact phase definitions
typedef enum {
    PHASE_SWING = 0,        // Foot in air
    PHASE_HEEL_STRIKE,      // Initial contact
    PHASE_LOADING,          // Weight acceptance
    PHASE_MIDSTANCE,        // Full weight bearing
    PHASE_PUSH_OFF,         // Propulsion
    PHASE_TOE_OFF          // Final contact
} contact_phase_t;

// Pressure region indices
typedef enum {
    REGION_HEEL = 0,       // Sensors 0-1
    REGION_MIDFOOT,        // Sensors 2-4
    REGION_FOREFOOT        // Sensors 5-7
} pressure_region_t;

/**
 * @brief Fast ground contact detection with hysteresis
 * 
 * @param pressure Array of 8 pressure sensor values
 * @param previous_contact Previous contact state for hysteresis
 * @return true if foot is in contact with ground
 */
ALWAYS_INLINE bool detect_ground_contact(const uint16_t pressure[8], bool previous_contact)
{
    // Unrolled sum for speed
    uint32_t total_force = pressure[0] + pressure[1] + pressure[2] + pressure[3] +
                          pressure[4] + pressure[5] + pressure[6] + pressure[7];
    
    // Apply hysteresis to prevent bouncing
    if (LIKELY(previous_contact)) {
        // Currently in contact - need to drop below threshold minus hysteresis
        return total_force > (CONTACT_THRESHOLD_N - CONTACT_HYSTERESIS_N);
    } else {
        // Currently not in contact - need to exceed threshold plus hysteresis
        return total_force > (CONTACT_THRESHOLD_N + CONTACT_HYSTERESIS_N);
    }
}

/**
 * @brief Fast peak force detection
 * 
 * @param pressure Array of 8 pressure sensor values
 * @return Total force in arbitrary units (sum of all sensors)
 */
ALWAYS_INLINE uint16_t detect_peak_force(const uint16_t pressure[8])
{
    // Simple sum - let caller track the peak over time
    uint32_t total = pressure[0] + pressure[1] + pressure[2] + pressure[3] +
                     pressure[4] + pressure[5] + pressure[6] + pressure[7];
    
    // Saturate to uint16_t max
    return (total > UINT16_MAX) ? UINT16_MAX : (uint16_t)total;
}

/**
 * @brief Fast contact phase detection based on pressure distribution
 * 
 * @param pressure Array of 8 pressure sensor values
 * @param was_in_contact Previous contact state
 * @return Current contact phase
 */
ALWAYS_INLINE contact_phase_t detect_contact_phase(const uint16_t pressure[8], bool was_in_contact)
{
    // Calculate regional pressures
    uint32_t heel = pressure[0] + pressure[1];
    uint32_t midfoot = pressure[2] + pressure[3] + pressure[4];
    uint32_t forefoot = pressure[5] + pressure[6] + pressure[7];
    uint32_t total = heel + midfoot + forefoot;
    
    // No contact
    if (total < CONTACT_THRESHOLD_N) {
        return PHASE_SWING;
    }
    
    // Just made contact
    if (!was_in_contact) {
        return PHASE_HEEL_STRIKE;
    }
    
    // Determine phase by pressure distribution
    // Using percentages to avoid division
    uint32_t heel_pct = (heel * 100) / total;
    uint32_t forefoot_pct = (forefoot * 100) / total;
    
    if (heel_pct > 60) {
        return PHASE_LOADING;  // Heel dominant
    } else if (forefoot_pct > 60) {
        return PHASE_PUSH_OFF;  // Forefoot dominant
    } else {
        return PHASE_MIDSTANCE;  // Balanced distribution
    }
}

/**
 * @brief Quick pronation estimation from quaternion
 * 
 * @param quaternion Array of 4 quaternion values [w, x, y, z]
 * @return Pronation angle in degrees (-45 to +45)
 */
ALWAYS_INLINE int8_t quick_pronation_check(const float quaternion[4])
{
    // Extract roll angle from quaternion (simplified)
    // Roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    // Using small angle approximation for speed
    
    float w = quaternion[0];
    float x = quaternion[1];
    float y = quaternion[2];
    float z = quaternion[3];
    
    // Simplified roll calculation (good enough for ±45°)
    float roll_rad = 2.0f * (w * x + y * z);
    
    // Convert to degrees and clamp
    int16_t roll_deg = (int16_t)(roll_rad * 57.2958f);  // 180/PI
    
    // Clamp to int8_t range
    if (roll_deg > 45) return 45;
    if (roll_deg < -45) return -45;
    return (int8_t)roll_deg;
}

/**
 * @brief Calculate pressure distribution percentages
 * 
 * @param pressure Array of 8 pressure sensor values
 * @param heel_pct Output: heel pressure percentage (0-100)
 * @param mid_pct Output: midfoot pressure percentage (0-100)
 * @param fore_pct Output: forefoot pressure percentage (0-100)
 */
ALWAYS_INLINE void calculate_pressure_distribution(const uint16_t pressure[8],
                                                  uint8_t *heel_pct,
                                                  uint8_t *mid_pct,
                                                  uint8_t *fore_pct)
{
    uint32_t heel = pressure[0] + pressure[1];
    uint32_t mid = pressure[2] + pressure[3] + pressure[4];
    uint32_t fore = pressure[5] + pressure[6] + pressure[7];
    uint32_t total = heel + mid + fore;
    
    if (LIKELY(total > 0)) {
        // Calculate percentages without division
        *heel_pct = (uint8_t)((heel * 100) / total);
        *mid_pct = (uint8_t)((mid * 100) / total);
        *fore_pct = 100 - *heel_pct - *mid_pct;  // Ensure they sum to 100
    } else {
        *heel_pct = 0;
        *mid_pct = 0;
        *fore_pct = 0;
    }
}

/**
 * @brief Detect foot strike pattern from initial contact
 * 
 * @param heel_pct Heel pressure percentage at contact
 * @param mid_pct Midfoot pressure percentage at contact
 * @param fore_pct Forefoot pressure percentage at contact
 * @return 0=heel strike, 1=midfoot, 2=forefoot
 */
ALWAYS_INLINE uint8_t detect_strike_pattern(uint8_t heel_pct, uint8_t mid_pct, uint8_t fore_pct)
{
    (void)mid_pct;  // Currently unused but kept for API consistency
    
    if (heel_pct > 50) {
        return 0;  // Heel strike
    } else if (fore_pct > 50) {
        return 2;  // Forefoot strike
    } else {
        return 1;  // Midfoot strike
    }
}

#ifdef __cplusplus
}
#endif

#endif // SENSOR_DATA_FAST_PROCESSING_H
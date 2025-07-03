/**
 * @file sensor_data_consolidated.h
 * @brief Consolidated sensor data structure shared between modules
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#ifndef SENSOR_DATA_CONSOLIDATED_H
#define SENSOR_DATA_CONSOLIDATED_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Consolidated sensor data structure
typedef struct {
    // Timing - only delta time, no timestamps!
    uint16_t delta_time_ms;  // Time since last sample (typically 10ms for 100Hz)
    
    // Contact state
    uint8_t left_contact_phase;
    uint8_t right_contact_phase;
    bool left_in_contact;
    bool right_in_contact;
    
    // Contact duration (in ms) - only valid when transitioning out of contact
    uint16_t left_contact_duration_ms;
    uint16_t right_contact_duration_ms;
    
    // Forces and loading
    uint16_t left_peak_force;
    uint16_t right_peak_force;
    uint16_t left_loading_rate;  // N/s
    uint16_t right_loading_rate;  // N/s
    
    // Pressure distribution
    uint8_t left_heel_pct;
    uint8_t left_mid_pct;
    uint8_t left_fore_pct;
    uint8_t right_heel_pct;
    uint8_t right_mid_pct;
    uint8_t right_fore_pct;
    
    // Center of pressure
    int16_t left_cop_x;  // mm, lateral-medial
    int16_t left_cop_y;  // mm, posterior-anterior
    int16_t right_cop_x;
    int16_t right_cop_y;
    
    // IMU data
    float quaternion[4];
    float linear_acc[3];
    float gyro[3];
    
    // Enhanced biomechanics
    int8_t left_pronation_deg;   // Enhanced with pressure validation
    int8_t right_pronation_deg;
    int8_t left_strike_angle;    // Foot angle at contact
    int8_t right_strike_angle;
    uint8_t left_arch_collapse;  // 0-100 index
    uint8_t right_arch_collapse;
    
    // Bilateral timing
    uint16_t true_flight_time_ms;    // Both feet off ground
    uint16_t double_support_time_ms; // Both feet on ground
    
    // Step count
    uint32_t step_count;
    uint8_t step_count_delta;  // Steps since last sample (usually 0-2)
} sensor_data_consolidated_t;

#ifdef __cplusplus
}
#endif

#endif // SENSOR_DATA_CONSOLIDATED_H
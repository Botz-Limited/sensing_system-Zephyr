/**
 * @file d2d_metrics_fixed.h
 * @brief D2D Calculated Metrics with Fixed-Point Encoding
 * 
 * This file defines a compact fixed-point structure for transmitting calculated metrics
 * from secondary to primary device via D2D communication.
 * Uses fixed-point arithmetic to reduce packet size from 134 to ~80 bytes.
 */

#ifndef D2D_METRICS_FIXED_H
#define D2D_METRICS_FIXED_H

#include <stdint.h>
#include "d2d_metrics.h"  // For metric indices

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Fixed-point scaling factors for different metric types
 * These define the precision for each metric type
 */
#define FP_SCALE_TIME_MS      1      // Time in ms (no scaling needed)
#define FP_SCALE_LENGTH_CM    10     // Length in 0.1 cm units
#define FP_SCALE_ANGLE_DEG    10     // Angles in 0.1 degree units
#define FP_SCALE_PRESSURE     100    // Pressure in 0.01 kPa units
#define FP_SCALE_VELOCITY     100    // Velocity in 0.01 m/s units
#define FP_SCALE_PERCENT      10     // Percentages in 0.1% units
#define FP_SCALE_FORCE        100    // Force/impact in 0.01 g units
#define FP_SCALE_FREQUENCY    100    // Frequency in 0.01 Hz units
#define FP_SCALE_POWER        10     // Power in 0.1 W units
#define FP_SCALE_ENERGY       10     // Energy in 0.1 J units

/**
 * @brief Compact D2D Metrics Packet Structure with Fixed-Point
 * 
 * Uses int16_t for most metrics to reduce size.
 * Total size: ~80 bytes (fits in 121 byte MTU with room for headers)
 */
typedef struct __attribute__((packed)) {
    // Basic gait metrics (6 metrics * 2 bytes = 12 bytes)
    int16_t gct;                    // Ground contact time (ms)
    int16_t stride_length;          // Stride length (0.1 cm units)
    int16_t cadence;                // Cadence (steps/min)
    int16_t pronation;              // Pronation angle (0.1 degrees)
    int16_t stance_time;            // Stance time (ms)
    int16_t swing_time;             // Swing time (ms)
    
    // Pressure metrics (4 metrics * 2 bytes = 8 bytes)
    int16_t peak_pressure;          // Peak pressure (0.01 kPa units)
    int16_t cop_x;                  // Center of pressure X (0.1 mm units, range ±3276.7mm)
    int16_t cop_y;                  // Center of pressure Y (0.1 mm units, range ±3276.7mm)
    int16_t pressure_integral;      // Pressure time integral (scaled)
    
    // Kinematic metrics (4 metrics * 2 bytes = 8 bytes)
    int16_t vertical_osc;           // Vertical oscillation (0.1 cm units)
    int16_t flight_time;            // Flight time (ms)
    int16_t max_velocity;           // Max velocity (0.01 m/s units)
    int16_t foot_strike_angle;      // Foot strike angle (0.1 degrees)
    
    // Step metrics (4 metrics * 2 bytes = 8 bytes)
    uint16_t step_count;            // Step count since start
    int16_t step_frequency;         // Step frequency (0.01 Hz units)
    int16_t step_length;            // Step length (0.1 cm units)
    int16_t step_time;              // Step time (ms)
    
    // Energy metrics (3 metrics * 2 bytes = 6 bytes)
    int16_t power;                  // Power (0.1 W units)
    int16_t work;                   // Work (0.1 J units)
    int16_t efficiency;             // Efficiency (0.1% units)
    
    // Impact metrics (3 metrics * 2 bytes = 6 bytes)
    int16_t peak_impact;            // Peak impact (0.01 g units)
    int16_t loading_rate;           // Loading rate (0.01 g/s units)
    int16_t impact_duration;        // Impact duration (ms)
    
    // Balance metrics (3 metrics * 2 bytes = 6 bytes)
    int16_t balance_index;          // Balance index (0-1000 for 0-100%)
    int16_t ml_sway;                // Medial-lateral sway (mm)
    int16_t ap_sway;                // Anterior-posterior sway (mm)
    
    // Fatigue indicators (3 metrics * 2 bytes = 6 bytes)
    int16_t fatigue_index;          // Fatigue index (0-1000 for 0-100%)
    int16_t recovery_score;         // Recovery score (0-1000 for 0-100%)
    int16_t effort_level;           // Effort level (0-1000 for 0-100%)
    
    // Metadata (14 bytes)
    uint32_t timestamp;             // Milliseconds since boot
    uint16_t sequence_num;          // Packet sequence number
    uint32_t valid_mask;            // Bit mask for valid metrics (30 bits used)
    uint8_t  calculation_status;    // 0=OK, 1=Warm-up, 2=Error
    uint8_t  reserved[3];           // Padding for alignment
} d2d_metrics_fixed_t;

// Total size: 60 bytes metrics + 14 bytes metadata = 74 bytes
// With ATT header (3 bytes) = 77 bytes total (well under 121 MTU limit)

/**
 * @brief Pack float metrics into fixed-point packet
 * @param fixed Output fixed-point packet
 * @param floating Input floating-point packet
 */
static inline void d2d_metrics_pack(d2d_metrics_fixed_t *fixed, const d2d_metrics_packet_t *floating) {
    // Basic gait metrics
    fixed->gct = (int16_t)(floating->metrics[IDX_GCT]);
    fixed->stride_length = (int16_t)(floating->metrics[IDX_STRIDE_LENGTH] * FP_SCALE_LENGTH_CM);
    fixed->cadence = (int16_t)(floating->metrics[IDX_CADENCE]);
    fixed->pronation = (int16_t)(floating->metrics[IDX_PRONATION] * FP_SCALE_ANGLE_DEG);
    fixed->stance_time = (int16_t)(floating->metrics[IDX_STANCE_TIME]);
    fixed->swing_time = (int16_t)(floating->metrics[IDX_SWING_TIME]);
    
    // Pressure metrics - with clamping for COP values
    fixed->peak_pressure = (int16_t)(floating->metrics[IDX_PEAK_PRESSURE] * FP_SCALE_PRESSURE);
    
    // COP values: scale by 10 to get 0.1mm precision and clamp to int16 range
    float cop_x_scaled = floating->metrics[IDX_COP_X] * 10.0f;  // Convert mm to 0.1mm units
    float cop_y_scaled = floating->metrics[IDX_COP_Y] * 10.0f;
    
    // Clamp to int16_t range
    if (cop_x_scaled > 32767.0f) cop_x_scaled = 32767.0f;
    else if (cop_x_scaled < -32768.0f) cop_x_scaled = -32768.0f;
    if (cop_y_scaled > 32767.0f) cop_y_scaled = 32767.0f;
    else if (cop_y_scaled < -32768.0f) cop_y_scaled = -32768.0f;
    
    fixed->cop_x = (int16_t)cop_x_scaled;
    fixed->cop_y = (int16_t)cop_y_scaled;
    fixed->pressure_integral = (int16_t)(floating->metrics[IDX_PRESSURE_TIME_INTEGRAL]);
    
    // Kinematic metrics
    fixed->vertical_osc = (int16_t)(floating->metrics[IDX_VERTICAL_OSC] * FP_SCALE_LENGTH_CM);
    fixed->flight_time = (int16_t)(floating->metrics[IDX_FLIGHT_TIME]);
    fixed->max_velocity = (int16_t)(floating->metrics[IDX_MAX_VELOCITY] * FP_SCALE_VELOCITY);
    fixed->foot_strike_angle = (int16_t)(floating->metrics[IDX_FOOT_STRIKE_ANGLE] * FP_SCALE_ANGLE_DEG);
    
    // Step metrics
    fixed->step_count = (uint16_t)(floating->metrics[IDX_STEP_COUNT]);
    fixed->step_frequency = (int16_t)(floating->metrics[IDX_STEP_FREQUENCY] * FP_SCALE_FREQUENCY);
    fixed->step_length = (int16_t)(floating->metrics[IDX_STEP_LENGTH] * FP_SCALE_LENGTH_CM);
    fixed->step_time = (int16_t)(floating->metrics[IDX_STEP_TIME]);
    
    // Energy metrics
    fixed->power = (int16_t)(floating->metrics[IDX_POWER] * FP_SCALE_POWER);
    fixed->work = (int16_t)(floating->metrics[IDX_WORK] * FP_SCALE_ENERGY);
    fixed->efficiency = (int16_t)(floating->metrics[IDX_EFFICIENCY] * FP_SCALE_PERCENT);
    
    // Impact metrics
    fixed->peak_impact = (int16_t)(floating->metrics[IDX_PEAK_IMPACT] * FP_SCALE_FORCE);
    fixed->loading_rate = (int16_t)(floating->metrics[IDX_LOADING_RATE] * FP_SCALE_FORCE);
    fixed->impact_duration = (int16_t)(floating->metrics[IDX_IMPACT_DURATION]);
    
    // Balance metrics
    fixed->balance_index = (int16_t)(floating->metrics[IDX_BALANCE_INDEX] * FP_SCALE_PERCENT);
    fixed->ml_sway = (int16_t)(floating->metrics[IDX_ML_SWAY]);
    fixed->ap_sway = (int16_t)(floating->metrics[IDX_AP_SWAY]);
    
    // Fatigue indicators
    fixed->fatigue_index = (int16_t)(floating->metrics[IDX_FATIGUE_INDEX] * FP_SCALE_PERCENT);
    fixed->recovery_score = (int16_t)(floating->metrics[IDX_RECOVERY_SCORE] * FP_SCALE_PERCENT);
    fixed->effort_level = (int16_t)(floating->metrics[IDX_EFFORT_LEVEL] * FP_SCALE_PERCENT);
    
    // Copy metadata
    fixed->timestamp = floating->timestamp;
    fixed->sequence_num = floating->sequence_num;
    fixed->valid_mask = (floating->valid_mask[0]) | 
                        (floating->valid_mask[1] << 8) |
                        (floating->valid_mask[2] << 16) |
                        (floating->valid_mask[3] << 24);
    fixed->calculation_status = floating->calculation_status;
}

/**
 * @brief Unpack fixed-point packet into float metrics
 * @param floating Output floating-point packet
 * @param fixed Input fixed-point packet
 */
static inline void d2d_metrics_unpack(d2d_metrics_packet_t *floating, const d2d_metrics_fixed_t *fixed) {
    // Clear the packet first
    d2d_metrics_clear(floating);
    
    // Basic gait metrics
    floating->metrics[IDX_GCT] = (float)fixed->gct;
    floating->metrics[IDX_STRIDE_LENGTH] = (float)fixed->stride_length / FP_SCALE_LENGTH_CM;
    floating->metrics[IDX_CADENCE] = (float)fixed->cadence;
    floating->metrics[IDX_PRONATION] = (float)fixed->pronation / FP_SCALE_ANGLE_DEG;
    floating->metrics[IDX_STANCE_TIME] = (float)fixed->stance_time;
    floating->metrics[IDX_SWING_TIME] = (float)fixed->swing_time;
    
    // Pressure metrics - unscale COP values
    floating->metrics[IDX_PEAK_PRESSURE] = (float)fixed->peak_pressure / FP_SCALE_PRESSURE;
    floating->metrics[IDX_COP_X] = (float)fixed->cop_x / 10.0f;  // Convert 0.1mm units back to mm
    floating->metrics[IDX_COP_Y] = (float)fixed->cop_y / 10.0f;  // Convert 0.1mm units back to mm
    floating->metrics[IDX_PRESSURE_TIME_INTEGRAL] = (float)fixed->pressure_integral;
    
    // Kinematic metrics
    floating->metrics[IDX_VERTICAL_OSC] = (float)fixed->vertical_osc / FP_SCALE_LENGTH_CM;
    floating->metrics[IDX_FLIGHT_TIME] = (float)fixed->flight_time;
    floating->metrics[IDX_MAX_VELOCITY] = (float)fixed->max_velocity / FP_SCALE_VELOCITY;
    floating->metrics[IDX_FOOT_STRIKE_ANGLE] = (float)fixed->foot_strike_angle / FP_SCALE_ANGLE_DEG;
    
    // Step metrics
    floating->metrics[IDX_STEP_COUNT] = (float)fixed->step_count;
    floating->metrics[IDX_STEP_FREQUENCY] = (float)fixed->step_frequency / FP_SCALE_FREQUENCY;
    floating->metrics[IDX_STEP_LENGTH] = (float)fixed->step_length / FP_SCALE_LENGTH_CM;
    floating->metrics[IDX_STEP_TIME] = (float)fixed->step_time;
    
    // Energy metrics
    floating->metrics[IDX_POWER] = (float)fixed->power / FP_SCALE_POWER;
    floating->metrics[IDX_WORK] = (float)fixed->work / FP_SCALE_ENERGY;
    floating->metrics[IDX_EFFICIENCY] = (float)fixed->efficiency / FP_SCALE_PERCENT;
    
    // Impact metrics
    floating->metrics[IDX_PEAK_IMPACT] = (float)fixed->peak_impact / FP_SCALE_FORCE;
    floating->metrics[IDX_LOADING_RATE] = (float)fixed->loading_rate / FP_SCALE_FORCE;
    floating->metrics[IDX_IMPACT_DURATION] = (float)fixed->impact_duration;
    
    // Balance metrics
    floating->metrics[IDX_BALANCE_INDEX] = (float)fixed->balance_index / FP_SCALE_PERCENT;
    floating->metrics[IDX_ML_SWAY] = (float)fixed->ml_sway;
    floating->metrics[IDX_AP_SWAY] = (float)fixed->ap_sway;
    
    // Fatigue indicators
    floating->metrics[IDX_FATIGUE_INDEX] = (float)fixed->fatigue_index / FP_SCALE_PERCENT;
    floating->metrics[IDX_RECOVERY_SCORE] = (float)fixed->recovery_score / FP_SCALE_PERCENT;
    floating->metrics[IDX_EFFORT_LEVEL] = (float)fixed->effort_level / FP_SCALE_PERCENT;
    
    // Copy metadata
    floating->timestamp = fixed->timestamp;
    floating->sequence_num = fixed->sequence_num;
    floating->valid_mask[0] = (uint8_t)(fixed->valid_mask & 0xFF);
    floating->valid_mask[1] = (uint8_t)((fixed->valid_mask >> 8) & 0xFF);
    floating->valid_mask[2] = (uint8_t)((fixed->valid_mask >> 16) & 0xFF);
    floating->valid_mask[3] = (uint8_t)((fixed->valid_mask >> 24) & 0xFF);
    floating->calculation_status = fixed->calculation_status;
}

/**
 * @brief Check if a specific metric is valid in fixed-point packet
 */
static inline int d2d_metric_fixed_is_valid(const d2d_metrics_fixed_t *packet, enum d2d_metric_index idx) {
    if (idx >= D2D_METRICS_COUNT) return 0;
    return (packet->valid_mask & (1 << idx)) ? 1 : 0;
}

/**
 * @brief Set a metric as valid in fixed-point packet
 */
static inline void d2d_metric_fixed_set_valid(d2d_metrics_fixed_t *packet, enum d2d_metric_index idx) {
    if (idx >= D2D_METRICS_COUNT) return;
    packet->valid_mask |= (1 << idx);
}

#ifdef __cplusplus
}
#endif

#endif // D2D_METRICS_FIXED_H
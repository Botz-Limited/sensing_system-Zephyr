/**
 * @file d2d_metrics.h
 * @brief D2D Calculated Metrics Data Structures
 * 
 * This file defines the structure for transmitting calculated metrics
 * from secondary to primary device via D2D communication.
 * All metrics are pre-calculated on each device to reduce D2D bandwidth.
 */

#ifndef D2D_METRICS_H
#define D2D_METRICS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Number of metrics in the array
#define D2D_METRICS_COUNT 30  // Expandable as needed

// Metric index definitions - Both devices must agree on these
enum d2d_metric_index {
    // Basic gait metrics
    IDX_GCT = 0,                    // Ground contact time (ms)
    IDX_STRIDE_LENGTH = 1,          // Stride length (cm)
    IDX_CADENCE = 2,                // Cadence (steps/min)
    IDX_PRONATION = 3,              // Pronation angle (degrees)
    IDX_STANCE_TIME = 4,            // Stance time (ms)
    IDX_SWING_TIME = 5,             // Swing time (ms)
    
    // Pressure metrics
    IDX_PEAK_PRESSURE = 6,          // Peak pressure (kPa)
    IDX_COP_X = 7,                  // Center of pressure X (mm)
    IDX_COP_Y = 8,                  // Center of pressure Y (mm)
    IDX_PRESSURE_TIME_INTEGRAL = 9, // Pressure time integral
    
    // Kinematic metrics
    IDX_VERTICAL_OSC = 10,          // Vertical oscillation (cm)
    IDX_FLIGHT_TIME = 11,           // Flight time (ms)
    IDX_MAX_VELOCITY = 12,          // Max velocity (m/s)
    IDX_FOOT_STRIKE_ANGLE = 13,     // Foot strike angle (degrees)
    
    // Step metrics
    IDX_STEP_COUNT = 14,            // Step count since start
    IDX_STEP_FREQUENCY = 15,        // Step frequency (Hz)
    IDX_STEP_LENGTH = 16,           // Step length (cm)
    IDX_STEP_TIME = 17,             // Step time (ms)
    
    // Energy metrics
    IDX_POWER = 18,                 // Power (W)
    IDX_WORK = 19,                  // Work (J)
    IDX_EFFICIENCY = 20,            // Efficiency (%)
    
    // Impact metrics
    IDX_PEAK_IMPACT = 21,           // Peak impact (g)
    IDX_LOADING_RATE = 22,          // Loading rate (g/s)
    IDX_IMPACT_DURATION = 23,       // Impact duration (ms)
    
    // Balance metrics
    IDX_BALANCE_INDEX = 24,         // Balance index (0-100)
    IDX_ML_SWAY = 25,              // Medial-lateral sway (mm)
    IDX_AP_SWAY = 26,              // Anterior-posterior sway (mm)
    
    // Fatigue indicators
    IDX_FATIGUE_INDEX = 27,         // Fatigue index (0-100)
    IDX_RECOVERY_SCORE = 28,        // Recovery score (0-100)
    IDX_EFFORT_LEVEL = 29,          // Effort level (0-100)
};

/**
 * @brief D2D Metrics Packet Structure
 * 
 * This structure contains all calculated metrics from one foot.
 * Sent from secondary to primary at 1Hz rate.
 */
typedef struct __attribute__((packed)) {
    // Fixed-size array for all metrics
    float metrics[D2D_METRICS_COUNT];
    
    // Metadata
    uint32_t timestamp;             // Milliseconds since boot
    uint16_t sequence_num;          // Packet sequence number
    uint8_t  valid_mask[4];         // Bit mask for valid metrics (30 bits used)
    uint8_t  calculation_status;    // 0=OK, 1=Warm-up, 2=Error
    uint8_t  reserved[3];           // Padding for alignment
} d2d_metrics_packet_t;

// Total size: 30*4 + 4 + 2 + 4 + 1 + 3 = 120 + 14 = 134 bytes

/**
 * @brief Check if a specific metric is valid
 */
static inline int d2d_metric_is_valid(const d2d_metrics_packet_t *packet, enum d2d_metric_index idx) {
    if (idx >= D2D_METRICS_COUNT) return 0;
    uint8_t byte_idx = idx / 8;
    uint8_t bit_idx = idx % 8;
    return (packet->valid_mask[byte_idx] & (1 << bit_idx)) ? 1 : 0;
}

/**
 * @brief Set a metric as valid
 */
static inline void d2d_metric_set_valid(d2d_metrics_packet_t *packet, enum d2d_metric_index idx) {
    if (idx >= D2D_METRICS_COUNT) return;
    uint8_t byte_idx = idx / 8;
    uint8_t bit_idx = idx % 8;
    packet->valid_mask[byte_idx] |= (1 << bit_idx);
}

/**
 * @brief Clear all metrics and validity
 */
static inline void d2d_metrics_clear(d2d_metrics_packet_t *packet) {
    for (int i = 0; i < D2D_METRICS_COUNT; i++) {
        packet->metrics[i] = 0.0f;
    }
    for (int i = 0; i < 4; i++) {
        packet->valid_mask[i] = 0;
    }
}

#ifdef __cplusplus
}
#endif

#endif // D2D_METRICS_H
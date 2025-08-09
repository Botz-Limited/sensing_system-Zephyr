/**
 * @file realtime_metrics.h
 * @brief Real-time metrics structures and algorithms
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#ifndef REALTIME_METRICS_H
#define REALTIME_METRICS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Constants for metrics calculation
#define CADENCE_WINDOW_SIZE 5       // Number of steps to average for cadence
#define PACE_SMOOTHING_FACTOR 0.3f  // Exponential smoothing for pace
#define FORM_SCORE_MAX 100          // Maximum form score
#define BALANCE_THRESHOLD 10        // Percentage threshold for imbalance

// Default stride length model parameters
#define DEFAULT_STRIDE_BASE_CM 70   // Base stride length in cm
#define STRIDE_CADENCE_FACTOR 0.3f  // How much cadence affects stride
#define STRIDE_HEIGHT_FACTOR 0.43f  // Stride as percentage of height

// Real-time metrics structure for BLE transmission
typedef struct {
    // Timing
    uint32_t timestamp_ms;
    
    // Basic metrics
    uint16_t cadence_spm;           // Steps per minute
    uint16_t pace_sec_km;           // Seconds per kilometer
    uint16_t distance_m;            // Distance in meters (without GPS)
    
    // Form metrics
    uint8_t form_score;             // 0-100 overall form score
    int8_t balance_lr_pct;          // Left/right balance (-50 to +50)
    uint16_t ground_contact_ms;     // Average ground contact time
    uint16_t flight_time_ms;        // Average flight time
    
    // Asymmetry metrics
    uint8_t contact_time_asymmetry;    // Percentage difference
    uint8_t flight_time_asymmetry;     // Percentage difference
    uint8_t force_asymmetry;            // Percentage difference
    
    // Strike pattern
    uint8_t left_strike_pattern;    // 0=heel, 1=mid, 2=fore
    uint8_t right_strike_pattern;   // 0=heel, 1=mid, 2=fore
    
    // Pronation
    int8_t avg_pronation_deg;       // Average of both feet
    uint8_t pronation_asymmetry;    // Difference between feet
    
    // Efficiency indicators
    uint8_t vertical_oscillation_cm; // Vertical oscillation in cm
    uint8_t vertical_ratio;         // Vertical oscillation ratio
    uint8_t efficiency_score;       // Running efficiency 0-100
    
    // Alerts/warnings (bit flags)
    uint8_t alerts;                 // Bit 0: High asymmetry, Bit 1: Poor form, etc.
  uint16_t stride_duration_ms;   // Average stride time in ms
  uint16_t stride_length_cm;     // Average stride length in cm
  uint8_t stride_duration_asymmetry;  // % difference L/R in stride duration
  uint8_t stride_length_asymmetry;  // % difference L/R in stride length
} realtime_metrics_t;

// Cadence tracking structure
typedef struct {
    uint32_t step_timestamps[CADENCE_WINDOW_SIZE];
    uint32_t step_counts[CADENCE_WINDOW_SIZE];
    uint8_t window_index;
    uint8_t window_count;
    float current_cadence_spm;
    float smoothed_cadence_spm;
} cadence_tracker_t;

// Pace estimation structure
typedef struct {
    float stride_length_m;          // Current estimated stride length
    float base_stride_m;            // User's base stride length
    float pace_sec_km;              // Current pace
    float smoothed_pace_sec_km;     // Smoothed pace
    uint32_t distance_m;            // Accumulated distance
} pace_estimator_t;

// Form score components
typedef struct {
    float contact_time_score;       // Based on ground contact time
    float balance_score;            // Based on L/R symmetry
    float consistency_score;        // Based on variability
    float pronation_score;          // Based on pronation angle
    float overall_score;            // Combined score 0-100
} form_score_t;

// Alert flags for real-time metrics (bit flags)
#define RT_ALERT_HIGH_ASYMMETRY    (1 << 0)
#define RT_ALERT_POOR_FORM         (1 << 1)
#define RT_ALERT_HIGH_IMPACT       (1 << 2)
#define RT_ALERT_FATIGUE_DETECTED  (1 << 3)
#define RT_ALERT_OVERPRONATION     (1 << 4)

// Function prototypes for metrics calculation
void cadence_tracker_init(cadence_tracker_t *tracker);
void cadence_tracker_update(cadence_tracker_t *tracker, uint32_t step_count, uint32_t timestamp_ms);
float cadence_tracker_get_current(const cadence_tracker_t *tracker);

void pace_estimator_init(pace_estimator_t *estimator, float user_height_cm);
void pace_estimator_update(pace_estimator_t *estimator, float cadence_spm, uint32_t delta_time_ms);
float pace_estimator_get_pace(const pace_estimator_t *estimator);

void form_score_calculate(form_score_t *score, 
                         uint16_t contact_time_ms,
                         int8_t balance_pct,
                         float variability,
                         int8_t pronation_deg);

uint8_t calculate_asymmetry_percentage(uint16_t left_value, uint16_t right_value);
int8_t calculate_balance_percentage(uint16_t left_force, uint16_t right_force);

void pace_estimator_update_with_gps(pace_estimator_t *estimator, float gps_speed_cms, uint32_t delta_time_ms);

float calibrate_stride_with_gps(pace_estimator_t *estimator, float gps_distance_m, uint32_t step_count_delta);

float estimate_stride_without_gps(float cadence, uint16_t contact_time, float height_cm);

#ifdef __cplusplus
}
#endif

#endif // REALTIME_METRICS_H
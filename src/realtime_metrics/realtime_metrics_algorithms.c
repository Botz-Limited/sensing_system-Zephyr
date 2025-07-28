/**
 * @file realtime_metrics_algorithms.c
 * @brief Implementation of real-time metrics calculation algorithms
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#include "realtime_metrics.h"
#include <math.h>
#include <string.h>

// Initialize cadence tracker
void cadence_tracker_init(cadence_tracker_t *tracker)
{
    memset(tracker, 0, sizeof(cadence_tracker_t));
}

// Update cadence with new step count
void cadence_tracker_update(cadence_tracker_t *tracker, uint32_t step_count, uint32_t timestamp_ms)
{
    // Store in circular buffer
    tracker->step_timestamps[tracker->window_index] = timestamp_ms;
    tracker->step_counts[tracker->window_index] = step_count;
    
    // Update index
    tracker->window_index = (tracker->window_index + 1) % CADENCE_WINDOW_SIZE;
    if (tracker->window_count < CADENCE_WINDOW_SIZE) {
        tracker->window_count++;
    }
    
    // Need at least 2 samples to calculate cadence
    if (tracker->window_count < 2) {
        tracker->current_cadence_spm = 0;
        return;
    }
    
    // Find oldest and newest valid samples
    uint8_t oldest_idx = (tracker->window_index + CADENCE_WINDOW_SIZE - tracker->window_count) % CADENCE_WINDOW_SIZE;
    uint8_t newest_idx = (tracker->window_index + CADENCE_WINDOW_SIZE - 1) % CADENCE_WINDOW_SIZE;
    
    uint32_t time_diff_ms = tracker->step_timestamps[newest_idx] - tracker->step_timestamps[oldest_idx];
    uint32_t step_diff = tracker->step_counts[newest_idx] - tracker->step_counts[oldest_idx];
    
    if (time_diff_ms > 0) {
        // Calculate cadence in steps per minute
        tracker->current_cadence_spm = (float)(step_diff * 60000) / (float)time_diff_ms;
        
        // Apply exponential smoothing
        if (tracker->smoothed_cadence_spm == 0) {
            tracker->smoothed_cadence_spm = tracker->current_cadence_spm;
        } else {
            tracker->smoothed_cadence_spm = (0.7f * tracker->smoothed_cadence_spm) + 
                                           (0.3f * tracker->current_cadence_spm);
        }
    }
}

// Get current cadence
float cadence_tracker_get_current(const cadence_tracker_t *tracker)
{
    return tracker->smoothed_cadence_spm;
}

// Initialize pace estimator
void pace_estimator_init(pace_estimator_t *estimator, float user_height_cm)
{
    memset(estimator, 0, sizeof(pace_estimator_t));
    
    // Initialize base stride length based on height
    // Typical stride length is 40-45% of height
    estimator->base_stride_m = (user_height_cm * STRIDE_HEIGHT_FACTOR) / 100.0f;
    
    // Default to base stride
    estimator->stride_length_m = estimator->base_stride_m;
}

// Update pace estimation based on cadence
void pace_estimator_update(pace_estimator_t *estimator, float cadence_spm, uint32_t delta_time_ms)
{
    if (cadence_spm <= 0 || delta_time_ms == 0) {
        return;
    }
    
    // Adjust stride length based on cadence
    // Higher cadence typically means longer strides (up to a point)
    float cadence_factor = 1.0f + ((cadence_spm - 160.0f) / 100.0f * STRIDE_CADENCE_FACTOR);
    
    // Clamp cadence factor to reasonable range
    if (cadence_factor < 0.7f) cadence_factor = 0.7f;
    if (cadence_factor > 1.5f) cadence_factor = 1.5f;
    
    // Update stride length
    estimator->stride_length_m = estimator->base_stride_m * cadence_factor;
    
    // Calculate speed in m/s
    float steps_per_sec = cadence_spm / 60.0f;
    float speed_m_s = steps_per_sec * estimator->stride_length_m;
    
    // Convert to pace (seconds per km)
    if (speed_m_s > 0) {
        estimator->pace_sec_km = 1000.0f / speed_m_s;
        
        // Apply smoothing
        if (estimator->smoothed_pace_sec_km == 0) {
            estimator->smoothed_pace_sec_km = estimator->pace_sec_km;
        } else {
            estimator->smoothed_pace_sec_km = (estimator->smoothed_pace_sec_km * (1.0f - PACE_SMOOTHING_FACTOR)) +
                                             (estimator->pace_sec_km * PACE_SMOOTHING_FACTOR);
        }
    }
    
    // Update distance
    float distance_delta = (steps_per_sec * estimator->stride_length_m * delta_time_ms) / 1000.0f;
    estimator->distance_m += distance_delta;
}

// Get current pace
float pace_estimator_get_pace(const pace_estimator_t *estimator)
{
    return estimator->smoothed_pace_sec_km;
}

// Update pace estimation with GPS data
void pace_estimator_update_with_gps(pace_estimator_t *estimator, float gps_speed_cms, uint32_t delta_time_ms)
{
    if (gps_speed_cms <= 0 || delta_time_ms == 0) {
        return;
    }

    float speed_m_s = gps_speed_cms / 100.0f;
    estimator->pace_sec_km = (speed_m_s > 0) ? 1000.0f / speed_m_s : 0;

    // Apply smoothing
    if (estimator->smoothed_pace_sec_km == 0) {
        estimator->smoothed_pace_sec_km = estimator->pace_sec_km;
    } else {
        estimator->smoothed_pace_sec_km = (estimator->smoothed_pace_sec_km * (1.0f - PACE_SMOOTHING_FACTOR)) +
                                         (estimator->pace_sec_km * PACE_SMOOTHING_FACTOR);
    }

    // Update distance
    float distance_delta_m = speed_m_s * (delta_time_ms / 1000.0f);
    estimator->distance_m += (uint32_t)distance_delta_m;
}

// Calculate form score from components
void form_score_calculate(form_score_t *score, 
                         uint16_t contact_time_ms,
                         int8_t balance_pct,
                         float variability,
                         int8_t pronation_deg)
{
    // Contact time score (shorter is generally better for running)
    // Optimal range: 200-250ms for recreational runners
    if (contact_time_ms < 200) {
        score->contact_time_score = 100.0f;
    } else if (contact_time_ms > 350) {
        score->contact_time_score = 40.0f;
    } else {
        // Linear scale from 200-350ms
        score->contact_time_score = 100.0f - ((contact_time_ms - 200) * 0.4f);
    }
    
    // Balance score (closer to 0% difference is better)
    uint8_t abs_balance = (balance_pct < 0) ? -balance_pct : balance_pct;
    if (abs_balance <= 5) {
        score->balance_score = 100.0f;
    } else if (abs_balance >= 20) {
        score->balance_score = 50.0f;
    } else {
        // Linear scale from 5-20%
        score->balance_score = 100.0f - ((abs_balance - 5) * 3.33f);
    }
    
    // Consistency score (lower variability is better)
    if (variability < 5.0f) {
        score->consistency_score = 100.0f;
    } else if (variability > 20.0f) {
        score->consistency_score = 60.0f;
    } else {
        // Linear scale from 5-20%
        score->consistency_score = 100.0f - ((variability - 5.0f) * 2.67f);
    }
    
    // Pronation score (normal range: -5 to +10 degrees)
    uint8_t abs_pronation = (pronation_deg < 0) ? -pronation_deg : pronation_deg;
    if (pronation_deg >= -5 && pronation_deg <= 10) {
        score->pronation_score = 100.0f;
    } else if (abs_pronation > 20) {
        score->pronation_score = 60.0f;
    } else {
        // Penalty for over/under pronation
        if (pronation_deg > 10) {
            score->pronation_score = 100.0f - ((pronation_deg - 10) * 2.0f);
        } else {
            score->pronation_score = 100.0f - ((abs_pronation - 5) * 2.33f);
        }
    }
    
    // Calculate overall score (weighted average)
    score->overall_score = (score->contact_time_score * 0.3f) +
                          (score->balance_score * 0.3f) +
                          (score->consistency_score * 0.2f) +
                          (score->pronation_score * 0.2f);
    
    // Ensure score is in valid range
    if (score->overall_score > 100.0f) score->overall_score = 100.0f;
    if (score->overall_score < 0.0f) score->overall_score = 0.0f;
}

// Calculate asymmetry percentage between left and right
uint8_t calculate_asymmetry_percentage(uint16_t left_value, uint16_t right_value)
{
    if (left_value == 0 && right_value == 0) {
        return 0;
    }
    
    uint16_t max_value = (left_value > right_value) ? left_value : right_value;
    uint16_t min_value = (left_value < right_value) ? left_value : right_value;
    
    if (max_value == 0) {
        return 0;
    }
    
    // Calculate percentage difference
    uint16_t diff = max_value - min_value;
    uint8_t asymmetry = (uint8_t)((diff * 100) / max_value);
    
    // Clamp to 100%
    return (asymmetry > 100) ? 100 : asymmetry;
}

// Calculate balance percentage (negative = left dominant, positive = right dominant)
int8_t calculate_balance_percentage(uint16_t left_force, uint16_t right_force)
{
    uint32_t total = left_force + right_force;
    
    if (total == 0) {
        return 0;
    }
    
    // Calculate percentage for each side
    uint8_t left_pct = (uint8_t)((left_force * 100) / total);
    uint8_t right_pct = 100 - left_pct;
    
    // Return difference (positive means right side dominant)
    int8_t balance = (int8_t)right_pct - (int8_t)left_pct;
    
    // Clamp to valid range
    if (balance > 50) return 50;
    if (balance < -50) return -50;
    
    return balance;
}

float calibrate_stride_with_gps(pace_estimator_t *estimator, float gps_distance_m, uint32_t step_count_delta) {
    if (step_count_delta == 0 || gps_distance_m <= 0) {
        return estimator->base_stride_m;
    }

    float measured_stride_m = gps_distance_m / step_count_delta;

    // Smooth into base_stride_m
    estimator->base_stride_m = 0.9f * estimator->base_stride_m + 0.1f * measured_stride_m;

    return estimator->base_stride_m;
}

float estimate_stride_without_gps(float cadence, uint16_t contact_time, float height_cm) {
    // Simple model: base from height, adjust by cadence and contact time
    float base_stride_cm = height_cm * STRIDE_HEIGHT_FACTOR;

    float cadence_factor = 1.0f + ((cadence - 160.0f) / 100.0f * STRIDE_CADENCE_FACTOR);

    // Shorter contact time might indicate longer stride
    float contact_factor = (contact_time > 0) ? 300.0f / contact_time : 1.0f;

    float stride_cm = base_stride_cm * cadence_factor * contact_factor;

    return stride_cm;
}
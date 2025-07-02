/**
 * @file step_detection.cpp
 * @brief Step detection and gait analysis implementation
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#include <activity_session.hpp>
#include <step_detection.hpp>
#include <math.h>
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(step_detection, LOG_LEVEL_DBG);

// Constants for step detection
#define CONTACT_THRESHOLD_N     50.0f   // 50 Newtons minimum for ground contact
#define HYSTERESIS_N           10.0f    // Hysteresis to prevent bouncing
#define MIN_CONTACT_TIME_MS    100.0f   // Minimum contact time for valid step
#define MAX_CONTACT_TIME_MS    500.0f   // Maximum contact time for running
#define MIN_FLIGHT_TIME_MS     20.0f    // Minimum flight time

// Sensor position constants (in mm from heel center)
static const float sensor_x_positions[8] = {-15, 15, -20, 0, 20, -15, 0, 15};
static const float sensor_y_positions[8] = {-80, -80, -40, -40, -40, 20, 20, 20};

// Global state variables
StepDetectionState left_foot_state;
StepDetectionState right_foot_state;

// Step event callback
static step_event_callback_t step_callback = NULL;

// Helper function to calculate total force
static float calculate_total_force(float pressure_values[8])
{
    float total = 0;
    for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
        total += pressure_values[i];
    }
    return total;
}

// Initialize step detection
void step_detection_init(void)
{
    LOG_INF("Initializing step detection");
    
    // Initialize left foot state
    memset(&left_foot_state, 0, sizeof(StepDetectionState));
    left_foot_state.pressure_threshold = CONTACT_THRESHOLD_N;
    left_foot_state.current_phase = SWING_PHASE;
    
    // Initialize right foot state
    memset(&right_foot_state, 0, sizeof(StepDetectionState));
    right_foot_state.pressure_threshold = CONTACT_THRESHOLD_N;
    right_foot_state.current_phase = SWING_PHASE;
}

// Calculate pressure distribution metrics
void calculate_pressure_metrics(float pressure_values[8], PressureMetrics* metrics)
{
    // Calculate regional pressures
    float heel = pressure_values[HEEL_MEDIAL] + pressure_values[HEEL_LATERAL];
    float midfoot = pressure_values[MIDFOOT_MEDIAL] + pressure_values[MIDFOOT_CENTER] + 
                   pressure_values[MIDFOOT_LATERAL];
    float forefoot = pressure_values[FOREFOOT_MEDIAL] + pressure_values[FOREFOOT_CENTER] + 
                    pressure_values[FOREFOOT_LATERAL];
    
    metrics->total_force = heel + midfoot + forefoot;
    
    if (metrics->total_force > 0) {
        metrics->heel_pct = (heel / metrics->total_force) * 100.0f;
        metrics->midfoot_pct = (midfoot / metrics->total_force) * 100.0f;
        metrics->forefoot_pct = (forefoot / metrics->total_force) * 100.0f;
        
        // Calculate center of pressure
        float x_weighted = 0;
        float y_weighted = 0;
        
        for (int i = 0; i < NUM_PRESSURE_SENSORS; i++) {
            x_weighted += pressure_values[i] * sensor_x_positions[i];
            y_weighted += pressure_values[i] * sensor_y_positions[i];
        }
        
        metrics->cop_x = x_weighted / metrics->total_force;
        metrics->cop_y = y_weighted / metrics->total_force;
        
        // Calculate medial-lateral ratio
        float medial = pressure_values[HEEL_MEDIAL] + pressure_values[MIDFOOT_MEDIAL] + 
                      pressure_values[FOREFOOT_MEDIAL];
        float lateral = pressure_values[HEEL_LATERAL] + pressure_values[MIDFOOT_LATERAL] + 
                       pressure_values[FOREFOOT_LATERAL];
        
        metrics->medial_lateral_ratio = medial / (medial + lateral);
    } else {
        metrics->heel_pct = 0;
        metrics->midfoot_pct = 0;
        metrics->forefoot_pct = 0;
        metrics->cop_x = 0;
        metrics->cop_y = 0;
        metrics->medial_lateral_ratio = 0.5f;
    }
}

// Detect foot strike pattern
uint8_t detect_strike_pattern(float pressure_values[8])
{
    float heel = pressure_values[HEEL_MEDIAL] + pressure_values[HEEL_LATERAL];
    float midfoot = pressure_values[MIDFOOT_MEDIAL] + pressure_values[MIDFOOT_CENTER] + 
                   pressure_values[MIDFOOT_LATERAL];
    float forefoot = pressure_values[FOREFOOT_MEDIAL] + pressure_values[FOREFOOT_CENTER] + 
                    pressure_values[FOREFOOT_LATERAL];
    float total = heel + midfoot + forefoot;
    
    if (total < CONTACT_THRESHOLD_N) {
        return STRIKE_HEEL; // Default if no clear contact
    }
    
    // Determine strike pattern based on initial contact distribution
    if (heel / total > 0.6f) {
        return STRIKE_HEEL;
    } else if (forefoot / total > 0.6f) {
        return STRIKE_FOREFOOT;
    } else {
        return STRIKE_MIDFOOT;
    }
}

// Calculate loading rate
float calculate_loading_rate(StepDetectionState* state, float current_force, uint32_t current_time)
{
    // Add to circular buffer
    state->force_history[state->history_index] = current_force;
    state->time_history[state->history_index] = current_time;
    state->history_index = (state->history_index + 1) % 10;
    
    // Find maximum rate of force increase
    float max_loading_rate = 0;
    
    for (int i = 0; i < 9; i++) {
        int idx1 = i;
        int idx2 = i + 1;
        
        float dF = state->force_history[idx2] - state->force_history[idx1];
        float dt = (state->time_history[idx2] - state->time_history[idx1]) / 1000.0f;
        
        if (dt > 0 && dF > 0) {
            float rate = dF / dt;  // N/s
            if (rate > max_loading_rate) {
                max_loading_rate = rate;
            }
        }
    }
    
    return max_loading_rate;
}

// Process pressure data for step detection
void step_detection_process_pressure(uint8_t foot, float pressure_values[8], uint32_t timestamp_ms)
{
    StepDetectionState* state = (foot == 0) ? &left_foot_state : &right_foot_state;
    PressureMetrics metrics;
    
    // Calculate pressure metrics
    calculate_pressure_metrics(pressure_values, &metrics);
    
    // Update pressure distribution
    state->heel_pressure_pct = metrics.heel_pct;
    state->midfoot_pressure_pct = metrics.midfoot_pct;
    state->forefoot_pressure_pct = metrics.forefoot_pct;
    
    // Update center of pressure and calculate path length
    if (state->in_contact) {
        float dx = metrics.cop_x - state->last_cop_x;
        float dy = metrics.cop_y - state->last_cop_y;
        state->cop_path_length_mm += sqrtf(dx * dx + dy * dy);
    }
    state->cop_x_mm = metrics.cop_x;
    state->cop_y_mm = metrics.cop_y;
    state->last_cop_x = metrics.cop_x;
    state->last_cop_y = metrics.cop_y;
    
    // Ground contact detection with hysteresis
    if (!state->in_contact && metrics.total_force > state->pressure_threshold + HYSTERESIS_N) {
        // Contact started
        state->in_contact = true;
        state->contact_start_time = timestamp_ms;
        state->current_phase = LOADING_PHASE;
        state->phase_start_time = timestamp_ms;
        state->cop_path_length_mm = 0;
        
        // Detect initial strike pattern
        state->initial_contact_region = detect_strike_pattern(pressure_values);
        
        // Calculate flight time if we have a previous contact
        if (state->last_contact_end_time > 0) {
            state->flight_time_ms = timestamp_ms - state->last_contact_end_time;
        }
        
        LOG_DBG("Foot %d: Contact started, strike pattern: %d", foot, state->initial_contact_region);
        
    } else if (state->in_contact && metrics.total_force < state->pressure_threshold - HYSTERESIS_N) {
        // Contact ended
        state->in_contact = false;
        state->last_contact_end_time = timestamp_ms;
        state->current_phase = SWING_PHASE;
        
        // Calculate contact time
        state->ground_contact_time_ms = timestamp_ms - state->contact_start_time;
        
        // Validate ground contact (we get step count from BHI360)
        if (state->ground_contact_time_ms >= MIN_CONTACT_TIME_MS && 
            state->ground_contact_time_ms <= MAX_CONTACT_TIME_MS) {
            
            // Create step event with timing and pressure data
            if (step_callback) {
                StepEvent event = {
                    .foot = foot,
                    .timestamp = timestamp_ms,
                    .contact_time_ms = state->ground_contact_time_ms,
                    .flight_time_ms = state->flight_time_ms,
                    .peak_force = state->peak_force,
                    .loading_rate = state->loading_rate,
                    .strike_pattern = state->initial_contact_region,
                    .pronation_estimate = (metrics.medial_lateral_ratio - 0.5f) * 30.0f // Simple estimate
                };
                step_callback(&event);
            }
            
            LOG_DBG("Foot %d: Ground contact ended! Contact time: %.1f ms, Flight time: %.1f ms", 
                    foot, state->ground_contact_time_ms, state->flight_time_ms);
        }
        
        // Reset peak force for next step
        state->peak_force = 0;
    }
    
    // Update metrics during contact
    if (state->in_contact) {
        // Track peak force
        if (metrics.total_force > state->peak_force) {
            state->peak_force = metrics.total_force;
        }
        
        // Calculate loading rate
        state->loading_rate = calculate_loading_rate(state, metrics.total_force, timestamp_ms);
        
        // Update gait phase based on pressure distribution
        uint32_t contact_duration = timestamp_ms - state->contact_start_time;
        
        if (contact_duration < 50) {
            state->current_phase = LOADING_PHASE;
        } else if (metrics.forefoot_pct > 60) {
            state->current_phase = PUSH_OFF_PHASE;
        } else {
            state->current_phase = MIDSTANCE_PHASE;
        }
    }
}

// Get current metrics for a foot
void step_detection_get_metrics(uint8_t foot, StepDetectionState* state_out)
{
    StepDetectionState* state = (foot == 0) ? &left_foot_state : &right_foot_state;
    memcpy(state_out, state, sizeof(StepDetectionState));
}

// Register callback for step events
void step_detection_register_callback(step_event_callback_t callback)
{
    step_callback = callback;
}

// Estimate step width from pressure distribution
float estimate_step_width(float medial_lateral_ratio, float cop_lateral_deviation)
{
    // Base width in cm
    float base_width = 8.0f;
    
    // Adjust based on pressure distribution
    float pressure_adjustment = (medial_lateral_ratio - 0.5f) * 10.0f;
    
    // Adjust based on CoP deviation
    float cop_adjustment = cop_lateral_deviation * 0.5f;
    
    float step_width = base_width + pressure_adjustment + cop_adjustment;
    
    // Constrain to reasonable range
    if (step_width < 0) step_width = 0;
    if (step_width > 30) step_width = 30;
    
    return step_width;
}
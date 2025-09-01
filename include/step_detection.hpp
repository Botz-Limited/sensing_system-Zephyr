/**
 * @file step_detection.hpp
 * @brief Step detection and gait analysis from pressure sensors
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#ifndef APP_INCLUDE_STEP_DETECTION_HEADER_
#define APP_INCLUDE_STEP_DETECTION_HEADER_

#include <stdint.h>
#include <stdbool.h>

// Number of pressure sensors per foot
#define NUM_PRESSURE_SENSORS 8

// Pressure sensor channel mapping
// Channels 0-1: Heel region
// Channels 2-4: Midfoot region  
// Channels 5-7: Forefoot region
enum PressureChannels {
    HEEL_MEDIAL = 0,
    HEEL_LATERAL = 1,
    MIDFOOT_MEDIAL = 2,
    MIDFOOT_CENTER = 3,
    MIDFOOT_LATERAL = 4,
    FOREFOOT_MEDIAL = 5,
    FOREFOOT_CENTER = 6,
    FOREFOOT_LATERAL = 7
};

// Gait phases
enum GaitPhase {
    SWING_PHASE = 0,
    LOADING_PHASE = 1,
    MIDSTANCE_PHASE = 2,
    PUSH_OFF_PHASE = 3
};

// Step detection state for each foot
typedef struct {
    // Contact detection
    bool in_contact;
    uint32_t contact_start_time;
    uint32_t last_contact_end_time;
    float pressure_threshold;
    
    // Current step metrics
    float ground_contact_time_ms;
    float flight_time_ms;
    float peak_force;
    float loading_rate;
    
    // Pressure distribution
    float heel_pressure_pct;
    float midfoot_pressure_pct;
    float forefoot_pressure_pct;
    
    // Center of pressure
    float cop_x_mm;  // Medial-lateral position
    float cop_y_mm;  // Anterior-posterior position
    float cop_path_length_mm;
    float last_cop_x;
    float last_cop_y;
    
    // Gait phase tracking
    enum GaitPhase current_phase;
    uint32_t phase_start_time;
    
    // Strike pattern detection
    uint8_t initial_contact_region;  // 0=heel, 1=midfoot, 2=forefoot
    
    // Force history for loading rate
    float force_history[10];
    uint32_t time_history[10];
    int history_index;
    
} StepDetectionState;

// Pressure metrics calculated per step
typedef struct {
    float total_force;
    float heel_pct;
    float midfoot_pct;
    float forefoot_pct;
    float cop_x;
    float cop_y;
    float medial_lateral_ratio;
} PressureMetrics;

// Step event data
typedef struct {
    uint8_t foot;  // 0=left, 1=right
    uint32_t timestamp;
    float contact_time_ms;
    float flight_time_ms;
    float peak_force;
    float loading_rate;
    uint8_t strike_pattern;
    float pronation_estimate;
} StepEvent;

// Initialize step detection
void step_detection_init(void);

// Process pressure data for a foot
void step_detection_process_pressure(
    uint8_t foot,                    // 0=left, 1=right
    float pressure_values[8],        // 8 channel pressure data
    uint32_t timestamp_ms           // Current timestamp
);

// Get current metrics for a foot
void step_detection_get_metrics(
    uint8_t foot,
    StepDetectionState* state
);

// Calculate pressure distribution metrics
void calculate_pressure_metrics(
    float pressure_values[8],
    PressureMetrics* metrics
);

// Detect foot strike pattern from initial contact
uint8_t detect_strike_pattern(
    float pressure_values[8]
);

// Calculate loading rate from force history
float calculate_loading_rate(
    StepDetectionState* state,
    float current_force,
    uint32_t current_time
);

// Estimate step width from pressure distribution
float estimate_step_width(
    float medial_lateral_ratio,
    float cop_lateral_deviation
);

// Global step detection states
extern StepDetectionState left_foot_state;
extern StepDetectionState right_foot_state;

// Callback for step events
typedef void (*step_event_callback_t)(const StepEvent* event);
void step_detection_register_callback(step_event_callback_t callback);

#endif // APP_INCLUDE_STEP_DETECTION_HEADER_
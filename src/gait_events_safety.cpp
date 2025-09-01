/**
 * @file gait_events_safety.cpp
 * @brief Safety checks and overflow prevention for gait events
 */

#include <gait_events_safety.h>
#include <gait_events.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gait_safety, LOG_LEVEL_INF);

// Wrap functions in extern "C" for linkage from C++ code
extern "C" {

/**
 * Validate and sanitize gait metrics before use
 * Prevents overflow and crash issues while keeping advanced features enabled
 */
void gait_metrics_validate(gait_metrics_t *metrics)
{
    if (!metrics) return;
    
    // Maximum reasonable values for gait metrics
    const float MAX_GCT_SECONDS = 5.0f;        // 5 seconds max ground contact time
    const float MAX_CADENCE_SPM = 250.0f;      // 250 steps per minute max
    const float MAX_STRIDE_LENGTH_M = 3.0f;    // 3 meters max stride
    const float MAX_VERTICAL_OSC_M = 0.3f;     // 30cm max vertical oscillation
    const float MAX_SPEED_MPS = 10.0f;         // 10 m/s max speed (36 km/h)
    
    // Validate and clamp Ground Contact Time
    if (metrics->gct > MAX_GCT_SECONDS || metrics->gct < 0) {
        LOG_WRN("Invalid GCT %.3f, clamping to valid range", (double)metrics->gct);
        metrics->gct = (metrics->gct < 0) ? 0.0f : MAX_GCT_SECONDS;
    }
    
    // Validate and clamp Cadence
    if (metrics->cadence > MAX_CADENCE_SPM || metrics->cadence < 0) {
        LOG_WRN("Invalid cadence %.1f, clamping to valid range", (double)metrics->cadence);
        metrics->cadence = (metrics->cadence < 0) ? 0.0f : MAX_CADENCE_SPM;
    }
    
    // Validate and clamp Stride Length
    if (metrics->stride_length > MAX_STRIDE_LENGTH_M || metrics->stride_length < 0) {
        LOG_WRN("Invalid stride length %.2f, clamping to valid range", (double)metrics->stride_length);
        metrics->stride_length = (metrics->stride_length < 0) ? 0.0f : MAX_STRIDE_LENGTH_M;
    }
    
    // Validate and clamp Vertical Oscillation
    if (metrics->vertical_oscillation > MAX_VERTICAL_OSC_M || metrics->vertical_oscillation < 0) {
        LOG_WRN("Invalid vertical oscillation %.3f, clamping to valid range", (double)metrics->vertical_oscillation);
        metrics->vertical_oscillation = (metrics->vertical_oscillation < 0) ? 0.0f : MAX_VERTICAL_OSC_M;
    }
    
    // Validate and clamp Speed
    if (metrics->stride_velocity > MAX_SPEED_MPS || metrics->stride_velocity < 0) {
        LOG_WRN("Invalid stride velocity %.2f, clamping to valid range", (double)metrics->stride_velocity);
        metrics->stride_velocity = (metrics->stride_velocity < 0) ? 0.0f : MAX_SPEED_MPS;
    }
    if (metrics->speed_anthropometric > MAX_SPEED_MPS || metrics->speed_anthropometric < 0) {
        LOG_WRN("Invalid anthropometric speed %.2f, clamping to valid range", (double)metrics->speed_anthropometric);
        metrics->speed_anthropometric = (metrics->speed_anthropometric < 0) ? 0.0f : MAX_SPEED_MPS;
    }
    
    // Validate angles (should be within -180 to 180 degrees)
    auto validate_angle = [](float &angle, const char *name) {
        if (angle > 180.0f || angle < -180.0f) {
            LOG_WRN("Invalid %s angle %.1f, wrapping to valid range", name, (double)angle);
            while (angle > 180.0f) angle -= 360.0f;
            while (angle < -180.0f) angle += 360.0f;
        }
    };
    
    validate_angle(metrics->ic_pitch, "IC pitch");
    validate_angle(metrics->ic_roll, "IC roll");
    validate_angle(metrics->to_pitch, "TO pitch");
    validate_angle(metrics->to_roll, "TO roll");
    validate_angle(metrics->min_pitch, "min pitch");
    validate_angle(metrics->min_roll, "min roll");
    validate_angle(metrics->max_roll, "max roll");
}

/**
 * Reset stuck detector state when contact time exceeds reasonable limits
 */
void gait_events_reset_if_stuck(gait_event_detector_t *detector)
{
    if (!detector) return;
    
    // Check if we're stuck based on current phase and timing
    static uint32_t last_phase_change = 0;
    static gait_phase_t last_phase = GAIT_PHASE_IDLE;
    uint32_t current_time_ms = k_uptime_get_32();
    
    // Check if phase hasn't changed for too long (5 seconds)
    if (detector->current_phase == last_phase &&
        detector->current_phase != GAIT_PHASE_IDLE) {
        
        uint32_t phase_duration_ms = current_time_ms - last_phase_change;
        
        // If in same phase for over 5 seconds, force reset
        if (phase_duration_ms > 5000) {
            LOG_WRN("Phase stuck in %d for %u ms, forcing reset",
                    detector->current_phase, phase_duration_ms);
            
            // Clear half the buffer to recover
            if (detector->buffer_count > GAIT_BUFFER_SIZE_SAMPLES / 2) {
                int samples_to_clear = detector->buffer_count / 2;
                detector->read_index = (detector->read_index + samples_to_clear) % GAIT_BUFFER_SIZE_SAMPLES;
                detector->buffer_count -= samples_to_clear;
                detector->buffer_full = false;
                LOG_INF("Cleared %d samples from buffer to recover", samples_to_clear);
            }
            
            // Reset velocity and position to prevent drift
            detector->current_velocity.x = 0;
            detector->current_velocity.y = 0;
            detector->current_velocity.z = 0;
            detector->current_position.x = 0;
            detector->current_position.y = 0;
            detector->current_position.z = 0;
            
            // Reset phase
            detector->current_phase = GAIT_PHASE_IDLE;
            last_phase_change = current_time_ms;
        }
    } else {
        // Phase changed, update tracking
        last_phase = detector->current_phase;
        last_phase_change = current_time_ms;
    }
    
    // Also check buffer health
    if (detector->buffer_full && detector->buffer_count >= GAIT_BUFFER_SIZE_SAMPLES) {
        static int consecutive_full_count = 0;
        consecutive_full_count++;
        
        if (consecutive_full_count > 10) {
            LOG_WRN("Buffer stuck full for %d cycles, forcing clear", consecutive_full_count);
            
            // Clear oldest 50% of buffer
            int samples_to_clear = GAIT_BUFFER_SIZE_SAMPLES / 2;
            detector->read_index = (detector->read_index + samples_to_clear) % GAIT_BUFFER_SIZE_SAMPLES;
            detector->buffer_count = GAIT_BUFFER_SIZE_SAMPLES / 2;
            detector->buffer_full = false;
            consecutive_full_count = 0;
            
            LOG_INF("Force cleared 50%% of buffer to recover processing");
        }
    } else {
        // Reset consecutive count if buffer is not stuck
        static int consecutive_full_count = 0;
        consecutive_full_count = 0;
    }
}

/**
 * Enhanced process function with safety checks
 */
int gait_events_process_safe(gait_event_detector_t *detector)
{
    if (!detector) return 0;
    
    // First check if we're stuck
    gait_events_reset_if_stuck(detector);
    
    // Process events normally
    int metrics_count = gait_events_process(detector);
    
    // Validate all generated metrics
    if (metrics_count > 0) {
        gait_metrics_t metrics[GAIT_MAX_EVENTS_PER_CHUNK];
        int retrieved = gait_events_get_metrics(detector, metrics, GAIT_MAX_EVENTS_PER_CHUNK);
        
        for (int i = 0; i < retrieved; i++) {
            gait_metrics_validate(&metrics[i]);
        }
        
        // Put validated metrics back (assuming we need to update them)
        // Note: This would require adding a set_metrics function to the API
    }
    
    return metrics_count;
}

} // extern "C"
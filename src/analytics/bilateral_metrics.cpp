/**
 * @file bilateral_metrics.cpp
 * @brief Primary device bilateral metrics processing
 * 
 * This module receives metrics from the secondary device and combines them
 * with locally calculated metrics to produce bilateral gait analysis.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

#include <app.hpp>
#include <d2d_metrics.h>
#include "bilateral_metrics.h"
#include <choros_buffer.hpp>
#include "../realtime_metrics/realtime_metrics.h"

LOG_MODULE_REGISTER(bilateral_metrics, LOG_LEVEL_WRN);

// External Choros buffer instance
extern choros_ring_buffer_t choros_buffer;

// Message queue for analytics module (only on primary device)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
K_MSGQ_DEFINE(analytics_msgq, sizeof(generic_message_t), 10, 4);
#endif

// Store latest metrics from both feet
static d2d_metrics_packet_t left_foot_metrics;  // From secondary
static d2d_metrics_packet_t right_foot_metrics; // Local calculation
static bool left_metrics_valid = false;
static bool right_metrics_valid = false;

// Bilateral calculation results
static struct {
    float gait_symmetry_index;
    float phase_coordination;
    float load_distribution;
    float step_length_asymmetry;
    float cadence_variability;
    float bilateral_balance;
    uint32_t last_calculation_ms;
} bilateral_results;

// Timer for local metrics calculation
static struct k_timer primary_metrics_timer;
static uint16_t primary_sequence_num = 0;
static bool activity_active = false;  // Track if activity is running

/**
 * @brief Calculate local (right foot) metrics
 */
static void calculate_right_foot_metrics(void)
{
    // Clear packet
    d2d_metrics_clear(&right_foot_metrics);
    
    // Set metadata
    right_foot_metrics.timestamp = k_uptime_get_32();
    right_foot_metrics.sequence_num = primary_sequence_num++;
    
    // Check buffer health
    if (!choros_buffer_is_healthy()) {
        right_foot_metrics.calculation_status = 2; // Error
        LOG_WRN("Right foot buffer unhealthy");
        return;
    }
    
    if (choros_buffer.count < 100) {
        right_foot_metrics.calculation_status = 1; // Warm-up
        LOG_DBG("Right foot buffer warming up: %d samples", choros_buffer.count);
        return;
    }
    
    right_foot_metrics.calculation_status = 0; // OK
    
    // Calculate ground contact time
    float gct = choros_get_gct_safe(0);
    if (gct > 0) {
        right_foot_metrics.metrics[IDX_GCT] = gct;
        d2d_metric_set_valid(&right_foot_metrics, IDX_GCT);
    }
    
    // Calculate stride length
    float stride = choros_get_stride_length_safe(0);
    if (stride > 0) {
        right_foot_metrics.metrics[IDX_STRIDE_LENGTH] = stride * 100; // Convert to cm
        d2d_metric_set_valid(&right_foot_metrics, IDX_STRIDE_LENGTH);
    }
    
    // Calculate cadence
    float cadence = choros_get_cadence_safe(0);
    if (cadence > 0) {
        right_foot_metrics.metrics[IDX_CADENCE] = cadence;
        d2d_metric_set_valid(&right_foot_metrics, IDX_CADENCE);
    }
    
    // Calculate pronation
    float pronation = choros_get_pronation_safe(0);
    if (pronation != 0) {
        right_foot_metrics.metrics[IDX_PRONATION] = pronation;
        d2d_metric_set_valid(&right_foot_metrics, IDX_PRONATION);
    }
    
    right_metrics_valid = true;
    
    // Comprehensive logging of RIGHT foot metrics
    LOG_INF("=== PRIMARY DEVICE METRICS (Right Foot) ===");
    LOG_INF("Timestamp: %u ms, Sequence: %u", right_foot_metrics.timestamp, right_foot_metrics.sequence_num);
    
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_GCT)) {
        LOG_INF("  Ground Contact Time (GCT): %.1f ms", (double)right_foot_metrics.metrics[IDX_GCT]);
    }
    
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_STRIDE_LENGTH)) {
        LOG_INF("  Stride Length: %.2f cm (%.2f m)",
                (double)right_foot_metrics.metrics[IDX_STRIDE_LENGTH],
                (double)(right_foot_metrics.metrics[IDX_STRIDE_LENGTH] / 100.0f));
    }
    
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_CADENCE)) {
        LOG_INF("  Cadence: %.1f steps/min", (double)right_foot_metrics.metrics[IDX_CADENCE]);
    }
    
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_PRONATION)) {
        LOG_INF("  Pronation Angle: %.1f degrees", (double)right_foot_metrics.metrics[IDX_PRONATION]);
    }
    
    LOG_INF("=============================================");
}

/**
 * @brief Calculate gait symmetry index
 * @return Symmetry percentage (100% = perfect symmetry)
 */
static float calculate_gait_symmetry(void)
{
    if (!left_metrics_valid || !right_metrics_valid) {
        return 0;
    }
    
    float symmetry_sum = 0;
    int symmetry_count = 0;
    
    // Compare GCT
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_GCT) &&
        d2d_metric_is_valid(&right_foot_metrics, IDX_GCT)) {
        float left_gct = left_foot_metrics.metrics[IDX_GCT];
        float right_gct = right_foot_metrics.metrics[IDX_GCT];
        float avg_gct = (left_gct + right_gct) / 2.0f;
        if (avg_gct > 0) {
            float gct_symmetry = 100.0f * (1.0f - fabsf(left_gct - right_gct) / avg_gct);
            symmetry_sum += gct_symmetry;
            symmetry_count++;
        }
    }
    
    // Compare stride length
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_STRIDE_LENGTH) &&
        d2d_metric_is_valid(&right_foot_metrics, IDX_STRIDE_LENGTH)) {
        float left_stride = left_foot_metrics.metrics[IDX_STRIDE_LENGTH];
        float right_stride = right_foot_metrics.metrics[IDX_STRIDE_LENGTH];
        float avg_stride = (left_stride + right_stride) / 2.0f;
        if (avg_stride > 0) {
            float stride_symmetry = 100.0f * (1.0f - fabsf(left_stride - right_stride) / avg_stride);
            symmetry_sum += stride_symmetry;
            symmetry_count++;
        }
    }
    
    // Compare cadence
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_CADENCE) &&
        d2d_metric_is_valid(&right_foot_metrics, IDX_CADENCE)) {
        float left_cadence = left_foot_metrics.metrics[IDX_CADENCE];
        float right_cadence = right_foot_metrics.metrics[IDX_CADENCE];
        float avg_cadence = (left_cadence + right_cadence) / 2.0f;
        if (avg_cadence > 0) {
            float cadence_symmetry = 100.0f * (1.0f - fabsf(left_cadence - right_cadence) / avg_cadence);
            symmetry_sum += cadence_symmetry;
            symmetry_count++;
        }
    }
    
    return (symmetry_count > 0) ? (symmetry_sum / symmetry_count) : 0;
}

/**
 * @brief Calculate phase coordination between feet
 * @return Phase coordination index (0-100)
 */
static float calculate_phase_coordination(void)
{
    if (!left_metrics_valid || !right_metrics_valid) {
        return 0;
    }
    
    // Check if we have stance and swing times
    if (!d2d_metric_is_valid(&left_foot_metrics, IDX_STANCE_TIME) ||
        !d2d_metric_is_valid(&right_foot_metrics, IDX_STANCE_TIME) ||
        !d2d_metric_is_valid(&left_foot_metrics, IDX_SWING_TIME) ||
        !d2d_metric_is_valid(&right_foot_metrics, IDX_SWING_TIME)) {
        
        // Estimate from GCT if available
        if (d2d_metric_is_valid(&left_foot_metrics, IDX_GCT) &&
            d2d_metric_is_valid(&right_foot_metrics, IDX_GCT)) {
            float left_gct = left_foot_metrics.metrics[IDX_GCT];
            float right_gct = right_foot_metrics.metrics[IDX_GCT];
            
            // Ideal phase is 180 degrees (0.5 cycle) offset
            // Estimate phase from GCT ratio
            float phase_diff = fabsf(left_gct - right_gct) / (left_gct + right_gct);
            float coordination = 100.0f * (1.0f - 2.0f * phase_diff);
            return fmaxf(0, coordination);
        }
    }
    
    float left_stance = left_foot_metrics.metrics[IDX_STANCE_TIME];
    float right_stance = right_foot_metrics.metrics[IDX_STANCE_TIME];
    float left_swing = left_foot_metrics.metrics[IDX_SWING_TIME];
    float right_swing = right_foot_metrics.metrics[IDX_SWING_TIME];
    
    // Calculate phase relationship
    float left_cycle = left_stance + left_swing;
    float right_cycle = right_stance + right_swing;
    
    if (left_cycle > 0 && right_cycle > 0) {
        // Calculate phase offset (should be ~0.5 for normal gait)
        float phase_offset = fabsf(left_stance / left_cycle - right_stance / right_cycle);
        float ideal_offset = 0.5f;
        float phase_error = fabsf(phase_offset - ideal_offset);
        
        // Convert to 0-100 scale
        return 100.0f * (1.0f - 2.0f * phase_error);
    }
    
    return 0;
}

/**
 * @brief Calculate load distribution between feet
 * @return Load distribution ratio (-100 to +100, 0 = balanced)
 */
static float calculate_load_distribution(void)
{
    if (!left_metrics_valid || !right_metrics_valid) {
        return 0;
    }
    
    // Use peak pressure if available
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_PEAK_PRESSURE) &&
        d2d_metric_is_valid(&right_foot_metrics, IDX_PEAK_PRESSURE)) {
        float left_pressure = left_foot_metrics.metrics[IDX_PEAK_PRESSURE];
        float right_pressure = right_foot_metrics.metrics[IDX_PEAK_PRESSURE];
        float total_pressure = left_pressure + right_pressure;
        
        if (total_pressure > 0) {
            // Calculate distribution (-100 = all left, +100 = all right, 0 = balanced)
            float distribution = 100.0f * (right_pressure - left_pressure) / total_pressure;
            return distribution;
        }
    }
    
    // Fallback: use GCT as proxy for load
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_GCT) &&
        d2d_metric_is_valid(&right_foot_metrics, IDX_GCT)) {
        float left_gct = left_foot_metrics.metrics[IDX_GCT];
        float right_gct = right_foot_metrics.metrics[IDX_GCT];
        float total_gct = left_gct + right_gct;
        
        if (total_gct > 0) {
            // Longer GCT typically indicates higher load
            float distribution = 100.0f * (right_gct - left_gct) / total_gct;
            return distribution;
        }
    }
    
    return 0;
}

/**
 * @brief Timer callback for primary metrics calculation
 */
static void primary_metrics_timer_callback(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Calculate local (right foot) metrics
    calculate_right_foot_metrics();
    
    // Check if we have fresh left foot metrics
    // Consider metrics stale if older than 5 seconds
    bool left_metrics_fresh = false;
    if (left_metrics_valid) {
        uint32_t age_ms = k_uptime_get_32() - left_foot_metrics.timestamp;
        left_metrics_fresh = (age_ms < 5000);
        
        if (!left_metrics_fresh) {
            LOG_DBG("Left foot metrics stale (age: %u ms)", age_ms);
            left_metrics_valid = false;
        }
    }
    
    // If no fresh left foot metrics, zero out
    if (!left_metrics_fresh) {
        d2d_metrics_clear(&left_foot_metrics);
        left_foot_metrics.calculation_status = 3; // No connection/stale
        left_metrics_valid = false;
    }
    
    // Calculate bilateral metrics if both feet data available
    if (left_metrics_valid && right_metrics_valid) {
        // Check timestamps are reasonably close (within 2 seconds)
        int32_t time_diff = (int32_t)(right_foot_metrics.timestamp - left_foot_metrics.timestamp);
        if (abs(time_diff) > 2000) {
            LOG_WRN("Metrics timestamp mismatch: %d ms", time_diff);
        }
        
        // Calculate bilateral metrics
        bilateral_results.gait_symmetry_index = calculate_gait_symmetry();
        bilateral_results.phase_coordination = calculate_phase_coordination();
        bilateral_results.load_distribution = calculate_load_distribution();
        bilateral_results.last_calculation_ms = k_uptime_get_32();
        
        // Comprehensive logging of BILATERAL metrics
        LOG_INF("=== BILATERAL METRICS (Both Feet) ===");
        LOG_INF("D2D Status: CONNECTED");
        LOG_INF("  Gait Symmetry Index: %.1f%% (100%% = perfect symmetry)",
                (double)bilateral_results.gait_symmetry_index);
        LOG_INF("  Phase Coordination: %.1f (0-100, higher = better coordination)",
                (double)bilateral_results.phase_coordination);
        LOG_INF("  Load Distribution: %.1f%% (-100=all left, 0=balanced, +100=all right)",
                (double)bilateral_results.load_distribution);
        
        // Log left foot metrics received from secondary
        LOG_INF("--- Left Foot (from D2D) ---");
        if (d2d_metric_is_valid(&left_foot_metrics, IDX_GCT)) {
            LOG_INF("  Left GCT: %.1f ms", (double)left_foot_metrics.metrics[IDX_GCT]);
        }
        if (d2d_metric_is_valid(&left_foot_metrics, IDX_STRIDE_LENGTH)) {
            LOG_INF("  Left Stride: %.2f cm", (double)left_foot_metrics.metrics[IDX_STRIDE_LENGTH]);
        }
        if (d2d_metric_is_valid(&left_foot_metrics, IDX_CADENCE)) {
            LOG_INF("  Left Cadence: %.1f spm", (double)left_foot_metrics.metrics[IDX_CADENCE]);
        }
        if (d2d_metric_is_valid(&left_foot_metrics, IDX_PRONATION)) {
            LOG_INF("  Left Pronation: %.1f deg", (double)left_foot_metrics.metrics[IDX_PRONATION]);
        }
        if (d2d_metric_is_valid(&left_foot_metrics, IDX_PEAK_PRESSURE)) {
            LOG_INF("  Left Peak Pressure: %.1f kPa", (double)left_foot_metrics.metrics[IDX_PEAK_PRESSURE]);
        }
        
        // Calculate and log asymmetry values
        if (d2d_metric_is_valid(&left_foot_metrics, IDX_GCT) &&
            d2d_metric_is_valid(&right_foot_metrics, IDX_GCT)) {
            float gct_diff = right_foot_metrics.metrics[IDX_GCT] - left_foot_metrics.metrics[IDX_GCT];
            LOG_INF("  GCT Asymmetry: %.1f ms (R-L)", (double)gct_diff);
        }
        
        if (d2d_metric_is_valid(&left_foot_metrics, IDX_STRIDE_LENGTH) &&
            d2d_metric_is_valid(&right_foot_metrics, IDX_STRIDE_LENGTH)) {
            float stride_diff = right_foot_metrics.metrics[IDX_STRIDE_LENGTH] - left_foot_metrics.metrics[IDX_STRIDE_LENGTH];
            LOG_INF("  Stride Asymmetry: %.1f cm (R-L)", (double)stride_diff);
        }
        
        LOG_INF("=============================================");
        
        // Send consolidated metrics to phone via realtime_metrics
        realtime_metrics_t consolidated = {};
        
        // Always send right foot metrics (local data)
        if (d2d_metric_is_valid(&right_foot_metrics, IDX_GCT)) {
            consolidated.ground_contact_ms = right_foot_metrics.metrics[IDX_GCT];
        }
        if (d2d_metric_is_valid(&right_foot_metrics, IDX_STRIDE_LENGTH)) {
            consolidated.stride_length_cm = right_foot_metrics.metrics[IDX_STRIDE_LENGTH]; // Already in cm
        }
        if (d2d_metric_is_valid(&right_foot_metrics, IDX_CADENCE)) {
            consolidated.cadence_spm = right_foot_metrics.metrics[IDX_CADENCE];
        }
        if (d2d_metric_is_valid(&right_foot_metrics, IDX_PRONATION)) {
            consolidated.avg_pronation_deg = right_foot_metrics.metrics[IDX_PRONATION];
        }
        
        // Only include bilateral metrics if both feet valid
        if (left_metrics_valid && right_metrics_valid) {
            // Bilateral metrics - store in balance field (convert to -50 to +50 range)
            // Symmetry of 100% = balance of 0, symmetry of 80% = balance of ±10
            consolidated.balance_lr_pct = (int8_t)((100.0f - bilateral_results.gait_symmetry_index) / 2.0f);
            if (bilateral_results.load_distribution > 0) {
                consolidated.balance_lr_pct = -consolidated.balance_lr_pct; // Right-heavy is negative
            }
        } else {
            // No bilateral data - set balance to neutral (0)
            consolidated.balance_lr_pct = 0;
        }
        
        // Send to sensor data queue (realtime metrics input)
        generic_message_t msg = {};
        msg.sender = SENDER_ANALYTICS;
        msg.type = MSG_TYPE_REALTIME_METRICS;
        memcpy(&msg.data.realtime_metrics, &consolidated, sizeof(realtime_metrics_t));
        
        extern struct k_msgq sensor_data_queue;
        if (k_msgq_put(&sensor_data_queue, &msg, K_NO_WAIT) != 0) {
            LOG_ERR("Failed to send consolidated metrics to realtime module");
        }
    } else {
        // Only right foot metrics available (unilateral mode)
        if (right_metrics_valid && !left_metrics_valid) {
            LOG_INF("=== UNILATERAL MODE (Right Foot Only) ===");
            LOG_INF("D2D Status: %s", left_metrics_fresh ? "CONNECTED" : "NO DATA FROM SECONDARY");
            LOG_INF("  Bilateral metrics not available");
            LOG_INF("  Continuing with right foot metrics only");
            LOG_INF("=============================================");
            
            // Zero out bilateral results when no secondary connected
            bilateral_results.gait_symmetry_index = 0;
            bilateral_results.phase_coordination = 0;
            bilateral_results.load_distribution = 0;
            bilateral_results.last_calculation_ms = k_uptime_get_32();
        } else {
            LOG_DBG("Waiting for metrics - Left:%s, Right:%s",
                    left_metrics_valid ? "valid" : "invalid",
                    right_metrics_valid ? "valid" : "invalid");
        }
    }
#endif
}

/**
 * @brief Process received D2D metrics from secondary
 */
void bilateral_metrics_process_d2d(const d2d_metrics_packet_t *metrics)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!metrics) {
        LOG_ERR("Received NULL metrics packet");
        return;
    }
    
    // Store left foot metrics
    memcpy(&left_foot_metrics, metrics, sizeof(d2d_metrics_packet_t));
    left_metrics_valid = true;
    
    LOG_INF("=== RECEIVED D2D METRICS FROM SECONDARY (Left Foot) ===");
    LOG_INF("Timestamp: %u ms, Sequence: %u, Status: %u",
            metrics->timestamp, metrics->sequence_num, metrics->calculation_status);
    
    // Log ALL received metrics from secondary
    if (d2d_metric_is_valid(metrics, IDX_GCT)) {
        LOG_INF("  [D2D] Ground Contact Time: %.1f ms", (double)metrics->metrics[IDX_GCT]);
    }
    if (d2d_metric_is_valid(metrics, IDX_STRIDE_LENGTH)) {
        LOG_INF("  [D2D] Stride Length: %.2f cm", (double)metrics->metrics[IDX_STRIDE_LENGTH]);
    }
    if (d2d_metric_is_valid(metrics, IDX_CADENCE)) {
        LOG_INF("  [D2D] Cadence: %.1f steps/min", (double)metrics->metrics[IDX_CADENCE]);
    }
    if (d2d_metric_is_valid(metrics, IDX_PRONATION)) {
        LOG_INF("  [D2D] Pronation: %.1f degrees", (double)metrics->metrics[IDX_PRONATION]);
    }
    if (d2d_metric_is_valid(metrics, IDX_COP_X) && d2d_metric_is_valid(metrics, IDX_COP_Y)) {
        LOG_INF("  [D2D] Center of Pressure: X=%.1f mm, Y=%.1f mm",
                (double)metrics->metrics[IDX_COP_X], (double)metrics->metrics[IDX_COP_Y]);
    }
    if (d2d_metric_is_valid(metrics, IDX_PEAK_PRESSURE)) {
        LOG_INF("  [D2D] Peak Pressure: %.1f kPa", (double)metrics->metrics[IDX_PEAK_PRESSURE]);
    }
    if (d2d_metric_is_valid(metrics, IDX_STANCE_TIME)) {
        LOG_INF("  [D2D] Stance Time: %.1f ms", (double)metrics->metrics[IDX_STANCE_TIME]);
    }
    if (d2d_metric_is_valid(metrics, IDX_SWING_TIME)) {
        LOG_INF("  [D2D] Swing Time: %.1f ms", (double)metrics->metrics[IDX_SWING_TIME]);
    }
    if (d2d_metric_is_valid(metrics, IDX_STEP_COUNT)) {
        LOG_INF("  [D2D] Step Count: %u", (uint32_t)metrics->metrics[IDX_STEP_COUNT]);
    }
    LOG_INF("=============================================");
#endif
}

/**
 * @brief Start bilateral metrics processing
 */
static void bilateral_metrics_start(void)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (activity_active) {
        LOG_DBG("Bilateral metrics already active");
        return;
    }
    
    LOG_INF("Starting bilateral metrics processing");
    
    // Clear state
    left_metrics_valid = false;
    right_metrics_valid = false;
    primary_sequence_num = 0;
    memset(&bilateral_results, 0, sizeof(bilateral_results));
    
    // Start timer for periodic calculations (3s warmup, then 1Hz)
    k_timer_start(&primary_metrics_timer, K_SECONDS(3), K_SECONDS(1));
    activity_active = true;
    
    LOG_INF("Bilateral metrics started - timer will fire in 3 seconds");
#endif
}

/**
 * @brief Stop bilateral metrics processing
 */
static void bilateral_metrics_stop(void)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!activity_active) {
        LOG_DBG("Bilateral metrics already stopped");
        return;
    }
    
    LOG_INF("Stopping bilateral metrics processing");
    
    // Stop timer
    k_timer_stop(&primary_metrics_timer);
    
    // Clear state
    left_metrics_valid = false;
    right_metrics_valid = false;
    activity_active = false;
    
    LOG_INF("Bilateral metrics stopped");
#endif
}

/**
 * @brief Thread function for processing analytics messages
 */
static void bilateral_metrics_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    generic_message_t msg;
    
    LOG_INF("Bilateral metrics thread started");
    
    while (1) {
        // Wait for messages
        if (k_msgq_get(&analytics_msgq, &msg, K_FOREVER) == 0) {
            switch (msg.type) {
                case MSG_TYPE_D2D_METRICS_RECEIVED:
                    // Only process if activity is active
                    if (activity_active) {
                        bilateral_metrics_process_d2d(&msg.data.d2d_metrics);
                    } else {
                        LOG_DBG("Ignoring D2D metrics - activity not active");
                    }
                    break;
                    
                case MSG_TYPE_COMMAND:
                    // Handle start/stop commands - listen for REALTIME_PROCESSING commands
                    // These are sent by analytics.cpp when activity starts/stops
                    if (strncmp(msg.data.command_str, "START_REALTIME_PROCESSING", MAX_COMMAND_STRING_LEN) == 0) {
                        LOG_INF("Bilateral metrics received START_REALTIME_PROCESSING command");
                        bilateral_metrics_start();
                    } else if (strncmp(msg.data.command_str, "STOP_REALTIME_PROCESSING", MAX_COMMAND_STRING_LEN) == 0) {
                        LOG_INF("Bilateral metrics received STOP_REALTIME_PROCESSING command");
                        bilateral_metrics_stop();
                    }
                    break;
                    
                default:
                    // Ignore other message types
                    break;
            }
        }
    }
#endif
}

// Thread stack and data
K_THREAD_STACK_DEFINE(bilateral_metrics_stack, 2048);
static struct k_thread bilateral_metrics_thread_data;

/**
 * @brief Initialize bilateral metrics processing
 */
void bilateral_metrics_init(void)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    LOG_INF("Initializing bilateral metrics processing (primary device)");
    
    // Clear initial state
    memset(&bilateral_results, 0, sizeof(bilateral_results));
    left_metrics_valid = false;
    right_metrics_valid = false;
    
    // Create processing thread
    k_thread_create(&bilateral_metrics_thread_data,
                    bilateral_metrics_stack,
                    K_THREAD_STACK_SIZEOF(bilateral_metrics_stack),
                    bilateral_metrics_thread,
                    NULL, NULL, NULL,
                    7, // Priority
                    0,
                    K_NO_WAIT);
    
    k_thread_name_set(&bilateral_metrics_thread_data, "bilateral_metrics");
    
    // Initialize timer but don't start it yet
    k_timer_init(&primary_metrics_timer, primary_metrics_timer_callback, NULL);
    
    LOG_INF("Bilateral metrics initialized - waiting for START_REALTIME_PROCESSING command");
#else
    LOG_INF("Bilateral metrics disabled on secondary device");
#endif
}

/**
 * @brief Get current bilateral metrics results
 */
void bilateral_metrics_get_results(bilateral_metrics_results_t *results)
{
    if (results) {
        results->gait_symmetry = bilateral_results.gait_symmetry_index;
        results->phase_coordination = bilateral_results.phase_coordination;
        results->load_distribution = bilateral_results.load_distribution;
        results->valid = (bilateral_results.last_calculation_ms > 0 &&
                         (k_uptime_get_32() - bilateral_results.last_calculation_ms) < 5000);
    }
}
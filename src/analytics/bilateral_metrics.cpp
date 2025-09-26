/**
 * @file bilateral_metrics.cpp
 * @brief Primary device bilateral metrics processing
 * 
 * This module receives pre-calculated metrics from gait events (right foot) and 
 * D2D (left foot), then combines them for bilateral gait analysis.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

#include <app.hpp>
#include <d2d_metrics.h>
#include "bilateral_metrics.h"
#include "../realtime_metrics/realtime_metrics.h"

LOG_MODULE_REGISTER(bilateral_metrics, CONFIG_ANALYTICS_MODULE_LOG_LEVEL);

// Message queue for analytics module (only on primary device)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
K_MSGQ_DEFINE(analytics_msgq, sizeof(generic_message_t), 10, 4);
#endif

// Store latest metrics from both feet
static d2d_metrics_packet_t left_foot_metrics;  // From secondary via D2D
static d2d_metrics_packet_t right_foot_metrics; // From local gait events module
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

static bool activity_active = false;  // Track if activity is running

/**
 * @brief Receive pre-calculated right foot metrics from gait events module
 * This replaces the duplicate calculation - we consume instead of recalculate
 */
void bilateral_metrics_update_right_foot(const gait_metrics_t *right_metrics)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!right_metrics || !right_metrics->valid) {
        LOG_DBG("Invalid right foot metrics received");
        right_metrics_valid = false;
        return;
    }
    
    // Store the pre-calculated metrics (no recalculation needed)
    right_foot_metrics.timestamp = (uint32_t)(right_metrics->timestamp * 1000); // s → ms
    static uint16_t right_sequence_num = 0;
    right_foot_metrics.sequence_num = right_sequence_num++;
    
    // Clear previous metrics
    d2d_metrics_clear(&right_foot_metrics);
    
    // Use the already-calculated values from gait_events
    right_foot_metrics.metrics[IDX_GCT] = right_metrics->gct * 1000.0f; // s → ms
    d2d_metric_set_valid(&right_foot_metrics, IDX_GCT);
    
    right_foot_metrics.metrics[IDX_STRIDE_LENGTH] = right_metrics->stride_length * 100.0f; // m → cm
    d2d_metric_set_valid(&right_foot_metrics, IDX_STRIDE_LENGTH);
    
    right_foot_metrics.metrics[IDX_CADENCE] = right_metrics->cadence;
    d2d_metric_set_valid(&right_foot_metrics, IDX_CADENCE);
    
    right_foot_metrics.metrics[IDX_PRONATION] = right_metrics->max_pronation;
    d2d_metric_set_valid(&right_foot_metrics, IDX_PRONATION);
    
    // Add other metrics if available
    if (right_metrics->peak_pressure > 0) {
        right_foot_metrics.metrics[IDX_PEAK_PRESSURE] = (float)right_metrics->peak_pressure;
        d2d_metric_set_valid(&right_foot_metrics, IDX_PEAK_PRESSURE);
    }
    
    // Add timing metrics if available
    if (right_metrics->duration > 0) {
        right_foot_metrics.metrics[IDX_STANCE_TIME] = right_metrics->gct * 1000.0f;
        right_foot_metrics.metrics[IDX_SWING_TIME] = (right_metrics->duration - right_metrics->gct) * 1000.0f;
        d2d_metric_set_valid(&right_foot_metrics, IDX_STANCE_TIME);
        d2d_metric_set_valid(&right_foot_metrics, IDX_SWING_TIME);
    }
    
    right_metrics_valid = true;
    right_foot_metrics.calculation_status = 0; // OK
    
    LOG_DBG("Right foot metrics updated: GCT=%.1fms, cadence=%.1f, stride=%.2fm", 
            (double)(right_metrics->gct * 1000.0f), 
            (double)right_metrics->cadence,
            (double)right_metrics->stride_length);
    
    // Trigger bilateral calculation if we have both feet data
    if (left_metrics_valid) {
        calculate_bilateral_metrics();
    }
#endif
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
            LOG_DBG("GCT symmetry: L=%.1fms, R=%.1fms, symmetry=%.1f%%", 
                    (double)left_gct, (double)right_gct, (double)gct_symmetry);
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
            LOG_DBG("Stride symmetry: L=%.1fcm, R=%.1fcm, symmetry=%.1f%%", 
                    (double)left_stride, (double)right_stride, (double)stride_symmetry);
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
            LOG_DBG("Cadence symmetry: L=%.1fspm, R=%.1fspm, symmetry=%.1f%%", 
                    (double)left_cadence, (double)right_cadence, (double)cadence_symmetry);
        }
    }
    
    float overall_symmetry = (symmetry_count > 0) ? (symmetry_sum / symmetry_count) : 0;
    LOG_DBG("Overall gait symmetry: %.1f%% (based on %d metrics)", 
            (double)overall_symmetry, symmetry_count);
    
    return overall_symmetry;
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
    
    // Use stance/swing times if available, otherwise estimate from GCT
    float left_stance, right_stance, left_swing, right_swing;
    
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_STANCE_TIME) &&
        d2d_metric_is_valid(&left_foot_metrics, IDX_SWING_TIME) &&
        d2d_metric_is_valid(&right_foot_metrics, IDX_STANCE_TIME) &&
        d2d_metric_is_valid(&right_foot_metrics, IDX_SWING_TIME)) {
        
        // Use precise timing data
        left_stance = left_foot_metrics.metrics[IDX_STANCE_TIME];
        left_swing = left_foot_metrics.metrics[IDX_SWING_TIME];
        right_stance = right_foot_metrics.metrics[IDX_STANCE_TIME];
        right_swing = right_foot_metrics.metrics[IDX_SWING_TIME];
    } else {
        // Estimate from GCT and cadence
        float left_gct = d2d_metric_is_valid(&left_foot_metrics, IDX_GCT) ? 
                        left_foot_metrics.metrics[IDX_GCT] : 250.0f;
        float right_gct = d2d_metric_is_valid(&right_foot_metrics, IDX_GCT) ? 
                         right_foot_metrics.metrics[IDX_GCT] : 250.0f;
        
        // Estimate cycle time from cadence
        float avg_cadence = 160.0f; // Default
        if (d2d_metric_is_valid(&left_foot_metrics, IDX_CADENCE) &&
            d2d_metric_is_valid(&right_foot_metrics, IDX_CADENCE)) {
            avg_cadence = (left_foot_metrics.metrics[IDX_CADENCE] + 
                          right_foot_metrics.metrics[IDX_CADENCE]) / 2.0f;
        }
        
        float cycle_time = 60000.0f / avg_cadence; // ms per cycle
        left_swing = cycle_time - left_gct;
        right_swing = cycle_time - right_gct;
        left_stance = left_gct;
        right_stance = right_gct;
    }
    
    // Calculate phase relationship
    float left_cycle = left_stance + left_swing;
    float right_cycle = right_stance + right_swing;
    
    if (left_cycle > 0 && right_cycle > 0) {
        // Calculate phase offset (should be ~0.5 for normal gait)
        float left_phase = left_stance / left_cycle;
        float right_phase = right_stance / right_cycle;
        float phase_offset = fabsf(left_phase - right_phase);
        float ideal_offset = 0.5f;
        float phase_error = fabsf(phase_offset - ideal_offset);
        
        // Convert to 0-100 scale (100 = perfect coordination)
        float coordination = 100.0f * (1.0f - 2.0f * phase_error);
        coordination = MAX(0.0f, MIN(100.0f, coordination));
        
        LOG_DBG("Phase coordination: L_phase=%.2f, R_phase=%.2f, error=%.2f, coordination=%.1f", 
                (double)left_phase, (double)right_phase, (double)phase_error, (double)coordination);
        
        return coordination;
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
            LOG_DBG("Load distribution from pressure: L=%.0f, R=%.0f, dist=%.1f%%", 
                    (double)left_pressure, (double)right_pressure, (double)distribution);
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
            LOG_DBG("Load distribution from GCT: L=%.1fms, R=%.1fms, dist=%.1f%%", 
                    (double)left_gct, (double)right_gct, (double)distribution);
            return distribution;
        }
    }
    
    return 0;
}

/**
 * @brief Calculate flight time and other bilateral timing metrics
 */
static void calculate_timing_metrics(float *flight_time, float *double_support_time)
{
    if (flight_time) *flight_time = 0;
    if (double_support_time) *double_support_time = 0;
    
    if (!left_metrics_valid || !right_metrics_valid) return;
    
    // Get GCT from both feet
    float left_gct = d2d_metric_is_valid(&left_foot_metrics, IDX_GCT) ? 
                    left_foot_metrics.metrics[IDX_GCT] : 0;
    float right_gct = d2d_metric_is_valid(&right_foot_metrics, IDX_GCT) ? 
                     right_foot_metrics.metrics[IDX_GCT] : 0;
    
    // Estimate stride duration from cadence
    float avg_cadence = 160.0f;
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_CADENCE) &&
        d2d_metric_is_valid(&right_foot_metrics, IDX_CADENCE)) {
        avg_cadence = (left_foot_metrics.metrics[IDX_CADENCE] + 
                      right_foot_metrics.metrics[IDX_CADENCE]) / 2.0f;
    }
    
    float stride_duration = 60000.0f / avg_cadence; // ms
    
    // Calculate flight time: stride_duration - (GCT_left + GCT_right)
    if (flight_time && left_gct > 0 && right_gct > 0) {
        *flight_time = stride_duration - (left_gct + right_gct);
        *flight_time = MAX(0.0f, *flight_time); // Ensure non-negative
    }
    
    // Estimate double support time (simplified)
    if (double_support_time) {
        // Double support occurs during gait transitions
        // Simplified: assume 10% of stride duration as double support
        *double_support_time = stride_duration * 0.1f;
    }
}

/**
 * @brief Main bilateral metrics calculation
 */
static void calculate_bilateral_metrics(void)
{
    if (!left_metrics_valid || !right_metrics_valid) {
        LOG_DBG("Cannot calculate bilateral metrics - missing foot data");
        return;
    }
    
    // Check if metrics are reasonably recent (within 5 seconds)
    uint32_t current_time = k_uptime_get_32();
    uint32_t left_age = current_time - left_foot_metrics.timestamp;
    uint32_t right_age = current_time - right_foot_metrics.timestamp;
    
    if (left_age > 5000 || right_age > 5000) {
        LOG_WRN("Bilateral metrics stale: L_age=%ums, R_age=%ums", left_age, right_age);
        if (left_age > 5000) left_metrics_valid = false;
        if (right_age > 5000) right_metrics_valid = false;
        return;
    }
    
    // Calculate all bilateral metrics
    bilateral_results.gait_symmetry_index = calculate_gait_symmetry();
    bilateral_results.phase_coordination = calculate_phase_coordination();
    bilateral_results.load_distribution = calculate_load_distribution();
    bilateral_results.last_calculation_ms = current_time;
    
    // Calculate timing metrics
    float flight_time, double_support_time;
    calculate_timing_metrics(&flight_time, &double_support_time);
    
    // Comprehensive logging
    LOG_INF("=== BILATERAL METRICS ANALYSIS ===");
    LOG_INF("Gait Symmetry: %.1f%% (100%% = perfect)", (double)bilateral_results.gait_symmetry_index);
    LOG_INF("Phase Coordination: %.1f/100", (double)bilateral_results.phase_coordination);
    LOG_INF("Load Distribution: %.1f%%", (double)bilateral_results.load_distribution);
    LOG_INF("Flight Time: %.1fms, Double Support: %.1fms", (double)flight_time, (double)double_support_time);
    
    // Log individual foot metrics
    LOG_INF("--- Left Foot (Secondary) ---");
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_GCT)) {
        LOG_INF("  GCT: %.1fms", (double)left_foot_metrics.metrics[IDX_GCT]);
    }
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_CADENCE)) {
        LOG_INF("  Cadence: %.1fspm", (double)left_foot_metrics.metrics[IDX_CADENCE]);
    }
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_STRIDE_LENGTH)) {
        LOG_INF("  Stride: %.1fcm", (double)left_foot_metrics.metrics[IDX_STRIDE_LENGTH]);
    }
    
    LOG_INF("--- Right Foot (Primary) ---");
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_GCT)) {
        LOG_INF("  GCT: %.1fms", (double)right_foot_metrics.metrics[IDX_GCT]);
    }
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_CADENCE)) {
        LOG_INF("  Cadence: %.1fspm", (double)right_foot_metrics.metrics[IDX_CADENCE]);
    }
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_STRIDE_LENGTH)) {
        LOG_INF("  Stride: %.1fcm", (double)right_foot_metrics.metrics[IDX_STRIDE_LENGTH]);
    }
    
    // Calculate and log asymmetry values
    if (d2d_metric_is_valid(&left_foot_metrics, IDX_GCT) &&
        d2d_metric_is_valid(&right_foot_metrics, IDX_GCT)) {
        float gct_diff = right_foot_metrics.metrics[IDX_GCT] - left_foot_metrics.metrics[IDX_GCT];
        float gct_asymmetry = fabsf(gct_diff) / ((left_foot_metrics.metrics[IDX_GCT] + 
                                                right_foot_metrics.metrics[IDX_GCT]) / 2.0f) * 100.0f;
        LOG_INF("GCT Asymmetry: %.1fms (R-L), %.1f%%", (double)gct_diff, (double)gct_asymmetry);
    }
    
    LOG_INF("===================================");
    
    // Send consolidated metrics to realtime module
    send_consolidated_metrics();
}

/**
 * @brief Send consolidated metrics to realtime metrics module
 */
static void send_consolidated_metrics(void)
{
    realtime_metrics_t consolidated = {};
    
    // Always send right foot metrics (local data)
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_GCT)) {
        consolidated.ground_contact_ms = right_foot_metrics.metrics[IDX_GCT];
    }
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_STRIDE_LENGTH)) {
        consolidated.stride_length_cm = right_foot_metrics.metrics[IDX_STRIDE_LENGTH];
    }
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_CADENCE)) {
        consolidated.cadence_spm = right_foot_metrics.metrics[IDX_CADENCE];
    }
    if (d2d_metric_is_valid(&right_foot_metrics, IDX_PRONATION)) {
        consolidated.avg_pronation_deg = right_foot_metrics.metrics[IDX_PRONATION];
    }
    
    // Include bilateral metrics if both feet valid
    if (left_metrics_valid && right_metrics_valid) {
        // Convert symmetry to balance metric (-50 to +50)
        consolidated.balance_lr_pct = (int8_t)((100.0f - bilateral_results.gait_symmetry_index) / 2.0f);
        if (bilateral_results.load_distribution > 0) {
            consolidated.balance_lr_pct = -consolidated.balance_lr_pct; // Right-heavy is negative
        }
    } else {
        consolidated.balance_lr_pct = 0;
    }
    
    // Send to sensor data queue
    generic_message_t msg = {};
    msg.sender = SENDER_ANALYTICS;
    msg.type = MSG_TYPE_REALTIME_METRICS;
    memcpy(&msg.data.realtime_metrics, &consolidated, sizeof(realtime_metrics_t));
    
    extern struct k_msgq sensor_data_queue;
    if (k_msgq_put(&sensor_data_queue, &msg, K_NO_WAIT) != 0) {
        LOG_ERR("Failed to send consolidated metrics to realtime module");
    } else {
        LOG_DBG("Sent consolidated metrics to realtime module");
    }
}

/**
 * @brief Process received D2D metrics from secondary (left foot)
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
    
    LOG_INF("=== RECEIVED LEFT FOOT METRICS VIA D2D ===");
    LOG_INF("Timestamp: %u ms, Sequence: %u, Status: %u",
            metrics->timestamp, metrics->sequence_num, metrics->calculation_status);
    
    // Log received metrics
    if (d2d_metric_is_valid(metrics, IDX_GCT)) {
        LOG_INF("  GCT: %.1f ms", (double)metrics->metrics[IDX_GCT]);
    }
    if (d2d_metric_is_valid(metrics, IDX_STRIDE_LENGTH)) {
        LOG_INF("  Stride Length: %.2f cm", (double)metrics->metrics[IDX_STRIDE_LENGTH]);
    }
    if (d2d_metric_is_valid(metrics, IDX_CADENCE)) {
        LOG_INF("  Cadence: %.1f spm", (double)metrics->metrics[IDX_CADENCE]);
    }
    if (d2d_metric_is_valid(metrics, IDX_PRONATION)) {
        LOG_INF("  Pronation: %.1f deg", (double)metrics->metrics[IDX_PRONATION]);
    }
    LOG_INF("===========================================");
    
    // Trigger bilateral calculation if we have both feet data
    if (right_metrics_valid) {
        calculate_bilateral_metrics();
    }
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
    memset(&bilateral_results, 0, sizeof(bilateral_results));
    
    activity_active = true;
    LOG_INF("Bilateral metrics started - waiting for foot metrics");
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
                    // Process left foot metrics from secondary
                    if (activity_active) {
                        bilateral_metrics_process_d2d(&msg.data.d2d_metrics);
                    }
                    break;
                    
                case MSG_TYPE_COMMAND:
                    // Handle start/stop commands
                    if (strncmp(msg.data.command_str, "START_REALTIME_PROCESSING", MAX_COMMAND_STRING_LEN) == 0) {
                        bilateral_metrics_start();
                    } else if (strncmp(msg.data.command_str, "STOP_REALTIME_PROCESSING", MAX_COMMAND_STRING_LEN) == 0) {
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
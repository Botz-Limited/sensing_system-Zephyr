/**
 * @file analytics.cpp
 * @brief Complex analytics calculation module (1-5Hz)
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#define MODULE analytics

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <string.h>
#include <math.h>

#include <app.hpp>
#include <events/app_state_event.h>
#include <errors.hpp>

LOG_MODULE_REGISTER(MODULE, CONFIG_ANALYTICS_MODULE_LOG_LEVEL);

// Include Choros buffer for enhanced algorithms (safely added)
#include <choros_buffer.hpp>

extern void choros_buffer_init(void);

// Global Choros buffer instance definition
#if CHOROS_ENABLED
choros_ring_buffer_t choros_buffer;
#endif

// Thread configuration
static constexpr int analytics_stack_size = CONFIG_ANALYTICS_MODULE_STACK_SIZE;
static constexpr int analytics_priority = CONFIG_ANALYTICS_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(analytics_stack_area, analytics_stack_size);
static struct k_thread analytics_thread_data;
static k_tid_t analytics_tid;

// Work queue configuration
static constexpr int analytics_workq_stack_size = 4096;
K_THREAD_STACK_DEFINE(analytics_workq_stack, analytics_workq_stack_size);
static struct k_work_q analytics_work_q;

// Work items for different message types
static struct k_work process_realtime_metrics_work;
static struct k_work process_command_work;
static struct k_work_delayable analytics_periodic_work;

// Message buffers for different work items
static char pending_command[MAX_COMMAND_STRING_LEN];
// Flag to indicate new metrics are available
static bool new_metrics_available;

// Message queues are defined in app.cpp
extern struct k_msgq realtime_queue;    // Input from realtime_metrics
extern struct k_msgq analytics_queue;   // Output to activity_metrics

// Module state
static bool module_initialized = false;
static atomic_t processing_active = ATOMIC_INIT(0);

// Metric history for baseline and trend analysis
#define METRIC_HISTORY_SIZE 120  // 2 minutes at 1Hz
static realtime_metrics_t metric_history[METRIC_HISTORY_SIZE];
static uint8_t history_write_index = 0;
static uint16_t history_count = 0;

// Analytics state
static struct {
    // Baseline tracking
    bool baseline_established;
    uint32_t baseline_start_time;
    float baseline_contact_time;
    float baseline_efficiency;
    float baseline_cadence;
    float baseline_pronation;
    
    // Complex metrics
    float running_efficiency;
    float fatigue_index;
    float injury_risk;
    float stride_length;
    float pronation_angle;
    
    // Processing counters
    uint32_t analytics_count;
    uint32_t last_analytics_time;
    
    // Latest metrics from realtime module
    realtime_metrics_t latest_metrics;
    bool metrics_updated;
} analytics_state;


// Forward declarations
static void analytics_init(void);
static float calculate_fatigue_index_placeholder(void);
static float calculate_injury_risk_placeholder(void);
static float calculate_stride_length_placeholder(void);
static float calculate_pronation_analysis_placeholder(void);
static float calculate_vertical_stiffness_placeholder(void);
static float calculate_recovery_score_placeholder(void);
/**
 * @brief Main processing thread for analytics, waits for messages and queues work.
 * @param arg1 Unused.
 * @param arg2 Unused.
 * @param arg3 Unused.
 * @note This function is complete and handles incoming messages for real-time metrics and commands.
 *       Data Requirements: Indirectly relies on data from motion and foot sensors via real-time metrics.
 */
static void analytics_thread_fn(void *arg1, void *arg2, void *arg3);
/**
 * @brief Performs complex analytics calculations for efficiency, fatigue, and injury risk.
 * @note This function is a placeholder with TODO for actual complex calculations. Currently simulates results.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet for comprehensive analysis).
 */
static void perform_complex_analytics(void);
/**
 * @brief Establishes baseline metrics during the initial period of activity.
 * @note This function is a placeholder with TODO for accumulating baseline metrics. Currently logs progress only.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet for baseline metrics).
 */
static void establish_baseline(void);
/**
 * @brief Work handler for processing real-time metrics data.
 * @param work Pointer to the work item.
 * @note This function is complete and processes real-time metrics at a controlled rate, establishing baselines.
 *       Data Requirements: Indirectly relies on data from motion and foot sensors via real-time metrics.
 */
static void process_realtime_metrics_work_handler(struct k_work *work);
/**
 * @brief Work handler for processing commands such as start/stop analytics.
 * @param work Pointer to the work item.
 * @note This function is complete and handles commands to start or stop analytics processing.
 *       Data Requirements: None directly, but commands trigger processing of sensor data.
 */
static void process_command_work_handler(struct k_work *work);
/**
 * @brief Work handler for periodic analytics tasks.
 * @param work Pointer to the work item.
 * @note This function is a placeholder with no specific tasks defined yet, just reschedules itself.
 *       Data Requirements: None currently, but could involve motion and foot sensor data in future implementations.
 */
static void analytics_periodic_work_handler(struct k_work *work);

// Initialize the analytics module
static void analytics_init(void)
{
    LOG_INF("Analytics init called - starting initialization");
    
    // Initialize state
    memset(&analytics_state, 0, sizeof(analytics_state));
    
    // Initialize Choros buffer (safe addition)
    choros_buffer_init();
    
    // Initialize work queue
    k_work_queue_init(&analytics_work_q);
    k_work_queue_start(&analytics_work_q, analytics_workq_stack,
                       K_THREAD_STACK_SIZEOF(analytics_workq_stack),
                       analytics_priority - 1, NULL);
    k_thread_name_set(&analytics_work_q.thread, "analytics_wq");
    
    // Initialize work items
    k_work_init(&process_realtime_metrics_work, process_realtime_metrics_work_handler);
    k_work_init(&process_command_work, process_command_work_handler);
    k_work_init_delayable(&analytics_periodic_work, analytics_periodic_work_handler);
    
    // Create the message processing thread
    analytics_tid = k_thread_create(
        &analytics_thread_data,
        analytics_stack_area,
        K_THREAD_STACK_SIZEOF(analytics_stack_area),
        analytics_thread_fn,
        NULL, NULL, NULL,
        analytics_priority,
        0,
        K_NO_WAIT
    );
    
    k_thread_name_set(analytics_tid, "analytics");
    
    module_initialized = true;
    module_set_state(MODULE_STATE_READY);
    LOG_INF("Analytics module initialized");
}

// Main processing thread - waits for messages and queues appropriate work
static void analytics_thread_fn(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    
    generic_message_t msg;
    
    while (true) {
        // Wait for messages from realtime queue (for metrics) and analytics queue (for commands)
        // Try realtime queue first for metrics data
        int ret = k_msgq_get(&realtime_queue, &msg, K_MSEC(10));
        if (ret != 0) {
            // If no realtime data, check analytics queue for commands
            ret = k_msgq_get(&analytics_queue, &msg, K_MSEC(100));
        }
        
        if (ret == 0) {
           // LOG_WRN("Analytics: Received message type %d from sender %d", msg.type, msg.sender);
            // Queue different work based on message type
            switch (msg.type) {
                case MSG_TYPE_REALTIME_METRICS:
                    // Copy the metrics data from the message
                    memcpy(&analytics_state.latest_metrics, &msg.data.realtime_metrics, 
                           sizeof(realtime_metrics_t));
                    analytics_state.metrics_updated = true;
                    
                    // Store in history buffer
                    memcpy(&metric_history[history_write_index], &msg.data.realtime_metrics,
                           sizeof(realtime_metrics_t));
                    history_write_index = (history_write_index + 1) % METRIC_HISTORY_SIZE;
                    if (history_count < METRIC_HISTORY_SIZE) {
                        history_count++;
                    }
                    
                    // Add to Choros buffer if enabled (safe parallel processing)
                    #if CHOROS_ENABLED
                    if (msg.sender == SENDER_SENSOR_DATA && msg.type == MSG_TYPE_SENSOR_DATA_CONSOLIDATED) {
                        // Store consolidated data in Choros buffer for enhanced algorithms
                        choros_buffer_add_consolidated(&msg.data.sensor_consolidated);
                    }
                    #endif
                    
                    // Mark that new metrics are available
                    new_metrics_available = true;
                    k_work_submit_to_queue(&analytics_work_q, &process_realtime_metrics_work);
                    break;
                    
                case MSG_TYPE_COMMAND:
                    // Copy command and queue command processing work
                    strncpy(pending_command, msg.data.command_str, MAX_COMMAND_STRING_LEN - 1);
                    pending_command[MAX_COMMAND_STRING_LEN - 1] = '\0';
                    k_work_submit_to_queue(&analytics_work_q, &process_command_work);
                    break;
                    
                default:
                    // Only log unknown message types, not expected ones
                    LOG_DBG("Received message type %d from %s", msg.type, get_sender_name(msg.sender));
                    break;
            }
        }
    }
}

// Work handler for processing realtime metrics
static void process_realtime_metrics_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    const uint32_t ANALYTICS_INTERVAL_MS = 200; // 5Hz max
    const uint32_t BASELINE_DURATION_MS = 120000; // 2 minutes
    
    if (atomic_get(&processing_active) == 1 && new_metrics_available) {
        uint32_t now = k_uptime_get_32();
        
      //  LOG_WRN("Processing realtime metrics");
        
        // Establish baseline during first 2 minutes
        if (!analytics_state.baseline_established) {
            if (now - analytics_state.baseline_start_time < BASELINE_DURATION_MS) {
                establish_baseline();
            } else {
                analytics_state.baseline_established = true;
                //LOG_INF("Baseline established after 2 minutes");
            }
        }
        
        // Perform analytics at controlled rate
        if (now - analytics_state.last_analytics_time >= ANALYTICS_INTERVAL_MS) {
            perform_complex_analytics();
            analytics_state.last_analytics_time = now;
            analytics_state.analytics_count++;
        }
        
        // Clear the flag
        new_metrics_available = false;
    }
}

// Work handler for processing commands
static void process_command_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
   // LOG_WRN("Processing command: %s", pending_command);
    
    if (strcmp(pending_command, "START_REALTIME_PROCESSING") == 0) {
        // START: Begin new session with state reset
        // Clear any pending messages before starting
        generic_message_t dummy_msg;
        while (k_msgq_get(&analytics_queue, &dummy_msg, K_NO_WAIT) == 0) {
            // Discard message
        }
        
        // Reset state for clean start
        memset(&analytics_state, 0, sizeof(analytics_state));
        history_write_index = 0;
        history_count = 0;
        new_metrics_available = false;
        
        // Analytics should start when realtime metrics start
        atomic_set(&processing_active, 1);
        analytics_state.baseline_start_time = k_uptime_get_32();
        LOG_INF("Analytics processing started (via START_REALTIME_PROCESSING) - buffers cleared");
        // Start periodic work
        k_work_schedule_for_queue(&analytics_work_q, &analytics_periodic_work, K_MSEC(200));
        
        // ALSO forward this command for realtime_metrics module
        // Send to sensor_data_queue which is the input queue for realtime_metrics
        generic_message_t fwd_msg = {};
        fwd_msg.sender = SENDER_ACTIVITY_METRICS;
        fwd_msg.type = MSG_TYPE_COMMAND;
        strcpy(fwd_msg.data.command_str, pending_command);
        extern struct k_msgq sensor_data_queue;
        k_msgq_put(&sensor_data_queue, &fwd_msg, K_NO_WAIT);
        LOG_INF("Also forwarded START_REALTIME_PROCESSING for realtime_metrics");
    } else if (strcmp(pending_command, "STOP_REALTIME_PROCESSING") == 0) {
        // STOP: End session completely with state reset
        // Analytics should stop when realtime metrics stop
        atomic_set(&processing_active, 0);
        LOG_INF("Analytics processing stopped");
        // Cancel periodic work
        k_work_cancel_delayable(&analytics_periodic_work);
        
        // Clear all pending messages from both queues
        generic_message_t dummy_msg;
        while (k_msgq_get(&analytics_queue, &dummy_msg, K_NO_WAIT) == 0) {
            // Discard message
        }
        while (k_msgq_get(&realtime_queue, &dummy_msg, K_NO_WAIT) == 0) {
            // Discard message
        }
        LOG_INF("Analytics and realtime queues cleared");
        
        // Reset state for clean restart
        memset(&analytics_state, 0, sizeof(analytics_state));
        history_write_index = 0;
        history_count = 0;
        new_metrics_available = false;
        LOG_INF("Analytics state reset for clean restart");
        
        // ALSO forward this command for realtime_metrics module
        // Send to sensor_data_queue which is the input queue for realtime_metrics
        generic_message_t fwd_msg = {};
        fwd_msg.sender = SENDER_ACTIVITY_METRICS;
        fwd_msg.type = MSG_TYPE_COMMAND;
        strcpy(fwd_msg.data.command_str, pending_command);
        extern struct k_msgq sensor_data_queue;
        k_msgq_put(&sensor_data_queue, &fwd_msg, K_NO_WAIT);
        LOG_INF("Also forwarded STOP_REALTIME_PROCESSING for realtime_metrics");
    } else if (strcmp(pending_command, "PAUSE_ANALYTICS") == 0) {
        // PAUSE: Suspend processing but preserve state
        atomic_set(&processing_active, 0);
        k_work_cancel_delayable(&analytics_periodic_work);
        LOG_INF("Analytics processing PAUSED - state preserved");
        LOG_INF("  History buffer: %u samples preserved", history_count);
        LOG_INF("  Baseline data preserved: contact=%.1f, cadence=%.1f",
                (double)analytics_state.baseline_contact_time,
                (double)analytics_state.baseline_cadence);
        
        // NOTE: Do NOT reset analytics_state or history buffers
    } else if (strcmp(pending_command, "UNPAUSE_ANALYTICS") == 0) {
        // UNPAUSE: Resume processing with preserved state
        atomic_set(&processing_active, 1);
        LOG_INF("Analytics processing UNPAUSED - continuing with preserved state");
        LOG_INF("  History buffer: %u samples available", history_count);
        LOG_INF("  Baseline: %s",
                analytics_state.baseline_established ? "established" : "in progress");
        // Resume periodic work
        k_work_schedule_for_queue(&analytics_work_q, &analytics_periodic_work, K_MSEC(200));
    } else if (strcmp(pending_command, "START_SENSOR_PROCESSING") == 0) {
        // Analytics doesn't directly handle sensor processing, but should forward
        LOG_INF("Forwarding START_SENSOR_PROCESSING command");
        // Forward to sensor_data module via appropriate queue
        generic_message_t fwd_msg = {};
        fwd_msg.sender = SENDER_ANALYTICS;
        fwd_msg.type = MSG_TYPE_COMMAND;
        strcpy(fwd_msg.data.command_str, pending_command);
        
        // Forward to sensor_data module
        extern struct k_msgq sensor_data_msgq;
        k_msgq_put(&sensor_data_msgq, &fwd_msg, K_NO_WAIT);
    } else if (strcmp(pending_command, "STOP_SENSOR_PROCESSING") == 0) {
        // Analytics doesn't directly handle sensor processing, but should forward
        LOG_INF("Forwarding STOP_SENSOR_PROCESSING command");
        // Forward to sensor_data module via appropriate queue
        generic_message_t fwd_msg = {};
        fwd_msg.sender = SENDER_ANALYTICS;
        fwd_msg.type = MSG_TYPE_COMMAND;
        strcpy(fwd_msg.data.command_str, pending_command);
        
        // Forward to sensor_data module
        extern struct k_msgq sensor_data_msgq;
        k_msgq_put(&sensor_data_msgq, &fwd_msg, K_NO_WAIT);
    } else {
        LOG_WRN("Unknown command: %s", pending_command);
    }
}

// Work handler for periodic analytics processing
static void analytics_periodic_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    // This can be used for any periodic analytics tasks if needed
    // For now, just reschedule
    if (atomic_get(&processing_active) == 1) {
        k_work_schedule_for_queue(&analytics_work_q, &analytics_periodic_work, K_MSEC(200));
    }
}

// Perform complex analytics calculations
static void perform_complex_analytics(void)
{
    // TODO: Implement actual complex calculations
    // - Running efficiency (multi-factor)
    // - Fatigue index (baseline comparison)
    // - Injury risk assessment (composite)
    // - CPEI path analysis
    // - Stride length estimation with corrections
    
    // Placeholder implementations - replace with actual algorithms
    
    // Running efficiency calculation placeholder
    // TODO: Should consider: cadence, vertical oscillation, contact time, duty factor
    analytics_state.running_efficiency = 75.0f + (rand() % 10);
    
    // Fatigue index calculation placeholder
    // TODO: Should track: contact time increase, form degradation, asymmetry increase
    analytics_state.fatigue_index = calculate_fatigue_index_placeholder();
    
    // Injury risk assessment placeholder
    // TODO: Should analyze: loading rate, pronation, asymmetry, fatigue level
    analytics_state.injury_risk = calculate_injury_risk_placeholder();
    
    // Stride length - Use Choros if available, otherwise placeholder
    #if CHOROS_ENABLED && CHOROS_USE_FOR_GCT
    analytics_state.stride_length = choros_get_stride_length_safe(calculate_stride_length_placeholder());
    #else
    analytics_state.stride_length = calculate_stride_length_placeholder();
    #endif
    
    // Pronation analysis - Use Choros if available, otherwise placeholder
    #if CHOROS_ENABLED && CHOROS_USE_FOR_GCT
    analytics_state.pronation_angle = choros_get_pronation_safe(calculate_pronation_analysis_placeholder());
    #else
    analytics_state.pronation_angle = calculate_pronation_analysis_placeholder();
    #endif
    
    // Log periodically
    if (analytics_state.analytics_count % 25 == 0) {
        LOG_INF("Analytics: efficiency=%.2f%%, fatigue=%.2f, risk=%.2f",
                (double)analytics_state.running_efficiency,
                (double)analytics_state.fatigue_index,
                (double)analytics_state.injury_risk);
    }
    
    // Send to session management with actual analytics data
    generic_message_t out_msg = {};
    out_msg.sender = SENDER_ANALYTICS;
    out_msg.type = MSG_TYPE_ANALYTICS_RESULTS;
    
    // Populate analytics results
    out_msg.data.analytics_results.running_efficiency = analytics_state.running_efficiency;
    out_msg.data.analytics_results.fatigue_index = analytics_state.fatigue_index;
    out_msg.data.analytics_results.injury_risk = analytics_state.injury_risk;
    out_msg.data.analytics_results.stride_length = analytics_state.stride_length;
    out_msg.data.analytics_results.pronation_angle = analytics_state.pronation_angle;
    out_msg.data.analytics_results.vertical_stiffness = calculate_vertical_stiffness_placeholder();
    out_msg.data.analytics_results.timestamp_ms = k_uptime_get_32();
    
    k_msgq_put(&analytics_queue, &out_msg, K_NO_WAIT);
}
// Establish baseline metrics
static void establish_baseline(void)
{
    // TODO: Implement more sophisticated baseline calculation
    // For now, calculate simple averages from history
    
    if (history_count < 10) {
        // Not enough data yet
        return;
    }
    
    // Calculate baseline averages from history
    float total_contact_time = 0;
    float total_cadence = 0;
    float total_pronation = 0;
    float total_efficiency = 0;
    
    uint16_t samples = MIN(history_count, 60); // Use up to 60 seconds of data
    uint16_t start_idx = (history_write_index + METRIC_HISTORY_SIZE - samples) % METRIC_HISTORY_SIZE;
    
    for (uint16_t i = 0; i < samples; i++) {
        uint16_t idx = (start_idx + i) % METRIC_HISTORY_SIZE;
        total_contact_time += metric_history[idx].ground_contact_ms;
        total_cadence += metric_history[idx].cadence_spm;
        total_pronation += abs(metric_history[idx].avg_pronation_deg);
        total_efficiency += metric_history[idx].efficiency_score;
    }
    
    analytics_state.baseline_contact_time = total_contact_time / samples;
    analytics_state.baseline_cadence = total_cadence / samples;
    analytics_state.baseline_pronation = total_pronation / samples;
    analytics_state.baseline_efficiency = total_efficiency / samples;
    
//    LOG_WRN("Baseline: contact=%.1fms, cadence=%.1f, pronation=%.1f°, efficiency=%.1f%%",
  //          (double)analytics_state.baseline_contact_time,
    //        (double)analytics_state.baseline_cadence,
      //      (double)analytics_state.baseline_pronation,
        //    (double)analytics_state.baseline_efficiency);
}

// Module event handler
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh)) {
        const struct module_state_event *event = cast_module_state_event(aeh);
        
        // Wait for realtime_metrics module to be ready
        if (check_state(event, MODULE_ID(realtime_metrics), MODULE_STATE_READY)) {
            if (!module_initialized) {
                analytics_init();
            }
        }
        return false;
    }
    
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);

// Placeholder function implementations
// TODO: Replace these with actual biomechanical algorithms

static float calculate_fatigue_index_placeholder(void)
{
    // TODO: Implement actual fatigue detection algorithm
    // Should track:
    // - Contact time increase over baseline
    // - Form score degradation
    // - Asymmetry increase
    // - Cadence decrease
    // For now, simple linear increase based on time
    return fminf(analytics_state.analytics_count * 0.1f, 100.0f);
}

static float calculate_injury_risk_placeholder(void)
{
    // TODO: Implement actual injury risk assessment
    // Should analyze:
    // - Loading rate trends
    // - Pronation angles (excessive)
    // - Asymmetry levels
    // - Fatigue level
    // - Previous injury history
    // For now, random value between 20-50%
    return 20.0f + (rand() % 30);
}

static float calculate_stride_length_placeholder(void)
{
    // TODO: Implement actual stride length calculation
    // Should use:
    // - User height
    // - Current cadence
    // - Current speed (from GPS or estimated)
    // - Terrain adjustment factor
    // - Running style (overstriding detection)
    // For now, return typical value 1.2-1.4m
    return 1.2f + (rand() % 20) / 100.0f;
}

static float calculate_pronation_analysis_placeholder(void)
{
    // TODO: Implement pronation trend analysis
    // Should track:
    // - Average pronation angle
    // - Left/right differences
    // - Changes with fatigue
    // - Correlation with strike pattern
    // For now, return normal range value
    return -5.0f + (rand() % 10);
}

static float calculate_vertical_stiffness_placeholder(void)
{
    // TODO: Implement vertical stiffness calculation
    // Should calculate:
    // - Spring-mass model stiffness
    // - Based on vertical oscillation and ground contact
    // - Normalized by body weight
    // For now, return typical value
    return 200.0f + (rand() % 50);
}

static float calculate_recovery_score_placeholder(void)
{
    // TODO: Implement recovery quality assessment
    // Should evaluate:
    // - Heart rate recovery (if available)
    // - Gait quality post-activity
    // - Asymmetry resolution
    // - Return to baseline metrics
    // For now, return good recovery score
    return 75.0f + (rand() % 25);
}
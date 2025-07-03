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

// Analytics state
static struct {
    // Baseline tracking
    bool baseline_established;
    uint32_t baseline_start_time;
    float baseline_contact_time;
    float baseline_efficiency;
    
    // Complex metrics
    float running_efficiency;
    float fatigue_index;
    float injury_risk;
    float stride_length;
    float pronation_angle;
    
    // Processing counters
    uint32_t analytics_count;
    uint32_t last_analytics_time;
} analytics_state;

// Forward declarations
static void analytics_init(void);
static void analytics_thread_fn(void *arg1, void *arg2, void *arg3);
static void perform_complex_analytics(void);
static void establish_baseline(void);
static void process_realtime_metrics_work_handler(struct k_work *work);
static void process_command_work_handler(struct k_work *work);
static void analytics_periodic_work_handler(struct k_work *work);

// Initialize the analytics module
static void analytics_init(void)
{
    LOG_INF("Initializing analytics module");
    
    // Initialize state
    memset(&analytics_state, 0, sizeof(analytics_state));
    
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
        // Wait for realtime metrics
        int ret = k_msgq_get(&realtime_queue, &msg, K_FOREVER);
        
        if (ret == 0) {
            // Queue different work based on message type
            switch (msg.type) {
                case MSG_TYPE_REALTIME_METRICS:
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
                    LOG_DBG("Received unsupported message type %d", msg.type);
                    break;
            }
        }
    }
}

// Work handler for processing realtime metrics
static void process_realtime_metrics_work_handler(struct k_work *work)
{
    const uint32_t ANALYTICS_INTERVAL_MS = 200; // 5Hz max
    const uint32_t BASELINE_DURATION_MS = 120000; // 2 minutes
    
    if (atomic_get(&processing_active) == 1 && new_metrics_available) {
        uint32_t now = k_uptime_get_32();
        
        LOG_DBG("Processing realtime metrics");
        
        // Establish baseline during first 2 minutes
        if (!analytics_state.baseline_established) {
            if (now - analytics_state.baseline_start_time < BASELINE_DURATION_MS) {
                establish_baseline();
            } else {
                analytics_state.baseline_established = true;
                LOG_INF("Baseline established after 2 minutes");
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
    LOG_DBG("Processing command: %s", pending_command);
    
    if (strcmp(pending_command, "START_ANALYTICS") == 0) {
        atomic_set(&processing_active, 1);
        analytics_state.baseline_start_time = k_uptime_get_32();
        LOG_INF("Analytics processing started");
        // Start periodic work
        k_work_schedule_for_queue(&analytics_work_q, &analytics_periodic_work, K_MSEC(200));
    } else if (strcmp(pending_command, "STOP_ANALYTICS") == 0) {
        atomic_set(&processing_active, 0);
        LOG_INF("Analytics processing stopped");
        // Cancel periodic work
        k_work_cancel_delayable(&analytics_periodic_work);
    } else {
        LOG_WRN("Unknown command: %s", pending_command);
    }
}

// Work handler for periodic analytics processing
static void analytics_periodic_work_handler(struct k_work *work)
{
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
    
    // For now, just simulate some calculations
    analytics_state.running_efficiency = 75.0f + (rand() % 10);
    analytics_state.fatigue_index = fminf(analytics_state.analytics_count * 0.1f, 100.0f);
    analytics_state.injury_risk = 20.0f + (rand() % 30);
    analytics_state.stride_length = 1.2f + (rand() % 20) / 100.0f;
    
    // Log periodically
    if (analytics_state.analytics_count % 25 == 0) {
        LOG_INF("Analytics: efficiency=%.1f%%, fatigue=%.1f%%, risk=%.1f%%",
                analytics_state.running_efficiency,
                analytics_state.fatigue_index,
                analytics_state.injury_risk);
    }
    
    // Send to session management
    generic_message_t out_msg = {
        .sender = SENDER_ANALYTICS,
        .type = MSG_TYPE_ANALYTICS_RESULTS
    };
    
    k_msgq_put(&analytics_queue, &out_msg, K_NO_WAIT);
}

// Establish baseline metrics
static void establish_baseline(void)
{
    // TODO: Accumulate baseline metrics during first 2 minutes
    // - Average contact time
    // - Average efficiency
    // - Normal pronation range
    // - Typical stride length
    
    LOG_DBG("Establishing baseline...");
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
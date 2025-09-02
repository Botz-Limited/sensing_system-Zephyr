/**
 * @file realtime_metrics.cpp
 * @brief Real-time metrics calculation module for BLE updates (10-50Hz)
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#define MODULE realtime_metrics

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <string.h>
#include <math.h>

#include <app.hpp>
#include <events/app_state_event.h>
#include <events/motion_sensor_event.h>
#include <errors.hpp>
#include <ble_connection_state.h>
#include "realtime_metrics.h"

// Include sensor data structures
#include "../sensor_data/sensor_data_consolidated.hpp"

LOG_MODULE_REGISTER(MODULE, CONFIG_REALTIME_METRICS_MODULE_LOG_LEVEL);

// Include Choros buffer for enhanced metrics (safe inclusion)
#include <choros_buffer.hpp>

// Thread configuration
static constexpr int realtime_metrics_stack_size = CONFIG_REALTIME_METRICS_MODULE_STACK_SIZE;
static constexpr int realtime_metrics_priority = CONFIG_REALTIME_METRICS_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(realtime_metrics_stack_area, realtime_metrics_stack_size);
static struct k_thread realtime_metrics_thread_data;
static k_tid_t realtime_metrics_tid;

// Work queue configuration
static constexpr int realtime_metrics_workq_stack_size = 2048;
K_THREAD_STACK_DEFINE(realtime_metrics_workq_stack, realtime_metrics_workq_stack_size);
static struct k_work_q realtime_metrics_work_q;

// Work items for different message types
static struct k_work process_consolidated_data_work;
static struct k_work process_command_work;
static struct k_work process_gps_update_work;
static struct k_work_delayable ble_update_work;

// Message buffers for different work items
static char pending_command[MAX_COMMAND_STRING_LEN];
static sensor_data_consolidated_t latest_sensor_data;
static GPSUpdateCommand pending_gps_update;
static bool new_sensor_data_available;
K_MUTEX_DEFINE(sensor_data_mutex);

// Message queues are defined in app.cpp
extern struct k_msgq sensor_data_queue;  // Input from sensor_data
extern struct k_msgq realtime_queue;     // Output to analytics
extern struct k_msgq bluetooth_msgq;     // Output to bluetooth
extern struct k_msgq data_msgq;          // Output to data

// Module state
static bool module_initialized = false;
static atomic_t processing_active = ATOMIC_INIT(0);

// Metrics state
static struct {
    // Tracking structures
    cadence_tracker_t cadence_tracker;
    pace_estimator_t pace_estimator;
    form_score_t form_score;
    
    // Current metrics
    realtime_metrics_t current_metrics;
    
    // Contact time tracking (moving averages) with overflow protection
    uint32_t left_contact_sum_ms;
    uint32_t right_contact_sum_ms;
    uint32_t left_flight_sum_ms;
    uint32_t right_flight_sum_ms;
    uint16_t contact_count;
    uint16_t avg_left_contact_ms;  // Cached average for overflow handling
    uint16_t avg_right_contact_ms; // Cached average for overflow handling
    
    // Force tracking with overflow protection
    uint32_t left_force_sum;
    uint32_t right_force_sum;
    uint16_t force_count;
    uint16_t avg_left_force;  // Cached average for overflow handling
    uint16_t avg_right_force; // Cached average for overflow handling
    
    // Variability tracking for consistency score
    float contact_time_variance;
    float last_contact_times[10];
    uint8_t variance_index;
    
    // Stride length buffer for averaging (10 samples)
    #define STRIDE_LENGTH_BUFFER_SIZE 10
    float stride_lengths[STRIDE_LENGTH_BUFFER_SIZE];
    uint8_t stride_length_index;
    uint8_t stride_length_count;  // Track how many valid samples we have
    
    // BLE update timing
    uint32_t last_ble_update;
    uint32_t metrics_count;
    uint32_t current_time_ms;

    // GPS data
    float latest_gps_speed_cms;
    uint32_t last_gps_timestamp;
    
    uint32_t last_left_strike_time;
    uint32_t last_right_strike_time;
    uint16_t left_stride_duration;
    uint16_t right_stride_duration;
    float left_stride_length;
    float right_stride_length;
    bool last_left_contact;
    bool last_right_contact;
} metrics_state;

// User configuration (should come from settings)
static struct {
    uint16_t user_height_cm;
    uint16_t user_weight_kg;
} user_config = {
    .user_height_cm = 175,  // Default 175cm
    .user_weight_kg = 70    // Default 70kg
};

// Forward declarations
static void realtime_metrics_init(void);
/**
 * @brief Main processing thread for real-time metrics, waits for messages and queues work.
 * @param arg1 Unused.
 * @param arg2 Unused.
 * @param arg3 Unused.
 * @note This function is complete and handles incoming messages for consolidated sensor data and commands.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet on primary device).
 */
static void realtime_metrics_thread_fn(void *arg1, void *arg2, void *arg3);
/**
 * @brief Calculates real-time metrics for BLE updates and feedback.
 * @note This function is complete and calculates metrics like cadence, pace, form score, and balance.
 *       Data Requirements: Motion sensor data (BHI360 for step count and cadence), Foot sensor data (both feet for contact times and balance).
 */
static void calculate_realtime_metrics(void);
/**
 * @brief Sends real-time metrics updates to Bluetooth for user feedback.
 * @note This function is complete and sends formatted metrics data to the Bluetooth module.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet).
 */
static void send_ble_update(void);
/**
 * @brief Work handler for processing consolidated sensor data.
 * @param work Pointer to the work item.
 * @note This function is complete and processes sensor data to calculate real-time metrics.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet on primary device).
 */
static void process_consolidated_data_work_handler(struct k_work *work);
/**
 * @brief Work handler for processing commands such as start/stop real-time processing.
 * @param work Pointer to the work item.
 * @note This function is complete and handles commands to control real-time metrics processing.
 *       Data Requirements: None directly, but commands trigger processing of sensor data.
 */
static void process_command_work_handler(struct k_work *work);
/**
 * @brief Work handler for periodic BLE updates of real-time metrics.
 * @param work Pointer to the work item.
 * @note This function is complete and triggers BLE updates at 1Hz when processing is active.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet).
 */
static void ble_update_work_handler(struct k_work *work);
static void process_gps_update_work_handler(struct k_work *work);

// Initialize the realtime metrics module
static void realtime_metrics_init(void)
{
    LOG_INF("Initializing realtime metrics module");
    
    // Initialize state
    memset(&metrics_state, 0, sizeof(metrics_state));
    
    // Initialize tracking structures
    cadence_tracker_init(&metrics_state.cadence_tracker);
    pace_estimator_init(&metrics_state.pace_estimator, (float)user_config.user_height_cm);
    
    // Initialize work queue
    k_work_queue_init(&realtime_metrics_work_q);
    k_work_queue_start(&realtime_metrics_work_q, realtime_metrics_workq_stack,
                       K_THREAD_STACK_SIZEOF(realtime_metrics_workq_stack),
                       realtime_metrics_priority - 1, NULL);
    k_thread_name_set(&realtime_metrics_work_q.thread, "realtime_metrics_wq");
    
    // Initialize work items
    k_work_init(&process_consolidated_data_work, process_consolidated_data_work_handler);
    k_work_init(&process_command_work, process_command_work_handler);
    k_work_init(&process_gps_update_work, process_gps_update_work_handler);
    k_work_init_delayable(&ble_update_work, ble_update_work_handler);
    
    // Create the message processing thread
    realtime_metrics_tid = k_thread_create(
        &realtime_metrics_thread_data,
        realtime_metrics_stack_area,
        K_THREAD_STACK_SIZEOF(realtime_metrics_stack_area),
        realtime_metrics_thread_fn,
        NULL, NULL, NULL,
        realtime_metrics_priority,
        0,
        K_NO_WAIT
    );
    
    k_thread_name_set(realtime_metrics_tid, "realtime_metrics");
    
    // Start BLE update timer (1Hz)
    k_work_schedule_for_queue(&realtime_metrics_work_q, &ble_update_work, K_SECONDS(1));
    
    module_initialized = true;
    module_set_state(MODULE_STATE_READY);
    LOG_INF("Realtime metrics module initialized");
}

// Main processing thread - waits for messages and queues appropriate work
static void realtime_metrics_thread_fn(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    
    generic_message_t msg;
    
    while (true) {
        // Wait for sensor data from sensor_data_queue (not realtime_queue which is output!)
        int ret = k_msgq_get(&sensor_data_queue, &msg, K_FOREVER);
        
        if (ret == 0) {
            // Queue different work based on message type
            switch (msg.type) {
                case MSG_TYPE_SENSOR_DATA_CONSOLIDATED:
                    // Copy sensor data with mutex protection
                    k_mutex_lock(&sensor_data_mutex, K_FOREVER);
                    memcpy(&latest_sensor_data, &msg.data.sensor_consolidated, sizeof(sensor_data_consolidated_t));
                    new_sensor_data_available = true;
                    k_mutex_unlock(&sensor_data_mutex);
                    
                    metrics_state.current_time_ms = k_uptime_get_32();
                    k_work_submit_to_queue(&realtime_metrics_work_q, &process_consolidated_data_work);
                    break;
                    
                case MSG_TYPE_COMMAND:
                    // Copy command and queue command processing work
                    strncpy(pending_command, msg.data.command_str, MAX_COMMAND_STRING_LEN - 1);
                    pending_command[MAX_COMMAND_STRING_LEN - 1] = '\0';
                    k_work_submit_to_queue(&realtime_metrics_work_q, &process_command_work);
                    break;

                case MSG_TYPE_GPS_UPDATE:
                    // Copy GPS data and queue processing work
                    k_mutex_lock(&sensor_data_mutex, K_FOREVER);
                    memcpy(&pending_gps_update, &msg.data.gps_update, sizeof(GPSUpdateCommand));
                    k_mutex_unlock(&sensor_data_mutex);
                    k_work_submit_to_queue(&realtime_metrics_work_q, &process_gps_update_work);
                    break;

                case MSG_TYPE_REALTIME_METRICS:
                    // Directly use the calculated bilateral metrics from gait events
                    LOG_INF("Received bilateral metrics from gait events system");
                    // Copy the metrics and update timestamp
                    memcpy(&metrics_state.current_metrics, &msg.data.realtime_metrics, sizeof(realtime_metrics_t));
                    metrics_state.current_metrics.timestamp_ms = k_uptime_get_32();
                    metrics_state.metrics_count++;
                    
                    // Log the received values for debugging
                    LOG_INF("DIAG: Received metrics - Cadence=%u spm, Pace=%u s/km, GCT=%u ms",
                            metrics_state.current_metrics.cadence_spm,
                            metrics_state.current_metrics.pace_sec_km,
                            metrics_state.current_metrics.ground_contact_ms);
                    LOG_INF("DIAG: Form=%u%%, Balance=%d%%, Efficiency=%u%%",
                            metrics_state.current_metrics.form_score,
                            metrics_state.current_metrics.balance_lr_pct,
                            metrics_state.current_metrics.efficiency_score);
                    
                    // Send the metrics immediately since they're already calculated
                    send_ble_update();
                    break;
                    
                default:
                    LOG_WRN("Received unsupported message type %d", msg.type);
                    break;
            }
        }
    }
}

// Work handler for processing consolidated sensor data
static void process_consolidated_data_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (atomic_get(&processing_active) == 1 && new_sensor_data_available) {
       // LOG_WRN("Processing consolidated sensor data");
        
        // Calculate real-time metrics
        calculate_realtime_metrics();
        metrics_state.metrics_count++;
        
        // Clear the flag
        new_sensor_data_available = false;
    }
}

// Work handler for processing commands
static void process_command_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    LOG_WRN("Processing command: %s", pending_command);
    
    if (strcmp(pending_command, "START_REALTIME_PROCESSING") == 0) {
        // START: Begin new session with state reset
        // Clear any pending messages in input queue before starting
        generic_message_t dummy_msg;
        while (k_msgq_get(&sensor_data_queue, &dummy_msg, K_NO_WAIT) == 0) {
            // Discard message
        }
        
        // Reset metrics state for clean start
        memset(&metrics_state, 0, sizeof(metrics_state));
        new_sensor_data_available = false;
        
        atomic_set(&processing_active, 1);
        LOG_INF("Realtime processing started - buffers cleared");
    } else if (strcmp(pending_command, "STOP_REALTIME_PROCESSING") == 0) {
        // STOP: End session completely with state reset
        atomic_set(&processing_active, 0);
        
        // Clear all pending messages in output queues
        generic_message_t dummy_msg;
        while (k_msgq_get(&realtime_queue, &dummy_msg, K_NO_WAIT) == 0) {
            // Discard message
        }
        while (k_msgq_get(&bluetooth_msgq, &dummy_msg, K_NO_WAIT) == 0) {
            // Discard message
        }
        while (k_msgq_get(&data_msgq, &dummy_msg, K_NO_WAIT) == 0) {
            // Discard message
        }
        
        // Reset metrics state
        memset(&metrics_state, 0, sizeof(metrics_state));
        new_sensor_data_available = false;
    } else if (strcmp(pending_command, "PAUSE_REALTIME_PROCESSING") == 0) {
        // PAUSE: Suspend processing but preserve state
        atomic_set(&processing_active, 0);
        LOG_INF("Realtime metrics processing PAUSED - state preserved");
        
        // Clear pending messages to prevent buffer overflow during pause
        generic_message_t dummy_msg;
        while (k_msgq_get(&sensor_data_queue, &dummy_msg, K_NO_WAIT) == 0) {
            // Discard message
        }
        while (k_msgq_get(&realtime_queue, &dummy_msg, K_NO_WAIT) == 0) {
            // Discard message
        }
        LOG_INF("Message queues cleared for pause");
        // NOTE: We do NOT reset metrics_state
        // This preserves metrics continuity when unpaused
    } else if (strcmp(pending_command, "UNPAUSE_REALTIME_PROCESSING") == 0) {
        // UNPAUSE: Resume processing with preserved state
        atomic_set(&processing_active, 1);
        LOG_INF("Realtime metrics processing UNPAUSED - continuing with preserved state");
        LOG_INF("  Cadence: %.1f spm preserved", (double)metrics_state.cadence_tracker.current_cadence_spm);
        // Pace estimator state preserved but not logged due to struct member availability
        LOG_INF("  Metrics state preserved, processing will resume");
        // No state reset - continue from where we paused
        
        LOG_INF("Realtime processing stopped - all buffers cleared");
    } else {
        LOG_WRN("Unknown command: %s", pending_command);
    }
}

// Work handler for periodic BLE updates
static void ble_update_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (atomic_get(&processing_active) == 1) {
        send_ble_update();
        metrics_state.last_ble_update = k_uptime_get_32();
    }
    
    // Reschedule for next update (1Hz)
    k_work_schedule_for_queue(&realtime_metrics_work_q, &ble_update_work, K_SECONDS(1));
}

// Work handler for processing GPS updates
static void process_gps_update_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    
    k_mutex_lock(&sensor_data_mutex, K_FOREVER);
    metrics_state.latest_gps_speed_cms = pending_gps_update.speed_cms;
    metrics_state.last_gps_timestamp = metrics_state.current_time_ms;
    k_mutex_unlock(&sensor_data_mutex);
}

// Calculate real-time metrics
// NOTE: Event-based parameter calculation happens in sensor_data_events module:
// - sensor_data_events calculates: cadence, GCT, stride length, pronation from gait events
// - Those parameters are sent via D2D every 1-2 seconds (sensor_data_1hz_timer_callback)
// - This function provides fallback calculation when event-based data unavailable
// - Primary device: Calculates bilateral metrics from both feet
// - Secondary device: Calculates left foot metrics only for D2D transmission
static void calculate_realtime_metrics(void)
{
    // Copy sensor data with mutex protection
    sensor_data_consolidated_t local_data;
    k_mutex_lock(&sensor_data_mutex, K_FOREVER);
    memcpy(&local_data, &latest_sensor_data, sizeof(sensor_data_consolidated_t));
    k_mutex_unlock(&sensor_data_mutex);
    
    sensor_data_consolidated_t *data = &local_data;
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // PRIMARY DEVICE - Calculate bilateral metrics from both feet
    
    // Check if we have BHI360 data (step_count > 0 or changing)
    static uint32_t last_step_count = 0;
    bool has_bhi360 = (data->step_count > 0) || (data->step_count != last_step_count);
    last_step_count = data->step_count;
    
    float current_cadence = 0;
    
    if (has_bhi360) {
        // Use BHI360-based cadence calculation (original method)
        cadence_tracker_update(&metrics_state.cadence_tracker,
                              data->step_count,
                              metrics_state.current_time_ms);
        
        current_cadence = cadence_tracker_get_current(&metrics_state.cadence_tracker);
    } else {
        // Fallback: Calculate cadence from foot contact events
        static uint16_t last_left_contact_duration = 0;
        static uint16_t last_right_contact_duration = 0;
        static uint32_t last_step_time_ms = 0;
        static uint32_t step_intervals[10] = {0};  // Buffer for averaging
        static uint8_t step_interval_idx = 0;
        static uint32_t step_count_fallback = 0;
        
        // Detect step from increasing contact duration (contact detected)
        // The duration accumulates during contact, so we detect when it starts increasing
        bool left_step = (data->left_contact_duration_ms > 100 &&
                         data->left_contact_duration_ms > last_left_contact_duration);
        bool right_step = (data->right_contact_duration_ms > 100 &&
                          data->right_contact_duration_ms > last_right_contact_duration);
        
        if (left_step || right_step) {
            step_count_fallback++;
            
            // Calculate interval since last step
            if (last_step_time_ms > 0) {
                uint32_t interval = metrics_state.current_time_ms - last_step_time_ms;
                
                // Store interval for averaging (filter outliers)
                if (interval > 200 && interval < 2000) {  // 30-300 spm range
                    step_intervals[step_interval_idx] = interval;
                    step_interval_idx = (step_interval_idx + 1) % 10;
                    
                    // Calculate average interval
                    uint32_t sum = 0;
                    uint8_t count = 0;
                    for (int i = 0; i < 10; i++) {
                        if (step_intervals[i] > 0) {
                            sum += step_intervals[i];
                            count++;
                        }
                    }
                    
                    if (count > 0) {
                        uint32_t avg_interval = sum / count;
                        // Convert to steps per minute
                        current_cadence = 60000.0f / avg_interval;
                        LOG_DBG("Foot-based cadence: %.1f spm (interval: %u ms)",
                                (double)current_cadence, avg_interval);
                    }
                }
            }
            last_step_time_ms = metrics_state.current_time_ms;
        }
        
        // Update for next iteration
        last_left_contact_duration = data->left_contact_duration_ms;
        last_right_contact_duration = data->right_contact_duration_ms;
        
        // Also update the cadence tracker with our fallback count for consistency
        cadence_tracker_update(&metrics_state.cadence_tracker,
                              step_count_fallback,
                              metrics_state.current_time_ms);
    }
    
    // Update pace estimator with GPS if available
    if (metrics_state.last_gps_timestamp > 0 && (metrics_state.current_time_ms - metrics_state.last_gps_timestamp) < 5000) {
        pace_estimator_update_with_gps(&metrics_state.pace_estimator, metrics_state.latest_gps_speed_cms, data->delta_time_ms);
    } else {
        pace_estimator_update(&metrics_state.pace_estimator, current_cadence, data->delta_time_ms);
    }
    
    // Track contact times for averaging with overflow protection
    if (data->left_contact_duration_ms > 0) {
        metrics_state.left_contact_sum_ms += data->left_contact_duration_ms;
        metrics_state.contact_count++;
        
        // Store for variability calculation
        metrics_state.last_contact_times[metrics_state.variance_index] =
            (float)data->left_contact_duration_ms;
        metrics_state.variance_index = (metrics_state.variance_index + 1) % 10;
        
        // Check for overflow and reset with running average
        if (metrics_state.contact_count >= 1000) {
            // Calculate current averages before reset
            metrics_state.avg_left_contact_ms = metrics_state.left_contact_sum_ms / metrics_state.contact_count;
            metrics_state.avg_right_contact_ms = metrics_state.right_contact_sum_ms / metrics_state.contact_count;
            
            // Reset with seed values to maintain continuity
            metrics_state.left_contact_sum_ms = metrics_state.avg_left_contact_ms * 100;
            metrics_state.right_contact_sum_ms = metrics_state.avg_right_contact_ms * 100;
            metrics_state.contact_count = 100;
            
            LOG_WRN("Contact time accumulators reset after 1000 samples");
        }
    }
    
    if (data->right_contact_duration_ms > 0) {
        metrics_state.right_contact_sum_ms += data->right_contact_duration_ms;
    }
    
    // Track forces for balance calculation with overflow protection
    if (data->left_in_contact || data->right_in_contact) {
        metrics_state.left_force_sum += data->left_peak_force;
        metrics_state.right_force_sum += data->right_peak_force;
        metrics_state.force_count++;
        
        // Check for overflow and reset with running average
        if (metrics_state.force_count >= 1000) {
            // Calculate current averages before reset
            metrics_state.avg_left_force = metrics_state.left_force_sum / metrics_state.force_count;
            metrics_state.avg_right_force = metrics_state.right_force_sum / metrics_state.force_count;
            
            // Reset with seed values to maintain continuity
            metrics_state.left_force_sum = metrics_state.avg_left_force * 100;
            metrics_state.right_force_sum = metrics_state.avg_right_force * 100;
            metrics_state.force_count = 100;
            
            LOG_WRN("Force accumulators reset after 1000 samples");
        }
    }
    
    // Calculate averages using cached values if available
    uint16_t avg_left_contact = metrics_state.avg_left_contact_ms;
    uint16_t avg_right_contact = metrics_state.avg_right_contact_ms;
    if (metrics_state.contact_count > 0) {
        avg_left_contact = metrics_state.left_contact_sum_ms / metrics_state.contact_count;
        avg_right_contact = metrics_state.right_contact_sum_ms / metrics_state.contact_count;
    }
    
    // Calculate balance using cached values if available
    int8_t balance = 0;
    uint16_t avg_left_force = metrics_state.avg_left_force;
    uint16_t avg_right_force = metrics_state.avg_right_force;
    if (metrics_state.force_count > 0) {
        avg_left_force = metrics_state.left_force_sum / metrics_state.force_count;
        avg_right_force = metrics_state.right_force_sum / metrics_state.force_count;
    }
    balance = calculate_balance_percentage(avg_left_force, avg_right_force);

    // Calculate stride duration for left foot
    if (data->left_in_contact && !metrics_state.last_left_contact) {
        uint32_t now = k_uptime_get_32();
        if (metrics_state.last_left_strike_time > 0) {
            metrics_state.left_stride_duration = now - metrics_state.last_left_strike_time;
        }
        metrics_state.last_left_strike_time = now;
    }
    metrics_state.last_left_contact = data->left_in_contact;

    // Similar for right foot
    if (data->right_in_contact && !metrics_state.last_right_contact) {
        uint32_t now = k_uptime_get_32();
        if (metrics_state.last_right_strike_time > 0) {
            metrics_state.right_stride_duration = now - metrics_state.last_right_strike_time;
        }
        metrics_state.last_right_strike_time = now;
    }
    metrics_state.last_right_contact = data->right_in_contact;

    // Average stride duration
    metrics_state.current_metrics.stride_duration_ms = (metrics_state.left_stride_duration + metrics_state.right_stride_duration) / 2;

    // Stride duration asymmetry
    metrics_state.current_metrics.stride_duration_asymmetry = calculate_asymmetry_percentage(
        metrics_state.left_stride_duration,
        metrics_state.right_stride_duration
    );

    // Estimate stride length - check if we have valid pace first
    float speed_ms = 0;
    if (metrics_state.current_metrics.pace_sec_km > 0) {
        speed_ms = 1000.0f / metrics_state.current_metrics.pace_sec_km; // m/s
    }
    
    // If no BHI360 or no valid speed, use empirical formula based on GCT
    if (!has_bhi360 || speed_ms == 0) {
        // Estimate stride length from ground contact time (empirical formula)
        if (data->left_contact_duration_ms > 0 || data->right_contact_duration_ms > 0) {
            uint16_t avg_gct = (data->left_contact_duration_ms + data->right_contact_duration_ms) / 2;
            if (avg_gct > 0) {
                // Empirical formula: stride_length ≈ 1.8 - (GCT_ms / 500)
                // Adjusted for typical running (GCT 200-400ms -> stride 1.0-1.4m)
                float estimated_stride = 1.8f - (avg_gct / 500.0f);
                if (estimated_stride < 0.5f) estimated_stride = 0.5f;
                if (estimated_stride > 2.0f) estimated_stride = 2.0f;
                
                metrics_state.left_stride_length = estimated_stride;
                metrics_state.right_stride_length = estimated_stride;
                
                LOG_DBG("Estimated stride from GCT: %.2f m (GCT: %u ms)",
                        estimated_stride, avg_gct);
            }
        }
    } else {
        // Original calculation when we have valid speed
        metrics_state.left_stride_length = speed_ms * (metrics_state.left_stride_duration / 1000.0f);
        metrics_state.right_stride_length = speed_ms * (metrics_state.right_stride_duration / 1000.0f);
    }
    
    // Add to stride length buffer for averaging
    float current_stride_length = (metrics_state.left_stride_length + metrics_state.right_stride_length) / 2;
    if (current_stride_length > 0) {
        metrics_state.stride_lengths[metrics_state.stride_length_index] = current_stride_length;
        metrics_state.stride_length_index = (metrics_state.stride_length_index + 1) % STRIDE_LENGTH_BUFFER_SIZE;
        
        // Track how many valid samples we have
        if (metrics_state.stride_length_count < STRIDE_LENGTH_BUFFER_SIZE) {
            metrics_state.stride_length_count++;
        }
        
        // Calculate average from buffer
        float stride_sum = 0;
        for (uint8_t i = 0; i < metrics_state.stride_length_count; i++) {
            stride_sum += metrics_state.stride_lengths[i];
        }
        float avg_stride_length = stride_sum / metrics_state.stride_length_count;
        
        // Update metric with averaged value
        metrics_state.current_metrics.stride_length_cm = (uint16_t)(avg_stride_length * 100);
        
      //  LOG_WRN("Stride length: current=%.2fm, avg=%.2fm (n=%d)",
        //        current_stride_length, avg_stride_length, metrics_state.stride_length_count);
    } else {
        // No valid stride, use last known value
        metrics_state.current_metrics.stride_length_cm = metrics_state.current_metrics.stride_length_cm;
    }

    // Stride length asymmetry
    metrics_state.current_metrics.stride_length_asymmetry = calculate_asymmetry_percentage(
        (uint16_t)(metrics_state.left_stride_length * 100),
        (uint16_t)(metrics_state.right_stride_length * 100)
    );

    // Calculate vertical oscillation using proper integration
    // TODO: Further improve with Kalman filtering for better drift correction
    // Note: linear_acc already has gravity removed by BHI360
    static float vertical_velocity = 0;
    static float vertical_position = 0;
    static float max_vertical_pos = 0;
    static float min_vertical_pos = 0;
    static uint32_t last_reset_time = 0;
    
    float dt = data->delta_time_ms / 1000.0f;
    
    // Integrate acceleration to get velocity
    vertical_velocity += data->linear_acc[2] * dt;
    
    // Integrate velocity to get position
    vertical_position += vertical_velocity * dt;
    
    // Track min/max over a stride
    if (vertical_position > max_vertical_pos) max_vertical_pos = vertical_position;
    if (vertical_position < min_vertical_pos) min_vertical_pos = vertical_position;
    
    // Reset tracking every ~1 second or on foot strike
    if (metrics_state.current_time_ms - last_reset_time > 1000 ||
        (data->left_in_contact && !metrics_state.last_left_contact)) {
        float vertical_oscillation_m = max_vertical_pos - min_vertical_pos;
        metrics_state.current_metrics.vertical_oscillation_cm = (uint8_t)MIN(vertical_oscillation_m * 100.0f, 255);
        
        // Reset for next stride
        max_vertical_pos = vertical_position;
        min_vertical_pos = vertical_position;
        last_reset_time = metrics_state.current_time_ms;
        
        // Apply high-pass filter to remove drift
        vertical_position *= 0.95f;
        vertical_velocity *= 0.95f;
    }

    // Calculate vertical ratio using actual stride length
    // TODO: Consider terrain adjustment factor for uphill/downhill running
    float vertical_ratio = 0;
    if (metrics_state.current_metrics.stride_length_cm > 0) {
        vertical_ratio = ((float)metrics_state.current_metrics.vertical_oscillation_cm /
                         (float)metrics_state.current_metrics.stride_length_cm) * 100.0f;
    }
    metrics_state.current_metrics.vertical_ratio = (uint8_t)MIN(vertical_ratio, 100);
    // Calculate variability for consistency score
    float variability = 0;
    if (metrics_state.contact_count >= 5) {
        // Simple variance calculation
        float mean = (float)(metrics_state.left_contact_sum_ms + metrics_state.right_contact_sum_ms) / 
                    (float)(metrics_state.contact_count * 2);
        float sum_sq_diff = 0;
        for (int i = 0; i < 10 && i < metrics_state.contact_count; i++) {
            float diff = metrics_state.last_contact_times[i] - mean;
            sum_sq_diff += diff * diff;
        }
        variability = sqrtf(sum_sq_diff / 10.0f) / mean * 100.0f; // CV%
    }
    
    // Calculate form score
    int8_t avg_pronation = (data->left_pronation_deg + data->right_pronation_deg) / 2;
    form_score_calculate(&metrics_state.form_score,
                        avg_left_contact,
                        balance,
                        variability,
                        avg_pronation);
    
    // Update current metrics structure
    metrics_state.current_metrics.timestamp_ms = metrics_state.current_time_ms;
    metrics_state.current_metrics.cadence_spm = (uint16_t)current_cadence;
    
    // Calculate pace - either from estimator or fallback from cadence
    uint16_t estimated_pace = (uint16_t)pace_estimator_get_pace(&metrics_state.pace_estimator);
    if (estimated_pace == 0 && current_cadence > 0) {
        // Fallback: Calculate pace from cadence and estimated stride length
        // Pace (s/km) = 1000 / (cadence * stride_length / 60)
        // Using average stride length of 1.2m for moderate running
        float stride_length_m = metrics_state.current_metrics.stride_length_cm / 100.0f;
        if (stride_length_m == 0) stride_length_m = 1.2f; // Default 1.2m
        float speed_mps = (current_cadence / 60.0f) * stride_length_m;
        if (speed_mps > 0) {
            estimated_pace = (uint16_t)(1000.0f / speed_mps);
            LOG_DBG("Pace from cadence fallback: %d s/km (cadence: %.1f, stride: %.2fm)",
                    estimated_pace, current_cadence, stride_length_m);
        }
    }
    metrics_state.current_metrics.pace_sec_km = estimated_pace;
    metrics_state.current_metrics.distance_m = metrics_state.pace_estimator.distance_m;  // No cast needed, both are uint32_t now
    
    metrics_state.current_metrics.form_score = (uint8_t)metrics_state.form_score.overall_score;
    metrics_state.current_metrics.balance_lr_pct = balance;
    
    // Calculate ground contact time
    if (avg_left_contact > 0 || avg_right_contact > 0) {
        metrics_state.current_metrics.ground_contact_ms = (avg_left_contact + avg_right_contact) / 2;
    } else if (current_cadence > 0) {
        // Fallback: estimate from cadence if no contact data available
        // Typical GCT is 30-40% of step cycle
        uint32_t step_cycle_ms = 60000 / current_cadence;
        metrics_state.current_metrics.ground_contact_ms = step_cycle_ms * 0.35f;
    }
    
    // Calculate flight time
    if (data->true_flight_time_ms > 0) {
        metrics_state.current_metrics.flight_time_ms = data->true_flight_time_ms;
    } else if (current_cadence > 0 && metrics_state.current_metrics.ground_contact_ms > 0) {
        // Estimate flight time from cadence and GCT
        uint32_t step_cycle_ms = 60000 / current_cadence;
        uint32_t estimated_flight = step_cycle_ms - metrics_state.current_metrics.ground_contact_ms;
        if (estimated_flight > 0 && estimated_flight < 1000) {
            metrics_state.current_metrics.flight_time_ms = estimated_flight;
        }
    }
    
    // Calculate asymmetries
    metrics_state.current_metrics.contact_time_asymmetry = 
        calculate_asymmetry_percentage(avg_left_contact, avg_right_contact);
    metrics_state.current_metrics.force_asymmetry = 
        calculate_asymmetry_percentage(data->left_peak_force, data->right_peak_force);
    metrics_state.current_metrics.pronation_asymmetry = 
        calculate_asymmetry_percentage(abs(data->left_pronation_deg), abs(data->right_pronation_deg));
    
    // Strike patterns
    metrics_state.current_metrics.left_strike_pattern = data->left_contact_phase == 1 ? 0 : 
                                                       (data->left_fore_pct > 50 ? 2 : 1);
    metrics_state.current_metrics.right_strike_pattern = data->right_contact_phase == 1 ? 0 : 
                                                        (data->right_fore_pct > 50 ? 2 : 1);
    
    // Pronation
    metrics_state.current_metrics.avg_pronation_deg = avg_pronation;
    
    // Calculate running efficiency based on multiple factors
    // TODO: Add user-specific calibration for optimal values
    // TODO: Consider pace-dependent adjustments for efficiency calculation
    uint8_t efficiency_score = 100;
    
    // Factor 1: Duty factor (contact time / stride time) - optimal is around 35%
    float duty_factor = 0;
    if (metrics_state.current_metrics.stride_duration_ms > 0) {
        duty_factor = (float)metrics_state.current_metrics.ground_contact_ms /
                     (float)metrics_state.current_metrics.stride_duration_ms;
        float duty_factor_penalty = fabsf(duty_factor - 0.35f) * 100.0f;
        efficiency_score -= MIN(duty_factor_penalty, 20); // Max 20 point penalty
    }
    
    // Factor 2: Vertical ratio - lower is better (typical 8-10%)
    if (metrics_state.current_metrics.vertical_ratio > 10) {
        efficiency_score -= MIN((metrics_state.current_metrics.vertical_ratio - 10) * 2, 20);
    }
    
    // Factor 3: Cadence - optimal range 170-180 spm
    if (current_cadence < 170 || current_cadence > 180) {
        float cadence_penalty = fabsf(current_cadence - 175.0f) / 5.0f;
        efficiency_score -= MIN(cadence_penalty, 15);
    }
    
    // Factor 4: Form score contribution
    efficiency_score = (efficiency_score * 0.7f) + (metrics_state.form_score.overall_score * 0.3f);
    
    metrics_state.current_metrics.efficiency_score = (uint8_t)MIN(efficiency_score, 100);
    
    // Check for alerts
    metrics_state.current_metrics.alerts = 0;
    if (metrics_state.current_metrics.contact_time_asymmetry > 15) {
        metrics_state.current_metrics.alerts |= RT_ALERT_HIGH_ASYMMETRY;
    }
    if (metrics_state.current_metrics.form_score < 60) {
        metrics_state.current_metrics.alerts |= RT_ALERT_POOR_FORM;
    }
    if (abs(avg_pronation) > 15) {
        metrics_state.current_metrics.alerts |= RT_ALERT_OVERPRONATION;
    }
    
    // Log periodically
    if (metrics_state.metrics_count % 50 == 0) {
        LOG_INF("Metrics: Cadence=%d spm, Pace=%d s/km, Form=%d%%, Balance=%d%%",
                metrics_state.current_metrics.cadence_spm,
                metrics_state.current_metrics.pace_sec_km,
                metrics_state.current_metrics.form_score,
                balance);
        
        // Also log if using fallback mode
        if (!has_bhi360) {
            LOG_INF("  Using foot-sensor fallback mode (no BHI360 data)");
        }
    }
    
    // Send to analytics thread with actual metrics data
    generic_message_t out_msg = {};
    out_msg.sender = SENDER_REALTIME_METRICS;
    out_msg.type = MSG_TYPE_REALTIME_METRICS;
    
    // Copy the current metrics to the message
    memcpy(&out_msg.data.realtime_metrics, &metrics_state.current_metrics, 
           sizeof(realtime_metrics_t));
    
    k_msgq_put(&realtime_queue, &out_msg, K_NO_WAIT);
    
    // No longer need periodic reset - overflow protection handles it
    // Removed the periodic reset code that was here
    
#else  // SECONDARY DEVICE
    
    // Secondary device calculates metrics for LEFT foot only
    // These metrics will be sent via D2D to primary for bilateral calculations
    
    // Check if we have BHI360 data (step_count > 0 or changing)
    static uint32_t last_step_count = 0;
    bool has_bhi360 = (data->step_count > 0) || (data->step_count != last_step_count);
    last_step_count = data->step_count;
    
    float current_cadence = 0;
    
    if (has_bhi360) {
        // Use BHI360-based cadence calculation (original method)
        cadence_tracker_update(&metrics_state.cadence_tracker,
                              data->step_count,
                              metrics_state.current_time_ms);
        
        current_cadence = cadence_tracker_get_current(&metrics_state.cadence_tracker);
    } else {
        // Fallback: Calculate cadence from foot contact events (LEFT FOOT ONLY)
        static uint16_t last_left_contact_duration = 0;
        static uint32_t last_step_time_ms = 0;
        static uint32_t step_intervals[10] = {0};  // Buffer for averaging
        static uint8_t step_interval_idx = 0;
        static uint32_t step_count_fallback = 0;
        
        // Detect step from increasing contact duration (LEFT foot only for secondary)
        bool left_step = (data->left_contact_duration_ms > 100 &&
                         data->left_contact_duration_ms > last_left_contact_duration);
        
        if (left_step) {
            step_count_fallback++;
            
            // Calculate interval since last step
            if (last_step_time_ms > 0) {
                uint32_t interval = metrics_state.current_time_ms - last_step_time_ms;
                
                // Store interval for averaging (filter outliers)
                if (interval > 200 && interval < 2000) {  // 30-300 spm range
                    step_intervals[step_interval_idx] = interval;
                    step_interval_idx = (step_interval_idx + 1) % 10;
                    
                    // Calculate average interval
                    uint32_t sum = 0;
                    uint8_t count = 0;
                    for (int i = 0; i < 10; i++) {
                        if (step_intervals[i] > 0) {
                            sum += step_intervals[i];
                            count++;
                        }
                    }
                    
                    if (count > 0) {
                        uint32_t avg_interval = sum / count;
                        // Convert to steps per minute (multiply by 2 for bilateral cadence)
                        current_cadence = (60000.0f / avg_interval) * 2;  // x2 because we only see left steps
                        LOG_DBG("Secondary foot-based cadence: %.1f spm (interval: %u ms)",
                                (double)current_cadence, avg_interval);
                    }
                }
            }
            last_step_time_ms = metrics_state.current_time_ms;
        }
        
        // Update for next iteration
        last_left_contact_duration = data->left_contact_duration_ms;
        
        // Also update the cadence tracker with our fallback count for consistency
        cadence_tracker_update(&metrics_state.cadence_tracker,
                              step_count_fallback * 2,  // x2 for bilateral equivalent
                              metrics_state.current_time_ms);
    }
    
    // Update pace estimator (simplified for secondary)
    pace_estimator_update(&metrics_state.pace_estimator, current_cadence, data->delta_time_ms);
    
    // Track LEFT foot contact times only
    if (data->left_contact_duration_ms > 0) {
        metrics_state.left_contact_sum_ms += data->left_contact_duration_ms;
        metrics_state.contact_count++;
        
        // Check for overflow and reset with running average
        if (metrics_state.contact_count >= 1000) {
            metrics_state.avg_left_contact_ms = metrics_state.left_contact_sum_ms / metrics_state.contact_count;
            metrics_state.left_contact_sum_ms = metrics_state.avg_left_contact_ms * 100;
            metrics_state.contact_count = 100;
            LOG_WRN("Secondary: Contact time accumulator reset");
        }
    }
    
    // Calculate average contact time for left foot
    uint16_t avg_left_contact = 0;
    if (metrics_state.contact_count > 0) {
        avg_left_contact = metrics_state.left_contact_sum_ms / metrics_state.contact_count;
    }
    
    // Estimate stride length for left foot
    if (!has_bhi360 || metrics_state.current_metrics.pace_sec_km == 0) {
        // Estimate stride length from ground contact time
        if (data->left_contact_duration_ms > 0) {
            float estimated_stride = 1.8f - (data->left_contact_duration_ms / 500.0f);
            if (estimated_stride < 0.5f) estimated_stride = 0.5f;
            if (estimated_stride > 2.0f) estimated_stride = 2.0f;
            
            metrics_state.left_stride_length = estimated_stride;
            LOG_DBG("Secondary estimated stride from GCT: %.2f m", estimated_stride);
        }
    }
    
    // Update current metrics structure (LEFT foot data only)
    metrics_state.current_metrics.timestamp_ms = metrics_state.current_time_ms;
    metrics_state.current_metrics.cadence_spm = (uint16_t)current_cadence;
    metrics_state.current_metrics.pace_sec_km = (uint16_t)pace_estimator_get_pace(&metrics_state.pace_estimator);
    
    // Ground contact time (left foot only)
    if (avg_left_contact > 0) {
        metrics_state.current_metrics.ground_contact_ms = avg_left_contact;
    } else if (current_cadence > 0) {
        // Fallback: estimate from cadence
        uint32_t step_cycle_ms = 60000 / current_cadence;
        metrics_state.current_metrics.ground_contact_ms = step_cycle_ms * 0.35f;
    }
    
    // Flight time estimation
    if (current_cadence > 0 && metrics_state.current_metrics.ground_contact_ms > 0) {
        uint32_t step_cycle_ms = 60000 / current_cadence;
        uint32_t estimated_flight = step_cycle_ms - metrics_state.current_metrics.ground_contact_ms;
        if (estimated_flight > 0 && estimated_flight < 1000) {
            metrics_state.current_metrics.flight_time_ms = estimated_flight;
        }
    }
    
    // Set strike pattern for left foot
    metrics_state.current_metrics.left_strike_pattern = data->left_contact_phase == 1 ? 0 :
                                                       (data->left_fore_pct > 50 ? 2 : 1);
    
    // Set pronation for left foot
    metrics_state.current_metrics.avg_pronation_deg = data->left_pronation_deg;
    
    // Simple form score based on left foot only
    metrics_state.current_metrics.form_score = 85;  // Default reasonable value
    
    // No bilateral metrics on secondary (balance, asymmetry set to neutral)
    metrics_state.current_metrics.balance_lr_pct = 50;  // Neutral balance
    metrics_state.current_metrics.contact_time_asymmetry = 0;
    metrics_state.current_metrics.force_asymmetry = 0;
    metrics_state.current_metrics.pronation_asymmetry = 0;
    
    // Log periodically
    if (metrics_state.metrics_count % 50 == 0) {
        LOG_INF("Secondary Metrics: Cadence=%d spm, GCT=%d ms, Pace=%d s/km",
                metrics_state.current_metrics.cadence_spm,
                metrics_state.current_metrics.ground_contact_ms,
                metrics_state.current_metrics.pace_sec_km);
        
        if (!has_bhi360) {
            LOG_INF("  Secondary using foot-sensor fallback mode (no BHI360)");
        }
    }
    
    // Send metrics to analytics/data modules (these will be transmitted via D2D)
    generic_message_t out_msg = {};
    out_msg.sender = SENDER_REALTIME_METRICS;
    out_msg.type = MSG_TYPE_REALTIME_METRICS;
    
    memcpy(&out_msg.data.realtime_metrics, &metrics_state.current_metrics,
           sizeof(realtime_metrics_t));
    
    k_msgq_put(&realtime_queue, &out_msg, K_NO_WAIT);
    
#endif  // CONFIG_PRIMARY_DEVICE
}

// Send BLE update
static void send_ble_update(void)
{
  
  //  LOG_INF("Sending BLE update: cadence=%d spm, pace=%d s/km, form=%d%%",
    //        metrics_state.current_metrics.cadence_spm,
      //      metrics_state.current_metrics.pace_sec_km,
        //    metrics_state.current_metrics.form_score);
    
    // Create message for bluetooth module
    generic_message_t ble_msg = {};
    ble_msg.sender = SENDER_REALTIME_METRICS;
    ble_msg.type = MSG_TYPE_ACTIVITY_METRICS_BLE;
    
    // Copy metrics data to message
    // Note: We need to ensure the message can carry the metrics data
    // For now, we'll use a pointer in the data union (to be defined)
    // In production, we might want to copy the data or use a shared buffer
    memcpy(&ble_msg.data, &metrics_state.current_metrics,
           sizeof(metrics_state.current_metrics) > sizeof(ble_msg.data) ?
           sizeof(ble_msg.data) : sizeof(metrics_state.current_metrics));
    
    if (ble_phone_is_connected()) {       
    // Send to bluetooth module
    int ret = k_msgq_put(&bluetooth_msgq, &ble_msg, K_NO_WAIT);
    if (ret != 0) {
        LOG_WRN("Failed to send metrics to bluetooth queue: %d", ret);  
    }
    }

    // Create message for data module
    generic_message_t data_msg = {};
    data_msg.sender = SENDER_REALTIME_METRICS;
    data_msg.type = MSG_TYPE_REALTIME_METRICS_DATA;  // You may need to define this in your message types
    
    // Copy metrics data to message
    memcpy(&data_msg.data, &metrics_state.current_metrics,
           sizeof(metrics_state.current_metrics) > sizeof(data_msg.data) ?
           sizeof(data_msg.data) : sizeof(metrics_state.current_metrics));
    
    
    // Send to data module
    int ret = k_msgq_put(&data_msgq, &data_msg, K_NO_WAIT);
    if (ret != 0) {
        LOG_WRN("Failed to send metrics to data queue: %d", ret);
    }
    
    // Log alerts if any
    if (metrics_state.current_metrics.alerts != 0) {
        LOG_WRN("Alerts active: 0x%02x", metrics_state.current_metrics.alerts);
        if (metrics_state.current_metrics.alerts & RT_ALERT_HIGH_ASYMMETRY) {
            LOG_WRN("High asymmetry detected: %d%%", 
                    metrics_state.current_metrics.contact_time_asymmetry);
        }
        if (metrics_state.current_metrics.alerts & RT_ALERT_POOR_FORM) {
            LOG_WRN("Poor running form: score %d%%", 
                    metrics_state.current_metrics.form_score);
        }
        if (metrics_state.current_metrics.alerts & RT_ALERT_OVERPRONATION) {
            LOG_WRN("Overpronation detected: %d degrees", 
                    metrics_state.current_metrics.avg_pronation_deg);
        }
    }
}

// Module event handler
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh)) {
         auto *event = cast_module_state_event(aeh);
        
        // Wait for sensor_data module to be ready
        if (check_state(event, MODULE_ID(foot_sensor), MODULE_STATE_READY)) {
            if (!module_initialized) {
                LOG_INF("Got realtime metric Init");
                realtime_metrics_init();
            }
        }
        return false;
    }
    
    // Handle start activity events
    if (is_motion_sensor_start_activity_event(aeh)) {
        LOG_INF("Realtime metrics: Activity started");
        atomic_set(&processing_active, 1);
        // Could trigger processing start here if needed
        return false;
    }
    
    // Handle stop activity events
    if (is_motion_sensor_stop_activity_event(aeh)) {
        LOG_INF("Realtime metrics: Activity stopped");
        atomic_set(&processing_active, 0);
        // Could trigger processing stop here if needed
        return false;
    }
    
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_state_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_sensor_stop_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_stop_activity_event);
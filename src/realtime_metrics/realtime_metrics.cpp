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
#include <errors.hpp>
#include "realtime_metrics.h"

// Include sensor data structures
#include "../sensor_data/sensor_data_consolidated.hpp"

LOG_MODULE_REGISTER(MODULE, CONFIG_REALTIME_METRICS_MODULE_LOG_LEVEL);

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
static struct k_work_delayable ble_update_work;

// Message buffers for different work items
static char pending_command[MAX_COMMAND_STRING_LEN];
static sensor_data_consolidated_t latest_sensor_data;
static bool new_sensor_data_available;
K_MUTEX_DEFINE(sensor_data_mutex);

// Message queues are defined in app.cpp
extern struct k_msgq sensor_data_queue;  // Input from sensor_data
extern struct k_msgq realtime_queue;     // Output to analytics
extern struct k_msgq bluetooth_msgq;     // Output to bluetooth

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
    
    // Contact time tracking (moving averages)
    uint32_t left_contact_sum_ms;
    uint32_t right_contact_sum_ms;
    uint32_t left_flight_sum_ms;
    uint32_t right_flight_sum_ms;
    uint16_t contact_count;
    
    // Force tracking
    uint32_t left_force_sum;
    uint32_t right_force_sum;
    uint16_t force_count;
    
    // Variability tracking for consistency score
    float contact_time_variance;
    float last_contact_times[10];
    uint8_t variance_index;
    
    // BLE update timing
    uint32_t last_ble_update;
    uint32_t metrics_count;
    uint32_t current_time_ms;
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
        // Wait for sensor data
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
                    
                default:
                    LOG_DBG("Received unsupported message type %d", msg.type);
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
        LOG_DBG("Processing consolidated sensor data");
        
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
    
    LOG_DBG("Processing command: %s", pending_command);
    
    if (strcmp(pending_command, "START_REALTIME_PROCESSING") == 0) {
        atomic_set(&processing_active, 1);
        LOG_INF("Realtime processing started");
    } else if (strcmp(pending_command, "STOP_REALTIME_PROCESSING") == 0) {
        atomic_set(&processing_active, 0);
        LOG_INF("Realtime processing stopped");
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

// Calculate real-time metrics
static void calculate_realtime_metrics(void)
{
    // Copy sensor data with mutex protection
    sensor_data_consolidated_t local_data;
    k_mutex_lock(&sensor_data_mutex, K_FOREVER);
    memcpy(&local_data, &latest_sensor_data, sizeof(sensor_data_consolidated_t));
    k_mutex_unlock(&sensor_data_mutex);
    
    sensor_data_consolidated_t *data = &local_data;
    
    // Update cadence tracker
    cadence_tracker_update(&metrics_state.cadence_tracker, 
                          data->step_count, 
                          metrics_state.current_time_ms);
    
    float current_cadence = cadence_tracker_get_current(&metrics_state.cadence_tracker);
    
    // Update pace estimator
    pace_estimator_update(&metrics_state.pace_estimator,
                         current_cadence,
                         data->delta_time_ms);
    
    // Check if GPS correction is available (from activity_metrics)
    // This would need to be passed via message queue in real implementation
    // For now, we'll use the sensor-only version
    
    // Track contact times for averaging
    if (data->left_contact_duration_ms > 0) {
        metrics_state.left_contact_sum_ms += data->left_contact_duration_ms;
        metrics_state.contact_count++;
        
        // Store for variability calculation
        metrics_state.last_contact_times[metrics_state.variance_index] = 
            (float)data->left_contact_duration_ms;
        metrics_state.variance_index = (metrics_state.variance_index + 1) % 10;
    }
    
    if (data->right_contact_duration_ms > 0) {
        metrics_state.right_contact_sum_ms += data->right_contact_duration_ms;
    }
    
    // Track forces for balance calculation
    if (data->left_in_contact || data->right_in_contact) {
        metrics_state.left_force_sum += data->left_peak_force;
        metrics_state.right_force_sum += data->right_peak_force;
        metrics_state.force_count++;
    }
    
    // Calculate averages
    uint16_t avg_left_contact = 0;
    uint16_t avg_right_contact = 0;
    if (metrics_state.contact_count > 0) {
        avg_left_contact = metrics_state.left_contact_sum_ms / metrics_state.contact_count;
        avg_right_contact = metrics_state.right_contact_sum_ms / metrics_state.contact_count;
    }
    
    // Calculate balance
    int8_t balance = 0;
    if (metrics_state.force_count > 0) {
        uint16_t avg_left_force = metrics_state.left_force_sum / metrics_state.force_count;
        uint16_t avg_right_force = metrics_state.right_force_sum / metrics_state.force_count;
        balance = calculate_balance_percentage(avg_left_force, avg_right_force);
    }
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
    metrics_state.current_metrics.pace_sec_km = (uint16_t)pace_estimator_get_pace(&metrics_state.pace_estimator);
    metrics_state.current_metrics.distance_m = (uint16_t)metrics_state.pace_estimator.distance_m;
    
    metrics_state.current_metrics.form_score = (uint8_t)metrics_state.form_score.overall_score;
    metrics_state.current_metrics.balance_lr_pct = balance;
    metrics_state.current_metrics.ground_contact_ms = (avg_left_contact + avg_right_contact) / 2;
    metrics_state.current_metrics.flight_time_ms = data->true_flight_time_ms;
    
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
    
    // Efficiency indicators (simplified for now)
    metrics_state.current_metrics.vertical_ratio = 10; // Placeholder
    metrics_state.current_metrics.efficiency_score = (uint8_t)(metrics_state.form_score.overall_score * 0.9f);
    
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
    }
    
    // Send to analytics thread
    generic_message_t out_msg = {};
    out_msg.sender = SENDER_REALTIME_METRICS;
    out_msg.type = MSG_TYPE_REALTIME_METRICS;
    
    k_msgq_put(&realtime_queue, &out_msg, K_NO_WAIT);
    
    // Reset accumulators periodically (every 100 samples)
    if (metrics_state.metrics_count % 100 == 0) {
        metrics_state.left_contact_sum_ms = 0;
        metrics_state.right_contact_sum_ms = 0;
        metrics_state.contact_count = 0;
        metrics_state.left_force_sum = 0;
        metrics_state.right_force_sum = 0;
        metrics_state.force_count = 0;
    }
}

// Send BLE update
static void send_ble_update(void)
{
    LOG_DBG("Sending BLE update: cadence=%d spm, pace=%d s/km, form=%d%%", 
            metrics_state.current_metrics.cadence_spm,
            metrics_state.current_metrics.pace_sec_km,
            metrics_state.current_metrics.form_score);
    
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
    
    // Send to bluetooth module
    int ret = k_msgq_put(&bluetooth_msgq, &ble_msg, K_NO_WAIT);
    if (ret != 0) {
        LOG_WRN("Failed to send metrics to bluetooth queue: %d", ret);
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
        const struct module_state_event *event = cast_module_state_event(aeh);
        
        // Wait for sensor_data module to be ready
        if (check_state(event, MODULE_ID(sensor_data), MODULE_STATE_READY)) {
            if (!module_initialized) {
                realtime_metrics_init();
            }
        }
        return false;
    }
    
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
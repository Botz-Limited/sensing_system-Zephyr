/**
 * @file activity_metrics_module.cpp
 * @brief Activity metrics processing module with dedicated thread
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#define MODULE activity_metrics

#include <zephyr/logging/log.h>
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <math.h>
#include <string.h>
#include <stdio.h>


#include <app.hpp>
#include <events/app_state_event.h>
#include <app_event_manager.h>
#include <activity_session.hpp>
#include <step_detection.hpp>
#include <activity_metrics.hpp>
#include <errors.hpp>
#include <ble_services.hpp>
#include <events/app_state_event.h>
#include <events/foot_sensor_event.h>
#include <events/motion_sensor_event.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_ACTIVITY_METRICS_MODULE_LOG_LEVEL);

// Helper macros
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

// Thread configuration
static constexpr int activity_metrics_stack_size = CONFIG_ACTIVITY_METRICS_MODULE_STACK_SIZE;
static constexpr int activity_metrics_priority = CONFIG_ACTIVITY_METRICS_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(activity_metrics_stack_area, activity_metrics_stack_size);
static struct k_thread activity_metrics_thread_data;
static k_tid_t activity_metrics_tid;

// Work queue configuration
static constexpr int activity_metrics_workq_stack_size = 4096;
K_THREAD_STACK_DEFINE(activity_metrics_workq_stack, activity_metrics_workq_stack_size);
static struct k_work_q activity_metrics_work_q;

/**
 * THREAD-SAFE WORK ITEM PATTERN
 * 

 * 
 * Each work item now contains its own copy of the data. We use double buffering
 * to allow one work item to be processed while another is being prepared.
 * 
 * How it works:
 * 1. Message arrives in activity_metrics_thread_fn
 * 2. Get next buffer index using atomic_inc() & 1 (alternates between 0 and 1)
 * 3. Copy message data into the work item's embedded data structure
 * 4. Submit the work item to the work queue
 * 5. Work handler uses CONTAINER_OF to get the work item and access its data
 * 
 * This ensures each work handler gets the exact data that was intended for it,
 * even if multiple messages arrive in quick succession.
 */

// Work item structures that include data to avoid race conditions
struct foot_data_work {
    struct k_work work;        // Zephyr work item (must be first member)
    foot_samples_t data;       // Foot pressure sensor data
    uint8_t foot_id;          // Which foot: 0=left, 1=right
};

struct bhi360_data_work {
    struct k_work work;        // Zephyr work item (must be first member)
    union {
        bhi360_log_record_t log_record;   // Full BHI360 sensor data
        bhi360_step_count_t step_count;   // Just step count update
    } data;
    msg_type_t type;          // Message type to distinguish union member
};

struct command_work {
    struct k_work work;        // Zephyr work item (must be first member)
    char command[MAX_COMMAND_STRING_LEN];  // Command string
};
// Static work item arrays for double buffering
// Double buffering allows one item to be processed while another is being prepared
static struct foot_data_work foot_work_items[2];      // Two buffers for foot data
static struct bhi360_data_work bhi360_work_items[2];  // Two buffers for BHI360 data
static struct command_work command_work_items[2];     // Two buffers for commands

// These work items don't need double buffering as they're triggered differently
static struct k_work_delayable periodic_update_work;   // Periodic 1Hz updates
static struct k_work weight_measurement_work;          // Weight measurement process
static struct k_work weight_calibration_work;          // Weight calibration process

// Atomic indices for selecting which buffer to use (0 or 1)
// atomic_inc() returns previous value, so (atomic_inc() & 1) alternates between 0 and 1
static atomic_t foot_work_idx = ATOMIC_INIT(0);
static atomic_t bhi360_work_idx = ATOMIC_INIT(0);
static atomic_t command_work_idx = ATOMIC_INIT(0);

// Message queues are defined in app.cpp
extern struct k_msgq activity_metrics_msgq;  // Input queue for this module
extern struct k_msgq sensor_data_msgq;       // Output to sensor_data module
extern struct k_msgq realtime_queue;         // Output to realtime_metrics module
extern struct k_msgq bluetooth_msgq;         // Output to bluetooth module
extern struct k_msgq data_msgq;              // Output to data module

// Module state
static bool module_initialized = false;
static atomic_t processing_active = ATOMIC_INIT(0);

// Activity session state
static ActivitySessionState session_state;

// Timing for periodic updates
static uint32_t last_periodic_update = 0;
static const uint32_t PERIODIC_UPDATE_INTERVAL_MS = 1000; // 1 second for data logging

// BLE update timing
static uint32_t last_ble_update = 0;
static const uint32_t BLE_UPDATE_INTERVAL_MS = 1000; // 1Hz for BLE

// Latest sensor data
static struct {
    // From BHI360
    float quat_w, quat_x, quat_y, quat_z;
    float lacc_x, lacc_y, lacc_z;
    float gyro_x, gyro_y, gyro_z;
    uint32_t step_count;
    uint32_t last_step_count;
    uint32_t last_step_time;
    
    // From foot sensors
    float left_pressure[8];
    float right_pressure[8];
    uint32_t last_left_pressure_update;
    uint32_t last_right_pressure_update;
    
    // Calculated metrics
    float current_cadence;
    float avg_contact_time;
    float avg_flight_time;
    uint8_t primary_strike_pattern;
    
    // Rolling buffers
    float contact_time_buffer[10];
    float flight_time_buffer[10];
    int buffer_index;
    
    // Weight measurement
    float calculated_weight_kg;
    uint32_t last_weight_measurement_time;
    bool weight_measurement_valid;
    
} sensor_data;

// Weight measurement state
static struct {
    bool measurement_in_progress;
    bool calibration_mode;  // True when doing calibration, false for normal measurement
    uint32_t measurement_start_time;
    float pressure_sum_buffer[20];  // Buffer for averaging
    int buffer_count;
    float motion_threshold;
    uint32_t stable_samples_required;
    uint32_t stable_samples_count;
    bool debug_mode;  // Show raw ADC values for calibration
    float known_weight_kg;  // Known weight for calibration (0 if not provided)
} weight_measurement;

// Weight calibration variables - can be updated at runtime
static struct {
    float scale_factor;     // Newtons per ADC unit
    float nonlinear_a;      // Nonlinear coefficient A
    float nonlinear_b;      // Nonlinear coefficient B
    float nonlinear_c;      // Nonlinear coefficient C
    float temp_coeff;       // Temperature coefficient (%/°C)
    float temp_ref;         // Reference temperature (°C)
} weight_cal = {
    .scale_factor = 0.0168f,   // ~100N max per sensor (16 sensors total)
    .nonlinear_a = 1.0f,       // Linear model by default
    .nonlinear_b = 0.0f,
    .nonlinear_c = 0.0f,
    .temp_coeff = 0.0f,
    .temp_ref = 25.0f
};

// Forward declarations
static void activity_metrics_init(void);
/**
 * @brief Main processing thread for activity metrics, waits for messages and queues work.
 * @param arg1 Unused.
 * @param arg2 Unused.
 * @param arg3 Unused.
 * @note This function is complete and handles incoming messages for foot sensor data, BHI360 data, and commands.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet on primary device).
 */
static void activity_metrics_thread_fn(void *arg1, void *arg2, void *arg3);
/**
 * @brief Processes foot sensor data to update sensor state and detect steps.
 * @param data Pointer to foot sensor sample data.
 * @param foot Foot identifier (0 for left, 1 for right).
 * @note This function is complete and processes pressure data for step detection and metrics calculation.
 *       Data Requirements: Foot sensor data (specific to one foot per call).
 */
static void process_foot_sensor_data(const foot_samples_t *data, uint8_t foot);
/**
 * @brief Processes BHI360 IMU data to update sensor state.
 * @param msg Pointer to the generic message containing BHI360 data.
 * @note This function is complete and updates quaternion, acceleration, gyro, and step count data.
 *       Data Requirements: Motion sensor data (BHI360).
 */
static void process_bhi360_data(const generic_message_t *msg);
/**
 * @brief Calculates real-time metrics such as cadence and form score.
 * @note This function is complete and calculates metrics based on current sensor data.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet for comprehensive metrics).
 */
static void calculate_realtime_metrics(void);
/**
 * @brief Sends periodic activity records to the data module for logging.
 * @note This function is a placeholder with TODO for proper message structure. Currently logs metrics only.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet).
 */
static void send_periodic_record(void);
/**
 * @brief Sends activity metrics updates to Bluetooth for real-time feedback.
 * @note This function is partially complete with TODO for defining proper message type. It calculates and sends BLE updates.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet).
 */
static void send_ble_update(void);
/**
 * @brief Work handler for processing foot sensor data from the queue.
 * @param work Pointer to the work item.
 * @note This function is complete and processes foot data safely using double buffering.
 *       Data Requirements: Foot sensor data (specific to one foot per call).
 */
static void process_foot_data_work_handler(struct k_work *work);
/**
 * @brief Work handler for processing BHI360 IMU data from the queue.
 * @param work Pointer to the work item.
 * @note This function is complete and processes IMU data safely using double buffering.
 *       Data Requirements: Motion sensor data (BHI360).
 */
static void process_bhi360_data_work_handler(struct k_work *work);
/**
 * @brief Work handler for processing commands such as start/stop activity.
 * @param work Pointer to the work item.
 * @note This function is complete and handles commands for activity session management and weight measurement.
 *       Data Requirements: None directly, but commands may trigger data processing from both motion and foot sensors.
 */
static void process_command_work_handler(struct k_work *work);
/**
 * @brief Work handler for periodic updates of metrics and BLE notifications.
 * @param work Pointer to the work item.
 * @note This function is complete and performs periodic updates at 1Hz for data logging and BLE.
 *       Data Requirements: Motion sensor data (BHI360), Foot sensor data (both feet).
 */
static void periodic_update_work_handler(struct k_work *work);
/**
 * @brief Work handler for initiating and processing weight measurement.
 * @param work Pointer to the work item.
 * @note This function is complete and manages the weight measurement process.
 *       Data Requirements: Foot sensor data (both feet for accurate measurement).
 */
static void weight_measurement_work_handler(struct k_work *work);
/**
 * @brief Work handler for weight calibration procedure.
 * @param work Pointer to the work item.
 * @note This function is complete and handles weight calibration with user interaction.
 *       Data Requirements: Foot sensor data (both feet for calibration).
 */
static void weight_calibration_work_handler(struct k_work *work);
/**
 * @brief Calculates running pace based on cadence.
 * @param cadence Current cadence in steps per minute.
 * @return Estimated pace in seconds per kilometer.
 * @note This function is complete and estimates pace using cadence and height-based stride length.
 *       Data Requirements: Motion sensor data (BHI360 for cadence).
 */
static float calculate_pace_from_cadence(float cadence);
/**
 * @brief Calculates balance score between left and right foot contact times.
 * @return Balance score as a signed integer representing asymmetry percentage.
 * @note This function is complete and calculates balance based on contact time differences.
 *       Data Requirements: Foot sensor data (both feet).
 */
static int8_t calculate_balance_score(void);
/**
 * @brief Calculates overall form score for running technique.
 * @return Form score as a percentage (0-100).
 * @note This function is complete and assesses form based on cadence, contact time, balance, and strike pattern.
 *       Data Requirements: Motion sensor data (BHI360 for cadence), Foot sensor data (both feet for contact and strike).
 */
static uint8_t calculate_form_score(void);
/**
 * @brief Calculates efficiency score based on contact and flight times.
 * @param contact_time Ground contact time in milliseconds.
 * @param flight_time Flight time in milliseconds.
 * @return Efficiency score as a percentage (0-100).
 * @note This function is complete and calculates efficiency using duty factor.
 *       Data Requirements: Foot sensor data (both feet for contact and flight times).
 */
static uint8_t calculate_efficiency_score(float contact_time, float flight_time);
/**
 * @brief Calculates fatigue level based on changes in contact time.
 * @return Fatigue level as a percentage (0-100).
 * @note This function is complete and estimates fatigue from contact time increase compared to baseline.
 *       Data Requirements: Foot sensor data (both feet for contact time).
 */
static uint8_t calculate_fatigue_level(void);
/**
 * @brief Initiates the weight measurement process.
 * @note This function is complete and sets up state for weight measurement.
 *       Data Requirements: Foot sensor data (both feet).
 */
static void start_weight_measurement(void);
/**
 * @brief Processes weight measurement data until completion.
 * @note This function is complete and handles weight calculation during measurement.
 *       Data Requirements: Foot sensor data (both feet), Motion sensor data (BHI360 for motion detection).
 */
static void process_weight_measurement(void);
/**
 * @brief Calculates weight from foot pressure data.
 * @return Calculated weight in kilograms.
 * @note This function is complete and converts pressure readings to weight using calibration.
 *       Data Requirements: Foot sensor data (both feet).
 */
static float calculate_weight_from_pressure(void);
/**
 * @brief Applies calibration to raw weight measurement.
 * @param raw_weight_kg Raw weight in kilograms.
 * @return Calibrated weight in kilograms.
 * @note This function is complete and applies calibration factors to weight measurement.
 *       Data Requirements: Foot sensor data (both feet for raw weight).
 */
static float apply_weight_calibration(float raw_weight_kg);
/**
 * @brief Checks if a person is standing still based on motion data.
 * @return True if person is standing still, false otherwise.
 * @note This function is complete and uses acceleration and gyro data to detect stillness.
 *       Data Requirements: Motion sensor data (BHI360 for acceleration and gyro).
 */
static bool is_person_standing_still(void);
/**
 * @brief Performs weight calibration using measured pressure data.
 * @note This function is complete and calculates new calibration factors based on known or default weight.
 *       Data Requirements: Foot sensor data (both feet for pressure readings).
 */
static void perform_weight_calibration(void);
static uint32_t calculate_total_distance(void);
static void update_splits();

// Initialize the activity metrics module
static void activity_metrics_init(void)
{
    LOG_INF("Initializing activity metrics module");
    
    // Initialize step detection
    step_detection_init();
    
    // Initialize session state
    memset(&session_state, 0, sizeof(session_state));
    memset(&sensor_data, 0, sizeof(sensor_data));
    
    // Initialize weight measurement state
    memset(&weight_measurement, 0, sizeof(weight_measurement));
    weight_measurement.motion_threshold = 0.5f; // m/s² threshold for standing still
    weight_measurement.stable_samples_required = 30; // 3 seconds at 10Hz
    
    // Initialize work queue
    k_work_queue_init(&activity_metrics_work_q);
    k_work_queue_start(&activity_metrics_work_q, activity_metrics_workq_stack,
                       K_THREAD_STACK_SIZEOF(activity_metrics_workq_stack),
                       activity_metrics_priority - 1, NULL);
    k_thread_name_set(&activity_metrics_work_q.thread, "activity_metrics_wq");
    
    // Initialize work items with double buffering
    for (int i = 0; i < 2; i++) {
        k_work_init(&foot_work_items[i].work, process_foot_data_work_handler);
        k_work_init(&bhi360_work_items[i].work, process_bhi360_data_work_handler);
        k_work_init(&command_work_items[i].work, process_command_work_handler);
    }
    k_work_init_delayable(&periodic_update_work, periodic_update_work_handler);
    k_work_init(&weight_measurement_work, weight_measurement_work_handler);
    k_work_init(&weight_calibration_work, weight_calibration_work_handler);
    
    // Register step event callback
    step_detection_register_callback([](const StepEvent* event) {
        // Process step events
        if (event->foot == 0) { // Left foot
            // Boundary check and rolling mechanism for total_steps_left
            // Risk: Overflows after ~4 billion steps (unlikely, but prevent silent wrap)
            // Strategy: If approaching UINT32_MAX, reset to 0 and trigger session summary export
            // This rolls over safely without losing data (export before reset)
            if (session_state.total_steps_left >= UINT32_MAX - 1) {
                // Trigger export of current session data before reset
                // TODO: Implement session summary export to data module
                LOG_WRN("total_steps_left rolling over - exporting session data");
                session_state.total_steps_left = 0; // Reset to 0
            }
            session_state.total_steps_left++;
        } else { // Right foot
            // Boundary check and rolling mechanism for total_steps_right
            // Same strategy as above
            if (session_state.total_steps_right >= UINT32_MAX - 1) {
                // Trigger export of current session data before reset
                // TODO: Implement session summary export to data module
                LOG_WRN("total_steps_right rolling over - exporting session data");
                session_state.total_steps_right = 0; // Reset to 0
            }
            session_state.total_steps_right++;
        }
        
        // Update rolling buffers
        int idx = sensor_data.buffer_index;
        sensor_data.contact_time_buffer[idx] = event->contact_time_ms;
        sensor_data.flight_time_buffer[idx] = event->flight_time_ms;
        sensor_data.buffer_index = (idx + 1) % 10;
        
        // Calculate averages
        float total_contact = 0, total_flight = 0;
        int valid_samples = 0;
        
        for (int i = 0; i < 10; i++) {
            if (sensor_data.contact_time_buffer[i] > 0) {
                total_contact += sensor_data.contact_time_buffer[i];
                total_flight += sensor_data.flight_time_buffer[i];
                valid_samples++;
            }
        }
        
        if (valid_samples > 0) {
            sensor_data.avg_contact_time = total_contact / valid_samples;
            sensor_data.avg_flight_time = total_flight / valid_samples;
        }
        
        LOG_WRN("Step event: foot=%d, contact=%.1fms, flight=%.1fms, strike=%d",
        event->foot, (double)event->contact_time_ms, (double)event->flight_time_ms, event->strike_pattern);
        });
        
        // Request weight calibration data from data module
        LOG_INF("Activity Metrics: Requesting weight calibration data from data module");
        generic_message_t request_msg;
        memset(&request_msg, 0, sizeof(request_msg));
        request_msg.sender = SENDER_ACTIVITY_METRICS;
        request_msg.type = MSG_TYPE_REQUEST_WEIGHT_CALIBRATION;
        
        if (k_msgq_put(&data_msgq, &request_msg, K_NO_WAIT) != 0) {
        LOG_ERR("Failed to request weight calibration data from data module");
        }
        
        // Create the processing thread
    activity_metrics_tid = k_thread_create(
        &activity_metrics_thread_data,
        activity_metrics_stack_area,
        K_THREAD_STACK_SIZEOF(activity_metrics_stack_area),
        activity_metrics_thread_fn,
        NULL, NULL, NULL,
        activity_metrics_priority,
        0,
        K_NO_WAIT
    );
    
    k_thread_name_set(activity_metrics_tid, "activity_metrics");
    
    module_initialized = true;
    module_set_state(MODULE_STATE_READY);
    LOG_INF("Activity metrics module initialized");
}

// Main processing thread - waits for messages and queues appropriate work
static void activity_metrics_thread_fn(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    
    generic_message_t msg;
    
    while (true) {
        // Wait for messages
        int ret = k_msgq_get(&activity_metrics_msgq, &msg, K_FOREVER);
        
        if (ret == 0) {
            // Queue different work based on message type
            switch (msg.type) {
                case MSG_TYPE_FOOT_SAMPLES: {
                    /**
                     * FOOT SENSOR DATA HANDLING
                     * 
                     * Double buffering pattern:
                     * 1. atomic_inc(&foot_work_idx) increments the index atomically
                     * 2. & 1 ensures we alternate between 0 and 1
                     * 3. This allows concurrent processing without data corruption
                     * 
                     * Example sequence:
                     * - Message 1: idx = 0, uses foot_work_items[0]
                     * - Message 2: idx = 1, uses foot_work_items[1]
                     * - Message 3: idx = 0, reuses foot_work_items[0] (safe if handler finished)
                     */
                    int idx = atomic_inc(&foot_work_idx) & 1;
                    struct foot_data_work *work_item = &foot_work_items[idx];
                    
                    // Copy foot pressure data into work item's buffer
                    memcpy(&work_item->data, &msg.data.foot_samples, sizeof(foot_samples_t));
                    
                    // Determine which foot based on message sender
                    // This is needed because the primary device handles right foot locally
                    // and receives left foot data from secondary via D2D
                    if (msg.sender == SENDER_FOOT_SENSOR_THREAD) {
                        #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                        work_item->foot_id = 1; // Primary device's local sensor = right foot
                        #else
                        work_item->foot_id = 0; // Secondary device's local sensor = left foot
                        #endif
                    } else if (msg.sender == SENDER_D2D_SECONDARY) {
                        work_item->foot_id = 0; // Data from secondary is always left foot
                    }
                    
                    // Submit work item - the handler will process this specific data copy
                    k_work_submit_to_queue(&activity_metrics_work_q, &work_item->work);
                    break;
                }
                    
                case MSG_TYPE_BHI360_LOG_RECORD: {
                    // Get next work item index
                    int idx = atomic_inc(&bhi360_work_idx) & 1;
                    struct bhi360_data_work *work_item = &bhi360_work_items[idx];
                    
                    // Copy data to work item
                    memcpy(&work_item->data.log_record, &msg.data.bhi360_log_record, sizeof(bhi360_log_record_t));
                    work_item->type = msg.type;
                    
                    k_work_submit_to_queue(&activity_metrics_work_q, &work_item->work);
                    break;
                }
                    
                case MSG_TYPE_BHI360_STEP_COUNT: {
                    // Get next work item index
                    int idx = atomic_inc(&bhi360_work_idx) & 1;
                    struct bhi360_data_work *work_item = &bhi360_work_items[idx];
                    
                    // Copy data to work item
                    memcpy(&work_item->data.step_count, &msg.data.bhi360_step_count, sizeof(bhi360_step_count_t));
                    work_item->type = msg.type;
                    
                    k_work_submit_to_queue(&activity_metrics_work_q, &work_item->work);
                    break;
                }
                    
                case MSG_TYPE_BHI360_3D_MAPPING:
                case MSG_TYPE_BHI360_LINEAR_ACCEL: {
                    // Get next work item index
                    int idx = atomic_inc(&bhi360_work_idx) & 1;
                    struct bhi360_data_work *work_item = &bhi360_work_items[idx];
                    
                    // For these types, we just need the type
                    work_item->type = msg.type;
                    
                    k_work_submit_to_queue(&activity_metrics_work_q, &work_item->work);
                    break;
                }
                    
                case MSG_TYPE_COMMAND: {
                    // Get next work item index
                    int idx = atomic_inc(&command_work_idx) & 1;
                    struct command_work *work_item = &command_work_items[idx];
                    
                    // Copy command to work item
                    strncpy(work_item->command, msg.data.command_str, MAX_COMMAND_STRING_LEN - 1);
                    work_item->command[MAX_COMMAND_STRING_LEN - 1] = '\0';
                    
                    k_work_submit_to_queue(&activity_metrics_work_q, &work_item->work);
                    break;
                }

                case MSG_TYPE_SAVE_WEIGHT_CALIBRATION:
                    // Update weight calibration from D2D or data module
                    weight_cal.scale_factor = msg.data.weight_calibration.scale_factor;
                    weight_cal.nonlinear_a = msg.data.weight_calibration.nonlinear_a;
                    weight_cal.nonlinear_b = msg.data.weight_calibration.nonlinear_b;
                    weight_cal.nonlinear_c = msg.data.weight_calibration.nonlinear_c;
                    weight_cal.temp_coeff = msg.data.weight_calibration.temp_coeff;
                    weight_cal.temp_ref = msg.data.weight_calibration.temp_ref;
                    LOG_INF("Weight calibration updated: scale=%.6f, offset=%.6f", 
                           (double)weight_cal.scale_factor, (double)weight_cal.nonlinear_a);
                    break;

                case MSG_TYPE_WEIGHT_MEASUREMENT:
                    // Trigger weight measurement
                    LOG_INF("Weight measurement requested");
                    k_work_submit_to_queue(&activity_metrics_work_q, &weight_measurement_work);
                    break;
                    
                case MSG_TYPE_GPS_UPDATE:
                    // Process GPS update
                    LOG_INF("GPS update received");
                    if (session_state.session_active) {
                        // Process GPS data for stride calibration
                        activity_session_process_gps_update(&msg.data.gps_update);
                    } else {
                        LOG_WRN("GPS update received but no active session");
                    }
                    break;
                    
                case MSG_TYPE_START_WEIGHT_CALIBRATION:
                    // Check if we have known weight data
                    if (msg.data.weight_calibration_step.known_weight_kg > 0) {
                        // Store the known weight for calibration
                        weight_measurement.known_weight_kg = msg.data.weight_calibration_step.known_weight_kg;
                        LOG_INF("Weight calibration requested with known weight: %.1f kg", 
                                (double)weight_measurement.known_weight_kg);
                    } else {
                        // Legacy mode - no known weight provided
                        weight_measurement.known_weight_kg = 0;
                        LOG_INF("Weight calibration procedure requested (no known weight)");
                    }
                    k_work_submit_to_queue(&activity_metrics_work_q, &weight_calibration_work);
                    break;
                    
                case MSG_TYPE_WEIGHT_CALIBRATION_DATA:
                    // Update weight calibration from data module
                    LOG_INF("Received weight calibration data from data module");
                    weight_cal.scale_factor = msg.data.weight_calibration.scale_factor;
                    weight_cal.nonlinear_a = msg.data.weight_calibration.nonlinear_a;
                    weight_cal.nonlinear_b = msg.data.weight_calibration.nonlinear_b;
                    weight_cal.nonlinear_c = msg.data.weight_calibration.nonlinear_c;
                    weight_cal.temp_coeff = msg.data.weight_calibration.temp_coeff;
                    weight_cal.temp_ref = msg.data.weight_calibration.temp_ref;
                    if (msg.data.weight_calibration.is_calibrated) {
                        LOG_INF("Weight calibration loaded: scale=%.6f", 
                                (double)weight_cal.scale_factor);
                    } else {
                        LOG_INF("Using default weight calibration values");
                    }
                    break;
                    
                default:
                    LOG_WRN("Received unsupported message type %d from %s", 
                            msg.type, get_sender_name(msg.sender));
                    break;
            }
        }
    }
}

/**
 * WORK HANDLER: Process foot sensor data
 * 
 * This handler is called by the work queue thread when foot sensor data is ready.
 * It uses the CONTAINER_OF macro to get the full work item structure from the k_work pointer.
 * 
 * CONTAINER_OF works because:
 * - The k_work structure is the first member of foot_data_work
 * - Given a pointer to k_work, we can calculate the address of the containing structure
 * - This gives us access to the data that was copied when the work was submitted
 * 
 * Thread safety: Each work item has its own data copy, so no race conditions
 */
static void process_foot_data_work_handler(struct k_work *work)
{
    // Get the containing work item structure from the k_work pointer
    // This is safe because k_work is the first member of foot_data_work
    struct foot_data_work *work_item = CONTAINER_OF(work, struct foot_data_work, work);
    
    LOG_WRN("Processing foot sensor data for foot %d", work_item->foot_id);
    
    // Process the foot data that was copied into this work item
    // work_item->data contains the exact data that was in the message
    process_foot_sensor_data(&work_item->data, work_item->foot_id);
}

// Work handler for processing BHI360 data
static void process_bhi360_data_work_handler(struct k_work *work)
{
    // Get the containing work item structure
    struct bhi360_data_work *work_item = CONTAINER_OF(work, struct bhi360_data_work, work);
    
    LOG_WRN("Processing BHI360 data, type: %d", work_item->type);
    
    // Create a temporary message with the appropriate data
    generic_message_t temp_msg;
    temp_msg.type = work_item->type;
    
    switch (work_item->type) {
        case MSG_TYPE_BHI360_LOG_RECORD:
            memcpy(&temp_msg.data.bhi360_log_record, &work_item->data.log_record, sizeof(bhi360_log_record_t));
            break;
        case MSG_TYPE_BHI360_STEP_COUNT:
            memcpy(&temp_msg.data.bhi360_step_count, &work_item->data.step_count, sizeof(bhi360_step_count_t));
            break;
        default:
            // For other types, we'll handle them in process_bhi360_data
            break;
    }
    
    process_bhi360_data(&temp_msg);
}

// Work handler for processing commands
static void process_command_work_handler(struct k_work *work)
{
    // Get the containing work item structure
    struct command_work *work_item = CONTAINER_OF(work, struct command_work, work);
    
    LOG_INF("Processing command: %s", work_item->command);
    
    if (strcmp(work_item->command, "START_ACTIVITY") == 0) {
        // Start new activity session
        SessionHeader header = {
            .session_id = k_uptime_get_32(),
            .start_timestamp = k_uptime_get_32() / 1000,
            .activity_type = ACTIVITY_TYPE_RUNNING,
            .activity_subtype = RUN_TYPE_EVERYDAY,
            .firmware_version = {1, 0, 0},
            .user_weight_kg = (uint16_t)(sensor_data.weight_measurement_valid ? 
                             (sensor_data.calculated_weight_kg * 10) : 700),
            .user_height_cm = 175,
            .user_age = 30,
            .user_gender = 0,
            .left_battery_pct = 100,
            .right_battery_pct = 100,
            .calibration_id = 0,
            .gps_mode = GPS_MODE_OFF,
            .reserved = {0}
        };
        activity_session_start(&header);
        atomic_set(&processing_active, 1);
        // Start periodic update work
        k_work_schedule_for_queue(&activity_metrics_work_q, &periodic_update_work, K_MSEC(100));
        LOG_INF("Activity session started");
        
        // Send start commands to other modules
        generic_message_t start_msg;
        memset(&start_msg, 0, sizeof(start_msg));
        start_msg.sender = SENDER_ACTIVITY_METRICS;
        start_msg.type = MSG_TYPE_COMMAND;
        
        // Start sensor data processing
        strcpy(start_msg.data.command_str, "START_SENSOR_PROCESSING");
        k_msgq_put(&sensor_data_msgq, &start_msg, K_NO_WAIT);
        
        // Start realtime metrics processing
        strcpy(start_msg.data.command_str, "START_REALTIME_PROCESSING");
        k_msgq_put(&realtime_queue, &start_msg, K_NO_WAIT);
        
        LOG_INF("Sent start commands to sensor_data and realtime_metrics modules");
    } else if (strcmp(work_item->command, "STOP_ACTIVITY") == 0) {
        activity_session_stop();
        atomic_set(&processing_active, 0);
        // Cancel periodic work
        k_work_cancel_delayable(&periodic_update_work);
        LOG_INF("Activity session stopped");
        
        // Send stop commands to other modules
        generic_message_t stop_msg;
        memset(&stop_msg, 0, sizeof(stop_msg));
        stop_msg.sender = SENDER_ACTIVITY_METRICS;
        stop_msg.type = MSG_TYPE_COMMAND;
        
        // Stop sensor data processing
        strcpy(stop_msg.data.command_str, "STOP_SENSOR_PROCESSING");
        k_msgq_put(&sensor_data_msgq, &stop_msg, K_NO_WAIT);
        
        // Stop realtime metrics processing
        strcpy(stop_msg.data.command_str, "STOP_REALTIME_PROCESSING");
        k_msgq_put(&realtime_queue, &stop_msg, K_NO_WAIT);
        
        LOG_INF("Sent stop commands to sensor_data and realtime_metrics modules");
    } else if (strcmp(work_item->command, "MEASURE_WEIGHT") == 0) {
        // Normal weight measurement (not calibration)
        weight_measurement.calibration_mode = false;
        k_work_submit_to_queue(&activity_metrics_work_q, &weight_measurement_work);
        LOG_INF("Weight measurement requested");
    } else if (strncmp(work_item->command, "CALIBRATE_WEIGHT:", 17) == 0) {
        // Handle weight calibration command
        // Format: "CALIBRATE_WEIGHT:SCALE:0.0168"
        char param[32];
        float value;
        if (sscanf(work_item->command + 17, "%31[^:]:%f", param, &value) == 2) {
            if (strcmp(param, "SCALE") == 0) {
                weight_cal.scale_factor = value;
                LOG_INF("Weight calibration: scale_factor set to %.6f", (double)value);
            } else if (strcmp(param, "NONLINEAR_A") == 0) {
                weight_cal.nonlinear_a = value;
                LOG_INF("Weight calibration: nonlinear_a set to %.6f", (double)value);
            } else if (strcmp(param, "NONLINEAR_B") == 0) {
                weight_cal.nonlinear_b = value;
                LOG_INF("Weight calibration: nonlinear_b set to %.6f", (double)value);
            } else if (strcmp(param, "NONLINEAR_C") == 0) {
                weight_cal.nonlinear_c = value;
                LOG_INF("Weight calibration: nonlinear_c set to %.6f", (double)value);
            } else {
                LOG_WRN("Unknown calibration parameter: %s", param);
            }
        } else {
            LOG_WRN("Invalid calibration command format: %s", work_item->command);
        }
    } else if (strcmp(work_item->command, "GET_WEIGHT_CAL") == 0) {
        // Report current calibration values
        LOG_INF("Weight calibration values:");
        LOG_INF("  scale_factor: %.6f", (double)weight_cal.scale_factor);
        LOG_INF("  nonlinear_a: %.6f", (double)weight_cal.nonlinear_a);
        LOG_INF("  nonlinear_b: %.6f", (double)weight_cal.nonlinear_b);
        LOG_INF("  nonlinear_c: %.6f", (double)weight_cal.nonlinear_c);
    } else if (strcmp(work_item->command, "WEIGHT_DEBUG_ON") == 0) {
        weight_measurement.debug_mode = true;
        LOG_INF("Weight measurement debug mode enabled");
    } else if (strcmp(work_item->command, "WEIGHT_DEBUG_OFF") == 0) {
        weight_measurement.debug_mode = false;
        LOG_INF("Weight measurement debug mode disabled");
    } else {
        LOG_WRN("Unknown command: %s", work_item->command);
    }
}

// Work handler for periodic updates
static void periodic_update_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (atomic_get(&processing_active) == 1) {
        uint32_t now = k_uptime_get_32();
        
        // Calculate real-time metrics
        calculate_realtime_metrics();
        
        // Update total distance and splits
        session_state.total_distance_cm = calculate_total_distance();
        update_splits();
        
        // Send periodic record to data module
        if (now - last_periodic_update >= PERIODIC_UPDATE_INTERVAL_MS) {
            send_periodic_record();
            last_periodic_update = now;
        }
        
        // Send BLE update
        if (now - last_ble_update >= BLE_UPDATE_INTERVAL_MS) {
            send_ble_update();
            last_ble_update = now;
        }
        
        // Reschedule for next update
        k_work_schedule_for_queue(&activity_metrics_work_q, &periodic_update_work, K_MSEC(100));
    }
}

// Work handler for weight measurement
static void weight_measurement_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    start_weight_measurement();
    
    // Continue processing weight measurement until complete
    while (weight_measurement.measurement_in_progress) {
        process_weight_measurement();
        k_msleep(100); // Check every 100ms
    }
}

// Work handler for weight calibration
static void weight_calibration_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    LOG_INF("Starting weight calibration procedure");
    LOG_INF("Please stand still on both feet for calibration");
    
    // Set up calibration state
    weight_measurement.measurement_in_progress = true;
    weight_measurement.calibration_mode = true;
    weight_measurement.measurement_start_time = k_uptime_get_32();
    weight_measurement.buffer_count = 0;
    weight_measurement.stable_samples_count = 0;
    weight_measurement.debug_mode = true; // Enable debug for calibration
    memset(weight_measurement.pressure_sum_buffer, 0, sizeof(weight_measurement.pressure_sum_buffer));
    
    // Continue processing until calibration is complete
    while (weight_measurement.measurement_in_progress) {
        uint32_t now = k_uptime_get_32();
        
        // Check for timeout (10 seconds)
        if (now - weight_measurement.measurement_start_time > 10000) {
            LOG_WRN("Weight calibration timeout - could not get stable reading");
            weight_measurement.measurement_in_progress = false;
            weight_measurement.calibration_mode = false;
            weight_measurement.debug_mode = false;
            break;
        }
        
        // Check if person is standing still
        if (!is_person_standing_still()) {
            weight_measurement.stable_samples_count = 0;
            LOG_WRN("Motion detected - resetting stable sample count");
            k_msleep(100);
            continue;
        }
        
        // Calculate total pressure from both feet
        float total_pressure = 0;
        bool data_valid = true;
        
        // Check if we have recent data from both feet
        if (now - sensor_data.last_left_pressure_update > 200 ||
            now - sensor_data.last_right_pressure_update > 200) {
            data_valid = false;
        }
        
        if (data_valid) {
            // Sum all pressure values from both feet
            for (int i = 0; i < 8; i++) {
                total_pressure += sensor_data.left_pressure[i];
                total_pressure += sensor_data.right_pressure[i];
            }
            
            // Debug output for calibration
            if (weight_measurement.debug_mode) {
                LOG_INF("CALIBRATION DEBUG: Total ADC = %.2f", (double)total_pressure);
            }
            
            // Add to buffer
            if (weight_measurement.buffer_count < 20) {
                // Boundary check for buffer_count (though bounded by 20)
                // Risk: Theoretical overflow if loop malfunctions
                // Strategy: Cap at 20 and log; prevent increment beyond bound
                if (weight_measurement.buffer_count >= 20) {
                    LOG_WRN("buffer_count cap reached unexpectedly - resetting buffer");
                    weight_measurement.buffer_count = 0; // Reset to 0 as rolling mechanism
                    memset(weight_measurement.pressure_sum_buffer, 0, sizeof(weight_measurement.pressure_sum_buffer));
                }
                weight_measurement.pressure_sum_buffer[weight_measurement.buffer_count++] = total_pressure;
            } else {
                // Shift buffer and add new value
                for (int i = 0; i < 19; i++) {
                    weight_measurement.pressure_sum_buffer[i] = weight_measurement.pressure_sum_buffer[i + 1];
                }
                weight_measurement.pressure_sum_buffer[19] = total_pressure;
            }
            
            weight_measurement.stable_samples_count++;
            
            // Check if we have enough stable samples
            if (weight_measurement.stable_samples_count >= weight_measurement.stable_samples_required &&
                weight_measurement.buffer_count >= 10) {
                
                // Perform calibration
                perform_weight_calibration();
                weight_measurement.measurement_in_progress = false;
                weight_measurement.calibration_mode = false;
                weight_measurement.debug_mode = false;
                break;
            }
        }
        
        k_msleep(100); // Check every 100ms
    }
}

// Process foot sensor data
static void process_foot_sensor_data(const foot_samples_t *data, uint8_t foot)
{
    uint32_t timestamp = k_uptime_get_32();
    
    // Convert to float array for step detection
    float pressure_values[8];
    for (int i = 0; i < 8; i++) {
        pressure_values[i] = (float)data->values[i];
    }
    
    // Store in sensor data
    if (foot == 0) { // Left foot
        memcpy(sensor_data.left_pressure, pressure_values, sizeof(pressure_values));
        sensor_data.last_left_pressure_update = timestamp;
    } else { // Right foot
        memcpy(sensor_data.right_pressure, pressure_values, sizeof(pressure_values));
        sensor_data.last_right_pressure_update = timestamp;
    }
    
    // Process through step detection
    step_detection_process_pressure(foot, pressure_values, timestamp);
}

// Process BHI360 data
static void process_bhi360_data(const generic_message_t *msg)
{
    switch (msg->type) {
        case MSG_TYPE_BHI360_LOG_RECORD:
            // Full sensor data
            sensor_data.quat_w = msg->data.bhi360_log_record.quat_w;
            sensor_data.quat_x = msg->data.bhi360_log_record.quat_x;
            sensor_data.quat_y = msg->data.bhi360_log_record.quat_y;
            sensor_data.quat_z = msg->data.bhi360_log_record.quat_z;
            sensor_data.lacc_x = msg->data.bhi360_log_record.lacc_x;
            sensor_data.lacc_y = msg->data.bhi360_log_record.lacc_y;
            sensor_data.lacc_z = msg->data.bhi360_log_record.lacc_z;
            sensor_data.gyro_x = msg->data.bhi360_log_record.gyro_x;
            sensor_data.gyro_y = msg->data.bhi360_log_record.gyro_y;
            sensor_data.gyro_z = msg->data.bhi360_log_record.gyro_z;
            sensor_data.step_count = msg->data.bhi360_log_record.step_count;
            break;
            
        case MSG_TYPE_BHI360_STEP_COUNT: {
            sensor_data.last_step_count = sensor_data.step_count;
            sensor_data.step_count = msg->data.bhi360_step_count.step_count;
            
            // Calculate cadence from step count changes
            uint32_t now = k_uptime_get_32();
            if (sensor_data.last_step_count > 0 && sensor_data.last_step_time > 0) {
                uint32_t step_diff = sensor_data.step_count - sensor_data.last_step_count;
                uint32_t time_diff_ms = now - sensor_data.last_step_time;
                if (time_diff_ms > 0 && step_diff > 0) {
                    // Calculate cadence in steps per minute
                    sensor_data.current_cadence = (step_diff * 60000.0f) / time_diff_ms;
                }
            }
            sensor_data.last_step_time = now;
            break;
        }
            
        default:
            break;
    }
}

// Calculate real-time metrics
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static void calculate_realtime_metrics(void)
{
    // Update baseline if not established
    if (!session_state.baseline_established && sensor_data.avg_contact_time > 0) {
        // Need at least 5 valid samples
        int valid_samples = 0;
        for (int i = 0; i < 10; i++) {
            if (sensor_data.contact_time_buffer[i] > 0) {
                valid_samples++;
            }
        }
        
        if (valid_samples >= 5) {
            session_state.baseline_contact_time = sensor_data.avg_contact_time;
            session_state.baseline_form_score = calculate_form_score();
            session_state.baseline_established = true;
            LOG_INF("Baseline established: contact=%.1fms, form=%f", 
                    (double)session_state.baseline_contact_time, 
                    (double)session_state.baseline_form_score);
        }
    }
}
#else
static void calculate_realtime_metrics(void)
{
    // Secondary device: no bilateral metrics
}
#endif

// Send periodic record to data module
static void send_periodic_record(void)
{
    // For now, just log the metrics
    LOG_WRN("Periodic update: cadence=%.1f, contact=%.1fms, flight=%.1fms",
            (double)sensor_data.current_cadence, (double)sensor_data.avg_contact_time, (double)sensor_data.avg_flight_time);
    
    // TODO: Create proper message structure for activity records
    // This would send to data module for logging
}

// Send BLE update
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static void send_ble_update(void)
{
    RealtimeMetricsPacket packet;
    memset(&packet, 0, sizeof(packet));
    
    // Calculate delta time
    uint32_t now = k_uptime_get_32();
    packet.delta_time_ms = now - session_state.last_ble_update_time;
    session_state.last_ble_update_time = now;
    
    // Populate metrics
    packet.cadence_x2 = (uint16_t)(sensor_data.current_cadence * 2);
    packet.contact_time_ms = (uint16_t)sensor_data.avg_contact_time;
    packet.distance_m = (uint16_t)(session_state.total_distance_cm / 100);
    packet.step_count = sensor_data.step_count;
    
    // Calculate pace
    float pace_sec_km = calculate_pace_from_cadence(sensor_data.current_cadence);
    packet.pace_sec_per_km = (uint16_t)pace_sec_km;
    
    // Get strike pattern from step detection
    StepDetectionState left_state, right_state;
    step_detection_get_metrics(0, &left_state);
    step_detection_get_metrics(1, &right_state);
    
    // Use most common strike pattern
    packet.foot_strike = (left_state.initial_contact_region + right_state.initial_contact_region) / 2;
    
    // Composite scores
    packet.form_score = calculate_form_score();
    packet.balance_lr = calculate_balance_score();
    packet.efficiency = calculate_efficiency_score(
        sensor_data.avg_contact_time,
        sensor_data.avg_flight_time
    );
    packet.fatigue_level = calculate_fatigue_level();
    
    // Send to Bluetooth module
    // TODO: Define proper message type for activity metrics
    LOG_WRN("BLE update: pace=%d s/km, cadence=%d, form=%d",
            packet.pace_sec_per_km, packet.cadence_x2/2, packet.form_score);
}
#else
static void send_ble_update(void)
{
    // Secondary device: no BLE updates for bilateral metrics
}
#endif

// Helper functions
static float calculate_pace_from_cadence(float cadence)
{
    if (cadence < 1.0f) return 999.0f; // Max pace
    
    // Estimate stride length based on height and cadence
    float height_m = session_state.header.user_height_cm / 100.0f;
    float base_stride = height_m * 0.75f; // 75% of height
    
    // Adjust based on cadence (higher cadence = shorter stride)
    if (cadence > 180) {
        base_stride *= 0.9f;
    } else if (cadence < 160) {
        base_stride *= 1.1f;
    }
    
    // Calculate speed and pace
    float speed_ms = (cadence / 60.0f) * base_stride;
    float pace_sec_km = 1000.0f / speed_ms;
    
    return pace_sec_km;
}


#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static int8_t calculate_balance_score(void)
{
    // Get metrics from both feet
    StepDetectionState left_state, right_state;
    step_detection_get_metrics(0, &left_state);
    step_detection_get_metrics(1, &right_state);
    
    // Compare contact times
    float left_contact = left_state.ground_contact_time_ms;
    float right_contact = right_state.ground_contact_time_ms;
    
    if (left_contact > 0 && right_contact > 0) {
        float avg = (left_contact + right_contact) / 2.0f;
        float diff = left_contact - right_contact;
        float asymmetry = (diff / avg) * 100.0f;
        
        // Clamp to range
        if (asymmetry < -100) asymmetry = -100;
        if (asymmetry > 100) asymmetry = 100;
        
        return (int8_t)asymmetry;
    }
    
    return 0;
}
#else
static int8_t calculate_balance_score(void)
{
    return 0;
}
#endif

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static uint8_t calculate_form_score(void)
{
    uint8_t score = 100;
    
    // Deduct for poor cadence
    if (sensor_data.current_cadence < 160 || sensor_data.current_cadence > 200) {
        score -= 10;
    }
    
    // Deduct for high contact time
    if (sensor_data.avg_contact_time > 300) {
        score -= 15;
    }
    
    // Deduct for asymmetry
    int8_t balance = calculate_balance_score();
    if (abs(balance) > 10) {
        score -= abs(balance) / 2;
    }
    
    // Deduct for heel striking (if detected)
    StepDetectionState left_state, right_state;
    step_detection_get_metrics(0, &left_state);
    step_detection_get_metrics(1, &right_state);
    
    if (left_state.initial_contact_region == STRIKE_HEEL || 
        right_state.initial_contact_region == STRIKE_HEEL) {
        score -= 5;
    }
    
    return score;
}
#else
static uint8_t calculate_form_score(void)
{
    return 0;
}
#endif

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static uint8_t calculate_efficiency_score(float contact_time, float flight_time)
{
    if (contact_time <= 0 || flight_time <= 0) return 50;
    
    float duty_factor = contact_time / (contact_time + flight_time);
    
    // Optimal duty factor is around 0.35 for efficient running
    float optimal_df = 0.35f;
    float df_error = fabsf(duty_factor - optimal_df);
    
    // Convert to score (0-100)
    uint8_t score = (uint8_t)(100 - (df_error * 200));
    if (score > 100) score = 100;
    
    return score;
}
#else
static uint8_t calculate_efficiency_score(float contact_time, float flight_time)
{
    return 0;
}
#endif

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static uint8_t calculate_fatigue_level(void)
{
    if (!session_state.baseline_established) {
        return 0;
    }
    
    // Simple fatigue calculation based on contact time increase
    float contact_increase = (sensor_data.avg_contact_time - session_state.baseline_contact_time) / 
                            session_state.baseline_contact_time;
    
    uint8_t fatigue = (uint8_t)(contact_increase * 100);
    if (fatigue > 100) fatigue = 100;
    
    return fatigue;
}
#else
static uint8_t calculate_fatigue_level(void)
{
    return 0;
}
#endif

// Activity session management functions
void activity_session_init(void)
{
    // Already initialized in module init
}

void activity_session_start(const SessionHeader* header)
{
    if (!header) return;
    
    // Copy header
    memcpy(&session_state.header, header, sizeof(SessionHeader));
    
    // Initialize session state
    session_state.session_active = true;
    session_state.session_start_time = k_uptime_get_32();
    session_state.last_record_time = session_state.session_start_time;
    session_state.last_ble_update_time = session_state.session_start_time;
    
    // Reset counters
    session_state.total_steps_left = 0;
    session_state.total_steps_right = 0;
    session_state.total_distance_cm = 0;
    session_state.num_splits = 0;
    memset(session_state.split_times_sec, 0, sizeof(session_state.split_times_sec));
    session_state.split_distance_cm = 100000; // 1km
    
    // Reset buffers
    memset(sensor_data.contact_time_buffer, 0, sizeof(sensor_data.contact_time_buffer));
    memset(sensor_data.flight_time_buffer, 0, sizeof(sensor_data.flight_time_buffer));
    sensor_data.buffer_index = 0;
    
    // Reset baseline
    session_state.baseline_established = false;
    session_state.baseline_contact_time = 0;
    session_state.baseline_form_score = 0;
    
    // Reset GPS calibration
    session_state.stride_correction_factor = 1.0f;
    session_state.last_gps_update_time = 0;
    
    LOG_INF("Activity session started: id=%u, type=%d, subtype=%d",
            header->session_id, header->activity_type, header->activity_subtype);
}

void activity_session_stop(void)
{
    if (!session_state.session_active) return;
    
    session_state.session_active = false;
    
    // Calculate session summary
    uint32_t duration_ms = k_uptime_get_32() - session_state.session_start_time;
    uint32_t total_steps = session_state.total_steps_left + session_state.total_steps_right;
    uint32_t distance_m = session_state.total_distance_cm / 100;
    
    LOG_INF("Activity session stopped: duration=%us, steps=%u, distance=%um",
            duration_ms/1000, total_steps, distance_m);
    
    // TODO: Send session summary to data module
}

// Calculate pace without GPS (sensor-only)
static uint16_t calculate_pace_no_gps(uint32_t steps, uint32_t time_ms)
{
    if (steps == 0 || time_ms == 0) {
        return 0;
    }
    
    // Estimate stride length based on height (simplified)
    float stride_length_m = 0.7f; // Default 0.7m
    
    // Calculate distance
    float distance_m = steps * stride_length_m;
    
    // Calculate pace in seconds per km
    if (distance_m > 0) {
        float pace_sec_km = (time_ms / 1000.0f) / (distance_m / 1000.0f);
        return (uint16_t)pace_sec_km;
    }
    
    return 0;
}

// Calculate pace with GPS correction
static uint16_t calculate_pace_with_gps(uint32_t gps_distance_m, uint32_t time_ms)
{
    if (gps_distance_m == 0 || time_ms == 0) {
        return 0;
    }
    
    // Direct calculation from GPS distance
    float pace_sec_km = (time_ms / 1000.0f) / (gps_distance_m / 1000.0f);
    return (uint16_t)pace_sec_km;
}

// Main pace calculation that selects appropriate method
static uint16_t calculate_current_pace(void)
{
    uint32_t current_time = k_uptime_get_32();
    uint32_t time_diff = current_time - session_state.session_start_time;
    
    // Check if we have recent GPS data (within last 30 seconds)
    if (session_state.last_gps_update_time > 0 &&
        (current_time - session_state.last_gps_update_time) < 30000) {
        // Use GPS-based pace
        uint32_t gps_distance_m = session_state.total_distance_cm / 100;
        return calculate_pace_with_gps(gps_distance_m, time_diff);
    } else {
        // Fall back to sensor-based pace
        uint32_t total_steps = session_state.total_steps_left + session_state.total_steps_right;
        return calculate_pace_no_gps(total_steps, time_diff);
    }
}


void activity_session_process_gps_update(const GPSUpdateCommand* gps_data)
{
    if (!gps_data || !session_state.session_active) {
        return;
    }
    
    uint32_t now = k_uptime_get_32();
    
    LOG_INF("GPS update: lat=%d, lon=%d, speed=%u cm/s, dist=%u m, acc=%u m",
            gps_data->latitude_e7, gps_data->longitude_e7, 
            gps_data->speed_cms, gps_data->distance_m, gps_data->accuracy_m);
    
    // Check GPS accuracy - ignore if too poor
    if (gps_data->accuracy_m > 20) {
        LOG_WRN("GPS accuracy too poor (%u m), ignoring update", gps_data->accuracy_m);
        return;
    }
    
    // If this is the first GPS update, just store the time
    if (session_state.last_gps_update_time == 0) {
        session_state.last_gps_update_time = now;
        LOG_INF("First GPS update received, initializing GPS tracking");
        return;
    }
    
    // Calculate time since last GPS update
    uint32_t time_diff_ms = now - session_state.last_gps_update_time;
    if (time_diff_ms < 5000) { // Less than 5 seconds
        LOG_WRN("GPS updates too frequent, ignoring");
        return;
    }
    
    // Use GPS distance to calibrate stride length
    if (gps_data->distance_m > 0) {
        // Calculate sensor-based distance since last GPS update
        uint32_t sensor_distance_cm = session_state.total_distance_cm -
                                     session_state.distance_at_last_gps;
        float sensor_distance_m = sensor_distance_cm / 100.0f;
        
        // Calculate correction factor
        if (sensor_distance_m > 10.0f) { // Only calibrate if we've moved at least 10m
            float correction = (float)gps_data->distance_m / sensor_distance_m;
            
            // Limit correction to reasonable bounds (±20%)
            if (correction > 0.8f && correction < 1.2f) {
                // Apply exponential smoothing to correction factor
                session_state.stride_correction_factor =
                    0.9f * session_state.stride_correction_factor + 0.1f * correction;
                
                LOG_INF("Stride calibration: GPS=%um, sensor=%.1fm, correction=%.3f",
                        gps_data->distance_m, (double)sensor_distance_m,
                        (double)session_state.stride_correction_factor);
            } else {
                LOG_WRN("GPS correction out of bounds: %.2f", (double)correction);
            }
        }
        
        // Update total distance with GPS-corrected value
        session_state.total_distance_cm = session_state.distance_at_last_gps +
                                         (gps_data->distance_m * 100);
        
        update_splits();
    }
    
    // Store current values for next update
    session_state.last_gps_update_time = now;
    session_state.distance_at_last_gps = session_state.total_distance_cm;
    
    // Update elevation if provided
    if (gps_data->elevation_change_m != 0) {
        session_state.total_elevation_gain_cm += gps_data->elevation_change_m * 100;
        LOG_WRN("Elevation change: %dm, total gain: %dcm",
                gps_data->elevation_change_m, session_state.total_elevation_gain_cm);
    }

    }

// Calculate distance without GPS (sensor-only)
static uint32_t calculate_distance_no_gps(uint32_t steps)
{
    // Simple stride length estimation based on height
    // This should be calibrated per user
    float stride_length_m = 0.7f; // Default 0.7m
    return (uint32_t)(steps * stride_length_m * 100.0f); // Return in cm
}

// Calculate distance with GPS correction
static uint32_t calculate_distance_with_gps(uint32_t sensor_steps)
{
    // Apply GPS correction factor if available
    float stride_length_m = 0.7f; // Base stride length
    float corrected_stride = stride_length_m * session_state.stride_correction_factor;
    return (uint32_t)(sensor_steps * corrected_stride * 100.0f); // Return in cm
}

// Main distance calculation that selects appropriate method
static uint32_t calculate_total_distance(void)
{
    uint32_t total_steps = session_state.total_steps_left + session_state.total_steps_right;
    
    // Check if GPS correction is available and valid
    if (session_state.stride_correction_factor > 0.8f && 
        session_state.stride_correction_factor < 1.2f &&
        session_state.last_gps_update_time > 0) {
        // Use GPS-corrected calculation
        return calculate_distance_with_gps(total_steps);
    } else {
        // Fall back to sensor-only calculation
        return calculate_distance_no_gps(total_steps);
    }
}

// Elevation gain calculations
static uint32_t calculate_elevation_gain_no_gps() {
    return 0; // Cannot estimate without GPS or altimeter
}

static uint32_t calculate_elevation_gain_with_gps() {
    return (uint32_t)MAX(0, session_state.total_elevation_gain_cm);
}

static uint32_t calculate_total_elevation_gain() {
    if (session_state.last_gps_update_time > 0) {
        return calculate_elevation_gain_with_gps();
    } else {
        return calculate_elevation_gain_no_gps();
    }
}


// Split times update
static void update_splits() {
    if (session_state.split_distance_cm == 0) return;
    
    uint32_t current_split = session_state.total_distance_cm / session_state.split_distance_cm;
    
    uint32_t now = k_uptime_get_32();
    uint32_t elapsed_sec = (now - session_state.session_start_time) / 1000;
    
    while (session_state.num_splits < current_split && session_state.num_splits < 50) {
        // Boundary check and rolling mechanism for num_splits
        // Risk: Overflows after ~4 billion splits (very unlikely, but prevent issues)
        // Strategy: Already capped at 50; enhance with reset if approaching max (though cap prevents it)
        // If cap hit, reset to 0 and archive old splits
        if (session_state.num_splits >= 50) {
            // Archive old splits (e.g., log or send to data module)
            // TODO: Implement split archiving
            LOG_WRN("num_splits cap reached - archiving and resetting");
            session_state.num_splits = 0; // Reset to 0
            memset(session_state.split_times_sec, 0, sizeof(session_state.split_times_sec)); // Clear array
        }
        session_state.num_splits++;
        session_state.split_times_sec[session_state.num_splits - 1] = elapsed_sec;
        LOG_INF("Split %d completed at %u seconds", session_state.num_splits, elapsed_sec);
    }
}


// Weight measurement functions
static void start_weight_measurement(void)
{
    weight_measurement.measurement_in_progress = true;
    weight_measurement.calibration_mode = false; // Ensure this is normal measurement
    weight_measurement.measurement_start_time = k_uptime_get_32();
    weight_measurement.buffer_count = 0;
    weight_measurement.stable_samples_count = 0;
    memset(weight_measurement.pressure_sum_buffer, 0, sizeof(weight_measurement.pressure_sum_buffer));
    
    LOG_INF("Weight measurement started - please stand still on both feet");
    
    // Send notification to BLE that weight measurement is starting
    // TODO: Add BLE notification for weight measurement status
}

static void process_weight_measurement(void)
{
    uint32_t now = k_uptime_get_32();
    
    // Check for timeout (10 seconds)
    if (now - weight_measurement.measurement_start_time > 10000) {
        LOG_WRN("Weight measurement timeout - could not get stable reading");
        weight_measurement.measurement_in_progress = false;
        return;
    }
    
    // Check if person is standing still
    if (!is_person_standing_still()) {
        weight_measurement.stable_samples_count = 0;
        LOG_WRN("Motion detected - resetting stable sample count");
        return;
    }
    
    // Calculate total pressure from both feet
    float total_pressure = 0;
    bool data_valid = true;
    
    // Check if we have recent data from both feet
    if (now - sensor_data.last_left_pressure_update > 200 ||
        now - sensor_data.last_right_pressure_update > 200) {
        data_valid = false;
    }
    
    if (data_valid) {
        // Sum all pressure values from both feet
        for (int i = 0; i < 8; i++) {
        total_pressure += sensor_data.left_pressure[i];
        total_pressure += sensor_data.right_pressure[i];
        }
        
        // Debug output for calibration
        if (weight_measurement.debug_mode) {
        LOG_INF("CALIBRATION DEBUG: Total ADC = %.2f", (double)total_pressure);
        }
        
        // Add to buffer
        if (weight_measurement.buffer_count < 20) {
        // Boundary check for buffer_count (though bounded by 20)
        // Risk: Theoretical overflow if loop malfunctions
        // Strategy: Cap at 20 and log; prevent increment beyond bound
        if (weight_measurement.buffer_count >= 20) {
        LOG_WRN("buffer_count cap reached unexpectedly - resetting buffer");
        weight_measurement.buffer_count = 0; // Reset to 0 as rolling mechanism
        memset(weight_measurement.pressure_sum_buffer, 0, sizeof(weight_measurement.pressure_sum_buffer));
        }
        weight_measurement.pressure_sum_buffer[weight_measurement.buffer_count++] = total_pressure;
        } else {
        // Shift buffer and add new value
        for (int i = 0; i < 19; i++) {
        weight_measurement.pressure_sum_buffer[i] = weight_measurement.pressure_sum_buffer[i + 1];
        }
        weight_measurement.pressure_sum_buffer[19] = total_pressure;
        }
        
        weight_measurement.stable_samples_count++;
        
        // Check if we have enough stable samples
        if (weight_measurement.stable_samples_count >= weight_measurement.stable_samples_required &&
        weight_measurement.buffer_count >= 10) {
        
        // Normal measurement mode: Calculate weight
        float raw_weight_kg = calculate_weight_from_pressure();
        
        if (raw_weight_kg > 20.0f && raw_weight_kg < 300.0f) { // Sanity check
        // Apply calibration to get final weight
        float calibrated_weight_kg = apply_weight_calibration(raw_weight_kg);
        
        sensor_data.calculated_weight_kg = calibrated_weight_kg;
        sensor_data.last_weight_measurement_time = now;
        sensor_data.weight_measurement_valid = true;
        
        LOG_INF("Weight measurement complete: raw=%.1f kg, calibrated=%.1f kg", 
        (double)raw_weight_kg, (double)calibrated_weight_kg);
        
        // Send calibrated weight to bluetooth module
        generic_message_t weight_msg;
        memset(&weight_msg, 0, sizeof(weight_msg));
        weight_msg.sender = SENDER_ACTIVITY_METRICS;
        weight_msg.type = MSG_TYPE_WEIGHT_MEASUREMENT;
        weight_msg.data.weight_measurement.weight_kg = calibrated_weight_kg;
        k_msgq_put(&bluetooth_msgq, &weight_msg, K_NO_WAIT);
        
        weight_measurement.measurement_in_progress = false;
        } else {
        LOG_WRN("Invalid weight calculation: %.1f kg", (double)raw_weight_kg);
        }
        }
        }
        }

// Calibration procedure (foot sensor data is already zero calibrated):
// 1. LINEAR CALIBRATION: Apply known weights and calculate weight_cal.scale_factor
//    weight_cal.scale_factor = (Force_in_Newtons) / ADC_reading
// 2. NONLINEAR CALIBRATION (optional): For FSR sensors, fit polynomial
//    Force = A * ADC² + B * ADC + C
// 3. TEMPERATURE CALIBRATION (optional): Measure drift with temperature

static float calculate_weight_from_pressure(void)
{
    // Average the buffered pressure readings
    float avg_pressure = 0;
    int valid_samples = MIN(weight_measurement.buffer_count, 20);
    
    for (int i = 0; i < valid_samples; i++) {
        avg_pressure += weight_measurement.pressure_sum_buffer[i];
    }
    avg_pressure /= valid_samples;
    
    // Foot sensor data is already zero calibrated, so use directly
    float calibrated_adc = avg_pressure;
    
    // Convert to force using calibration
    float total_force_n = 0;
    
    // Check if using linear or nonlinear calibration
    if (weight_cal.nonlinear_a != 1.0f || weight_cal.nonlinear_b != 0.0f || weight_cal.nonlinear_c != 0.0f) {
        // Use nonlinear calibration (polynomial)
        total_force_n = weight_cal.nonlinear_a * calibrated_adc * calibrated_adc +
                       weight_cal.nonlinear_b * calibrated_adc +
                       weight_cal.nonlinear_c;
    } else {
        // Use simple linear calibration
        total_force_n = calibrated_adc * weight_cal.scale_factor;
    }
    
    // Apply temperature compensation if available
    // Note: This would require a temperature sensor reading
    // float temp_correction = 1.0f + (current_temp - weight_cal.temp_ref) * weight_cal.temp_coeff / 100.0f;
    // total_force_n *= temp_correction;
    
    // Convert force to mass (F = mg, where g = 9.81 m/s²)
    float weight_kg = total_force_n / 9.81f;
    
    // Apply sanity limits
    if (weight_kg < 0.0f) weight_kg = 0.0f;
    
    return weight_kg;
}

// Apply weight calibration to raw weight measurement
static float apply_weight_calibration(float raw_weight_kg)
{
    // The raw_weight_kg is already the final weight calculated from ADC values
    // using the calibration scale factor. No need to apply it again.
    // This function is kept for future enhancements like temperature compensation
    
    float calibrated_weight = raw_weight_kg;
    
    // Apply temperature compensation if available
    // Note: This would require a temperature sensor reading
    // float current_temp = get_temperature(); // TODO: Implement temperature reading
    // float temp_correction = 1.0f + (current_temp - weight_cal.temp_ref) * weight_cal.temp_coeff / 100.0f;
    // calibrated_weight *= temp_correction;
    
    // Apply sanity limits
    if (calibrated_weight < 0.0f) calibrated_weight = 0.0f;
    if (calibrated_weight > 500.0f) calibrated_weight = 500.0f; // Max reasonable weight
    
    return calibrated_weight;
}

static bool is_person_standing_still(void)
{
    // Check linear acceleration magnitude
    float acc_magnitude = sqrtf(sensor_data.lacc_x * sensor_data.lacc_x +
                               sensor_data.lacc_y * sensor_data.lacc_y +
                               sensor_data.lacc_z * sensor_data.lacc_z);
    
    // Check gyroscope magnitude
    float gyro_magnitude = sqrtf(sensor_data.gyro_x * sensor_data.gyro_x +
                                sensor_data.gyro_y * sensor_data.gyro_y +
                                sensor_data.gyro_z * sensor_data.gyro_z);
    
    // Person is standing still if acceleration and rotation are below thresholds
    bool still = (acc_magnitude < weight_measurement.motion_threshold) && 
                 (gyro_magnitude < 0.1f); // 0.1 rad/s threshold for rotation
    
    return still;
}

static void perform_weight_calibration(void)
{
    // Average the buffered pressure readings
    float avg_pressure = 0;
    int valid_samples = MIN(weight_measurement.buffer_count, 20);
    
    for (int i = 0; i < valid_samples; i++) {
        avg_pressure += weight_measurement.pressure_sum_buffer[i];
    }
    avg_pressure /= valid_samples;
    
    LOG_INF("Weight calibration complete!");
    LOG_INF("Average pressure reading: %.2f ADC units", (double)avg_pressure);
    
    // Check if we have a known weight from the mobile app
    float actual_weight_kg;
    if (weight_measurement.known_weight_kg > 0) {
        actual_weight_kg = weight_measurement.known_weight_kg;
        LOG_INF("Using provided weight: %.1f kg", (double)actual_weight_kg);
    } else {
        // Legacy mode - no weight provided, use default or prompt user
        LOG_INF("No weight provided. To complete calibration:");
        LOG_INF("1. Note your actual weight in kg (from a calibrated scale)");
        LOG_INF("2. Calculate scale factor: scale = (weight_kg * 9.81) / %.2f", (double)avg_pressure);
        LOG_INF("3. Send calibration command: CALIBRATE_WEIGHT:SCALE:<calculated_value>");
        
        // For demonstration, use a default weight
        actual_weight_kg = 70.0f;
        LOG_INF("Using default weight for demo: %.1f kg", (double)actual_weight_kg);
    }
    
    float force_n = actual_weight_kg * 9.81f;  // Convert to Newtons
    // Foot sensor data is already zero calibrated
    float calibrated_adc = avg_pressure;
    
    if (calibrated_adc > 0) {
        float new_scale_factor = force_n / calibrated_adc;
        
        LOG_INF("Calibration calculation:");
        LOG_INF("  Weight: %.1f kg", (double)actual_weight_kg);
        LOG_INF("  Force: %.1f N", (double)force_n);
        LOG_INF("  Average ADC: %.2f", (double)calibrated_adc);
        LOG_INF("  New scale factor: %.6f N/ADC", (double)new_scale_factor);
        LOG_INF("  Previous scale factor: %.6f N/ADC", (double)weight_cal.scale_factor);
        
        // Update the scale factor
        weight_cal.scale_factor = new_scale_factor;
        LOG_INF("Weight calibration updated! Test with a weight measurement.");
        
        // Save calibration to storage
        generic_message_t save_msg;
        memset(&save_msg, 0, sizeof(save_msg));
        save_msg.sender = SENDER_ACTIVITY_METRICS;
        save_msg.type = MSG_TYPE_SAVE_WEIGHT_CALIBRATION;
        save_msg.data.weight_calibration.scale_factor = weight_cal.scale_factor;
        save_msg.data.weight_calibration.nonlinear_a = weight_cal.nonlinear_a;
        save_msg.data.weight_calibration.nonlinear_b = weight_cal.nonlinear_b;
        save_msg.data.weight_calibration.nonlinear_c = weight_cal.nonlinear_c;
        save_msg.data.weight_calibration.temp_coeff = weight_cal.temp_coeff;
        save_msg.data.weight_calibration.temp_ref = weight_cal.temp_ref;
        save_msg.data.weight_calibration.is_calibrated = true;
        k_msgq_put(&data_msgq, &save_msg, K_NO_WAIT);
        
        // Send calibration complete notification to BLE
        // TODO: Add notification to mobile app
        
    } else {
        LOG_ERR("Invalid calibration: ADC reading (%.2f) is not positive", 
                (double)avg_pressure);
        LOG_ERR("Please ensure person is standing on both feet during calibration");
    }
}

// Module event handler
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh)) {
        auto *event = cast_module_state_event(aeh);
        if (
            check_state(event, MODULE_ID(foot_sensor), MODULE_STATE_READY)) {
            if (!module_initialized) {
                activity_metrics_init();
            }
        }
        return false;
    }
    
    // Handle activity start/stop events
    if (is_foot_sensor_start_activity_event(aeh)) {
            // Send start command to our thread
            generic_message_t msg = {
                .sender = SENDER_NONE,
                .type = MSG_TYPE_COMMAND,
            };
            strcpy(msg.data.command_str, "START_ACTIVITY");
            k_msgq_put(&activity_metrics_msgq, &msg, K_NO_WAIT);
        return false;
    }
    
    if (is_motion_sensor_stop_activity_event(aeh) ||
        is_foot_sensor_stop_activity_event(aeh)) {

            // Send stop command to our thread
            generic_message_t msg = {
                .sender = SENDER_NONE,
                .type = MSG_TYPE_COMMAND,
            };
            strcpy(msg.data.command_str, "STOP_ACTIVITY");
            k_msgq_put(&activity_metrics_msgq, &msg, K_NO_WAIT);

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
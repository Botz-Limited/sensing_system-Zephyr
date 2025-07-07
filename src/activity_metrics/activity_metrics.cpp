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

// Work items for different message types
static struct k_work process_foot_data_work;
static struct k_work process_bhi360_data_work;
static struct k_work process_command_work;
static struct k_work_delayable periodic_update_work;
static struct k_work weight_measurement_work;
static struct k_work weight_calibration_work;

// Message buffers for different work items
static foot_samples_t pending_foot_data;
static bhi360_log_record_t pending_bhi360_data;
static bhi360_step_count_t pending_step_count;
static char pending_command[MAX_COMMAND_STRING_LEN];
static uint8_t pending_foot_id;  // Which foot (0=left, 1=right)
static msg_type_t pending_bhi360_type;  // Which type of BHI360 message

// Message queue is defined in app.cpp
extern struct k_msgq activity_metrics_msgq;

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
static void activity_metrics_thread_fn(void *arg1, void *arg2, void *arg3);
static void process_foot_sensor_data(const foot_samples_t *data, uint8_t foot);
static void process_bhi360_data(const generic_message_t *msg);
static void calculate_realtime_metrics(void);
static void send_periodic_record(void);
static void send_ble_update(void);
static void process_foot_data_work_handler(struct k_work *work);
static void process_bhi360_data_work_handler(struct k_work *work);
static void process_command_work_handler(struct k_work *work);
static void periodic_update_work_handler(struct k_work *work);
static void weight_measurement_work_handler(struct k_work *work);
static void weight_calibration_work_handler(struct k_work *work);
static float calculate_pace_from_cadence(float cadence);
static int8_t calculate_balance_score(void);
static uint8_t calculate_form_score(void);
static uint8_t calculate_efficiency_score(float contact_time, float flight_time);
static uint8_t calculate_fatigue_level(void);
static void start_weight_measurement(void);
static void process_weight_measurement(void);
static float calculate_weight_from_pressure(void);
static float apply_weight_calibration(float raw_weight_kg);
static bool is_person_standing_still(void);
static void perform_weight_calibration(void);

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
    
    // Initialize work items
    k_work_init(&process_foot_data_work, process_foot_data_work_handler);
    k_work_init(&process_bhi360_data_work, process_bhi360_data_work_handler);
    k_work_init(&process_command_work, process_command_work_handler);
    k_work_init_delayable(&periodic_update_work, periodic_update_work_handler);
    k_work_init(&weight_measurement_work, weight_measurement_work_handler);
    k_work_init(&weight_calibration_work, weight_calibration_work_handler);
    
    // Register step event callback
    step_detection_register_callback([](const StepEvent* event) {
        // Process step events
        if (event->foot == 0) { // Left foot
            session_state.total_steps_left++;
        } else { // Right foot
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
        
        LOG_DBG("Step event: foot=%d, contact=%.1fms, flight=%.1fms, strike=%d",
        event->foot, (double)event->contact_time_ms, (double)event->flight_time_ms, event->strike_pattern);
        });
        
        // Request weight calibration data from data module
        LOG_INF("Activity Metrics: Requesting weight calibration data from data module");
        generic_message_t request_msg = {};
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
                case MSG_TYPE_FOOT_SAMPLES:
                    // Copy foot data and queue foot processing work
                    memcpy(&pending_foot_data, &msg.data.foot_samples, sizeof(foot_samples_t));
                    // Determine which foot based on sender
                    if (msg.sender == SENDER_FOOT_SENSOR_THREAD) {
                        #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                        pending_foot_id = 1; // Right foot on primary
                        #else
                        pending_foot_id = 0; // Left foot on secondary
                        #endif
                    } else if (msg.sender == SENDER_D2D_SECONDARY) {
                        pending_foot_id = 0; // Left foot from secondary
                    }
                    k_work_submit_to_queue(&activity_metrics_work_q, &process_foot_data_work);
                    break;
                    
                case MSG_TYPE_BHI360_LOG_RECORD:
                    // Copy full BHI360 data
                    memcpy(&pending_bhi360_data, &msg.data.bhi360_log_record, sizeof(bhi360_log_record_t));
                    pending_bhi360_type = msg.type;
                    k_work_submit_to_queue(&activity_metrics_work_q, &process_bhi360_data_work);
                    break;
                    
                case MSG_TYPE_BHI360_STEP_COUNT:
                    // Copy step count data
                    memcpy(&pending_step_count, &msg.data.bhi360_step_count, sizeof(bhi360_step_count_t));
                    pending_bhi360_type = msg.type;
                    k_work_submit_to_queue(&activity_metrics_work_q, &process_bhi360_data_work);
                    break;
                    
                case MSG_TYPE_BHI360_3D_MAPPING:
                case MSG_TYPE_BHI360_LINEAR_ACCEL:
                    // For these, we'll use the same BHI360 work handler
                    pending_bhi360_type = msg.type;
                    k_work_submit_to_queue(&activity_metrics_work_q, &process_bhi360_data_work);
                    break;
                    
                case MSG_TYPE_COMMAND:
                    // Copy command and queue command processing work
                    strncpy(pending_command, msg.data.command_str, MAX_COMMAND_STRING_LEN - 1);
                    pending_command[MAX_COMMAND_STRING_LEN - 1] = '\0';
                    k_work_submit_to_queue(&activity_metrics_work_q, &process_command_work);
                    break;

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
                    LOG_DBG("Received unsupported message type %d from %s", 
                            msg.type, get_sender_name(msg.sender));
                    break;
            }
        }
    }
}

// Work handler for processing foot sensor data
static void process_foot_data_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    LOG_DBG("Processing foot sensor data for foot %d", pending_foot_id);
    process_foot_sensor_data(&pending_foot_data, pending_foot_id);
}

// Work handler for processing BHI360 data
static void process_bhi360_data_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    LOG_DBG("Processing BHI360 data, type: %d", pending_bhi360_type);
    
    // Create a temporary message with the appropriate data
    generic_message_t temp_msg;
    temp_msg.type = pending_bhi360_type;
    
    switch (pending_bhi360_type) {
        case MSG_TYPE_BHI360_LOG_RECORD:
            memcpy(&temp_msg.data.bhi360_log_record, &pending_bhi360_data, sizeof(bhi360_log_record_t));
            break;
        case MSG_TYPE_BHI360_STEP_COUNT:
            memcpy(&temp_msg.data.bhi360_step_count, &pending_step_count, sizeof(bhi360_step_count_t));
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
    LOG_DBG("Processing command: %s", pending_command);
    
    if (strcmp(pending_command, "START_ACTIVITY") == 0) {
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
    } else if (strcmp(pending_command, "STOP_ACTIVITY") == 0) {
        activity_session_stop();
        atomic_set(&processing_active, 0);
        // Cancel periodic work
        k_work_cancel_delayable(&periodic_update_work);
        LOG_INF("Activity session stopped");
    } else if (strcmp(pending_command, "MEASURE_WEIGHT") == 0) {
        // Normal weight measurement (not calibration)
        weight_measurement.calibration_mode = false;
        k_work_submit_to_queue(&activity_metrics_work_q, &weight_measurement_work);
        LOG_INF("Weight measurement requested");
    } else if (strncmp(pending_command, "CALIBRATE_WEIGHT:", 17) == 0) {
        // Handle weight calibration command
        // Format: "CALIBRATE_WEIGHT:SCALE:0.0168"
        char param[32];
        float value;
        if (sscanf(pending_command + 17, "%31[^:]:%f", param, &value) == 2) {
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
            LOG_WRN("Invalid calibration command format: %s", pending_command);
        }
    } else if (strcmp(pending_command, "GET_WEIGHT_CAL") == 0) {
        // Report current calibration values
        LOG_INF("Weight calibration values:");
        LOG_INF("  scale_factor: %.6f", (double)weight_cal.scale_factor);
        LOG_INF("  nonlinear_a: %.6f", (double)weight_cal.nonlinear_a);
        LOG_INF("  nonlinear_b: %.6f", (double)weight_cal.nonlinear_b);
        LOG_INF("  nonlinear_c: %.6f", (double)weight_cal.nonlinear_c);
    } else if (strcmp(pending_command, "WEIGHT_DEBUG_ON") == 0) {
        weight_measurement.debug_mode = true;
        LOG_INF("Weight measurement debug mode enabled");
    } else if (strcmp(pending_command, "WEIGHT_DEBUG_OFF") == 0) {
        weight_measurement.debug_mode = false;
        LOG_INF("Weight measurement debug mode disabled");
        } else {
        LOG_WRN("Unknown command: %s", pending_command);
    }
}

// Work handler for periodic updates
static void periodic_update_work_handler(struct k_work *work)
{
    if (atomic_get(&processing_active) == 1) {
        uint32_t now = k_uptime_get_32();
        
        // Calculate real-time metrics
        calculate_realtime_metrics();
        
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
            LOG_DBG("Motion detected - resetting stable sample count");
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
            LOG_INF("Baseline established: contact=%.1fms, form=%d", 
                    (double)session_state.baseline_contact_time, 
                    session_state.baseline_form_score);
        }
    }
}

// Send periodic record to data module
static void send_periodic_record(void)
{
    // For now, just log the metrics
    LOG_DBG("Periodic update: cadence=%.1f, contact=%.1fms, flight=%.1fms",
            (double)sensor_data.current_cadence, (double)sensor_data.avg_contact_time, (double)sensor_data.avg_flight_time);
    
    // TODO: Create proper message structure for activity records
    // This would send to data module for logging
}

// Send BLE update
static void send_ble_update(void)
{
    RealtimeMetricsPacket packet = {0};
    
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
    LOG_DBG("BLE update: pace=%d s/km, cadence=%d, form=%d",
            packet.pace_sec_per_km, packet.cadence_x2/2, packet.form_score);
}

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

// Module event handler
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh)) {
        const struct module_state_event *event = cast_module_state_event(aeh);
        
        // Wait for both motion sensor and foot sensor to be ready
        if (check_state(event, MODULE_ID(motion_sensor), MODULE_STATE_READY) &&
            check_state(event, MODULE_ID(foot_sensor), MODULE_STATE_READY)) {
            if (!module_initialized) {
                activity_metrics_init();
            }
        }
        return false;
    }
    
    // Handle activity start/stop events
    if (is_motion_sensor_start_activity_event(aeh) || 
        is_foot_sensor_start_activity_event(aeh)) {
        if (!session_state.session_active) {
            // Send start command to our thread
            generic_message_t msg = {
                .sender = SENDER_NONE,
                .type = MSG_TYPE_COMMAND,
            };
            strcpy(msg.data.command_str, "START_ACTIVITY");
            k_msgq_put(&activity_metrics_msgq, &msg, K_NO_WAIT);
        }
        return false;
    }
    
    if (is_motion_sensor_stop_activity_event(aeh) ||
        is_foot_sensor_stop_activity_event(aeh)) {
        if (session_state.session_active) {
            // Send stop command to our thread
            generic_message_t msg = {
                .sender = SENDER_NONE,
                .type = MSG_TYPE_COMMAND,
            };
            strcpy(msg.data.command_str, "STOP_ACTIVITY");
            k_msgq_put(&activity_metrics_msgq, &msg, K_NO_WAIT);
        }
        return false;
    }
    
    return false;
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
        LOG_DBG("Motion detected - resetting stable sample count");
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
            generic_message_t weight_msg = {};
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
        generic_message_t save_msg = {};
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


APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_sensor_stop_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_stop_activity_event);
/**
 * @file sensor_data.cpp
 * @brief High-frequency sensor data consolidation module (100Hz)
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#define MODULE sensor_data

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <string.h>

#include <app.hpp>
#include <events/app_state_event.h>
#include <errors.hpp>
#include "sensor_data_ring_buffer.h"
#include "sensor_data_fast_processing.h"
#include "sensor_data_enhanced_algorithms.h"

LOG_MODULE_REGISTER(MODULE, CONFIG_SENSOR_DATA_MODULE_LOG_LEVEL);

// Consolidated sensor data structure
typedef struct {
    // Timing - only delta time, no timestamps!
    uint16_t delta_time_ms;  // Time since last sample (typically 10ms for 100Hz)
    
    // Contact state
    uint8_t left_contact_phase;
    uint8_t right_contact_phase;
    bool left_in_contact;
    bool right_in_contact;
    
    // Contact duration (in ms) - only valid when transitioning out of contact
    uint16_t left_contact_duration_ms;
    uint16_t right_contact_duration_ms;
    
    // Forces and loading
    uint16_t left_peak_force;
    uint16_t right_peak_force;
    uint16_t left_loading_rate;  // N/s
    uint16_t right_loading_rate;  // N/s
    
    // Pressure distribution
    uint8_t left_heel_pct;
    uint8_t left_mid_pct;
    uint8_t left_fore_pct;
    uint8_t right_heel_pct;
    uint8_t right_mid_pct;
    uint8_t right_fore_pct;
    
    // Center of pressure
    int16_t left_cop_x;  // mm, lateral-medial
    int16_t left_cop_y;  // mm, posterior-anterior
    int16_t right_cop_x;
    int16_t right_cop_y;
    
    // IMU data
    float quaternion[4];
    float linear_acc[3];
    float gyro[3];
    
    // Enhanced biomechanics
    int8_t left_pronation_deg;   // Enhanced with pressure validation
    int8_t right_pronation_deg;
    int8_t left_strike_angle;    // Foot angle at contact
    int8_t right_strike_angle;
    uint8_t left_arch_collapse;  // 0-100 index
    uint8_t right_arch_collapse;
    
    // Bilateral timing
    uint16_t true_flight_time_ms;    // Both feet off ground
    uint16_t double_support_time_ms; // Both feet on ground
    
    // Step count
    uint32_t step_count;
    uint8_t step_count_delta;  // Steps since last sample (usually 0-2)
} sensor_data_consolidated_t;

// Thread configuration
static constexpr int sensor_data_stack_size = CONFIG_SENSOR_DATA_MODULE_STACK_SIZE;
static constexpr int sensor_data_priority = CONFIG_SENSOR_DATA_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(sensor_data_stack_area, sensor_data_stack_size);
static struct k_thread sensor_data_thread_data;
static k_tid_t sensor_data_tid;

// Work queue configuration
static constexpr int sensor_data_workq_stack_size = 2048;
K_THREAD_STACK_DEFINE(sensor_data_workq_stack, sensor_data_workq_stack_size);
static struct k_work_q sensor_data_work_q;

// Work items for different message types
static struct k_work process_foot_data_work;
static struct k_work process_imu_data_work;
static struct k_work process_command_work;
static struct k_work_delayable periodic_sample_work;

// Ring buffers for different message types
static foot_data_ring_t foot_ring = {0};
static imu_data_ring_t imu_ring = {0};
static command_ring_t command_ring = {0};

// Statistics - initialize all fields to avoid warnings
static sensor_data_stats_t stats = {
    .foot_data_received = 0,
    .foot_data_dropped = 0,
    .imu_data_received = 0,
    .imu_data_dropped = 0,
    .commands_received = 0,
    .commands_dropped = 0,
    .max_foot_ring_depth = 0,
    .max_imu_ring_depth = 0
};

// Message queues are defined in app.cpp
extern struct k_msgq sensor_data_msgq;  // Input from sensors
extern struct k_msgq sensor_data_queue; // Output to realtime_metrics (legacy name)

// Module state
static bool module_initialized = false;
static atomic_t processing_active = ATOMIC_INIT(0);

// Sensor data state
static struct {
    // Timing - using elapsed times, not timestamps
    uint32_t last_sample_time;  // For delta calculation only
    uint32_t sample_count;
    
    // Contact tracking - store elapsed time in contact
    bool left_foot_contact;
    bool right_foot_contact;
    uint16_t left_contact_elapsed_ms;   // Time spent in current contact
    uint16_t right_contact_elapsed_ms;  // Time spent in current contact
    contact_phase_t left_phase;
    contact_phase_t right_phase;
    
    // Peak force tracking
    uint16_t left_peak_force_current;
    uint16_t right_peak_force_current;
    
    // Latest sensor data
    uint16_t left_pressure[8];
    uint16_t right_pressure[8];
    float latest_quaternion[4];
    float latest_linear_acc[3];
    float latest_gyro[3];
    uint32_t latest_step_count;
    uint32_t last_step_count;
    
    // Strike pattern detection
    uint8_t left_strike_pattern;
    uint8_t right_strike_pattern;
    
    // Pronation angles
    int8_t left_pronation_deg;
    int8_t right_pronation_deg;
    
    // Center of pressure tracking
    int16_t left_cop_x;
    int16_t left_cop_y;
    int16_t right_cop_x;
    int16_t right_cop_y;
    
    // Loading rate tracking
    uint16_t left_previous_force;
    uint16_t right_previous_force;
    uint16_t left_loading_rate;
    uint16_t right_loading_rate;
    
    // Bilateral timing
    uint16_t true_flight_time_ms;
    uint16_t double_support_time_ms;
    
    // Additional metrics
    int8_t left_strike_angle;
    int8_t right_strike_angle;
    uint8_t left_arch_collapse;
    uint8_t right_arch_collapse;
} sensor_state;

// Forward declarations
static void sensor_data_init(void);
static void sensor_data_thread_fn(void *arg1, void *arg2, void *arg3);
static void process_sensor_sample(void);
static void process_foot_data_work_handler(struct k_work *work);
static void process_imu_data_work_handler(struct k_work *work);
static void process_command_work_handler(struct k_work *work);
static void periodic_sample_work_handler(struct k_work *work);

// Initialize the sensor data module
static void sensor_data_init(void)
{
    LOG_INF("Initializing sensor data module");
    
    // Initialize state
    memset(&sensor_state, 0, sizeof(sensor_state));
    
    // Initialize work queue
    k_work_queue_init(&sensor_data_work_q);
    k_work_queue_start(&sensor_data_work_q, sensor_data_workq_stack,
                       K_THREAD_STACK_SIZEOF(sensor_data_workq_stack),
                       sensor_data_priority - 1, NULL);
    k_thread_name_set(&sensor_data_work_q.thread, "sensor_data_wq");
    
    // Initialize work items
    k_work_init(&process_foot_data_work, process_foot_data_work_handler);
    k_work_init(&process_imu_data_work, process_imu_data_work_handler);
    k_work_init(&process_command_work, process_command_work_handler);
    k_work_init_delayable(&periodic_sample_work, periodic_sample_work_handler);
    
    // Create the message processing thread
    sensor_data_tid = k_thread_create(
        &sensor_data_thread_data,
        sensor_data_stack_area,
        K_THREAD_STACK_SIZEOF(sensor_data_stack_area),
        sensor_data_thread_fn,
        NULL, NULL, NULL,
        sensor_data_priority,
        0,
        K_NO_WAIT
    );
    
    k_thread_name_set(sensor_data_tid, "sensor_data");
    
    // Start periodic sampling work at 100Hz
    k_work_schedule_for_queue(&sensor_data_work_q, &periodic_sample_work, K_MSEC(10));
    
    module_initialized = true;
    module_set_state(MODULE_STATE_READY);
    LOG_INF("Sensor data module initialized");
}

// Main processing thread - waits for messages and queues appropriate work
static void sensor_data_thread_fn(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    
    generic_message_t msg;
    
    while (true) {
        // Wait for messages
        int ret = k_msgq_get(&sensor_data_msgq, &msg, K_FOREVER);
        
        if (ret == 0) {
            // Queue different work based on message type
            switch (msg.type) {
                case MSG_TYPE_FOOT_SAMPLES: {
                    // Determine which foot based on sender
                    uint8_t foot_id;
                    if (msg.sender == SENDER_FOOT_SENSOR_THREAD) {
                        #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                        foot_id = 1; // Right foot on primary
                        #else
                        foot_id = 0; // Left foot on secondary
                        #endif
                    } else if (msg.sender == SENDER_D2D_SECONDARY) {
                        foot_id = 0; // Left foot from secondary
                    } else {
                        foot_id = 0; // Default
                    }
                    
                    // Add to ring buffer
                    if (foot_ring_put(&foot_ring, &msg.data.foot_samples, foot_id)) {
                        stats.foot_data_received++;
                        
                        // Update max depth statistic
                        uint32_t depth = atomic_get(&foot_ring.count);
                        if (depth > stats.max_foot_ring_depth) {
                            stats.max_foot_ring_depth = depth;
                        }
                        
                        // Queue work only if not already pending
                        if (!k_work_is_pending(&process_foot_data_work)) {
                            k_work_submit_to_queue(&sensor_data_work_q, &process_foot_data_work);
                        }
                    } else {
                        stats.foot_data_dropped++;
                        LOG_WRN("Foot data ring buffer full, dropping sample");
                    }
                    break;
                }
                    
                case MSG_TYPE_BHI360_LOG_RECORD:
                    // Add to ring buffer
                    if (imu_ring_put(&imu_ring, &msg.data.bhi360_log_record)) {
                        stats.imu_data_received++;
                        
                        // Update max depth statistic
                        uint32_t depth = atomic_get(&imu_ring.count);
                        if (depth > stats.max_imu_ring_depth) {
                            stats.max_imu_ring_depth = depth;
                        }
                        
                        // Queue work only if not already pending
                        if (!k_work_is_pending(&process_imu_data_work)) {
                            k_work_submit_to_queue(&sensor_data_work_q, &process_imu_data_work);
                        }
                    } else {
                        stats.imu_data_dropped++;
                        LOG_WRN("IMU data ring buffer full, dropping sample");
                    }
                    break;
                    
                case MSG_TYPE_BHI360_3D_MAPPING:
                case MSG_TYPE_BHI360_LINEAR_ACCEL:
                    // For now, also use IMU work handler for these
                    // Could create separate handlers if needed
                    k_work_submit_to_queue(&sensor_data_work_q, &process_imu_data_work);
                    break;
                    
                case MSG_TYPE_COMMAND:
                    // Add to ring buffer
                    if (command_ring_put(&command_ring, msg.data.command_str)) {
                        stats.commands_received++;
                        
                        // Queue work only if not already pending
                        if (!k_work_is_pending(&process_command_work)) {
                            k_work_submit_to_queue(&sensor_data_work_q, &process_command_work);
                        }
                    } else {
                        stats.commands_dropped++;
                        LOG_WRN("Command ring buffer full, dropping command");
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
    
    foot_samples_t foot_data;
    uint8_t foot_id;
    
    // Process all available foot data in the ring buffer
    while (foot_ring_get(&foot_ring, &foot_data, &foot_id)) {
        LOG_DBG("Processing foot sensor data for foot %d", foot_id);
        
        if (foot_id == 0) {
            // Left foot processing
            // Copy pressure values
            for (int i = 0; i < 8; i++) {
                sensor_state.left_pressure[i] = foot_data.values[i];
            }
            
            // Detect ground contact
            bool was_in_contact = sensor_state.left_foot_contact;
            sensor_state.left_foot_contact = detect_ground_contact(
                sensor_state.left_pressure, was_in_contact);
            
            // Track contact transitions
            if (!was_in_contact && sensor_state.left_foot_contact) {
                // Just made contact - reset elapsed time
                sensor_state.left_contact_elapsed_ms = 0;
                
                // Detect strike pattern at initial contact
                uint8_t heel_pct, mid_pct, fore_pct;
                calculate_pressure_distribution(sensor_state.left_pressure,
                                              &heel_pct, &mid_pct, &fore_pct);
                sensor_state.left_strike_pattern = detect_strike_pattern(
                    heel_pct, mid_pct, fore_pct);
                
                // Calculate foot strike angle using IMU + pressure
                uint16_t heel_force = sensor_state.left_pressure[0] + sensor_state.left_pressure[1];
                uint16_t fore_force = sensor_state.left_pressure[5] + sensor_state.left_pressure[6] + 
                                     sensor_state.left_pressure[7];
                sensor_state.left_strike_angle = calculate_foot_strike_angle(
                    sensor_state.latest_quaternion, heel_force, fore_force);
            } else if (was_in_contact && !sensor_state.left_foot_contact) {
                // Just lost contact - elapsed time will be captured in next consolidation
            }
            
            // Update contact phase
            sensor_state.left_phase = detect_contact_phase(
                sensor_state.left_pressure, was_in_contact);
            
            // Track peak force during contact
            if (sensor_state.left_foot_contact) {
                uint16_t current_force = detect_peak_force(sensor_state.left_pressure);
                if (current_force > sensor_state.left_peak_force_current) {
                    sensor_state.left_peak_force_current = current_force;
                }
                
                // Calculate center of pressure
                calculate_center_of_pressure(sensor_state.left_pressure,
                                           &sensor_state.left_cop_x,
                                           &sensor_state.left_cop_y);
                
                // Calculate loading rate (only during loading phase)
                if (sensor_state.left_phase == PHASE_LOADING) {
                    sensor_state.left_loading_rate = calculate_loading_rate(
                        current_force,
                        sensor_state.left_previous_force,
                        10  // Assuming 10ms between samples at 100Hz
                    );
                }
                sensor_state.left_previous_force = current_force;
                
                // Detect arch collapse
                sensor_state.left_arch_collapse = detect_arch_collapse(
                    sensor_state.left_pressure,
                    sensor_state.left_phase
                );
            } else {
                // Reset metrics when not in contact
                sensor_state.left_peak_force_current = 0;
                sensor_state.left_previous_force = 0;
                sensor_state.left_loading_rate = 0;
            }
            
        } else {
            // Right foot processing (same logic)
            // Copy pressure values
            for (int i = 0; i < 8; i++) {
                sensor_state.right_pressure[i] = foot_data.values[i];
            }
            
            // Detect ground contact
            bool was_in_contact = sensor_state.right_foot_contact;
            sensor_state.right_foot_contact = detect_ground_contact(
                sensor_state.right_pressure, was_in_contact);
            
            // Track contact transitions
            if (!was_in_contact && sensor_state.right_foot_contact) {
                // Just made contact - reset elapsed time
                sensor_state.right_contact_elapsed_ms = 0;
                
                // Detect strike pattern at initial contact
                uint8_t heel_pct, mid_pct, fore_pct;
                calculate_pressure_distribution(sensor_state.right_pressure,
                                              &heel_pct, &mid_pct, &fore_pct);
                sensor_state.right_strike_pattern = detect_strike_pattern(
                    heel_pct, mid_pct, fore_pct);
                
                // Calculate foot strike angle using IMU + pressure
                uint16_t heel_force = sensor_state.right_pressure[0] + sensor_state.right_pressure[1];
                uint16_t fore_force = sensor_state.right_pressure[5] + sensor_state.right_pressure[6] + 
                                     sensor_state.right_pressure[7];
                sensor_state.right_strike_angle = calculate_foot_strike_angle(
                    sensor_state.latest_quaternion, heel_force, fore_force);
            } else if (was_in_contact && !sensor_state.right_foot_contact) {
                // Just lost contact - elapsed time will be captured in next consolidation
            }
            
            // Update contact phase
            sensor_state.right_phase = detect_contact_phase(
                sensor_state.right_pressure, was_in_contact);
            
            // Track peak force during contact
            if (sensor_state.right_foot_contact) {
                uint16_t current_force = detect_peak_force(sensor_state.right_pressure);
                if (current_force > sensor_state.right_peak_force_current) {
                    sensor_state.right_peak_force_current = current_force;
                }
                
                // Calculate center of pressure
                calculate_center_of_pressure(sensor_state.right_pressure,
                                           &sensor_state.right_cop_x,
                                           &sensor_state.right_cop_y);
                
                // Calculate loading rate (only during loading phase)
                if (sensor_state.right_phase == PHASE_LOADING) {
                    sensor_state.right_loading_rate = calculate_loading_rate(
                        current_force,
                        sensor_state.right_previous_force,
                        10  // Assuming 10ms between samples at 100Hz
                    );
                }
                sensor_state.right_previous_force = current_force;
                
                // Detect arch collapse
                sensor_state.right_arch_collapse = detect_arch_collapse(
                    sensor_state.right_pressure,
                    sensor_state.right_phase
                );
            } else {
                // Reset metrics when not in contact
                sensor_state.right_peak_force_current = 0;
                sensor_state.right_previous_force = 0;
                sensor_state.right_loading_rate = 0;
            }
        }
    }
}

// Work handler for processing IMU data
static void process_imu_data_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    bhi360_log_record_t imu_data;
    
    // Process all available IMU data in the ring buffer
    while (imu_ring_get(&imu_ring, &imu_data)) {
        LOG_DBG("Processing IMU data");
        
        // Store quaternion values
        sensor_state.latest_quaternion[0] = imu_data.quat_w;
        sensor_state.latest_quaternion[1] = imu_data.quat_x;
        sensor_state.latest_quaternion[2] = imu_data.quat_y;
        sensor_state.latest_quaternion[3] = imu_data.quat_z;
        
        // Store linear acceleration
        sensor_state.latest_linear_acc[0] = imu_data.lacc_x;
        sensor_state.latest_linear_acc[1] = imu_data.lacc_y;
        sensor_state.latest_linear_acc[2] = imu_data.lacc_z;
        
        // Store gyroscope data
        sensor_state.latest_gyro[0] = imu_data.gyro_x;
        sensor_state.latest_gyro[1] = imu_data.gyro_y;
        sensor_state.latest_gyro[2] = imu_data.gyro_z;
        
        // Update step count
        sensor_state.latest_step_count = imu_data.step_count;
        
        // Calculate enhanced pronation using IMU + pressure data
        // This provides more accurate pronation assessment
        sensor_state.left_pronation_deg = calculate_pronation_enhanced(
            sensor_state.latest_quaternion,
            sensor_state.left_pressure,
            sensor_state.left_foot_contact
        );
        
        sensor_state.right_pronation_deg = calculate_pronation_enhanced(
            sensor_state.latest_quaternion,
            sensor_state.right_pressure,
            sensor_state.right_foot_contact
        );
    }
}

// Work handler for processing commands
static void process_command_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    char command[MAX_COMMAND_STRING_LEN];
    
    // Process all available commands in the ring buffer
    while (command_ring_get(&command_ring, command)) {
        LOG_DBG("Processing command: %s", command);
        
        if (strcmp(command, "START_SENSOR_PROCESSING") == 0) {
            atomic_set(&processing_active, 1);
            LOG_INF("Sensor processing started");
        } else if (strcmp(command, "STOP_SENSOR_PROCESSING") == 0) {
            atomic_set(&processing_active, 0);
            LOG_INF("Sensor processing stopped");
        } else if (strcmp(command, "SENSOR_STATS") == 0) {
            LOG_INF("Sensor Data Statistics:");
            LOG_INF("  Foot data: %u received, %u dropped (max depth: %u)",
                    stats.foot_data_received, stats.foot_data_dropped, 
                    stats.max_foot_ring_depth);
            LOG_INF("  IMU data: %u received, %u dropped (max depth: %u)",
                    stats.imu_data_received, stats.imu_data_dropped,
                    stats.max_imu_ring_depth);
            LOG_INF("  Commands: %u received, %u dropped",
                    stats.commands_received, stats.commands_dropped);
        } else {
            LOG_WRN("Unknown command: %s", command);
        }
    }
}

// Work handler for periodic 100Hz sampling
static void periodic_sample_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (atomic_get(&processing_active) == 1) {
        process_sensor_sample();
        sensor_state.sample_count++;
    }
    
    // Reschedule for next sample (10ms = 100Hz)
    k_work_schedule_for_queue(&sensor_data_work_q, &periodic_sample_work, K_MSEC(10));
}

// Process sensor data at 100Hz
static void process_sensor_sample(void)
{
    uint32_t current_time = k_uptime_get_32();
    
    // Calculate delta time since last sample
    uint16_t delta_time_ms = 10;  // Default to 10ms (100Hz)
    if (sensor_state.last_sample_time > 0) {
        delta_time_ms = current_time - sensor_state.last_sample_time;
        // Clamp to reasonable values (5-50ms)
        if (delta_time_ms < 5) delta_time_ms = 5;
        if (delta_time_ms > 50) delta_time_ms = 50;
    }
    sensor_state.last_sample_time = current_time;
    
    // Update contact elapsed times
    if (sensor_state.left_foot_contact) {
        // Check for overflow before adding
        if ((UINT16_MAX - sensor_state.left_contact_elapsed_ms) > delta_time_ms) {
            sensor_state.left_contact_elapsed_ms += delta_time_ms;
        } else {
            // Saturate at max uint16_t (65.535 seconds)
            sensor_state.left_contact_elapsed_ms = UINT16_MAX;
        }
    }
    
    if (sensor_state.right_foot_contact) {
        // Check for overflow before adding
        if ((UINT16_MAX - sensor_state.right_contact_elapsed_ms) > delta_time_ms) {
            sensor_state.right_contact_elapsed_ms += delta_time_ms;
        } else {
            // Saturate at max uint16_t (65.535 seconds)
            sensor_state.right_contact_elapsed_ms = UINT16_MAX;
        }
    }
    
    // Update bilateral timing metrics
    sensor_state.true_flight_time_ms = update_true_flight_time(
        sensor_state.left_foot_contact,
        sensor_state.right_foot_contact,
        delta_time_ms,
        sensor_state.true_flight_time_ms
    );
    
    sensor_state.double_support_time_ms = update_double_support_time(
        sensor_state.left_foot_contact,
        sensor_state.right_foot_contact,
        delta_time_ms,
        sensor_state.double_support_time_ms
    );
    
    // Create consolidated data structure
    sensor_data_consolidated_t consolidated = {
        .delta_time_ms = delta_time_ms,
        
        // Contact state
        .left_contact_phase = (uint8_t)sensor_state.left_phase,
        .right_contact_phase = (uint8_t)sensor_state.right_phase,
        .left_in_contact = sensor_state.left_foot_contact,
        .right_in_contact = sensor_state.right_foot_contact,
        
        // Contact duration - only valid when transitioning out of contact
        .left_contact_duration_ms = sensor_state.left_foot_contact ? 0 : sensor_state.left_contact_elapsed_ms,
        .right_contact_duration_ms = sensor_state.right_foot_contact ? 0 : sensor_state.right_contact_elapsed_ms,
        
        // Forces and loading
        .left_peak_force = sensor_state.left_peak_force_current,
        .right_peak_force = sensor_state.right_peak_force_current,
        .left_loading_rate = sensor_state.left_loading_rate,
        .right_loading_rate = sensor_state.right_loading_rate,
        
        // Center of pressure
        .left_cop_x = sensor_state.left_cop_x,
        .left_cop_y = sensor_state.left_cop_y,
        .right_cop_x = sensor_state.right_cop_x,
        .right_cop_y = sensor_state.right_cop_y,
        
        // Enhanced biomechanics
        .left_pronation_deg = sensor_state.left_pronation_deg,
        .right_pronation_deg = sensor_state.right_pronation_deg,
        .left_strike_angle = sensor_state.left_strike_angle,
        .right_strike_angle = sensor_state.right_strike_angle,
        .left_arch_collapse = sensor_state.left_arch_collapse,
        .right_arch_collapse = sensor_state.right_arch_collapse,
        
        // Bilateral timing
        .true_flight_time_ms = sensor_state.true_flight_time_ms,
        .double_support_time_ms = sensor_state.double_support_time_ms,
        
        // Step count
        .step_count = sensor_state.latest_step_count,
        .step_count_delta = (uint8_t)(sensor_state.latest_step_count - sensor_state.last_step_count)
    };
    
    // Update last step count
    sensor_state.last_step_count = sensor_state.latest_step_count;
    
    // Calculate pressure distribution for left foot
    calculate_pressure_distribution(sensor_state.left_pressure,
                                  &consolidated.left_heel_pct,
                                  &consolidated.left_mid_pct,
                                  &consolidated.left_fore_pct);
    
    // Calculate pressure distribution for right foot
    calculate_pressure_distribution(sensor_state.right_pressure,
                                  &consolidated.right_heel_pct,
                                  &consolidated.right_mid_pct,
                                  &consolidated.right_fore_pct);
    
    // Copy IMU data
    memcpy(consolidated.quaternion, sensor_state.latest_quaternion, sizeof(consolidated.quaternion));
    memcpy(consolidated.linear_acc, sensor_state.latest_linear_acc, sizeof(consolidated.linear_acc));
    memcpy(consolidated.gyro, sensor_state.latest_gyro, sizeof(consolidated.gyro));
    
    // Log periodically
    if (sensor_state.sample_count % 100 == 0) {
        LOG_DBG("Processed %u samples - L:%s/%ums R:%s/%ums", 
                sensor_state.sample_count,
                sensor_state.left_foot_contact ? "Contact" : "Swing",
                sensor_state.left_contact_elapsed_ms,
                sensor_state.right_foot_contact ? "Contact" : "Swing",
                sensor_state.right_contact_elapsed_ms);
    }
    
    // Send consolidated data to other threads
    generic_message_t out_msg = {};
    out_msg.sender = SENDER_SENSOR_DATA;
    out_msg.type = MSG_TYPE_SENSOR_DATA_CONSOLIDATED;
    
    // Copy consolidated data to message
    // Note: We need to define how to pass this data in the generic_message_t structure
    // For now, we'll just send the notification that data is ready
    
    // Send to realtime metrics, analytics, and session threads
    k_msgq_put(&sensor_data_queue, &out_msg, K_NO_WAIT);
    
    // Also send to realtime_metrics module if enabled
    #if IS_ENABLED(CONFIG_REALTIME_METRICS_MODULE)
    extern struct k_msgq realtime_queue;
    k_msgq_put(&realtime_queue, &out_msg, K_NO_WAIT);
    #endif
}

// Module event handler
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh)) {
        const struct module_state_event *event = cast_module_state_event(aeh);
        
        // Wait for motion sensor and foot sensor to be ready
        if (check_state(event, MODULE_ID(motion_sensor), MODULE_STATE_READY) &&
            check_state(event, MODULE_ID(foot_sensor), MODULE_STATE_READY)) {
            if (!module_initialized) {
                sensor_data_init();
            }
        }
        return false;
    }
    
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
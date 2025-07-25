/**
 * @file sensor_data.cpp
 * @brief High-frequency sensor data consolidation module (100Hz)
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#define MODULE sensor_data

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "sensor_data_enhanced_algorithms.hpp"
#include "sensor_data_fast_processing.hpp"
#include "sensor_data_ring_buffer.hpp"
#include "sensor_data_sync.hpp"
#include "sensor_data_sync_enhanced.h"
#include <app.hpp>
#include <errors.hpp>
#include <events/app_state_event.h>

// External queues from app.cpp
extern struct k_msgq analytics_queue;
extern struct k_msgq realtime_queue;

LOG_MODULE_REGISTER(MODULE, CONFIG_SENSOR_DATA_MODULE_LOG_LEVEL);

// Include the consolidated data structure
#include "sensor_data_consolidated.hpp"

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

// Ring buffers for different message types - properly initialized
static foot_data_ring_t foot_ring = {
    .data = {{0}}, .foot_id = {0}, .write_idx = 0, .read_idx = 0, .count = ATOMIC_INIT(0)};
static imu_data_ring_t imu_ring = {.data = {{0}}, .write_idx = 0, .read_idx = 0, .count = ATOMIC_INIT(0)};
static command_ring_t command_ring = {.data = {{0}}, .write_idx = 0, .read_idx = 0, .count = ATOMIC_INIT(0)};

// Statistics - initialize all fields to avoid warnings
static sensor_data_stats_t stats = {.foot_data_received = 0,
                                    .foot_data_dropped = 0,
                                    .imu_data_received = 0,
                                    .imu_data_dropped = 0,
                                    .commands_received = 0,
                                    .commands_dropped = 0,
                                    .max_foot_ring_depth = 0,
                                    .max_imu_ring_depth = 0};

// Message queues are defined in app.cpp
extern struct k_msgq sensor_data_msgq;  // Input from sensors
extern struct k_msgq sensor_data_queue; // Output to realtime_metrics (legacy name)

// Module state
static bool module_initialized = false;
static atomic_t processing_active = ATOMIC_INIT(0);

// Sensor data state
static struct
{
    // Timing - using elapsed times, not timestamps
    uint32_t last_sample_time; // For delta calculation only
    uint32_t sample_count;

    // Contact tracking - store elapsed time in contact
    bool left_foot_contact;
    bool right_foot_contact;
    uint16_t left_contact_elapsed_ms;  // Time spent in current contact
    uint16_t right_contact_elapsed_ms; // Time spent in current contact
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

      // D2D data freshness tracking
    uint32_t left_foot_last_update_time;  // When D2D data last updated left foot
    uint32_t left_imu_last_update_time;   // When D2D IMU data last updated
} sensor_state;

// Forward declarations
static void sensor_data_init(void);
static void sensor_data_thread_fn(void *arg1, void *arg2, void *arg3);
static void process_sensor_sample(void);
static void process_foot_data_work_handler(struct k_work *work);
static void process_imu_data_work_handler(struct k_work *work);
static void process_command_work_handler(struct k_work *work);
static void periodic_sample_work_handler(struct k_work *work);
static void process_full_synchronized_data(const full_synchronized_data_t *full_sync);
static void process_synchronized_foot_data(const synchronized_foot_data_t *sync_data);
static void update_foot_state_from_d2d(const foot_samples_t *foot_data, uint8_t foot_id);
static void update_imu_state_from_d2d(const bhi360_log_record_t *imu_data);

// Initialize the sensor data module
static void sensor_data_init(void)
{
    LOG_INF("Initializing sensor data module");

    // Initialize enhanced synchronization on primary
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    sensor_data_sync_enhanced_init();
#endif

    // Initialize state
    memset(&sensor_state, 0, sizeof(sensor_state));

    // Initialize work queue
    k_work_queue_init(&sensor_data_work_q);
    k_work_queue_start(&sensor_data_work_q, sensor_data_workq_stack, K_THREAD_STACK_SIZEOF(sensor_data_workq_stack),
                       sensor_data_priority - 1, NULL);
    k_thread_name_set(&sensor_data_work_q.thread, "sensor_data_wq");

    // Initialize work items
    k_work_init(&process_foot_data_work, process_foot_data_work_handler);
    k_work_init(&process_imu_data_work, process_imu_data_work_handler);
    k_work_init(&process_command_work, process_command_work_handler);
    k_work_init_delayable(&periodic_sample_work, periodic_sample_work_handler);

    // Create the message processing thread
    sensor_data_tid =
        k_thread_create(&sensor_data_thread_data, sensor_data_stack_area, K_THREAD_STACK_SIZEOF(sensor_data_stack_area),
                        sensor_data_thread_fn, NULL, NULL, NULL, sensor_data_priority, 0, K_NO_WAIT);

    k_thread_name_set(sensor_data_tid, "sensor_data");

    // Start periodic sampling work at 80Hz
    k_work_schedule_for_queue(&sensor_data_work_q, &periodic_sample_work, K_MSEC(12));

    // Initialize synchronization
    sensor_data_sync_init();

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

    while (true)
    {
        // Wait for messages
        int ret = k_msgq_get(&sensor_data_msgq, &msg, K_FOREVER);

        if (ret == 0)
        {
            // Queue different work based on message type
            switch (msg.type)
            {

                case MSG_TYPE_FOOT_SAMPLES: {
                    const foot_samples_t *foot_data = &msg.data.foot_samples;

                    // Determine foot ID and source
                    uint8_t foot_id;
                    bool is_d2d_data = false;

                    if (msg.sender == SENDER_FOOT_SENSOR_THREAD)
                    {
// Local foot sensor data
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                        foot_id = 1; // Primary = right foot
#else
                        foot_id = 0; // Secondary = left foot
#endif
                        is_d2d_data = false;
                    }
                    else if (msg.sender == SENDER_BTH || msg.sender == SENDER_D2D_SECONDARY)
                    {
                        // D2D data from secondary (always left foot)
                        foot_id = 0;
                        is_d2d_data = true;
                    }
                    else
                    {
                        LOG_WRN("Unknown foot data sender: %s", get_sender_name(msg.sender));
                        break;
                    }

                    // Process based on whether it's local or D2D data
                    if (!is_d2d_data)
                    {
                        // Local data - add to ring buffer for work handler processing
                        if (foot_ring_put(&foot_ring, foot_data, foot_id))
                        {
                            stats.foot_data_received++;

                            // Update max depth statistic
                            uint32_t depth = atomic_get(&foot_ring.count);
                            if (depth > stats.max_foot_ring_depth)
                            {
                                stats.max_foot_ring_depth = depth;
                            }

                            // Queue work only if not already pending
                            if (!k_work_is_pending(&process_foot_data_work))
                            {
                                k_work_submit_to_queue(&sensor_data_work_q, &process_foot_data_work);
                            }
                        }
                        else
                        {
                            stats.foot_data_dropped++;
                            LOG_WRN("Foot data ring buffer full, dropping sample");
                            // Implement basic backpressure: slow down sensor sampling if buffer is full
                            if (stats.foot_data_dropped > 10) {
                                LOG_ERR("Excessive foot data drops, increasing thread priority");
                                // Dynamically increase thread priority to handle backlog
                                int current_priority = k_thread_priority_get(k_current_get());
                                if (current_priority > 2) { // Assuming lower number is higher priority
                                    k_thread_priority_set(k_current_get(), current_priority - 1);
                                    LOG_INF("Increased sensor data thread priority to %d", current_priority - 1);
                                }
                            }
                        }
                    }
                    else
                    {
// D2D data - update sensor state directly (primary device only)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                        update_foot_state_from_d2d(foot_data, foot_id);
                        LOG_DBG("Updated left foot state from D2D data");
#endif
                    }

// Process synchronization (only on primary device)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    // Call sync functions with appropriate is_secondary flag
                    bool is_secondary = is_d2d_data;
                    sensor_data_sync_process(foot_data, is_secondary);
                    sensor_data_sync_enhanced_process_foot(foot_data, is_secondary);
#endif

                    break;
                }

                case MSG_TYPE_BHI360_LOG_RECORD: {
                    const bhi360_log_record_t *imu_data = &msg.data.bhi360_log_record;

                    // Determine if this is D2D data
                    bool is_d2d_data = (msg.sender == SENDER_BTH || msg.sender == SENDER_D2D_SECONDARY);

                    // Process D2D IMU data if from secondary
                    if (is_d2d_data) {
                        #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                        update_imu_state_from_d2d(imu_data);
                        #endif
                    }

                    // Add to ring buffer only for local data (not D2D)
                    if (msg.sender == SENDER_MOTION_SENSOR)
                    {
                        if (imu_ring_put(&imu_ring, imu_data))
                        {
                            stats.imu_data_received++;

                            // Update max depth statistic
                            uint32_t depth = atomic_get(&imu_ring.count);
                            if (depth > stats.max_imu_ring_depth)
                            {
                                stats.max_imu_ring_depth = depth;
                            }

                            // Queue work only if not already pending
                            if (!k_work_is_pending(&process_imu_data_work))
                            {
                                k_work_submit_to_queue(&sensor_data_work_q, &process_imu_data_work);
                            }
                        }
                        else
                        {
                            stats.imu_data_dropped++;
                            LOG_WRN("IMU data ring buffer full, dropping sample");
                            // Implement basic backpressure: slow down sensor sampling if buffer is full
                            if (stats.imu_data_dropped > 10) {
                                LOG_ERR("Excessive IMU data drops, increasing thread priority");
                                // Dynamically increase thread priority to handle backlog
                                int current_priority = k_thread_priority_get(k_current_get());
                                if (current_priority > 2) { // Assuming lower number is higher priority
                                    k_thread_priority_set(k_current_get(), current_priority - 1);
                                    LOG_INF("Increased sensor data thread priority to %d", current_priority - 1);
                                }
                            }
                        }
                    }

// Process for synchronization on primary only
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    bool is_secondary_imu = is_d2d_data;
                    sensor_data_sync_enhanced_process_imu(imu_data, is_secondary_imu);
#endif

                    break;
                }

                case MSG_TYPE_BHI360_3D_MAPPING:
                case MSG_TYPE_BHI360_LINEAR_ACCEL:
                    // For now, also use IMU work handler for these
                    // Could create separate handlers if needed
                    k_work_submit_to_queue(&sensor_data_work_q, &process_imu_data_work);
                    break;

                case MSG_TYPE_COMMAND:
                    // Add to ring buffer
                    if (command_ring_put(&command_ring, msg.data.command_str))
                    {
                        stats.commands_received++;

                        // Queue work only if not already pending
                        if (!k_work_is_pending(&process_command_work))
                        {
                            k_work_submit_to_queue(&sensor_data_work_q, &process_command_work);
                        }
                    }
                    else
                    {
                        stats.commands_dropped++;
                        LOG_WRN("Command ring buffer full, dropping command");
                    }
                    break;

                default:
                    LOG_DBG("Received unsupported message type %d from %s", msg.type, get_sender_name(msg.sender));
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
    while (foot_ring_get(&foot_ring, &foot_data, &foot_id))
    {
        LOG_DBG("Processing foot sensor data for foot %d", foot_id);

        if (foot_id == 0)
        {
            // Left foot processing
            // Copy pressure values
            for (int i = 0; i < 8; i++)
            {
                sensor_state.left_pressure[i] = foot_data.values[i];
            }

            // Detect ground contact
            bool was_in_contact = sensor_state.left_foot_contact;
            sensor_state.left_foot_contact = detect_ground_contact(sensor_state.left_pressure, was_in_contact);

            // Track contact transitions
            if (!was_in_contact && sensor_state.left_foot_contact)
            {
                // Just made contact - reset elapsed time
                sensor_state.left_contact_elapsed_ms = 0;

                // Detect strike pattern at initial contact
                uint8_t heel_pct, mid_pct, fore_pct;
                calculate_pressure_distribution(sensor_state.left_pressure, &heel_pct, &mid_pct, &fore_pct);
                sensor_state.left_strike_pattern = detect_strike_pattern(heel_pct, mid_pct, fore_pct);

                // Calculate foot strike angle using IMU + pressure
                uint16_t heel_force = sensor_state.left_pressure[0] + sensor_state.left_pressure[1];
                uint16_t fore_force =
                    sensor_state.left_pressure[5] + sensor_state.left_pressure[6] + sensor_state.left_pressure[7];
                sensor_state.left_strike_angle =
                    calculate_foot_strike_angle(sensor_state.latest_quaternion, heel_force, fore_force);
            }
            else if (was_in_contact && !sensor_state.left_foot_contact)
            {
                // Just lost contact - elapsed time will be captured in next consolidation
            }

            // Update contact phase
            sensor_state.left_phase = detect_contact_phase(sensor_state.left_pressure, was_in_contact);

            // Track peak force during contact
            if (sensor_state.left_foot_contact)
            {
                uint16_t current_force = detect_peak_force(sensor_state.left_pressure);
                if (current_force > sensor_state.left_peak_force_current)
                {
                    sensor_state.left_peak_force_current = current_force;
                }

                // Calculate center of pressure
                calculate_center_of_pressure(sensor_state.left_pressure, &sensor_state.left_cop_x,
                                             &sensor_state.left_cop_y);

                // Calculate loading rate (only during loading phase)
                if (sensor_state.left_phase == PHASE_LOADING)
                {
                    sensor_state.left_loading_rate =
                        calculate_loading_rate(current_force, sensor_state.left_previous_force,
                                               12 // Adjusted to 12.5ms between samples at 80Hz, approximated as 12ms
                    );
                }
                sensor_state.left_previous_force = current_force;

                // Detect arch collapse
                sensor_state.left_arch_collapse =
                    detect_arch_collapse(sensor_state.left_pressure, sensor_state.left_phase);
            }
            else
            {
                // Reset metrics when not in contact
                sensor_state.left_peak_force_current = 0;
                sensor_state.left_previous_force = 0;
                sensor_state.left_loading_rate = 0;
            }
        }
        else
        {
            // Right foot processing (same logic)
            // Copy pressure values
            for (int i = 0; i < 8; i++)
            {
                sensor_state.right_pressure[i] = foot_data.values[i];
            }

            // Detect ground contact
            bool was_in_contact = sensor_state.right_foot_contact;
            sensor_state.right_foot_contact = detect_ground_contact(sensor_state.right_pressure, was_in_contact);

            // Track contact transitions
            if (!was_in_contact && sensor_state.right_foot_contact)
            {
                // Just made contact - reset elapsed time
                sensor_state.right_contact_elapsed_ms = 0;

                // Detect strike pattern at initial contact
                uint8_t heel_pct, mid_pct, fore_pct;
                calculate_pressure_distribution(sensor_state.right_pressure, &heel_pct, &mid_pct, &fore_pct);
                sensor_state.right_strike_pattern = detect_strike_pattern(heel_pct, mid_pct, fore_pct);

                // Calculate foot strike angle using IMU + pressure
                uint16_t heel_force = sensor_state.right_pressure[0] + sensor_state.right_pressure[1];
                uint16_t fore_force =
                    sensor_state.right_pressure[5] + sensor_state.right_pressure[6] + sensor_state.right_pressure[7];
                sensor_state.right_strike_angle =
                    calculate_foot_strike_angle(sensor_state.latest_quaternion, heel_force, fore_force);
            }
            else if (was_in_contact && !sensor_state.right_foot_contact)
            {
                // Just lost contact - elapsed time will be captured in next consolidation
            }

            // Update contact phase
            sensor_state.right_phase = detect_contact_phase(sensor_state.right_pressure, was_in_contact);

            // Track peak force during contact
            if (sensor_state.right_foot_contact)
            {
                uint16_t current_force = detect_peak_force(sensor_state.right_pressure);
                if (current_force > sensor_state.right_peak_force_current)
                {
                    sensor_state.right_peak_force_current = current_force;
                }

                // Calculate center of pressure
                calculate_center_of_pressure(sensor_state.right_pressure, &sensor_state.right_cop_x,
                                             &sensor_state.right_cop_y);

                // Calculate loading rate (only during loading phase)
                if (sensor_state.right_phase == PHASE_LOADING)
                {
                    sensor_state.right_loading_rate =
                        calculate_loading_rate(current_force, sensor_state.right_previous_force,
                                               12 // Adjusted to 12.5ms between samples at 80Hz, approximated as 12ms
                        );
                }
                sensor_state.right_previous_force = current_force;

                // Detect arch collapse
                sensor_state.right_arch_collapse =
                    detect_arch_collapse(sensor_state.right_pressure, sensor_state.right_phase);
            }
            else
            {
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
    while (imu_ring_get(&imu_ring, &imu_data))
    {
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
            sensor_state.latest_quaternion, sensor_state.left_pressure, sensor_state.left_foot_contact);

        sensor_state.right_pronation_deg = calculate_pronation_enhanced(
            sensor_state.latest_quaternion, sensor_state.right_pressure, sensor_state.right_foot_contact);
    }
}

// Work handler for processing commands
static void process_command_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    char command[MAX_COMMAND_STRING_LEN];

    // Process all available commands in the ring buffer
    while (command_ring_get(&command_ring, command))
    {
        LOG_DBG("Processing command: %s", command);

        if (strcmp(command, "START_SENSOR_PROCESSING") == 0)
        {
            atomic_set(&processing_active, 1);
            LOG_INF("Sensor processing started");
        }
        else if (strcmp(command, "STOP_SENSOR_PROCESSING") == 0)
        {
            atomic_set(&processing_active, 0);
            LOG_INF("Sensor processing stopped");
        }
        else if (strcmp(command, "SENSOR_STATS") == 0)
        {
            LOG_INF("Sensor Data Statistics:");
            LOG_INF("  Foot data: %u received, %u dropped (max depth: %u)", stats.foot_data_received,
                    stats.foot_data_dropped, stats.max_foot_ring_depth);
            LOG_INF("  IMU data: %u received, %u dropped (max depth: %u)", stats.imu_data_received,
                    stats.imu_data_dropped, stats.max_imu_ring_depth);
            LOG_INF("  Commands: %u received, %u dropped", stats.commands_received, stats.commands_dropped);
        }
        else
        {
            LOG_WRN("Unknown command: %s", command);
        }
    }
}

// Work handler for periodic 100Hz sampling
static void periodic_sample_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (atomic_get(&processing_active) == 1)
    {
        process_sensor_sample();
        sensor_state.sample_count++;
        
        // Check if thread priority needs to be adjusted back to normal after processing backlog
        if (stats.foot_data_dropped == 0 && stats.imu_data_dropped == 0)
        {
            int current_priority = k_thread_priority_get(k_current_get());
            if (current_priority < sensor_data_priority) // Assuming lower number is higher priority
            {
                k_thread_priority_set(k_current_get(), sensor_data_priority);
                LOG_INF("Restored sensor data thread priority to default %d", sensor_data_priority);
            }
        }
    }

    // Reschedule for next sample (12.5ms ≈ 80Hz, using 12ms as approximation)
    k_work_schedule_for_queue(&sensor_data_work_q, &periodic_sample_work, K_MSEC(12));
}

// Process sensor data at 100Hz
static void process_sensor_sample(void)
{
    uint32_t current_time = k_uptime_get_32();

    // Calculate delta time since last sample
    uint16_t delta_time_ms = 12; // Default to 12ms (approximately 80Hz)
    if (sensor_state.last_sample_time > 0)
    {
        delta_time_ms = current_time - sensor_state.last_sample_time;
        // Clamp to reasonable values (5-50ms)
        if (delta_time_ms < 5)
            delta_time_ms = 5;
        if (delta_time_ms > 50)
            delta_time_ms = 50;
    }
    sensor_state.last_sample_time = current_time;

    // Update contact elapsed times
    if (sensor_state.left_foot_contact)
    {
        // Check for overflow before adding
        if ((UINT16_MAX - sensor_state.left_contact_elapsed_ms) > delta_time_ms)
        {
            sensor_state.left_contact_elapsed_ms += delta_time_ms;
        }
        else
        {
            // Saturate at max uint16_t (65.535 seconds)
            sensor_state.left_contact_elapsed_ms = UINT16_MAX;
        }
    }

    if (sensor_state.right_foot_contact)
    {
        // Check for overflow before adding
        if ((UINT16_MAX - sensor_state.right_contact_elapsed_ms) > delta_time_ms)
        {
            sensor_state.right_contact_elapsed_ms += delta_time_ms;
        }
        else
        {
            // Saturate at max uint16_t (65.535 seconds)
            sensor_state.right_contact_elapsed_ms = UINT16_MAX;
        }
    }

    // Update bilateral timing metrics
    sensor_state.true_flight_time_ms =
        update_true_flight_time(sensor_state.left_foot_contact, sensor_state.right_foot_contact, delta_time_ms,
                                sensor_state.true_flight_time_ms);

    sensor_state.double_support_time_ms =
        update_double_support_time(sensor_state.left_foot_contact, sensor_state.right_foot_contact, delta_time_ms,
                                   sensor_state.double_support_time_ms);

    // Create consolidated data structure - initialize all fields
    sensor_data_consolidated_t consolidated = {};

    // Set time delta
    consolidated.delta_time_ms = delta_time_ms;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // PRIMARY DEVICE: Has data from BOTH feet
    // Left foot = from D2D (secondary), Right foot = local
    
    // Check D2D data freshness for left foot with buffering mechanism
    uint32_t left_data_age = current_time - sensor_state.left_foot_last_update_time;
    bool left_data_valid = (left_data_age < 200); // Extended window to 200ms for buffering
    
    // Buffer for primary (right) foot data to wait for secondary (left) data
    static sensor_data_consolidated_t buffered_primary_data = {};
    static bool primary_data_buffered = false;
    static uint32_t primary_buffer_time = 0;
    
    if (left_data_valid) {
        // Left foot data (from D2D) is available and fresh
        consolidated.left_contact_phase = (uint8_t)sensor_state.left_phase;
        consolidated.left_in_contact = sensor_state.left_foot_contact;
        consolidated.left_contact_duration_ms =
            (uint16_t)(sensor_state.left_foot_contact ? 0 : sensor_state.left_contact_elapsed_ms);
        consolidated.left_peak_force = sensor_state.left_peak_force_current;
        consolidated.left_loading_rate = sensor_state.left_loading_rate;
        consolidated.left_cop_x = sensor_state.left_cop_x;
        consolidated.left_cop_y = sensor_state.left_cop_y;
        consolidated.left_pronation_deg = sensor_state.left_pronation_deg;
        consolidated.left_strike_angle = sensor_state.left_strike_angle;
        consolidated.left_arch_collapse = sensor_state.left_arch_collapse;
        calculate_pressure_distribution(sensor_state.left_pressure,
                                      &consolidated.left_heel_pct,
                                      &consolidated.left_mid_pct,
                                      &consolidated.left_fore_pct);
        
        // Check if we have buffered primary data to pair with this secondary data
        if (primary_data_buffered && (current_time - primary_buffer_time) < 200) {
            // Use buffered primary data for right foot to ensure synchronization
            consolidated.right_contact_phase = buffered_primary_data.right_contact_phase;
            consolidated.right_in_contact = buffered_primary_data.right_in_contact;
            consolidated.right_contact_duration_ms = buffered_primary_data.right_contact_duration_ms;
            consolidated.right_peak_force = buffered_primary_data.right_peak_force;
            consolidated.right_loading_rate = buffered_primary_data.right_loading_rate;
            consolidated.right_cop_x = buffered_primary_data.right_cop_x;
            consolidated.right_cop_y = buffered_primary_data.right_cop_y;
            consolidated.right_pronation_deg = buffered_primary_data.right_pronation_deg;
            consolidated.right_strike_angle = buffered_primary_data.right_strike_angle;
            consolidated.right_arch_collapse = buffered_primary_data.right_arch_collapse;
            consolidated.right_heel_pct = buffered_primary_data.right_heel_pct;
            consolidated.right_mid_pct = buffered_primary_data.right_mid_pct;
            consolidated.right_fore_pct = buffered_primary_data.right_fore_pct;
            consolidated.true_flight_time_ms = buffered_primary_data.true_flight_time_ms;
            consolidated.double_support_time_ms = buffered_primary_data.double_support_time_ms;
            primary_data_buffered = false; // Clear buffer after use
            LOG_DBG("Used buffered primary data for synchronized processing");
        }
    } else {
        // D2D data is stale - buffer the primary data and mark left as invalid
        if (!primary_data_buffered) {
            // Buffer current primary data for right foot
            buffered_primary_data.right_contact_phase = (uint8_t)sensor_state.right_phase;
            buffered_primary_data.right_in_contact = sensor_state.right_foot_contact;
            buffered_primary_data.right_contact_duration_ms =
                (uint16_t)(sensor_state.right_foot_contact ? 0 : sensor_state.right_contact_elapsed_ms);
            buffered_primary_data.right_peak_force = sensor_state.right_peak_force_current;
            buffered_primary_data.right_loading_rate = sensor_state.right_loading_rate;
            buffered_primary_data.right_cop_x = sensor_state.right_cop_x;
            buffered_primary_data.right_cop_y = sensor_state.right_cop_y;
            buffered_primary_data.right_pronation_deg = sensor_state.right_pronation_deg;
            buffered_primary_data.right_strike_angle = sensor_state.right_strike_angle;
            buffered_primary_data.right_arch_collapse = sensor_state.right_arch_collapse;
            calculate_pressure_distribution(sensor_state.right_pressure,
                                          &buffered_primary_data.right_heel_pct,
                                          &buffered_primary_data.right_mid_pct,
                                          &buffered_primary_data.right_fore_pct);
            buffered_primary_data.true_flight_time_ms = sensor_state.true_flight_time_ms;
            buffered_primary_data.double_support_time_ms = sensor_state.double_support_time_ms;
            primary_data_buffered = true;
            primary_buffer_time = current_time;
            LOG_DBG("Buffered primary data waiting for secondary data");
        }
        
        // Mark left foot data as invalid since it's stale
        consolidated.left_contact_phase = PHASE_NO_CONTACT;
        consolidated.left_in_contact = false;
        consolidated.left_contact_duration_ms = 0;
        consolidated.left_peak_force = 0;
        consolidated.left_loading_rate = 0;
        consolidated.left_cop_x = 0;
        consolidated.left_cop_y = 0;
        consolidated.left_pronation_deg = 0;
        consolidated.left_strike_angle = 0;
        consolidated.left_arch_collapse = 0;
        consolidated.left_heel_pct = 0;
        consolidated.left_mid_pct = 0;
        consolidated.left_fore_pct = 0;
        
        if (sensor_state.sample_count % 100 == 0 && left_data_age > 1000) {
            LOG_WRN("Left foot D2D data is stale (age: %u ms), buffering primary data", left_data_age);
        }
    }
    
    // Right foot data (local)
    consolidated.right_contact_phase = (uint8_t)sensor_state.right_phase;
    consolidated.right_in_contact = sensor_state.right_foot_contact;
    consolidated.right_contact_duration_ms = 
        (uint16_t)(sensor_state.right_foot_contact ? 0 : sensor_state.right_contact_elapsed_ms);
    consolidated.right_peak_force = sensor_state.right_peak_force_current;
    consolidated.right_loading_rate = sensor_state.right_loading_rate;
    consolidated.right_cop_x = sensor_state.right_cop_x;
    consolidated.right_cop_y = sensor_state.right_cop_y;
    consolidated.right_pronation_deg = sensor_state.right_pronation_deg;
    consolidated.right_strike_angle = sensor_state.right_strike_angle;
    consolidated.right_arch_collapse = sensor_state.right_arch_collapse;
    calculate_pressure_distribution(sensor_state.right_pressure,
                                  &consolidated.right_heel_pct,
                                  &consolidated.right_mid_pct,
                                  &consolidated.right_fore_pct);
    
    // Bilateral timing (only valid when both feet have fresh data)
    if (left_data_valid) {
        consolidated.true_flight_time_ms = sensor_state.true_flight_time_ms;
        consolidated.double_support_time_ms = sensor_state.double_support_time_ms;
    } else {
        consolidated.true_flight_time_ms = 0;
        consolidated.double_support_time_ms = 0;
    }
    
#else
    // SECONDARY DEVICE: Only has LEFT foot data (local)
    
    // Left foot data (local)
    consolidated.left_contact_phase = (uint8_t)sensor_state.left_phase;
    consolidated.left_in_contact = sensor_state.left_foot_contact;
    consolidated.left_contact_duration_ms = 
        (uint16_t)(sensor_state.left_foot_contact ? 0 : sensor_state.left_contact_elapsed_ms);
    consolidated.left_peak_force = sensor_state.left_peak_force_current;
    consolidated.left_loading_rate = sensor_state.left_loading_rate;
    consolidated.left_cop_x = sensor_state.left_cop_x;
    consolidated.left_cop_y = sensor_state.left_cop_y;
    consolidated.left_pronation_deg = sensor_state.left_pronation_deg;
    consolidated.left_strike_angle = sensor_state.left_strike_angle;
    consolidated.left_arch_collapse = sensor_state.left_arch_collapse;
    calculate_pressure_distribution(sensor_state.left_pressure,
                                  &consolidated.left_heel_pct,
                                  &consolidated.left_mid_pct,
                                  &consolidated.left_fore_pct);
    
    // Right foot data - NONE on secondary, set to zero/invalid
    consolidated.right_contact_phase = PHASE_NO_CONTACT;
    consolidated.right_in_contact = false;
    consolidated.right_contact_duration_ms = 0;
    consolidated.right_peak_force = 0;
    consolidated.right_loading_rate = 0;
    consolidated.right_cop_x = 0;
    consolidated.right_cop_y = 0;
    consolidated.right_pronation_deg = 0;
    consolidated.right_strike_angle = 0;
    consolidated.right_arch_collapse = 0;
    consolidated.right_heel_pct = 0;
    consolidated.right_mid_pct = 0;
    consolidated.right_fore_pct = 0;
    
    // No bilateral timing on secondary
    consolidated.true_flight_time_ms = 0;
    consolidated.double_support_time_ms = 0;
#endif

    // IMU data (local on both devices)
    memcpy(consolidated.quaternion, sensor_state.latest_quaternion, sizeof(consolidated.quaternion));
    memcpy(consolidated.linear_acc, sensor_state.latest_linear_acc, sizeof(consolidated.linear_acc));
    memcpy(consolidated.gyro, sensor_state.latest_gyro, sizeof(consolidated.gyro));
    
    // Step count
    consolidated.step_count = sensor_state.latest_step_count;
    consolidated.step_count_delta = (uint8_t)(sensor_state.latest_step_count - sensor_state.last_step_count);
    
    // Update last step count
    sensor_state.last_step_count = sensor_state.latest_step_count;

    // Log periodically
    if (sensor_state.sample_count % 100 == 0)
    {
        LOG_DBG("Processed %u samples - L:%s/%ums R:%s/%ums", sensor_state.sample_count,
                sensor_state.left_foot_contact ? "Contact" : "Swing", sensor_state.left_contact_elapsed_ms,
                sensor_state.right_foot_contact ? "Contact" : "Swing", sensor_state.right_contact_elapsed_ms);
    }

    // Send consolidated data to other threads
    generic_message_t out_msg = {};
    out_msg.sender = SENDER_SENSOR_DATA;
    out_msg.type = MSG_TYPE_SENSOR_DATA_CONSOLIDATED;

    // Copy consolidated data to message
    memcpy(&out_msg.data.sensor_consolidated, &consolidated, sizeof(sensor_data_consolidated_t));

    // Send to realtime metrics, analytics, and session threads
    k_msgq_put(&sensor_data_queue, &out_msg, K_NO_WAIT);

// Also send to realtime_metrics module if enabled
#if IS_ENABLED(CONFIG_REALTIME_METRICS_MODULE)
    extern struct k_msgq realtime_queue;
    k_msgq_put(&realtime_queue, &out_msg, K_NO_WAIT);
#endif
}

static void process_synchronized_foot_data(const synchronized_foot_data_t *sync_data)
{
    if (!sync_data)
        return;

    // Log synchronized data
    LOG_DBG("Synchronized L/R foot data at time %u", sync_data->sync_time);

    // Calculate contact state from pressure values
    uint32_t left_total = 0, right_total = 0;
    for (int i = 0; i < 8; i++)
    {
        left_total += sync_data->left.values[i];
        right_total += sync_data->right.values[i];
    }

    // Use threshold to determine contact (adjust threshold as needed)
    const uint32_t CONTACT_THRESHOLD = 100;
    bool left_contact = (left_total > CONTACT_THRESHOLD);
    bool right_contact = (right_total > CONTACT_THRESHOLD);

    // Detect bilateral states
    bool both_on_ground = left_contact && right_contact;
    bool both_in_air = !left_contact && !right_contact;

    if (both_in_air)
    {
        LOG_DBG("TRUE FLIGHT detected! (L:%u, R:%u)", left_total, right_total);
        // Update bilateral timing in sensor state
        sensor_state.true_flight_time_ms += 10; // Assuming 100Hz sampling
    }
    else if (both_on_ground)
    {
        LOG_DBG("DOUBLE SUPPORT phase (L:%u, R:%u)", left_total, right_total);
        sensor_state.double_support_time_ms += 10;
    }
    else
    {
        // Reset counters when not in pure bilateral state
        sensor_state.true_flight_time_ms = 0;
        sensor_state.double_support_time_ms = 0;
    }

    // Check if logging is active using atomic variable
    if (atomic_get(&processing_active) == 1)
    {
        // TODO: Create combined log entry for synchronized data
        // This would log both feet data with synchronized timestamp
    }
}

static void process_full_synchronized_data(const full_synchronized_data_t *full_sync)
{
    if (!full_sync)
        return;

    LOG_DBG("Full sync data at time %u (quality: %u%%)", full_sync->sync_time, full_sync->sync_quality);

    // Calculate contact states from pressure
    uint32_t left_total = 0, right_total = 0;
    for (int i = 0; i < 8; i++)
    {
        left_total += full_sync->left_foot.values[i];
        right_total += full_sync->right_foot.values[i];
    }

    const uint32_t CONTACT_THRESHOLD = 100;
    bool left_contact = (left_total > CONTACT_THRESHOLD);
    bool right_contact = (right_total > CONTACT_THRESHOLD);

    // Bilateral gait phase detection
    if (!left_contact && !right_contact)
    {
        LOG_DBG("TRUE FLIGHT phase detected");

        // With bilateral IMU data, we can calculate:
        if (full_sync->left_imu.valid && full_sync->right_imu.valid)
        {
            // 1. Body center of mass trajectory
            float com_acc_y = (full_sync->left_imu.linear_acc[1] + full_sync->right_imu.linear_acc[1]) / 2.0f;

            // 2. Body rotation (comparing left vs right quaternions)
            // This could detect body lean, rotation during flight

            LOG_DBG("Flight phase - CoM vertical acc: %.2f m/s²", (double)com_acc_y);
        }
    }
    else if (left_contact && right_contact)
    {
        LOG_DBG("DOUBLE SUPPORT phase");

        if (full_sync->left_imu.valid && full_sync->right_imu.valid)
        {
            // Calculate weight distribution based on forces and orientations
            float left_weight_pct = (float)left_total / (left_total + right_total) * 100;
            LOG_DBG("Weight distribution - L: %.1f%%, R: %.1f%%", (double)left_weight_pct,
                    (double)(100 - left_weight_pct));
        }
    }
    else if (left_contact && !right_contact)
    {
        LOG_DBG("LEFT SINGLE SUPPORT");
    }
    else
    {
        LOG_DBG("RIGHT SINGLE SUPPORT");
    }

    // Advanced bilateral metrics (only with both IMUs)
    if (full_sync->left_imu.valid && full_sync->right_imu.valid)
    {
        // 1. Gait symmetry based on accelerations
        float acc_diff_x = fabsf(full_sync->left_imu.linear_acc[0] - full_sync->right_imu.linear_acc[0]);
        float acc_diff_y = fabsf(full_sync->left_imu.linear_acc[1] - full_sync->right_imu.linear_acc[1]);
        float acc_diff_z = fabsf(full_sync->left_imu.linear_acc[2] - full_sync->right_imu.linear_acc[2]);

        float symmetry_index = 100.0f - (acc_diff_x + acc_diff_y + acc_diff_z) * 10.0f;
        if (symmetry_index < 0)
            symmetry_index = 0;

        // 2. Step count synchronization check
        int32_t step_diff = (int32_t)full_sync->left_imu.step_count - (int32_t)full_sync->right_imu.step_count;

        if (abs(step_diff) > 2)
        {
            LOG_WRN("Step count mismatch - L: %u, R: %u (diff: %d)", full_sync->left_imu.step_count,
                    full_sync->right_imu.step_count, step_diff);
        }

        LOG_DBG("Gait symmetry: %.1f%%, Step diff: %d", (double)symmetry_index, step_diff);
    }

    // Create enhanced consolidated message if needed
    // This could be sent to analytics for long-term bilateral analysis
}

// Function to update foot state directly from D2D data
static void update_foot_state_from_d2d(const foot_samples_t *foot_data, uint8_t foot_id)
{
    if (!foot_data) return;
    
    if (foot_id == 0) {
        // Track when we last updated from D2D
        sensor_state.left_foot_last_update_time = k_uptime_get_32();
        
        // Update left foot state from D2D data
        for (int i = 0; i < 8; i++) {
            sensor_state.left_pressure[i] = foot_data->values[i];
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
            sensor_state.left_strike_pattern = detect_strike_pattern(heel_pct, mid_pct, fore_pct);
            
            // Calculate foot strike angle using IMU + pressure
            uint16_t heel_force = sensor_state.left_pressure[0] + sensor_state.left_pressure[1];
            uint16_t fore_force = sensor_state.left_pressure[5] + 
                                 sensor_state.left_pressure[6] + 
                                 sensor_state.left_pressure[7];
            sensor_state.left_strike_angle = calculate_foot_strike_angle(
                sensor_state.latest_quaternion, heel_force, fore_force);
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
                sensor_state.left_pressure, sensor_state.left_phase);
        } else {
            // Reset metrics when not in contact
            sensor_state.left_peak_force_current = 0;
            sensor_state.left_previous_force = 0;
            sensor_state.left_loading_rate = 0;
        }
        
        LOG_DBG("D2D left foot data updated at %u ms", sensor_state.left_foot_last_update_time);
    }
    // Note: foot_id == 1 (right foot) should never come from D2D on primary
}

// Function to update IMU state directly from D2D data
static void update_imu_state_from_d2d(const bhi360_log_record_t *imu_data)
{
    if (!imu_data) return;
    
    // Track when we last updated IMU from D2D
    sensor_state.left_imu_last_update_time = k_uptime_get_32();
    
    // For D2D IMU data, we might want to store it separately
    // or merge it with local IMU data for bilateral analysis
    // This depends on your specific requirements
    
    LOG_DBG("D2D IMU data updated at %u ms", sensor_state.left_imu_last_update_time);
}

// Module event handler
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        const struct module_state_event *event = cast_module_state_event(aeh);

        // Wait for motion sensor and foot sensor to be ready
        if (check_state(event, MODULE_ID(motion_sensor), MODULE_STATE_READY) &&
            check_state(event, MODULE_ID(foot_sensor), MODULE_STATE_READY))
        {
            if (!module_initialized)
            {
                sensor_data_init();
            }
        }
        return false;
    }

    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
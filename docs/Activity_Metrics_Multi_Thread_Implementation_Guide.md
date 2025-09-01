# Activity Metrics Multi-Thread Implementation Guide

**Version:** 1.0  
**Date:** June 2025  
**Purpose:** Step-by-step guide to implement the multi-thread architecture for activity metrics

---

## Executive Summary

This guide explains how to transition from the current single-thread `activity_metrics` module to the multi-thread architecture described in the Activity Metrics Threading Architecture document. We'll create three additional modules while keeping the existing one, using Zephyr's work queue system for efficient task scheduling.

---

## Current vs. Target Architecture

### Current Architecture (Single Thread)
```
activity_metrics_thread
├── Sensor data collection (100Hz)
├── Real-time calculations (variable)
├── Complex analytics (heavy)
└── Session management (file I/O)
```

### Target Architecture (Multi-Thread)
```
sensor_collection (Thread 1) ──┐
                               ├── Message Queues ──┐
realtime_metrics (Thread 2) ───┤                    ├── analytics_processing (Thread 3)
                               │                    │   └── Complex analytics
                               └────────────────────┴── session_management (Thread 4)
                                                        └── Uses existing data module
```

---

## Implementation Strategy

### Phase 1: Create Module Structure

We'll create three new modules that work alongside the existing `activity_metrics` module:

1. **sensor_collection** - High-frequency sensor data gathering
2. **realtime_metrics** - Time-sensitive calculations for BLE
3. **session_management** - File I/O and logging

The existing `activity_metrics` module will be refactored to handle only complex analytics.

### Phase 2: Use Work Queues for Task Distribution

Instead of direct function calls, we'll use Zephyr's work queue system to distribute tasks based on priority.

---

## Step-by-Step Implementation

### Step 1: Create the Sensor Collection Module

Create `src/sensor_collection/sensor_collection.cpp`:

```c
/**
 * @file sensor_collection.cpp
 * @brief High-frequency sensor data collection module
 */

#define MODULE sensor_collection

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_SENSOR_COLLECTION_LOG_LEVEL);

// Thread configuration
#define SENSOR_THREAD_STACK_SIZE 4096
#define SENSOR_THREAD_PRIORITY 6

K_THREAD_STACK_DEFINE(sensor_thread_stack, SENSOR_THREAD_STACK_SIZE);
static struct k_thread sensor_thread_data;
static k_tid_t sensor_thread_id;

// Message queue for incoming sensor data
K_MSGQ_DEFINE(sensor_input_queue, sizeof(generic_message_t), 50, 4);

// Work queue for distributing processed data
static struct k_work_q sensor_work_q;
K_THREAD_STACK_DEFINE(sensor_work_q_stack, 2048);

// Work items for different processing levels
struct sensor_work_item {
    struct k_work work;
    SensorDataMsg data;
    uint8_t priority;
};

// Pool of work items to avoid dynamic allocation
#define WORK_ITEM_POOL_SIZE 20
static struct sensor_work_item work_item_pool[WORK_ITEM_POOL_SIZE];
static struct k_mem_slab work_item_slab;
static char __aligned(4) work_item_buffer[WORK_ITEM_POOL_SIZE * sizeof(struct sensor_work_item)];

// Contact detection state
static ContactDetector left_contact = {0};
static ContactDetector right_contact = {0};

// Forward declarations
static void sensor_thread_fn(void *p1, void *p2, void *p3);
static void process_sensor_work(struct k_work *work);
static void distribute_sensor_data(SensorDataMsg *data, uint8_t priority);

// Initialize module
static int sensor_collection_init(void)
{
    LOG_INF("Initializing sensor collection module");
    
    // Initialize work item pool
    k_mem_slab_init(&work_item_slab, work_item_buffer, 
                    sizeof(struct sensor_work_item), WORK_ITEM_POOL_SIZE);
    
    // Initialize work queue
    k_work_queue_init(&sensor_work_q);
    k_work_queue_start(&sensor_work_q, sensor_work_q_stack,
                      K_THREAD_STACK_SIZEOF(sensor_work_q_stack),
                      SENSOR_THREAD_PRIORITY + 1, NULL);
    k_thread_name_set(&sensor_work_q.thread, "sensor_work_q");
    
    // Create sensor thread
    sensor_thread_id = k_thread_create(&sensor_thread_data,
                                      sensor_thread_stack,
                                      K_THREAD_STACK_SIZEOF(sensor_thread_stack),
                                      sensor_thread_fn,
                                      NULL, NULL, NULL,
                                      SENSOR_THREAD_PRIORITY,
                                      0, K_NO_WAIT);
    
    k_thread_name_set(sensor_thread_id, "sensor_collection");
    
    module_set_state(MODULE_STATE_READY);
    return 0;
}

// Main sensor thread - 100Hz collection
static void sensor_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("Sensor collection thread started");
    
    while (1) {
        generic_message_t msg;
        
        // Wait for sensor data with 10ms timeout (100Hz)
        if (k_msgq_get(&sensor_input_queue, &msg, K_MSEC(10)) == 0) {
            SensorDataMsg sensor_data = {0};
            sensor_data.timestamp = k_uptime_get_32();
            
            // Process based on message type
            switch (msg.type) {
                case MSG_TYPE_FOOT_SAMPLES: {
                    // Determine foot and process
                    uint8_t foot = (msg.sender == SENDER_FOOT_SENSOR_THREAD) ? 
                                  #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                                  1 : 0;  // Primary = right
                                  #else
                                  0 : 1;  // Secondary = left
                                  #endif
                    
                    // Convert to float and store
                    for (int i = 0; i < 8; i++) {
                        sensor_data.pressure[i + (foot * 8)] = 
                            (float)msg.data.foot_samples.values[i];
                    }
                    
                    // Detect ground contact (critical calculation)
                    float total_force = 0;
                    for (int i = 0; i < 8; i++) {
                        total_force += sensor_data.pressure[i + (foot * 8)];
                    }
                    
                    ContactDetector *contact = foot ? &right_contact : &left_contact;
                    bool contact_changed = detect_ground_contact(contact, 
                                                               total_force,
                                                               sensor_data.timestamp);
                    
                    if (contact_changed) {
                        // High priority - send immediately to realtime thread
                        distribute_sensor_data(&sensor_data, PRIORITY_HIGH);
                    }
                    break;
                }
                
                case MSG_TYPE_BHI360_3D_MAPPING:
                    // Extract quaternion and gyro
                    sensor_data.quaternion[0] = msg.data.bhi360_3d_mapping.quat_w;
                    sensor_data.quaternion[1] = msg.data.bhi360_3d_mapping.quat_x;
                    sensor_data.quaternion[2] = msg.data.bhi360_3d_mapping.quat_y;
                    sensor_data.quaternion[3] = msg.data.bhi360_3d_mapping.quat_z;
                    // Medium priority
                    distribute_sensor_data(&sensor_data, PRIORITY_MEDIUM);
                    break;
                    
                case MSG_TYPE_BHI360_STEP_COUNT:
                    sensor_data.step_count = msg.data.bhi360_step_count.step_count;
                    // Medium priority
                    distribute_sensor_data(&sensor_data, PRIORITY_MEDIUM);
                    break;
            }
        }
        
        // Periodic sensor reading can go here if needed
    }
}

// Distribute sensor data to appropriate threads via work queue
static void distribute_sensor_data(SensorDataMsg *data, uint8_t priority)
{
    struct sensor_work_item *item;
    
    // Get work item from pool
    if (k_mem_slab_alloc(&work_item_slab, (void**)&item, K_NO_WAIT) == 0) {
        // Copy data
        memcpy(&item->data, data, sizeof(SensorDataMsg));
        item->priority = priority;
        
        // Initialize work
        k_work_init(&item->work, process_sensor_work);
        
        // Submit to work queue
        k_work_submit_to_queue(&sensor_work_q, &item->work);
    } else {
        LOG_WRN("Work item pool exhausted");
    }
}

// Process sensor work item
static void process_sensor_work(struct k_work *work)
{
    struct sensor_work_item *item = CONTAINER_OF(work, struct sensor_work_item, work);
    
    // Send to appropriate queue based on priority
    if (item->priority == PRIORITY_HIGH) {
        // Send to realtime metrics queue
        k_msgq_put(&realtime_metrics_queue, &item->data, K_NO_WAIT);
    } else {
        // Send to activity metrics queue for complex processing
        k_msgq_put(&activity_metrics_msgq, &item->data, K_NO_WAIT);
    }
    
    // Return item to pool
    k_mem_slab_free(&work_item_slab, (void*)item);
}

// Module event handler
static bool app_event_handler(const struct app_event_header *aeh)
{
    // Handle module state events
    if (is_module_state_event(aeh)) {
        const struct module_state_event *event = cast_module_state_event(aeh);
        
        if (check_state(event, MODULE_ID(motion_sensor), MODULE_STATE_READY) &&
            check_state(event, MODULE_ID(foot_sensor), MODULE_STATE_READY)) {
            sensor_collection_init();
        }
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
```

### Step 2: Create the Realtime Metrics Module

Create `src/realtime_metrics/realtime_metrics.cpp`:

```c
/**
 * @file realtime_metrics.cpp
 * @brief Real-time metrics calculation for BLE updates
 */

#define MODULE realtime_metrics

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_REALTIME_METRICS_LOG_LEVEL);

// Thread configuration
#define REALTIME_THREAD_STACK_SIZE 8192
#define REALTIME_THREAD_PRIORITY 5  // Highest priority

K_THREAD_STACK_DEFINE(realtime_thread_stack, REALTIME_THREAD_STACK_SIZE);
static struct k_thread realtime_thread_data;

// Input queue from sensor collection
K_MSGQ_DEFINE(realtime_metrics_queue, sizeof(SensorDataMsg), 20, 4);

// Work queue for BLE updates
static struct k_work_q ble_work_q;
K_THREAD_STACK_DEFINE(ble_work_q_stack, 2048);

// Delayed work for 1Hz BLE updates
static struct k_work_delayable ble_update_work;

// Metrics state
static struct {
    CadenceTracker cadence;
    PaceCalculator pace;
    FormAnalyzer form;
    RealtimeMetricsMsg current_metrics;
    uint32_t last_ble_update;
} metrics_state;

// Forward declarations
static void realtime_thread_fn(void *p1, void *p2, void *p3);
static void ble_update_handler(struct k_work *work);
static void calculate_realtime_metrics(SensorDataMsg *data);

// Initialize module
static int realtime_metrics_init(void)
{
    LOG_INF("Initializing realtime metrics module");
    
    // Initialize work queue for BLE
    k_work_queue_init(&ble_work_q);
    k_work_queue_start(&ble_work_q, ble_work_q_stack,
                      K_THREAD_STACK_SIZEOF(ble_work_q_stack),
                      REALTIME_THREAD_PRIORITY + 1, NULL);
    
    // Initialize delayed work for BLE updates
    k_work_init_delayable(&ble_update_work, ble_update_handler);
    
    // Create realtime thread
    k_thread_create(&realtime_thread_data,
                    realtime_thread_stack,
                    K_THREAD_STACK_SIZEOF(realtime_thread_stack),
                    realtime_thread_fn,
                    NULL, NULL, NULL,
                    REALTIME_THREAD_PRIORITY,
                    0, K_NO_WAIT);
    
    k_thread_name_set(&realtime_thread_data, "realtime_metrics");
    
    // Schedule first BLE update
    k_work_schedule_for_queue(&ble_work_q, &ble_update_work, K_MSEC(1000));
    
    return 0;
}

// Main realtime thread
static void realtime_thread_fn(void *p1, void *p2, void *p3)
{
    LOG_INF("Realtime metrics thread started");
    
    while (1) {
        SensorDataMsg sensor_data;
        
        // Wait for high-priority sensor data
        if (k_msgq_get(&realtime_metrics_queue, &sensor_data, K_MSEC(50)) == 0) {
            // Calculate time-sensitive metrics
            calculate_realtime_metrics(&sensor_data);
        }
    }
}

// Calculate realtime metrics
static void calculate_realtime_metrics(SensorDataMsg *data)
{
    // Update cadence from step count
    if (data->step_count > 0) {
        float cadence = calculate_cadence_from_bhi360(&metrics_state.cadence,
                                                     data->step_count,
                                                     data->timestamp);
        metrics_state.current_metrics.cadence_x2 = (uint16_t)(cadence * 2);
    }
    
    // Calculate pace
    float pace_sec_km = calculate_pace_with_sensors(
        metrics_state.current_metrics.cadence_x2 / 2.0f,
        get_avg_contact_time(),
        get_avg_flight_time(),
        user_profile.height_cm,
        get_vertical_oscillation()
    );
    metrics_state.current_metrics.pace_sec_per_km = (uint16_t)pace_sec_km;
    
    // Update form score
    metrics_state.current_metrics.form_score = calculate_form_score(
        &metrics_state.form,
        data->quaternion,
        data->pressure,
        get_avg_contact_time(),
        metrics_state.current_metrics.cadence_x2 / 2.0f
    );
    
    // Calculate balance
    metrics_state.current_metrics.balance_lr = calculate_lr_balance(data->pressure);
}

// BLE update handler - runs at 1Hz
static void ble_update_handler(struct k_work *work)
{
    uint32_t now = k_uptime_get_32();
    
    // Prepare BLE packet
    RealtimeMetricsPacket packet = {
        .delta_time_ms = (uint16_t)(now - metrics_state.last_ble_update),
        .cadence_x2 = metrics_state.current_metrics.cadence_x2,
        .pace_sec_per_km = metrics_state.current_metrics.pace_sec_per_km,
        .form_score = metrics_state.current_metrics.form_score,
        .balance_lr = metrics_state.current_metrics.balance_lr,
        // ... other fields
    };
    
    // Send to BLE
    send_ble_metrics(&packet);
    
    metrics_state.last_ble_update = now;
    
    // Schedule next update
    k_work_schedule_for_queue(&ble_work_q, &ble_update_work, K_MSEC(1000));
}

SYS_INIT(realtime_metrics_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
```

### Step 3: Create the Session Management Module

Create `src/session_management/session_management.cpp`:

```c
/**
 * @file session_management.cpp
 * @brief Session management and file I/O module
 */

#define MODULE session_management

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_SESSION_MANAGEMENT_LOG_LEVEL);

// Thread configuration
#define SESSION_THREAD_STACK_SIZE 4096
#define SESSION_THREAD_PRIORITY 12  // Lowest priority

K_THREAD_STACK_DEFINE(session_thread_stack, SESSION_THREAD_STACK_SIZE);
static struct k_thread session_thread_data;

// Message queue for session commands
K_MSGQ_DEFINE(session_cmd_queue, sizeof(SessionCommand), 5, 4);

// Work queue for file operations
static struct k_work_q file_work_q;
K_THREAD_STACK_DEFINE(file_work_q_stack, 4096);

// Delayed work for periodic logging
static struct k_work_delayable periodic_log_work;

// Session state
static struct {
    bool active;
    SessionHeader header;
    struct fs_file_t log_file;
    uint32_t last_record_time;
    uint32_t record_count;
} session;

// Forward declarations
static void session_thread_fn(void *p1, void *p2, void *p3);
static void periodic_log_handler(struct k_work *work);
static void write_session_record(AnalyticsResultMsg *analytics);

// Initialize module
static int session_management_init(void)
{
    LOG_INF("Initializing session management module");
    
    // Initialize file work queue
    k_work_queue_init(&file_work_q);
    k_work_queue_start(&file_work_q, file_work_q_stack,
                      K_THREAD_STACK_SIZEOF(file_work_q_stack),
                      SESSION_THREAD_PRIORITY, NULL);
    
    // Initialize delayed work
    k_work_init_delayable(&periodic_log_work, periodic_log_handler);
    
    // Create session thread
    k_thread_create(&session_thread_data,
                    session_thread_stack,
                    K_THREAD_STACK_SIZEOF(session_thread_stack),
                    session_thread_fn,
                    NULL, NULL, NULL,
                    SESSION_THREAD_PRIORITY,
                    0, K_NO_WAIT);
    
    k_thread_name_set(&session_thread_data, "session_mgmt");
    
    return 0;
}

// Main session thread
static void session_thread_fn(void *p1, void *p2, void *p3)
{
    LOG_INF("Session management thread started");
    
    while (1) {
        SessionCommand cmd;
        
        // Wait for commands
        if (k_msgq_get(&session_cmd_queue, &cmd, K_MSEC(500)) == 0) {
            switch (cmd.type) {
                case SESSION_START:
                    start_new_session(&cmd.header);
                    break;
                    
                case SESSION_STOP:
                    stop_current_session();
                    break;
                    
                case GPS_UPDATE:
                    process_gps_update(&cmd.gps_data);
                    break;
            }
        }
        
        // Check for analytics results to log
        if (session.active) {
            AnalyticsResultMsg analytics;
            if (k_msgq_get(&analytics_queue, &analytics, K_NO_WAIT) == 0) {
                write_session_record(&analytics);
            }
        }
    }
}

// Periodic logging handler
static void periodic_log_handler(struct k_work *work)
{
    if (!session.active) return;
    
    // Create periodic record
    PeriodicRecord record = {
        .delta_time_ms = k_uptime_get_32() - session.last_record_time,
        // ... populate other fields
    };
    
    // Write to file (in work queue context)
    fs_write(&session.log_file, &record, sizeof(record));
    
    session.record_count++;
    session.last_record_time = k_uptime_get_32();
    
    // Schedule next write
    k_work_schedule_for_queue(&file_work_q, &periodic_log_work, K_SECONDS(1));
}

SYS_INIT(session_management_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
```

### Step 4: Refactor the Existing Activity Metrics Module

The existing `activity_metrics` module should be refactored to focus only on complex analytics:

```c
// In activity_metrics.cpp - refactored version

// Remove sensor collection code
// Remove real-time metrics code  
// Remove file I/O code
// Keep only complex analytics

static void activity_metrics_thread_fn(void *arg1, void *arg2, void *arg3)
{
    LOG_INF("Activity metrics (analytics) thread started");
    
    // Initialize analytics components
    FatigueAnalyzer fatigue = {0};
    EfficiencyCalculator efficiency = {0};
    InjuryRiskAssessor injury_risk = {0};
    
    while (1) {
        RealtimeMetricsMsg metrics;
        
        // Get metrics from realtime thread
        if (k_msgq_get(&realtime_queue, &metrics, K_MSEC(200)) == 0) {
            // Perform complex calculations
            AnalyticsResultMsg results = {0};
            
            // Running efficiency
            results.efficiency_score = calculate_running_efficiency(
                &efficiency,
                metrics.cadence_x2 / 2.0f,
                get_duty_factor(),
                get_vertical_oscillation(),
                get_forward_lean()
            );
            
            // Fatigue analysis
            results.fatigue_index = calculate_fatigue_index(
                &fatigue,
                get_avg_contact_time(),
                get_avg_flight_time(),
                get_loading_rate(),
                metrics.form_score,
                k_uptime_get_32()
            );
            
            // Injury risk
            results.injury_risk = assess_injury_risk(
                &injury_risk,
                get_loading_rate(),
                metrics.balance_lr,
                get_pronation_angle(),
                get_strike_pattern(),
                results.fatigue_index
            );
            
            // Send to session management
            k_msgq_put(&analytics_queue, &results, K_NO_WAIT);
        }
    }
}
```

---

## Configuration Changes

### Kconfig Updates

Add to `Kconfig`:

```kconfig
# Sensor Collection Module
config SENSOR_COLLECTION_MODULE
    bool "Enable sensor collection module"
    default y
    help
      High-frequency sensor data collection at 100Hz

config SENSOR_COLLECTION_MODULE_STACK_SIZE
    int "Sensor collection thread stack size"
    default 4096

config SENSOR_COLLECTION_MODULE_PRIORITY
    int "Sensor collection thread priority"
    default 6

# Realtime Metrics Module
config REALTIME_METRICS_MODULE
    bool "Enable realtime metrics module"
    default y
    help
      Real-time metrics calculation for BLE updates

config REALTIME_METRICS_MODULE_STACK_SIZE
    int "Realtime metrics thread stack size"
    default 8192

config REALTIME_METRICS_MODULE_PRIORITY
    int "Realtime metrics thread priority"
    default 5

# Session Management Module
config SESSION_MANAGEMENT_MODULE
    bool "Enable session management module"
    default y
    help
      Session management and file I/O operations

config SESSION_MANAGEMENT_MODULE_STACK_SIZE
    int "Session management thread stack size"
    default 4096

config SESSION_MANAGEMENT_MODULE_PRIORITY
    int "Session management thread priority"
    default 12
```

### CMakeLists.txt Updates

Add new modules to build:

```cmake
# Add new modules
add_subdirectory_ifdef(CONFIG_SENSOR_COLLECTION_MODULE sensor_collection)
add_subdirectory_ifdef(CONFIG_REALTIME_METRICS_MODULE realtime_metrics)
add_subdirectory_ifdef(CONFIG_SESSION_MANAGEMENT_MODULE session_management)
```

---

## Message Flow Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│ Motion/Foot     │     │ Sensor Collection│     │ Realtime Metrics│
│ Sensor Threads  │────►│   (Thread 1)     │────►│   (Thread 2)    │
└────────────��────┘     └──────────────────┘     └────────┬────────┘
                                                           │
                        ┌──────────────────┐               │ 1Hz BLE
                        │ Activity Metrics │◄──────────────┘
                        │   (Thread 4)     │
                        └────────┬─────────┘
                                 │
                        ┌────────▼─────────┐
                        │Session Management│
                        │   (Thread 3)     │
                        └──────────────────┘
```

---

## Benefits of This Implementation

### 1. **Modular Design**
- Each module has a single responsibility
- Easy to test and debug independently
- Can disable modules for power saving

### 2. **Priority-Based Processing**
- Work queues ensure high-priority tasks execute first
- BLE updates never blocked by file I/O
- Critical calculations get CPU time

### 3. **Memory Efficiency**
- Pre-allocated work item pools
- No dynamic memory allocation
- Bounded message queues

### 4. **Power Management**
- Can suspend analytics thread when battery low
- Session thread sleeps between writes
- Sensor thread can reduce rate when idle

### 5. **Scalability**
- Easy to add new analytics algorithms
- Can redistribute work between threads
- Future ML models can be added as separate module

---

## Testing Strategy

### Unit Testing Each Module

```c
// Test sensor collection independently
void test_sensor_collection(void)
{
    // Inject test sensor data
    generic_message_t test_msg = {
        .type = MSG_TYPE_FOOT_SAMPLES,
        .sender = SENDER_FOOT_SENSOR_THREAD,
        // ... test data
    };
    
    k_msgq_put(&sensor_input_queue, &test_msg, K_NO_WAIT);
    
    // Verify output in realtime queue
    SensorDataMsg output;
    zassert_equal(k_msgq_get(&realtime_metrics_queue, &output, K_MSEC(100)), 0);
}
```

### Integration Testing

```c
// Test full pipeline
void test_activity_pipeline(void)
{
    // Start activity
    SessionCommand start_cmd = {
        .type = SESSION_START,
        .header = { /* test session */ }
    };
    k_msgq_put(&session_cmd_queue, &start_cmd, K_NO_WAIT);
    
    // Inject sensor data
    inject_test_sensor_stream();
    
    // Wait for processing
    k_sleep(K_SECONDS(5));
    
    // Verify all stages completed
    verify_ble_updates_sent();
    verify_analytics_calculated();
    verify_session_logged();
}
```

---

## Migration Timeline

### Week 1: Infrastructure
- Create new module directories
- Set up Kconfig and CMake
- Implement message queue definitions

### Week 2: Sensor Collection
- Implement sensor collection module
- Test with existing sensor threads
- Verify data flow to queues

### Week 3: Realtime Metrics
- Implement realtime metrics module
- Connect to BLE updates
- Test 1Hz update rate

### Week 4: Session Management
- Implement session management module
- Test file I/O operations
- Verify logging functionality

### Week 5: Integration
- Refactor existing activity_metrics
- Connect all modules
- Full system testing

### Week 6: Optimization
- Profile CPU usage
- Tune thread priorities
- Optimize memory usage

---

## Summary

This implementation guide provides a practical path to transition from the current single-thread architecture to a multi-thread system using:

1. **Zephyr work queues** for priority-based task scheduling
2. **Pre-allocated memory pools** to avoid dynamic allocation
3. **Message queues** for inter-thread communication
4. **Modular design** for maintainability

The new architecture will provide better real-time performance, more efficient CPU usage, and the flexibility to add complex algorithms without impacting BLE updates.
# Activity Metrics Threading Architecture

**Version:** 1.0  
**Date:** June 2025  
**Purpose:** Define the optimal threading architecture for implementing the Activity Session Calculated Data system on nRF5340

---

## Executive Summary

This document outlines the recommended multi-threaded architecture for processing activity metrics on the nRF5340 platform. Given the computational complexity of the algorithms specified in the Activity Session Calculated Data Specification and the real-time requirements for BLE data transmission, a multi-threaded approach with priority-based task scheduling is essential.

**Key Constraints:**
- nRF5340 has dual cores: Application core (for our threads) and Network core (dedicated to Bluetooth)
- BHI360 provides step count directly (no step detection needed)
- Real-time BLE updates required at 1Hz
- Complex algorithms range from <1ms to >50ms execution time
- File I/O must not block real-time processing

---

## Table of Contents

1. [Platform Architecture](#platform-architecture)
2. [Threading Requirements Analysis](#threading-requirements-analysis)
3. [Recommended Architecture](#recommended-architecture)
4. [Thread Specifications](#thread-specifications)
5. [Data Flow Design](#data-flow-design)
6. [Implementation Guidelines](#implementation-guidelines)
7. [Performance Considerations](#performance-considerations)
8. [Alternative Approaches](#alternative-approaches)

---

## Platform Architecture

### nRF5340 Dual-Core Design

The nRF5340 features a dual-core architecture:

1. **Application Core (Cortex-M33)**
   - 128 MHz, 1MB Flash, 512KB RAM
   - Runs application code and RTOS
   - **This is where all our threads will run**

2. **Network Core (Cortex-M33)**
   - 64 MHz, 256KB Flash, 64KB RAM
   - **Dedicated to Bluetooth protocol stack**
   - Handles all BLE communication
   - Isolated from application processing

### Implications for Our Design

Since the Network Core handles all Bluetooth operations independently:
- Application core is free for sensor processing and calculations
- BLE stack won't compete for CPU with our algorithms
- Inter-core communication via IPC (Inter-Processor Communication)
- We can focus on optimizing single-core threading

---

## Threading Requirements Analysis

### Algorithm Computational Complexity

Based on the Activity Session Calculated Data Specification:

#### Fast Algorithms (<1ms execution)
- Ground contact/flight time detection
- Basic pressure distribution
- Peak force detection
- Simple asymmetry calculation
- Impact G-force reading

#### Medium Algorithms (1-10ms execution)
- Cadence extraction from BHI360 steps
- Pace estimation
- Form score calculation
- Pronation analysis with validation
- Loading rate calculation
- Balance L/R computation

#### Slow Algorithms (10-50ms execution)
- Running efficiency (multi-factor)
- Fatigue index (baseline comparison)
- Injury risk assessment (composite)
- CPEI path analysis
- Stride length estimation with corrections

#### Very Slow Operations (>50ms)
- Session statistics aggregation
- File writing operations
- GPS calibration processing
- Historical data analysis

### Real-Time Constraints

1. **Sensor Sampling**: 100Hz (10ms deadline)
2. **BLE Updates**: 1Hz minimum (1000ms deadline)
3. **User Feedback**: <100ms latency for alerts
4. **File Logging**: 0.5-2Hz (relaxed timing)

---

## Recommended Architecture

### Four-Thread Model

Given the single application core and varying computational requirements, we recommend four specialized threads:

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Core (M33)                     │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────────┐  ┌──────────────────┐                  │
│  │  Sensor Thread  │  │ Real-time Thread │                  │
│  │   (100Hz)       │  │    (10-50Hz)     │                  │
│  │   Priority: 6   │  │   Priority: 5    │                  │
│  └────────┬────────┘  └────────┬─────────┘                  │
│           │                     │                             │
│           ▼                     ▼                             │
│      ┌─────────────────────────────────┐                     │
│      │        Message Queues           │                     │
│      └────────┬───────────┬────────────┘                     │
│               │           │                                   │
│               ▼           ▼                                   │
│  ┌──────────────────┐  ┌──────────────────┐                │
│  │ Analytics Thread │  │  Session Thread  │                │
│  │    (1-5Hz)       │  │    (0.5-2Hz)     │                │
│  │   Priority: 10   │  │   Priority: 12   │                │
│  └──────────────────┘  └──────────────────┘                │
│                                                               │
└───────────────────────────┬───────────────────────────────────┘
                            │ IPC
┌───────────────────────────▼───────────────────────────────────┐
│                    Network Core (M33)                          │
│                  Bluetooth Stack (Independent)                 │
└────────────────────────────────────────────────────────────���───┘
```

---

## Thread Specifications

### 1. Sensor Data Thread

**Purpose**: Collect and timestamp all sensor data at consistent 100Hz rate

**Responsibilities**:
- Read BHI360 data (quaternions, linear acceleration, step count)
- Read 8-channel pressure sensors
- Detect ground contact/flight phases from pressure
- Calculate instantaneous peak forces
- Compute basic pressure distribution
- Timestamp all data with microsecond precision
- Push data to message queues

**Key Algorithms**:
```c
// Ground contact detection from pressure
typedef struct {
    uint32_t contact_start_time;
    uint32_t contact_end_time;
    bool in_contact;
    float pressure_threshold;
} ContactDetector;

// Pressure distribution
typedef struct {
    float heel_pct;
    float midfoot_pct;
    float forefoot_pct;
    float total_force;
} PressureDistribution;
```

**Configuration**:
- Priority: 6 (High)
- Stack Size: 4096 bytes
- Period: 10ms (100Hz)
- CPU Budget: ~2ms per cycle

### 2. Real-Time Metrics Thread

**Purpose**: Calculate time-sensitive metrics for BLE transmission

**Responsibilities**:
- Extract cadence from BHI360 step count changes
- Calculate current pace using step timing
- Compute form score from multiple inputs
- Detect balance and asymmetry
- Generate real-time alerts
- Prepare BLE notification packets
- Maintain 1Hz update rate to BLE

**Key Algorithms**:
```c
// Cadence from BHI360 steps
typedef struct {
    uint32_t last_step_count;
    uint32_t last_step_time;
    float instant_cadence;
    float avg_cadence;
} CadenceTracker;

// Real-time pace calculation
typedef struct {
    float current_speed_ms;
    uint16_t pace_sec_per_km;
    uint16_t avg_pace_sec_per_km;
} PaceMetrics;
```

**Configuration**:
- Priority: 5 (Highest)
- Stack Size: 8192 bytes
- Period: 20-100ms (variable)
- CPU Budget: ~5ms per cycle

### 3. Analytics Thread

**Purpose**: Perform complex biomechanical calculations

**Responsibilities**:
- Calculate running efficiency score
- Track fatigue index with baseline
- Assess injury risk factors
- Analyze pronation with IMU+pressure
- Estimate stride length dynamically
- Compute CPEI and pressure paths
- Update session aggregates

**Key Algorithms**:
```c
// Fatigue analysis with baseline
typedef struct {
    float baseline_metrics[10];
    uint32_t baseline_complete_time;
    float current_fatigue_score;
    float degradation_rate;
} FatigueAnalyzer;

// Injury risk assessment
typedef struct {
    float loading_rate_score;
    float asymmetry_score;
    float pronation_score;
    float cumulative_load;
    uint8_t risk_level;  // 0-100
} InjuryRiskAssessment;
```

**Configuration**:
- Priority: 10 (Medium)
- Stack Size: 12288 bytes
- Period: 200-1000ms (adaptive)
- CPU Budget: ~20ms per cycle

### 4. Session Management Thread

**Purpose**: Handle file I/O and session-level operations

**Responsibilities**:
- Write periodic records to flash
- Process GPS calibration updates
- Calculate session statistics
- Generate kilometer splits
- Manage activity log files
- Handle session start/stop
- Compute session summary

**Key Data Structures**:
```c
// From Activity Session spec
typedef struct {
    uint32_t session_id;
    uint32_t start_timestamp;
    uint8_t activity_type;
    uint8_t activity_subtype;
    uint16_t user_weight_kg;
    // ... other fields
} SessionHeader;  // 32 bytes

typedef struct {
    uint16_t delta_time_ms;
    uint16_t cadence_x2;
    uint16_t avg_contact_time_ms;
    FootSummary left_foot;
    FootSummary right_foot;
    // ... other fields
} PeriodicRecord;  // 48 bytes
```

**Configuration**:
- Priority: 12 (Low)
- Stack Size: 4096 bytes
- Period: 500-2000ms
- CPU Budget: ~50ms (during writes)

---

## Data Flow Design

### Message Queue Architecture

```c
// High-frequency sensor data
typedef struct {
    uint32_t timestamp;
    float pressure[8];
    float quaternion[4];
    float linear_acc[3];
    uint32_t step_count;
} SensorDataMsg;

// Real-time metrics for BLE
typedef struct {
    uint16_t cadence_x2;
    uint16_t pace_sec_per_km;
    uint8_t form_score;
    int8_t balance_lr;
    uint8_t fatigue_level;
} RealtimeMetricsMsg;

// Analytics results
typedef struct {
    float efficiency_score;
    float injury_risk;
    float stride_length;
    float pronation_angle;
} AnalyticsResultMsg;

// Queue definitions
K_MSGQ_DEFINE(sensor_data_queue, sizeof(SensorDataMsg), 50, 4);
K_MSGQ_DEFINE(realtime_queue, sizeof(RealtimeMetricsMsg), 20, 4);
K_MSGQ_DEFINE(analytics_queue, sizeof(AnalyticsResultMsg), 10, 4);
K_MSGQ_DEFINE(session_queue, sizeof(PeriodicRecord), 5, 4);
```

### Inter-Thread Communication Flow

```
BHI360 + Pressure Sensors
         │
         ▼
┌─────────────────┐
│  Sensor Thread  │ ──────► sensor_data_queue
└─────────────────┘              │
                                 ▼
                        ┌──────────────────┐
                        │ Real-time Thread │ ──────► BLE IPC
                        └──────────────────┘         
                                 │
                                 ▼
                          realtime_queue
                                 ���
                                 ▼
                        ┌──────────────────┐
                        │ Analytics Thread │
                        └──────────────────┘
                                 │
                                 ▼
                          analytics_queue
                                 │
                                 ▼
                        ┌──────────────────┐
                        │  Session Thread  │ ──────► Flash Storage
                        └──────────────────┘
```

---

## Implementation Guidelines

### Thread Creation (Zephyr RTOS)

```c
// Thread stack definitions
K_THREAD_STACK_DEFINE(sensor_thread_stack, 4096);
K_THREAD_STACK_DEFINE(realtime_thread_stack, 8192);
K_THREAD_STACK_DEFINE(analytics_thread_stack, 12288);
K_THREAD_STACK_DEFINE(session_thread_stack, 4096);

// Thread data structures
struct k_thread sensor_thread_data;
struct k_thread realtime_thread_data;
struct k_thread analytics_thread_data;
struct k_thread session_thread_data;

// Thread creation
void init_activity_threads(void) {
    // Sensor thread - highest frequency
    k_thread_create(&sensor_thread_data, sensor_thread_stack,
                    K_THREAD_STACK_SIZEOF(sensor_thread_stack),
                    sensor_thread_entry, NULL, NULL, NULL,
                    SENSOR_THREAD_PRIORITY, 0, K_NO_WAIT);
    
    // Real-time thread - BLE updates
    k_thread_create(&realtime_thread_data, realtime_thread_stack,
                    K_THREAD_STACK_SIZEOF(realtime_thread_stack),
                    realtime_thread_entry, NULL, NULL, NULL,
                    REALTIME_THREAD_PRIORITY, 0, K_NO_WAIT);
    
    // Analytics thread - complex calculations
    k_thread_create(&analytics_thread_data, analytics_thread_stack,
                    K_THREAD_STACK_SIZEOF(analytics_thread_stack),
                    analytics_thread_entry, NULL, NULL, NULL,
                    ANALYTICS_THREAD_PRIORITY, 0, K_NO_WAIT);
    
    // Session thread - file I/O
    k_thread_create(&session_thread_data, session_thread_stack,
                    K_THREAD_STACK_SIZEOF(session_thread_stack),
                    session_thread_entry, NULL, NULL, NULL,
                    SESSION_THREAD_PRIORITY, 0, K_NO_WAIT);
}
```

### Priority Inversion Prevention

```c
// Use priority inheritance mutexes for shared resources
struct k_mutex sensor_data_mutex;
struct k_mutex session_data_mutex;

// Initialize with priority inheritance
k_mutex_init(&sensor_data_mutex);
k_mutex_init(&session_data_mutex);

// Usage example
k_mutex_lock(&sensor_data_mutex, K_FOREVER);
// Access shared data
k_mutex_unlock(&sensor_data_mutex);
```

### CPU Load Monitoring

```c
// Monitor thread execution times
typedef struct {
    uint32_t exec_count;
    uint32_t total_time_us;
    uint32_t max_time_us;
    uint32_t overruns;
} ThreadStats;

// Update in each thread
void update_thread_stats(ThreadStats* stats, uint32_t start_time) {
    uint32_t exec_time = k_cycle_get_32() - start_time;
    stats->exec_count++;
    stats->total_time_us += exec_time;
    if (exec_time > stats->max_time_us) {
        stats->max_time_us = exec_time;
    }
}
```

---

## Performance Considerations

### CPU Budget Analysis

With 128 MHz application core:
- Total CPU cycles per second: 128,000,000
- Available for application: ~100,000,000 (accounting for RTOS overhead)

#### Per-Thread Budget

1. **Sensor Thread (100Hz)**
   - Budget: 2ms × 100 = 200ms/sec = 20% CPU
   - Actual estimate: ~15% CPU

2. **Real-time Thread (20Hz avg)**
   - Budget: 5ms × 20 = 100ms/sec = 10% CPU
   - Actual estimate: ~8% CPU

3. **Analytics Thread (2Hz avg)**
   - Budget: 20ms × 2 = 40ms/sec = 4% CPU
   - Actual estimate: ~3% CPU

4. **Session Thread (1Hz avg)**
   - Budget: 50ms × 1 = 50ms/sec = 5% CPU
   - Actual estimate: ~3% CPU

**Total CPU Usage**: ~29% (plenty of headroom)

### Memory Usage Estimates

```
Thread Stacks:     28KB (4+8+12+4)
Message Queues:    ~8KB
Algorithm Buffers: ~16KB
BLE Buffers:       ~4KB
Session Data:      ~8KB
----------------------------
Total RAM:         ~64KB (out of 512KB available)
```

### Power Optimization Strategies

1. **Dynamic Thread Suspension**
   ```c
   // Suspend analytics when battery low
   if (battery_level < 20) {
       k_thread_suspend(&analytics_thread_data);
   }
   ```

2. **Adaptive Sampling**
   ```c
   // Reduce rates during low activity
   if (activity_level == STATIONARY) {
       sensor_period_ms = 100;  // 10Hz instead of 100Hz
   }
   ```

3. **Batch Processing**
   ```c
   // Process multiple samples together
   if (k_msgq_num_used_get(&sensor_data_queue) >= 10) {
       // Process batch
   }
   ```

---

## Alternative Approaches

### 1. Thread Pool Architecture

Instead of dedicated threads, use 2-3 worker threads with priority queue:

```c
typedef struct {
    uint8_t priority;
    void (*task_func)(void* data);
    void* data;
    uint32_t deadline_ms;
} Task;

// Workers pull highest priority tasks
void worker_thread(void* p1, void* p2, void* p3) {
    Task task;
    while (1) {
        if (get_highest_priority_task(&task) == 0) {
            task.task_func(task.data);
        }
    }
}
```

**Pros**: Fewer threads, dynamic load balancing  
**Cons**: More complex scheduling, harder to guarantee deadlines

### 2. Single Thread with Time Slicing

Run all algorithms in one thread with strict time budgets:

```c
void unified_processing_thread(void) {
    while (1) {
        uint32_t cycle_start = k_uptime_get_32();
        
        // Phase 1: Sensor (2ms budget)
        process_sensors();
        
        // Phase 2: Real-time (5ms budget)
        if (k_uptime_get_32() - cycle_start < 2) {
            calculate_realtime_metrics();
        }
        
        // Phase 3: Analytics (if time remains)
        if (k_uptime_get_32() - cycle_start < 7) {
            run_analytics();
        }
        
        // Sleep until next cycle
        k_sleep(K_MSEC(10));
    }
}
```

**Pros**: Simple, predictable  
**Cons**: Complex algorithms may not complete, less parallelism

### 3. Cooperative Multitasking

Use Zephyr's cooperative threads with explicit yield points:

```c
void cooperative_analytics(void) {
    calculate_efficiency();
    k_yield();  // Let other threads run
    
    calculate_fatigue();
    k_yield();
    
    assess_injury_risk();
    k_yield();
}
```

**Pros**: Predictable execution order  
**Cons**: Requires careful yield placement, one blocked thread blocks all

---

## Conclusion

The recommended four-thread architecture provides the best balance of:

1. **Real-time Performance**: Guaranteed BLE updates at 1Hz
2. **Computational Capability**: Complex algorithms don't block simple ones
3. **Maintainability**: Clear separation of concerns
4. **Scalability**: Easy to add/remove algorithms
5. **Power Efficiency**: Threads can be suspended independently

Given that the nRF5340's network core handles all Bluetooth operations independently, the application core is free to focus entirely on sensor processing and calculations. The multi-threaded approach ensures that the varying computational requirements (from <1ms to >50ms) don't interfere with each other or compromise real-time performance.

### Key Success Factors

1. **Use BHI360's step count** - Don't reimplement step detection
2. **Respect thread priorities** - Real-time metrics must not be delayed
3. **Monitor CPU usage** - Stay within budgets
4. **Test under load** - Verify performance with all algorithms active
5. **Plan for degradation** - System should handle CPU overload gracefully

This architecture will successfully implement the comprehensive activity metrics system while maintaining the real-time performance required for a responsive user experience.

---

## Detailed Implementation Examples

### Sensor Thread Implementation

```c
// Sensor thread main loop
void sensor_thread_entry(void *p1, void *p2, void *p3) {
    SensorDataMsg sensor_msg;
    ContactDetector left_contact = {0};
    ContactDetector right_contact = {0};
    ThreadStats stats = {0};
    
    // Initialize sensors
    init_bhi360();
    init_pressure_sensors();
    
    while (1) {
        uint32_t start_time = k_cycle_get_32();
        
        // Read all sensors
        sensor_msg.timestamp = k_uptime_get_32();
        
        // Get BHI360 data
        bhi360_get_quaternion(sensor_msg.quaternion);
        bhi360_get_linear_accel(sensor_msg.linear_acc);
        sensor_msg.step_count = bhi360_get_step_count();
        
        // Read pressure sensors
        read_pressure_sensors(sensor_msg.pressure);
        
        // Detect ground contact (critical for other calculations)
        float left_force = sensor_msg.pressure[0] + sensor_msg.pressure[1] + 
                          sensor_msg.pressure[2] + sensor_msg.pressure[3];
        float right_force = sensor_msg.pressure[4] + sensor_msg.pressure[5] + 
                           sensor_msg.pressure[6] + sensor_msg.pressure[7];
        
        // Update contact states
        bool left_contact_changed = detect_ground_contact(&left_contact, 
                                                         left_force, 
                                                         sensor_msg.timestamp);
        bool right_contact_changed = detect_ground_contact(&right_contact, 
                                                          right_force, 
                                                          sensor_msg.timestamp);
        
        // Send to real-time thread
        if (k_msgq_put(&sensor_data_queue, &sensor_msg, K_NO_WAIT) != 0) {
            stats.overruns++;
            LOG_WRN("Sensor queue full, dropping sample");
        }
        
        // Send contact events immediately
        if (left_contact_changed || right_contact_changed) {
            ContactEventMsg contact_event = {
                .timestamp = sensor_msg.timestamp,
                .left_contact = left_contact.in_contact,
                .right_contact = right_contact.in_contact,
                .left_contact_time = left_contact.contact_end_time - 
                                    left_contact.contact_start_time,
                .right_contact_time = right_contact.contact_end_time - 
                                     right_contact.contact_start_time
            };
            k_msgq_put(&contact_event_queue, &contact_event, K_NO_WAIT);
        }
        
        // Update statistics
        update_thread_stats(&stats, start_time);
        
        // Sleep until next sample time
        k_sleep(K_MSEC(10));  // 100Hz
    }
}
```

### Real-Time Thread Implementation

```c
// Real-time metrics thread
void realtime_thread_entry(void *p1, void *p2, void *p3) {
    RealtimeMetricsMsg metrics = {0};
    CadenceTracker cadence_tracker = {0};
    PaceCalculator pace_calc = {0};
    FormAnalyzer form_analyzer = {0};
    AlertGenerator alert_gen = {0};
    
    uint32_t last_ble_update = 0;
    const uint32_t BLE_UPDATE_INTERVAL_MS = 1000;  // 1Hz
    
    while (1) {
        SensorDataMsg sensor_data;
        
        // Wait for sensor data (with timeout)
        if (k_msgq_get(&sensor_data_queue, &sensor_data, K_MSEC(50)) == 0) {
            // Calculate cadence from BHI360 step count
            float current_cadence = calculate_cadence_from_bhi360(
                &cadence_tracker, 
                sensor_data.step_count,
                sensor_data.timestamp
            );
            
            // Calculate pace using multiple inputs
            float contact_time = get_avg_contact_time();
            float flight_time = get_avg_flight_time();
            float pace_sec_km = calculate_pace_with_sensors(
                current_cadence,
                contact_time,
                flight_time,
                user_profile.height_cm,
                get_vertical_oscillation()
            );
            
            // Calculate form score
            float form_score = calculate_form_score(
                &form_analyzer,
                sensor_data.quaternion,
                sensor_data.pressure,
                contact_time,
                current_cadence
            );
            
            // Calculate balance
            int8_t balance = calculate_lr_balance(
                sensor_data.pressure
            );
            
            // Check for alerts
            check_and_generate_alerts(
                &alert_gen,
                form_score,
                balance,
                sensor_data.linear_acc,
                sensor_data.timestamp
            );
            
            // Update metrics
            metrics.cadence_x2 = (uint16_t)(current_cadence * 2);
            metrics.pace_sec_per_km = (uint16_t)pace_sec_km;
            metrics.form_score = (uint8_t)form_score;
            metrics.balance_lr = balance;
            
            // Send to analytics thread
            k_msgq_put(&realtime_queue, &metrics, K_NO_WAIT);
        }
        
        // Send BLE update at 1Hz
        uint32_t now = k_uptime_get_32();
        if (now - last_ble_update >= BLE_UPDATE_INTERVAL_MS) {
            send_ble_metrics(&metrics);
            last_ble_update = now;
        }
    }
}
```

### Analytics Thread Implementation

```c
// Complex analytics thread
void analytics_thread_entry(void *p1, void *p2, void *p3) {
    FatigueAnalyzer fatigue = {0};
    EfficiencyCalculator efficiency = {0};
    InjuryRiskAssessor injury_risk = {0};
    StrideAnalyzer stride = {0};
    
    // Baseline establishment phase
    bool baseline_complete = false;
    uint32_t activity_start_time = k_uptime_get_32();
    
    while (1) {
        RealtimeMetricsMsg metrics;
        
        // Get real-time metrics
        if (k_msgq_get(&realtime_queue, &metrics, K_MSEC(200)) == 0) {
            uint32_t current_time = k_uptime_get_32();
            
            // Establish baseline (first 2 minutes)
            if (!baseline_complete && 
                (current_time - activity_start_time) < 120000) {
                update_baseline(&fatigue, &metrics);
                update_baseline(&efficiency, &metrics);
                continue;
            } else if (!baseline_complete) {
                finalize_baseline(&fatigue);
                finalize_baseline(&efficiency);
                baseline_complete = true;
            }
            
            // Calculate complex metrics
            AnalyticsResultMsg results = {0};
            
            // Running efficiency (multi-factor calculation)
            results.efficiency_score = calculate_running_efficiency(
                &efficiency,
                metrics.cadence_x2 / 2.0,
                get_duty_factor(),
                get_vertical_oscillation(),
                get_forward_lean()
            );
            
            // Fatigue index (requires baseline comparison)
            results.fatigue_index = calculate_fatigue_index(
                &fatigue,
                get_avg_contact_time(),
                get_avg_flight_time(),
                get_loading_rate(),
                metrics.form_score,
                current_time
            );
            
            // Injury risk assessment (composite score)
            results.injury_risk = assess_injury_risk(
                &injury_risk,
                get_loading_rate(),
                metrics.balance_lr,
                get_pronation_angle(),
                get_strike_pattern(),
                results.fatigue_index
            );
            
            // Stride length estimation
            results.stride_length = estimate_stride_length(
                &stride,
                metrics.cadence_x2 / 2.0,
                get_duty_factor(),
                user_profile.height_cm,
                get_push_off_power()
            );
            
            // Send to session thread
            k_msgq_put(&analytics_queue, &results, K_NO_WAIT);
            
            // Log if concerning values
            if (results.injury_risk > 70) {
                LOG_WRN("High injury risk detected: %d", results.injury_risk);
            }
        }
    }
}
```

### Session Thread Implementation

```c
// Session management and logging thread
void session_thread_entry(void *p1, void *p2, void *p3) {
    SessionHeader header = {0};
    SessionSummary summary = {0};
    PeriodicRecord record = {0};
    
    uint32_t last_record_time = 0;
    uint32_t record_count = 0;
    bool session_active = false;
    
    // Open log file
    struct fs_file_t log_file;
    fs_file_t_init(&log_file);
    
    while (1) {
        // Check for session commands
        SessionCommand cmd;
        if (k_msgq_get(&session_cmd_queue, &cmd, K_NO_WAIT) == 0) {
            switch (cmd.type) {
                case SESSION_START:
                    start_new_session(&header, &log_file, cmd.session_id);
                    session_active = true;
                    last_record_time = k_uptime_get_32();
                    break;
                    
                case SESSION_STOP:
                    if (session_active) {
                        finalize_session(&summary, &log_file);
                        session_active = false;
                    }
                    break;
                    
                case GPS_UPDATE:
                    process_gps_calibration(&cmd.gps_data);
                    break;
            }
        }
        
        // Process analytics results
        if (session_active) {
            AnalyticsResultMsg analytics;
            if (k_msgq_get(&analytics_queue, &analytics, K_MSEC(100)) == 0) {
                uint32_t now = k_uptime_get_32();
                
                // Update periodic record
                record.delta_time_ms = now - last_record_time;
                record.efficiency_score = (uint8_t)analytics.efficiency_score;
                record.fatigue_level = (uint8_t)analytics.fatigue_index;
                
                // Get latest metrics
                get_current_metrics(&record);
                
                // Write record every 1-2 seconds
                if (now - last_record_time >= 1000) {
                    write_periodic_record(&log_file, &record);
                    last_record_time = now;
                    record_count++;
                    
                    // Update session statistics
                    update_session_stats(&summary, &record);
                }
            }
        }
        
        // Low priority - can sleep longer
        k_sleep(K_MSEC(100));
    }
}
```

---

## Error Handling and Recovery

### Queue Overflow Handling

```c
// Adaptive queue management
typedef struct {
    uint32_t drops;
    uint32_t last_drop_time;
    bool degraded_mode;
} QueueManager;

void handle_queue_overflow(QueueManager* mgr, const char* queue_name) {
    mgr->drops++;
    uint32_t now = k_uptime_get_32();
    
    // If dropping frequently, enter degraded mode
    if (mgr->drops > 10 && (now - mgr->last_drop_time) < 1000) {
        LOG_ERR("%s queue overflowing, entering degraded mode", queue_name);
        mgr->degraded_mode = true;
        
        // Reduce sampling rates
        reduce_sensor_rate();
        
        // Skip non-essential calculations
        disable_analytics();
    }
    
    mgr->last_drop_time = now;
}
```

### Thread Watchdog

```c
// Thread health monitoring
typedef struct {
    uint32_t last_heartbeat;
    uint32_t timeout_ms;
    bool is_healthy;
} ThreadHealth;

ThreadHealth thread_health[4] = {
    [0] = {.timeout_ms = 100},   // Sensor thread
    [1] = {.timeout_ms = 2000},  // Real-time thread
    [2] = {.timeout_ms = 5000},  // Analytics thread
    [3] = {.timeout_ms = 10000}  // Session thread
};

void watchdog_thread(void) {
    while (1) {
        uint32_t now = k_uptime_get_32();
        
        for (int i = 0; i < 4; i++) {
            if (now - thread_health[i].last_heartbeat > 
                thread_health[i].timeout_ms) {
                
                if (thread_health[i].is_healthy) {
                    LOG_ERR("Thread %d appears stuck", i);
                    thread_health[i].is_healthy = false;
                    
                    // Take recovery action
                    recover_stuck_thread(i);
                }
            }
        }
        
        k_sleep(K_MSEC(1000));
    }
}
```

---

## Testing and Validation

### Load Testing

```c
// Synthetic load generator for testing
void generate_test_load(void) {
    // Simulate worst-case scenario
    for (int i = 0; i < 100; i++) {
        SensorDataMsg msg = {
            .timestamp = k_uptime_get_32(),
            .step_count = i * 2,
            // Fill with test data
        };
        
        k_msgq_put(&sensor_data_queue, &msg, K_NO_WAIT);
        k_sleep(K_MSEC(10));  // 100Hz
    }
}

// Measure thread performance
void benchmark_thread_performance(void) {
    uint32_t start = k_cycle_get_32();
    
    // Run algorithm
    calculate_running_efficiency(...);
    
    uint32_t cycles = k_cycle_get_32() - start;
    uint32_t us = k_cyc_to_us_floor32(cycles);
    
    LOG_INF("Efficiency calculation took %u us", us);
}
```

### Integration Testing

```c
// End-to-end test
void test_full_pipeline(void) {
    // Start all threads
    init_activity_threads();
    
    // Inject test data
    inject_test_sensor_data();
    
    // Wait for processing
    k_sleep(K_SECONDS(5));
    
    // Verify outputs
    assert(ble_metrics_received > 0);
    assert(analytics_completed > 0);
    assert(records_written > 0);
    
    LOG_INF("Pipeline test passed");
}
```

---

## Migration Path

### From Single Thread to Multi-Thread

If currently using a single thread, here's a migration strategy:

1. **Phase 1**: Extract sensor reading
   - Move sensor reading to dedicated thread
   - Keep all calculations in main thread

2. **Phase 2**: Separate real-time metrics
   - Move BLE updates to real-time thread
   - Use simple message passing

3. **Phase 3**: Offload analytics
   - Move complex calculations to analytics thread
   - Add queue management

4. **Phase 4**: Isolate file I/O
   - Move all file operations to session thread
   - Complete separation of concerns

### Rollback Strategy

```c
// Feature flag for thread architecture
#if CONFIG_MULTI_THREAD_METRICS
    init_activity_threads();
#else
    init_single_thread_metrics();
#endif
```

---

## Future Enhancements

### Machine Learning Integration

When ML algorithms are added:

1. **Dedicated ML Thread**: Priority 8, between real-time and analytics
2. **GPU/NPU Offload**: If available on future hardware
3. **Model Updates**: Handle via session thread during idle

### Multi-Core Optimization

If future nRF chips allow app core access to network core:

1. **Move Analytics**: Run heavy calculations on network core
2. **Parallel Processing**: Split algorithms across cores
3. **Shared Memory**: Use IPC for large data transfers

---

## Summary

This threading architecture ensures:

1. **Reliable sensor sampling** at 100Hz
2. **Consistent BLE updates** at 1Hz
3. **Complex analytics** without blocking real-time
4. **Efficient file I/O** without impacting performance
5. **Graceful degradation** under high load

The architecture is designed to scale with the comprehensive requirements of the Activity Session Calculated Data Specification while working within the constraints of the nRF5340's single application core.
# Multi-Thread Work Queue Architecture Guide

**Date:** June 2025  
**Version:** 1.0  
**Status:** Architecture Defined - Ready for Implementation

---

## Table of Contents
1. [Executive Summary](#executive-summary)
2. [Architecture Overview](#architecture-overview)
3. [Work Queue Design Patterns](#work-queue-design-patterns)
4. [Data Aggregation Strategies](#data-aggregation-strategies)
5. [Implementation Status](#implementation-status)
6. [Performance Considerations](#performance-considerations)
7. [🚀 NEXT DEVELOPMENT STAGES](#-next-development-stages)

---

## Executive Summary

This document describes the work queue architecture for the multi-thread activity metrics system. Each of the 4 modules (sensor_data, realtime_metrics, analytics, activity_metrics) has:
- A dedicated thread that waits for messages
- A dedicated work queue for processing
- Ring buffers for data storage
- Multiple work items for different processing tasks

---

## Architecture Overview

### Thread + Work Queue Pattern

```
┌─────────────────────────────────────────────────────────────┐
│                     Module Structure                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Message Queue ──► Thread (waits) ──► Ring Buffer          │
│                         │                  │                │
│                         └──► Queue Work    │                │
│                                   │        │                │
│                                   ▼        ▼                │
│                            Work Queue Thread                │
│                                   │                         │
│                                   ├──► Work Handler 1       │
│                                   ├──► Work Handler 2       │
│                                   └──► Work Handler N       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Key Components

1. **Message Thread**: Waits for messages with `K_FOREVER`
2. **Ring Buffers**: Store data by type (foot, IMU, commands)
3. **Work Queue**: Processes work items asynchronously
4. **Work Items**: Specific handlers for each data type

---

## Work Queue Design Patterns

### Pattern 1: Ring Buffer Approach (Currently Implemented)

```c
// Thread receives message and stores in ring buffer
static void sensor_data_thread_fn(void *arg1, void *arg2, void *arg3)
{
    generic_message_t msg;
    while (true) {
        k_msgq_get(&sensor_data_msgq, &msg, K_FOREVER);
        
        if (msg.type == MSG_TYPE_FOOT_SAMPLES) {
            // Store in ring buffer
            foot_ring_put(&foot_ring, &msg.data.foot_samples, foot_id);
            
            // Queue work (only if not pending)
            if (!k_work_is_pending(&process_foot_data_work)) {
                k_work_submit_to_queue(&sensor_data_work_q, &process_foot_data_work);
            }
        }
    }
}

// Work handler reads from ring buffer
static void process_foot_data_work_handler(struct k_work *work)
{
    foot_samples_t foot_data;
    uint8_t foot_id;
    
    // Process all available data
    while (foot_ring_get(&foot_ring, &foot_data, &foot_id)) {
        // Process foot_data
    }
}
```

**Advantages:**
- Handles data bursts well
- Natural buffering for high-frequency data
- Easy statistics tracking
- Decouples data storage from work scheduling

### Pattern 2: Container/Parameter Approach (Alternative)

```c
// Define work item with embedded data
struct foot_data_work_item {
    struct k_work work;
    foot_samples_t data;
    uint8_t foot_id;
};

// Work handler uses CONTAINER_OF
static void process_foot_data_handler(struct k_work *work)
{
    struct foot_data_work_item *item = 
        CONTAINER_OF(work, struct foot_data_work_item, work);
    
    // Direct access to item->data and item->foot_id
}
```

**Advantages:**
- Direct parameter passing
- No extra copies
- More intuitive for simple cases

**Disadvantages:**
- Need work item pool management
- Can exhaust work items during bursts

---

## Data Aggregation Strategies

### Aggregating Multiple Data Types

The sensor_data module shows how to aggregate foot sensor + IMU data:

```c
static void process_sensor_sample(void)
{
    // Access multiple ring buffers
    foot_samples_t left_foot, right_foot;
    bhi360_log_record_t imu_data;
    uint8_t foot_id;
    
    // Get latest foot data
    if (foot_ring_get(&foot_ring, &left_foot, &foot_id) && foot_id == 0) {
        // Process left foot
    }
    if (foot_ring_get(&foot_ring, &right_foot, &foot_id) && foot_id == 1) {
        // Process right foot
    }
    
    // Get latest IMU data
    if (imu_ring_get(&imu_ring, &imu_data)) {
        // Process IMU
    }
    
    // Now aggregate all data
    // - Combine pressure + quaternion for pronation
    // - Use IMU + pressure for ground contact validation
    // - Calculate consolidated metrics
}
```

### Ring Buffer Sizes

```c
// From sensor_data_ring_buffer.h
#define FOOT_DATA_RING_SIZE 8      // 8 samples buffer
#define IMU_DATA_RING_SIZE 8       // 8 samples buffer
#define COMMAND_RING_SIZE 4        // 4 commands buffer
```

---

## Implementation Status

### ✅ Completed
- Module structure with threads + work queues
- Ring buffer implementation
- Work item definitions
- Message routing
- Basic work handlers

### 🔴 Not Implemented
- Actual sensor processing algorithms
- Data aggregation logic
- Metric calculations
- Inter-module communication

---

## Performance Considerations

### 100Hz Processing Requirements

For the sensor_data module running at 100Hz:
- **Period**: 10ms between samples
- **Target Processing Time**: < 2ms per work item
- **Safety Margin**: 5x headroom

### Work Queue Sizing

```c
// Work queue stack size
static constexpr int sensor_data_workq_stack_size = 2048;  // 2KB

// Work queue priority (slightly lower than main thread)
k_work_queue_start(&sensor_data_work_q, 
                   sensor_data_workq_stack,
                   K_THREAD_STACK_SIZEOF(sensor_data_workq_stack),
                   sensor_data_priority - 1,  // Priority 5 (main thread is 6)
                   NULL);
```

### Preventing Saturation

1. **Check Pending Status**: Only queue if not already pending
2. **Ring Buffer Monitoring**: Track max depth in statistics
3. **Drop Policy**: Log and count dropped samples
4. **Processing Time**: Keep under 2ms for 100Hz operation

---

# 🚀 NEXT DEVELOPMENT STAGES

## 🎯 Priority 1: Implement Sensor Data Processing (Week 1)

### 1.1 Update sensor_data Module
**File:** `src/sensor_data/sensor_data.cpp`

```c
// TODO: Implement in process_foot_data_work_handler
- Extract pressure values from all 16 channels
- Call detect_ground_contact() from fast_processing.h
- Call detect_peak_force() 
- Store results in consolidated structure

// TODO: Implement in process_imu_data_work_handler
- Extract quaternion, linear_acc, gyro
- Call quick_pronation_check()
- Update step count tracking
- Store IMU state

// TODO: Implement in process_sensor_sample (100Hz)
- Consolidate foot + IMU data
- Create consolidated message with:
  - Ground contact state (both feet)
  - Peak forces
  - Contact phase
  - Pronation angle
  - Step count delta
- Send to sensor_data_queue
```

### 1.2 Create Fast Processing Algorithms
**File:** `src/sensor_data/sensor_data_fast_processing.h`

```c
// Implement these functions (target: <0.1ms each)
bool detect_ground_contact(uint16_t pressure[8]);
uint16_t detect_peak_force(uint16_t pressure[8]);
contact_phase_t detect_contact_phase(uint16_t pressure[8], bool was_in_contact);
int8_t quick_pronation_check(float quaternion[4]);
```

## 🎯 Priority 2: Implement Realtime Metrics (Week 1-2)

### 2.1 Update realtime_metrics Module
**File:** `src/realtime_metrics/realtime_metrics.cpp`

```c
// TODO: Implement in process_sensor_data_work_handler
- Receive consolidated sensor data
- Calculate cadence from step count changes
- Calculate pace from cadence + stride length
- Calculate form score
- Calculate L/R balance

// TODO: Implement in ble_update_work_handler (1Hz)
- Package metrics for BLE
- Send to bluetooth_msgq
```

## 🎯 Priority 3: Implement Analytics (Week 2)

### 3.1 Update analytics Module
**File:** `src/analytics/analytics.cpp`

```c
// TODO: Implement baseline establishment (first 2 min)
- Collect baseline metrics
- Calculate averages
- Set baseline_established flag

// TODO: Implement complex calculations
- Running efficiency
- Fatigue index
- Injury risk assessment
- CPEI calculation
```

## 🎯 Priority 4: Integration Testing (Week 2-3)

### 4.1 Update Sensor Modules
```c
// motion_sensor.cpp - Already updated ✅
// foot_sensor.cpp - Already updated ✅
```

### 4.2 Create Integration Tests
```bash
# Test data flow
foot_sensor → sensor_data → realtime_metrics → bluetooth
motion_sensor → sensor_data → analytics → activity_metrics
```

### 4.3 Performance Validation
- Measure actual processing times
- Verify 100Hz timing
- Check ring buffer depths
- Monitor CPU usage

## 🎯 Priority 5: Refactor activity_metrics (Week 3)

### 5.1 Focus on Session Management
```c
// Remove sensor processing code
// Keep only:
- Session start/stop/pause
- Periodic record creation
- GPS integration
- File logging via data module
```

---

## Development Checklist

### Week 1 Tasks
- [ ] Implement sensor_data processing algorithms
- [ ] Test ground contact detection
- [ ] Test peak force calculation
- [ ] Implement cadence calculation
- [ ] Test with real sensor data

### Week 2 Tasks
- [ ] Implement form score calculation
- [ ] Implement balance calculation
- [ ] Implement fatigue detection
- [ ] Implement injury risk assessment
- [ ] Integration testing

### Week 3 Tasks
- [ ] Performance optimization
- [ ] Refactor activity_metrics
- [ ] Full system testing
- [ ] Documentation update
- [ ] Code review

---

## Quick Reference

### Message Flow
```
foot_sensor ──┐
              ├──► sensor_data ──► realtime_metrics ──► bluetooth
motion_sensor ┘                └──► analytics ──► activity_metrics
```

### Processing Rates
- **sensor_data**: 100Hz (every 10ms)
- **realtime_metrics**: 10-50Hz (adaptive)
- **analytics**: 1-5Hz (adaptive)
- **activity_metrics**: 0.5Hz (every 2 seconds)

### Key Files to Edit
1. `src/sensor_data/sensor_data.cpp` - Main processing
2. `src/sensor_data/sensor_data_fast_processing.h` - Algorithms
3. `src/realtime_metrics/realtime_metrics.cpp` - Metrics
4. `src/analytics/analytics.cpp` - Complex analysis

---

## Conclusion

The architecture is ready for implementation. Start with Priority 1 tasks in the sensor_data module, implementing the fast processing algorithms first. The ring buffer approach provides good buffering for the 100Hz data rate while keeping the implementation clean and testable.
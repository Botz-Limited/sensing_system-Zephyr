# Activity Metrics Work Queue Architecture

**Date:** June 2025  
**Version:** 1.0  
**Status:** Implementation Guide

---

## Overview

This document explains the work queue architecture implemented in the activity metrics modules, including how data is passed between threads and work handlers, and critical timing considerations.

---

## Data Passing Architecture

### How Work Handlers Receive Data

Each module uses **module-level static buffers** to pass data from threads to work handlers. This is necessary because work handlers in Zephyr only receive a `struct k_work *` parameter.

### Example: sensor_data Module

```c
// Module-level buffers (static)
static foot_samples_t pending_foot_data;
static bhi360_log_record_t pending_imu_data;
static char pending_command[MAX_COMMAND_STRING_LEN];
static uint8_t pending_foot_id;

// Thread receives message and copies data
static void sensor_data_thread_fn(void *arg1, void *arg2, void *arg3)
{
    generic_message_t msg;
    while (true) {
        k_msgq_get(&sensor_data_msgq, &msg, K_FOREVER);
        
        switch (msg.type) {
            case MSG_TYPE_FOOT_SAMPLES:
                // Copy foot sensor data to buffer
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
                
                // Queue the work
                k_work_submit_to_queue(&sensor_data_work_q, &process_foot_data_work);
                break;
        }
    }
}

// Work handler accesses the buffer
static void process_foot_data_work_handler(struct k_work *work)
{
    // Data is available in module-level buffers
    LOG_DBG("Processing foot %d", pending_foot_id);
    
    // Access the 8 pressure sensors
    for (int i = 0; i < 8; i++) {
        uint16_t pressure = pending_foot_data.samples[i];
        // Process pressure value...
    }
}
```

### Data Flow Sequence

1. **Sensor Module** sends message → Message Queue
2. **Thread** receives message → Copies to module buffer
3. **Thread** queues work → Work item submitted to work queue
4. **Work Handler** executes → Reads from module buffer
5. **Work Handler** processes → Sends results to next module

### Buffer Management Patterns

#### Current Implementation (Single Buffer)
- **Pros**: Simple, low memory usage
- **Cons**: Only one pending message per type
- **Risk**: Data loss if messages arrive faster than processing

#### Recommended Improvements

1. **Ring Buffer Pattern**
```c
#define FOOT_DATA_RING_SIZE 4
static foot_samples_t foot_data_ring[FOOT_DATA_RING_SIZE];
static uint8_t foot_data_write_idx = 0;
static uint8_t foot_data_read_idx = 0;
```

2. **Memory Pool Pattern**
```c
K_MEM_POOL_DEFINE(sensor_data_pool, 64, 256, 10, 4);
```

3. **Queue Depth Monitoring**
```c
static atomic_t pending_work_count = ATOMIC_INIT(0);
if (atomic_get(&pending_work_count) > MAX_PENDING_WORK) {
    LOG_WRN("Work queue backlog detected");
    // Skip or handle overflow
}
```

---

## Work Queue Timing Analysis

### 100Hz Thread (sensor_data) Critical Timing

The sensor_data module is the most timing-critical as it must process data at 100Hz without falling behind.

#### Timing Budget

| Parameter | Value | Notes |
|-----------|-------|-------|
| Update Rate | 100Hz | Fixed requirement |
| Period | 10ms | Time between samples |
| Max Processing Time | < 10ms | Absolute maximum |
| Target Processing Time | < 5ms | 50% safety margin |
| Recommended Target | < 2ms | For critical path only |

#### Processing Time Breakdown

```
Typical sensor_data work item processing:
├── Foot data processing:     0.5ms
├── IMU data processing:      0.5ms
├── Data consolidation:       0.5ms
├── Message distribution:     0.5ms
└── Total:                    2.0ms (20% of budget)
```

#### Queue Saturation Analysis

**What happens if processing takes too long:**

| Processing Time | Queue Growth Rate | Time to 1000 Items | Result |
|----------------|-------------------|-------------------|---------|
| 5ms | 0 items/sec | Never | Stable ✅ |
| 10ms | 0 items/sec | Never | Borderline ⚠️ |
| 15ms | 50 items/sec | 20 seconds | Unstable ❌ |
| 20ms | 100 items/sec | 10 seconds | Critical ❌ |

**Memory Impact:**
- Each queued work item: ~64 bytes
- At 100 items/sec: 6.4KB/sec memory consumption
- System crash inevitable without mitigation

#### Mitigation Strategies

1. **Keep Processing Minimal**
   ```c
   // Good: Simple operations only
   sensor_state.left_foot_contact = (pressure > THRESHOLD);
   
   // Bad: Complex calculations in 100Hz thread
   calculate_complex_biomechanics(); // Move to analytics thread
   ```

2. **Use Conditional Processing**
   ```c
   if (sensor_state.sample_count % 10 == 0) {
       // Do expensive operation only every 100ms
   }
   ```

3. **Implement Overload Protection**
   ```c
   static atomic_t work_pending = ATOMIC_INIT(0);
   
   if (atomic_inc(&work_pending) > MAX_PENDING) {
       atomic_dec(&work_pending);
       LOG_WRN("Dropping sample due to overload");
       return;
   }
   ```

4. **Monitor Performance**
   ```c
   uint32_t start = k_cycle_get_32();
   process_sensor_sample();
   uint32_t duration = k_cyc_to_us_floor32(k_cycle_get_32() - start);
   
   if (duration > 5000) { // > 5ms
       LOG_WRN("Processing took %u us", duration);
   }
   ```

### Other Thread Timing Requirements

| Module | Frequency | Period | Max Time | Target | Work Queue Priority |
|--------|-----------|--------|----------|---------|-------------------|
| realtime_metrics | 10-50Hz | 20-100ms | 50ms | 25ms | 6 |
| analytics | 1-5Hz | 200-1000ms | 500ms | 250ms | 11 |
| activity_metrics | 0.5-2Hz | 500-2000ms | 1000ms | 500ms | 13 |

---

## Memory Analysis

### Work Queue Memory

| Component | Size | Count | Total |
|-----------|------|-------|-------|
| struct k_work | 32B | ~20 | 640B |
| Queue node overhead | 32B | ~20 | 640B |
| Work queue stacks | 2KB | 4 | 8KB |
| **Total Work Queue** | | | **~9.3KB** |

### Message Buffers

| Module | Buffer Type | Size | Count | Total |
|--------|-------------|------|-------|-------|
| sensor_data | foot_samples_t | 16B | 1 | 16B |
| | bhi360_log_record_t | 64B | 1 | 64B |
| | command | 64B | 1 | 64B |
| realtime_metrics | sensor_data | 128B | 1 | 128B |
| | command | 64B | 1 | 64B |
| analytics | metrics | 64B | 1 | 64B |
| | command | 64B | 1 | 64B |
| activity_metrics | various | 512B | 1 | 512B |
| **Total Buffers** | | | | **~1KB** |

### Message Queues

| Queue | Message Size | Depth | Total |
|-------|--------------|-------|-------|
| sensor_data_msgq | 256B | 50 | 12.5KB |
| sensor_data_queue | 256B | 50 | 12.5KB |
| realtime_queue | 256B | 20 | 5KB |
| analytics_queue | 256B | 10 | 2.5KB |
| **Total Queues** | | | **32.5KB** |

### Total Memory Usage

| Category | Size |
|----------|------|
| Work Queues | 9.3KB |
| Message Buffers | 1KB |
| Message Queues | 32.5KB |
| Thread Stacks | 28KB |
| **Total** | **~71KB** |

---

## Best Practices

### 1. Work Item Design

```c
// Good: Fast, focused work items
static void process_foot_data_work_handler(struct k_work *work)
{
    // Quick processing only
    update_contact_state();
    calculate_peak_force();
    
    // Defer complex work
    if (need_complex_analysis) {
        send_to_analytics_thread();
    }
}
```

### 2. Buffer Management

```c
// Good: Check for pending work before overwriting
static atomic_t foot_data_pending = ATOMIC_INIT(0);

// In thread:
if (atomic_get(&foot_data_pending) == 0) {
    memcpy(&pending_foot_data, &msg.data.foot_samples, sizeof(foot_samples_t));
    atomic_set(&foot_data_pending, 1);
    k_work_submit_to_queue(&sensor_data_work_q, &process_foot_data_work);
} else {
    LOG_WRN("Dropping foot data - previous still pending");
}

// In work handler:
atomic_set(&foot_data_pending, 0);
```

### 3. Timing Verification

```c
// Add timing checks during development
#ifdef CONFIG_TIMING_FUNCTIONS
static void log_processing_time(const char *name, uint32_t start_cycles)
{
    uint32_t duration_us = k_cyc_to_us_floor32(k_cycle_get_32() - start_cycles);
    if (duration_us > WARNING_THRESHOLD_US) {
        LOG_WRN("%s took %u us", name, duration_us);
    }
}
#endif
```

### 4. Priority Management

- Work queue priority = Thread priority - 1
- Ensures thread can always receive messages
- Prevents priority inversion

---

## Common Issues and Solutions

### Issue 1: Work Queue Backlog

**Symptom**: Increasing memory usage, delayed processing

**Solution**:
```c
// Monitor queue depth
if (k_work_pending(&process_foot_data_work)) {
    stats.dropped_samples++;
    return; // Skip this sample
}
```

### Issue 2: Data Corruption

**Symptom**: Incorrect values in work handlers

**Solution**: Ensure complete copy before queuing work
```c
// Always use memcpy for structures
memcpy(&pending_data, &msg.data, sizeof(data_type));
// Never pass pointers to message data
```

### Issue 3: Timing Drift

**Symptom**: Gradual shift in 100Hz timing

**Solution**: Use absolute timing
```c
static int64_t next_sample_time = 0;

if (next_sample_time == 0) {
    next_sample_time = k_uptime_get() + 10;
}

k_work_schedule_for_queue(&sensor_data_work_q, 
                         &periodic_sample_work, 
                         K_TIMEOUT_ABS_MS(next_sample_time));

next_sample_time += 10; // Always 10ms intervals
```

---

## Testing Recommendations

### 1. Timing Tests

```c
// Add test mode to measure processing times
void test_processing_performance(void)
{
    uint32_t start = k_cycle_get_32();
    
    for (int i = 0; i < 1000; i++) {
        process_sensor_sample();
    }
    
    uint32_t total_us = k_cyc_to_us_floor32(k_cycle_get_32() - start);
    uint32_t avg_us = total_us / 1000;
    
    LOG_INF("Average processing time: %u us", avg_us);
    zassert_true(avg_us < 5000, "Processing too slow");
}
```

### 2. Stress Tests

- Send messages at 200Hz to test overload handling
- Verify no memory leaks under sustained load
- Check queue overflow behavior

### 3. Integration Tests

- Verify end-to-end data flow timing
- Measure latency from sensor to BLE
- Validate all calculations under load

---

## Conclusion

The work queue architecture provides:
- **Decoupling** between message reception and processing
- **Flexibility** in handling different message types
- **Scalability** through separate work items

Critical success factors:
- **sensor_data must process in < 5ms** to maintain 100Hz
- **Buffer management** prevents data loss
- **Timing monitoring** ensures system stability

With proper implementation of these patterns, the system can reliably process activity metrics at the required rates while maintaining system responsiveness.
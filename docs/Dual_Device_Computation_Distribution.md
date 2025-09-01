# Dual Device Computation Distribution Strategy

**Version:** 1.0  
**Date:** June 2025  
**Purpose:** Optimize computational load distribution between primary and secondary devices

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Metric Classification](#metric-classification)
3. [Current Architecture Analysis](#current-architecture-analysis)
4. [Proposed Load Balancing Strategy](#proposed-load-balancing-strategy)
5. [Implementation Approach](#implementation-approach)
6. [Communication Protocol](#communication-protocol)
7. [Dynamic Load Balancing](#dynamic-load-balancing)
8. [Benefits and Trade-offs](#benefits-and-trade-offs)

---

## Executive Summary

With two nRF5340 devices (primary/right and secondary/left), each with identical computational capabilities, we can intelligently distribute the processing load. This document categorizes all metrics from the Activity Session Calculated Data Specification into:

- **Local-only metrics**: Must be calculated on each device for its own foot
- **Aggregated metrics**: Require data from both feet
- **Offloadable metrics**: Can be calculated on either device

### Key Findings

1. **~40% of metrics are local-only** (ground contact, pressure distribution)
2. **~30% require aggregation** (asymmetry, balance, gait symmetry)
3. **~30% are offloadable** (efficiency, fatigue, injury risk)

---

## Metric Classification

### 🟢 Local-Only Metrics (Calculate on Each Device)

These metrics are specific to each foot and must be calculated locally:

| Metric | Update Rate | Complexity | Thread |
|--------|-------------|------------|--------|
| **Ground Contact Time** | Every step | Low | sensor_data |
| **Flight Time** | Every step | Low | sensor_data |
| **Peak Force** | Every step | Low | sensor_data |
| **Pressure Distribution** | 10Hz | Low | sensor_data |
| **Center of Pressure** | 10Hz | Medium | sensor_data |
| **Foot Strike Angle** | Every step | Medium | realtime_metrics |
| **Pronation Angle** | Every step | Medium | realtime_metrics |
| **Loading Rate** | Every step | Medium | realtime_metrics |
| **Push-off Power** | Every step | Medium | realtime_metrics |
| **CPEI** | Every step | High | analytics |

### 🟨 Aggregated Metrics (Require Both Feet)

These metrics need data from both devices:

| Metric | Primary Data | Secondary Data | Complexity | Thread |
|--------|--------------|----------------|------------|--------|
| **Step Frequency/Cadence** | R foot timing | L foot timing | Low | realtime_metrics |
| **Contact Time Asymmetry** | R contact time | L contact time | Low | realtime_metrics |
| **Flight Time Asymmetry** | R flight time | L flight time | Low | realtime_metrics |
| **Force Asymmetry** | R peak force | L peak force | Low | realtime_metrics |
| **Step Length Asymmetry** | R step length | L step length | Medium | analytics |
| **Pronation Asymmetry** | R pronation | L pronation | Low | realtime_metrics |
| **Balance L/R** | R force integral | L force integral | Medium | realtime_metrics |
| **True Flight Time** | R contact state | L contact state | Low | sensor_data |
| **Double Support Time** | R contact state | L contact state | Low | sensor_data |
| **Step Width** | R position | L position | High | analytics |

### 🟦 Offloadable Metrics (Can Calculate on Either Device)

These complex metrics could be calculated on the less busy device:

| Metric | Input Data | Complexity | Typical Thread | Can Offload? |
|--------|------------|------------|----------------|--------------|
| **Running Efficiency** | Multiple factors from both | High | analytics | ✅ Yes |
| **Fatigue Index** | Historical comparison | High | analytics | ✅ Yes |
| **Form Score** | Multiple components | Medium | realtime_metrics | ⚠️ Maybe |
| **Injury Risk Score** | Composite assessment | High | analytics | ✅ Yes |
| **Stride Length** | Step timing + model | Medium | analytics | ✅ Yes |
| **Vertical Oscillation** | IMU integration | High | analytics | ✅ Yes |
| **Vertical Stiffness** | Spring-mass model | High | analytics | ✅ Yes |
| **Movement Smoothness** | Signal analysis | High | analytics | ✅ Yes |
| **Pace Estimation** | Distance/time model | Medium | realtime_metrics | ⚠️ Maybe |
| **Calories Burned** | Metabolic model | Medium | activity_metrics | ✅ Yes |

---

## Current Architecture Analysis

### Current State (Both Devices Calculate Everything)

```
┌──────────────────────────────┬───────────────────────────────┐
│      PRIMARY (Right)         │      SECONDARY (Left)         │
├──────────────────────────────┼───────────────────────────────┤
│ • All 4 threads running      │ • All 4 threads running       │
│ • Calculate all R metrics    │ • Calculate all L metrics     │
│ • Some aggregate metrics     │ • Send raw data to primary    │
│ • BLE to phone              │ • D2D to primary              │
│                              │                               │
│ CPU Load: ~40%               │ CPU Load: ~35%                │
└──────────────────────────────┴───────────────────────────────┘
```

### Issues with Current Approach

1. **Duplicate calculations**: Both devices calculate similar metrics
2. **Incomplete aggregation**: Some metrics calculated without full data
3. **Unbalanced load**: Primary has extra BLE overhead
4. **Wasted capacity**: Secondary has spare CPU cycles

---

## Proposed Load Balancing Strategy

### Strategy 1: Secondary as Analytics Processor (Recommended)

```
┌──────────────────────────────┬───────────────────────────────┐
│      PRIMARY (Right)         │      SECONDARY (Left)         │
├──────────────────────────────┼───────────────────────────────┤
│ • sensor_data (100Hz)        │ • sensor_data (100Hz)         │
│ • realtime_metrics (10Hz)    │ • realtime_metrics (10Hz)     │
│ • activity_metrics (0.5Hz)   │ • analytics (1-5Hz) ENHANCED  │
│                              │                               │
│ Focus on:                    │ Focus on:                     │
│ • Real-time processing       │ • Complex calculations        │
│ • BLE updates               │ • Efficiency metrics          │
│ • Session management        │ • Fatigue analysis            │
│ • Aggregation               │ • Injury risk                 │
│                              │ • Vertical stiffness          │
│                              │                               │
│ CPU Load: ~25%               │ CPU Load: ~25%                │
└──────────────────────────────┴───────────────────────────────┘
```

**Data Flow:**
```
Secondary sensor_data ──D2D──► Primary aggregation
Secondary analytics ◄──D2D──── Primary sensor data
Secondary results ────D2D───► Primary for BLE/logging
```

### Strategy 2: Dynamic Task Migration

```c
typedef struct {
    uint8_t cpu_load_percent;
    uint8_t queue_depth;
    uint16_t processing_time_ms;
    bool can_accept_work;
} device_load_t;

// Primary monitors load and delegates work
void balance_workload(device_load_t *primary_load, 
                     device_load_t *secondary_load) {
    // If primary overloaded and secondary available
    if (primary_load->cpu_load_percent > 60 && 
        secondary_load->can_accept_work) {
        // Offload analytics tasks
        send_analytics_request_to_secondary();
    }
}
```

### Strategy 3: Specialized Processing Roles

```
┌──────────────────────────────┬───────────────────────────────┐
│      PRIMARY (Right)         │      SECONDARY (Left)         │
├──────────────────────────────┼───────────────────────────────┤
│ Specializes in:              │ Specializes in:               │
│ • Time-critical metrics      │ • Computationally intensive   │
│ • Aggregation & symmetry     │ • Pattern recognition         │
│ • BLE communication          │ • Historical analysis         │
│ • Session coordination       │ • Predictive algorithms       │
│                              │                               │
│ Threads:                     │ Threads:                      │
│ • sensor_data (100Hz)        │ • sensor_data (100Hz)         │
│ • realtime_metrics (10Hz)    │ • analytics_heavy (1Hz)       │
│ • aggregation (5Hz)          │ • pattern_detect (0.5Hz)      │
│ • session_mgmt (0.5Hz)       │ • ml_inference (0.2Hz)        │
└──────────────────────────────┴───────────────────────────────┘
```

---

## Implementation Approach

### Phase 1: Identify Offloadable Work

```c
// Define work packages that can be offloaded
typedef enum {
    WORK_EFFICIENCY_CALC = 0x01,
    WORK_FATIGUE_ANALYSIS = 0x02,
    WORK_INJURY_RISK = 0x04,
    WORK_VERTICAL_STIFFNESS = 0x08,
    WORK_STRIDE_ANALYSIS = 0x10,
    WORK_PATTERN_DETECTION = 0x20,
} offloadable_work_t;

typedef struct {
    uint8_t work_type;
    uint32_t timestamp;
    union {
        efficiency_input_t efficiency;
        fatigue_input_t fatigue;
        injury_input_t injury;
    } data;
} work_request_t;
```

### Phase 2: Implement Work Distribution

```c
// Primary device work distributor
void distribute_analytics_work(void) {
    static uint8_t round_robin = 0;
    
    // Check if we have work to distribute
    if (has_pending_analytics()) {
        work_request_t request;
        
        // Alternate between local and remote processing
        if (round_robin++ & 1 && secondary_connected()) {
            // Send to secondary
            prepare_work_request(&request);
            d2d_send_work_request(&request);
        } else {
            // Process locally
            queue_local_analytics(&request);
        }
    }
}

// Secondary device work processor
void process_remote_work(work_request_t *request) {
    analytics_result_t result = {0};
    
    switch (request->work_type) {
        case WORK_EFFICIENCY_CALC:
            result.efficiency = calculate_running_efficiency(
                &request->data.efficiency
            );
            break;
            
        case WORK_FATIGUE_ANALYSIS:
            result.fatigue = analyze_fatigue(
                &request->data.fatigue
            );
            break;
            
        // ... other work types
    }
    
    // Send result back to primary
    d2d_send_work_result(&result);
}
```

### Phase 3: Enhanced D2D Protocol

```c
// Extended D2D message types
typedef enum {
    D2D_MSG_SENSOR_DATA = 0x01,      // Existing
    D2D_MSG_WORK_REQUEST = 0x10,     // New
    D2D_MSG_WORK_RESULT = 0x11,      // New
    D2D_MSG_LOAD_STATUS = 0x12,      // New
    D2D_MSG_SYNC_STATE = 0x13,       // New
} d2d_msg_type_t;

// Load status exchange
typedef struct {
    uint8_t msg_type;  // D2D_MSG_LOAD_STATUS
    uint8_t cpu_load_percent;
    uint8_t analytics_queue_depth;
    uint8_t sensor_queue_depth;
    uint16_t avg_processing_time_ms;
    uint8_t available_work_types;  // Bitmask
} d2d_load_status_t;

// Work result message
typedef struct {
    uint8_t msg_type;  // D2D_MSG_WORK_RESULT
    uint8_t work_type;
    uint32_t request_timestamp;
    uint16_t processing_time_ms;
    union {
        float efficiency_score;
        uint8_t fatigue_index;
        uint8_t injury_risk;
        float vertical_stiffness;
    } result;
} d2d_work_result_t;
```

---

## Communication Protocol

### Efficient D2D Work Distribution

```c
// Minimize D2D traffic with smart batching
typedef struct {
    uint8_t num_requests;
    work_request_t requests[4];  // Batch up to 4 requests
} work_batch_t;

// Send only essential data for remote processing
typedef struct {
    // Minimal data for efficiency calculation
    uint16_t cadence_x2;
    uint8_t duty_factor_pct;
    uint8_t vertical_osc_mm;
    int8_t forward_lean_deg;
} efficiency_input_minimal_t;

// Result compression
typedef struct {
    uint8_t efficiency_score;      // 0-100 instead of float
    uint8_t fatigue_index;         // 0-100
    uint8_t injury_risk;           // 0-100
    uint8_t confidence;            // 0-100
} analytics_result_compressed_t;
```

### Priority-Based Work Queue

```c
// Work priority for load balancing
typedef enum {
    PRIORITY_CRITICAL = 0,    // Never offload
    PRIORITY_HIGH = 1,        // Offload only if very busy
    PRIORITY_MEDIUM = 2,      // Normal offload candidate
    PRIORITY_LOW = 3,         // Always offload if possible
} work_priority_t;

typedef struct {
    struct k_work work;
    work_priority_t priority;
    bool can_offload;
    uint32_t deadline_ms;
    work_request_t request;
} prioritized_work_t;

// Decision logic
bool should_offload_work(prioritized_work_t *work, 
                        device_load_t *local_load,
                        device_load_t *remote_load) {
    // Never offload critical work
    if (work->priority == PRIORITY_CRITICAL) {
        return false;
    }
    
    // Check deadline
    uint32_t time_to_deadline = work->deadline_ms - k_uptime_get_32();
    if (time_to_deadline < 100) {  // Less than 100ms
        return false;  // Process locally to meet deadline
    }
    
    // Load-based decision
    int load_difference = local_load->cpu_load_percent - 
                         remote_load->cpu_load_percent;
    
    switch (work->priority) {
        case PRIORITY_HIGH:
            return load_difference > 30;  // Only if significantly loaded
            
        case PRIORITY_MEDIUM:
            return load_difference > 10;  // Moderate threshold
            
        case PRIORITY_LOW:
            return remote_load->can_accept_work;  // Always if possible
    }
    
    return false;
}
```

---

## Dynamic Load Balancing

### Adaptive Work Distribution Algorithm

```c
typedef struct {
    // Performance tracking
    uint32_t local_completed;
    uint32_t remote_completed;
    uint32_t total_local_time_ms;
    uint32_t total_remote_time_ms;
    
    // Load balancing state
    uint8_t offload_percentage;  // 0-100%
    uint32_t last_adjustment_time;
    
    // Thresholds
    uint8_t target_cpu_load;      // e.g., 30%
    uint8_t max_cpu_load;         // e.g., 50%
} load_balancer_t;

void adaptive_load_balance(load_balancer_t *lb,
                          device_load_t *local,
                          device_load_t *remote) {
    uint32_t now = k_uptime_get_32();
    
    // Adjust every 5 seconds
    if (now - lb->last_adjustment_time < 5000) {
        return;
    }
    
    // Calculate average processing times
    uint32_t avg_local_time = lb->local_completed > 0 ?
        lb->total_local_time_ms / lb->local_completed : 0;
    uint32_t avg_remote_time = lb->remote_completed > 0 ?
        lb->total_remote_time_ms / lb->remote_completed : 0;
    
    // Adjust offload percentage based on performance
    if (local->cpu_load_percent > lb->max_cpu_load) {
        // Increase offloading
        lb->offload_percentage = MIN(100, lb->offload_percentage + 10);
    } else if (local->cpu_load_percent < lb->target_cpu_load &&
               remote->cpu_load_percent > lb->target_cpu_load) {
        // Decrease offloading
        lb->offload_percentage = MAX(0, lb->offload_percentage - 10);
    }
    
    // Consider communication overhead
    if (avg_remote_time > avg_local_time * 2) {
        // Remote processing too slow (including comm overhead)
        lb->offload_percentage = MAX(0, lb->offload_percentage - 20);
    }
    
    lb->last_adjustment_time = now;
    
    LOG_INF("Load balance adjusted: offload=%u%%, local_cpu=%u%%, remote_cpu=%u%%",
            lb->offload_percentage, local->cpu_load_percent, 
            remote->cpu_load_percent);
}
```

### Fault Tolerance

```c
// Handle secondary device unavailability
typedef struct {
    bool secondary_available;
    uint32_t last_heartbeat;
    uint32_t pending_work_count;
    work_request_t pending_work[10];
} secondary_state_t;

void handle_secondary_timeout(secondary_state_t *state) {
    if (k_uptime_get_32() - state->last_heartbeat > 1000) {
        LOG_WRN("Secondary device timeout, processing work locally");
        
        state->secondary_available = false;
        
        // Process all pending work locally
        for (int i = 0; i < state->pending_work_count; i++) {
            queue_local_analytics(&state->pending_work[i]);
        }
        
        state->pending_work_count = 0;
    }
}
```

---

## Benefits and Trade-offs

### Benefits of Load Distribution

1. **Reduced Peak CPU Usage**
   - Primary: 40% → 25% (15% reduction)
   - Secondary: 35% → 25% (10% reduction)

2. **Better Real-time Performance**
   - Fewer deadline misses
   - More consistent BLE updates
   - Smoother user experience

3. **Extended Battery Life**
   - Lower average CPU usage
   - Fewer thermal events
   - More efficient processing

4. **Scalability**
   - Room for additional features
   - ML model deployment ready
   - Future algorithm complexity

### Trade-offs and Challenges

1. **Communication Overhead**
   - D2D messages for work distribution
   - Result synchronization
   - ~5-10ms latency per request

2. **Complexity**
   - State synchronization
   - Error handling
   - Debugging distributed system

3. **Memory Usage**
   - Work queues on both devices
   - Result caching
   - ~4KB additional RAM

### Implementation Complexity vs Benefit

| Strategy | Complexity | Benefit | Recommendation |
|----------|------------|---------|----------------|
| **Fixed Role Assignment** | Low | Medium | ✅ Start here |
| **Round-Robin Distribution** | Medium | Medium | ⚠️ Good second step |
| **Dynamic Load Balancing** | High | High | 🔄 Future enhancement |
| **ML-Based Prediction** | Very High | Very High | 🚀 Long-term goal |

---

## Recommended Implementation Plan

### Phase 1: Fixed Role Assignment (Week 1-2)
1. Assign analytics thread to secondary only
2. Primary focuses on real-time and aggregation
3. Measure performance improvement

### Phase 2: Basic Work Distribution (Week 3-4)
1. Implement work request/result protocol
2. Add round-robin distribution
3. Handle timeout/failure cases

### Phase 3: Dynamic Balancing (Week 5-6)
1. Add load monitoring
2. Implement adaptive algorithm
3. Optimize based on real-world data

### Phase 4: Advanced Features (Future)
1. Predictive load balancing
2. Work stealing queue
3. Distributed state management

---

## Example Implementation

### Secondary as Analytics Processor

```c
// On Secondary Device - Enhanced analytics thread
void analytics_thread_secondary(void) {
    // Process local analytics
    process_local_analytics();
    
    // Check for remote work requests
    work_request_t request;
    if (d2d_get_work_request(&request)) {
        analytics_result_t result;
        
        uint32_t start = k_uptime_get_32();
        
        switch (request.work_type) {
            case WORK_EFFICIENCY_CALC:
                // Calculate for both feet combined
                result.efficiency = calculate_running_efficiency_dual(
                    request.data.efficiency.left,
                    request.data.efficiency.right
                );
                break;
                
            case WORK_FATIGUE_ANALYSIS:
                // Analyze fatigue patterns
                result.fatigue = analyze_fatigue_patterns(
                    request.data.fatigue.history,
                    request.data.fatigue.current
                );
                break;
        }
        
        uint32_t processing_time = k_uptime_get_32() - start;
        
        // Send result back
        d2d_send_analytics_result(&result, processing_time);
    }
}

// On Primary Device - Work distributor
void distribute_analytics_work(void) {
    // Prepare work request with data from both feet
    work_request_t request = {
        .work_type = WORK_EFFICIENCY_CALC,
        .timestamp = k_uptime_get_32(),
        .data.efficiency = {
            .left = get_left_foot_metrics(),
            .right = get_right_foot_metrics(),
            .cadence = get_combined_cadence(),
            .vertical_osc = get_vertical_oscillation()
        }
    };
    
    // Send to secondary for processing
    if (secondary_available()) {
        d2d_queue_work_request(&request);
    } else {
        // Fallback to local processing
        queue_local_analytics(&request);
    }
}
```

---

## Conclusion

Distributing computation between primary and secondary devices offers significant benefits:

1. **Balanced CPU usage** - Both devices at ~25% instead of 35-40%
2. **Specialized processing** - Each device optimized for specific tasks
3. **Scalability** - Room for future features and algorithms
4. **Reliability** - Graceful degradation if one device is busy

The recommended approach is to start with fixed role assignment (secondary as analytics processor) and gradually add dynamic load balancing based on real-world performance data.

Key success factors:
- Minimize D2D communication overhead
- Implement robust error handling
- Monitor and adapt based on actual usage patterns
- Keep the system debuggable and maintainable
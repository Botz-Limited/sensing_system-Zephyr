# Dual Device Computation Distribution Strategy

**Version:** 1.0  
**Date:** January 2025  
**Purpose:** Define optimal distribution of activity metrics calculations between primary and secondary devices to maximize computational efficiency

---

## Executive Summary

With two nRF5340 devices (primary/right and secondary/left), each with identical computational capabilities, we can intelligently distribute the processing load. This document categorizes all metrics from the Activity Session Calculated Data Specification into:

1. **Device-Specific Calculations** - Can be computed locally on each device
2. **Dual-Device Calculations** - Require data from both devices
3. **Primary-Only Calculations** - Aggregated metrics computed on primary

By distributing calculations optimally, we can:
- Reduce computational load on each device by ~40%
- Minimize D2D data transfer
- Improve real-time performance
- Enable more complex algorithms

---

## Table of Contents

1. [System Architecture Overview](#system-architecture-overview)
2. [Metric Classification](#metric-classification)
3. [Device-Specific Calculations](#device-specific-calculations)
4. [Dual-Device Calculations](#dual-device-calculations)
5. [Computation Distribution Strategy](#computation-distribution-strategy)
6. [D2D Communication Optimization](#d2d-communication-optimization)
7. [Implementation Recommendations](#implementation-recommendations)

---

## System Architecture Overview

### Current Setup
```
┌─────────────────────┐         ┌─────────────────────┐
│   Primary Device    │   D2D   │  Secondary Device   │
│   (Right Shoe)      │◄────────►│   (Left Shoe)      │
├─────────────────────┤         ├─────────────────────┤
│ • BHI360 IMU        │         │ • BHI360 IMU        │
│ • 8 Pressure Sensors│         │ • 8 Pressure Sensors│
│ • nRF5340 (128MHz)  │         │ • nRF5340 (128MHz)  │
│ • BLE to Phone      │         │ • D2D to Primary    │
└─────────────────────┘         └─────────────────────┘
```

### Key Capabilities
- **Each device has**: Full sensor suite, identical processing power
- **Primary advantages**: Phone connection, aggregation point
- **Secondary advantages**: Available CPU for complex calculations
- **D2D bandwidth**: Limited, should minimize data transfer

---

## Metric Classification

Based on the Activity Session Calculated Data Specification, here's how metrics are classified:

### Classification Criteria
- **🟦 Device-Specific**: Uses only local sensor data
- **🟨 Dual-Device Required**: Needs synchronized data from both feet
- **🟩 Primary Aggregation**: Combines results from both devices

---

## Device-Specific Calculations

These metrics can be calculated independently on each device using only local sensor data:

### 🟦 Core Step Metrics (Per Foot)
| Metric | Local Data Used | Computation Load | Update Rate |
|--------|-----------------|------------------|-------------|
| **Ground Contact Time** | Local pressure sensors | Light (<1ms) | Every step |
| **Flight Time** | Local pressure sensors | Light (<1ms) | Every step |
| **Peak Force** | Local pressure sensors | Light (<1ms) | Every step |
| **Loading Rate** | Local pressure (dF/dt) | Medium (1-5ms) | Every step |
| **Push-off Power** | Local pressure + IMU | Medium (1-5ms) | Every step |

### 🟦 Pressure Distribution (Per Foot)
| Metric | Local Data Used | Computation Load | Update Rate |
|--------|-----------------|------------------|-------------|
| **Heel Pressure %** | 8 local channels | Light (<1ms) | 10Hz |
| **Midfoot Pressure %** | 8 local channels | Light (<1ms) | 10Hz |
| **Forefoot Pressure %** | 8 local channels | Light (<1ms) | 10Hz |
| **Center of Pressure X/Y** | 8 local channels | Medium (1-5ms) | 10Hz |
| **Pressure Path Length** | CoP history | Medium (1-5ms) | Every step |
| **CPEI** | CoP excursion | Heavy (5-10ms) | Every step |

### 🟦 Motion Dynamics (IMU-based)
| Metric | Local Data Used | Computation Load | Update Rate |
|--------|-----------------|------------------|-------------|
| **Foot Strike Angle** | Local quaternion | Light (<1ms) | Every step |
| **Pronation Angle** | Local quaternion + pressure | Medium (1-5ms) | Every step |
| **Pronation Velocity** | Local gyroscope | Light (<1ms) | Every step |
| **Impact G-force** | Local accelerometer | Light (<1ms) | Every step |
| **Local Movement Smoothness** | Local IMU trajectory | Heavy (5-10ms) | Every second |

### 🟦 Local Performance Indicators
| Metric | Local Data Used | Computation Load | Update Rate |
|--------|-----------------|------------------|-------------|
| **Local Vertical Oscillation** | Local IMU integration | Heavy (10-20ms) | Every 2 steps |
| **Local Duty Factor** | Contact/flight ratio | Light (<1ms) | Every 5 seconds |
| **Local Form Score** | Multiple local metrics | Medium (5-10ms) | Every second |
| **Local Fatigue Detection** | Local pattern changes | Heavy (10-20ms) | Every 30 seconds |

---

## Dual-Device Calculations

These metrics require synchronized data from both devices:

### 🟨 Gait Symmetry Metrics
| Metric | Data Required | Primary Computes | Secondary Provides | Update Rate |
|--------|---------------|------------------|-------------------|-------------|
| **Contact Time Asymmetry** | L/R contact times | ✓ | Contact time | Every 2-4 steps |
| **Flight Time Asymmetry** | L/R flight times | ✓ | Flight time | Every 2-4 steps |
| **Force Asymmetry** | L/R peak forces | ✓ | Peak force | Every 2-4 steps |
| **Step Length Asymmetry** | L/R step timing | ✓ | Step events | Every 2-4 steps |
| **Pronation Asymmetry** | L/R pronation angles | ✓ | Pronation angle | Every 2-4 steps |
| **Loading Rate Asymmetry** | L/R loading rates | ✓ | Loading rate | Every 2-4 steps |
| **Push-off Timing Offset** | L/R push-off times | ✓ | Push-off time | Every 2-4 steps |

### 🟨 Synchronized Performance Metrics
| Metric | Data Required | Computation Strategy | Update Rate |
|--------|---------------|---------------------|-------------|
| **True Flight Time** | Both feet airborne | Primary tracks both contact states | Every step |
| **Double Support Time** | Both feet on ground | Primary tracks overlap | Every step |
| **Step Width** | L/R foot positions | Primary estimates from CoP + IMU | Every 2 steps |
| **Cadence** | Step count from both | Primary aggregates BHI360 counts | Every step |
| **Stride Length** | Full gait cycle | Primary calculates from timing | Every stride |

### 🟨 Advanced Biomechanical Metrics
| Metric | Data Required | Computation Strategy | Update Rate |
|--------|---------------|---------------------|-------------|
| **Running Efficiency** | Multiple factors from both | **Secondary computes** (CPU available) | Every 5-10 seconds |
| **Overall Vertical Stiffness** | Both feet dynamics | **Secondary computes** | Every 10 steps |
| **Bilateral Coordination** | Phase relationships | Primary tracks timing | Every 5 seconds |

---

## Computation Distribution Strategy

### Optimal Load Distribution

```
┌─────────────────────────────────────────────────────────────┐
│                     PRIMARY DEVICE (Right)                    │
├─────────────────────────────────────────────────────────────┤
│ Local Calculations:          �� Aggregated Calculations:       │
│ • Right foot metrics (🟦)    │ • All asymmetry metrics (🟨)   │
│ • Right pressure distribution│ • Cadence aggregation         │
│ • Right motion dynamics      │ • True flight/support times   │
│                              │ • BLE packet preparation      │
│ CPU Load: ~25%               │ CPU Load: ~15%                │
└──────────────────────────────┴───────────────────────────────┘
                                        ▲
                                        │ D2D
                                        │ (Compressed metrics)
                                        ▼
┌─────────────────────────────────────────────────────────────┐
│                    SECONDARY DEVICE (Left)                   │
├─────────────────────────────────────────────────────────────┤
│ Local Calculations:          │ Complex Calculations:          │
│ • Left foot metrics (🟦)     │ • Running efficiency (🟨)      │
│ • Left pressure distribution │ • Injury risk assessment      │
│ • Left motion dynamics       │ • Fatigue index (both feet)   │
│                              │ • Vertical stiffness          │
│ CPU Load: ~25%               │ CPU Load: ~20%                │
└──────────────────────────────┴───────────────────────────────┘
```

### Task Assignment Rationale

#### Primary Device Focus
1. **All device-specific calculations** for right foot
2. **Asymmetry calculations** (has both datasets)
3. **Real-time aggregation** for BLE
4. **Time synchronization** master

#### Secondary Device Focus
1. **All device-specific calculations** for left foot
2. **Complex multi-factor algorithms** (CPU available)
3. **Heavy computations** that don't need real-time
4. **Backup calculations** for redundancy

---

## D2D Communication Optimization

### Data Transfer Requirements

#### Secondary → Primary (Every 100ms)
```c
typedef struct {
    // Essential metrics for asymmetry (24 bytes)
    uint16_t contact_time_ms;
    uint16_t flight_time_ms;
    uint16_t peak_force;
    uint16_t loading_rate;
    int8_t pronation_angle;
    uint8_t strike_pattern;
    
    // Aggregated local calculations (12 bytes)
    uint8_t local_form_score;
    uint8_t local_fatigue_level;
    uint16_t cop_path_length;
    uint8_t pressure_distribution[3];  // Heel/mid/fore percentages
    
    // Complex calculation results (8 bytes)
    uint8_t running_efficiency;
    uint8_t injury_risk_component;
    uint16_t vertical_stiffness;
    uint8_t coordination_score;
} SecondaryMetricsPacket;  // Total: 44 bytes
```

#### Primary → Secondary (Every 1000ms)
```c
typedef struct {
    // User profile for calculations (8 bytes)
    uint16_t user_weight_kg;
    uint16_t user_height_cm;
    uint8_t activity_type;
    uint8_t gps_available;
    
    // Session state (4 bytes)
    uint32_t session_time_ms;
    
    // Primary metrics for efficiency calc (8 bytes)
    uint16_t right_contact_time;
    uint16_t right_flight_time;
    uint8_t right_form_score;
    uint8_t current_pace_min_km;
} PrimaryContextPacket;  // Total: 20 bytes
```

### Bandwidth Analysis
- **Secondary → Primary**: 44 bytes × 10Hz = 440 bytes/sec
- **Primary → Secondary**: 20 bytes × 1Hz = 20 bytes/sec
- **Total D2D bandwidth**: ~460 bytes/sec (well within BLE capacity)

---

## Implementation Recommendations

### 1. Thread Distribution

#### Primary Device Threads
```c
// Thread 1: Sensor Collection (100Hz)
- Read right foot sensors
- Calculate local metrics
- Send to processing queue

// Thread 2: Asymmetry & Aggregation (10Hz)
- Receive secondary metrics via D2D
- Calculate all asymmetry metrics
- Aggregate performance indicators

// Thread 3: BLE Communication (1Hz)
- Prepare real-time packets
- Handle phone commands
- Manage GPS updates

// Thread 4: Session Management (0.5Hz)
- Log aggregated data
- Manage file system
- Calculate session statistics
```

#### Secondary Device Threads
```c
// Thread 1: Sensor Collection (100Hz)
- Read left foot sensors
- Calculate local metrics
- Send to processing queue

// Thread 2: Complex Analytics (2-5Hz)
- Running efficiency calculation
- Injury risk assessment
- Fatigue analysis (both feet data)
- Vertical stiffness computation

// Thread 3: D2D Communication (10Hz)
- Send metrics to primary
- Receive context from primary
- Handle sync commands

// Thread 4: Local Optimization (1Hz)
- Gait pattern analysis
- Form improvement suggestions
- Predictive analytics
```

### 2. Synchronization Strategy

```c
// Time synchronization packet
typedef struct {
    uint32_t primary_timestamp;
    uint32_t secondary_timestamp;
    int16_t clock_offset_ms;
    uint8_t sync_quality;
} TimeSyncPacket;

// Synchronize every 10 seconds
// Maintain <1ms accuracy for asymmetry calculations
```

### 3. Failover Handling

If D2D connection is lost:
- **Primary**: Continues with right-foot-only metrics
- **Secondary**: Stores calculated metrics locally
- **Both**: Set "degraded mode" flag in status

### 4. Power Optimization

```c
// Adaptive computation based on battery
if (battery_level < 20) {
    // Primary: Disable non-essential asymmetry calcs
    // Secondary: Reduce complex analytics frequency
    // Both: Maintain core safety metrics
}
```

---

## Benefits of This Distribution

### Performance Improvements
1. **CPU Load Reduction**: ~40% lower on each device vs. single-device processing
2. **Real-time Performance**: BLE updates never blocked by complex calculations
3. **Parallel Processing**: True parallel computation of independent metrics
4. **Scalability**: Easy to add new algorithms to less loaded device

### System Advantages
1. **Redundancy**: Critical metrics calculated on both devices
2. **Flexibility**: Can reassign tasks based on load
3. **Power Efficiency**: Balanced power consumption
4. **Maintainability**: Clear separation of concerns

### Future Enhancements
1. **Dynamic Load Balancing**: Shift calculations based on real-time CPU usage
2. **ML Model Distribution**: Run inference on secondary while primary handles I/O
3. **Cooperative Processing**: Split single algorithm across both devices
4. **Edge Computing**: Secondary as edge processor for pattern recognition

---

## Summary

By intelligently distributing calculations between primary and secondary devices:

- **Device-specific calculations** (🟦) run locally on each device
- **Asymmetry and aggregation** (🟨) computed on primary (has all data)
- **Complex analytics** (🟨) offloaded to secondary (available CPU)
- **D2D traffic minimized** to essential metrics only
- **Both devices utilized** efficiently at ~40-45% CPU each

This architecture maximizes the computational resources available across both devices while maintaining real-time performance and minimizing power consumption.
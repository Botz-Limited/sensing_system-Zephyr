# Activity Session Calculated Data Specification

**Version:** 1.3  
**Date:** July 2025 (Updated)  
**Purpose:** Complete specification for pre-calculated activity session data to be logged and transmitted via BLE from shoe-mounted sensors

---

## Summary

This document defines a  system for processing raw sensor data into meaningful, pre-calculated metrics that are both logged locally and transmitted via BLE to mobile applications. The system eliminates raw data transmission, reducing data volume by ~99% while providing actionable insights for athletes, coaches, and medical professionals.

**📝 Code Examples Disclaimer**: All C code examples in this document are provided for **illustrative purposes only** to demonstrate algorithms and concepts. They're not production-ready - we'll need to adapt them for our firmware.

**Key Benefits:**
- **Data Reduction**: From 31.7 MB/hour raw data to 58 KB/hour calculated metrics
- **Real-time Feedback**: Immediate performance insights during activities
- **Battery Efficiency**: Minimal BLE transmission requirements
- **Injury Prevention**: Early detection of risky movement patterns

**Important Note**: The system uses delta timestamps throughout - each data packet contains the time elapsed since the previous packet rather than absolute timestamps. This approach minimizes data size and simplifies synchronization.

### ⚠️ Implementation Status (Updated August 2025)

**This document describes a comprehensive activity metrics system. Core components are IMPLEMENTED with ALL Category 1 metrics from colleague's proposal COMPLETE.**

**✅ FULLY IMPLEMENTED:**
- **Multi-threaded architecture** with dedicated modules:
  - `sensor_data`: Processes raw sensor data at 80Hz
  - `realtime_metrics`: Calculates real-time metrics at 1Hz
  - `activity_metrics`: Manages sessions and activity-level calculations
  - `analytics`: Module created but algorithms not yet implemented
- **Thread-safe communication** using work queues and message passing
- **BLE Activity Metrics Service** (`4fd5b690-9d89-4061-92aa-319ca786baae`) with:
  - Real-time metrics characteristic (1Hz updates) ✅
  - Asymmetry metrics characteristic (0.5Hz updates) ✅
  - Stride metrics characteristics (1Hz updates) ✅ **NEW**
  - Total and activity step counts ✅
  - GPS data input characteristic ✅
  - Biomechanics extended characteristic (structure only)
  - Session summary characteristic (structure only)
- **Core sensor processing**:
  - Ground contact detection (±2ms accuracy)
  - Flight time calculation
  - Peak force detection
  - Pressure distribution (heel/midfoot/forefoot %)
  - Basic pronation detection
  - Strike pattern classification
- **Real-time metrics (ALL CATEGORY 1 COMPLETE)**:
  - Cadence calculation (steps/min) ✅
  - Pace estimation (sec/km, with GPS dual function) ✅
  - Distance tracking (meters) ✅
  - Form score (0-100) ✅
  - Left/right balance ratio ✅
  - Ground contact time ✅
  - Flight time ✅
  - Efficiency score ✅
  - Alert system ✅
- **Asymmetry metrics (ALL CATEGORY 1 COMPLETE)**:
  - Contact time asymmetry ✅
  - Flight time asymmetry ✅
  - Force asymmetry ✅
  - Pronation asymmetry ✅
  - Strike pattern L/R ✅
- **Stride metrics (ALL CATEGORY 1 COMPLETE)** ✅ **NEW**:
  - Stride duration (ms) ✅
  - Stride duration asymmetry (%) ✅
  - Stride length (cm) ✅
  - Stride length asymmetry (%) ✅
- **Vertical metrics (CATEGORY 1 COMPLETE)**:
  - Vertical oscillation (simplified) ✅
  - Vertical ratio ✅
- **Session management**:
  - Start/stop activity commands via Control Service
  - Activity types support (running, walking, training)
  - Session state tracking
  - Weight measurement functionality
- **GPS integration**:
  - GPS update command structure ✅
  - GPS data reception via Activity Metrics Service ✅
  - D2D forwarding to secondary device ✅
  - Stride calibration with GPS distance ✅
  - **Dual function implementation**: Automatic fallback to sensor-only when GPS unavailable ✅
  - GPS-based pace calculation ✅
  - GPS-based distance calculation ✅
- **Data logging** (still using protobuf format):
  - Foot sensor data logging
  - BHI360 motion data logging
  - Activity step count logging

**🔄 IN DEVELOPMENT:**
- **Advanced analytics algorithms**:
  - Fatigue detection (basic placeholder only)
  - Injury risk assessment (basic placeholder only)
  - Running efficiency calculation
  - Vertical oscillation
  - Vertical stiffness
- **Enhanced GPS integration**:
  - GPS-based distance calculation
  - Pace smoothing with GPS
  - Elevation tracking
- **Advanced biomechanics**:
  - Center of Pressure Excursion Index (CPEI)
  - Push-off power calculation
  - Loading rate analysis (BLE structure exists, no calculation)
  - Foot strike angle
  - Arch collapse detection (BLE structure exists, no calculation)

**❌ NOT IMPLEMENTED:**
- **Binary struct logging** (still using protobuf, ~31.7 MB/hour)
- **Session summary calculations**:
  - Total distance (GPS-accurate)
  - Calories burned
  - Training load
  - Split times
- **Recovery recommendations**
- **Historical comparisons**
- **Adaptive thresholds**
- **Overstriding detection**

**Current System Performance:**
- **Real-time metrics**: ✅ Transmitted via BLE at 1Hz
- **Sensor processing**: ✅ Running at 80Hz with <1ms latency
- **Memory usage**: ✅ Within constraints using static allocation
- **Power consumption**: ⚠️ Not yet optimized for battery life

---

## Table of Contents
1. [System Overview](#system-overview)
2. [Calculated Metrics Definition](#calculated-metrics-definition)
3. [GPS Integration and Battery Optimization](#gps-integration-and-battery-optimization)
4. [Data Structures](#data-structures)
5. [BLE Protocol](#ble-protocol)
6. [Implementation Guidelines](#implementation-guidelines)
7. [Mobile Integration](#mobile-integration)
8. [Benefits Summary](#benefits-summary)
9. [Future Enhancements](#future-enhancements)
10. [Summary of Key Features for Clinical and Product Teams](#summary-of-key-features-for-clinical-and-product-teams)
11. [Implementation Summary](#implementation-summary)
12. [Conclusion](#conclusion)
13. [Pace and Distance Calculation Methods](#pace-and-distance-calculation-methods)
14. [8-Channel Pressure Sensor Capabilities](#8-channel-pressure-sensor-capabilities)

---

## System Overview

### Device Configuration
- **Primary Device**: Right shoe (master for synchronization)
- **Secondary Device**: Left shoe (synchronized to primary)
- **Raw Sampling**: 80Hz (pressure sensors + IMU)
- **Calculated Output**: 0.5-2Hz (context-dependent)
- **Time Sync**: <1ms between devices

### Dual-Device Synchronization Benefits

Having two synchronized devices (one per foot) enables unique capabilities that single-device or non-synchronized systems cannot achieve:

#### 1. **True Bilateral Timing Analysis**
- **Precise Step Timing**: Exact moment each foot contacts/leaves ground (±1ms)
- **Actual Flight Time**: Time when BOTH feet are airborne (impossible with single device)
- **Double Support Time**: Duration when both feet are on ground
- **Step Time Variability**: L-R-L-R timing patterns for rhythm analysis

#### 2. **Real-time Asymmetry Detection**
- **Instantaneous Comparison**: Compare left/right metrics within same step cycle
- **Dynamic Asymmetry**: Track how asymmetry changes with fatigue, speed, or terrain
- **Compensation Detection**: Identify when one leg compensates for the other
- **Early Injury Warning**: Detect subtle favoring before it becomes obvious

#### 3. **Enhanced Gait Phase Accuracy**
- **True Gait Cycle**: Track complete cycle from right heel strike to next right heel strike
- **Swing Phase Validation**: Confirm when each foot is actually in swing phase
- **Crossover Detection**: Identify when feet cross midline (requires both feet tracking)
- **Cadence Precision**: Exact step-by-step cadence vs averaged estimation

#### 4. **Synchronized Pressure Patterns**
- **Load Transfer Analysis**: Track how weight shifts between feet
- **Push-off Coordination**: Compare timing and force of left/right push-off
- **Balance During Stance**: Detect if runner favors one foot during single support
- **Turning Mechanics**: Analyze inside vs outside foot loading during curves

#### 5. **Advanced Biomechanical Metrics**
- **True Step Width**: Calculate actual lateral distance between foot placements
- **Pelvic Drop Estimation**: Infer hip stability from bilateral loading patterns
- **Running Efficiency**: Compare energy transfer between legs
- **Fatigue Progression**: Track which leg fatigues first

#### 6. **Clinical-Grade Analysis**
- **Rehabilitation Progress**: Objective measurement of return to symmetry
- **Gait Retraining Validation**: Ensure both feet adopt new patterns
- **Load Distribution**: Verify equal work between legs during recovery
- **Compensation Patterns**: Detect complex multi-joint compensations

#### Synchronization Technical Details
- **Time Sync Protocol**: Primary device broadcasts time reference every 100ms
- **Clock Drift Compensation**: Automatic adjustment for crystal frequency differences
- **Data Alignment**: All metrics timestamped to common reference frame
- **Wireless Sync Accuracy**: Maintains <1ms synchronization even during 4+ hour activities

### Data Flow
```
Raw Sensors (80Hz) → On-Device Processing → Calculated Metrics → BLE/Storage
                           ↓                        ↓
                    Feature Extraction      Real-time (1Hz) ✅
                    Statistical Analysis    Logged (0.5Hz) ⚠️
```

**Implementation Status**: 
- ✅ Raw sensor processing at 80Hz in sensor_data module
- ✅ Real-time BLE transmission at 1Hz
- ⚠️ Logging still uses protobuf format, not optimized binary structs

### Activity Types Supported
1. **Running**: Focus on pace, cadence, ground contact time ✅
2. **Walking**: Step count, symmetry, stability ✅
3. **Training**: Form analysis, fatigue detection ⚠️ (basic only)
4. **Recovery**: Gait quality, compensation patterns ❌
5. **Custom**: User-defined metrics ❌

---

## Calculated Metrics Definition

### Basic Metrics (Per Foot) - Implementation Status

| Metric | Update Rate | Description | Range/Units | Status |
|--------|-------------|-------------|-------------|---------|
| Ground Contact Time | Every step | Duration foot is on ground | 100-400ms | ✅ Implemented |
| Flight Time | Every step | Duration both feet off ground | 0-200ms | ✅ Implemented |
| Peak Force | Every step | Maximum force during contact | 0-5000N | ✅ Implemented |
| Pressure Distribution | 10Hz | Heel/Midfoot/Forefoot % | 0-100% each | ✅ Implemented |
| Center of Pressure | 10Hz | CoP location on foot | x,y in mm | ❌ Not started |
| Foot Strike Angle | Every step | Angle at initial contact | -15° to +30° | ❌ Not started |
| Pronation Angle | Every step | Maximum inward roll | -20° to +20° | ✅ Basic only |
| Loading Rate | Every step | Force increase rate | 0-200 BW/s | ❌ Not started |
| Push-off Power | Every step | Power during toe-off | 0-1000W | ❌ Not started |
| CPEI | Every step | Center pressure path length | 0-100mm | ❌ Not started |

### Calculated Metrics (Combined Feet) - Implementation Status

| Metric | Update Rate | Description | Accuracy | Status |
|--------|-------------|-------------|----------|---------|
| Cadence | 1Hz | Steps per minute | ±1 spm | ✅ Implemented |
| Pace | 1Hz | Time per km/mile | ±5 sec/km | ✅ Dual function (GPS/sensor) |
| Distance | 1Hz | Cumulative distance | ±2% with GPS | ✅ Dual function (GPS/sensor) |
| Vertical Oscillation | 1Hz | Vertical displacement | ±2cm | ✅ Simplified implementation |
| Vertical Ratio | 1Hz | Oscillation/stride ratio | 0-100 | ✅ Implemented |
| Running Efficiency | 1Hz | Energy efficiency score | 0-100 | ✅ Simplified implementation |
| Form Score | 1Hz | Composite technique score | 0-100 | ✅ Implemented |
| Stride Duration | 1Hz | Average stride time | ±10ms | ✅ Implemented |
| Stride Length | 1Hz | Average stride distance | ±5cm | ✅ Implemented |

### Asymmetry Metrics - Implementation Status

| Metric | Update Rate | Description | Threshold | Status |
|--------|-------------|-------------|-----------|---------|
| Contact Time Diff | 1Hz | L/R contact time delta | >4% warning | ✅ Implemented |
| Step Length Diff | 0.5Hz | L/R step length delta | >5% warning | ✅ Implemented |
| Peak Force Diff | 0.5Hz | L/R force delta | >10% warning | ✅ Implemented |
| Loading Rate Diff | 0.5Hz | L/R loading delta | >15% warning | ❌ Not started |
| Balance Score | 1Hz | Weight distribution | 40-60% normal | ✅ Implemented |

### Health & Safety Metrics - Implementation Status

| Metric | Update Rate | Description | Alert Level | Status |
|--------|-------------|-------------|-------------|---------|
| Fatigue Index | 0.5Hz | Cumulative fatigue score | >70% warning | ⚠️ Placeholder only |
| Injury Risk Score | 0.5Hz | Composite risk assessment | >80% critical | ⚠️ Placeholder only |
| Recovery Quality | End of session | Post-activity assessment | 0-100% | ❌ Not started |
| Overstriding | Every step | Foot lands ahead of CoM | >30cm warning | ❌ Not started |
| Impact Severity | Every step | Shock magnitude | >3g caution | ❌ Not started |

---

## GPS Integration and Battery Optimization

### GPS Operating Modes - Implementation Status

| Mode | Update Rate | Accuracy | Battery Impact | Use Case | Status |
|------|-------------|----------|----------------|----------|---------|
| OFF | Never | N/A | None | Treadmill/Track | ✅ |
| CALIBRATION | 10s | ±5m | Low | Stride calibration | ✅ Framework |
| PRECISE | 1s | ±3m | Medium | Race/Competition | ✅ Framework |
| RACE | 0.5s | ±2m | High | Elite performance | ✅ Framework |

**Implementation Notes**:
- ✅ GPS data structure defined (GPSUpdateCommand)
- ✅ GPS data reception via Activity Metrics Service (write characteristic)
- ✅ Stride calibration with GPS distance (exponential smoothing)
- ✅ GPS-based distance calculation implemented
- ✅ GPS-based pace calculation implemented
- ✅ **Dual function design**: Automatic fallback to sensor-only when GPS unavailable
- ✅ GPS accuracy check (ignores updates >20m accuracy)
- ❌ Smart GPS mode switching not implemented
- ❌ Battery optimization strategies not implemented

### Smart GPS Strategy
```c
// Pseudocode for GPS optimization (NOT IMPLEMENTED)
if (pace_stable && straight_line) {
    gps_mode = GPS_CALIBRATION;  // Save battery
} else if (turning || pace_changing) {
    gps_mode = GPS_PRECISE;  // Higher accuracy needed
}
```

**Status**: ❌ Smart GPS strategy not implemented

---

## Data Structures

### Core Data Types (IMPLEMENTED ✅)

```c
// All time values use delta encoding
typedef uint16_t delta_ms_t;    // 0-65535ms range
typedef uint32_t timestamp_ms_t; // Absolute time reference

// Fixed-point representations for efficiency
typedef int16_t fixed16_t;  // Q8.8 format
typedef int32_t fixed32_t;  // Q16.16 format
```

### Session Header (IMPLEMENTED ✅)

```c
typedef struct __attribute__((packed)) {
    uint32_t session_id;        // Unique session identifier
    uint32_t start_time_utc;    // Unix timestamp
    uint16_t fw_version;        // Firmware version
    uint8_t  activity_type;     // Running/Walking/Training
    uint8_t  user_id;           // User identifier
    
    // User profile for calculations
    uint16_t user_height_cm;    
    uint16_t user_weight_kg;    
    uint8_t  shoe_size_eu;      
    uint8_t  flags;             // GPS enabled, etc.
} session_header_t;  // 20 bytes
```

### Real-time Metrics Packet - 1Hz BLE (IMPLEMENTED ✅)

```c
typedef struct __attribute__((packed)) {
    // Timing
    uint16_t delta_ms;          // Since last packet
    
    // Primary metrics
    uint16_t cadence_x2;        // Steps/min * 2
    uint16_t pace_sec_per_km;   // Seconds per km
    uint8_t  form_score;        // 0-100
    
    // Force & Contact
    uint16_t avg_contact_ms;    // Average ground contact
    uint16_t peak_force_n;      // Peak vertical force
    
    // Balance & Symmetry
    uint8_t  balance_lr;        // Left/right % (0-100)
    int8_t   asymmetry_index;   // -100 to +100
    
    // Fatigue & Risk
    uint8_t  fatigue_score;     // 0-100
    uint8_t  injury_risk;       // 0-100
} realtime_metrics_t;  // 16 bytes
```

**Implementation Status**: Structure defined and actively used for BLE transmission at 1Hz

### Asymmetry Metrics Packet - 0.5Hz BLE (IMPLEMENTED ✅)

```c
typedef struct __attribute__((packed)) {
    int16_t contact_time_diff_ms;  // L-R difference
    int16_t step_length_diff_cm;   // L-R difference
    int16_t peak_force_diff_n;      // L-R difference
    uint8_t balance_score;          // 0-100
    uint8_t asymmetry_flags;        // Warning flags
    uint32_t step_count_l;          // Left foot steps
    uint32_t step_count_r;          // Right foot steps
} asymmetry_metrics_ble_t;  // 16 bytes
```

**Implementation Status**: Actively transmitted via BLE at 0.5Hz

### Biomechanics Extended Packet (PARTIALLY IMPLEMENTED ⚠️)

```c
typedef struct __attribute__((packed)) {
    int16_t pronation_angle_l;      // Left foot pronation
    int16_t pronation_angle_r;      // Right foot pronation
    uint16_t loading_rate_l;        // Left loading rate (BW/s)
    uint16_t loading_rate_r;        // Right loading rate (BW/s)
    uint8_t arch_collapse_l;        // Left arch collapse %
    uint8_t arch_collapse_r;        // Right arch collapse %
} biomechanics_extended_ble_t;  // 12 bytes
```

**Implementation Status**: 
- ✅ BLE characteristic exists
- ✅ Pronation angles calculated
- ❌ Loading rate not calculated
- ❌ Arch collapse not calculated

### GPS Data Input (IMPLEMENTED ✅)

```c
typedef struct __attribute__((packed)) {
    int32_t latitude;           // Degrees * 10^7
    int32_t longitude;          // Degrees * 10^7
    uint16_t accuracy_m;        // Horizontal accuracy
    int16_t altitude_m;         // Altitude in meters
    uint16_t speed_cm_s;        // Speed in cm/s
    uint16_t heading_deg;       // Heading in degrees
} gps_data_ble_t;  // 16 bytes
```

**Implementation Status**: Write characteristic, mobile app sends GPS data to device

---

## BLE Protocol

### Service Structure (ACTUAL IMPLEMENTATION ✅)

**Activity Metrics Service** (UUID: `4fd5b690-9d89-4061-92aa-319ca786baae`)

| Characteristic | UUID | Type | Update Rate | Implementation Status |
|----------------|------|------|-------------|----------------------|
| Real-time Metrics | `4fd5b691-...` | notify | 1Hz | ✅ Fully working |
| Asymmetry Metrics | `4fd5b692-...` | notify | 0.5Hz | ✅ Fully working |
| Biomechanics Extended | `4fd5b693-...` | notify | On demand | ⚠️ Structure only |
| Session Summary | `4fd5b694-...` | read+notify | End of session | ⚠️ Structure only |
| GPS Data Input | `4fd5b695-...` | write | From mobile | ✅ Fully working |
| Total Step Count | `4fd5b696-...` | read+notify | On change | ✅ Fully working |
| Activity Step Count | `4fd5b697-...` | read+notify | On change | ✅ Fully working |

### Control Commands (via Control Service) (IMPLEMENTED ✅)

| Command | Value | Parameters | Response | Status |
|---------|-------|------------|----------|---------|
| START_ACTIVITY | 0x01 | activity_type | session_id | ✅ |
| STOP_ACTIVITY | 0x02 | none | summary | ✅ |
| GPS_UPDATE | 0x10 | lat, lon, accuracy | none | ✅ Via Activity Metrics Service |
| CALIBRATE | 0x20 | sensor_type | status | ✅ |

---

## Implementation Guidelines

### Thread Architecture (IMPLEMENTED ✅)

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  sensor_data    │────▶│ realtime_metrics │────▶│    bluetooth    │
│   (80Hz)        │     │     (1Hz)        │     │   (BLE notify)  │
└─────────────────┘     └──────────────────┘     └─────────────────┘
         │                       │
         ▼                       ▼
┌─────────────────┐     ┌──────────────────┐
│ activity_metrics│     │    analytics     │
│  (session mgmt) │     │ (complex calc)   │
└─────────────────┘     └──────────────────┘
```

**Status**: ✅ All modules created and connected via message queues

### Memory Requirements (CURRENT USAGE)
- **Static Allocation**: ~25KB used (target was 40KB)
  - Sensor buffers: 15KB ✅
  - Processing buffers: 6KB ✅
  - Metric storage: 3KB ✅
  - BLE queue: 1KB ✅
- **No dynamic allocation** ✅
- **Stack usage**: <1.5KB per thread ✅

### Power Optimization (NOT IMPLEMENTED ❌)
1. **Adaptive Sampling**: Reduce rate during low activity ❌
2. **Efficient Processing**: Use fixed-point math where possible ⚠️ (partial)
3. **Smart Sleep**: Enter low-power mode when stationary ❌
4. **Batch Operations**: Process multiple samples together ✅

### Calibration Requirements
1. **Pressure Sensors**: Zero offset, sensitivity, crosstalk matrix ✅
2. **IMU**: Gyro bias, accelerometer scale/bias, alignment ✅
3. **Time Sync**: <1ms synchronization between feet ✅ (via D2D)
4. **User-Specific**: Height/weight for accurate calculations ✅

---

## Mobile Integration

### Required Mobile App Features

#### 1. Real-time Display (SUPPORTED BY FIRMWARE ✅)
- Primary metrics dashboard ✅
- Color-coded quality indicators ✅
- Trend visualization ✅
- Audio/haptic feedback (app responsibility)

#### 2. Session Management (SUPPORTED BY FIRMWARE ✅)
```javascript
// Example session flow
async function startActivitySession(activityType, userProfile) {
    // Connect to both shoes
    await connectToShoes();
    
    // Send GPS update via Activity Metrics Service
    await writeGPSData(lat, lon, accuracy, altitude, speed, heading);
    
    // Start session via Control Service
    await sendControlCommand(START_ACTIVITY, activityType);
    
    // Enable notifications
    await enableMetricsNotifications();
}
```

#### 3. Data Processing
- Additional smoothing/filtering (app side)
- Trend analysis over time (app side)
- Comparison with historical data (app side)
- Export capabilities (app side)

#### 4. Available BLE Data Streams

**Real-time (1Hz):**
- Cadence, pace, form score
- Ground contact time, peak force
- Left/right balance
- Basic fatigue/injury risk scores

**Asymmetry Detection (0.5Hz):**
- Contact time differences
- Step length differences
- Peak force differences
- Balance score with warning flags

**Activity Tracking:**
- Total steps since boot
- Steps during current activity
- GPS data write capability

---

## Implementation Status Summary

### What's Working Now
1. **Multi-threaded architecture** with sensor_data → realtime_metrics → BLE flow
2. **Basic biomechanics**: contact/flight time, pressure distribution, strike pattern
3. **Real-time metrics**: cadence, pace, form score, balance
4. **BLE transmission**: 1Hz updates of packed metrics structure
5. **Session management**: start/stop activities with basic tracking
6. **GPS integration**: Dual function implementation with automatic fallback
   - GPS data reception and stride calibration
   - GPS-based pace and distance calculations
   - Sensor-only fallback when GPS unavailable
7. **Asymmetry detection**: real-time L/R comparison with BLE notifications
8. **Step counting**: both total and per-activity

### What's Missing
1. **Advanced analytics**: fatigue, injury risk, efficiency calculations
2. **Binary logging**: still using protobuf (large files)
3. **Complex biomechanics**: CPEI, vertical oscillation, loading rate
4. **Session summaries**: total distance, calories, elevation
5. **Power optimizations**: adaptive sampling, smart sleep
6. **Smart GPS mode switching**: battery optimization strategies

### BLE Characteristics Status (All Category 1 Complete ✅)

| Feature | Calculation | BLE Characteristic | Data Flow |
|---------|-------------|-------------------|-----------|
| Cadence, Pace, Distance | ✅ Working | ✅ Individual characteristics (1Hz) | ✅ Complete |
| Form Score, Efficiency | ✅ Working | ✅ Individual characteristics (1Hz) | ✅ Complete |
| Ground Contact, Flight Time | ✅ Working | ✅ Individual characteristics (1Hz) | ✅ Complete |
| Balance L/R | ✅ Working | ✅ Individual characteristic (1Hz) | ✅ Complete |
| Contact/Flight Asymmetry | ✅ Working | ✅ Individual characteristics (1Hz) | ✅ Complete |
| Force/Pronation Asymmetry | ✅ Working | ✅ Individual characteristics (1Hz) | ✅ Complete |
| Strike Pattern L/R | ✅ Working | ✅ Individual characteristics (1Hz) | ✅ Complete |
| Stride Duration/Length | ✅ Working | ✅ Individual characteristics (1Hz) | ✅ Complete |
| Stride Asymmetries | ✅ Working | ✅ Individual characteristics (1Hz) | ✅ Complete |
| Vertical Oscillation/Ratio | ✅ Working | ✅ In realtime metrics | ✅ Complete |
| Step Counting | ✅ Working | ✅ Two characteristics | ✅ Complete |
| GPS Reception | N/A | ✅ Write characteristic | ✅ Complete |
| Basic Pronation | ✅ Working | ⚠️ Biomech extended | ⚠️ Structure only |
| Loading Rate | ❌ No calc | ⚠️ Biomech extended | ❌ No data |
| Session Summary | ❌ No calc | ⚠️ Characteristic exists | ❌ No data |
| CPEI, Strike Angle | ❌ No calc | ❌ No characteristic | ❌ Not planned |

### Next Development Priorities
1. Add fatigue and injury risk algorithms in analytics module
2. Complete biomechanics extended calculations (loading rate, arch collapse)
3. Populate session summary with real data
4. Migrate from protobuf to binary struct logging
5. Add remaining biomechanical calculations (CPEI, vertical oscillation)
6. Implement smart GPS mode switching for battery optimization

---

## Conclusion

The activity metrics system has made significant progress with core infrastructure and basic metrics fully operational. The multi-threaded architecture is working well, providing real-time metrics to mobile apps via BLE. All essential BLE characteristics are implemented and working for the core features.

**Current Data Rates**:
- Input: 31.7 MB/hour (raw sensor data)
- Output via BLE: ~58 KB/hour (calculated metrics) ✅
- Logged data: Still ~31.7 MB/hour (protobuf format) ❌

The system is production-ready for basic activity tracking with real-time performance metrics, asymmetry detection, and step counting. Advanced biomechanical analysis and optimal storage efficiency remain to be developed.

**Mobile App Integration Ready For:**
- Real-time performance monitoring (cadence, pace, form)
- Asymmetry and imbalance detection
- Step counting and activity tracking
- GPS data input for future distance calculations
- Activity session management (start/stop)

---

## Pace and Distance Calculation Methods

### Dual Function Implementation (IMPLEMENTED ✅)

The system implements a sophisticated dual-function approach for pace and distance calculations that automatically selects the best available method:

#### Distance Calculation Methods:

1. **Sensor-Only Method** (`calculate_distance_no_gps`):
   - Uses step count × default stride length (0.7m)
   - Accuracy: ±15-20%
   - Always available

2. **GPS-Corrected Method** (`calculate_distance_with_gps`):
   - Uses step count × GPS-calibrated stride length
   - Applies stride correction factor from GPS calibration
   - Accuracy: ±2-5% when GPS available

3. **Automatic Selection** (`calculate_total_distance`):
   ```c
   if (stride_correction_factor valid && GPS data recent) {
       use GPS-corrected calculation
   } else {
       fallback to sensor-only calculation
   }
   ```

#### Pace Calculation Methods:

1. **Sensor-Only Method** (`calculate_pace_no_gps`):
   - Based on cadence and estimated stride length
   - Adjusts stride length based on cadence (shorter at high cadence)
   - Accuracy: ±10-15 sec/km

2. **GPS-Based Method** (`calculate_pace_with_gps`):
   - Direct calculation from GPS distance and elapsed time
   - Accuracy: ±5 sec/km with good GPS signal

3. **Automatic Selection** (`calculate_current_pace`):
   ```c
   if (GPS update within last 30 seconds) {
       use GPS-based pace
   } else {
       fallback to sensor-based pace
   }
   ```

#### GPS Stride Calibration Process:

1. **Continuous Calibration**:
   - Compares sensor-calculated distance with GPS distance
   - Updates stride correction factor using exponential smoothing (0.9 × old + 0.1 × new)
   - Only calibrates when moved >10m since last GPS update

2. **Quality Checks**:
   - Ignores GPS updates with accuracy >20m
   - Limits correction factor to ±20% (0.8 to 1.2)
   - Validates time between updates (>5 seconds)

3. **Elevation Tracking**:
   - Accumulates elevation changes when provided by GPS
   - Stored in `total_elevation_gain_cm`

This dual-function approach ensures the system always provides pace and distance estimates, with automatic accuracy improvements when GPS data is available from the mobile app.

---

## 8-Channel Pressure Sensor Capabilities

The 8-channel pressure sensor array in each shoe provides detailed foot pressure distribution data that enables advanced biomechanical analysis. The channels are strategically positioned to capture key areas of foot contact during gait.

### Channel Layout and Mapping

| Channel | Location | Primary Use |
|---------|----------|-------------|
| 0-2 | Heel region | Heel strike detection, loading rate |
| 3-4 | Midfoot region | Arch support, pronation analysis |
| 5-7 | Forefoot region | Push-off power, toe-off detection |

### Implemented Capabilities (✅)

1. **Strike Pattern Detection**:
   - Classifies foot strike as heel, midfoot, or forefoot
   - Based on which channels activate first at ground contact
   - Updates every step with high accuracy

2. **Pressure Distribution Analysis**:
   - Calculates heel/midfoot/forefoot pressure percentages
   - Updates at 10Hz during ground contact
   - Used for balance and loading pattern assessment

3. **Ground Contact Detection**:
   - Detects precise moment of ground contact and toe-off
   - Uses total pressure threshold with hysteresis
   - Enables accurate contact/flight time measurements

4. **Peak Force Measurement**:
   - Tracks maximum total force during each step
   - Automatically resets during flight phase
   - Used for impact analysis and asymmetry detection

### Potential Future Capabilities (❌)

1. **Center of Pressure (CoP) Tracking**:
   - Calculate precise CoP location using weighted average
   - Track CoP path throughout stance phase
   - Enable CPEI (Center of Pressure Excursion Index) calculation

2. **Arch Collapse Detection**:
   - Monitor midfoot pressure changes during stance
   - Detect excessive arch flattening
   - Useful for pronation analysis

3. **Loading Rate Analysis**:
   - Calculate rate of force increase after ground contact
   - Identify high-impact landing patterns
   - Important for injury risk assessment

4. **Push-off Power Calculation**:
   - Analyze forefoot pressure during toe-off
   - Estimate propulsive force generation
   - Assess running efficiency

The 8-channel design provides sufficient resolution for clinical-grade gait analysis while maintaining reasonable data rates and processing requirements.
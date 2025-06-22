# Activity Session Calculated Data Specification

**Version:** 1.0  
**Date:** June 2025  
**Purpose:** Complete specification for pre-calculated activity session data to be logged and transmitted via BLE from shoe-mounted sensors

---

## Summary

This document defines a comprehensive system for processing raw sensor data into meaningful, pre-calculated metrics that are both logged locally and transmitted via BLE to mobile applications. The system eliminates raw data transmission, reducing data volume by ~99% while providing actionable insights for athletes, coaches, and medical professionals.

**Key Benefits:**
- **Data Reduction**: From 31.7 MB/hour raw data to 58 KB/hour calculated metrics
- **Real-time Feedback**: Immediate performance insights during activities
- **Battery Efficiency**: Minimal BLE transmission requirements
- **Injury Prevention**: Early detection of risky movement patterns

**Important Note**: The system uses delta timestamps throughout - each data packet contains the time elapsed since the previous packet rather than absolute timestamps. This approach minimizes data size and simplifies synchronization.

### ⚠️ Implementation Status

**This is a PROPOSAL document. The system described here is NOT YET IMPLEMENTED.**

**Current System (Implemented):**
- Raw sensor data logging using **Protobuf** format
- Logs ALL sensor data at 100Hz
- Large files: ~31.7 MB/hour
- Used for debugging and development

**Proposed System (This Document):**
- Calculated metrics only using **binary C struct** format
- NO raw data storage during activities
- Small files: ~45 KB/hour
- Optimized for production use

**Key Difference**: This proposal suggests replacing protobuf with efficient binary structs for activity sessions, logging only calculated metrics instead of raw sensor data.

---

## Table of Contents
1. [System Overview](#system-overview)
2. [Calculated Metrics Definition](#calculated-metrics-definition)
3. [Data Structures](#data-structures)
4. [BLE Protocol](#ble-protocol)
5. [Implementation Guidelines](#implementation-guidelines)
6. [Mobile Integration](#mobile-integration)

---

## System Overview

### Device Configuration
- **Primary Device**: Right shoe (master for synchronization)
- **Secondary Device**: Left shoe (synchronized to primary)
- **Raw Sampling**: 100Hz (pressure sensors + IMU)
- **Calculated Output**: 0.5-2Hz (context-dependent)
- **Time Sync**: <1ms between devices

### Data Flow
```
Raw Sensors (100Hz) → On-Device Processing → Calculated Metrics → BLE/Storage
                           ↓                        ↓
                    Feature Extraction      Real-time (1Hz)
                    Statistical Analysis    Logged (0.5Hz)
```

### Activity Types Supported
1. **Running**: Focus on pace, cadence, ground contact time
2. **Walking**: Step count, symmetry, stability  
3. **Training**: Form analysis, fatigue detection
4. **Recovery**: Gait quality, compensation patterns
5. **Custom**: User-defined metrics

---

## Calculated Metrics Definition

### 1. Core Step Metrics (Per Foot)

| Metric | Description | Unit | Range | Update Rate |
|---:|:---|:---:|:---:|:---:|
| Ground Contact Time | Duration foot is on ground | ms | 100-400 | Every step |
| Flight Time | Duration foot is in air | ms | 0-200 | Every step |
| Step Frequency | Cadence per foot | spm | 0-220 | Every step |
| Peak Force | Maximum pressure during step | N | 0-3000 | Every step |
| Loading Rate | Force increase rate at impact | N/s | 0-10000 | Every step |
| Push-off Power | Power during toe-off phase | W/kg | 0-50 | Every step |

### 2. Pressure Distribution

| Metric | Description | Unit | Range | Update Rate |
|---:|:---|:---:|:---:|:---:|
| Heel Pressure % | Relative heel loading | % | 0-100 | 10Hz during contact |
| Midfoot Pressure % | Relative midfoot loading | % | 0-100 | 10Hz during contact |
| Forefoot Pressure % | Relative forefoot loading | % | 0-100 | 10Hz during contact |
| Center of Pressure X | Medial-lateral position | mm | -50 to +50 | 10Hz during contact |
| Center of Pressure Y | Anterior-posterior position | mm | -100 to +100 | 10Hz during contact |
| Pressure Path Length | Total CoP movement | mm | 0-500 | Every step |

### 3. Motion Dynamics (IMU-based)

| Metric | Description | Unit | Range | Update Rate |
|---:|:---|:---:|:---:|:---:|
| Foot Strike Angle | Angle at initial contact | degrees | -30 to +30 | Every step |
| Pronation Angle | Maximum inward roll | degrees | -20 to +20 | Every step |
| Pronation Velocity | Speed of pronation | deg/s | 0-500 | Every step |
| Vertical Oscillation | Vertical movement amplitude | mm | 0-200 | Every 2 steps |
| Impact G-force | Peak acceleration at landing | g | 0-10 | Every step |
| Movement Smoothness | Quality of foot trajectory | score | 0-100 | Every second |

### 4. Gait Symmetry

| Metric | Description | Unit | Range | Update Rate |
|---:|:---|:---:|:---:|:---:|
| Contact Time Asymmetry | L/R contact time difference | % | -50 to +50 | Every 2-4 steps |
| Flight Time Asymmetry | L/R flight time difference | % | -50 to +50 | Every 2-4 steps |
| Force Asymmetry | L/R peak force difference | % | -50 to +50 | Every 2-4 steps |
| Step Length Asymmetry | L/R step length difference | % | -50 to +50 | Every 2-4 steps |
| Pronation Asymmetry | L/R pronation difference | degrees | -20 to +20 | Every 2-4 steps |

*Note: Negative values indicate left bias, positive indicate right bias*

### 5. Performance Indicators

| Metric | Description | Unit | Range | Update Rate |
|---:|:---|:---:|:---:|:---:|
| Running Efficiency | Energy cost estimate | J/kg/m | 0-10 | Every 5-10 seconds |
| Stride Length | Distance per complete gait cycle | cm | 0-300 | Every stride |
| Estimated Speed | Calculated from step metrics | m/s | 0-10 | Every second |
| Vertical Stiffness | Spring-mass model stiffness | kN/m | 0-100 | Every 10 steps |
| Duty Factor | Ground contact time ratio | % | 0-100 | Every 5 seconds |

### 6. Health & Risk Indicators

| Metric | Description | Unit | Range | Update Rate |
|---:|:---|:---:|:---:|:---:|
| Cumulative Impact Load | Total impact stress | AU | 0-10000 | Every 30 seconds |
| Fatigue Index | Performance degradation score | 0-100 | 0-100 | Every 30 seconds |
| Form Deterioration | Technique quality change | 0-100 | 0-100 | Every 30 seconds |
| Overstriding Indicator | Excessive stride detection | 0-100 | 0-100 | Every 10 steps |
| Lateral Instability | Side-to-side movement | 0-100 | 0-100 | Every 10 seconds |

---

## Data Structures

### Session Log File Format

The activity session file uses a compact binary format with the following structure:

**📌 Format Note**: These are **C structs with binary packing**, NOT protobuf messages. This provides:
- Fixed-size records for easy seeking
- Minimal overhead (no protobuf field tags)
- Direct memory mapping capability
- Predictable file sizes

#### File Header (Written Once at Start)
```c
typedef struct {
    uint32_t session_id;              // Unique session identifier
    uint32_t start_timestamp;         // Unix epoch time
    uint8_t  activity_type;           // RUNNING, WALKING, etc.
    uint8_t  firmware_version[3];     // Major.Minor.Patch
    uint16_t user_weight_kg;          // ×10 for 0.1kg precision
    uint16_t user_height_cm;          
    uint8_t  user_age;
    uint8_t  user_gender;             // 0=M, 1=F, 2=Other
    uint8_t  left_battery_pct;        // At start
    uint8_t  right_battery_pct;       // At start
    uint16_t calibration_id;          // Reference to calibration data
    uint8_t  reserved[8];             // Future use
} SessionHeader; // 32 bytes
```

#### Periodic Summary Records (Every 1-2 seconds)
```c
typedef struct {
    uint16_t delta_time_ms;           // Time since last record
    
    // Basic metrics (both feet averaged where applicable)
    uint16_t cadence_x2;              // Steps/min × 2 (0.5 precision)
    uint16_t avg_contact_time_ms;     
    uint16_t avg_flight_time_ms;
    uint16_t distance_delta_cm;       // Distance since last record
    
    // Per-foot summary
    FootSummary left_foot;            // 12 bytes
    FootSummary right_foot;           // 12 bytes
    
    // Composite metrics
    int8_t   balance_lr;              // -100 to +100
    uint8_t  form_score;              // 0-100
    uint8_t  efficiency_score;        // 0-100
    uint8_t  fatigue_level;           // 0-100
    
    // Flags for events
    uint8_t  event_flags;             // Bit flags for events
    uint8_t  reserved[3];
} PeriodicRecord; // 48 bytes

typedef struct {
    uint16_t peak_force;              // Normalized units
    uint8_t  heel_pct;                // 0-100
    uint8_t  midfoot_pct;             // 0-100
    uint8_t  forefoot_pct;            // 0-100
    int8_t   pronation_angle;         // -45 to +45 degrees
    uint16_t loading_rate;            // Normalized
    uint8_t  strike_pattern;          // 0=heel, 1=mid, 2=fore
    uint8_t  push_power;              // Normalized 0-255
    uint8_t  step_count;              // Steps in this period
    uint8_t  quality_score;           // 0-100
} FootSummary; // 12 bytes
```

#### Session Summary (Written Once at End)
```c
typedef struct {
    uint32_t end_timestamp;
    uint32_t total_duration_sec;
    uint32_t total_steps;
    uint32_t total_distance_m;
    uint16_t avg_pace_sec_per_km;
    uint16_t calories_burned;
    
    // Performance summary
    uint8_t  overall_form_score;      // 0-100
    uint8_t  max_fatigue_reached;     // 0-100
    uint8_t  injury_risk_score;       // 0-100
    uint8_t  consistency_score;       // 0-100
    
    // Key statistics
    uint16_t avg_cadence;
    uint16_t avg_contact_time_ms;
    int8_t   avg_balance;             // -100 to +100
    uint8_t  primary_strike_pattern;  // Most common
    
    // Alerts summary
    uint8_t  alert_counts[8];         // Count per alert type
    
    uint32_t crc32;                   // File integrity check
} SessionSummary; // 40 bytes
```

### Storage Estimates
- **Header**: 32 bytes
- **Records**: 48 bytes × 1800 (1 hour @ 2Hz) = 86.4 KB
- **Summary**: 40 bytes
- **Total for 1 hour**: ~87 KB uncompressed, ~45 KB compressed
- **Compared to raw**: 31.7 MB → 45 KB = 99.86% reduction

---

## BLE Protocol

### Service Architecture

```
Shoe Sensor Device
├── Device Information Service (0x180A) [Standard]
│   ├── Manufacturer Name (0x2A29)
│   ├── Model Number (0x2A24)
│   ├── Firmware Revision (0x2A26)
│   └── Battery Level (0x2A19)
│
└── Activity Metrics Service (Custom UUID: 00000001-7e57-4b8c-a577-b6e5c9b2e100)
    ├── Real-time Metrics (Notify, 1Hz)
    ├── Alert Notifications (Notify, Event-driven)
    ├── Control Point (Write)
    └── Session Status (Read/Notify)
```

### Real-time Metrics Characteristic (20 bytes, 1Hz)
```c
typedef struct __attribute__((packed)) {
    uint16_t delta_time_ms;         // 0-1: Time since last packet
    uint16_t cadence_x2;            // 2-3: Steps/min × 2
    uint16_t pace_sec_per_km;       // 4-5: Current pace
    uint16_t distance_m;            // 6-7: Total distance
    uint16_t contact_time_ms;       // 8-9: Average both feet
    uint8_t  form_score;            // 10: 0-100
    int8_t   balance_lr;            // 11: -100 to +100
    uint8_t  efficiency;            // 12: 0-100
    uint8_t  fatigue_level;         // 13: 0-100
    uint8_t  foot_strike;           // 14: 0=heel, 1=mid, 2=fore
    uint8_t  flags;                 // 15: Status flags
    uint32_t step_count;            // 16-19: Total steps
} RealtimeMetricsPacket;
```

### Alert Notification (20 bytes, Event-driven)
```c
typedef struct __attribute__((packed)) {
    uint16_t delta_time_ms;         // 0-1: Time since last packet
    uint8_t  alert_type;            // 2: See AlertType enum
    uint8_t  severity;              // 3: 1-10
    uint16_t current_value;         // 4-5: Metric that triggered
    uint16_t threshold_value;       // 6-7: Threshold exceeded
    uint8_t  affected_foot;         // 8: 0=both, 1=left, 2=right
    uint8_t  recommendation;        // 9: Coaching tip code
    uint8_t  duration_sec;          // 10: How long detected
    uint8_t  reserved[9];           // 11-19
} AlertPacket;

enum AlertType {
    ALERT_POOR_FORM = 0,
    ALERT_HIGH_ASYMMETRY = 1,
    ALERT_OVERSTRIDING = 2,
    ALERT_HIGH_IMPACT = 3,
    ALERT_FATIGUE = 4,
    ALERT_UNUSUAL_PATTERN = 5
};
```

### Control Point Commands
```c
// Commands from mobile app
#define CP_START_SESSION        0x01
#define CP_STOP_SESSION         0x02
#define CP_PAUSE_SESSION        0x03
#define CP_SET_USER_PROFILE     0x05
#define CP_REQUEST_SYNC         0x09

// Start session command structure
typedef struct __attribute__((packed)) {
    uint8_t  opcode;                // 0: = 0x01
    uint8_t  activity_type;         // 1: Running/Walking/etc
    uint16_t user_weight_kg;        // 2-3: ×10 for precision
    uint16_t user_height_cm;        // 4-5
    uint8_t  user_age;              // 6
    uint8_t  user_gender;           // 7: 0=M, 1=F
    uint32_t session_id;            // 8-11: From mobile app
    uint8_t  reserved[8];           // 12-19
} StartSessionCommand;
```

### Connection Parameters
- **Real-time Mode**: 15-30ms interval, 0 latency
- **Power Save Mode**: 100-200ms interval, 4 latency
- **MTU**: Negotiate maximum (247 bytes) for efficiency

---

## Implementation Guidelines

### Processing Pipeline

#### 1. Raw Data Collection (100Hz)
```c
// Sensor buffers
typedef struct {
    float pressure[8];              // 8 pressure channels
    float quaternion[4];            // Orientation
    float gyro[3];                  // Angular velocity
    float linear_acc[3];            // Linear acceleration
    uint32_t timestamp;
} SensorSample;

#define BUFFER_SIZE 200             // 2 seconds at 100Hz
```

#### Timestamp Management
```c
// Delta timestamp tracking
typedef struct {
    uint32_t last_packet_time;      // Last transmission time
    uint32_t session_start_time;    // Absolute start reference
    uint16_t max_delta_ms;          // Maximum allowed delta (overflow protection)
} TimestampTracker;

// Calculate delta time for packet
uint16_t calculate_delta_time(TimestampTracker* tracker, uint32_t current_time) {
    uint32_t delta = current_time - tracker->last_packet_time;
    
    // Handle overflow or excessive gaps
    if (delta > tracker->max_delta_ms) {
        delta = tracker->max_delta_ms;
    }
    
    tracker->last_packet_time = current_time;
    return (uint16_t)delta;
}
```

#### 2. Step Detection
```c
// State machine for gait phase detection
typedef enum {
    SWING_PHASE,
    LOADING_PHASE,
    MIDSTANCE_PHASE,
    PUSH_OFF_PHASE
} GaitPhase;

// Detect ground contact from pressure sum
bool detect_ground_contact(float pressure_sum) {
    const float CONTACT_THRESHOLD = 50.0;  // Newtons
    const float HYSTERESIS = 10.0;
    
    static bool in_contact = false;
    
    if (!in_contact && pressure_sum > CONTACT_THRESHOLD + HYSTERESIS) {
        in_contact = true;
        return true;  // Contact started
    } else if (in_contact && pressure_sum < CONTACT_THRESHOLD - HYSTERESIS) {
        in_contact = false;
        return true;  // Contact ended
    }
    return false;  // No change
}
```

#### 3. Pressure Distribution Calculation
```c
// Map 8 channels to anatomical regions
void calculate_pressure_distribution(float channels[8], 
                                   PressureDistribution* dist) {
    // Channel mapping (device-specific)
    float heel = channels[0] + channels[1];
    float midfoot = channels[2] + channels[3] + channels[4];
    float forefoot = channels[5] + channels[6] + channels[7];
    float total = heel + midfoot + forefoot;
    
    if (total > 0) {
        dist->heel_pct = (heel / total) * 100;
        dist->midfoot_pct = (midfoot / total) * 100;
        dist->forefoot_pct = (forefoot / total) * 100;
    }
}
```

#### 4. Pronation Analysis
```c
// Combine IMU and pressure for pronation
float calculate_pronation(Quaternion* q, float pressure_lateral_shift) {
    // Extract roll angle from quaternion
    float roll = atan2(2*(q->w*q->x + q->y*q->z), 
                      1 - 2*(q->x*q->x + q->y*q->y));
    
    // Combine with pressure shift
    float pronation = roll * RAD_TO_DEG + 
                     pressure_lateral_shift * PRESSURE_PRONATION_FACTOR;
    
    return constrain(pronation, -45, 45);
}
```

#### 5. Asymmetry Calculation
```c
// Normalized asymmetry score
int8_t calculate_asymmetry(float left_value, float right_value) {
    float avg = (left_value + right_value) / 2;
    if (avg == 0) return 0;
    
    float diff = left_value - right_value;
    float asymmetry = (diff / avg) * 100;
    
    return constrain(asymmetry, -100, 100);
}
```

### Memory Requirements
- **Static Allocation**: ~40KB total
  - Sensor buffers: 25.6KB
  - Processing buffers: 8KB
  - Metric storage: 4KB
  - BLE queue: 2KB
- **No dynamic allocation** (for reliability)
- **Stack usage**: <2KB maximum depth

### Power Optimization
1. **Adaptive Sampling**: Reduce rate during low activity
2. **Efficient Processing**: Use fixed-point math where possible
3. **Smart Sleep**: Enter low-power mode when stationary
4. **Batch Operations**: Process multiple samples together

### Calibration Requirements
1. **Pressure Sensors**: Zero offset, sensitivity, crosstalk matrix
2. **IMU**: Gyro bias, accelerometer scale/bias, alignment
3. **Time Sync**: <1ms synchronization between feet
4. **User-Specific**: Height/weight for accurate calculations

---

## Mobile Integration

### Required Mobile App Features

#### 1. Real-time Display
- Primary metrics dashboard
- Color-coded quality indicators  
- Trend visualization
- Audio/haptic feedback

#### 2. Session Management
```javascript
// Example session flow
async function startActivitySession(activityType, userProfile) {
    // Connect to both shoes
    await connectToShoes();
    
    // Send user profile
    await sendControlCommand(CP_SET_USER_PROFILE, userProfile);
    
    // Start session
    const sessionId = generateSessionId();
    await sendControlCommand(CP_START_SESSION, {
        activityType,
        sessionId,
        ...userProfile
    });
    
    // Enable notifications
    await enableMetricsNotifications();
}
```

#### 3. Data Processing
- Additional smoothing/filtering
- Trend analysis over time
- Comparison with historical data
- Export capabilities

#### 4. Coaching Features
- Real-time form corrections
- Personalized recommendations
- Training plan integration
- Progress tracking

### API Requirements
The mobile app should implement:
- BLE connection management with auto-reconnection
- Data buffering for connection interruptions
- Local storage of session data
- Cloud sync capabilities
- Sharing/export functions

---

## Benefits Summary

### For Athletes
- **Immediate Feedback**: Real-time coaching during activities
- **Injury Prevention**: Early warning of risky patterns
- **Performance Optimization**: Data-driven training insights
- **Progress Tracking**: Objective measurement of improvement

### For Coaches
- **Remote Monitoring**: Live athlete data access
- **Training Optimization**: Evidence-based programming
- **Team Management**: Compare athlete metrics
- **Injury Risk Assessment**: Proactive intervention

### For Medical Professionals
- **Gait Analysis**: Clinical-grade movement data
- **Rehabilitation Tracking**: Objective recovery metrics
- **Asymmetry Detection**: Return-to-play decisions
- **Load Management**: Prevent overuse injuries

---

## Future Enhancements

1. **Machine Learning Integration**
   - Personalized form recommendations
   - Predictive injury risk modeling
   - Automatic activity classification

2. **Extended Metrics**
   - Heart rate variability correlation
   - Environmental adaptation (surface, weather)
   - Multi-sport profiles

3. **Cloud Analytics**
   - Population-level insights
   - Benchmarking against peers
   - Long-term trend analysis

---

## Conclusion

This specification provides a complete framework for transforming raw sensor data into actionable insights while dramatically reducing data storage and transmission requirements. The system balances scientific validity with practical implementation constraints, delivering real value to users across the performance spectrum.

By processing data on-device and transmitting only calculated metrics, we achieve:
- 99.86% reduction in data volume
- Real-time performance feedback
- Extended battery life
- Scalable architecture for future enhancements

### Data Summary: What Gets Sent vs What Gets Logged

The following tables clearly outline the data transmitted via BLE characteristics versus data stored in the activity log file:

#### BLE Transmitted Data (Real-time)

| **Characteristic** | **Update Rate** | **Data Fields** | **Size** | **Purpose** |
|---:|:---|:---|:---:|:---|
| **Real-time Metrics** | 1Hz | • Delta time (ms)<br>• Cadence (steps/min)<br>• Pace (sec/km)<br>• Distance (m)<br>• Contact time (ms)<br>• Form score (0-100)<br>• Balance L/R (-100 to +100)<br>• Efficiency (0-100)<br>• Fatigue level (0-100)<br>• Foot strike pattern<br>• Status flags<br>• Total step count | 20 bytes | Live performance feedback during activity |
| **Alert Notifications** | Event-driven | • Delta time (ms)<br>• Alert type<br>• Severity (1-10)<br>• Current value<br>• Threshold value<br>• Affected foot<br>• Recommendation code<br>• Duration detected | 20 bytes | Real-time warnings for form issues, asymmetry, fatigue |
| **Session Status** | On change | • Session state<br>• Activity type<br>• Session ID<br>• Elapsed time<br>• Step count<br>• Distance<br>• Battery levels<br>• Memory usage | 20 bytes | Session management and device status |

#### Activity Log File Data (Stored on Device)

| **Section** | **Frequency** | **Data Fields** | **Size** | **Purpose** |
|---:|:---|:---|:---:|:---|
| **Session Header** | Once at start | • Session ID<br>• Start timestamp<br>• Activity type<br>• User profile (weight, height, age, gender)<br>• Firmware version<br>• Battery levels<br>• Calibration reference | 32 bytes | Session identification and context |
| **Periodic Records** | Every 1-2 sec | • Delta time (ms)<br>• Cadence<br>• Contact/flight times<br>• Distance increment<br>• **Per foot:**<br>&nbsp;&nbsp;- Peak force<br>&nbsp;&nbsp;- Pressure distribution (heel/mid/forefoot %)<br>&nbsp;&nbsp;- Pronation angle<br>&nbsp;&nbsp;- Loading rate<br>&nbsp;&nbsp;- Strike pattern<br>&nbsp;&nbsp;- Push-off power<br>&nbsp;&nbsp;- Step count<br>&nbsp;&nbsp;- Quality score<br>• Balance L/R<br>• Form score<br>• Efficiency score<br>• Fatigue level<br>• Event flags | 48 bytes | Detailed biomechanical analysis |
| **Session Summary** | Once at end | • End timestamp<br>• Total duration<br>• Total steps & distance<br>• Average pace<br>• Calories burned<br>• Overall scores (form, fatigue, injury risk)<br>• Average metrics<br>• Alert counts by type<br>• CRC checksum | 40 bytes | Session statistics and summary |

#### Key Differences

| **Aspect** | **BLE (Real-time)** | **Log File** |
|---:|:---|:---|
| **Update Rate** | 1Hz (metrics), event-driven (alerts) | 0.5-2Hz |
| **Data Detail** | Essential metrics only | Complete biomechanical data |
| **Per-foot Data** | Averaged/combined | Separate L/R measurements |
| **Pressure Details** | Not included | Full distribution (heel/mid/forefoot) |
| **Storage Impact** | ~72 KB/hour transmitted | ~45 KB/hour stored |
| **Primary Use** | Live feedback & coaching | Post-activity analysis |

### Mobile Team Integration Notes

1. **BLE Connection**: Implement auto-reconnection and buffering for the three characteristics
2. **Real-time Display**: Focus on the 1Hz metrics for live dashboard
3. **Alert Handling**: Show notifications immediately with recommended actions
4. **Session Management**: Use control point for start/stop/pause commands
5. **Data Sync**: Activity log files can be transferred post-session for detailed analysis
6. **3D Visualization**: When logging is NOT active, use the dedicated 3D Orientation Service (UUID: `0c372ec0-27eb-437e-bef4-775aefaf3c97`) for real-time quaternion data at 20Hz

### Alternative Real-time Data Sources

When activity logging is not active, the following services provide real-time data:

1. **3D Orientation Service** (Primary device only)
   - UUID: `0c372ec0-27eb-437e-bef4-775aefaf3c97`
   - Characteristic: `0c372ec1-27eb-437e-bef4-775aefaf3c97`
   - Update Rate: 20Hz
   - Data: Quaternions for both shoes + ground contact state
   - Purpose: Real-time 3D visualization without full logging overhead

2. **Information Service BHI360 Characteristics** (When logging active)
   - BHI360 3D Mapping: Includes quaternion data at lower rate
   - BHI360 Linear Accel: Motion dynamics
   - Update Rate: Variable based on activity

### Metric Accuracy and Feasibility

Based on the combination of BHI360 IMU and 8-channel pressure sensors per foot, here's the realistic accuracy we can achieve:

#### High Accuracy Metrics (±5-10% error)
- **Contact/Flight Times**: Direct measurement from pressure threshold
- **Step Count & Cadence**: Reliable detection from pressure + IMU
- **Pressure Distribution**: Direct measurement from 8 channels
- **Foot Strike Pattern**: Clear from initial contact location
- **Balance/Asymmetry**: Direct comparison between feet
- **Impact Forces**: Peak acceleration from IMU
- **Pronation Angle**: Quaternion-based with pressure validation

#### Medium Accuracy Metrics (±15-25% error)
- **Speed/Distance**: Estimated from cadence × stride length model
- **Loading Rate**: Calculated from force rise time
- **Vertical Oscillation**: Double integration of acceleration
- **Stride Length**: Model-based estimation

#### Derived Metrics (Relative accuracy)
- **Form Score**: Weighted combination of other metrics
- **Efficiency Score**: Model-based, requires calibration
- **Fatigue Level**: Pattern detection over time
- **Push-off Power**: Force × estimated velocity

**Important Notes for Mobile Team:**
1. **Calibration Required**: Speed/distance accuracy improves with user calibration
2. **Relative vs Absolute**: Derived scores are best used for tracking changes
3. **Baseline Establishment**: Fatigue detection needs 2-3 minutes to establish baseline
4. **Environmental Factors**: Accuracy may vary with surface type and running style

The proposed system is ready for implementation and will provide the mobile team with valuable biomechanical insights while maintaining realistic accuracy expectations.
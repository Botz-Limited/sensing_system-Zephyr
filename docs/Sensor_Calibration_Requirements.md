# Sensor Calibration Requirements Document

**Version:** 1.0  
**Date:** December 2024  
**Purpose:** Define calibration procedures for all sensors in the dual-device biomechanical sensing system  
**Audience:** Firmware developers and mobile app developers

---

## Executive Summary

This document outlines the calibration requirements for the sensing device's three main sensor systems:
1. **Foot Pressure Sensors** (8 points per foot)
2. **BHI360 Motion Sensor** (IMU with 6-axis + sensor fusion)
3. **Weight Measurement System** (using combined pressure sensors)

Each sensor requires specific calibration procedures to ensure accurate measurements. Some calibrations are performed during manufacturing, while others require user participation through the mobile app onboarding process.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Foot Pressure Sensor Calibration](#2-foot-pressure-sensor-calibration)
3. [BHI360 Motion Sensor Calibration](#3-bhi360-motion-sensor-calibration)
4. [Weight Measurement Calibration](#4-weight-measurement-calibration)
5. [Mobile App Integration](#5-mobile-app-integration)
6. [Calibration Data Storage](#6-calibration-data-storage)
7. [Implementation Recommendations](#7-implementation-recommendations)

---

## 1. Overview

### 1.1 Calibration Types

| Type | When Performed | Who Performs | Storage |
|------|----------------|--------------|---------|
| **Factory Calibration** | Manufacturing | Production line | Flash (permanent) |
| **User Calibration** | First use / Onboarding | End user via app | Flash (updateable) |
| **Runtime Calibration** | Each power-on | Automatic | RAM (temporary) |

### 1.2 Calibration Goals

- **Accuracy**: Achieve specified measurement accuracy for each sensor
- **Consistency**: Ensure repeatable measurements across devices
- **User Experience**: Minimize calibration complexity for end users
- **Adaptability**: Account for sensor drift and environmental changes

---

## 2. Foot Pressure Sensor Calibration

### 2.1 Sensor Configuration
- **Type**: Force Sensitive Resistors (FSR) or similar
- **Count**: 8 sensors per foot (16 total)
- **Layout**: Heel (2), Midfoot (3), Forefoot (3)
- **ADC Resolution**: 12-bit (0-4095)

### 2.2 Calibration Requirements

#### 2.2.1 Zero Calibration (Factory)
**Purpose**: Remove sensor offset when no load is applied

**Procedure**:
1. Place device on flat surface with no load
2. Read all 16 sensors for 3 seconds
3. Calculate average for each sensor
4. Store offsets in flash

**Data Structure**:
```c
typedef struct {
    uint16_t zero_offset[16];  // ADC offset for each sensor
    uint32_t timestamp;        // Calibration timestamp
} pressure_zero_cal_t;
```

#### 2.2.2 Dynamic Range Calibration (User - Onboarding)
**Purpose**: Determine optimal ADC range for individual user weight/gait

**Procedure**:
1. User wears shoes and stands normally
2. Collect pressure data for 5 seconds
3. User performs specific movements:
   - Shift weight to left foot (2 seconds)
   - Shift weight to right foot (2 seconds)
   - Stand on toes (2 seconds)
   - Stand on heels (2 seconds)
4. Calculate max pressure per sensor
5. Set gain/scaling factors

**Mobile App Commands**:
```
// Start dynamic range calibration
Control Service -> Write 0x01 to Calibration Trigger

// Send calibration movements
Control Service -> Write command strings:
"CAL_PRESSURE:START"
"CAL_PRESSURE:WEIGHT_LEFT"
"CAL_PRESSURE:WEIGHT_RIGHT"
"CAL_PRESSURE:TOES"
"CAL_PRESSURE:HEELS"
"CAL_PRESSURE:COMPLETE"
```

**Data Structure**:
```c
typedef struct {
    uint16_t max_pressure[16];     // Max ADC per sensor
    float scale_factor[16];        // Scaling to normalize
    uint8_t user_weight_kg;        // For reference
} pressure_range_cal_t;
```

#### 2.2.3 Nonlinearity Compensation (Factory - Optional)
**Purpose**: Correct for FSR nonlinear response

**Options**:
1. **Simple Linear**: Assume linear response (default)
2. **Polynomial Fit**: Force = a×ADC² + b×ADC + c
3. **Lookup Table**: Pre-characterized response curve

---

## 3. BHI360 Motion Sensor Calibration

### 3.1 Sensor Configuration
- **Accelerometer**: ±16g range
- **Gyroscope**: ±2000 dps range
- **Virtual Sensors**: Quaternion, Linear Acceleration, Step Counter
- **Mounting**: Vertical orientation in shoe

### 3.2 Calibration Requirements

#### 3.2.1 Orientation Calibration (User - Onboarding)
**Purpose**: Determine exact mounting orientation in shoe

**Procedure**:
1. User places shoes on flat surface
2. Shoes must be level and not moving
3. Collect quaternion data for 3 seconds
4. Calculate rotation matrix from sensor to world frame
5. Store orientation offset

**Mobile App Commands**:
```
// Trigger BHI360 orientation calibration
Control Service -> Write 0x01 to BHI360 Calibration Trigger
```

**Current Implementation**:
```c
// Already implemented in motion_sensor.cpp
void trigger_bhi360_calibration() {
    // Saves quaternion offset when shoes are flat
    // Stores in "bhi360_calibration.dat"
}
```

#### 3.2.2 Gyroscope Bias Calibration (Runtime)
**Purpose**: Remove gyroscope drift

**Procedure**:
1. Detect stationary periods (no movement)
2. Average gyroscope readings
3. Update bias compensation
4. Apply during motion processing

**Implementation**: Already handled internally by BHI360

#### 3.2.3 Accelerometer Calibration (Factory)
**Purpose**: Ensure accurate acceleration measurements

**Options**:
1. **6-Point Calibration**: Place sensor in 6 orientations (±X, ±Y, ±Z)
2. **Single-Point**: Use gravity reference when flat
3. **BHI360 Internal**: Rely on Bosch's factory calibration

### 3.3 Additional BHI360 Calibrations

#### 3.3.1 Magnetometer Calibration (User - Optional)
**Purpose**: Compass heading accuracy

**Procedure**:
1. Figure-8 motion pattern
2. Rotate device in all orientations
3. BHI360 calculates hard/soft iron compensation

**Note**: May not be needed for gait analysis

#### 3.3.2 Step Detection Tuning (User - Optional)
**Purpose**: Optimize step counting for user's gait

**Parameters**:
- Minimum peak height
- Minimum peak distance
- Debounce time

---

## 4. Weight Measurement Calibration

### 4.1 System Configuration
- Uses all 16 pressure sensors
- Requires person standing still
- Outputs total body weight

### 4.2 Calibration Requirements

#### 4.2.1 Basic Weight Calibration (User - Onboarding)
**Purpose**: Establish ADC-to-weight conversion

**Current Procedure**:
1. User enters known weight in app
2. User stands still on both feet
3. System collects pressure data
4. Calculates scale factor: Force(N) = Weight(kg) × 9.81 / Total_ADC

**Mobile App Commands**:
```
// Send known weight and trigger calibration
typedef struct {
    float known_weight_kg;
} weight_calibration_step_t;

Control Service -> Write weight_calibration_step_t to Weight Calibration Trigger
```

#### 4.2.2 Multi-Point Calibration (Factory - Recommended)
**Purpose**: Improve accuracy across weight range

**Procedure**:
1. Use 3 reference weights (e.g., 50kg, 70kg, 90kg)
2. Collect ADC readings for each
3. Fit polynomial or piecewise linear model
4. Store coefficients

**Data Structure**:
```c
typedef struct {
    float scale_factor;      // Linear scale
    float nonlinear_a;       // x² coefficient  
    float nonlinear_b;       // x coefficient
    float nonlinear_c;       // Constant
    float temp_coeff;        // Temperature compensation
    float temp_ref;          // Reference temperature
    bool is_calibrated;
} weight_calibration_data_t;
```

#### 4.2.3 Temperature Compensation (Factory - Optional)
**Purpose**: Correct for temperature-induced drift

**Procedure**:
1. Measure sensor response at different temperatures
2. Calculate temperature coefficient
3. Apply compensation: Force_corrected = Force × (1 + α×(T-T_ref))

---

## 5. Mobile App Integration

### 5.1 Onboarding Flow

```
1. Welcome Screen
   └─> "Let's calibrate your sensors for optimal performance"

2. Foot Sensor Calibration
   ├─> "Please put on both shoes"
   ├─> "Stand normally for 5 seconds"
   ├─> "Shift weight to left foot"
   ├─> "Shift weight to right foot"
   ├─> "Stand on your toes"
   ├─> "Stand on your heels"
   └─> "Calibration complete!"

3. Motion Sensor Calibration
   ├─> "Place shoes on flat surface"
   ├─> "Ensure shoes are level and still"
   ├─> "Wait 3 seconds..."
   └─> "Orientation calibrated!"

4. Weight Calibration
   ├─> "Enter your current weight: [___] kg"
   ├─> "Stand still on both feet"
   ├─> "Measuring..."
   └─> "Weight calibration saved!"

5. Completion
   └─> "All sensors calibrated! You're ready to go."
```

### 5.2 Calibration Status Monitoring

The app should query calibration status on connection:

```
// Check calibration status
Information Service -> Read Status characteristic
- Bit 9 (0x200): CALIBRATING flag

// Get calibration data
Control Service -> Write "GET_CAL_STATUS" command
Response via notification with calibration flags
```

### 5.3 Recalibration Triggers

App should prompt recalibration when:
- First time setup
- Significant weight change (>5kg)
- Poor data quality detected
- User requests via settings
- After firmware update

---

## 6. Calibration Data Storage

### 6.1 Storage Locations

| Calibration Type | File Path | Format | Size |
|-----------------|-----------|---------|------|
| Pressure Zero | `/lfs1/config/pressure_zero.dat` | Binary | 32 bytes |
| Pressure Range | `/lfs1/config/pressure_range.dat` | Binary | 64 bytes |
| BHI360 Orientation | `/lfs1/config/bhi360_calibration.dat` | Binary | 64 bytes |
| Weight Calibration | `/lfs1/config/weight_calibration.dat` | Binary | 32 bytes |

### 6.2 Data Persistence

- All calibration data persists across power cycles
- Stored in LittleFS partition
- Loaded automatically on boot
- Can be reset to factory defaults

### 6.3 Firmware Interface

```c
// Save calibration
generic_message_t msg = {
    .type = MSG_TYPE_SAVE_CALIBRATION,
    .data.calibration = cal_data
};
k_msgq_put(&data_msgq, &msg, K_NO_WAIT);

// Load calibration
generic_message_t msg = {
    .type = MSG_TYPE_REQUEST_CALIBRATION
};
k_msgq_put(&data_msgq, &msg, K_NO_WAIT);
```

---

## 7. Implementation Recommendations

### 7.1 Priority Order

1. **High Priority** (Required for basic function):
   - BHI360 orientation calibration (already implemented)
   - Basic weight calibration with known weight (already implemented)
   - Pressure sensor zero calibration

2. **Medium Priority** (Improves accuracy):
   - Pressure sensor dynamic range calibration
   - Multi-point weight calibration
   - Temperature compensation

3. **Low Priority** (Nice to have):
   - Pressure sensor nonlinearity compensation
   - Step detection parameter tuning
   - Magnetometer calibration

### 7.2 Development Phases

**Phase 1 - Basic Calibration** (Current):
- ✅ BHI360 orientation calibration
- ✅ Single-point weight calibration
- ⏳ Pressure zero calibration

**Phase 2 - Enhanced Accuracy**:
- Pressure dynamic range calibration
- Multi-point weight calibration
- Calibration status reporting

**Phase 3 - Advanced Features**:
- Temperature compensation
- Nonlinearity correction
- Auto-recalibration detection

### 7.3 Testing Requirements

Each calibration procedure should be tested for:
- Repeatability (±2% variation)
- Accuracy (meets specification)
- User experience (< 2 minutes total)
- Error handling (motion detection, timeout)
- Data persistence (survives reboot)

### 7.4 Error Handling

Mobile app should handle:
- Motion during calibration → Retry
- Timeout (>30 seconds) → Abort with message
- Invalid data → Request user verification
- Communication loss → Save partial data

---

## Appendix A: Calibration Commands Summary

| Command | Service | Characteristic | Data | Purpose |
|---------|---------|----------------|------|---------|
| Start Pressure Cal | Control | Command | "CAL_PRESSURE:START" | Begin pressure calibration |
| BHI360 Cal | Control | BHI360 Calibration | 0x01 | Trigger orientation cal |
| Weight Cal | Control | Weight Calibration | weight_calibration_step_t | Calibrate with known weight |
| Get Status | Control | Command | "GET_CAL_STATUS" | Query calibration state |
| Save Cal | Internal | Message Queue | MSG_TYPE_SAVE_CALIBRATION | Store to flash |

## Appendix B: Future Enhancements

1. **Cloud Backup**: Store calibration data in cloud for device replacement
2. **Auto-Calibration**: Detect drift and recalibrate automatically
3. **Cross-Device Sync**: Share calibration between left/right devices
4. **Machine Learning**: Improve calibration using population data
5. **Diagnostic Mode**: Validate calibration with known test patterns

---

**End of Document**
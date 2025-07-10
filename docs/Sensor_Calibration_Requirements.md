# Sensor Calibration Requirements Document

**Version:** 1.2  
**Date:** July 2025  
**Purpose:** Define calibration procedures for all sensors in the dual-device sensing system  

---

## Summary

This document outlines the calibration requirements for the sensing device's sensor system:
1. **Foot Pressure Sensors** (8 points per foot)
2. **BHI360 Motion Sensor** (IMU with 6-axis + sensor fusion)
3. **Weight Measurement System** (using combined pressure sensors)

The calibration process has been optimized to minimize user friction during onboarding while maintaining measurement accuracy. The approach intelligently combines calibration steps and leverages the natural device usage flow.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Optimized Calibration Flow](#2-optimized-calibration-flow)
3. [Foot Pressure Sensor Calibration](#3-foot-pressure-sensor-calibration)
4. [BHI360 Motion Sensor Calibration](#4-bhi360-motion-sensor-calibration)
5. [Weight Measurement Calibration](#5-weight-measurement-calibration)
6. [Mobile App Integration](#6-mobile-app-integration)
7. [Calibration Data Storage](#7-calibration-data-storage)
8. [Implementation Recommendations](#8-implementation-recommendations)

---

## 1. Overview

### 1.1 Calibration Types

| Type | When Performed | Who Performs | Storage |
|------|----------------|--------------|---------|
| **Factory Calibration** | Manufacturing | Production line | Flash (permanent) |
| **User Calibration** | First use / Onboarding | End user via app | Flash (updateable) |
| **Runtime Calibration** | Each power-on | Automatic | RAM (temporary) |

### 1.2 Calibration Goals

- **Minimal User Friction**: Complete essential calibration seamlessly
- **Accuracy**: Achieve specified measurement accuracy for each sensor
- **Consistency**: Ensure repeatable measurements across devices
- **User Experience**: Natural flow that doesn't feel like "calibration"

### 1.3 Optimized Approach

**Previous approach**: 16+ seconds with multiple movements  
**New approach**: Automatic + 5 seconds of standing

---

## 2. Optimized Calibration Flow

### 2.1 Intelligent Calibration Sequence

The calibration is split into two natural phases:

**Phase 1: Automatic Zero Calibration** (Before wearing)
- Triggered when devices are powered on/connected
- Captures pressure sensor zero offsets
- No user action required

**Phase 2: Combined Active Calibration** (5 seconds while wearing)
- BHI360 orientation reference
- Pressure sensor maximum range detection
- Weight calculation (using pre-entered weight value)

### 2.2 Complete Onboarding Flow

```
1. App Setup
   ├─> User enters profile (including weight)
   └─> App connects to devices

2. Automatic Calibration (Hidden)
   ├─> Devices detect they're not being worn
   ├─> Pressure sensor zero calibration runs
   └─> App shows "Setting up your shoes..."

3. Active Calibration (5 seconds)
   ├─> "Please put on both shoes"
   ├─> "Stand normally on a flat surface"
   ├─> [5-second countdown]
   ├─> Simultaneous:
   │   ├─> BHI360 orientation calibration
   │   ├─> Pressure max range detection
   │   └─> Weight scale factor calculation
   └─> "Perfect! Your shoes are ready"
```

---

## 3. Foot Pressure Sensor Calibration

### 3.1 Sensor Configuration
- **Type**: Force Sensitive Resistors (FSR) or similar
- **Count**: 8 sensors per foot (16 total)
- **Layout**: Heel (2), Midfoot (3), Forefoot (3)
- **ADC Resolution**: 12-bit (0-4095)

### 3.2 Two-Phase Calibration

#### 3.2.1 Phase 1: Zero Offset Calibration (Automatic)
**Purpose**: Capture true zero values when no pressure applied

**Trigger**: Automatically when:
- Devices first powered on
- Connected to app but not worn
- No pressure detected for >2 seconds

**Procedure**:
```c
void auto_zero_calibration() {
    if (detect_not_worn()) {  // All sensors < threshold
        for (int i = 0; i < 16; i++) {
            zero_offset[i] = read_adc_average(i, 100);  // 100 samples
        }
        zero_cal_complete = true;
        notify_app("ZERO_CAL_COMPLETE");
    }
}
```

**Data Captured**:
```c
typedef struct {
    uint16_t zero_offset[16];      // ADC value when no load
    uint32_t timestamp;            // When calibrated
    bool valid;                    // Calibration successful
} pressure_zero_cal_t;
```

#### 3.2.2 Phase 2: Range Calibration (During 5-second stand)
**Purpose**: Determine operating range with user's weight

**Procedure**:
1. User wearing shoes, standing normally
2. Collect pressure data for 5 seconds
3. Calculate:
   - Maximum values per sensor
   - Pressure distribution pattern
   - Total force for weight verification

**Data Structure**:
```c
typedef struct {
    uint16_t max_values[16];       // Peak ADC per sensor
    uint8_t distribution_pattern;   // Foot type classification
    float total_force;             // Sum of all sensors
    uint32_t user_weight_g;        // From user profile
} pressure_range_cal_t;
```

---

## 4. BHI360 Motion Sensor Calibration

### 4.1 Single-Phase Calibration (During 5-second stand)

**No changes needed** - BHI360 calibration works perfectly during the 5-second standing phase:

**Procedure**:
1. User stands still with shoes on feet
2. Collect quaternion data
3. Calculate mounting orientation
4. Store calibration data

**Implementation**:
```c
// Runs during the 5-second combined calibration
void bhi360_orientation_calibration() {
    // Average quaternion over 5 seconds
    // Calculate rotation offset
    // Save to persistent storage
}
```

---

## 5. Weight Measurement Calibration

### 5.1 Integrated into 5-Second Calibration

Since the user has already entered their weight during profile setup, we can calculate the scale factor during the same 5-second standing period.

#### 5.1.1 Weight Scale Calculation
**Prerequisites**:
- User weight entered in app profile
- Zero calibration completed (Phase 1)
- User standing normally (Phase 2)

**Procedure**:
```c
void calculate_weight_scale_factor() {
    // During 5-second stand
    float total_adc = 0;
    
    // Sum all pressure sensors
    for (int i = 0; i < 16; i++) {
        total_adc += (adc_value[i] - zero_offset[i]);
    }
    
    // Calculate scale factor
    weight_cal.scale_factor = user_weight_g / total_adc;
    weight_cal.is_calibrated = true;
}
```

**Benefits**:
- No additional calibration step
- Immediate weight tracking capability
- Verification of entered weight

---

## 6. Mobile App Integration

### 6.1 Seamless Onboarding Flow

```
1. Profile Setup
   ├─> "Enter your details"
   ├─> Height: [___] cm
   ├─> Weight: [___] kg
   └─> [Continue]

2. Connect Devices
   ├─> "Turn on your smart shoes"
   ├─> [Searching...]
   ├─> "Found! Setting up..." (Hidden zero cal happens here)
   └─> [✓] Ready

3. Quick Calibration (5 seconds)
   ├─> "Put on both shoes"
   ├─> "Stand normally on a flat surface"
   ├─> [5-second countdown with progress]
   └─> "All set! Let's get started"
```

### 6.2 Backend Calibration Sequence

```javascript
// App automatically manages calibration phases
async function setupDevices() {
    // Phase 1: Connect and auto-zero
    await connectToDevices();
    // Devices auto-detect not worn and zero calibrate
    await waitForNotification("ZERO_CAL_COMPLETE");
    
    // Phase 2: User calibration
    showInstruction("Please put on both shoes");
    await sendCommand("START_COMBINED_CALIBRATION", {
        duration_ms: 5000,
        user_weight_g: userProfile.weight * 1000
    });
    
    await waitForNotification("CALIBRATION_COMPLETE");
}
```

### 6.3 Calibration Commands

```c
// Command structure for combined calibration
typedef struct {
    uint16_t duration_ms;      // 5000ms standard
    uint32_t user_weight_g;    // User weight in grams
    bool calibrate_bhi360;     // true
    bool calibrate_pressure;   // true (range only)
    bool calculate_weight;     // true
} combined_cal_params_t;
```

---

## 7. Calibration Data Storage

### 7.1 Storage Locations

| Calibration Type | File Path | Format | Size |
|-----------------|-----------|---------|------|
| Pressure Zero | `/lfs1/config/pressure_zero.dat` | Binary | 32 bytes |
| Pressure Range | `/lfs1/config/pressure_range.dat` | Binary | 64 bytes |
| BHI360 Orientation | `/lfs1/config/bhi360_calibration.dat` | Binary | 64 bytes |
| Weight Scale | `/lfs1/config/weight_calibration.dat` | Binary | 32 bytes |

### 7.2 Automatic Loading

```c
void load_calibrations_on_boot() {
    // Always load zero calibration
    load_pressure_zero_cal();
    
    // Load others if they exist
    if (file_exists("/lfs1/config/pressure_range.dat")) {
        load_pressure_range_cal();
    }
    
    // Check if recalibration needed
    if (!zero_cal_valid() || cal_age_days() > 30) {
        request_recalibration();
    }
}
```

---

## 8. Implementation Recommendations

### 8.1 Key Implementation Details

**Automatic Zero Detection**:
```c
bool detect_not_worn() {
    const uint16_t WORN_THRESHOLD = 100;  // ADC counts
    int sensors_active = 0;
    
    for (int i = 0; i < 16; i++) {
        if (read_adc(i) > WORN_THRESHOLD) {
            sensors_active++;
        }
    }
    
    return (sensors_active < 2);  // Less than 2 sensors active
}
```

**Combined Calibration Handler**:
```c
void handle_combined_calibration(combined_cal_params_t *params) {
    // Start all calibrations
    start_pressure_range_cal();
    trigger_bhi360_calibration();
    start_weight_calculation(params->user_weight_g);
    
    // Single timer for all
    k_timer_start(&cal_timer, K_MSEC(params->duration_ms));
}
```

### 8.2 Testing Requirements

1. **Zero Calibration Accuracy**:
   - Verify auto-detection of "not worn" state
   - Confirm zero values are stable
   - Test with different surface materials

2. **Combined Calibration**:
   - Verify 5-second timing
   - Test weight calculation accuracy
   - Confirm BHI360 orientation capture

3. **User Experience**:
   - Seamless transition between phases
   - Clear instructions
   - No perception of "calibration"

### 8.3 Benefits of This Approach

1. **Hidden Complexity**: Zero calibration happens automatically
2. **Natural Flow**: Leverages the put-on sequence
3. **Single User Action**: Just 5 seconds of standing
4. **Complete Setup**: All sensors ready after onboarding
5. **Accurate Weight**: Immediate weight tracking capability

### 8.4 Error Handling

```c
typedef enum {
    CAL_SUCCESS = 0,
    CAL_ERR_NO_ZERO,        // Zero cal not completed
    CAL_ERR_MOVEMENT,       // User moved during calibration
    CAL_ERR_WEIGHT_MISMATCH,// Detected weight very different
    CAL_ERR_TIMEOUT         // Calibration timeout
} cal_error_t;

// Smart retry logic
if (error == CAL_ERR_NO_ZERO) {
    // Prompt user to remove shoes briefly
    // Retry zero calibration
}
```

---

## Appendix A: Calibration State Machine

```
[INIT] ──power on──> [ZERO_CAL_WAIT]
                          │
                          ├──not worn detected──> [ZERO_CAL_ACTIVE]
                          │                             │
                          │                             ▼
                          │                      [ZERO_CAL_COMPLETE]
                          │                             │
                          └─────────shoes worn─────────┘
                                                       │
                                                       ▼
                                              [READY_FOR_COMBINED]
                                                       │
                                     ┌─────────────────┘
                                     ▼
                            [COMBINED_CAL_ACTIVE] (5 sec)
                                     │
                                     ▼
                              [FULLY_CALIBRATED]
```

---

**End of Document**
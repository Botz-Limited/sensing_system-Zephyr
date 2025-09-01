# BLE Activity Metrics Service Architecture Analysis

**Date:** June 2025  
**Version:** 1.0  
**Purpose:** Analyze options for implementing BLE characteristics for activity metrics

---

## Executive Summary

We need to expose the real-time activity metrics calculated by the `realtime_metrics` module via BLE to mobile applications. This document analyzes whether to extend the existing Information Service or create a new dedicated Activity Metrics Service.

---

## Current State Analysis

### Existing Information Service

The Information Service (UUID: `4fd5b67f-9d89-4061-92aa-319ca786baae`) currently contains:

**Sensor Data Characteristics:**
- Foot Sensor Data (8 channels pressure)
- BHI360 Data Set 1 (3D mapping: quaternion + gyro)
- BHI360 Data Set 2 (Step count) - Deprecated
- BHI360 Data Set 3 (Linear acceleration)

**Step Count Characteristics:**
- Total Step Count (aggregated from both feet)
- Activity Step Count (steps during activity session)

**Status & Management:**
- Device Status (32-bit bitmask)
- Charge Status
- Current Time
- Weight Measurement

**File Management:**
- Foot/BHI360/Activity Log Available notifications
- File paths for each log type

**Secondary Device:** (Primary only)
- Secondary device info (manufacturer, model, etc.)
- Secondary FOTA progress
- Secondary file management

**Total Characteristics:** ~25+ characteristics

### New Metrics to Expose

From the `realtime_metrics` module, we need to expose:

**Basic Metrics (1Hz updates):**
- Cadence (steps/min)
- Pace (sec/km)
- Distance (meters)
- Form Score (0-100)
- Balance L/R (%)

**Asymmetry Metrics:**
- Contact Time Asymmetry (%)
- Flight Time Asymmetry (%)
- Force Asymmetry (%)
- Pronation Asymmetry (%)

**Biomechanics:**
- Ground Contact Time (ms)
- Flight Time (ms)
- Strike Pattern (L/R)
- Pronation Angle (degrees)
- Efficiency Score (0-100)

**Alerts:**
- Real-time alerts (bitmask)

**Total New Data Points:** ~20 metrics

---

## Architecture Options

### Option 1: Extend Information Service

**Approach:** Add new characteristics to the existing Information Service

**Pros:**
- Single service for all sensor/activity data
- Existing mobile apps already connect to this service
- Simpler service discovery

**Cons:**
- Information Service already has 25+ characteristics (getting unwieldy)
- Mixing raw sensor data with processed metrics
- Harder to maintain and document
- May hit BLE service size limits
- Different update rates (sensor data vs metrics)

### Option 2: Create New Activity Metrics Service

**Approach:** Create a dedicated service for activity metrics

**Proposed UUID:** `4fd5b690-9d89-4061-92aa-319ca786baae` (increment from existing pattern)

**Pros:**
- Clean separation of concerns
- Easier to version and maintain
- Can optimize for 1Hz update pattern
- Clear documentation
- Room for future expansion
- Can use efficient packed formats

**Cons:**
- Mobile apps need to discover/subscribe to new service
- Additional service overhead

---

## Recommendation: Create New Activity Metrics Service

### Rationale

1. **Separation of Concerns**: Raw sensor data (100Hz) vs processed metrics (1Hz)
2. **Maintainability**: Easier to update metrics without affecting sensor data flow
3. **Efficiency**: Can pack related metrics into fewer characteristics
4. **Future-proof**: Room for GPS data, advanced analytics, etc.
5. **Clean API**: Mobile developers get a focused interface

### Proposed Service Design

```
Activity Metrics Service (UUID: 4fd5b690-xxxx-xxxx-xxxx-xxxxxxxxxxxx)
│
├── Real-time Metrics (UUID: xxx1) - 20 bytes, Notify, 1Hz
│   ├── Cadence (uint16_t) - steps/min
│   ├── Pace (uint16_t) - sec/km
│   ├── Distance (uint32_t) - meters
│   ├── Form Score (uint8_t) - 0-100
│   ├── Balance L/R (int8_t) - -50 to +50
│   ├── Ground Contact Time (uint16_t) - ms
│   ├── Flight Time (uint16_t) - ms
│   ├── Efficiency Score (uint8_t) - 0-100
│   └── Alerts (uint8_t) - bitmask
│
├── Asymmetry Metrics (UUID: xxx2) - 8 bytes, Notify, 1Hz
│   ├── Contact Time Asymmetry (uint8_t) - %
│   ├── Flight Time Asymmetry (uint8_t) - %
│   ├── Force Asymmetry (uint8_t) - %
│   ├── Pronation Asymmetry (uint8_t) - %
│   ├── Strike Pattern Left (uint8_t) - 0=heel,1=mid,2=fore
│   ├── Strike Pattern Right (uint8_t) - 0=heel,1=mid,2=fore
│   └── Reserved (uint16_t)
│
├── Biomechanics Extended (UUID: xxx3) - 12 bytes, Read/Notify, On-demand
│   ├── Pronation Left (int8_t) - degrees
│   ├── Pronation Right (int8_t) - degrees
│   ├── Loading Rate Left (uint16_t) - N/s
│   ├── Loading Rate Right (uint16_t) - N/s
│   ├── Arch Collapse Left (uint8_t) - 0-100
│   ├── Arch Collapse Right (uint8_t) - 0-100
│   └── Reserved (uint16_t)
│
├── Session Summary (UUID: xxx4) - 20 bytes, Read/Notify, End of session
│   ├── Total Distance (uint32_t) - meters
│   ├── Average Pace (uint16_t) - sec/km
���   ├── Average Cadence (uint16_t) - steps/min
│   ├── Total Steps (uint32_t) - count
│   ├── Calories (uint16_t) - kcal
│   ├── Average Form Score (uint8_t) - 0-100
│   └── Reserved (uint16_t)
│
└── GPS Data (UUID: xxx5) - 16 bytes, Write, From phone
    ├── Latitude (int32_t) - degrees * 10^7
    ├── Longitude (int32_t) - degrees * 10^7
    ├── Distance Delta (uint16_t) - meters since last
    ├── Accuracy (uint8_t) - meters
    ├── Elevation Change (int16_t) - meters
    └── Reserved (uint8_t)
```

### Characteristic Packing Strategy

**Real-time Metrics (20 bytes):**
```c
typedef struct __attribute__((packed)) {
    uint16_t cadence_spm;        // 0-1
    uint16_t pace_sec_km;        // 2-3
    uint32_t distance_m;         // 4-7
    uint8_t  form_score;         // 8
    int8_t   balance_lr_pct;     // 9
    uint16_t ground_contact_ms;  // 10-11
    uint16_t flight_time_ms;     // 12-13
    uint8_t  efficiency_score;   // 14
    uint8_t  alerts;            // 15
    uint32_t reserved;          // 16-19
} realtime_metrics_ble_t;
```

**Asymmetry Metrics (8 bytes):**
```c
typedef struct __attribute__((packed)) {
    uint8_t contact_time_asym;   // 0
    uint8_t flight_time_asym;    // 1
    uint8_t force_asym;          // 2
    uint8_t pronation_asym;      // 3
    uint8_t strike_left;         // 4
    uint8_t strike_right;        // 5
    uint16_t reserved;           // 6-7
} asymmetry_metrics_ble_t;
```

### Implementation Benefits

1. **Efficient Updates**: Pack multiple metrics per notification
2. **Backward Compatible**: Existing apps continue using Information Service
3. **Power Efficient**: 1Hz updates with packed data
4. **Future GPS Support**: Dedicated characteristic ready
5. **Clean Mobile API**: Subscribe to 2-3 characteristics for all metrics

---

## Implementation Plan

### Phase 1: Core Service (Week 1)
1. Create new Activity Metrics Service
2. Implement Real-time Metrics characteristic
3. Implement Asymmetry Metrics characteristic
4. Update realtime_metrics module to populate BLE structures
5. Test with nRF Connect app

### Phase 2: Extended Features (Week 2)
1. Add Biomechanics Extended characteristic
2. Add Session Summary characteristic
3. Implement end-of-session notifications
4. Add GPS data write characteristic

### Phase 3: Integration (Week 3)
1. Update mobile app SDK
2. Add service to documentation
3. Performance testing
4. Battery impact analysis

---

## Alternative: Minimal Extension Approach

If creating a new service is not feasible, we could minimally extend Information Service with just 2-3 packed characteristics:

1. **Activity Metrics Basic** (16 bytes) - Core metrics only
2. **Activity Metrics Extended** (16 bytes) - Additional metrics

This would add minimal overhead but lacks the clean separation and future expansion room.

---

## Conclusion

Creating a dedicated **Activity Metrics Service** is the recommended approach because it:
- Provides clean separation between raw sensor data and processed metrics
- Allows for efficient packed data formats
- Leaves room for future features (GPS, advanced analytics)
- Makes the mobile API cleaner and more maintainable
- Follows BLE best practices for service design

The implementation effort is similar to extending the existing service, but the long-term benefits in maintainability and expandability make it the better choice.
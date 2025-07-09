# Activity Metrics Implementation Tracker

**Date:** December 2024  
**Version:** 1.0  
**Purpose:** Track implementation status and remaining tasks for the Activity Metrics multi-thread architecture

---

## System Architecture Overview

### Thread Structure
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
└───────────────────────────────────────────────────────────────┘
```

---

## Implementation Status Summary

### ✅ Completed Components

#### sensor_data Module (src/sensor_data/)
- [x] Thread and work queue infrastructure
- [x] Ring buffers for data buffering
- [x] Ground contact detection with hysteresis
- [x] 6-phase contact state machine
- [x] Peak force tracking
- [x] Pressure distribution calculation
- [x] Strike pattern detection
- [x] Enhanced pronation calculation (IMU + pressure)
- [x] Delta time tracking
- [x] Bilateral timing metrics
- [x] Center of pressure calculation
- [x] Loading rate calculation
- [x] Arch collapse detection
- [x] Foot strike angle calculation

#### realtime_metrics Module (src/realtime_metrics/)
- [x] Thread and work queue infrastructure
- [x] Cadence tracking with sliding window
- [x] Pace estimation using stride length model
- [x] Form score calculation
- [x] Balance L/R calculation
- [x] Real-time asymmetry detection
- [x] Alert generation system
- [x] 1Hz BLE update mechanism
- [x] Moving averages for smooth output
- [x] Fatigue level tracking

#### analytics Module (src/analytics/)
- [x] Basic infrastructure created
- [x] Thread and work queue setup
- [x] Baseline establishment framework
- [ ] Running efficiency calculation
- [ ] Fatigue index with baseline comparison
- [ ] Injury risk assessment
- [ ] CPEI calculation
- [ ] Vertical oscillation from IMU

#### activity_metrics Module (src/activity_metrics/)
- [x] Work queue architecture
- [x] Step detection integration
- [x] Weight measurement functionality
- [x] Basic session management
- [ ] Needs refactoring to remove real-time processing
- [ ] GPS integration
- [ ] Session record generation

### 🔴 Critical Issues to Fix

1. **Sensor Module Integration**
   - Motion sensor still sends to `activity_metrics_msgq` instead of `sensor_data_msgq`
   - Foot sensor still sends to `activity_metrics_msgq` instead of `sensor_data_msgq`
   - Need to ADD sending to new queue while keeping existing functionality

2. **Message Passing**
   - Consolidated sensor data structure not properly passed between modules
   - Need to enhance message structures to carry actual data
   - Currently using placeholder messages

3. **GPS Support**
   - No GPS characteristic in control service
   - No GPS message types defined
   - No GPS data structures implemented
   - No calibration logic

---

## Task List (Priority Order)

### 🚨 Immediate Tasks (December 2024)

#### 1. Create GPS Characteristic in Control Service
**Status:** ✅ COMPLETED (December 2024)
**Files modified:**
- `src/bluetooth/control_service.cpp` - Added GPS characteristic and handler
- `include/events/app_state_event.h` - Added MSG_TYPE_GPS_UPDATE
- `include/app.hpp` - Added GPSUpdateCommand to union
- `src/bluetooth/ble_d2d_tx.cpp` - Added GPS forwarding function
- `src/bluetooth/ble_d2d_rx.cpp` - Modified to handle GPS via weight calibration characteristic
- `src/activity_metrics/activity_metrics.cpp` - Added GPS update handling

**Completed Tasks:**
- [x] Add GPS Update characteristic to control service (UUID: 0x4fd5b68e...)
- [x] GPS data structure already defined in activity_session.hpp
- [x] Add MSG_TYPE_GPS_UPDATE to message types
- [x] Implement write handler for GPS data
- [x] Add D2D cascading for GPS updates

#### 2. Update Sensor Modules to Use sensor_data_msgq
**Status:** ✅ ALREADY IMPLEMENTED
**Files checked:**
- `src/motion_sensor/motion_sensor.cpp` - Already sends to sensor_data_msgq
- `src/foot_sensor/foot_sensor.cpp` - Already sends to sensor_data_msgq

**Findings:**
- [x] Both modules already send to sensor_data_msgq when CONFIG_SENSOR_DATA_MODULE=y
- [x] All existing message sends preserved (bluetooth_msgq, data_msgq)
- [x] No changes needed - implementation was already in place
- [x] Data flow verified

#### 3. Fix Message Passing Between Modules
**Status:** ✅ COMPLETED (December 2024)
**Files modified:**
- `include/app.hpp` - Added sensor_data_consolidated_t to union
- `src/sensor_data/sensor_data.cpp` - Fixed data copying
- `src/realtime_metrics/realtime_metrics.cpp` - Fixed data reception

**Completed Tasks:**
- [x] Define proper consolidated data structure in generic_message_t
- [x] Implement data copying in sensor_data module
- [x] Update realtime_metrics to receive actual data
- [x] Added thread safety (spinlock - to be changed to mutex)

#### 4. Verify activity_metrics Implementation
**Status:** ✅ COMPLETED  
**Files checked:**
- `src/activity_metrics/activity_metrics.cpp`

**Verified:**
- [x] All work queue handlers are complete and functional
- [x] Session management works (start/stop)
- [x] Weight measurement is fully functional
- [x] Work queue pushing system unchanged
- [x] GPS update handling added

### 🆕 New Immediate Tasks (December 2024)

#### 5. Replace Spinlock with Mutex/Semaphore
**Status:** Not Started
**Files to modify:**
- `src/realtime_metrics/realtime_metrics.cpp`

**Tasks:**
- [ ] Replace k_spinlock with k_mutex or k_sem
- [ ] Update lock/unlock calls appropriately
- [ ] Test thread safety with new locking mechanism

#### 6. Create Dual GPS/Non-GPS Function Versions
**Status:** Not Started
**Files to modify:**
- `src/activity_metrics/activity_metrics.cpp`
- `src/realtime_metrics/realtime_metrics.cpp`

**Tasks:**
- [ ] Create calculate_pace_with_gps() and calculate_pace_without_gps()
- [ ] Add GPS availability check
- [ ] Implement graceful fallback
- [ ] Update distance calculation functions
- [ ] Test both code paths

#### 7. Verify Cross-Module Data Access
**Status:** Not Started
**Analysis needed:**
- Functions requiring both motion and foot sensor data
- Current data availability in consolidated structure

**Tasks:**
- [ ] Identify all functions needing both sensor types
- [ ] Verify data is available in consolidated structure
- [ ] Add missing data if needed
- [ ] Test cross-sensor calculations

### 📅 Short Term Tasks (Week 2-3)

#### 5. Complete Analytics Algorithms
- [ ] Implement running efficiency calculation
- [ ] Add fatigue index with baseline
- [ ] Create injury risk assessment
- [ ] Add vertical oscillation from IMU

#### 6. Refactor activity_metrics Module
- [ ] Remove real-time processing code (move to other modules)
- [ ] Focus on session management only
- [ ] Implement periodic record creation
- [ ] Add GPS calibration processing

### 📆 Medium Term Tasks (Week 4-6)

#### 7. Integration Testing
- [ ] Test full data flow with all modules
- [ ] Measure timing accuracy (100Hz, 10Hz, 1Hz)
- [ ] Profile CPU usage
- [ ] Check memory usage

#### 8. System Testing
- [ ] End-to-end testing with real sensors
- [ ] Performance optimization
- [ ] Power consumption analysis
- [ ] Field testing with actual running

---

## Technical Constraints

### Message Queue Usage
- **DO NOT** remove existing message sends from sensors
- Sensors must continue sending to:
  - `bluetooth_msgq` (for BLE updates)
  - `data_msgq` (for logging)
  - `activity_metrics_msgq` (legacy, to be phased out)
  - `sensor_data_msgq` (NEW - to be added)

### Work Queue System
- **DO NOT** change how work queues function
- Current system uses work items submitted to dedicated queues
- Any changes must be discussed and approved first

### Memory Allocation
- All modules use static allocation
- No dynamic memory allocation allowed
- Fixed-size buffers and structures

---

## GPS Data Structure (from specification)

```c
// GPS Update Command Structure
typedef struct __attribute__((packed)) {
    uint8_t  opcode;                // = 0x10
    uint32_t timestamp;             // Unix time
    int32_t  latitude_e7;           // Latitude × 10^7
    int32_t  longitude_e7;          // Longitude × 10^7
    uint16_t speed_cms;             // Speed in cm/s
    uint16_t distance_m;            // Distance since last update
    uint8_t  accuracy_m;            // GPS accuracy
    int16_t  elevation_change_m;    // Elevation change
} GPSUpdateCommand;

// GPS Operating Modes
enum GPSMode {
    GPS_MODE_OFF = 0,
    GPS_MODE_CALIBRATION = 1,   // 60s updates
    GPS_MODE_PRECISE = 2,       // 30s updates
    GPS_MODE_RACE = 3           // 10-15s updates
};
```

---

## Performance Targets

- **sensor_data**: < 2ms processing @ 100Hz
- **realtime_metrics**: < 20ms @ 10Hz
- **analytics**: < 200ms @ 1Hz
- **Total CPU**: < 40%
- **RAM usage**: < 100KB

---

## Testing Checklist

### Unit Testing
- [ ] Test each algorithm independently
- [ ] Use known input/output data
- [ ] Verify edge cases

### Integration Testing
- [ ] Full system data flow
- [ ] Timing under load
- [ ] Memory leak detection
- [ ] Message queue overflow handling

### Field Testing
- [ ] Real running scenarios
- [ ] Battery life impact
- [ ] BLE connection stability
- [ ] GPS accuracy verification

---

## Risk Mitigation

### High Risks
1. **100Hz Timing Miss**
   - Current implementation uses optimized algorithms
   - Monitor actual processing time
   - Fallback: Reduce to 50Hz if needed

2. **Message Queue Overflow**
   - Ring buffers implemented in sensor_data
   - Monitor queue statistics
   - Drop old data if necessary

### Medium Risks
1. **GPS Integration Complexity**
   - Start with basic implementation
   - Add features incrementally
   - Test with/without GPS

2. **Module Communication**
   - Use existing patterns
   - Test each connection point
   - Add debug logging

---

## Notes and Decisions

### December 2024
- Decision to ADD sensor_data_msgq sends rather than replace existing
- GPS characteristic to be added to control service
- Work queue system must remain unchanged
- Message passing needs proper data structures

---

## Contact

For questions or changes to this tracker:
- Review with team before making architectural changes
- Update this document as tasks are completed
- Add notes for any deviations from plan
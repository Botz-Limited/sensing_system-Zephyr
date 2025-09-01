# Bluetooth Module Updates Summary

**Date:** July 2025  
**Version:** 1.0  
**Scope:** Summary of all Bluetooth module changes and updates

---

## Overview

This document summarizes the significant updates made to the Bluetooth module, including new services, moved characteristics, and architectural improvements.

---

## New Services Added

### 1. Activity Metrics Service

**UUID:** `4fd5b690-9d89-4061-92aa-319ca786baae`  
**Purpose:** Real-time activity metrics for sports and fitness applications  
**Availability:** Primary device only

**Characteristics:**
- **Real-time Metrics** (`...b691`): 20 bytes, 1Hz updates
  - Cadence, pace, distance, form score, balance, ground contact time, etc.
- **Asymmetry Metrics** (`...b692`): 8 bytes, 1Hz updates
  - Contact time, flight time, force, and pronation asymmetry
- **Biomechanics Extended** (`...b693`): 12 bytes, on-demand
  - Detailed biomechanical data
- **Session Summary** (`...b694`): 20 bytes, end of session
  - Total distance, average pace, calories, etc.
- **GPS Data** (`...b695`): 16 bytes, write from phone
  - GPS coordinates and accuracy for calibration
- **Total Step Count** (`...b696`): 8 bytes, notify on change
  - Moved from Information Service
- **Activity Step Count** (`...b697`): 8 bytes, notify on change
  - Moved from Information Service

**Implementation Details:**
- Fully integrated with `realtime_metrics` module
- Updates sent at 1Hz via dedicated work item
- Handles step count aggregation from both feet
- GPS data input for future distance calibration

### 2. Secondary Device Service

**UUID:** `4fd5b6a0-9d89-4061-92aa-319ca786baae`  
**Purpose:** Consolidated secondary device information and management  
**Availability:** Primary device only

**Characteristics:**
- Device information (manufacturer, model, serial, HW/FW revisions)
- FOTA progress for secondary device
- Log file availability and paths (foot, BHI360, activity)
- Weight measurement from secondary device

**Benefits:**
- All secondary device info in one service
- Reduces complexity for mobile apps
- Consistent interface with primary device patterns

---

## Information Service Updates

### Characteristics Moved

**Moved to Activity Metrics Service:**
- **Total Step Count** (was `...ec4`)
- **Activity Step Count** (was `...ec5`)

**Rationale:** These characteristics are activity-related and update frequently, making them more appropriate for the Activity Metrics Service.

### New Packed Characteristics

**Packed Device Status** (`...ec7`):
- 16-byte structure combining multiple status fields
- Includes status bitfield, battery, temperature, activity state, uptime, storage, etc.
- Reduces number of notifications needed

**Packed File Notification** (`...ec8`):
- 64-byte structure combining file ID and path
- Includes file type and device source
- Single notification instead of two separate ones

### Deprecated Characteristics

**BHI360 Step Count** (`...eb3`):
- Remains for backward compatibility
- Marked as DEPRECATED
- Mobile apps should use Activity Metrics Service instead

---

## Implementation Changes

### Step Count Handling

**Previous Architecture:**
- Information Service handled all step counts
- Individual foot counts sent to mobile
- Complex aggregation logic

**New Architecture:**
- Activity Metrics Service handles aggregated counts
- Step aggregation done in `bluetooth.cpp`
- Cleaner separation of concerns
- Individual foot counts only used internally

### File Structure

**New Files:**
- `activity_metrics_service.cpp/h` - Activity Metrics Service implementation
- `secondary_device_service.cpp` - Secondary Device Service (already existed)
- `ble_packed_structs.h` - Packed data structure definitions

**Modified Files:**
- `information_service.cpp` - Removed step count handlers, added packed characteristics
- `bluetooth.cpp` - Updated to use new service APIs
- `ble_services.hpp` - Added new function declarations

---

## Benefits of Changes

### For Mobile App Developers
- **Clearer Service Organization**: Activity data in Activity Metrics Service
- **Reduced Complexity**: Packed structures reduce notification count
- **Better Performance**: 1Hz updates for real-time metrics
- **Future-Ready**: GPS input for enhanced accuracy

### For Firmware Developers
- **Modular Architecture**: Clear separation between services
- **Easier Maintenance**: Activity metrics isolated from device info
- **Consistent Patterns**: All services follow same implementation style
- **Thread Safety**: Work queue architecture for all updates

### For System Performance
- **Reduced BLE Traffic**: Packed structures send more data per notification
- **Efficient Updates**: 1Hz rate for metrics balances responsiveness and power
- **Memory Efficiency**: Fixed-size structures, no dynamic allocation

---

## Migration Guide

### For Mobile Apps

1. **Step Counts**:
   - Old: Read from Information Service (`...ec4`, `...ec5`)
   - New: Read from Activity Metrics Service (`...b696`, `...b697`)

2. **Real-time Metrics**:
   - Subscribe to Activity Metrics Service (`...b691`)
   - Parse 20-byte structure for all metrics
   - Update at 1Hz rate

3. **Device Status**:
   - Can use new Packed Device Status (`...ec7`) for all status info
   - Or continue using individual characteristics

4. **File Notifications**:
   - Can use new Packed File Notification (`...ec8`)
   - Combines file ID and path in one notification

### For Firmware Developers

1. **Sending Step Counts**:
   ```c
   // Old way
   jis_total_step_count_notify(total_steps);
   
   // New way
   ams_update_total_step_count(total_steps);
   ```

2. **Activity Metrics Updates**:
   ```c
   // Send to bluetooth message queue
   generic_message_t msg = {
       .type = MSG_TYPE_ACTIVITY_METRICS_BLE,
       .sender = SENDER_REALTIME_METRICS,
       .data.activity_metrics_ble = metrics_data
   };
   k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT);
   ```

---

## Testing Recommendations

1. **Verify Step Count Migration**:
   - Confirm counts appear in Activity Metrics Service
   - Check aggregation from both feet works correctly
   - Verify activity step count resets properly

2. **Test Packed Characteristics**:
   - Read packed device status and verify all fields
   - Test file notification with various file types
   - Confirm backward compatibility with old characteristics

3. **Performance Testing**:
   - Monitor BLE bandwidth usage
   - Check 1Hz update timing accuracy
   - Verify no message queue overflows

4. **Integration Testing**:
   - Test with mobile app using new services
   - Verify GPS data write works
   - Check secondary device service updates

---

## Known Limitations

1. **GPS Integration**: GPS data can be received but is not yet used for calibration
2. **Biomechanics Extended**: Structure defined but algorithms not implemented
3. **Session Summary**: Structure defined but session management not complete

---

## Future Enhancements

1. **GPS Calibration**: Use GPS data to improve distance/pace accuracy
2. **Advanced Biomechanics**: Implement loading rate, push-off power calculations
3. **Session Recording**: Complete session management with file storage
4. **Power Optimization**: Adaptive update rates based on activity

---

## Document History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | July 2025 | Initial documentation of Bluetooth module updates |

---

**End of Document**
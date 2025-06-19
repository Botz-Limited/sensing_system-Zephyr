# FOTA Synchronization Implementation

## Overview

This document describes the implementation of synchronized FOTA updates for both primary and secondary devices, ensuring that the primary device doesn't reset until the secondary device has completed its firmware update.

## Problem Solved

Previously, when updating both devices:
1. Primary would complete its update and reset immediately
2. This broke the D2D connection to the secondary device
3. Secondary's update would fail due to lost connection
4. System would have mismatched firmware versions

## Solution Architecture

### 1. Completion Tracking

Added synchronization fields to FOTA proxy state:
```c
struct {
    // ... existing fields ...
    bool primary_complete;      // Primary update done
    bool secondary_complete;    // Secondary update done  
    bool waiting_for_secondary; // Primary waiting for secondary
} fota_proxy_state;
```

### 2. New Commands and Status

#### FOTA Proxy Commands
```c
enum fota_proxy_cmd {
    // ... existing commands ...
    FOTA_PROXY_CMD_SECONDARY_COMPLETE = 0x07,  // New
};
```

#### FOTA Proxy Status
```c
enum fota_proxy_status {
    // ... existing status ...
    FOTA_PROXY_STATUS_WAITING_SECONDARY = 0x05,  // New
    FOTA_PROXY_STATUS_BOTH_COMPLETE = 0x06,      // New
};
```

### 3. D2D Communication

#### New D2D Characteristic
- **UUID**: `e160ca86-3115-4ad6-9709-8c5ff3bf558b`
- **Purpose**: Secondary reports FOTA completion to primary
- **Direction**: Secondary → Primary

## Implementation Details

### Primary Device Flow

1. **Reset Command Handling** (`fota_proxy.cpp`)
```c
case FOTA_PROXY_CMD_RESET:
    if (target == FOTA_TARGET_ALL) {
        primary_complete = true;
        if (secondary_complete) {
            // Both done, safe to reset
            schedule_reset(2 seconds);
        } else {
            // Wait for secondary
            waiting_for_secondary = true;
            status = WAITING_SECONDARY;
            schedule_reset(30 seconds); // Timeout
        }
    }
```

2. **Secondary Completion Handler**
```c
int fota_proxy_handle_secondary_complete(void) {
    secondary_complete = true;
    if (waiting_for_secondary && primary_complete) {
        // Both complete now
        cancel_timeout();
        schedule_reset(2 seconds);
    }
}
```

### Secondary Device Flow

1. **FOTA Confirmation** (`app.cpp`)
```c
mgmt_cb_return fota_confirmed_callback(...) {
    // ... existing code ...
    
    #if IS_ENABLED(CONFIG_SECONDARY_DEVICE)
    // Notify primary of completion
    ble_d2d_tx_send_fota_complete();
    #endif
}
```

2. **D2D Notification** (`ble_d2d_tx.cpp`)
```c
int ble_d2d_tx_send_fota_complete(void) {
    uint8_t status = 1; // 1 = complete
    return bt_gatt_write_without_response(
        d2d_conn, 
        d2d_handles.fota_status_handle,
        &status, 
        sizeof(status), 
        false);
}
```

### Communication Flow

```
Phone → Primary: FOTA data for both devices
         ├── Primary: Updates own firmware
         └── Primary: Forwards to Secondary via SMP
                      Secondary: Updates firmware
                      
Phone → Primary: Reset command
         Primary: Marks self complete, waits for secondary
         
Secondary: FOTA confirmed → Sends completion via D2D
Primary: Receives completion → Both ready → Reset!
```

## Safety Features

### 1. Timeout Protection
- 30-second timeout if secondary doesn't report completion
- Primary will reset anyway to prevent permanent hang

### 2. Status Visibility
- New status values inform phone app of synchronization state
- `WAITING_SECONDARY` - Primary done, waiting
- `BOTH_COMPLETE` - Safe to proceed

### 3. Graceful Degradation
- If D2D connection lost, timeout ensures reset
- Single device updates work as before
- No impact on primary-only or secondary-only updates

## Mobile App Integration

### Status Monitoring
```swift
switch fotaStatus {
case 0x05: // WAITING_SECONDARY
    print("Primary complete, waiting for secondary...")
case 0x06: // BOTH_COMPLETE
    print("Both devices ready for reset")
}
```

### Recommended Flow
1. Set target = ALL
2. Send firmware data
3. Monitor status notifications
4. Wait for BOTH_COMPLETE status
5. Send reset command
6. Handle temporary disconnection
7. Reconnect after devices reset

## Testing Scenarios

### 1. Happy Path
- Both devices update successfully
- Secondary notifies completion
- Synchronized reset occurs

### 2. Secondary Failure
- Primary completes, waits
- Timeout after 30 seconds
- Primary resets alone

### 3. Connection Loss
- Primary completes
- D2D connection drops
- Timeout ensures reset

### 4. Power Loss Recovery
- State not persistent
- Requires restart of FOTA process

## Log Messages

### Primary Device
```
"Starting FOTA update, total size: X bytes"
"Reset command received for target: 2" (ALL)
"Waiting for secondary device to complete FOTA..."
"Secondary device FOTA completion received"
"Both devices now complete, scheduling reset"
"Performing system reset for FOTA completion"
```

### Secondary Device
```
"FOTA Image confirmed!"
"Notifying primary device of FOTA completion"
"D2D TX: Sending FOTA complete status to primary"
```

## Benefits

1. **Reliability**: Both devices update successfully or timeout gracefully
2. **Synchronization**: Firmware versions stay matched
3. **Visibility**: Clear status for mobile app
4. **Robustness**: Handles failures and edge cases

## Future Enhancements

1. **Persistent State**: Save FOTA state to resume after unexpected reset
2. **Retry Mechanism**: Retry secondary notification on failure
3. **Version Check**: Verify both devices on same version after reset
4. **Batch Updates**: Support updating multiple secondary devices

## Conclusion

The FOTA synchronization mechanism ensures reliable firmware updates for both devices in the system. The primary device now waits for secondary completion before resetting, preventing update failures due to premature connection loss.
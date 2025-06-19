# FOTA Synchronization Fix for Primary-Secondary Updates

## Problem Statement

The current FOTA proxy implementation has a critical flaw when updating both devices:

1. Primary device completes its FOTA and resets immediately
2. Secondary device may still be receiving/processing firmware
3. Connection loss causes secondary update to fail
4. System ends up with mismatched firmware versions

## Root Cause Analysis

### Current Flow (Problematic)
```
Phone → Primary: FOTA data
         ├── Primary: Writes to own flash
         └── Primary: Forwards to Secondary via SMP
                      Secondary: Writing to flash...
Primary: Update complete → RESET! ❌
Secondary: Connection lost → Update fails!
```

### Issues in Current Code

1. **No Secondary Completion Tracking**
   - Primary doesn't wait for secondary to confirm completion
   - No status feedback from secondary to primary

2. **Immediate Reset**
   ```cpp
   case FOTA_PROXY_CMD_RESET:
       if (fota_proxy_state.current_target == FOTA_TARGET_PRIMARY) {
           k_sleep(K_SECONDS(1));
           sys_reboot(SYS_REBOOT_WARM);  // Immediate reset!
       }
   ```

3. **No Coordination for ALL Target**
   - When target is ALL, both devices update simultaneously
   - No mechanism to ensure both complete before reset

## Proposed Solution

### 1. Add Secondary Status Tracking

```cpp
// Enhanced state structure
static struct {
    // ... existing fields ...
    bool primary_complete;
    bool secondary_complete;
    bool waiting_for_secondary;
    struct k_work_delayable reset_work;
} fota_proxy_state;
```

### 2. Implement Completion Handshake

```cpp
// New command for secondary to report completion
enum fota_proxy_cmd {
    // ... existing commands ...
    FOTA_PROXY_CMD_SECONDARY_COMPLETE = 0x07,
};

// Handler for secondary completion
case FOTA_PROXY_CMD_SECONDARY_COMPLETE:
    fota_proxy_state.secondary_complete = true;
    LOG_INF("Secondary device FOTA complete");
    
    // Check if we should reset now
    if (fota_proxy_state.waiting_for_secondary) {
        k_work_schedule(&fota_proxy_state.reset_work, K_SECONDS(2));
    }
    break;
```

### 3. Modified Reset Logic

```cpp
case FOTA_PROXY_CMD_RESET:
    if (fota_proxy_state.current_target == FOTA_TARGET_ALL) {
        // For ALL target, wait for both to complete
        fota_proxy_state.primary_complete = true;
        
        if (fota_proxy_state.secondary_complete) {
            // Both done, safe to reset
            k_work_schedule(&fota_proxy_state.reset_work, K_SECONDS(2));
        } else {
            // Wait for secondary
            fota_proxy_state.waiting_for_secondary = true;
            LOG_INF("Waiting for secondary to complete FOTA...");
            
            // Set a timeout in case secondary fails
            k_work_schedule(&fota_proxy_state.reset_work, K_SECONDS(30));
        }
    } else if (fota_proxy_state.current_target == FOTA_TARGET_PRIMARY) {
        // Primary only, reset immediately
        k_work_schedule(&fota_proxy_state.reset_work, K_SECONDS(2));
    }
    // Secondary-only doesn't reset primary
    break;
```

### 4. Add D2D Status Reporting

Create a new D2D characteristic for secondary to report FOTA status:

```cpp
// In ble_d2d_tx.cpp (secondary side)
int ble_d2d_tx_send_fota_complete(void) {
    if (!d2d_conn) return -ENOTCONN;
    
    uint8_t status = FOTA_STATUS_COMPLETE;
    return bt_gatt_write_without_response(d2d_conn, 
                                         d2d_handles.fota_status_handle,
                                         &status, sizeof(status), false);
}
```

## Implementation Steps

### Step 1: Enhance FOTA Proxy State
- Add completion flags
- Add reset work item
- Add timeout handling

### Step 2: Implement Secondary → Primary Status
- Add D2D characteristic for FOTA status
- Secondary sends completion notification
- Primary tracks secondary status

### Step 3: Modify Reset Behavior
- Delay reset until both complete
- Add timeout protection (30 seconds)
- Provide status to phone app

### Step 4: Update Phone App Protocol
```swift
// Phone app should:
1. Set target = ALL
2. Send firmware data
3. Monitor status notifications
4. Wait for "both devices ready" status
5. Send reset command
6. Expect temporary disconnection
7. Reconnect after reset
```

## Alternative Solutions

### Option A: Sequential Updates
- Update secondary first, wait for completion
- Then update primary
- Pros: Simpler, guaranteed completion
- Cons: Takes twice as long

### Option B: Primary as Pure Proxy
- Primary only forwards data, doesn't update itself
- After secondary completes, phone updates primary
- Pros: No synchronization needed
- Cons: More complex phone app logic

### Option C: Persistent State
- Store FOTA state in non-volatile memory
- Resume after unexpected reset
- Pros: Robust against failures
- Cons: Complex implementation

## Recommended Approach

Implement the completion handshake (main solution) with these safeguards:

1. **Timeout Protection**: 30-second maximum wait
2. **Status Visibility**: Phone can query completion status
3. **Fallback**: If secondary fails, primary still resets after timeout
4. **Clear Logging**: Detailed logs for debugging

## Testing Scenarios

1. **Happy Path**: Both devices complete successfully
2. **Secondary Fails**: Primary resets after timeout
3. **Connection Loss**: Handle gracefully
4. **Power Loss**: State recovery on reconnect
5. **Partial Update**: Rollback mechanism

## Mobile App Changes

```swift
func updateBothDevices() {
    // 1. Set target
    writeCharacteristic(FOTA_TARGET_CHAR, data: [FOTA_TARGET_ALL])
    
    // 2. Send firmware
    sendFirmwareChunks()
    
    // 3. Wait for status
    waitForNotification { status in
        if status == FOTA_STATUS_BOTH_COMPLETE {
            // 4. Safe to reset
            writeCharacteristic(FOTA_COMMAND_CHAR, data: [FOTA_CMD_RESET])
        }
    }
}
```

## Risk Mitigation

1. **Version Mismatch**: Add version checking before starting
2. **Partial Updates**: Implement rollback in bootloader
3. **Communication Failure**: Add retry mechanism
4. **User Interruption**: Save state for resume

## Conclusion

The current implementation's immediate reset is a critical flaw that will cause secondary device updates to fail. The proposed completion handshake ensures both devices finish updating before any reset occurs, making the FOTA process robust and reliable.
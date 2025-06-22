# Developer Quick Reference

**Version:** 1.1  
**Date:** June 2025  
**Scope:** Quick reference guide for developers working with the sensing firmware  
**Purpose:** Essential commands, UUIDs, debug tips, and common patterns for rapid development

---

## Table of Contents

1. [Build Commands](#build-commands)
2. [Key Files](#key-files)
3. [Important UUIDs](#important-uuids)
4. [Command Values](#command-values)
5. [Message Types](#message-types)
6. [Debug Commands](#debug-commands)
7. [Common Issues](#common-issues)
8. [Testing Checklist](#testing-checklist)
9. [Code Patterns](#code-patterns)
10. [Performance Tips](#performance-tips)
11. [Security Notes](#security-notes)

---

## 1. Build Commands

### Primary Device (Right Foot)
```bash
west build -b <board> -- -DCONFIG_PRIMARY_DEVICE=y
```

### Secondary Device (Left Foot)
```bash
west build -b <board> -- -DCONFIG_PRIMARY_DEVICE=n
```

## 2. Key Files

### D2D Communication
- `src/bluetooth/ble_d2d_tx.cpp` - Sends data/commands
- `src/bluetooth/ble_d2d_rx.cpp` - Receives data/commands
- Both files used by BOTH devices

### FOTA Proxy (Primary Only)
- `src/bluetooth/fota_proxy.cpp` - FOTA relay service
- `src/bluetooth/fota_proxy.hpp` - Service definitions

### Control Service (Primary Only)
- `src/bluetooth/control_service.cpp` - Phone commands
- Forwards all commands to secondary via D2D

## 3. Important UUIDs

### D2D RX Service
- Service: `e060ca1f-3115-4ad6-9709-8c5ff3bf558b`
- Set Time: `e160ca1f-3115-4ad6-9709-8c5ff3bf558b`
- Delete Foot Log: `e160ca82-3115-4ad6-9709-8c5ff3bf558b`
- Delete BHI360 Log: `e160ca83-3115-4ad6-9709-8c5ff3bf558b`
- Delete Activity Log: `e160ca87-3115-4ad6-9709-8c5ff3bf558b`
- Start Activity: `e160ca84-3115-4ad6-9709-8c5ff3bf558b`
- Stop Activity: `e160ca85-3115-4ad6-9709-8c5ff3bf558b`
- **Trigger BHI360 Calibration**: `e160ca86-3115-4ad6-9709-8c5ff3bf558b`
- FOTA Status: `e160ca87-3115-4ad6-9709-8c5ff3bf558b`

### FOTA Proxy Service (Primary Only)
- Service: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- Target: `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
- Command: `6e400003-b5a3-f393-e0a9-e50e24dcca9e`
- Data: `6e400004-b5a3-f393-e0a9-e50e24dcca9e`
- Status: `6e400005-b5a3-f393-e0a9-e50e24dcca9e`

## 4. Command Values

### FOTA Proxy Commands
```c
0x01 - START (+ 4 bytes size)
0x02 - DATA (+ firmware bytes)
0x03 - END
0x04 - ABORT
0x05 - STATUS
0x06 - RESET
0x07 - SECONDARY_COMPLETE
```

### FOTA Proxy Status
```c
0x00 - IDLE
0x01 - IN_PROGRESS
0x02 - SUCCESS
0x03 - ERROR
0x04 - NO_TARGET
0x05 - WAITING_SECONDARY
0x06 - BOTH_COMPLETE
```

### FOTA Target
```c
0x00 - PRIMARY
0x01 - SECONDARY
0xFF - ALL
```

## 5. Message Types

### Calibration Messages
```c
MSG_TYPE_TRIGGER_BHI360_CALIBRATION  // Trigger calibration
MSG_TYPE_REQUEST_BHI360_CALIBRATION  // Request stored calibration
MSG_TYPE_BHI360_CALIBRATION_DATA     // Calibration data response
MSG_TYPE_SAVE_BHI360_CALIBRATION     // Save calibration to storage
```

### Data Flow Messages
```c
MSG_TYPE_FOOT_SAMPLES         // Foot sensor data
MSG_TYPE_BHI360_3D_MAPPING    // BHI360 quaternion + gyro
MSG_TYPE_BHI360_LINEAR_ACCEL  // Linear acceleration
MSG_TYPE_BHI360_STEP_COUNT    // Step counter data
MSG_TYPE_BHI360_LOG_RECORD    // Complete BHI360 record for logging
```

### Control Messages
```c
MSG_TYPE_COMMAND              // Generic command string
MSG_TYPE_DELETE_FOOT_LOG      // Delete foot sensor log
MSG_TYPE_DELETE_BHI360_LOG    // Delete BHI360 log
MSG_TYPE_DELETE_ACTIVITY_LOG  // Delete activity log
MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE  // New log file notification
MSG_TYPE_NEW_BHI360_LOG_FILE       // New log file notification
MSG_TYPE_NEW_ACTIVITY_LOG_FILE      // New activity log file notification
```

## 6. Debug Commands

### Check Device Role
```bash
# Look for device name in logs
"SensingGR" = Primary (Right)
"SensingGL" = Secondary (Left)
```

### Monitor D2D Communication
```bash
# Primary logs
"D2D TX: Forwarding command..."
"D2D RX: Received data..."

# Secondary logs  
"D2D TX: Sending data..."
"D2D RX: Received command..."
```

### FOTA Synchronization
```bash
# Key log messages
"Waiting for secondary device to complete FOTA..."
"Secondary device FOTA completion received"
"Both devices now complete, scheduling reset"
```

## 7. Common Issues

### 1. Commands Not Reaching Secondary
- Check D2D connection established
- Verify service discovery completed
- Look for "D2D service discovery complete!"

### 2. FOTA Fails on Secondary
- Ensure D2D connection stable
- Check SMP client configured on primary
- Verify secondary not resetting early

### 3. Build Errors
- Check CONFIG_PRIMARY_DEVICE setting
- Verify all includes present
- Clean build directory

## 8. Testing Checklist

### Basic Connectivity
- [ ] Primary advertises as "SensingGR"
- [ ] Secondary scans and connects
- [ ] D2D service discovery completes
- [ ] Both directions work (TX/RX)

### Command Forwarding
- [ ] Set time syncs both devices
- [ ] Delete commands work on both
- [ ] Activity commands start/stop both

### FOTA Updates
- [ ] Single device updates work
- [ ] Dual device update synchronizes
- [ ] Timeout works if secondary fails
- [ ] Status notifications correct

## 9. Code Patterns

### Adding New Command (Primary → Secondary)

1. Add to Control Service handler:
```c
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
ble_d2d_tx_send_new_command(data);
#endif
```

2. Add to D2D TX:
```c
int ble_d2d_tx_send_new_command(uint8_t data) {
    // Validate connection and discovery
    // Write to characteristic
}
```

3. Add to D2D RX:
```c
static ssize_t d2d_new_command_write(...) {
    // Process command same as Control Service
}
```

### Adding Status Report (Secondary → Primary)

1. Define new characteristic in D2D RX
2. Add handler in D2D RX 
3. Add send function in D2D TX
4. Call from secondary code

## 10. Performance Tips

1. Use write-without-response for speed
2. Batch multiple commands when possible
3. Monitor connection parameters
4. Keep payloads small (<20 bytes)

## 11. Security Notes

1. All characteristics use encryption
2. Bonding required for phone connection
3. D2D connection should validate peer
4. No sensitive data in logs

---

**End of Reference**
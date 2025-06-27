# Developer Quick Reference

**Version:** 1.0  
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
# Using the build script (recommended)
./tools/build_primary.sh

# Manual command
west build --build-dir build_primary \
    --board nrf5340dk/nrf5340/cpuapp \
    --sysbuild \
    -- -DCONFIG_PRIMARY_DEVICE=y \
       -Dipc_radio_EXTRA_CONF_FILE=sysbuild/ipc_radio/prj_primary.conf
```

**Primary Device Characteristics:**
- Advertises as "SensingGR"
- Accepts connections from phones
- Accepts connections from secondary device
- Network core: BT_PERIPHERAL=y, BT_CENTRAL=n

### Secondary Device (Left Foot)
```bash
# Using the build script (recommended)
./tools/build_secondary.sh

# Manual command
west build --build-dir build_secondary \
    --board nrf5340dk/nrf5340/cpuapp \
    --sysbuild \
    -- -DCONFIG_PRIMARY_DEVICE=n \
       -Dipc_radio_EXTRA_CONF_FILE=sysbuild/ipc_radio/prj_secondary.conf
```

**Secondary Device Characteristics:**
- Advertises as "SensingGL"
- Connects to primary device
- Network core: BT_CENTRAL=y (for connecting to primary)

### Flash Commands

#### Flash Primary Device
```bash
# Using the flash script (handles recovery if needed)
./tools/flash_primary.sh

# Manual commands
nrfjprog --program build_primary/merged_CPUNET.hex --verify --chiperase --reset
sleep 2
nrfjprog --program build_primary/merged.hex --verify --chiperase --reset
```

#### Flash Secondary Device
```bash
# Using the flash script
./tools/flash_secondary.sh

# Manual commands
nrfjprog --program build_secondary/merged_CPUNET.hex --verify --chiperase --reset
sleep 2
nrfjprog --program build_secondary/merged.hex --verify --chiperase --reset
```

#### Recovery Mode (if access protection is enabled)
```bash
# Recover network core
nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK

# Recover application core
nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION
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
- Delete Activity Log: `e160ca86-3115-4ad6-9709-8c5ff3bf558b`
- Start Activity: `e160ca84-3115-4ad6-9709-8c5ff3bf558b`
- Stop Activity: `e160ca85-3115-4ad6-9709-8c5ff3bf558b`
- **Trigger BHI360 Calibration**: `e160ca87-3115-4ad6-9709-8c5ff3bf558b`
- FOTA Status: `e160ca88-3115-4ad6-9709-8c5ff3bf558b`

### SMP Proxy Service (Primary Only) - **RECOMMENDED**
- Service: `14387800-130c-49e7-b877-2881c89cb258`
- Characteristic: `14387802-130c-49e7-b877-2881c89cb258`
- Properties: Write Without Response, Notify
- **Usage**: Standard MCUmgr/SMP protocol with target in header
- **Target**: Bit 3 of flags (0=Primary, 1=Secondary)

### Legacy Services (Still Available)
#### FOTA Proxy Service
- Service: `14387800-130c-49e7-b877-2881c89cb258`
- Characteristic: `14387801-130c-49e7-b877-2881c89cb258`

#### File Proxy Service  
- Service: `14387810-130c-49e7-b877-2881c89cb258`
- Characteristic: `14387811-130c-49e7-b877-2881c89cb258`

## 4. Command Values

### SMP Protocol (Recommended)
```c
// SMP Header Structure (8 bytes)
struct smp_header {
    uint8_t  op;       // 0=read, 1=read_rsp, 2=write, 3=write_rsp
    uint8_t  flags;    // Bit 3: target (0=primary, 1=secondary)
    uint16_t len;      // Payload length
    uint16_t group;    // Command group
    uint8_t  seq;      // Sequence number
    uint8_t  id;       // Command ID
};

// Common Groups
0x0000 - OS Management (reset, echo, taskstat)
0x0001 - Image Management (list, upload, test, confirm)
0x0008 - File System (upload, download, stat, hash)
0x0009 - Shell Management
0x000A - Statistics

// Key Commands
IMG_UPLOAD   = 0x01  // Upload firmware image
IMG_LIST     = 0x00  // List images
IMG_TEST     = 0x02  // Test image (mark for next boot)
IMG_CONFIRM  = 0x03  // Confirm image (make permanent)
FS_DOWNLOAD  = 0x00  // Download file
FS_UPLOAD    = 0x01  // Upload file
OS_RESET     = 0x05  // Reset device
OS_ECHO      = 0x00  // Echo test
```

### Legacy FOTA Proxy Commands
```c
0x01 - START (+ 4 bytes size)
0x02 - DATA (+ firmware bytes)
0x03 - END
0x04 - ABORT
0x05 - STATUS
0x06 - RESET
0x07 - SECONDARY_COMPLETE
```

### Legacy FOTA Status
```c
0x00 - IDLE
0x01 - IN_PROGRESS
0x02 - SUCCESS
0x03 - ERROR
0x04 - NO_TARGET
0x05 - WAITING_SECONDARY
0x06 - BOTH_COMPLETE
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
MSG_TYPE_ACTIVITY_STEP_COUNT  // Activity step count data
```

### Control Messages
```c
MSG_TYPE_COMMAND              // Generic command string
MSG_TYPE_DELETE_FOOT_LOG      // Delete foot sensor log
MSG_TYPE_DELETE_BHI360_LOG    // Delete BHI360 log
MSG_TYPE_DELETE_ACTIVITY_LOG  // Delete activity log
MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE  // New log file notification
MSG_TYPE_NEW_BHI360_LOG_FILE       // New log file notification
MSG_TYPE_NEW_ACTIVITY_LOG_FILE      // New log file notification
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

### 1. SMP Proxy Not Working
- Verify using correct service/characteristic UUIDs
- Check target bit in flags (bit 3)
- Ensure D2D connection established for secondary
- Use mcumgr CLI to test: `mcumgr --conntype ble image list`

### 2. Commands Not Reaching Secondary
- Check D2D connection established
- Verify service discovery completed
- Look for "D2D service discovery complete!"
- For SMP: verify target bit = 1 in flags

### 3. FOTA Fails on Secondary
- Reduce chunk size to 256 bytes
- Ensure D2D connection stable
- Check SMP client configured on primary
- Verify secondary not resetting early
- Add delays between operations

### 4. Build Errors
- Check CONFIG_PRIMARY_DEVICE setting
- Verify all includes present
- Clean build directory
- Use `rm -rf build_primary` or `rm -rf build_secondary` before rebuild

### 5. Quick Build & Flash Workflow
```bash
# For Primary Device (Right Foot)
./tools/build_primary.sh && ./tools/flash_primary.sh

# For Secondary Device (Left Foot)
./tools/build_secondary.sh && ./tools/flash_secondary.sh
```

### 6. Identifying Connected Device
```bash
# Check which device is connected
nrfjprog --ids

# Read device info (if programmed)
nrfjprog --memrd 0x00FF8000 --n 32
```

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

### FOTA Updates (SMP Proxy)
- [ ] Primary device FOTA via SMP (target bit = 0)
- [ ] Secondary device FOTA via SMP (target bit = 1)
- [ ] File download from primary
- [ ] File download from secondary
- [ ] Echo test works for both targets
- [ ] Standard MCUmgr tools work

### Legacy FOTA Updates
- [ ] Single device updates work
- [ ] Dual device update synchronizes
- [ ] Timeout works if secondary fails
- [ ] Status notifications correct

## 9. Code Patterns

### Using SMP Proxy (Recommended for Mobile Apps)

```swift
// Update secondary device using standard MCUmgr
func updateSecondaryDevice(firmware: Data) {
    // Use standard MCUmgr with SMP proxy characteristic
    let transport = McuMgrBleTransport(peripheral)
    transport.smpCharacteristic = "14387802-130c-49e7-b877-2881c89cb258"
    
    // Set target in SMP header (bit 3 of flags)
    transport.targetDevice = .secondary  // Sets flag bit 3 = 1
    
    // Standard MCUmgr operations work transparently!
    let dfuManager = FirmwareUpgradeManager(transporter: transport)
    dfuManager.start(data: firmware)
}

// File operations work the same way
func downloadFileFromSecondary(path: String) async throws -> Data {
    let transport = McuMgrBleTransport(peripheral)
    transport.smpCharacteristic = "14387802-130c-49e7-b877-2881c89cb258"
    transport.targetDevice = .secondary
    
    let fileManager = FileSystemManager(transporter: transport)
    return try await fileManager.download(path)
}
```

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

### SMP Proxy Optimization
1. **Chunk Sizes**: 512 bytes for primary, 256 for secondary
2. **Connection Interval**: Request 15ms for FOTA operations
3. **MTU**: Negotiate maximum (247 bytes typical)
4. **Parallel Operations**: Avoid simultaneous primary/secondary ops

### General Tips
1. Use write-without-response for speed
2. Batch multiple commands when possible
3. Monitor connection parameters
4. Keep payloads small (<20 bytes) for control commands

## 11. Security Notes

1. All characteristics use encryption
2. Bonding required for phone connection
3. D2D connection should validate peer
4. No sensitive data in logs

---

**End of Reference**
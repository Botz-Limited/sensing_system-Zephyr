# D2D Complete Implementation Documentation

## Overview

This document provides a comprehensive summary of the complete Device-to-Device (D2D) communication implementation between primary and secondary devices in the sensing firmware.

## Architecture

```
┌─────────────────┐                    ┌─────────────────┐
│   Mobile App    │                    │ Secondary Device│
│                 │                    │   (Left Shoe)   │
└────────┬────────┘                    └────────┬────────┘
         │                                      │
         │ BLE                                  │ BLE (D2D)
         │                                      │
┌────────▼────────────────────────────────────▼─────────┐
│                   Primary Device (Right Shoe)          │
│                                                        │
│  ┌─────────────────┐        ┌──────────────────────┐  │
│  │ Phone-Facing    │        │ D2D Communication    │  │
│  │ Services        │        │                      │  │
│  ├─────────────────┤        ├──────────────────────┤  │
│  │ Info Service    │◄───────┤ D2D RX Client       │  │
│  │ Control Service │        │ (receives from sec) │  │
│  │ SMP Proxy       │        ├──────────────────────┤  │
│  │ File Proxy      │        │ D2D TX              │  │
│  │ FOTA Proxy      │────────► (sends to sec)      │  │
│  └─────────────────┘        └──────────────────────┘  │
└────────────────────────────────────────────────────────┘
```

## Services and Characteristics

### 1. Information Service (Primary Device Only)
**UUID**: `0c372eaa-27eb-437e-bef4-775aefaf3c97`

#### Primary Device Characteristics:
- Current Time
- Device Status  
- Error Status
- Foot Sensor Data
- BHI360 Data (3 characteristics)
- Charge Status
- Log Available (Foot, BHI360)
- Log Paths (Foot, BHI360)
- FOTA Progress

#### Secondary Device Characteristics (NEW):
- Secondary Device Info (5 characteristics)
- Secondary FOTA Progress (`0c372ebb`)
- Secondary Foot Log Available (`0c372ebc`)
- Secondary Foot Log Path (`0c372ebd`)
- Secondary BHI360 Log Available (`0c372ebe`)
- Secondary BHI360 Log Path (`0c372ebf`)
- Secondary Activity Log Available (`0c372ec0`)
- Secondary Activity Log Path (`0c372ec1`)

### 2. Control Service (Primary Device)
**UUID**: `4fd5b67f-9d89-4061-92aa-319ca786baae`

#### Primary-Only Commands:
- Set Time
- Delete Foot Log
- Delete BHI360 Log
- Delete Activity Log
- Start/Stop Activity
- Trigger BHI360 Calibration

#### Secondary-Only Commands (NEW):
- Delete Secondary Foot Log (`4fd5b688`)
- Delete Secondary BHI360 Log (`4fd5b689`)
- Delete Secondary Activity Log (`4fd5b68a`)

### 3. D2D TX Service (Secondary Device)
**UUID**: `75ad68d6-200c-437d-98b5-061862076c5f`

#### Data Characteristics (Secondary → Primary):
- Foot Sensor Data (`76ad68dc`)
- BHI360 Data1 (`76ad68dd`)
- BHI360 Data2 (`76ad68de`)
- BHI360 Data3 (`76ad68df`)
- Status (`76ad68d6`)
- Charge Status (`76ad68d8`)
- Foot Log Available (`76ad68d7`)
- BHI360 Log Available (`76ad68da`)
- Device Info (`76ad68e1`)
- FOTA Progress (`76ad68e3`)
- Foot Log Path (`76ad68d9`)
- BHI360 Log Path (`76ad68db`)
- Activity Log Available (`76ad68e4`)
- Activity Log Path (`76ad68e5`)

### 4. D2D RX Service (Secondary Device)
**UUID**: `e060ca1f-3115-4ad6-9709-8c5ff3bf558b`

#### Command Characteristics (Primary → Secondary):
- Set Time (`e160ca80`)
- Delete Foot Log (`e160ca81`)
- Delete BHI360 Log (`e160ca82`)
- Start Activity (`e160ca83`)
- Stop Activity (`e160ca84`)
- FOTA Status (`e160ca86`)
- Trigger BHI360 Calibration (`e160ca87`)
- Delete Activity Log (`e160ca88`)

### 5. Proxy Services (Primary Device Only)

#### SMP Proxy Service
**UUID**: `8D53DC1E-1DB7-4CD3-868B-8A527460AA84`
- Unified MCUmgr interface for both devices
- Mobile app uses standard MCUmgr protocol

#### File Proxy Service
**UUID**: `8D53DC1D-1DB7-4CD3-868B-8A527460AA84`
- Transparent file access to both devices
- Path-based routing (primary: `/lfs1/`, secondary: `/lfs2/`)

#### FOTA Proxy Service
**UUID**: `8D53DC1C-1DB7-4CD3-868B-8A527460AA84`
- Firmware updates for both devices
- Progress tracking via Information Service

## Data Flow Examples

### 1. Sensor Data Flow (Secondary → Primary → Phone)
```
Secondary Device:
1. Foot sensor generates data
2. Data sent to bluetooth_msgq
3. bluetooth.cpp forwards via ble_d2d_tx_send_foot_sensor_data()
4. d2d_tx_notify_foot_sensor_data() sends notification

Primary Device:
1. d2d_rx_client receives notification
2. foot_sensor_notify_handler() processes data
3. d2d_data_handler_process_foot_samples() called
4. Data forwarded to phone via jis_foot_sensor_notify()
```

### 2. Command Flow (Phone → Primary → Secondary)
```
Phone:
1. Writes delete command to Control Service

Primary Device:
1. Control service handler receives command
2. For secondary-specific command:
   - Calls ble_d2d_tx_send_delete_*_log_command()
3. Command sent via D2D TX to secondary

Secondary Device:
1. D2D RX service receives command
2. Handler queues message to data_msgq
3. Data module processes deletion
```

### 3. FOTA Progress Flow
```
Secondary Device:
1. FOTA update in progress
2. Progress sent to bluetooth_msgq
3. ble_d2d_tx_send_fota_progress() called
4. d2d_tx_notify_fota_progress() sends notification

Primary Device:
1. fota_progress_notify_handler() receives update
2. jis_secondary_fota_progress_notify() called
3. Phone notified via Secondary FOTA Progress characteristic
```

## File Management

### Primary Device Files
- Stored in `/lfs1/`
- Managed via primary delete commands
- Paths sent via primary log path characteristics

### Secondary Device Files  
- Stored in `/lfs2/` (from primary's perspective)
- Managed via secondary-specific delete commands
- Paths sent via secondary log path characteristics
- File Proxy handles transparent access

## Key Implementation Files

### Primary Device
- `information_service.cpp` - Phone-facing data service
- `control_service.cpp` - Phone-facing command service
- `ble_d2d_rx_client.cpp` - Receives data from secondary
- `ble_d2d_tx.cpp` - Sends commands to secondary
- `d2d_data_handler.cpp` - Processes secondary data
- `smp_proxy.cpp` - MCUmgr proxy
- `file_proxy.cpp` - File access proxy
- `fota_proxy.cpp` - FOTA proxy

### Secondary Device
- `ble_d2d_tx_service.cpp` - Sends data to primary
- `ble_d2d_rx.cpp` - Receives commands from primary
- `ble_d2d_file_transfer.cpp` - File transfer support

### Common
- `bluetooth.cpp` - Main BLE thread and message routing
- `app.hpp` - Common data structures

## Testing Checklist

### Basic Connectivity
- [ ] Secondary connects to primary automatically
- [ ] Primary accepts phone connections
- [ ] Both connections stable simultaneously

### Data Flow Testing
- [ ] Foot sensor data flows: Secondary → Primary → Phone
- [ ] BHI360 data flows: Secondary → Primary → Phone  
- [ ] Status updates flow correctly
- [ ] Charge status updates work
- [ ] Device info transmitted on connection

### Command Testing
- [ ] Set time cascades to both devices
- [ ] Primary delete commands affect only primary
- [ ] Secondary delete commands affect only secondary
- [ ] Start/Stop activity works on both devices
- [ ] BHI360 calibration triggers on both

### File Management
- [ ] Primary log notifications work
- [ ] Secondary log notifications work
- [ ] File paths transmitted correctly
- [ ] File Proxy accesses both filesystems
- [ ] Delete commands work correctly

### FOTA Testing
- [ ] Primary FOTA progress visible
- [ ] Secondary FOTA progress visible
- [ ] Both devices can be updated
- [ ] Progress percentages accurate

### Error Handling
- [ ] Disconnection recovery works
- [ ] Command failures handled gracefully
- [ ] File not found errors handled
- [ ] FOTA failures reported correctly

## Mobile App Integration

### Service Discovery
1. Connect to primary device (name: "SensingGR")
2. Discover Information Service for data
3. Discover Control Service for commands
4. Discover SMP Proxy for MCUmgr
5. Subscribe to all notify characteristics

### Characteristic Usage
- **Primary data**: Use standard characteristics
- **Secondary data**: Use secondary-specific characteristics
- **Commands**: Use appropriate delete commands for each device
- **File access**: Use File Proxy with correct paths
- **FOTA**: Monitor both progress characteristics

### Best Practices
1. Always check which device a file belongs to before deletion
2. Monitor both FOTA progress characteristics during updates
3. Use path prefixes to identify device (`/lfs1/` vs `/lfs2/`)
4. Handle connection state changes gracefully
5. Implement retry logic for failed commands

## Troubleshooting

### Connection Issues
- Check BLE advertising name (Primary: "SensingGR", Secondary: "SensingGL")
- Verify no pairing/bonding required for D2D
- Ensure primary is advertising when no phone connected

### Data Flow Issues
- Check subscription status in logs
- Verify message queue not full
- Ensure correct sender in messages
- Check thread priorities

### Command Issues
- Verify correct characteristic UUID
- Check command format and length
- Monitor both device logs
- Verify message queue processing

### File Issues
- Check file path format
- Verify filesystem mounted
- Check available space
- Monitor proxy service logs
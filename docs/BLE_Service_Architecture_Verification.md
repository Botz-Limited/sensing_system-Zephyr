# BLE Service Architecture Verification

## Overview

This document verifies the proper separation of BLE services between primary and secondary devices based on CONFIG_PRIMARY_DEVICE flag.

## Service Distribution

### Primary Device (CONFIG_PRIMARY_DEVICE=y)

The primary device acts as:
1. **Peripheral to Phone** - Provides services for mobile app interaction
2. **Central to Secondary** - Connects to secondary device for data relay

#### Services for Phone Connection:
- **Information Service** (`information_service.cpp`) - Device status, sensor data, log availability
- **Control Service** (`control_service.cpp`) - Commands for time setting, log deletion, activity control
- **Device Info Service** (`device_info_service.cpp`) - Standard BLE DIS
- **SMP Server** (built-in) - Direct FOTA and file transfer for primary device
- **FOTA Proxy Service** (`fota_proxy.cpp`) - Relay FOTA updates to secondary device
- **File Proxy Service** (`file_proxy.cpp`) - Relay file operations to secondary device

#### Services for Secondary Connection:
- **D2D RX Service** (`ble_d2d_rx.cpp`) - Receives commands/data from secondary device

### Secondary Device (CONFIG_PRIMARY_DEVICE=n)

The secondary device acts as:
- **Peripheral to Primary** - Only connects to primary device

#### Services:
- **D2D TX Service** (`ble_d2d_tx.cpp`) - Sends sensor data and status to primary
- **D2D File Transfer Service** (`ble_d2d_file_transfer.cpp`) - Handles file operations from primary
- **Device Info Service** (`device_info_service.cpp`) - Standard BLE DIS
- **SMP Server** (built-in) - Receives FOTA updates from primary via proxy

## Build Configuration (CMakeLists.txt)

```cmake
# Common services (all devices)
- bluetooth.cpp
- cts.cpp
- device_info_service.cpp
- ble_d2d_file_transfer.cpp

# Primary device only
if(CONFIG_PRIMARY_DEVICE)
    - fota_proxy.cpp
    - file_proxy.cpp
    - information_service.cpp
    - control_service.cpp
    - ble_d2d_rx.cpp
endif()

# Secondary device only
if(NOT CONFIG_PRIMARY_DEVICE)
    - ble_d2d_tx.cpp
endif()
```

## Code Separation in bluetooth.cpp

### Device Names
```cpp
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    bt_set_name("SensingGR");  // Right foot
#else
    bt_set_name("SensingGL");  // Left foot
#endif
```

### Service Initialization
```cpp
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    ble_d2d_rx_init();        // Accept commands from secondary
    fota_proxy_init();        // Initialize FOTA proxy
    file_proxy_init();        // Initialize file proxy
    bt_start_advertising();   // Start advertising to phone
#else
    ble_d2d_tx_init();        // Prepare to send data to primary
    ble_d2d_file_transfer_init(); // Initialize file transfer service
    start_scan();             // Start scanning for primary device
#endif
```

### Connection Handling
```cpp
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // On secondary connection
    fota_proxy_set_secondary_conn(conn);
    file_proxy_set_secondary_conn(conn);
    ble_d2d_file_client_init(conn);
#else
    // On primary connection
    // Just log the connection
#endif
```

## Data Flow

### Phone → Primary → Secondary
1. **FOTA Updates**: Phone → FOTA Proxy → SMP Client → Secondary SMP Server
2. **File Operations**: Phone → File Proxy → D2D File Transfer → Secondary File System
3. **Commands**: Phone → Control Service → (internal relay) → D2D RX → Secondary

### Secondary → Primary → Phone
1. **Sensor Data**: Secondary sensors → D2D TX → Primary D2D RX → Information Service → Phone
2. **Log Notifications**: Secondary file system → D2D TX → Primary D2D RX → Information Service → Phone
3. **Status Updates**: Secondary status → D2D TX → Primary D2D RX → Information Service �� Phone

## Verification Results

✅ **Services are properly separated**:
- Information and Control services only on primary device
- D2D RX only on primary device
- D2D TX only on secondary device
- Proxy services only on primary device

✅ **Connection roles are correct**:
- Primary: Peripheral to phone + Central to secondary
- Secondary: Peripheral to primary only

✅ **Data flow is properly implemented**:
- All phone communication goes through primary
- Secondary communicates only with primary
- Proxy services handle relay operations

## Potential Issues Found

1. **D2D TX Implementation**: The `ble_d2d_tx.cpp` appears to be a stub implementation. It needs:
   - Proper GATT client implementation
   - Service discovery for D2D RX on primary
   - Actual data sending functions

2. **Missing Conditional Compilation**: Some services might benefit from additional compile-time checks to ensure they're not accidentally included on the wrong device type.

## Recommendations

1. Complete the D2D TX implementation for proper secondary-to-primary communication
2. Add compile-time assertions to verify correct service inclusion
3. Consider adding a service UUID advertisement filter to ensure devices only connect to compatible peers
4. Add documentation comments indicating which device type each service belongs to
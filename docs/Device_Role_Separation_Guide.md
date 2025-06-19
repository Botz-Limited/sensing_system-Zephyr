# Device Role Separation Guide

## Overview

The sensing firmware supports two distinct device roles from a single codebase:
- **Primary Device**: Right foot, connects to mobile phone
- **Secondary Device**: Left foot, connects to primary device

## Build Configuration

### Compile-Time Selection
```bash
# Build for Primary Device (default)
west build -b <board> -- -DCONFIG_PRIMARY_DEVICE=y

# Build for Secondary Device
west build -b <board> -- -DCONFIG_PRIMARY_DEVICE=n
```

## Role-Specific Services

### Primary Device Services

1. **Phone-Facing Services** (Only on Primary)
   - Control Service (`control_service.cpp`)
   - Information Service (`information_service.cpp`)
   - FOTA Proxy (`fota_proxy.cpp`)
   - File Proxy (`file_proxy.cpp`)

2. **D2D Services** (Both directions)
   - D2D RX Service - Receives sensor data from secondary
   - D2D TX Module - Sends commands to secondary

### Secondary Device Services

1. **D2D Services** (Both directions)
   - D2D TX Module - Sends sensor data to primary
   - D2D RX Service - Receives commands from primary

2. **No Phone-Facing Services**
   - Does NOT compile control/information services
   - Cannot connect directly to phone

## Communication Flow

### Command Flow (Phone → Primary → Secondary)
```
Mobile Phone
    ↓ (Control Service)
Primary Device
    ↓ (D2D TX)
Secondary Device (D2D RX)
```

### Data Flow (Secondary → Primary → Phone)
```
Secondary Device (D2D TX)
    ↓
Primary Device (D2D RX)
    ↓ (Information Service)
Mobile Phone
```

## Code Organization

### CMakeLists.txt Structure
```cmake
# Primary device specific services
zephyr_library_sources_ifdef(CONFIG_PRIMARY_DEVICE 
    fota_proxy.cpp 
    file_proxy.cpp
    information_service.cpp
    control_service.cpp
    ble_d2d_rx.cpp  # Receives from secondary
    ble_d2d_tx.cpp  # Sends to secondary
)

# Secondary device specific services
if(NOT CONFIG_PRIMARY_DEVICE)
    zephyr_library_sources(
        ble_d2d_tx.cpp  # Sends to primary
        ble_d2d_rx.cpp  # Receives from primary
    )
endif()
```

### Conditional Compilation Pattern
```c
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Primary-only code
    jis_foot_sensor_notify(data);  // Notify phone
    ble_d2d_tx_send_command(cmd);  // Forward to secondary
#endif

#if IS_ENABLED(CONFIG_SECONDARY_DEVICE)
    // Secondary-only code
    ble_d2d_tx_send_sensor_data(data);  // Send to primary
#endif
```

## Key Differences

### Primary Device
- BLE Peripheral role (advertises to phone)
- BLE Peripheral role (accepts secondary connection)
- Runs control command handlers
- Forwards commands to secondary
- Aggregates data from both feet
- Manages FOTA for both devices

### Secondary Device
- BLE Central role (scans for primary)
- No direct phone connection
- Receives forwarded commands
- Sends sensor data to primary
- Local data logging only

## Testing Role Separation

### Build Tests
1. Build with `CONFIG_PRIMARY_DEVICE=y`
   - Verify control/information services are included
   - Check that D2D TX/RX are both included

2. Build with `CONFIG_PRIMARY_DEVICE=n`
   - Verify control/information services are NOT included
   - Check that D2D TX/RX are both included

### Runtime Tests
1. Primary Device
   - Should advertise as "SensingGR"
   - Should accept phone connections
   - Should accept secondary connections
   - Should forward commands

2. Secondary Device
   - Should scan for "SensingGL"
   - Should NOT advertise to phones
   - Should connect to primary
   - Should receive commands

## Common Pitfalls to Avoid

1. **Service Overlap**
   - Never expose phone-facing services on secondary
   - Always check CONFIG_PRIMARY_DEVICE before phone operations

2. **Connection Management**
   - Primary manages two connections (phone + secondary)
   - Secondary manages one connection (primary only)

3. **Data Flow**
   - Secondary never talks directly to phone
   - All phone communication goes through primary

4. **Command Forwarding**
   - Only primary forwards commands
   - Secondary only receives and executes

## Debugging Tips

### Log Messages
- Primary: "Secondary device connected!"
- Secondary: "Connected to primary device"
- D2D TX: "Forwarding command..."
- D2D RX: "Received command..."

### Connection States
- Check `d2d_conn` pointer on both devices
- Verify service discovery completes
- Monitor command forwarding logs
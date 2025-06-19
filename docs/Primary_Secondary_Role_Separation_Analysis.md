# Primary/Secondary Device Role Separation Analysis

## Current Configuration

### Build Configuration (Kconfig)
- `CONFIG_PRIMARY_DEVICE`: Boolean flag to build as primary device
- `CONFIG_SECONDARY_DEVICE`: Automatically set as `!PRIMARY_DEVICE`
- Secondary device gets additional BT configurations (CENTRAL, OBSERVER, GATT_CLIENT)

### Current Issues Found

1. **CMakeLists.txt Mismatch**
   - Primary includes: `ble_d2d_rx.cpp` (receives from secondary)
   - Secondary includes: `ble_d2d_tx.cpp` (sends to primary)
   - **PROBLEM**: Primary needs TX to forward commands to secondary!

2. **Service Compilation**
   - `control_service.cpp` - Only compiled for PRIMARY
   - `information_service.cpp` - Only compiled for PRIMARY
   - `ble_d2d_rx.cpp` - Only compiled for PRIMARY
   - `ble_d2d_tx.cpp` - Only compiled for SECONDARY

## Required Changes

### 1. Fix CMakeLists.txt
The D2D communication should be bidirectional:
- Primary needs both RX (receive sensor data) and TX (forward commands)
- Secondary needs both TX (send sensor data) and RX (receive commands)

### 2. Ensure Proper Conditional Compilation
All role-specific code must use:
```c
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Primary-only code
#endif

#if IS_ENABLED(CONFIG_SECONDARY_DEVICE)
    // Secondary-only code
#endif
```

### 3. Service Exposure
- **Primary Device Services**:
  - Control Service (from phone)
  - Information Service (to phone)
  - D2D RX Service (from secondary)
  - D2D TX Client (to secondary)
  
- **Secondary Device Services**:
  - D2D TX Service (to primary)
  - D2D RX Service (from primary)
  - NO Control/Information services

## Verification Checklist

1. ✓ Primary device can receive commands from phone
2. ✓ Primary device can forward commands to secondary
3. ✓ Secondary device can receive forwarded commands
4. ✓ Secondary device can send sensor data to primary
5. ✓ No service overlap between roles
6. ✓ Clear separation in CMakeLists.txt
7. ✓ Proper use of CONFIG flags throughout code
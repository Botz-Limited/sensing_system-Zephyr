# Legacy BLE Service Implementation

This directory contains the implementation of a legacy BLE service that mimics the behavior of the old device firmware to maintain compatibility with existing mobile applications.

## Overview

The legacy BLE service provides backward compatibility by implementing the exact same BLE services, characteristics, and data format as the old device. This is a temporary implementation controlled by the `CONFIG_LEGACY_BLE_ENABLED` configuration flag.

## Features

### 1. **Service and Characteristic UUIDs**
- **Insole Service**: `91bad492-b950-4226-aa2b-4ede9fa42f59`
- **Insole Data Characteristic**: `cba1d466-344c-4be3-ab3f-189f80dd7518` (Notify)
- **Config Characteristic**: `45678901-1234-5678-1234-56789abcdef3` (Write/Notify)
- **BNO Service**: `45678901-1234-5678-1234-56789abcdef2`
- **BNO Characteristic**: `45678901-1234-5678-1234-56789abcdef1` (Notify)

### 2. **Data Packet Format (107 bytes)**
1. **Timestamp** (6 bytes): Year, Month, Day, Hour, Minute, Second
2. **Insole data** (16 bytes): 8x uint16_t pressure values, MSB first
3. **Quaternion** (16 bytes): 4x float (w, z, y, x order)
4. **Euler angles** (12 bytes): 3x float (yaw, pitch, roll)
5. **Accelerometer** (12 bytes): 3x float (x, y, z)
6. **Linear acceleration** (12 bytes): 3x float (x, y, z)
7. **Gravity** (12 bytes): 3x float (x, y, z) - calculated from quaternion
8. **Magnetometer** (12 bytes): 3x float (z, y, x) - filled with zeros
9. **Temperature** (4 bytes): float - set to 0.0f
10. **Battery level** (4 bytes): float - fixed at 50.0f
11. **Checksum** (1 byte): XOR of all previous bytes

### 3. **Command Support**
The config characteristic accepts the following commands:
- `START` or `I1` or `I2`: Start streaming data
- `STOP`: Stop streaming data

Command responses are sent as notifications on the config characteristic:
- `OK`: Command accepted
- `STOPPED`: Streaming stopped
- `ERROR`: Unknown command

### 4. **Auto-start Behavior**
The service automatically starts streaming data 2 seconds after initialization, matching the legacy firmware behavior.

## Configuration

To enable the legacy BLE service:

1. Set `CONFIG_LEGACY_BLE_ENABLED=y` in your configuration file
2. Ensure `CONFIG_SENSOR_DATA_MODULE=y` is also enabled (required dependency)

Example configuration file (`prj_legacy_test.conf`):
```conf
# Include the main configuration
#include "prj.conf"

# Enable legacy BLE support
CONFIG_LEGACY_BLE_ENABLED=y
```

## Integration

The legacy service integrates with the current firmware through:

1. **Sensor Data Module**: Uses `get_sensor_snapshot()` to obtain current sensor readings
2. **Bluetooth Module**: Automatically initialized when `CONFIG_LEGACY_BLE_ENABLED` is set
3. **Isolated Implementation**: All code is wrapped in `#if CONFIG_LEGACY_BLE_ENABLED` to ensure zero impact when disabled

## Data Sources

- **Quaternion**: From BHI360 motion sensor
- **Linear Acceleration**: From BHI360 motion sensor
- **Gyroscope**: From BHI360 motion sensor
- **Pressure**: From foot sensors (right foot on primary device, left foot on secondary)
- **Accelerometer**: Calculated by adding gravity to linear acceleration
- **Gravity**: Calculated from quaternion orientation
- **Magnetometer**: Not available (BHI360 has no magnetometer) - filled with zeros
- **Temperature**: Not available - set to 0.0f
- **Battery**: Not available in real-time - fixed at 50.0f

## Limitations

1. **Magnetometer data**: The BHI360 sensor doesn't have a magnetometer, so this field is filled with zeros
2. **Temperature**: Cannot read temperature in real-time, set to 0.0f
3. **Battery level**: Fixed at 50% as per requirements
4. **Data rate**: Fixed at 20Hz (50ms intervals)

## Testing

To test the legacy BLE service:

1. Build with `CONFIG_LEGACY_BLE_ENABLED=y`
2. Connect with a BLE client (mobile app or debugging tool)
3. Subscribe to notifications on the insole characteristic
4. Data should start streaming automatically
5. Send commands to the config characteristic to control streaming

## Future Considerations

This is a temporary implementation. Once the mobile app is updated to use the new BLE services, this legacy support can be removed by setting `CONFIG_LEGACY_BLE_ENABLED=n`.
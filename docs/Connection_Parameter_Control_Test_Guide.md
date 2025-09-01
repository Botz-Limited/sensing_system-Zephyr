# Connection Parameter Control Test Guide

## Overview

This guide explains how to test the new connection parameter control feature that allows mobile apps to optimize BLE connection parameters for background execution.

## Feature Details

### Control Service Characteristic
- **Service UUID**: `4fd5b67f-9d89-4061-92aa-319ca786baae` (Control Service)
- **Characteristic UUID**: `4fd5b68b-9d89-4061-92aa-319ca786baae`
- **Properties**: Read, Write
- **Permissions**: Encrypted Read/Write
- **Data Format**: 1 byte (uint8_t)

### Connection Profiles

| Profile | Value | Interval | Latency | Timeout | Use Case |
|---------|-------|----------|---------|---------|----------|
| FOREGROUND | 0 | 15-30ms | 0 | 4s | Active app, real-time data |
| BACKGROUND | 1 | 50-100ms | 4 | 6s | Background app, power saving |
| BACKGROUND_IDLE | 2 | 200-500ms | 10 | 10s | Idle background, max power saving |

## Testing Steps

### 1. Connect to Device
- Use a BLE debugging app (e.g., nRF Connect, LightBlue)
- Connect to the sensing device
- Navigate to Control Service

### 2. Read Current Profile
- Find the Connection Parameter Control characteristic
- Read the current value (should be 0 for FOREGROUND by default)

### 3. Test Profile Changes

#### Switch to Background Mode:
```
Write: 0x01
Expected: Connection interval changes to 50-100ms
```

#### Switch to Background Idle Mode:
```
Write: 0x02
Expected: Connection interval changes to 200-500ms
```

#### Return to Foreground Mode:
```
Write: 0x00
Expected: Connection interval changes back to 15-30ms
```

### 4. Verify Changes
- Monitor connection parameters in your BLE tool
- Check device logs for confirmation messages
- Observe data rate changes

## Expected Log Output

When changing profiles, you should see logs like:
```
[INF] Mobile app requested connection profile change to: 1
[INF] Requesting connection parameter update to BACKGROUND profile:
[INF]   Interval: 50-100 ms (units: 40-80)
[INF]   Latency: 4, Timeout: 6000 ms
[INF] Connection parameter update requested successfully
[INF] Updating sensor data rates for BACKGROUND profile:
[INF]   Foot sensor: 50 ms
[INF]   Motion sensor: 100 ms
[INF]   Data aggregation: enabled (count: 3)
```

## Mobile App Integration Example

### iOS (Swift)
```swift
// Find the characteristic
let connParamUUID = CBUUID(string: "4fd5b68b-9d89-4061-92aa-319ca786baae")
let characteristic = service.characteristics?.first { $0.uuid == connParamUUID }

// Set background mode when app enters background
func applicationDidEnterBackground() {
    let data = Data([0x01]) // BACKGROUND mode
    peripheral.writeValue(data, for: characteristic!, type: .withResponse)
}

// Restore foreground mode when app becomes active
func applicationDidBecomeActive() {
    let data = Data([0x00]) // FOREGROUND mode
    peripheral.writeValue(data, for: characteristic!, type: .withResponse)
}
```

### Android (Kotlin)
```kotlin
// Find the characteristic
val connParamUUID = UUID.fromString("4fd5b68b-9d89-4061-92aa-319ca786baae")
val characteristic = service.getCharacteristic(connParamUUID)

// Set background mode
fun onEnterBackground() {
    val data = byteArrayOf(0x01) // BACKGROUND mode
    characteristic.value = data
    gatt.writeCharacteristic(characteristic)
}

// Restore foreground mode
fun onEnterForeground() {
    val data = byteArrayOf(0x00) // FOREGROUND mode
    characteristic.value = data
    gatt.writeCharacteristic(characteristic)
}
```

## Troubleshooting

### Write Fails
- Ensure connection is encrypted (paired/bonded)
- Check that value is 0, 1, or 2
- Verify characteristic permissions

### Parameters Don't Change
- Some phones may reject parameter updates
- Check phone's BLE settings
- Monitor controller response in logs

### Data Rate Doesn't Change
- Sensor rate changes are logged but not yet implemented
- This is expected in current version

## Notes

- Connection parameter updates are negotiated between devices
- The remote device (phone) may accept, reject, or modify the requested parameters
- Actual parameters may differ from requested values
- Changes take effect after negotiation completes (typically within 1-2 seconds)
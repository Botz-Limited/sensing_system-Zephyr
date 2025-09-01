# CRC32 for BLE Packets: Analysis and Alternatives

## Executive Summary

Adding CRC32 to each high-frequency sensor packet is **not recommended** due to significant overhead and redundancy with existing BLE error detection. This document analyzes the impact and provides better alternatives.

## Current Data Rates and Packet Sizes

### High-Frequency Data Streams

| Data Type | Rate | Current Size | With CRC32 | Overhead | Extra Bandwidth |
|-----------|------|--------------|------------|----------|-----------------|
| Foot Sensor | 20 Hz | 17 bytes | 21 bytes | +23.5% | +80 bytes/sec |
| BHI360 3D Mapping | 50 Hz | 16 bytes | 20 bytes | +25% | +200 bytes/sec |
| BHI360 Linear Accel | 50 Hz | 7 bytes | 11 bytes | +57% | +200 bytes/sec |
| **Total Impact** | - | - | - | - | **+480 bytes/sec** |

### Bandwidth Analysis

Current total bandwidth for sensor data:
- Foot: 20 Hz × 17 bytes = 340 bytes/sec
- 3D Mapping: 50 Hz × 16 bytes = 800 bytes/sec  
- Linear Accel: 50 Hz × 7 bytes = 350 bytes/sec
- **Total: 1,490 bytes/sec**

With CRC32:
- **Total: 1,970 bytes/sec (+32% increase)**

## Why CRC32 is Redundant

### 1. BLE Already Has CRC24

BLE Link Layer includes a 24-bit CRC on every packet:
- Polynomial: 0x100065B
- Detects all single and double bit errors
- Detects all odd number of bit errors
- Detects all burst errors up to 24 bits

```
BLE Packet Structure:
┌─────────┬────────┬─────────┬────────┐
│ Preamble│ Access │ PDU     │ CRC24  │
│ (1B)    │ Addr(4B)│ (2-257B)│ (3B)   │
└─────────┴────────┴─────────┴────────┘
```

### 2. Additional Protocol Layers

Each layer adds its own integrity checks:
- **Link Layer**: CRC24 (hardware-enforced)
- **L2CAP**: Length validation
- **ATT**: Opcode and handle validation
- **GATT**: UUID and permission checks

### 3. Performance Impact

```c
// CRC32 computation overhead
uint32_t crc32(const uint8_t *data, size_t len) {
    // ~8 cycles per byte on ARM Cortex-M4
    // For 16-byte packet: ~128 cycles
    // At 64MHz: ~2μs per packet
    // At 50Hz: ~100μs/sec (0.16% CPU)
}
```

While CPU impact is minimal, the bandwidth impact is significant.

## Better Alternatives

### 1. Use Existing Sequence Numbers

We already implemented sequence numbers for packet loss detection:

```c
typedef struct {
    uint8_t seq_num;    // Already implemented!
    uint16_t values[8]; // Sensor data
} foot_samples_ble_t;
```

Benefits:
- Only 1 byte overhead (vs 4 bytes for CRC32)
- Detects packet loss
- Enables recovery mechanisms
- Already implemented

### 2. Implement Application-Level Checksums for Critical Data

For truly critical data, use lightweight checksums:

```c
// 8-bit XOR checksum - 1 byte overhead
uint8_t xor_checksum(const uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

// 16-bit Fletcher checksum - 2 bytes overhead
uint16_t fletcher16(const uint8_t *data, size_t len) {
    uint16_t sum1 = 0, sum2 = 0;
    for (size_t i = 0; i < len; i++) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return (sum2 << 8) | sum1;
}
```

### 3. Selective Protection for Low-Frequency Data

Add CRC32 only to low-frequency, high-value data:

| Data Type | Frequency | Add CRC32? | Rationale |
|-----------|-----------|------------|-----------|
| Sensor Data | 20-50 Hz | ❌ No | Too frequent, low criticality |
| Log Metadata | On change | ✅ Yes | Infrequent, high value |
| Calibration Data | Rare | ✅ Yes | Critical for accuracy |
| FOTA Chunks | Variable | ✅ Yes | Already implemented |

### 4. End-to-End Integrity for Log Files

Instead of per-packet CRC, use file-level integrity:

```c
typedef struct {
    uint32_t file_size;
    uint32_t crc32;      // CRC of entire file
    uint8_t sha256[32];  // Optional: stronger hash
} log_file_metadata_t;
```

## Recommended Approach

### 1. Keep Current Implementation

The existing sequence number mechanism provides:
- Packet loss detection
- Minimal overhead (1 byte)
- Recovery capability

### 2. Add Optional Integrity Check

For mobile apps that require extra validation:

```c
// Optional integrity field in packet
typedef struct {
    uint8_t seq_num;
    uint16_t values[8];
    #ifdef ENABLE_PACKET_CHECKSUM
    uint8_t checksum;  // XOR or Fletcher-8
    #endif
} foot_samples_ble_t;
```

### 3. Focus on System-Level Integrity

- **Log Files**: Add CRC32 to file headers
- **Calibration**: Protect calibration data with CRC
- **Configuration**: Validate settings with checksums

## Mobile App Recommendations

### 1. Monitor Sequence Numbers

```swift
func validatePacketSequence(_ packet: SensorPacket) -> Bool {
    let expectedSeq = (lastSequence + 1) & 0xFF
    if packet.sequence != expectedSeq {
        // Log packet loss
        let lossCount = (packet.sequence - expectedSeq) & 0xFF
        logPacketLoss(count: lossCount)
    }
    lastSequence = packet.sequence
    return true
}
```

### 2. Implement Statistical Validation

```kotlin
class SensorDataValidator {
    fun validateSensorData(data: FootSensorData): Boolean {
        // Check for impossible values
        if (data.values.any { it > 4095 }) return false
        
        // Check for stuck sensors
        if (data.values.all { it == lastData.values[0] }) return false
        
        // Rate of change validation
        val maxDelta = 1000 // Maximum change between samples
        for (i in data.values.indices) {
            if (abs(data.values[i] - lastData.values[i]) > maxDelta) {
                return false
            }
        }
        return true
    }
}
```

### 3. Use BLE Connection Monitoring

```swift
// Monitor connection quality
func centralManager(_ central: CBCentralManager, 
                   didReadRSSI RSSI: NSNumber, 
                   error: Error?) {
    if RSSI.intValue < -90 {
        // Poor signal, expect more packet loss
        enableDataValidation()
    }
}
```

## Conclusion

**Recommendation: Do NOT add CRC32 to high-frequency packets**

Reasons:
1. **Redundant**: BLE already provides CRC24
2. **Overhead**: 25-57% size increase
3. **Bandwidth**: 480 bytes/sec additional
4. **Battery**: More data = more power
5. **Complexity**: No significant benefit

Better approach:
1. Use existing sequence numbers
2. Add integrity checks to files, not packets
3. Implement app-level validation
4. Monitor connection quality

The current implementation with sequence numbers provides the right balance of overhead vs. reliability for high-frequency sensor data.
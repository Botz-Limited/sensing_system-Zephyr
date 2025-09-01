# Fixed-Point Conversion Guide

**Version:** 1.0  
**Date:** June 2025  
**Purpose:** Comprehensive guide to the fixed-point conversion system used for file logging and Bluetooth data transmission

---

## Table of Contents

1. [Overview](#1-overview)
2. [Why Fixed-Point?](#2-why-fixed-point)
3. [Scaling Factors](#3-scaling-factors)
4. [Data Structures](#4-data-structures)
5. [Conversion Functions](#5-conversion-functions)
6. [Implementation Details](#6-implementation-details)
7. [Usage Examples](#7-usage-examples)
8. [Space Savings Analysis](#8-space-savings-analysis)
9. [Best Practices](#9-best-practices)

---

## 1. Overview

The sensing firmware uses fixed-point integer representation for all sensor data in:
- **File logging** (Protocol Buffer format)
- **Bluetooth LE transmission**
- **Device-to-device (D2D) communication**

This system converts floating-point sensor values to scaled integers, significantly reducing storage space and transmission bandwidth while maintaining sufficient precision for all use cases.

### Key Benefits

- **40-48% reduction** in data size
- **Portable** across different architectures (no float endianness issues)
- **Efficient** protobuf encoding with variable-length integers
- **Predictable** precision across the entire range

---

## 2. Why Fixed-Point?

### Float vs Fixed-Point Comparison

| Aspect | Float (32-bit) | Fixed-Point (16-bit) | Savings |
|--------|----------------|---------------------|---------|
| **Size per value** | 4 bytes | 2 bytes | 50% |
| **BHI360 packet** | 40 bytes | 21 bytes | 48% |
| **Hourly log size** | 10 MB | 5.8 MB | 42% |
| **Precision** | Variable | Consistent | - |
| **Portability** | Endian-dependent | Universal | ✓ |

### Real-World Impact

For a typical 8-hour activity session:
- **Float format**: ~80 MB of log data
- **Fixed-point format**: ~46 MB of log data
- **Savings**: 34 MB (42.5%)

---

## 3. Scaling Factors

All scaling factors are defined in `include/app_fixed_point.hpp`:

```cpp
namespace FixedPoint {
    static constexpr int32_t QUAT_SCALE = 10000;     // Quaternions: ×10,000
    static constexpr int32_t ACCEL_SCALE = 1000;     // Acceleration: ×1,000
    static constexpr int32_t GYRO_SCALE = 10000;     // Gyroscope: ×10,000
    static constexpr int32_t ACCURACY_SCALE = 100;   // Accuracy: ×100
}
```

### Detailed Scaling Table

| Data Type | Scale Factor | Precision | Range | Example |
|-----------|--------------|-----------|-------|---------|
| **Quaternion** | 10,000 | 0.0001 | ±1.0 | 0.7071 → 7071 |
| **Linear Acceleration** | 1,000 | 0.001 m/s² | ±20 m/s² | 9.81 → 9810 |
| **Gyroscope** | 10,000 | 0.0001 rad/s | ±2.0 rad/s | 1.5708 → 15708 |
| **Accuracy** | 100 | 0.01 | 0-3.0 | 2.5 → 250 |

### Precision Analysis

- **Quaternion**: 0.0001 precision is more than sufficient for orientation (0.006° error max)
- **Acceleration**: 0.001 m/s² = 1 mm/s² precision (human motion rarely needs better)
- **Gyroscope**: 0.0001 rad/s = 0.0057°/s precision (excellent for motion tracking)

---

## 4. Data Structures

### BLE Transmission Structures

```cpp
// 15 bytes (vs 28 bytes float version)
typedef struct {
    int16_t quat_x;         // Quaternion X × 10,000
    int16_t quat_y;         // Quaternion Y × 10,000
    int16_t quat_z;         // Quaternion Z × 10,000
    int16_t quat_w;         // Quaternion W × 10,000
    int16_t gyro_x;         // Gyroscope X × 10,000
    int16_t gyro_y;         // Gyroscope Y × 10,000
    int16_t gyro_z;         // Gyroscope Z × 10,000
    uint8_t quat_accuracy;  // Accuracy × 100
} bhi360_3d_mapping_fixed_t;

// 6 bytes (vs 12 bytes float version)
typedef struct {
    int16_t x;  // Acceleration X × 1,000
    int16_t y;  // Acceleration Y × 1,000
    int16_t z;  // Acceleration Z × 1,000
} bhi360_linear_accel_fixed_t;
```

### Protocol Buffer Definitions

From `bhi360_sensor_messages_fixed.proto`:

```protobuf
message BHI360LogRecord {
    // Quaternion components scaled by 10,000
    sint32 quat_x = 1 [(nanopb).int_size = IS_16];  // 2 bytes
    sint32 quat_y = 2 [(nanopb).int_size = IS_16];
    sint32 quat_z = 3 [(nanopb).int_size = IS_16];
    sint32 quat_w = 4 [(nanopb).int_size = IS_16];
    
    // Quaternion accuracy scaled by 100
    uint32 quat_accuracy = 5 [(nanopb).int_size = IS_8];  // 1 byte
    
    // Linear acceleration scaled by 1,000
    sint32 lacc_x = 6 [(nanopb).int_size = IS_16];  // 2 bytes
    sint32 lacc_y = 7 [(nanopb).int_size = IS_16];
    sint32 lacc_z = 8 [(nanopb).int_size = IS_16];
    
    // Gyroscope scaled by 10,000
    sint32 gyro_x = 11 [(nanopb).int_size = IS_16];  // 2 bytes
    sint32 gyro_y = 12 [(nanopb).int_size = IS_16];
    sint32 gyro_z = 13 [(nanopb).int_size = IS_16];
    
    uint32 step_count = 9;
    uint32 delta_ms = 10 [(nanopb).int_size = IS_16];
}
```

---

## 5. Conversion Functions

### Basic Conversion Functions

```cpp
// Float to fixed-point
inline int16_t float_to_fixed16(float value, int32_t scale) {
    return static_cast<int16_t>(value * scale);
}

// Fixed-point to float
inline float fixed16_to_float(int16_t value, int32_t scale) {
    return static_cast<float>(value) / scale;
}
```

### Structure Conversion Functions

```cpp
// Convert complete 3D mapping structure
inline void convert_3d_mapping_to_fixed(
    const bhi360_3d_mapping_t& src, 
    bhi360_3d_mapping_fixed_t& dst
) {
    dst.quat_x = float_to_fixed16(src.accel_x, FixedPoint::QUAT_SCALE);
    dst.quat_y = float_to_fixed16(src.accel_y, FixedPoint::QUAT_SCALE);
    dst.quat_z = float_to_fixed16(src.accel_z, FixedPoint::QUAT_SCALE);
    dst.quat_w = float_to_fixed16(src.quat_w, FixedPoint::QUAT_SCALE);
    dst.gyro_x = float_to_fixed16(src.gyro_x, FixedPoint::GYRO_SCALE);
    dst.gyro_y = float_to_fixed16(src.gyro_y, FixedPoint::GYRO_SCALE);
    dst.gyro_z = float_to_fixed16(src.gyro_z, FixedPoint::GYRO_SCALE);
    dst.quat_accuracy = 0;  // Set separately if available
}

// Convert linear acceleration
inline void convert_linear_accel_to_fixed(
    const bhi360_linear_accel_t& src, 
    bhi360_linear_accel_fixed_t& dst
) {
    dst.x = float_to_fixed16(src.x, FixedPoint::ACCEL_SCALE);
    dst.y = float_to_fixed16(src.y, FixedPoint::ACCEL_SCALE);
    dst.z = float_to_fixed16(src.z, FixedPoint::ACCEL_SCALE);
}
```

---

## 6. Implementation Details

### File Logging Implementation

From `src/data/data.cpp`:

```cpp
// Convert and store BHI360 data for logging
bhi360_log_msg.payload.bhi360_log_record.quat_x = 
    float_to_fixed16(record->quat_x, FixedPoint::QUAT_SCALE);
bhi360_log_msg.payload.bhi360_log_record.quat_y = 
    float_to_fixed16(record->quat_y, FixedPoint::QUAT_SCALE);
// ... similar for other fields

// Quaternion accuracy uses direct multiplication
bhi360_log_msg.payload.bhi360_log_record.quat_accuracy = 
    static_cast<uint8_t>(record->quat_accuracy * FixedPoint::ACCURACY_SCALE);
```

### BLE Transmission Implementation

From `src/bluetooth/information_service.cpp`:

```cpp
// Convert float data to fixed-point for BLE transmission
convert_3d_mapping_to_fixed(*data, bhi360_data1_value_fixed);

// Send via BLE characteristic
err = bt_gatt_notify(NULL, &information_service.attrs[attr_index], 
                     &bhi360_data1_value_fixed, 
                     sizeof(bhi360_data1_value_fixed));
```

### D2D Communication

From `src/bluetooth/ble_d2d_rx_client.cpp`:

```cpp
// Receive fixed-point data and convert back to float
mapping.accel_x = fixed16_to_float(fixed_data->quat_x, FixedPoint::QUAT_SCALE);
mapping.accel_y = fixed16_to_float(fixed_data->quat_y, FixedPoint::QUAT_SCALE);
mapping.accel_z = fixed16_to_float(fixed_data->quat_z, FixedPoint::QUAT_SCALE);
mapping.quat_w = fixed16_to_float(fixed_data->quat_w, FixedPoint::QUAT_SCALE);
```

---

## 7. Usage Examples

### Mobile App - iOS Swift

```swift
class SensorDataDecoder {
    static let QUAT_SCALE: Float = 10000.0
    static let ACCEL_SCALE: Float = 1000.0
    static let GYRO_SCALE: Float = 10000.0
    static let ACCURACY_SCALE: Float = 100.0
    
    func decodeBHI360Data(_ data: Data) -> BHI360SensorData? {
        guard data.count >= 15 else { return nil }
        
        let buffer = data.withUnsafeBytes { $0.bindMemory(to: Int16.self) }
        
        return BHI360SensorData(
            quaternion: Quaternion(
                x: Float(buffer[0]) / Self.QUAT_SCALE,
                y: Float(buffer[1]) / Self.QUAT_SCALE,
                z: Float(buffer[2]) / Self.QUAT_SCALE,
                w: Float(buffer[3]) / Self.QUAT_SCALE
            ),
            gyroscope: Vector3(
                x: Float(buffer[4]) / Self.GYRO_SCALE,
                y: Float(buffer[5]) / Self.GYRO_SCALE,
                z: Float(buffer[6]) / Self.GYRO_SCALE
            ),
            accuracy: Float(data[14]) / Self.ACCURACY_SCALE
        )
    }
}
```

### Mobile App - Android Kotlin

```kotlin
object FixedPointDecoder {
    const val QUAT_SCALE = 10000f
    const val ACCEL_SCALE = 1000f
    const val GYRO_SCALE = 10000f
    const val ACCURACY_SCALE = 100f
    
    fun decodeLinearAccel(data: ByteArray): Vector3 {
        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        
        return Vector3(
            x = buffer.getShort().toFloat() / ACCEL_SCALE,
            y = buffer.getShort().toFloat() / ACCEL_SCALE,
            z = buffer.getShort().toFloat() / ACCEL_SCALE
        )
    }
}
```

### Python Log Decoder

```python
import struct

class FixedPointDecoder:
    QUAT_SCALE = 10000
    ACCEL_SCALE = 1000
    GYRO_SCALE = 10000
    ACCURACY_SCALE = 100
    
    @staticmethod
    def decode_bhi360_record(data):
        # Unpack fixed-point values (little-endian)
        values = struct.unpack('<hhhhhhhBhhhI', data[:27])
        
        return {
            'quaternion': {
                'x': values[0] / FixedPointDecoder.QUAT_SCALE,
                'y': values[1] / FixedPointDecoder.QUAT_SCALE,
                'z': values[2] / FixedPointDecoder.QUAT_SCALE,
                'w': values[3] / FixedPointDecoder.QUAT_SCALE
            },
            'linear_accel': {
                'x': values[5] / FixedPointDecoder.ACCEL_SCALE,
                'y': values[6] / FixedPointDecoder.ACCEL_SCALE,
                'z': values[7] / FixedPointDecoder.ACCEL_SCALE
            },
            'gyroscope': {
                'x': values[9] / FixedPointDecoder.GYRO_SCALE,
                'y': values[10] / FixedPointDecoder.GYRO_SCALE,
                'z': values[11] / FixedPointDecoder.GYRO_SCALE
            },
            'accuracy': values[8] / FixedPointDecoder.ACCURACY_SCALE,
            'step_count': values[12]
        }
```

---

## 8. Space Savings Analysis

### Per-Record Comparison

| Component | Float Size | Fixed-Point Size | Savings |
|-----------|------------|------------------|---------|
| Quaternion (x,y,z,w) | 16 bytes | 8 bytes | 50% |
| Linear Acceleration | 12 bytes | 6 bytes | 50% |
| Gyroscope | 12 bytes | 6 bytes | 50% |
| Accuracy | 4 bytes | 1 byte | 75% |
| **Total per record** | 44 bytes | 21 bytes | 52% |

### Hourly Data Volume

At 50Hz sampling rate:
- **Float format**: 44 bytes × 50 Hz × 3600s = 7.92 MB/hour
- **Fixed-point**: 21 bytes × 50 Hz × 3600s = 3.78 MB/hour
- **Savings**: 4.14 MB/hour (52%)

### Protocol Buffer Efficiency

Protocol Buffers add additional savings through:
- Variable-length encoding for small integers
- Efficient field tagging
- Optional field compression

Typical protobuf overhead: 5-8 bytes per record

---

## 9. Best Practices

### 1. Choose Appropriate Scales

- Use the predefined scales in `FixedPoint` namespace
- Don't create custom scales unless absolutely necessary
- Document any custom scales clearly

### 2. Handle Overflow

```cpp
// Safe conversion with bounds checking
int16_t safe_float_to_fixed16(float value, int32_t scale) {
    float scaled = value * scale;
    if (scaled > INT16_MAX) return INT16_MAX;
    if (scaled < INT16_MIN) return INT16_MIN;
    return static_cast<int16_t>(scaled);
}
```

### 3. Maintain Precision

- Always use the same scale factor for encoding and decoding
- Be aware of precision limits (e.g., quaternion precision is 0.0001)
- Consider using higher precision (32-bit) for critical calculations

### 4. Testing

- Test with extreme values (±max range)
- Verify round-trip conversion accuracy
- Check for accumulation of rounding errors

### 5. Documentation

- Always document the scale factor used
- Include units in comments (e.g., "mm/s²" for acceleration)
- Provide conversion examples in API documentation

### Example Documentation

```cpp
/**
 * @brief BHI360 3D mapping data in fixed-point format
 * 
 * All values are transmitted as fixed-point integers to save bandwidth:
 * - Quaternion components: multiply by 10,000 (range ±1.0)
 * - Gyroscope values: multiply by 10,000 (range ±2.0 rad/s)
 * - Accuracy: multiply by 100 (range 0-3.0)
 * 
 * Example decoding (Swift):
 *   let quat_x = Float(data.quat_x) / 10000.0
 */
typedef struct { ... } bhi360_3d_mapping_fixed_t;
```

---

## Appendix: Quick Reference

### Scale Factors
- Quaternion: ×10,000
- Linear Acceleration: ×1,000
- Gyroscope: ×10,000
- Accuracy: ×100

### Common Conversions
- 1.0 quaternion → 10000
- 9.81 m/s² → 9810
- 1.5708 rad/s (90°/s) → 15708
- 2.5 accuracy → 250

### Data Sizes
- Float BHI360 packet: 40 bytes
- Fixed-point BHI360 packet: 21 bytes
- Savings: 48%

---

**End of Guide**
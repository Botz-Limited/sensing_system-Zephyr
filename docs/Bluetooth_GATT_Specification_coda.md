<!-- CODA OPTIMIZED VERSION -->
<!-- Images are set to fit within Coda's column width -->
<!-- Click any image to view full size in new tab -->

# Bluetooth GATT Specification

**Version:** 2.0  
**Date:** December 2024  
**Scope:** Complete Bluetooth GATT services, characteristics, and protocols for mobile app and device integration  
**Purpose:** Comprehensive reference for BLE integration including fixed-point data formats, service definitions, and implementation examples

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [System Architecture](#2-system-architecture)
3. [Fixed-Point Data Format](#3-fixed-point-data-format)
4. [Standard Services](#4-standard-services)
5. [Information Service](#5-information-service)
6. [Control Service](#6-control-service)
7. [Proxy Services](#7-proxy-services)
8. [Device-to-Device (D2D) Services](#8-device-to-device-d2d-services)
9. [Data Structures](#9-data-structures)
10. [Integration Examples](#10-integration-examples)
11. [Error Codes and Troubleshooting](#11-error-codes-and-troubleshooting)

---

## 1. Introduction

This device implements a comprehensive set of Bluetooth Low Energy (BLE) GATT services for:
- Real-time sensor data transmission
- Device control and configuration
- Log file management
- Firmware updates (FOTA)
- Device-to-device communication

### Key Features
- **Fixed-point integer format** for optimal bandwidth usage
- **Encrypted communication** for all services
- **Dual-device architecture** with primary/secondary roles
- **40% bandwidth reduction** compared to floating-point format

<div style="max-width: 100%; overflow: hidden;">
<img src="https://raw.githubusercontent.com/Botz-Limited/sensing_system-Zephyr/Nrf5340-framework/docs/mermaid_images/Bluetooth_GATT_Specification/diagram_1.png" alt="Diagram 1" style="max-width: 100%; height: auto; cursor: pointer;" onclick="window.open(this.src, '_blank')">
</div>
<p style="text-align: center; font-size: 0.9em; color: #666;">
Diagram 1 (Click to view full size)
</p>
<!-- Original diagram was Mermaid format - see mermaid_images/Bluetooth_GATT_Specification/diagram_1.mmd -->

---

## 2. System Architecture

### Device Roles

<div style="max-width: 100%; overflow: hidden;">
<img src="https://raw.githubusercontent.com/Botz-Limited/sensing_system-Zephyr/Nrf5340-framework/docs/mermaid_images/Bluetooth_GATT_Specification/diagram_2.png" alt="Diagram 2" style="max-width: 100%; height: auto; cursor: pointer;" onclick="window.open(this.src, '_blank')">
</div>
<p style="text-align: center; font-size: 0.9em; color: #666;">
Diagram 2 (Click to view full size)
</p>
<!-- Original diagram was Mermaid format - see mermaid_images/Bluetooth_GATT_Specification/diagram_2.mmd -->

| Feature | Primary Device | Secondary Device |
|---------|----------------|------------------|
| Device Name | "SensingGR" | "SensingGL" |
| BLE Role | Peripheral + Central | Peripheral only |
| Phone Connection | Yes | No |
| D2D Connection | Central (initiates) | Peripheral (accepts) |
| Services | Full set | D2D only |

---

## 3. Fixed-Point Data Format

All sensor data uses fixed-point integers to optimize bandwidth and ensure portability.

### Scaling Factors

| Data Type | Scale Factor | Precision | Range | Example |
|-----------|--------------|-----------|-------|---------|
| Quaternion | 10,000 | 0.0001 | ±1.0 | 0.7071 → 7071 |
| Linear Acceleration | 1,000 | 0.001 m/s² | ±20 m/s² | 9.81 → 9810 |
| Gyroscope | 10,000 | 0.0001 rad/s | ±2.0 rad/s | 1.5708 → 15708 |
| Accuracy | 100 | 0.01 | 0-3.0 | 2.5 → 250 |

### Bandwidth Comparison

<div style="max-width: 100%; overflow: hidden;">
<img src="https://raw.githubusercontent.com/Botz-Limited/sensing_system-Zephyr/Nrf5340-framework/docs/mermaid_images/Bluetooth_GATT_Specification/diagram_3.png" alt="Diagram 3" style="max-width: 100%; height: auto; cursor: pointer;" onclick="window.open(this.src, '_blank')">
</div>
<p style="text-align: center; font-size: 0.9em; color: #666;">
Diagram 3 (Click to view full size)
</p>
<!-- Original diagram was Mermaid format - see mermaid_images/Bluetooth_GATT_Specification/diagram_3.mmd -->

### Conversion Functions

```c
// Encoding (Device → BLE)
int16_t encode_quaternion(float value) {
    return (int16_t)(value * 10000.0f);
}

int16_t encode_acceleration(float value) {
    return (int16_t)(value * 1000.0f);
}

// Decoding (BLE → App)
float decode_quaternion(int16_t fixed) {
    return (float)fixed / 10000.0f;
}

float decode_acceleration(int16_t fixed) {
    return (float)fixed / 1000.0f;
}
```

---

## 4. Standard Services

### Device Information Service (DIS)
**UUID:** `0000180A-0000-1000-8000-00805F9B34FB`

| Characteristic | UUID | Properties | Data Type |
|----------------|------|------------|-----------|
| Manufacturer Name | 0x2A29 | Read | String |
| Model Number | 0x2A24 | Read | String |
| Serial Number | 0x2A25 | Read | String |
| Hardware Revision | 0x2A27 | Read | String |
| Firmware Revision | 0x2A26 | Read | String |

### Battery Service
**UUID:** `0000180F-0000-1000-8000-00805F9B34FB`

| Characteristic | UUID | Properties | Data Type | Description |
|----------------|------|------------|-----------|-------------|
| Battery Level | 0x2A19 | Read, Notify | uint8_t | 0-100% |

### Current Time Service (CTS)
**UUID:** `00001805-0000-1000-8000-00805F9B34FB`

| Characteristic | UUID | Properties | Data Type |
|----------------|------|------------|-----------|
| Current Time | 0x2A2B | Read, Write, Notify | CTS struct |

---

## 5. Information Service

**UUID:** `0c372eaa-27eb-437e-bef4-775aefaf3c97`  
**Availability:** Primary device only

### Characteristics

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|----------------|-------------|------------|-----------|-------------|
| Current Time | 0x2A2B | Read, Notify | CTS format | Device time |
| Status | `...eab` | Read, Notify | uint32_t | Status bitfield |
| Foot Sensor Samples | `...eaf` | Read, Notify | foot_samples_t | 16 ADC channels |
| Foot Log Available | `...eac` | Read, Notify | uint8_t | Latest log ID |
| Charge Status | `...ead` | Read, Notify | uint8_t | 0-100% |
| Foot Log Path | `...eae` | Read, Notify | char[] | UTF-8 path |
| BHI360 Log Available | `...eb0` | Read, Notify | uint8_t | Latest log ID |
| BHI360 Log Path | `...eb1` | Read, Notify | char[] | UTF-8 path |
| **BHI360 3D Mapping** | `...eb2` | Read, Notify | bhi360_3d_mapping_fixed_t | **Fixed-point** |
| **BHI360 Step Count** | `...eb3` | Read, Notify | bhi360_step_count_t | Steps + duration |
| **BHI360 Linear Accel** | `...eb4` | Read, Notify | bhi360_linear_accel_fixed_t | **Fixed-point** |
| FOTA Progress | `...eb5` | Read, Notify | fota_progress_t | Update status |

### Status Bitfield

```c
#define STATUS_IDLE                    0x00000001
#define STATUS_LOGGING                 0x00000002
#define STATUS_ERROR                   0x00000004
#define STATUS_LOW_BATTERY             0x00000008
#define STATUS_CHARGING                0x00000010
#define STATUS_BLUETOOTH_CONNECTED     0x00000020
#define STATUS_D2D_CONNECTED           0x00000040
#define STATUS_FILE_SYSTEM_ERROR       0x00000080
#define STATUS_SENSOR_ERROR            0x00000100
#define STATUS_CALIBRATING             0x00000200
```

---

## 6. Control Service

**UUID:** `4fd5b67f-9d89-4061-92aa-319ca786baae`  
**Availability:** Primary device only

### Characteristics

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|----------------|-------------|------------|-----------|-------------|
| Set Time | `...b681` | Write | uint32_t | Epoch time (big-endian) |
| Delete Foot Log | `...b682` | Write, Notify | uint8_t | Log ID to delete |
| Delete BHI360 Log | `...b683` | Write, Notify | uint8_t | Log ID to delete |
| Start Activity | `...b684` | Write, Notify | uint8_t | Write 1 to start |
| Stop Activity | `...b685` | Write, Notify | uint8_t | Write 1 to stop |

### Command Flow

<div style="max-width: 100%; overflow: hidden;">
<img src="https://raw.githubusercontent.com/Botz-Limited/sensing_system-Zephyr/Nrf5340-framework/docs/mermaid_images/Bluetooth_GATT_Specification/diagram_4.png" alt="Diagram 4" style="max-width: 100%; height: auto; cursor: pointer;" onclick="window.open(this.src, '_blank')">
</div>
<p style="text-align: center; font-size: 0.9em; color: #666;">
Diagram 4 (Click to view full size)
</p>
<!-- Original diagram was Mermaid format - see mermaid_images/Bluetooth_GATT_Specification/diagram_4.mmd -->

---

## 7. Proxy Services

### 7.1 FOTA Proxy Service

**UUID:** `6e400001-b5a3-f393-e0a9-e50e24dcca9e`  
**Availability:** Primary device only  
**Purpose:** Firmware updates for secondary device via primary

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|----------------|-------------|------------|-----------|-------------|
| Target Selection | `...0002` | Write | uint8_t | 0x00=Primary, 0x01=Secondary, 0xFF=All |
| Command | `...0003` | Write | uint8_t + data | See command table |
| Data | `...0004` | Write | byte[] | Firmware chunks |
| Status | `...0005` | Read, Notify | uint8_t | Operation status |

#### FOTA Commands

| Command | Value | Data | Description |
|---------|-------|------|-------------|
| Start | 0x01 | 4 bytes size | Begin update |
| Data | 0x02 | Firmware bytes | Send chunk |
| End | 0x03 | None | Complete update |
| Abort | 0x04 | None | Cancel update |
| Query | 0x05 | None | Get status |
| Reset | 0x06 | None | Reset device |

#### FOTA Flow

<div style="max-width: 100%; overflow: hidden;">
<img src="https://raw.githubusercontent.com/Botz-Limited/sensing_system-Zephyr/Nrf5340-framework/docs/mermaid_images/Bluetooth_GATT_Specification/diagram_5.png" alt="Diagram 5" style="max-width: 100%; height: auto; cursor: pointer;" onclick="window.open(this.src, '_blank')">
</div>
<p style="text-align: center; font-size: 0.9em; color: #666;">
Diagram 5 (Click to view full size)
</p>
<!-- Original diagram was Mermaid format - see mermaid_images/Bluetooth_GATT_Specification/diagram_5.mmd -->

### 7.2 File Proxy Service

**UUID:** `7e500001-b5a3-f393-e0a9-e50e24dcca9e`  
**Availability:** Primary device only  
**Purpose:** Access log files on secondary device

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|----------------|-------------|------------|-----------|-------------|
| Target Device | `...0002` | Write | uint8_t | 0x00=Primary, 0x01=Secondary |
| File Command | `...0003` | Write | Command struct | See below |
| File Data | `...0004` | Notify | byte[] | File chunks |
| File Status | `...0005` | Read, Notify | uint8_t | Operation status |

#### File Commands

```c
typedef struct {
    uint8_t cmd;      // Command type
    uint8_t file_id;  // File ID
    uint8_t type;     // 0x01=Foot, 0x02=BHI360
} file_command_t;
```

| Command | Value | Description |
|---------|-------|-------------|
| List Files | 0x01 | Get file list |
| Read File | 0x02 | Read by ID |
| Delete File | 0x03 | Delete by ID |
| Get Info | 0x04 | Get file metadata |
| Abort | 0x05 | Cancel operation |

---

## 8. Device-to-Device (D2D) Services

### 8.1 D2D RX Service (Primary Device)

**UUID:** `e060ca1f-3115-4ad6-9709-8c5ff3bf558b`  
**Purpose:** Receive commands from phone to relay to secondary

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|----------------|-------------|------------|-----------|-------------|
| D2D Set Time | `...ca1f` | Write | uint32_t | Time relay |
| D2D Delete Foot Log | `...ca82` | Write | uint8_t | Delete command |
| D2D Delete BHI360 Log | `...ca83` | Write | uint8_t | Delete command |
| D2D Start Activity | `...ca84` | Write | uint8_t | Start command |
| D2D Stop Activity | `...ca85` | Write | uint8_t | Stop command |

### 8.2 D2D TX Service (Secondary Device)

**UUID:** `75ad68d6-200c-437d-98b5-061862076c5f`  
**Purpose:** Transmit sensor data from secondary to primary

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|----------------|-------------|------------|-----------|-------------|
| D2D Status | `...68d6` | Notify | uint32_t | Status bitfield |
| D2D Foot Log Available | `...68d7` | Notify | uint8_t | Log ID |
| D2D Charge Status | `...68d8` | Notify | uint8_t | Battery % |
| D2D Foot Log Path | `...68d9` | Notify | char[] | File path |
| D2D BHI360 Log Available | `...68da` | Notify | uint8_t | Log ID |
| D2D BHI360 Log Path | `...68db` | Notify | char[] | File path |
| D2D Foot Samples | `...68dc` | Notify | foot_samples_t | ADC data |
| **D2D BHI360 3D Mapping** | `...68dd` | Notify | bhi360_3d_mapping_fixed_t | **Fixed-point** |
| **D2D BHI360 Step Count** | `...68de` | Notify | bhi360_step_count_t | Steps |
| **D2D BHI360 Linear Accel** | `...68df` | Notify | bhi360_linear_accel_fixed_t | **Fixed-point** |
| D2D Current Time | `...68e0` | Notify | CTS struct | Time sync |

### 8.3 D2D File Transfer Service

**UUID:** `8e600001-b5a3-f393-e0a9-e50e24dcca9e`  
**Purpose:** File transfer between devices

| Characteristic | UUID Suffix | Properties | Data Type |
|----------------|-------------|------------|-----------|
| Command | `...0002` | Write | Command packet |
| Data | `...0003` | Notify | Data packet |
| Status | `...0004` | Notify | Status byte |

### D2D Architecture

<div style="max-width: 100%; overflow: hidden;">
<img src="https://raw.githubusercontent.com/Botz-Limited/sensing_system-Zephyr/Nrf5340-framework/docs/mermaid_images/Bluetooth_GATT_Specification/diagram_6.png" alt="Diagram 6" style="max-width: 100%; height: auto; cursor: pointer;" onclick="window.open(this.src, '_blank')">
</div>
<p style="text-align: center; font-size: 0.9em; color: #666;">
Diagram 6 (Click to view full size)
</p>
<!-- Original diagram was Mermaid format - see mermaid_images/Bluetooth_GATT_Specification/diagram_6.mmd -->

---

## 9. Data Structures

### Fixed-Point Structures

```c
// BHI360 3D Mapping - 15 bytes
typedef struct {
    int16_t quat_x;        // Quaternion X × 10000
    int16_t quat_y;        // Quaternion Y × 10000
    int16_t quat_z;        // Quaternion Z × 10000
    int16_t quat_w;        // Quaternion W × 10000
    int16_t gyro_x;        // Gyroscope X × 10000 (rad/s)
    int16_t gyro_y;        // Gyroscope Y × 10000 (rad/s)
    int16_t gyro_z;        // Gyroscope Z × 10000 (rad/s)
    uint8_t quat_accuracy; // Accuracy × 100 (0-300)
} __packed bhi360_3d_mapping_fixed_t;

// BHI360 Linear Acceleration - 6 bytes
typedef struct {
    int16_t x;  // Acceleration X × 1000 (mm/s²)
    int16_t y;  // Acceleration Y × 1000 (mm/s²)
    int16_t z;  // Acceleration Z × 1000 (mm/s²)
} __packed bhi360_linear_accel_fixed_t;

// BHI360 Step Count - 8 bytes
typedef struct {
    uint32_t step_count;
    uint32_t activity_duration_s;
} __packed bhi360_step_count_t;

// Foot Sensor Samples - 16 bytes
typedef struct {
    uint16_t values[8];  // 8 ADC channels
} __packed foot_samples_t;

// FOTA Progress - 15 bytes
typedef struct {
    uint8_t is_active;        // 0=inactive, 1=active
    uint8_t status;           // 0=idle, 1=progress, 2=pending, 3=confirmed, 4=error
    uint8_t percent_complete; // 0-100
    uint32_t bytes_received;
    uint32_t total_size;
    int32_t error_code;
} __packed fota_progress_t;
```

### SMP Service

**UUID:** `8D53DC1D-1DB7-4CD3-868B-8A527460AA84`  
**Characteristic:** `DA2E7828-FBCE-4E01-AE9E-261174997C48`  
**Purpose:** MCUmgr protocol for firmware updates and file access

---

## 10. Integration Examples

### iOS Swift - BLE Characteristic Handler

```swift
import CoreBluetooth

class BHI360DataHandler {
    static let QUAT_SCALE: Float = 10000.0
    static let ACCEL_SCALE: Float = 1000.0
    static let GYRO_SCALE: Float = 10000.0
    
    func handle3DMapping(_ data: Data) -> BHI360Data? {
        guard data.count >= 15 else { return nil }
        
        // Parse fixed-point values (little-endian)
        let buffer = data.withUnsafeBytes { $0.bindMemory(to: Int16.self) }
        
        return BHI360Data(
            quaternion: SIMD4<Float>(
                Float(buffer[0]) / Self.QUAT_SCALE,
                Float(buffer[1]) / Self.QUAT_SCALE,
                Float(buffer[2]) / Self.QUAT_SCALE,
                Float(buffer[3]) / Self.QUAT_SCALE
            ),
            gyroscope: SIMD3<Float>(
                Float(buffer[4]) / Self.GYRO_SCALE,
                Float(buffer[5]) / Self.GYRO_SCALE,
                Float(buffer[6]) / Self.GYRO_SCALE
            ),
            accuracy: Float(data[14]) / 100.0
        )
    }
}
```

### Android Kotlin - Service Discovery

```kotlin
class SensingDeviceManager(private val context: Context) {
    companion object {
        val INFO_SERVICE_UUID = UUID.fromString("0c372eaa-27eb-437e-bef4-775aefaf3c97")
        val BHI360_3D_UUID = UUID.fromString("0c372eb2-27eb-437e-bef4-775aefaf3c97")
        
        const val QUAT_SCALE = 10000f
        const val ACCEL_SCALE = 1000f
    }
    
    fun onServicesDiscovered(gatt: BluetoothGatt) {
        val service = gatt.getService(INFO_SERVICE_UUID) ?: return
        val characteristic = service.getCharacteristic(BHI360_3D_UUID) ?: return
        
        // Enable notifications
        gatt.setCharacteristicNotification(characteristic, true)
        
        val descriptor = characteristic.getDescriptor(CLIENT_CHARACTERISTIC_CONFIG)
        descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        gatt.writeDescriptor(descriptor)
    }
    
    fun parse3DMapping(data: ByteArray): BHI360Data {
        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        
        return BHI360Data(
            quaternion = Quaternion(
                buffer.getShort().toFloat() / QUAT_SCALE,
                buffer.getShort().toFloat() / QUAT_SCALE,
                buffer.getShort().toFloat() / QUAT_SCALE,
                buffer.getShort().toFloat() / QUAT_SCALE
            ),
            gyroscope = Vector3(
                buffer.getShort().toFloat() / QUAT_SCALE,
                buffer.getShort().toFloat() / QUAT_SCALE,
                buffer.getShort().toFloat() / QUAT_SCALE
            ),
            accuracy = buffer.get().toFloat() / 100f
        )
    }
}
```

### Python - FOTA Update Script

```python
import asyncio
from bleak import BleakClient, BleakScanner

FOTA_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
TARGET_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
COMMAND_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
STATUS_CHAR_UUID = "6e400005-b5a3-f393-e0a9-e50e24dcca9e"

async def update_secondary_device(address: str, firmware: bytes):
    async with BleakClient(address) as client:
        # Set target to secondary
        await client.write_gatt_char(TARGET_CHAR_UUID, bytes([0x01]))
        
        # Start FOTA with size
        size_bytes = len(firmware).to_bytes(4, 'little')
        await client.write_gatt_char(COMMAND_CHAR_UUID, bytes([0x01]) + size_bytes)
        
        # Send firmware in chunks
        chunk_size = 240
        for i in range(0, len(firmware), chunk_size):
            chunk = firmware[i:i+chunk_size]
            await client.write_gatt_char(COMMAND_CHAR_UUID, bytes([0x02]) + chunk)
            await asyncio.sleep(0.1)  # Prevent congestion
            
            # Check status
            status = await client.read_gatt_char(STATUS_CHAR_UUID)
            print(f"Progress: {i}/{len(firmware)} bytes, Status: {status[0]}")
        
        # Complete update
        await client.write_gatt_char(COMMAND_CHAR_UUID, bytes([0x03]))
        print("FOTA update complete!")

# Usage
asyncio.run(update_secondary_device("XX:XX:XX:XX:XX:XX", firmware_data))
```

---

## 11. Error Codes and Troubleshooting

### Common BLE Error Codes

| Error | Code | Description | Solution |
|-------|------|-------------|----------|
| ENOTCONN | -128 | Not connected | Ensure connection established |
| ENOMEM | -12 | Out of memory | Reduce notification rate |
| EINVAL | -22 | Invalid parameter | Check data format |
| EACCES | -13 | Access denied | Ensure proper bonding |
| ETIMEDOUT | -116 | Operation timeout | Check connection stability |

### Troubleshooting Guide

<div style="max-width: 100%; overflow: hidden;">
<img src="https://raw.githubusercontent.com/Botz-Limited/sensing_system-Zephyr/Nrf5340-framework/docs/mermaid_images/Bluetooth_GATT_Specification/diagram_7.png" alt="Diagram 7" style="max-width: 100%; height: auto; cursor: pointer;" onclick="window.open(this.src, '_blank')">
</div>
<p style="text-align: center; font-size: 0.9em; color: #666;">
Diagram 7 (Click to view full size)
</p>
<!-- Original diagram was Mermaid format - see mermaid_images/Bluetooth_GATT_Specification/diagram_7.mmd -->

### Security Considerations

1. **All services require encryption** (BT_GATT_PERM_READ_ENCRYPT)
2. **Bonding required** before accessing characteristics
3. **Pairing process** must complete successfully
4. **Fixed PIN** or numeric comparison for pairing

### Performance Optimization

1. **Connection Parameters**
   - Min interval: 7.5ms
   - Max interval: 30ms
   - Latency: 0
   - Timeout: 5000ms

2. **MTU Size**
   - Request 247 bytes for optimal throughput
   - Fallback to 23 bytes if not supported

3. **Notification Rate**
   - Sensor data: 50Hz max
   - Status updates: On change only
   - File transfers: Chunked with flow control

---

**End of Specification**
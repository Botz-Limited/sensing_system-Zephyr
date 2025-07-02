# Bluetooth GATT Specification

**Version:** 2.6  
**Date:** January 2025  
**Scope:** Complete Bluetooth GATT services, characteristics, and protocols for mobile app and device integration  
**Purpose:** Comprehensive reference for BLE integration including fixed-point data formats, service definitions, and implementation examples

---

## Changelog

### Version 2.6 (January 2025)
- Added Weight Measurement characteristic (`...ec6`) to Information Service
- Added Weight Measurement Trigger (`...b68c`) to Control Service
- Added D2D Weight Measurement (`...68e7`) for secondary device weight data
- Weight measurement provides total weight from all 16 pressure sensors

### Version 2.5 (January 2025)
- Identified UUID conflicts between 3D Orientation Service and Information Service characteristics
- Documented actual MCUmgr SMP service UUIDs vs. documentation discrepancies
- Added notes about implementation issues to be fixed in future firmware updates

### Version 2.4 (January 2025)
- Modified Information Service to only send aggregated step counts to mobile phones
- Individual foot step counts (BHI360 Step Count) are now deprecated for phone communication
- D2D communication between primary and secondary devices remains unchanged
- Mobile apps should use Total Step Count and Activity Step Count characteristics only

### Version 2.3 (January 2025)
- Added D2D Activity Step Count characteristic (`76ad68e6-...`) for secondary device activity steps
- Fixed activity step count separation between primary and secondary devices
- Removed unreliable heuristic for detecting activity steps from secondary
- Added Total Step Count characteristic (`...ec4`) for aggregated steps from both feet
- Added Activity Step Count characteristic (`...ec5`) for activity-specific step counting
- Deprecated `activity_duration_s` field in `bhi360_step_count_t` - now always 0

### Version 2.2 (January 2025)
- Updated step count characteristics to provide count only (time tracking moved to separate characteristics)
- Clarified that BHI360 Step Count (`...eb3`) provides global step count

### Version 2.1 (June 2025)
- Added Connection Parameter Control characteristic (`...b68b`) to Control Service
- Added secondary device delete commands (`...b688`, `...b689`, `...b68a`) to Control Service
- Added detailed documentation for connection profiles (Foreground, Background, Background Idle)
- Added mobile app integration examples for background execution optimization

### Version 2.0 (June 2025)
- Initial comprehensive specification

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
10. [Packet Sequencing and Recovery](#10-packet-sequencing-and-recovery)
11. [Integration Examples](#11-integration-examples)
12. [Error Handling and Status Communication](#12-error-handling-and-status-communication)
13. [Common BLE Error Codes](#13-common-ble-error-codes)

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

DrawFlowchart(
  Syntax(
    "APP[Mobile Phone<br/>BLE Client] -->|BLE| PRIM[Primary Device<br/>GATT Services]",
    "PRIM --> CONV[Fixed-Point<br/>Converter]",
    "CONV --> D2DC[D2D Central]",
    "D2DC -->|BLE D2D| D2DP[Secondary Device<br/>D2D Peripheral]",
    "D2DP --> SENS[Sensor Data]"
  ),
  "TB",
  "default"
)

---

## 2. System Architecture

### Device Roles

DrawFlowchart(
  Syntax(
    "PHONE[Mobile App] -->|Direct BLE| P1[Primary Device<br/>Phone Services]",
    "P1 --> P3[Proxy Services]",
    "P3 -->|Relay| P2[D2D Central]",
    "P2 -->|D2D BLE| S1[Secondary Device<br/>D2D Peripheral]",
    "S1 --> S2[Sensor Services]"
  ),
  "LR",
  "default"
)

| Feature | Primary Device | Secondary Device |
|---:|:---:|:---:|
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
|---:|---:|---:|---:|---:|
| Quaternion | 10,000 | 0.0001 | ±1.0 | 0.7071 → 7071 |
| Linear Acceleration | 1,000 | 0.001 m/s² | ±20 m/s² | 9.81 → 9810 |
| Gyroscope | 10,000 | 0.0001 rad/s | ±2.0 rad/s | 1.5708 → 15708 |
| Accuracy | 100 | 0.01 | 0-3.0 | 2.5 → 250 |

### Bandwidth Comparison

DrawFlowchart(
  Syntax(
    "F1[Float Format<br/>28 bytes<br/>3D Mapping] -->|-46%| X1[Fixed-Point<br/>15 bytes<br/>3D Mapping]",
    "F2[Float Format<br/>12 bytes<br/>Linear Accel] -->|-50%| X2[Fixed-Point<br/>6 bytes<br/>Linear Accel]",
    "F3[Float Format<br/>40 bytes/update<br/>@ 50Hz = 2000 B/s] -->|-48%| X3[Fixed-Point<br/>21 bytes/update<br/>@ 50Hz = 1050 B/s]"
  ),
  "LR",
  "default"
)

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
|---:|---:|---:|---:|
| Manufacturer Name | 0x2A29 | Read | String |
| Model Number | 0x2A24 | Read | String |
| Serial Number | 0x2A25 | Read | String |
| Hardware Revision | 0x2A27 | Read | String |
| Firmware Revision | 0x2A26 | Read | String |

### Battery Service
**UUID:** `0000180F-0000-1000-8000-00805F9B34FB`

| Characteristic | UUID | Properties | Data Type | Description |
|---:|---:|---:|---:|---:|
| Battery Level | 0x2A19 | Read, Notify | uint8_t | 0-100% |

### Current Time Service (CTS)
**UUID:** `00001805-0000-1000-8000-00805F9B34FB`

| Characteristic | UUID | Properties | Data Type |
|---:|---:|---:|---:|
| Current Time | 0x2A2B | Read, Write, Notify | CTS struct |

---

## 5. Information Service

**UUID:** `0c372eaa-27eb-437e-bef4-775aefaf3c97`  
**Availability:** Primary device only

### Characteristics

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|---:|---:|---:|---:|---:|
| Current Time | 0x2A2B | Read, Notify | CTS format | Device time |
| Status | `...eab` | Read, Notify | uint32_t | Status bitfield |
| Foot Sensor Samples | `...eaf` | Read, Notify | foot_samples_t | 16 ADC channels |
| Foot Log Available | `...eac` | Read, Notify | uint8_t | Latest log ID |
| Charge Status | `...ead` | Read, Notify | uint8_t | 0-100% |
| Foot Log Path | `...eae` | Read, Notify | char[] | UTF-8 path |
| BHI360 Log Available | `...eb0` | Read, Notify | uint8_t | Latest log ID |
| BHI360 Log Path | `...eb1` | Read, Notify | char[] | UTF-8 path |
| **BHI360 3D Mapping** | `...eb2` | Read, Notify | bhi360_3d_mapping_fixed_t | **Fixed-point** |
| **BHI360 Step Count** | `...eb3` | Read, Notify | step_count_only_t | **DEPRECATED** - Individual foot steps |
| **BHI360 Linear Accel** | `...eb4` | Read, Notify | bhi360_linear_accel_fixed_t | **Fixed-point** |
| **Total Step Count** | `...ec4` | Read, Notify | step_count_only_t | Aggregated both feet |
| **Activity Step Count** | `...ec5` | Read, Notify | step_count_only_t | Activity-specific steps |
| **Weight Measurement** | `...ec6` | Read, Notify | uint16_t | Weight in kg × 10 (0.1kg precision) |
| FOTA Progress | `...eb5` | Read, Notify | fota_progress_t | Update status |
| **Activity Log Available** | `...ec2` | Read, Notify | uint8_t | Latest log ID |
| **Activity Log Path** | `...ec3` | Read, Notify | char[] | UTF-8 path |

### Secondary Device Characteristics (Primary Device Only)

These characteristics are only available on the primary device and relay information from the connected secondary device:

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|---:|---:|---:|---:|---:|
| Secondary Manufacturer | `...eb6` | Read | String | Secondary device manufacturer |
| Secondary Model | `...eb7` | Read | String | Secondary device model |
| Secondary Serial | `...eb8` | Read | String | Secondary device serial |
| Secondary HW Rev | `...eb9` | Read | String | Secondary hardware revision |
| Secondary FW Rev | `...eba` | Read | String | Secondary firmware revision |
| Secondary FOTA Progress | `...ebb` | Read, Notify | fota_progress_t | Secondary update status |
| Secondary Foot Log Available | `...ebc` | Read, Notify | uint8_t | Secondary foot log ID |
| Secondary Foot Log Path | `...ebd` | Read, Notify | char[] | Secondary foot log path |
| Secondary BHI360 Log Available | `...ebe` | Read, Notify | uint8_t | Secondary BHI360 log ID |
| Secondary BHI360 Log Path | `...ebf` | Read, Notify | char[] | Secondary BHI360 log path |
| Secondary Activity Log Available | `...ec0` | Read, Notify | uint8_t | Secondary activity log ID |
| Secondary Activity Log Path | `...ec1` | Read, Notify | char[] | Secondary activity log path |

**Note:** Secondary device characteristics show "Not Connected" when the secondary device is not connected via D2D.

### Step Count Characteristics

The Information Service provides aggregated step count characteristics for mobile applications:

| Characteristic | Description | Behavior | Status |
|---:|---:|---:|---:|
| **BHI360 Step Count** | Individual foot global count | Always counting from boot | **DEPRECATED** - Do not use |
| **Total Step Count** | Sum of both feet global counts | Always active | **RECOMMENDED** |
| **Activity Step Count** | Steps during current activity only | 0 when no activity, resets on start | **RECOMMENDED** |

**Important Note**: As of version 2.4, mobile applications should only use the aggregated step count characteristics (Total Step Count and Activity Step Count). The individual foot step count characteristic (BHI360 Step Count) is deprecated and no longer sends notifications to conserve bandwidth and simplify mobile app implementation.

#### Step Count Behavior

1. **Global Counts** (BHI360 Step Count, Total Step Count):
   - Start at 0 on device boot
   - Continuously increment throughout device operation
   - Never reset except on device restart
   - Continue counting regardless of activity state

2. **Activity Step Count**:
   - Shows 0 when no activity is active
   - Resets to 0 when activity starts (via Control Service)
   - Counts only during active sessions
   - Freezes at final value when activity stops
   - Remains at last value until next activity

#### Data Format

All step count characteristics use the same 4-byte format:
```c
typedef struct {
    uint32_t step_count;  // Step count value
} __packed step_count_only_t;
```

**Note**: The `activity_duration_s` field in legacy structures is deprecated and always set to 0. Time-based metrics should be calculated by the mobile application using its own timers or GPS data.

### Weight Measurement

The weight measurement feature calculates a person's total weight using all 16 pressure sensors (8 per foot). The measurement requires the person to stand still on both feet for accurate results.

#### Usage Flow

DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Control Service",
    "participant Activity Metrics",
    "participant Info Service",
    "App->>Control Service: Write 0x01 to Weight Trigger",
    "Control Service->>Activity Metrics: MEASURE_WEIGHT command",
    "Activity Metrics->>Activity Metrics: Check motion (must be still)",
    "Activity Metrics->>Activity Metrics: Collect samples (3 seconds)",
    "Activity Metrics->>Activity Metrics: Calculate weight",
    "Activity Metrics->>Info Service: Weight result",
    "Info Service-->>App: Notify weight (uint16_t)"
  ),
  "default"
)

#### Weight Data Format

- **Type**: `uint16_t`
- **Unit**: kg × 10 (for 0.1kg precision)
- **Range**: 0-6553.5 kg
- **Example**: Value 752 = 75.2 kg

#### Requirements

1. Person must be standing still (motion < 0.5 m/s²)
2. Both feet must be on the sensors
3. Measurement takes approximately 3 seconds
4. System must be calibrated for accurate results

#### Error Conditions

- If person moves during measurement, no notification is sent
- If weight calculation fails, no notification is sent
- Mobile app should implement timeout (e.g., 10 seconds)

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
|---:|---:|---:|---:|---:|
| Set Time | `...b681` | Write | uint32_t | Epoch time (big-endian) |
| Delete Foot Log | `...b682` | Write, Notify | uint8_t | Log ID to delete |
| Delete BHI360 Log | `...b683` | Write, Notify | uint8_t | Log ID to delete |
| Delete Activity Log | `...b687` | Write, Notify | uint8_t | Log ID to delete |
| Start Activity | `...b684` | Write, Notify | uint8_t | Write 1 to start |
| Stop Activity | `...b685` | Write, Notify | uint8_t | Write 1 to stop |
| **Trigger BHI360 Calibration** | `...b686` | Write, Notify | uint8_t | Write 1 to trigger calibration |
| **Connection Parameter Control** | `...b68b` | Read, Write | uint8_t | Connection profile (0=Foreground, 1=Background, 2=Background Idle) |
| Delete Secondary Foot Log | `...b688` | Write, Notify | uint8_t | Log ID to delete on secondary |
| Delete Secondary BHI360 Log | `...b689` | Write, Notify | uint8_t | Log ID to delete on secondary |
| Delete Secondary Activity Log | `...b68a` | Write, Notify | uint8_t | Log ID to delete on secondary |
| **Weight Measurement Trigger** | `...b68c` | Write | uint8_t | Write 1 to trigger weight measurement |

### Command Flow

DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Primary",
    "participant Secondary",
    "App->>Primary: Write Command",
    "Primary->>Primary: Process locally",
    "Primary->>Secondary: Forward via D2D",
    "Secondary->>Secondary: Execute command",
    "Secondary-->>Primary: Acknowledge",
    "Primary-->>App: Notify status"
  ),
  "dark"
)

---

## 7. Proxy Services

### 7.1 FOTA Proxy Service

**UUID:** `6e400001-b5a3-f393-e0a9-e50e24dcca9e`  
**Availability:** Primary device only  
**Purpose:** Firmware updates for secondary device via primary

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|---:|---:|---:|---:|---:|
| Target Selection | `...0002` | Write | uint8_t | 0x00=Primary, 0x01=Secondary, 0xFF=All |
| Command | `...0003` | Write | uint8_t + data | See command table |
| Data | `...0004` | Write | byte[] | Firmware chunks |
| Status | `...0005` | Read, Notify | uint8_t | Operation status |

#### FOTA Commands

| Command | Value | Data | Description |
|---:|---:|---:|---:|
| Start | 0x01 | 4 bytes size | Begin update |
| Data | 0x02 | Firmware bytes | Send chunk |
| End | 0x03 | None | Complete update |
| Abort | 0x04 | None | Cancel update |
| Query | 0x05 | None | Get status |
| Reset | 0x06 | None | Reset device |

#### FOTA Flow

DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Primary",
    "participant Secondary",
    "App->>Primary: Set Target = Secondary",
    "App->>Primary: Start + Size",
    "Primary->>Secondary: Init FOTA",
    "loop Firmware Chunks",
    "App->>Primary: Data Chunk",
    "Primary->>Secondary: Forward Data",
    "Secondary-->>Primary: ACK",
    "Primary-->>App: Progress Update",
    "end",
    "App->>Primary: End Command",
    "Primary->>Secondary: Finalize",
    "Secondary->>Secondary: Reboot",
    "Primary-->>App: Complete"
  ),
  "dark"
)

### 7.2 3D Orientation Service

**UUID:** `0c372ec0-27eb-437e-bef4-775aefaf3c97`  
**Availability:** Primary device only  
**Purpose:** High-rate 3D orientation data for real-time visualization

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|---:|---:|---:|---:|---:|
| 3D Orientation | `...2ec1` | Read, Notify | orientation_3d_packet_t | Combined quaternions |

#### 3D Orientation Packet Structure

```c
// 3D Orientation packet structure (20 bytes)
typedef struct __attribute__((packed)) {
    uint16_t delta_time_ms;         // 0-1: Time since last packet
    
    // Left shoe quaternion (scaled to int16)
    int16_t left_quat_w;            // 2-3: w × 16384
    int16_t left_quat_x;            // 4-5: x × 16384
    int16_t left_quat_y;            // 6-7: y × 16384
    int16_t left_quat_z;            // 8-9: z × 16384
    
    // Right shoe quaternion (scaled to int16)
    int16_t right_quat_w;           // 10-11: w × 16384
    int16_t right_quat_x;           // 12-13: x × 16384
    int16_t right_quat_y;           // 14-15: y × 16384
    int16_t right_quat_z;           // 16-17: z × 16384
    
    // Status flags
    uint8_t left_contact;           // 18: 0=air, 1=ground
    uint8_t right_contact;          // 19: 0=air, 1=ground
} orientation_3d_packet_t;
```

**Update Rate**: 20Hz (50ms intervals)  
**Availability**: Only when logging is NOT active  
**Purpose**: Real-time 3D visualization of both shoes

#### Integration Example

```swift
// iOS Swift
func handle3DOrientation(_ data: Data) {
    let buffer = data.withUnsafeBytes { $0.bindMemory(to: Int16.self) }
    
    // Skip delta time at index 0
    let leftQuat = simd_quatf(
        ix: Float(buffer[2]) / 16384.0,
        iy: Float(buffer[3]) / 16384.0,
        iz: Float(buffer[4]) / 16384.0,
        r: Float(buffer[1]) / 16384.0
    ).normalized
    
    let rightQuat = simd_quatf(
        ix: Float(buffer[6]) / 16384.0,
        iy: Float(buffer[7]) / 16384.0,
        iz: Float(buffer[8]) / 16384.0,
        r: Float(buffer[5]) / 16384.0
    ).normalized
    
    // Update 3D models
    updateShoeModel(left: leftQuat, right: rightQuat)
}
```

### 7.3 SMP Proxy Service

**Service UUID:** `8D53DC1E-1DB7-4CD3-868B-8A527460AA84`  
**Availability:** Primary device only  
**Purpose:** Unified MCUmgr/SMP access to both primary and secondary devices

This service allows mobile applications to use standard MCUmgr libraries to communicate with both devices through a single interface.

#### Characteristics

| Characteristic | UUID | Properties | Description |
|---:|---:|---:|---:|
| Target Select | `DA2E7829-FBCE-4E01-AE9E-261174997C48` | Read, Write | Select target device |
| SMP Data | `DA2E7828-FBCE-4E01-AE9E-261174997C48` | Write, Write Without Response, Notify | Standard SMP protocol |

#### Target Values
- `0x00`: Primary device (default)
- `0x01`: Secondary device

#### Benefits
- No custom protocols needed
- Same code for FOTA, file access, and all MCUmgr operations
- Transparent forwarding to secondary device

#### Usage Example

```swift
// Set target to secondary
writeCharacteristic(targetUUID, data: Data([0x01]))

// Use standard MCUmgr with proxy characteristic
let transport = McuMgrBleTransport(peripheral)
transport.smpCharacteristic = smpDataCharacteristic

// All MCUmgr operations now work with secondary!
let dfuManager = FirmwareUpgradeManager(transporter: transport)
dfuManager.start(data: firmware)
```

### 7.4 File Proxy Service

**UUID:** `7e500001-b5a3-f393-e0a9-e50e24dcca9e`  
**Availability:** Primary device only  
**Purpose:** Access log files on secondary device

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|---:|---:|---:|---:|---:|
| Target Device | `...0002` | Write | uint8_t | 0x00=Primary, 0x01=Secondary |
| File Command | `...0003` | Write | Command struct | See below |
| File Data | `...0004` | Notify | byte[] | File chunks |
| File Status | `...0005` | Read, Notify | uint8_t | Operation status |

#### File Commands

```c
typedef struct {
    uint8_t cmd;      // Command type
    uint8_t file_id;  // File ID
    uint8_t type;     // 0x01=Foot, 0x02=BHI360, 0x03=Activity
} file_command_t;
```

| Command | Value | Description |
|---:|---:|---:|
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
|---:|---:|---:|---:|---:|
| D2D Set Time | `...ca1f` | Write | uint32_t | Time relay |
| D2D Delete Foot Log | `...ca82` | Write | uint8_t | Delete command |
| D2D Delete BHI360 Log | `...ca83` | Write | uint8_t | Delete command |
| D2D Delete Activity Log | `...ca87` | Write | uint8_t | Delete command |
| D2D Start Activity | `...ca84` | Write | uint8_t | Start command |
| D2D Stop Activity | `...ca85` | Write | uint8_t | Stop command |
| **D2D Trigger BHI360 Calibration** | `...ca86` | Write | uint8_t | Calibration trigger |

### 8.2 D2D TX Service (Secondary Device)

**UUID:** `75ad68d6-200c-437d-98b5-061862076c5f`  
**Purpose:** Transmit sensor data from secondary to primary

| Characteristic | UUID Suffix | Properties | Data Type | Description |
|---:|---:|---:|---:|---:|
| D2D Status | `...68d6` | Notify | uint32_t | Status bitfield |
| D2D Foot Log Available | `...68d7` | Notify | uint8_t | Log ID |
| D2D Charge Status | `...68d8` | Notify | uint8_t | Battery % |
| D2D Foot Log Path | `...68d9` | Notify | char[] | File path |
| D2D BHI360 Log Available | `...68da` | Notify | uint8_t | Log ID |
| D2D BHI360 Log Path | `...68db` | Notify | char[] | File path |
| D2D Foot Samples | `...68dc` | Notify | foot_samples_t | ADC data |
| **D2D BHI360 3D Mapping** | `...68dd` | Notify | bhi360_3d_mapping_fixed_t | **Fixed-point** |
| **D2D BHI360 Step Count** | `...68de` | Notify | bhi360_step_count_t | Steps (for aggregation) |
| **D2D Activity Step Count** | `...68e6` | Notify | bhi360_step_count_t | Activity-specific steps |
| **D2D BHI360 Linear Accel** | `...68df` | Notify | bhi360_linear_accel_fixed_t | **Fixed-point** |
| D2D Current Time | `...68e0` | Notify | CTS struct | Time sync |
| D2D Device Info | `...68e1` | Notify | device_info_msg_t | Device information |
| D2D FOTA Progress | `...68e3` | Notify | fota_progress_t | FOTA update status |
| D2D Activity Log Available | `...68e4` | Notify | uint8_t | Log ID |
| D2D Activity Log Path | `...68e5` | Notify | char[] | File path |
| **D2D Weight Measurement** | `...68e7` | Notify | uint16_t | Weight in kg × 10 |

### 8.3 D2D File Transfer Service

**UUID:** `8e600001-b5a3-f393-e0a9-e50e24dcca9e`  
**Purpose:** File transfer between devices

| Characteristic | UUID Suffix | Properties | Data Type |
|---:|---:|---:|---:|
| Command | `...0002` | Write | Command packet |
| Data | `...0003` | Notify | Data packet |
| Status | `...0004` | Notify | Status byte |

### D2D Architecture

DrawFlowchart(
  Syntax(
    // Data Flow
    "SEC[Secondary Sensors] --> SECTX[D2D TX Service]",
    "SECTX --> PRIMRX[Primary D2D Client]",
    "PRIMRX --> PRIMINFO[Information Service]",
    "PRIMINFO --> PHONE[Mobile App]",
    // Command Flow
    "PHONE2[Mobile App] --> PRIMCTRL[Control Service]",
    "PRIMCTRL --> PRIMD2D[D2D TX Client]",
    "PRIMD2D --> SECRX[D2D RX Service]",
    "SECRX --> SECDEV[Secondary Device]"
  ),
  "TB",
  "default"
)

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

// Step Count Only - 4 bytes (no duration)
typedef struct {
    uint32_t step_count;
} __packed step_count_only_t;

// Legacy Step Count - 8 bytes (for backward compatibility)
typedef struct {
    uint32_t step_count;
    uint32_t activity_duration_s;  // DEPRECATED - always 0
} __packed bhi360_step_count_t;

// Foot Sensor Samples - 16 bytes (file logging)
typedef struct {
    uint16_t values[8];  // 8 ADC channels
} __packed foot_samples_t;
```

### BLE Data Structures with Sequence Numbers

For high-rate data streams, BLE transmission includes sequence numbers for packet loss detection:

```c
// Foot sensor data with sequence - 17 bytes (BLE only)
typedef struct {
    uint8_t seq_num;                           // Rolling sequence (0-255)
    uint16_t values[NUM_FOOT_SENSOR_CHANNELS]; // 8 channels
} __packed foot_samples_ble_t;

// BHI360 3D mapping with sequence - 16 bytes (BLE only)
typedef struct {
    uint8_t seq_num;       // Rolling sequence (0-255)
    int16_t quat_x;        // Quaternion X × 10000
    int16_t quat_y;        // Quaternion Y × 10000
    int16_t quat_z;        // Quaternion Z × 10000
    int16_t quat_w;        // Quaternion W × 10000
    int16_t gyro_x;        // Gyroscope X × 10000
    int16_t gyro_y;        // Gyroscope Y × 10000
    int16_t gyro_z;        // Gyroscope Z × 10000
    uint8_t quat_accuracy; // Accuracy × 100
} __packed bhi360_3d_mapping_ble_t;

// BHI360 linear acceleration with sequence - 7 bytes (BLE only)
typedef struct {
    uint8_t seq_num;  // Rolling sequence (0-255)
    int16_t x;        // Acceleration X × 1000
    int16_t y;        // Acceleration Y × 1000
    int16_t z;        // Acceleration Z × 1000
} __packed bhi360_linear_accel_ble_t;

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

## 10. Packet Sequencing and Recovery

### Overview

High-rate sensor data includes sequence numbers for packet loss detection and limited recovery. This mechanism is used ONLY for BLE transmission, not for file logging.

### Affected Data Types

| Data Type | Update Rate | Sequence Number | Recovery Buffer |
|---:|---:|---:|---:|
| Foot Sensor Samples | 20Hz | Yes (8-bit) | 10 packets (~500ms) |
| BHI360 3D Mapping | 50Hz | Yes (8-bit) | 10 packets (~200ms) |
| BHI360 Linear Accel | 50Hz | Yes (8-bit) | 10 packets (~200ms) |
| BHI360 Step Count | 5Hz | No | N/A |
| Other characteristics | Variable | No | N/A |

### Sequence Number Rules

1. **Range**: 0-255 (8-bit rolling counter)
2. **Initialization**: Starts at 0 on device boot
3. **Increment**: +1 for each packet sent
4. **Rollover**: After 255, next is 0
5. **Independent**: Each data type has its own sequence counter

### Packet Loss Detection

```swift
// iOS Example
func detectPacketLoss(lastSeq: UInt8, currentSeq: UInt8) -> UInt8 {
    if currentSeq >= lastSeq {
        return currentSeq - lastSeq - 1
    } else {
        // Handle rollover
        return (255 - lastSeq) + currentSeq
    }
}
```

### Recovery Mechanism

When packet loss is detected, the mobile app can request retransmission of recent packets:

DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Device",
    "App->>Device: Data packet (seq=10)",
    "Note over App: Missing seq 11-14",
    "App->>Device: Data packet (seq=15)",
    "App->>App: Detect gap",
    "App->>Device: Recovery Request<br/>(start=11, end=14)",
    "Device->>App: Recovery Response<br/>(4 packets)",
    "Device->>App: Retransmit seq 11-14"
  ),
  "default"
)

### Recovery Limitations

- **Buffer Size**: Only last 10 packets are kept
- **Time Window**: ~500ms for 20Hz data, ~200ms for 50Hz data
- **Best Effort**: Not all requested packets may be available
- **Rate Limited**: Max 1 recovery request per second

### Implementation Notes

1. **First Packet**: Apps should ignore sequence check on first packet after connection
2. **Large Gaps**: If gap > 10 packets, accept the loss and continue
3. **Statistics**: Track packet loss rate for connection quality monitoring
4. **Graceful Degradation**: System continues to function with packet loss

For detailed implementation, see [BLE Packet Sequencing and Recovery](BLE_Packet_Sequencing_Recovery_Coda.md).

---

## 11. Integration Examples

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

## 12. Error Handling and Status Communication

### Status Characteristic Details

The Status characteristic in the Information Service provides real-time device health monitoring through a 32-bit bitmask.

#### Status Bit Definitions

```c
#define STATUS_OK                     0x00000000  // No errors
#define STATUS_BATTERY_FAULT          (1 << 0)   // Battery fault detected
#define STATUS_BLUETOOTH_ERROR        (1 << 4)   // Bluetooth communication error
#define STATUS_HARDWARE_ERROR         (1 << 14)  // General hardware error
#define STATUS_DATA_ERROR             (1 << 15)  // Data module error
#define STATUS_DFU_ERROR              (1 << 16)  // DFU/firmware update error
#define STATUS_ADC_ERROR              (1 << 18)  // ADC/foot sensor error
#define STATUS_I2C_ERROR              (1 << 19)  // I2C communication error
#define STATUS_BATTERY_DISCONNECTED   (1 << 20)  // Battery disconnected
#define STATUS_MOTION_ERROR           (1 << 21)  // Motion sensor error
#define STATUS_RTC_ERROR              (1 << 22)  // Real-time clock error
#define STATUS_FILE_SYSTEM_ERROR      (1 << 23)  // File system error
#define STATUS_PROTO_ENCODE_ERROR     (1 << 24)  // Protocol buffer encoding error
#define STATUS_FILE_SYSTEM_NO_FILES   (1 << 25)  // No files in file system
#define STATUS_FILE_SYSTEM_FULL       (1 << 26)  // File system storage full
#define STATUS_FLASH_FAILURE          (1 << 27)  // Flash memory failure
#define STATUS_INVALID_PARAMETER      (1 << 28)  // Invalid parameter error
#define STATUS_QUEUE_FULL             (1 << 29)  // Message queue full
```

#### Status Examples

```
0x00000000 - System operating normally
0x00040000 - ADC error (foot sensor failure)
0x00200000 - Motion sensor error
0x00240000 - Both ADC and motion sensor errors
0x00800000 - File system error
0x04000000 - File system full
```

### Sensor Criticality Configuration

The firmware supports configurable sensor criticality, determining whether the system continues operating when a sensor fails.

#### Build-Time Configuration

```kconfig
choice PRIMARY_SENSING_FUNCTION
    prompt "Primary sensing function"
    default PRIMARY_FUNCTION_BOTH
    
    config PRIMARY_FUNCTION_FOOT
        bool "Foot pressure sensing is primary"
        help
          Motion sensor failures will be reported but system continues.
        
    config PRIMARY_FUNCTION_MOTION
        bool "Motion/IMU sensing is primary"
        help
          Foot sensor failures will be reported but system continues.
        
    config PRIMARY_FUNCTION_BOTH
        bool "Both sensors are required"
        help
          Either sensor failure will prevent system startup.
endchoice
```

#### Behavior by Configuration

| Configuration | Foot Sensor Fails | Motion Sensor Fails | Both Fail |
|---:|---:|---:|---:|
| **BOTH** (default) | System halts | System halts | System halts |
| **FOOT** primary | System continues* | System halts | System halts |
| **MOTION** primary | System halts | System continues* | System halts |

*Error is reported via BLE status characteristic but system operates in degraded mode

### Mobile App Status Handling

#### iOS Swift Example

```swift
func parseDeviceStatus(_ statusValue: UInt32) -> [String] {
    var errors: [String] = []
    
    if statusValue == 0 {
        return ["System OK"]
    }
    
    if statusValue & (1 << 18) != 0 {
        errors.append("Foot sensor error")
    }
    if statusValue & (1 << 21) != 0 {
        errors.append("Motion sensor error")
    }
    if statusValue & (1 << 23) != 0 {
        errors.append("File system error")
    }
    if statusValue & (1 << 26) != 0 {
        errors.append("Storage full")
    }
    
    return errors
}
```

#### Android Kotlin Example

```kotlin
fun parseDeviceStatus(statusValue: Int): List<String> {
    val errors = mutableListOf<String>()
    
    if (statusValue == 0) {
        return listOf("System OK")
    }
    
    if (statusValue and (1 shl 18) != 0) {
        errors.add("Foot sensor error")
    }
    if (statusValue and (1 shl 21) != 0) {
        errors.add("Motion sensor error")
    }
    if (statusValue and (1 shl 23) != 0) {
        errors.add("File system error")
    }
    if (statusValue and (1 shl 26) != 0) {
        errors.add("Storage full")
    }
    
    return errors
}
```

### Error Priority Guidelines

1. **Critical** (System cannot function):
   - Bluetooth errors
   - All sensors failed (if both required)

2. **High** (Degraded operation):
   - Single sensor failure
   - Storage full

3. **Medium** (Feature unavailable):
   - RTC error
   - Battery monitoring failure

4. **Low** (Informational):
   - Temporary file system errors
   - Queue full (usually transient)

## 13. Common BLE Error Codes

| Error | Code | Description | Solution |
|---:|---:|---:|---:|
| ENOTCONN | -128 | Not connected | Ensure connection established |
| ENOMEM | -12 | Out of memory | Reduce notification rate |
| EINVAL | -22 | Invalid parameter | Check data format |
| EACCES | -13 | Access denied | Ensure proper bonding |
| ETIMEDOUT | -116 | Operation timeout | Check connection stability |

### Troubleshooting Guide

DrawFlowchart(
  Syntax(
    "A[Connection Issues?] -->|Yes| B[Check Bonding]",
    "A -->|No| C[Data Issues?]",
    "B --> D[Clear bonds and re-pair]",
    "C -->|Yes| E[Check Format]",
    "C -->|No| F[Performance Issues?]",
    "E --> G[Verify fixed-point scaling]",
    "F -->|Yes| H[Optimize Parameters]",
    "F -->|No| I[Check Logs]",
    "H --> J[Reduce notification rate<br/>Increase connection interval]"
  ),
  "TD",
  "default"
)

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

## Appendix A: 3D Orientation Service Data Rate Analysis

### Data Rate Pipeline

#### Physical Sensors (Inside BHI360)
- **Accelerometer**: 400-1600Hz (configurable)
- **Gyroscope**: 400-1600Hz (configurable)
- **Magnetometer**: 100Hz typical

#### BHI360 Processing
1. **Raw sensor data**: Sampled at high rate (400Hz+)
2. **Sensor fusion algorithm**: Runs at high rate internally
3. **Virtual sensor output**: 
   - Currently configured: 50Hz
   - Maximum supported: 400Hz
   - Recommended for 3D viz: 100Hz

#### Data Flow Rates

```
Physical Sensors (400Hz+)
    ↓
BHI360 Fusion Algorithm (Internal high rate)
    ↓
Virtual Quaternion Output (Configurable: 50-400Hz)
    ↓
Motion Sensor Callback (Same as virtual sensor rate)
    ↓
For Primary Device:
    - Direct to BLE Service (No delay)
    
For Secondary Device:
    - D2D Transmission (Rate limited to 20-50Hz)
    ���
BLE Transmission (Rate limited: 20-50Hz)
    ↓
Mobile App (Receives at BLE rate)
```

### Rate Bottlenecks

1. **BHI360 Configuration**: Currently set to 50Hz (can be increased)
2. **BLE Connection Interval**: Typically 15-30ms (limits to ~33-66Hz max)
3. **BLE Bandwidth**: 20-byte packets at high rate can congest
4. **D2D Communication**: Secondary to primary adds latency

### Optimization Options

| Configuration | BHI360 Rate | BLE Rate | Power Impact |
|---:|---:|---:|---:|
| Standard | 50Hz | 20Hz | Baseline |
| High Rate | 100Hz | 50Hz | +15-20% |
| Maximum | 200Hz | 50Hz | +30-40% |

### Recommendations for 3D Orientation Service

1. **For 3D Visualization**: Use 100Hz BHI360 with 50Hz BLE
2. **For Battery Life**: Stay with 50Hz/20Hz configuration
3. **For Development**: Make rates configurable via Kconfig

The BHI360 performs sensor fusion at high internal rates regardless of output rate, ensuring smooth quaternion data even at lower transmission rates.

---

**End of Specification**
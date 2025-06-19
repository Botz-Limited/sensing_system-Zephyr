# Bluetooth GATT Specification

**Scope:** This document details the Bluetooth GATT services, characteristics, and protocols implemented on the device for mobile app and integration development.

**Purpose:** To provide a comprehensive reference for developers integrating with the device over BLE, including UUIDs, data types, permissions, and usage notes.

**Revision:** 1.0

**Author:** Firmware Team

**Last Updated:** December 2024

---

This document describes the Bluetooth GATT services and characteristics implemented on the device. It is intended for mobile app and integration developers. Each service is presented in a separate chapter, with all UUIDs, data types, permissions, and a short explanation for each characteristic. The SMP (Simple Management Protocol) server for firmware updates is also described.

---

## Table of Contents
- [Bluetooth GATT Specification](#bluetooth-gatt-specification)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [1. Information Service](#1-information-service)
  - [2. Control Service](#2-control-service)
  - [3. Device Information Service](#3-device-information-service)
  - [4. Battery Service](#4-battery-service)
  - [5. Current Time Service (CTS)](#5-current-time-service-cts)
  - [6. SMP Server](#6-smp-server)
  - [7. FOTA Proxy Service](#7-fota-proxy-service)
  - [8. File Proxy Service](#8-file-proxy-service)
  - [9. Device-to-Device (D2D) Services](#9-device-to-device-d2d-services)
    - [9.1 D2D RX Service (Primary Device Only)](#91-d2d-rx-service-primary-device-only)
    - [9.2 D2D TX Service (Secondary Device Only)](#92-d2d-tx-service-secondary-device-only)
    - [9.3 D2D File Transfer Service](#93-d2d-file-transfer-service)

---

## Introduction

This device exposes several Bluetooth Low Energy (BLE) GATT services for control, data logging, and device management. All services use encrypted communication. Characteristics are designed for real-time sensor data, device control, log management, and status/error reporting. The SMP server is used for firmware updates and device management.

---

## 1. Information Service
**Service UUID:** `**0c372eaa**-27eb-437e-bef4-775aefaf3c97`

| Characteristic Name                | UUID                                    | Properties         | Permissions         | Data Type                | Format/Endian | Description |
|------------------------------------|-----------------------------------------|--------------------|---------------------|--------------------------|---------------|-------------|
| Current Time                      | 0x2A2B (Standard CTS)                   | Read, Notify       | Read (encrypted)    | struct                   | Little-endian | Current time, updated from CTS. |
| Status                            | `**0c372eab**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | uint32_t (bitfield)      | Little-endian | Device status/error bitfield. |
| Foot Sensor Samples               | `**0c372eaf**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | foot_samples_t (struct)  | Little-endian | Foot sensor data samples. |
| Foot Sensor Log Available         | `**0c372eac**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | uint8_t                  | N/A           | ID of the latest available foot sensor log (after deletion, the newest closed log is sent; open files are not sent). |
| Charge Status                     | `**0c372ead**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | uint8_t                  | N/A           | Battery/charge status. |
| Foot Sensor Req ID/Path           | `**0c372eae**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | char[]                   | UTF-8         | Path/ID for the latest available foot sensor log file (never an open file). |
| BHI360 Log Available              | `**0c372eb0**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | uint8_t                  | N/A           | ID of the latest available BHI360 log (after deletion, the newest closed log is sent; open files are not sent). |
| BHI360 Req ID/Path                | `**0c372eb1**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | char[]                   | UTF-8         | Path/ID for the latest available BHI360 log file (never an open file). |
| BHI360 3D Mapping                 | `**0c372eb2**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | bhi360_3d_mapping_t      | Little-endian | BHI360 quaternion/gyro/accel data. |
| BHI360 Step Count                 | `**0c372eb3**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | bhi360_step_count_t      | Little-endian | BHI360 step count and duration. |
| BHI360 Linear Accel               | `**0c372eb4**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | bhi360_linear_accel_t    | Little-endian | BHI360 linear acceleration. |
| FOTA Progress                     | `**0c372eb5**-27eb-437e-bef4-775aefaf3c97`    | Read, Notify       | Read (encrypted)    | fota_progress_msg_t      | Little-endian | Real-time FOTA update progress. |

**Status Characteristic Bitfield:**

The Status characteristic is a 32-bit bitfield (uint32_t) that encodes the current device status and error conditions. Each bit represents a specific status or error flag. Multiple bits may be set simultaneously to indicate combined states. The bit definitions are as follows:

- **Bit 0 (0x00000001): STATUS_ERROR** – Indicates a device error condition.
- **Bit 1 (0x00000002): STATUS_CALIBRATING** – Device is currently calibrating.
- **Bit 2 (0x00000004): STATUS_READY** – Device is ready for operation.
- **Bit 3 (0x00000008): STATUS_IDLE** – Device is idle.

Additional bits may be defined in future firmware versions.

**Bitfield Usage and Examples:**
- The bitfield can be interpreted by checking which bits are set. For example:
    - `0x00000001` (only Bit 0 set): Device is in an error state.
    - `0x00000006` (Bits 1 and 2 set): Device is calibrating and ready.
    - `0x0000000C` (Bits 2 and 3 set): Device is ready and idle.
    - `0x00000000`: No status or error flags are set (normal state).
- Multiple bits may be set at once to indicate combined statuses (e.g., calibrating and error).
- Integrators should mask and interpret each bit individually to determine the current device state and error conditions.

Refer to the firmware header `status_codes.h` for the latest bit definitions.

**FOTA Progress Data Structure:**
```c
struct fota_progress_msg_t {
    uint8_t is_active;        // 0=inactive, 1=active
    uint8_t status;           // 0=idle, 1=in_progress, 2=pending, 3=confirmed, 4=error
    uint8_t percent_complete; // 0-100
    uint32_t bytes_received;  // Bytes received so far
    uint32_t total_size;      // Total firmware size
    int32_t error_code;       // Error code if status=4
};
```

**Notes:**
- All characteristics use encrypted read and notify permissions.
- After a log file is deleted, the device will update the "Log Available" and "Req ID/Path" characteristics to point to the newest (latest) closed log file. Open files (currently being written to) are never sent.
- Data types such as `foot_samples_t` and `bhi360_3d_mapping_t` are C structs (see firmware for details).
- Notification is used for real-time updates (e.g., status, sensor data, log availability).
- FOTA Progress provides real-time updates during firmware updates via SMP.

---

## 2. Control Service
**Service UUID:** `**4fd5b67f**-9d89-4061-92aa-319ca786baae`

| Characteristic Name         | UUID                                    | Properties         | Permissions                  | Data Type   | Format/Endian | Description |
|----------------------------|-----------------------------------------|--------------------|------------------------------|-------------|---------------|-------------|
| Set Time Command           | `**4fd5b681**-9d89-4061-92aa-319ca786baae`    | Read, Write        | Write (encrypted)            | uint32_t    | Big-endian    | Set device time (32-bit big-endian epoch time). |
| Delete Foot Log Command    | `**4fd5b682**-9d89-4061-92aa-319ca786baae`    | Read, Write, Notify| Read/Write (encrypted)       | uint8_t     | N/A           | Delete foot sensor log by ID. Notifies status. |
| Delete BHI360 Log Command  | `**4fd5b683**-9d89-4061-92aa-319ca786baae`    | Read, Write, Notify| Read/Write (encrypted)       | uint8_t     | N/A           | Delete BHI360 log by ID. Notifies status. |
| Start Activity Command     | `**4fd5b684**-9d89-4061-92aa-319ca786baae`    | Write, Notify      | Write (encrypted)            | uint8_t     | N/A           | Start activity (1 = start). Notifies status. |
| Stop Activity Command      | `**4fd5b685**-9d89-4061-92aa-319ca786baae`    | Write, Notify      | Write (encrypted)            | uint8_t     | N/A           | Stop activity (1 = stop). Notifies status. |

**Notes:**
- Write operations are used to send commands (e.g., set time, delete log, start/stop activity).
- Notify is used for status feedback after command execution.
- Set Time expects a 32-bit big-endian epoch time.

---

## 3. Device Information Service
**Service UUID:** Standard DIS UUID (0x180A)

| Characteristic Name      | UUID (Standard)         | Properties | Permissions      | Data Type | Description |
|-------------------------|-------------------------|------------|------------------|-----------|-------------|
| Manufacturer Name       | 0x2A29                  | Read       | Read (encrypted) | string    | Manufacturer name. |
| Model Number            | 0x2A24                  | Read       | Read (encrypted) | string    | Model number. |
| Hardware Revision       | 0x2A27                  | Read       | Read (encrypted) | string    | Hardware revision. |
| Firmware Revision       | 0x2A26                  | Read       | Read (encrypted) | string    | Firmware revision. |
| Serial Number           | 0x2A25                  | Read       | Read (encrypted) | string    | Serial number. |

**Notes:**
- All characteristics are read-only and use encrypted read permissions.
- Data is provided as null-terminated strings.

---

## 4. Battery Service

**Service UUID:** 0x180F (Standard)

| Characteristic Name | UUID (Standard) | Properties | Permissions      | Data Type | Description |
|---------------------|-----------------|------------|------------------|-----------|-------------|
| Battery Level       | 0x2A19          | Read, Notify| Read (encrypted) | uint8_t   | Battery level as a percentage (0–100). |

**Notes:**
- The Battery Level characteristic provides the current battery percentage.
- The value is a single unsigned byte (0–100).
- The characteristic supports notifications for real-time battery updates.

---

## 5. Current Time Service (CTS)
**Service UUID:** Standard CTS UUID (0x1805)

| Characteristic Name | UUID (Standard) | Properties | Permissions      | Data Type | Format/Endian | Description |
|--------------------|-----------------|------------|------------------|-----------|---------------|-------------|
| Current Time       | 0x2A2B          | Read, Notify| Read (encrypted) | struct    | Little-endian | Current time in standard CTS format. |

**Notes:**
- Used for time synchronization with the device.
- The device may update its internal clock from this characteristic.

---

## 6. SMP Server

The SMP (Simple Management Protocol) server is used for device firmware updates and management. It is implemented as a separate service using standard Zephyr/MCUboot characteristics and UUIDs.

- **Service UUID:** `**8D53DC1D**-1DB7-4CD3-868B-8A527460AA84` (Zephyr SMP default)
- **Characteristic UUID:** `**DA2E7828**-FBCE-4E01-AE9E-261174997C48`
- **Properties:** Write, Notify
- **Permissions:** Encrypted
- **Data Type:** SMP protocol binary packets
- **Description:** Used by the mobile app to perform firmware updates, retrieve device info, and perform management operations. See Zephyr SMP documentation for protocol details.

---

## 7. FOTA Proxy Service

**Service UUID:** `**6e400001**-b5a3-f393-e0a9-e50e24dcca9e`

**Purpose:** This service is only available on primary devices and allows the mobile app to perform firmware updates on secondary devices that are not directly connected to the phone. The primary device acts as a proxy, forwarding FOTA commands and data to the secondary device.

| Characteristic Name | UUID                                    | Properties         | Permissions         | Data Type   | Format/Endian | Description |
|--------------------|-----------------------------------------|--------------------|---------------------|-------------|---------------|-------------|
| Target Selection   | `**6e400002**-b5a3-f393-e0a9-e50e24dcca9e`    | Write              | Write (encrypted)   | uint8_t     | N/A           | Select FOTA target: 0x00=Primary, 0x01=Secondary, 0xFF=All devices. |
| Command            | `**6e400003**-b5a3-f393-e0a9-e50e24dcca9e`    | Write, Write No Resp| Write (encrypted)   | uint8_t + data | Little-endian | FOTA commands (see below). |
| Data               | `**6e400004**-b5a3-f393-e0a9-e50e24dcca9e`    | Write, Write No Resp| Write (encrypted)   | byte[]      | N/A           | Bulk firmware data transfer. |
| Status             | `**6e400005**-b5a3-f393-e0a9-e50e24dcca9e`    | Read, Notify       | Read (encrypted)    | uint8_t     | N/A           | FOTA proxy status (see below). |

**Command Values:**
- `0x01`: Start FOTA (followed by 4 bytes total size in little-endian)
- `0x02`: Data chunk (followed by firmware data bytes)
- `0x03`: End FOTA
- `0x04`: Abort FOTA
- `0x05`: Query status
- `0x06`: Reset target device

**Status Values:**
- `0x00`: Idle
- `0x01`: In progress
- `0x02`: Success
- `0x03`: Error
- `0x04`: No target device connected

**Usage Flow:**
1. Set target device (write to Target Selection)
2. Start FOTA with size (write 0x01 + size to Command)
3. Send firmware chunks (write 0x02 + data to Command)
4. End FOTA (write 0x03 to Command)
5. Monitor status via notifications
6. Reset device if needed (write 0x06 to Command)

**Notes:**
- This service is only present on primary devices (CONFIG_PRIMARY_DEVICE=y)
- The proxy has a 5-minute timeout for FOTA operations
- Chunk size should be smaller than negotiated MTU (typically 240 bytes)

---

## 8. File Proxy Service

**Service UUID:** `**7e500001**-b5a3-f393-e0a9-e50e24dcca9e`

**Purpose:** This service is only available on primary devices and allows the mobile app to access log files stored on secondary devices. The primary device acts as a proxy, forwarding file commands to the secondary device and returning file data.

| Characteristic Name | UUID                                    | Properties         | Permissions         | Data Type   | Format/Endian | Description |
|--------------------|-----------------------------------------|--------------------|---------------------|-------------|---------------|-------------|
| Target Device      | `**7e500002**-b5a3-f393-e0a9-e50e24dcca9e`    | Write              | Write (encrypted)   | uint8_t     | N/A           | Select target: 0x00=Primary, 0x01=Secondary. |
| File Command       | `**7e500003**-b5a3-f393-e0a9-e50e24dcca9e`    | Write              | Write (encrypted)   | uint8_t + data | N/A           | File operation commands (see below). |
| File Data          | `**7e500004**-b5a3-f393-e0a9-e50e24dcca9e`    | Notify             | Read (encrypted)    | byte[]      | N/A           | File data chunks from device. |
| File Status        | `**7e500005**-b5a3-f393-e0a9-e50e24dcca9e`    | Read, Notify       | Read (encrypted)    | uint8_t     | N/A           | Operation status (see below). |

**Command Format:**
- Byte 0: Command type
- Byte 1: File ID (if applicable)
- Byte 2: File type (if applicable)

**Command Types:**
- `0x01`: List log files
- `0x02`: Read file by ID
- `0x03`: Delete file by ID
- `0x04`: Get file info
- `0x05`: Abort operation

**File Types:**
- `0x01`: Foot sensor logs
- `0x02`: BHI360 logs
- `0xFF`: All types

**Status Values:**
- `0x00`: Idle
- `0x01`: Busy
- `0x02`: Success
- `0x03`: Error
- `0x04`: No target device
- `0x05`: File not found
- `0x06`: Transfer in progress

**Usage Example (List Files):**
1. Set target to secondary (write 0x01 to Target Device)
2. Send list command (write 0x01 to File Command)
3. Receive file list via File Data notifications
4. Check status for completion

**Notes:**
- This service is only present on primary devices
- File data is sent in chunks via notifications
- 30-second timeout for file operations

---

## 9. Device-to-Device (D2D) Services

The D2D services enable communication between primary and secondary devices. These services are conditionally compiled based on device type.

### 9.1 D2D RX Service (Primary Device Only)

**Service UUID:** `**e060ca1f**-3115-4ad6-9709-8c5ff3bf558b`

**Purpose:** This service runs on the primary device to receive commands and data from the secondary device. It mirrors the Control Service functionality for D2D communication.

| Characteristic Name         | UUID                                    | Properties | Permissions      | Data Type   | Format/Endian | Description |
|----------------------------|-----------------------------------------|------------|------------------|-------------|---------------|-------------|
| D2D Set Time Command       | `**e160ca1f**-3115-4ad6-9709-8c5ff3bf558b`    | Write      | Write            | uint32_t    | Big-endian    | Relay time setting to secondary. |
| D2D Delete Foot Log        | `**e160ca82**-3115-4ad6-9709-8c5ff3bf558b`    | Write      | Write            | uint8_t     | N/A           | Relay delete foot log command. |
| D2D Delete BHI360 Log      | `**e160ca83**-3115-4ad6-9709-8c5ff3bf558b`    | Write      | Write            | uint8_t     | N/A           | Relay delete BHI360 log command. |
| D2D Start Activity         | `**e160ca84**-3115-4ad6-9709-8c5ff3bf558b`    | Write      | Write            | uint8_t     | N/A           | Relay start activity command. |
| D2D Stop Activity          | `**e160ca85**-3115-4ad6-9709-8c5ff3bf558b`    | Write      | Write            | uint8_t     | N/A           | Relay stop activity command. |

**Notes:**
- This service only exists on primary devices
- Used by the primary to relay control commands from the phone to the secondary device
- The primary device acts as a GATT client to write these commands to the secondary

### 9.2 D2D TX Service (Secondary Device Only)

**Service UUID:** `**75ad68d6**-200c-437d-98b5-061862076c5f`

**Purpose:** This service runs on the secondary device to transmit sensor data and status information to the primary device. It mirrors the Information Service for D2D communication.

| Characteristic Name              | UUID                                    | Properties | Permissions      | Data Type                | Format/Endian | Description |
|---------------------------------|-----------------------------------------|------------|------------------|--------------------------|---------------|-------------|
| D2D Status                      | `**76ad68d6**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | uint32_t (bitfield)      | Little-endian | Device status relay to primary. |
| D2D Foot Sensor Log Available   | `**76ad68d7**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | uint8_t                  | N/A           | Notify primary of new foot logs. |
| D2D Charge Status               | `**76ad68d8**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | uint8_t                  | N/A           | Battery status relay to primary. |
| D2D Foot Sensor Req ID/Path     | `**76ad68d9**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | char[]                   | UTF-8         | Foot log path relay to primary. |
| D2D BHI360 Log Available        | `**76ad68da**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | uint8_t                  | N/A           | Notify primary of new BHI360 logs. |
| D2D BHI360 Req ID/Path          | `**76ad68db**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | char[]                   | UTF-8         | BHI360 log path relay to primary. |
| D2D Foot Sensor Samples         | `**76ad68dc**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | foot_samples_t           | Little-endian | Foot sensor data relay to primary. |
| D2D BHI360 3D Mapping           | `**76ad68dd**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | bhi360_3d_mapping_t      | Little-endian | BHI360 3D data relay to primary. |
| D2D BHI360 Step Count           | `**76ad68de**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | bhi360_step_count_t      | Little-endian | Step count relay to primary. |
| D2D BHI360 Linear Accel         | `**76ad68df**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | bhi360_linear_accel_t    | Little-endian | Linear accel relay to primary. |
| D2D Current Time                | `**76ad68e0**-200c-437d-98b5-061862076c5f`    | Notify     | Read             | struct                   | Little-endian | Time sync from secondary. |

**Notes:**
- This service only exists on secondary devices
- The secondary device acts as a GATT server, notifying the primary of data changes
- The primary device subscribes to these notifications and relays data to the phone

### 9.3 D2D File Transfer Service

**Service UUID:** `**8e600001**-b5a3-f393-e0a9-e50e24dcca9e`

**Purpose:** This service runs on both primary and secondary devices to handle file transfer operations between them. On secondary devices, it processes file commands from the primary. On primary devices, it acts as a client to request files.

| Characteristic Name | UUID                                    | Properties         | Permissions         | Data Type   | Format/Endian | Description |
|--------------------|-----------------------------------------|--------------------|---------------------|-------------|---------------|-------------|
| Command            | `**8e600002**-b5a3-f393-e0a9-e50e24dcca9e`    | Write              | Write               | uint8_t + data | N/A           | File operation commands. |
| Data               | `**8e600003**-b5a3-f393-e0a9-e50e24dcca9e`    | Notify             | Read                | byte[]      | N/A           | File data chunks. |
| Status             | `**8e600004**-b5a3-f393-e0a9-e50e24dcca9e`    | Notify             | Read                | uint8_t     | N/A           | Operation status. |

**Command Types:**
- `0x01`: List files
- `0x02`: Read file
- `0x03`: Delete file
- `0x04`: Get file info
- `0x05`: Abort operation

**Status Values:**
- `0x00`: OK
- `0x01`: Error
- `0x02`: Not found
- `0x03`: Busy
- `0x04`: End of data

**Data Packet Structure:**
```c
struct d2d_file_packet {
    uint8_t cmd;
    uint8_t status;
    uint16_t sequence;
    uint16_t length;
    uint8_t data[];  // Variable length
};
```

**Usage Flow (Primary requests file from Secondary):**
1. Primary writes command to Secondary's Command characteristic
2. Secondary processes command and sends data via Data notifications
3. Secondary sends status updates via Status notifications
4. Primary receives and processes the file data

**Notes:**
- This service exists on both device types but with different roles
- Secondary acts as GATT server, Primary acts as GATT client
- Used by File Proxy service to retrieve files from secondary devices


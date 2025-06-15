# Logging Specification

**Scope:** This document details the log file formats, BLE characteristics, and workflows for retrieving and managing sensor logs on the device.

**Purpose:** To provide a comprehensive reference for mobile app and integration developers on how to parse, retrieve, and manage log files from the device, including protobuf structures, BLE workflow, and SMP file transfer.

**Revision:** 0.1

**Author:** Firmware Team

---

This document details the log file formats, BLE characteristics, and workflows for retrieving and managing sensor logs on the device. It is intended for mobile app developers and integrators. Diagrams and protobuf field listings are included for clarity.

---

## Table of Contents
1. [Introduction](#introduction)
2. [Foot Sensor Log File](#foot-sensor-log-file)
3. [BHI360 Log File](#bhi360-log-file)
4. [BLE Characteristics for Logging](#ble-characteristics-for-logging)
5. [SMP Server File Retrieval](#smp-server-file-retrieval)
6. [Log Management Workflow](#log-management-workflow)
7. [Appendix: Protobuf Definitions](#appendix-protobuf-definitions)

---

## 1. Introduction

The device logs sensor data to files in a binary format using Protocol Buffers (protobuf). There are two main log types: Foot Sensor and BHI360. Each log file starts with a header (containing metadata such as sampling frequency and firmware version), followed by a sequence of data packets. **There are no per-packet timestamps.** BLE characteristics are used to discover, download, and delete log files. The SMP server is used for file transfer.

---

## 2. Foot Sensor Log File

### Description
- Contains time-synchronized readings from the foot sensor.
- Used for gait analysis, pressure mapping, and related analytics.

### File Structure
```
+-------------------+-------------------+-------------------+ ...
| Header (protobuf) | Packet 1 (protobuf) | Packet 2 (protobuf) | ...
+-------------------+-------------------+-------------------+ ...
```

#### Header (SensingData)
- Written as the first protobuf message in the file.
- Fields:
  - `firmware_version` (string): Firmware version string.
  - `sampling_frequency` (uint32): Logging frequency in Hz (e.g., 20 Hz).
  - `message_type` (enum): Indicates this is a header.

#### Data Packets (FootSensorLogMessage)
- Each packet is a protobuf message.
- Fields:
  - `readings` (repeated uint16): Array of 8 channel readings.
- **Packet size:** Each packet is a protobuf message. There are no explicit size fields or markers; the file is parsed by reading the header, then decoding each protobuf message in sequence. The size of each packet is determined by the protobuf encoding and may vary slightly depending on the data.
- **Timing:**
  - **No timestamps are stored in the packets.**
  - The first packet is time 0.
  - The time for packet N is: `time = N * (1 / frequency)` seconds, where frequency is from the header.
  - This means all packets are evenly spaced in time.

#### Example Layout
```
| Header | Packet 1 | Packet 2 | ... | Packet N |
```

#### Diagram
```
+--------+----------+----------+-----+
|Header  |Packet 1  |Packet 2  | ... |
+--------+----------+----------+-----+
```

---

## 3. BHI360 Log File

### Description
- Contains synchronized IMU data (quaternion, acceleration, step count, etc.) from the BHI360 sensor.
- Used for motion tracking, activity recognition, and analytics.

### File Structure
```
+-------------------+-------------------+-------------------+ ...
| Header (protobuf) | Packet 1 (protobuf) | Packet 2 (protobuf) | ...
+-------------------+-------------------+-------------------+ ...
```

#### Header (SensingData)
- Written as the first protobuf message in the file.
- Fields:
  - `firmware_version` (string): Firmware version string.
  - `sampling_frequency` (uint32): Logging frequency in Hz (e.g., 20 Hz).
  - `message_type` (enum): Indicates this is a header.

#### Data Packets (BHI360LogMessage)
- Each packet is a protobuf message, encoded in sequence.
- **Data structure:**  
  Each packet (of type `BHI360LogRecord`) contains the following fields, in this order:
  - `quat_x, quat_y, quat_z, quat_w` (float, 4 bytes each): Quaternion orientation (16 bytes total).
  - `quat_accuracy` (float, 4 bytes): Accuracy of the quaternion.
  - `lacc_x, lacc_y, lacc_z` (float, 4 bytes each): Linear acceleration (12 bytes total).
  - `step_count` (uint32, 4 bytes): Step count.
  - **Total payload size:** 36 bytes (not including protobuf encoding overhead).
- **How to parse:**  
  The mobile app should use the protobuf definition to decode each message in sequence. There are no explicit size fields or markers; the protobuf decoder will extract each message and advance to the next.
- **How to differentiate packets:**  
  Each packet is a `BHI360LogRecord` (see protobuf definition). The app should expect a series of these records, each with the above fields and sizes, and decode them using the protobuf library.
- **Timing:**
  - **No timestamps are stored in the packets.**
  - The first packet is time 0.
  - The time for packet N is: `time = N * (1 / frequency)` seconds, where frequency is from the header.
  - This means all packets are evenly spaced in time.

#### Example Layout
```
| Header | Packet 1 | Packet 2 | ... | Packet N |
```

#### Diagram
```
+--------+----------+----------+-----+
|Header  |Packet 1  |Packet 2  | ... |
+--------+----------+----------+-----+
```

---

## 4. BLE Characteristics for Logging

| Characteristic Name         | UUID (prefix bolded)                | Data Type         | Description |
|----------------------------|-------------------------------------|-------------------|-------------|
| Foot Sensor Log Available  | `**0c372eac**-27eb-437e-bef4-775aefaf3c97` | uint8_t          | ID of the latest available foot sensor log (after deletion, the newest closed log is sent; open files are not sent). |
| Foot Sensor Req ID/Path    | `**0c372eae**-27eb-437e-bef4-775aefaf3c97` | char[]           | Path/ID for the latest available foot sensor log file (never an open file). |
| Delete Foot Log Command    | `**4fd5b682**-9d89-4061-92aa-319ca786baae` | uint8_t (write)  | Delete foot sensor log by ID. |
| BHI360 Log Available       | `**0c372eb0**-27eb-437e-bef4-775aefaf3c97` | uint8_t          | ID of the latest available BHI360 log (after deletion, the newest closed log is sent; open files are not sent). |
| BHI360 Req ID/Path         | `**0c372eb1**-27eb-437e-bef4-775aefaf3c97` | char[]           | Path/ID for the latest available BHI360 log file (never an open file). |
| Delete BHI360 Log Command  | `**4fd5b683**-9d89-4061-92aa-319ca786baae` | uint8_t (write)  | Delete BHI360 log by ID. |

---

## 5. SMP Server File Retrieval

- After reading the file path characteristic (e.g., Foot Sensor Req ID/Path or BHI360 Req ID/Path), the mobile app uses the SMP server to request and download the file.
- **SMP Service UUID:** `**8D53DC1D**-1DB7-4CD3-868B-8A527460AA84`
- **Characteristic UUID:** `**DA2E7828**-FBCE-4E01-AE9E-261174997C48`
- The file path is used as the argument for the SMP file download command.
- See Zephyr SMP documentation for protocol details.

---

## 6. Log Management Workflow

### Overview Diagram
```
+-------------------+        +-------------------+        +-------------------+
|   Firmware        | <----> |   BLE Character.  | <----> |   Mobile App      |
+-------------------+        +-------------------+        +-------------------+
```

### Step-by-Step Workflow
1. **Discovery:**
   - Mobile app reads "Log Available" and "Req ID/Path" characteristics for both log types.
   - Receives the path/ID of the latest available log file (the newest closed log; open files are not sent).
2. **Download:**
   - Mobile app uses the SMP server and the file path to download the log file.
3. **Delete:**
   - After successful download, the app writes the log ID to the "Delete Log Command" characteristic to notify the firmware to delete the file.
4. **Repeat:**
   - The firmware updates the "Log Available" and "Req ID/Path" characteristics to point to the next latest file (the newest closed log remaining).
   - The app repeats the process until all logs are downloaded and deleted.

#### Sequence Diagram
```
Mobile App         Firmware
    |                  |
    |--Read Log Avail--|
    |<--Latest ID------|
    |--Read Path-------|
    |<--File Path------|
    |--SMP Download--->|
    |<--File Data------|
    |--Delete Log ID--->|
    |<--Ack/Update-----|
    | ... repeat ...   |
```

---

## 7. Appendix: Protobuf Definitions

### Foot Sensor Log Protobuf (foot_sensor_messages.proto)
```protobuf
message FootSensorLogMessage {
  oneof payload {
    FootSensorData foot_sensor = 1; // Data packet
    FootSensorSensingData sensing_data = 4; // Header
    FootSensorSessionEnd session_end = 5;  // End marker
  }
}

message FootSensorSensingData {
  string firmware_version = 1;
  uint32 sampling_frequency = 2;
  FootSensorMessageType message_type = 3;
}

message FootSensorData {
  repeated uint32 readings = 2; // 8 channels
}

message FootSensorSessionEnd {
  uint64 uptime_ms = 1;
}
```

### BHI360 Log Protobuf (bhi360_sensor_messages.proto)
```protobuf
message BHI360LogMessage {
  oneof payload {
    BHI360_3D_Data bhi360_3d = 2;
    BHI360_StepCounterData bhi360_step_counter = 3;
    BHI360SensingData sensing_data = 4;
    BHI360SessionEnd session_end = 5;
    BHI360LogRecord bhi360_log_record = 6; // New: packed log record
  }
}

message BHI360SensingData {
  string firmware_version = 1;
  uint32 sampling_frequency = 2;
  BHI360MessageType message_type = 3;
}

message BHI360LogRecord {
  float quat_x = 1;
  float quat_y = 2;
  float quat_z = 3;
  float quat_w = 4;
  float quat_accuracy = 5;
  float lacc_x = 6;
  float lacc_y = 7;
  float lacc_z = 8;
  uint32 step_count = 9;
  // No timestamp field in packets
}

message BHI360_3D_Data {
  Vector3f acceleration = 2;
  Vector3f angular_velocity = 3;
}

message BHI360_StepCounterData {
  uint32 step_count = 2;
  uint32 activity_duration_s = 3;
}

message BHI360SessionEnd {
  uint64 uptime_ms = 1;
}

message Vector3f {
  float x = 1;
  float y = 2;
  float z = 3;
}
```

---

For further questions or clarifications, refer to the firmware codebase or contact the firmware team.

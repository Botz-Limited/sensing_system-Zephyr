# Bluetooth GATT Specification

**Scope:** This document details the Bluetooth GATT services, characteristics, and protocols implemented on the device for mobile app and integration development.

**Purpose:** To provide a comprehensive reference for developers integrating with the device over BLE, including UUIDs, data types, permissions, and usage notes.

**Revision:** 0.1

**Author:** Firmware Team

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
  - [Exporting to CODA](#exporting-to-coda)

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

**Notes:**
- All characteristics use encrypted read and notify permissions.
- After a log file is deleted, the device will update the "Log Available" and "Req ID/Path" characteristics to point to the newest (latest) closed log file. Open files (currently being written to) are never sent.
- Data types such as `foot_samples_t` and `bhi360_3d_mapping_t` are C structs (see firmware for details).
- Notification is used for real-time updates (e.g., status, sensor data, log availability).

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

## Exporting to CODA

To export this document to CODA:
1. Open this Markdown file in your editor.
2. Copy all contents (Ctrl+A, Ctrl+C).
3. In CODA, create a new document or page.
4. Paste the Markdown content (Ctrl+V). CODA will automatically format headings, tables, and sections.
5. Review and adjust formatting as needed in CODA.

---

For further questions or clarifications, refer to the firmware codebase or contact the firmware team.

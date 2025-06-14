# Bluetooth Specification Document

This document details the Bluetooth GATT services and characteristics implemented on the device, including UUIDs, permissions, data types, and descriptions. It is intended for the mobile app team to facilitate communication with the device. SMP (Simple Management Protocol) server details are also included if present.

---

## Table of Contents
1. [Information Service](#information-service)
2. [Control Service](#control-service)
3. [Device Information Service](#device-information-service)
4. [SMP Server](#smp-server)

---

## 1. Information Service
**Service UUID:** `0c372eaa-27eb-437e-bef4-775aefaf3c97`

| Characteristic Name                | UUID                                    | Properties         | Permissions         | Data Type                | Description |
|------------------------------------|-----------------------------------------|--------------------|---------------------|--------------------------|-------------|
| Current Time                      | Standard CTS UUID                       | Read, Notify       | Read (encrypted)    | struct (time)            | Current time, updated from CTS. |
| Status                            | 0c372eab-27eb-437e-bef4-775aefaf3c97    | Read, Notify       | Read (encrypted)    | uint32_t                 | Error bitfield status. Notifies on change. |
| Foot Sensor Samples               | 0c372eaf-27eb-437e-bef4-775aefaf3c97    | Read, Notify       | Read (encrypted)    | foot_samples_t (struct)  | Foot sensor data samples. |
| Foot Sensor Log Available         | 0c372eac-27eb-437e-bef4-775aefaf3c97    | Read, Notify       | Read (encrypted)    | uint8_t                  | ID of available foot sensor log. |
| Charge Status                     | 0c372ead-27eb-437e-bef4-775aefaf3c97    | Read, Notify       | Read (encrypted)    | uint8_t                  | Battery/charge status. |
| Foot Sensor Req ID/Path           | 0c372eae-27eb-437e-bef4-775aefaf3c97    | Read, Notify       | Read (encrypted)    | char[] (max_path_length) | Path/ID for foot sensor log file. |
| BHI360 Log Available              | 0c372eb0-27eb-437e-bef4-775aefaf3c97    | Read, Notify       | Read (encrypted)    | uint8_t                  | ID of available BHI360 log. |
| BHI360 Req ID/Path                | 0c372eb1-27eb-437e-bef4-775aefaf3c97    | Read, Notify       | Read (encrypted)    | char[] (max_path_length) | Path/ID for BHI360 log file. |

**Notes:**
- All characteristics use encrypted read and notify permissions.
- Data types such as `foot_samples_t` are structs defined in firmware (see code for details).
- Notification is used for real-time updates (e.g., status, sensor data, log availability).

---

## 2. Control Service
**Service UUID:** `4fd5b67f-9d89-4061-92aa-319ca786baae`

| Characteristic Name         | UUID                                    | Properties         | Permissions                  | Data Type   | Description |
|----------------------------|-----------------------------------------|--------------------|------------------------------|-------------|-------------|
| Set Time Command           | 4fd5b681-9d89-4061-92aa-319ca786baae    | Read, Write        | Write (encrypted)            | uint32_t    | Set device time (epoch or custom format). |
| Delete Foot Log Command    | 4fd5b682-9d89-4061-92aa-319ca786baae    | Read, Write, Notify| Read/Write (encrypted)       | uint8_t     | Delete foot sensor log by ID. Notifies status. |
| Delete BHI360 Log Command  | 4fd5b683-9d89-4061-92aa-319ca786baae    | Read, Write, Notify| Read/Write (encrypted)       | uint8_t     | Delete BHI360 log by ID. Notifies status. |

**Notes:**
- Write operations are used to send commands (e.g., set time, delete log).
- Notify is used for status feedback after command execution.
- Data types are simple (uint8_t for IDs, uint32_t for time).

---

## 3. Device Information Service
**Service UUID:** Standard DIS UUID

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

## 4. SMP Server
- The SMP (Simple Management Protocol) server is typically used for device firmware updates and management.
- If present, it is usually implemented as a separate service (not detailed in the above files).
- The SMP server uses standard Zephyr/MCUboot characteristics and UUIDs.
- For mobile app integration, refer to Zephyr's SMP documentation for protocol details and supported commands.

---

## Additional Notes
- All services and characteristics use encrypted communication for security.
- For detailed data type definitions (e.g., `foot_samples_t`), refer to the firmware source code or request struct definitions from the firmware team.
- Notify properties are used for real-time updates; clients should subscribe to notifications as needed.

---

This document provides all necessary details for the mobile app team to communicate with the device over Bluetooth. For further questions or clarifications, refer to the firmware codebase or contact the firmware team.
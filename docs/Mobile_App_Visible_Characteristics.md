# Mobile App Visible Characteristics

## Overview

This document explains what BLE characteristics are visible to the mobile app when connected to the primary device.

## Data Flow Architecture

```
Secondary Device                Primary Device                    Mobile App
┌─────────────┐               ┌──────────────────┐              ┌──────────┐
│ Sensors     │               │ Information Svc  │              │          │
│ - Foot      │──D2D──────────>│ - Primary data   │──BLE────────>│  Sees    │
│ - BHI360    │  (Internal)   │ - Secondary data │              │  Both!   │
└─────────────┘               └──────────────────┘              └──────────┘
```

## What the Mobile App Sees

### Information Service (`0c372eaa-27eb-437e-bef4-775aefaf3c97`)

The mobile app sees BOTH primary and secondary device data through this single service:

#### Primary Device Data:
- **Foot Sensor Samples** - Primary foot pressure data
- **BHI360 3D Mapping Data** - Primary IMU quaternion/gyro data  
- **BHI360 Step Count** - Primary foot step count (global)
- **BHI360 Linear Acceleration** - Primary linear acceleration
- **Foot/BHI360/Activity Log Available/Path** - Primary device logs

#### Secondary Device Data (prefixed with "Secondary"):
- **Secondary Foot Log Available/Path** - Secondary foot sensor logs
- **Secondary BHI360 Log Available/Path** - Secondary IMU logs
- **Secondary Activity Log Available/Path** - Secondary activity logs
- **Secondary Manufacturer/Model/Serial/HW Rev/FW Rev** - Secondary device info
- **Secondary FOTA Progress** - Secondary firmware update status

#### Aggregated Data (combines both devices):
- **Total Step Count** - Sum of primary + secondary step counts
- **Activity Step Count** - Sum of primary + secondary activity steps
- **Device Status** - Combined status bits from both devices
- **Charge Status** - Primary device battery (secondary TBD)

### Control Service (`4fd5b67f-9d89-4061-92aa-319ca786baae`)

Commands that affect BOTH devices:
- **Start/Stop Activity** - Starts/stops on both devices
- **Set Time** - Sets time on both devices
- **Trigger BHI360 Calibration** - Calibrates both devices

Commands specific to primary:
- **Delete Foot/BHI360/Activity Log** - Deletes from primary only

Commands specific to secondary:
- **Delete Secondary Foot/BHI360/Activity Log** - Deletes from secondary only

### 3D Orientation Service (`0c372ec0-27eb-437e-bef4-775aefaf3c97`)

- **3D Orientation Data** - Combined quaternions from BOTH feet in one packet

## What the Mobile App Does NOT See

### Internal D2D Services (Hidden):
- **D2D RX Service** - Primary's internal service for receiving commands
- **D2D TX Service** - Secondary's service for sending data to primary
- These are used for device-to-device communication only

### How It Works:
1. Mobile app writes to Control Service
2. Primary device internally forwards to secondary via D2D RX
3. Secondary executes command and sends data back via D2D TX
4. Primary aggregates data and exposes through Information Service
5. Mobile app sees unified view of both devices

## Key Points

1. **Single Connection**: Mobile only connects to primary device
2. **Transparent Aggregation**: Secondary data appears alongside primary data
3. **Unified Control**: Commands automatically propagate to both devices
4. **No D2D Complexity**: Mobile app doesn't need to know about D2D protocol

## Example: Step Counting

```
Secondary BHI360 ──> D2D TX ──> Primary D2D RX ──> Information Service
    (1000 steps)                                    │
                                                    ├─> BHI360 Step Count: 1000
Primary BHI360 ─────────────────────────────────────┤
    (1500 steps)                                    ├─> Total Step Count: 2500
                                                    │
Mobile App <────────────────────────────────────────┘
```

The mobile app sees:
- Individual counts via existing characteristics
- Total aggregated count via new Total Step Count characteristic
- Activity-specific counts via Activity Step Count characteristic
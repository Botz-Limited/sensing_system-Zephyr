# Device-to-Device (D2D) Communication Analysis

## Overview

The firmware supports a primary-secondary device configuration where:
- **Primary Device** (`CONFIG_PRIMARY_DEVICE=y`): Right foot sensor that connects to the mobile app
- **Secondary Device** (`CONFIG_PRIMARY_DEVICE=n`): Left foot sensor that connects to the primary device

## Communication Flow

### 1. Commands: Mobile App → Primary → Secondary

The primary device forwards control commands from the mobile app to the secondary device:

| Command | Direction | Implementation Status |
|---------|-----------|----------------------|
| Set Time | Primary → Secondary | ✅ Implemented via `ble_d2d_tx_send_set_time_command()` |
| Delete Foot Log | Primary → Secondary | ✅ Implemented via `ble_d2d_tx_send_delete_foot_log_command()` |
| Delete BHI360 Log | Primary → Secondary | ✅ Implemented via `ble_d2d_tx_send_delete_bhi360_log_command()` |
| Start Activity | Primary → Secondary | ✅ Implemented via `ble_d2d_tx_send_start_activity_command()` |
| Stop Activity | Primary → Secondary | ✅ Implemented via `ble_d2d_tx_send_stop_activity_command()` |

### 2. Data: Secondary → Primary → Mobile App

The secondary device should send sensor data to the primary, but **the implementation is incomplete**:

| Data Type | Expected Flow | Current Status |
|-----------|--------------|----------------|
| Foot Sensor Samples | Secondary → Primary → Mobile | ❌ Function exists but doesn't send via GATT |
| BHI360 3D Mapping | Secondary → Primary → Mobile | ❌ Function exists but doesn't send via GATT |
| BHI360 Step Count | Secondary → Primary → Mobile | ❌ Function exists but doesn't send via GATT |
| BHI360 Linear Accel | Secondary → Primary → Mobile | ❌ Function exists but doesn't send via GATT |
| Foot Log Available | Secondary → Primary → Mobile | ❌ Function exists but doesn't send via GATT |
| BHI360 Log Available | Secondary → Primary → Mobile | ❌ Function exists but doesn't send via GATT |
| Device Status | Secondary → Primary → Mobile | ❌ Function exists but doesn't send via GATT |
| Charge Status | Secondary → Primary → Mobile | ❌ Function exists but doesn't send via GATT |
| FOTA Complete Status | Secondary → Primary | ✅ Implemented via GATT write |

## Current Implementation Issues

### 1. **Incomplete Secondary → Primary Data Transfer**

The `ble_d2d_tx.cpp` functions for sending sensor data are stubs:

```cpp
int ble_d2d_tx_send_foot_sensor_data(const foot_samples_t *samples) {
    if (!d2d_conn) return -ENOTCONN;
    
    LOG_DBG("D2D TX: Sending foot sensor data");
    
    // In a real implementation, you would:
    // 1. Find the characteristic handle for foot sensor data
    // 2. Use bt_gatt_write or bt_gatt_notify
    // For now, we'll put it in a message queue for the primary to handle
    
    generic_message_t msg;
    msg.sender = SENDER_D2D_SECONDARY;
    msg.type = MSG_TYPE_FOOT_SAMPLES;
    memcpy(&msg.data.foot_samples, samples, sizeof(foot_samples_t));
    
    // Send to bluetooth message queue on primary
    // This is a workaround - in production, use proper GATT
    return 0;
}
```

**Problem**: The data is prepared but never actually sent over BLE GATT.

### 2. **Missing GATT Service for Secondary → Primary**

While the primary device has a D2D RX service to receive commands, there's no corresponding TX service on the secondary device to send data back.

### 3. **Message Queue Workaround**

Currently, the secondary device prepares messages but doesn't send them via BLE. The primary device expects to receive these via its message queue, which won't work across devices.

## What Needs to Be Implemented

### 1. **Create D2D TX GATT Service on Secondary Device**

```cpp
// In ble_d2d_tx.cpp for secondary device
static struct bt_gatt_service d2d_tx_service = BT_GATT_SERVICE(
    &d2d_tx_service_uuid.uuid,
    
    // Foot sensor data characteristic (notify)
    BT_GATT_CHARACTERISTIC(&d2d_foot_sensor_samples_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(foot_sensor_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // BHI360 data characteristics (notify)
    BT_GATT_CHARACTERISTIC(&d2d_bhi360_data1_uuid.uuid,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(bhi360_data1_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // ... more characteristics
);
```

### 2. **Implement Actual GATT Notifications**

```cpp
int ble_d2d_tx_send_foot_sensor_data(const foot_samples_t *samples) {
    if (!d2d_conn) return -ENOTCONN;
    
    // Find the foot sensor characteristic (e.g., index 1)
    return bt_gatt_notify(d2d_conn, &d2d_tx_service.attrs[1], 
                         samples, sizeof(foot_samples_t));
}
```

### 3. **Primary Device Service Discovery**

The primary device needs to discover and subscribe to the secondary's TX service characteristics.

## File Transfer via D2D

There's a separate D2D file transfer service (`ble_d2d_file_transfer.cpp`) that handles:
- Listing log files on secondary device
- Reading log files from secondary device
- Deleting log files on secondary device
- Getting file info from secondary device

This appears to be more complete than the sensor data transfer.

## Recommendations

1. **Complete the D2D TX Service Implementation**
   - Add proper GATT service with notify characteristics
   - Implement actual bt_gatt_notify calls
   - Add service discovery on primary side

2. **Define Clear Data Flow**
   - Secondary collects sensor data
   - Secondary notifies primary via D2D TX service
   - Primary aggregates data from both devices
   - Primary notifies mobile app with combined data

3. **Consider Data Aggregation Strategy**
   - Should primary forward raw secondary data?
   - Should primary combine/synchronize data before sending?
   - How to handle timing synchronization?

4. **Test MTU Exchange for D2D**
   - The MTU fix should also benefit D2D communication
   - Larger packets = more efficient sensor data transfer

## Current Working Features

✅ **Primary → Secondary Commands**: All control commands are properly forwarded
✅ **FOTA Status**: Secondary can notify primary of FOTA completion
✅ **File Transfer**: D2D file transfer service appears functional
✅ **Connection Management**: D2D connections are properly established

## Not Working / Incomplete

❌ **Secondary → Primary Sensor Data**: Functions exist but don't use GATT
❌ **Secondary → Primary Log Notifications**: Functions exist but don't use GATT
❌ **Secondary → Primary Status Updates**: Functions exist but don't use GATT

The infrastructure is in place, but the actual BLE GATT implementation for secondary-to-primary data transfer needs to be completed.
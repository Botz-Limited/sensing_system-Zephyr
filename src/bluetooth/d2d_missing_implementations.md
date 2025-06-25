# Missing D2D Implementations Summary

## Current Status

### Working D2D Command Cascading (Phone → Primary → Secondary):
1. ✅ Set Time
2. ✅ Delete Foot Log
3. ✅ Delete BHI360 Log
4. ✅ Start Activity
5. ✅ Stop Activity
6. ✅ Trigger BHI360 Calibration
7. ❌ Delete Activity Log (missing in D2D RX)

### Working D2D Data Flow (Secondary → Primary → Phone):
1. ✅ Foot Sensor Data
2. ✅ BHI360 3D Mapping Data
3. ✅ BHI360 Step Count
4. ✅ BHI360 Linear Acceleration
5. ✅ Status
6. ✅ Charge Status
7. ✅ Foot Log Available
8. ✅ BHI360 Log Available
9. ✅ Device Info
10. ❌ Foot Log Path (not implemented in GATT)
11. ❌ BHI360 Log Path (not implemented in GATT)
12. ❌ Activity Log Available (not implemented in GATT)
13. ❌ Activity Log Path (not implemented in GATT)
14. ❌ FOTA Progress (not implemented in GATT)

## Required Changes

### 1. Add to ble_d2d_tx_service.cpp:
- Add missing UUID definitions for paths and FOTA progress
- Add characteristics to service definition
- Add notification functions
- Update characteristic indices

### 2. Add to ble_d2d_rx.cpp:
- Add delete activity log command handler
- Add characteristic to service definition

### 3. Update ble_d2d_tx.cpp:
- Implement path sending functions (currently stubbed)
- Add FOTA progress forwarding

### 4. Update bluetooth.cpp:
- Add FOTA progress forwarding from secondary to primary
- Ensure activity log notifications are forwarded

### 5. Update app.cpp:
- Add D2D FOTA progress notification in notify_fota_progress_to_ble() for secondary device

## Implementation Priority

1. **FOTA Progress** - Critical for secondary device updates visibility
2. **File Paths** - Needed for complete file management
3. **Activity Logs** - Complete the activity tracking feature

## Code Changes Needed

### In app.cpp, modify notify_fota_progress_to_ble():
```cpp
void notify_fota_progress_to_ble()
{
    generic_message_t msg;
    msg.sender = SENDER_NONE;
    msg.type = MSG_TYPE_FOTA_PROGRESS;
    
    // Pack progress data
    msg.data.fota_progress.is_active = fota_progress.is_active;
    msg.data.fota_progress.status = fota_progress.status;
    msg.data.fota_progress.percent_complete = fota_progress.percent_complete;
    msg.data.fota_progress.bytes_received = fota_progress.bytes_received;
    msg.data.fota_progress.total_size = fota_progress.total_size;
    msg.data.fota_progress.error_code = fota_progress.error_code;
    
    // Send to Bluetooth thread
    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send FOTA progress to Bluetooth thread");
    }
    
    #if IS_ENABLED(CONFIG_SECONDARY_DEVICE)
    // Also send via D2D to primary
    ble_d2d_tx_send_fota_progress(&msg.data.fota_progress);
    #endif
}
```

### In bluetooth.cpp, handle FOTA progress for secondary:
```cpp
case MSG_TYPE_FOTA_PROGRESS: {
    fota_progress_msg_t *progress = &msg.data.fota_progress;
    LOG_INF("Received FOTA Progress: active=%d, status=%d, percent=%d%%, bytes=%u/%u",
            progress->is_active, progress->status, progress->percent_complete,
            progress->bytes_received, progress->total_size);
    
    #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    jis_fota_progress_notify(progress);
    #else
    // Secondary device: Send to primary via D2D
    ble_d2d_tx_send_fota_progress(progress);
    #endif
    break;
}
```
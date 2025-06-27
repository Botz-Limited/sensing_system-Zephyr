# Step Count Implementation Verification Report

## 1. Logging Flag Control ✓

**Verified**: The logging flags are ONLY controlled by Bluetooth commands from the mobile phone.

### Control Flow:
1. Mobile phone writes to Control Service characteristics
2. Control Service submits events: `motion_sensor_start_activity_event` / `motion_sensor_stop_activity_event`
3. Motion sensor receives events and sets flags:
   - `atomic_set(&logging_active, 1)` on start
   - `atomic_set(&logging_active, 0)` on stop
   - `activity_logging_active = true/false`

**No other modules can enable/disable logging** - it's exclusively controlled via BLE commands.

## 2. Bluetooth Specification Updated ✓

The Bluetooth GATT Specification has been updated with:

### New Step Count Characteristics:
- **BHI360 Step Count** (`...eb3`): Individual foot global count (4 bytes)
- **Total Step Count** (`...ec4`): Aggregated both feet (4 bytes)
- **Activity Step Count** (`...ec5`): Activity-specific steps (4 bytes)

### Key Changes:
- Removed `activity_duration_s` field (deprecated, always 0)
- Added new `step_count_only_t` structure (4 bytes)
- Added detailed step count behavior documentation
- Clarified that time metrics should be calculated by mobile app

## 3. Step Count Separation Issues Found ⚠️

### Issue 1: Unreliable Activity Step Detection from Secondary Device

**Problem**: The code tries to detect if secondary device is sending activity steps by checking if the value is small:

```c
if (step_data->step_count < 10000 && secondary_step_count > 10000) {
    // Likely activity steps from secondary
    secondary_activity_steps = step_data->step_count;
}
```

**Why it's wrong**:
- Activity steps could be > 10000 in a long session
- Global steps could be < 10000 after device restart
- This heuristic is fundamentally unreliable

### Issue 2: Secondary Device Activity Step Transmission

The secondary device sends activity steps via the same channel as global steps, making it impossible to distinguish them reliably.

## Recommended Fixes:

### Fix 1: Add Activity Flag to Step Count Message
Modify the step count message to include a flag indicating if it's activity steps:

```c
// In motion_sensor.cpp when sending activity steps:
activity_step_msg.data.bhi360_step_count.activity_duration_s = 0xFF; // Use as activity flag
```

Then in bluetooth.cpp:
```c
if (msg.sender == SENDER_D2D_SECONDARY) {
    if (step_data->activity_duration_s == 0xFF) {
        // This is activity steps
        secondary_activity_steps = step_data->step_count;
    } else {
        // This is global steps
        secondary_step_count = step_data->step_count;
    }
}
```

### Fix 2: Create Separate Message Type
Add a new message type `MSG_TYPE_ACTIVITY_STEP_COUNT` specifically for activity steps, keeping it separate from global step counts.

### Fix 3: Use Different Sender ID
Have secondary device use `SENDER_MOTION_SENSOR` for activity steps (like primary does) and `SENDER_D2D_SECONDARY` only for global steps.

## Current Status:

1. **Logging Control**: ✓ Working correctly
2. **BLE Specification**: ✓ Updated
3. **Step Count Separation**: ⚠️ Has bugs that need fixing

The main issue is distinguishing activity vs global steps from the secondary device. The current heuristic based on step count value is unreliable and should be replaced with an explicit flag or separate message type.
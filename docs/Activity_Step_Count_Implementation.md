# Activity Step Count Implementation

## Overview

This document describes the implementation of separate activity step count tracking from global step counts, ensuring proper separation between primary and secondary devices.

## Problem Statement

The original implementation had a bug where the secondary device's activity step counts couldn't be distinguished from global step counts when received via D2D. The code used an unreliable heuristic (`step_count < 10000`) to guess if it was activity steps.

## Solution

Implemented separate D2D characteristics and message flows for activity step counts:

### 1. New D2D Characteristic

Added a dedicated D2D characteristic for activity step counts:
- UUID: `76ad68e6-200c-437d-98b5-061862076c5f`
- Same data structure as global step count (`bhi360_step_count_t`)
- Separate from the global step count characteristic

### 2. Message Flow

#### Primary Device:
1. Motion sensor sends activity steps with `sender = SENDER_MOTION_SENSOR`
2. Bluetooth module recognizes this sender and updates `primary_activity_steps`
3. Aggregates with `secondary_activity_steps` and notifies mobile app

#### Secondary Device:
1. Motion sensor sends activity steps via `ble_d2d_tx_send_activity_step_count()`
2. D2D TX service notifies via the new activity step count characteristic
3. Primary's D2D RX client receives and calls `d2d_data_handler_process_activity_step_count()`
4. Handler sends to bluetooth module with `MSG_TYPE_ACTIVITY_STEP_COUNT`
5. Bluetooth module updates `secondary_activity_steps` and aggregates

### 3. Key Components Modified

#### Motion Sensor (`motion_sensor.cpp`):
- Sends activity steps with `SENDER_MOTION_SENSOR` (primary)
- Uses `ble_d2d_tx_send_activity_step_count()` (secondary)

#### D2D TX Service:
- Added `d2d_activity_step_count_uuid`
- Added `d2d_tx_notify_activity_step_count()` function
- Added characteristic to GATT service definition

#### D2D RX Client:
- Added discovery and subscription for activity step count characteristic
- Added `activity_step_count_notify_handler()`

#### D2D Data Handler:
- Added `d2d_data_handler_process_activity_step_count()`
- Sends `MSG_TYPE_ACTIVITY_STEP_COUNT` to bluetooth module

#### Bluetooth Module:
- Removed unreliable heuristic check
- Added separate handler for `MSG_TYPE_ACTIVITY_STEP_COUNT`
- Properly updates `secondary_activity_steps`

## Benefits

1. **Reliable Separation**: No more guessing based on step count values
2. **Clean Architecture**: Activity and global steps use separate channels
3. **Consistent Design**: Both primary and secondary use the same pattern
4. **Future Proof**: Can handle any step count value without confusion

## Testing

To verify the implementation:

1. Start an activity on both devices
2. Check that activity step counts increment separately from global counts
3. Verify aggregation shows correct total activity steps
4. Confirm global step counts continue independently

## Bluetooth Specification Update

The Bluetooth GATT Specification has been updated to version 2.2 with:
- New Total Step Count characteristic for aggregated steps
- New Activity Step Count characteristic for activity-specific steps
- Clarification that duration field is deprecated (always 0)
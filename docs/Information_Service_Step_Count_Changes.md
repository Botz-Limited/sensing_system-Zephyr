# Information Service Step Count Changes

## Overview
The Information Service has been modified to only send aggregated step count data to mobile phones, while maintaining full D2D communication between primary and secondary devices.

## Changes Made

### 1. Bluetooth Module (bluetooth.cpp)
- Disabled individual foot step count notifications to phone
- The line `jis_bhi360_data2_notify(step_data)` has been commented out
- Only aggregated step counts are now sent to the phone

### 2. D2D Data Handler (d2d_data_handler.cpp)
- Removed forwarding of individual secondary foot step counts to phone
- Secondary step count data is still received and processed for aggregation
- The aggregation logic remains unchanged

### 3. Information Service (information_service.cpp)
- Added deprecation comment to the individual foot step count characteristic
- The characteristic remains in the service for backward compatibility but should not be used by new mobile apps

## Data Flow

### Before Changes:
1. Primary foot sensor → Primary device → Phone (individual primary steps)
2. Secondary foot sensor → Secondary device → Primary device → Phone (individual secondary steps)
3. Primary device calculates aggregated counts → Phone (total steps)

### After Changes:
1. Primary foot sensor → Primary device (internal processing only)
2. Secondary foot sensor → Secondary device → Primary device (D2D unchanged)
3. Primary device calculates aggregated counts → Phone (total steps only)

## Available Step Count Characteristics

### For Mobile Apps (Use These):
1. **Total Step Count** (UUID: 0x0c372ec4...)
   - Aggregated global step count from both feet
   - Updated whenever either foot reports new steps

2. **Activity Step Count** (UUID: 0x0c372ec5...)
   - Aggregated step count during current activity session
   - Reset when activity session ends

### Deprecated (Do Not Use):
1. **BHI360 Data2** (UUID: 0x0c372eb3...)
   - Individual foot step count
   - Kept for backward compatibility only
   - No longer sends notifications

## D2D Communication (Unchanged)
The secondary device continues to send all sensor data to the primary device via D2D, including:
- Foot sensor samples
- BHI360 3D mapping data
- BHI360 step counts (for aggregation)
- BHI360 linear acceleration
- Status and charge information
- Log availability notifications

## Mobile App Migration Guide
1. Subscribe to "Total Step Count" (0x0c372ec4...) for global steps
2. Subscribe to "Activity Step Count" (0x0c372ec5...) for activity-specific steps
3. Unsubscribe from "BHI360 Data2" (0x0c372eb3...) - individual foot steps
4. Update UI to show only aggregated counts, not left/right foot distinction
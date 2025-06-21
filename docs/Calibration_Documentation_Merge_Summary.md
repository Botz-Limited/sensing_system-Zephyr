# Calibration Documentation Merge Summary

## Overview
The BHI360 calibration message flow documentation has been successfully merged into the appropriate existing documents in the docs directory.

## Documents Updated

### 1. BHI360_Complete_Integration_Guide.md
**Added Section: "Calibration Message Flow"**
- Message types table with sender/receiver information
- Startup calibration loading sequence diagram
- Manual calibration trigger sequence diagram
- Periodic calibration updates description
- Calibration data storage format and location

This new section was inserted before the "Implementation Guide" section to provide a complete understanding of how calibration works at the system level.

### 2. Developer_Quick_Reference.md (and .coda)
**Added Section: "Message Types" (Section 5)**
- Added comprehensive list of message types organized by category:
  - Calibration Messages
  - Data Flow Messages
  - Control Messages
- Updated table of contents to include the new section
- Renumbered subsequent sections (6-11)

**Updated:**
- Added calibration-related message types that developers need to know
- Maintained the quick reference format for easy lookup

### 3. Previously Updated Documents
The following documents were already updated in the previous task:
- **Bluetooth_GATT_Specification.md/coda**: Added calibration trigger characteristics
- **D2D_Communication_Complete_Guide.md/coda**: Added calibration command to supported commands

## Key Information Merged

### Message Types
- `MSG_TYPE_TRIGGER_BHI360_CALIBRATION`: Triggers calibration from BLE/Control
- `MSG_TYPE_REQUEST_BHI360_CALIBRATION`: Requests stored calibration at startup
- `MSG_TYPE_BHI360_CALIBRATION_DATA`: Carries calibration data response
- `MSG_TYPE_SAVE_BHI360_CALIBRATION`: Saves calibration to storage

### Calibration Flow
1. **Startup**: Motion sensor requests stored calibration from data module
2. **Manual Trigger**: User initiates via BLE characteristic
3. **Periodic Check**: Automatic improvement detection every ~10 seconds
4. **Storage**: Calibration profiles saved to `/lfs1/calibration/`

### File Storage Format
```
/lfs1/calibration/
├── bhi360_calib_accel.bin
└── bhi360_calib_gyro.bin
```

## Benefits of Integration

1. **Centralized Documentation**: All calibration information is now in context within relevant guides
2. **No Duplication**: Information is placed where developers/users would naturally look for it
3. **Maintained Consistency**: Both MD and Coda versions updated identically
4. **Better Discoverability**: Calibration info integrated into existing navigation structures

## Removed Documents
- `BHI360_Calibration_Message_Flow.md` - Content fully integrated into existing documents

The calibration system documentation is now fully integrated into the project's documentation structure, making it easier for developers to find and understand the calibration functionality in context.
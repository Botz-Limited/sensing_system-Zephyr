# GATT Attribute Count Analysis

## Service Breakdown

### 1. Information Service (Primary Device Only)
- 1 Primary Service declaration
- Current Time: 2 attrs (char + value)
- Status: 3 attrs (char + value + CCC)
- Foot Sensor Samples: 3 attrs (char + value + CCC)
- Foot Log Available: 3 attrs (char + value + CCC)
- Charge Status: 2 attrs (char + value)
- Foot Log Path: 3 attrs (char + value + CCC)
- BHI360 Log Available: 3 attrs (char + value + CCC)
- BHI360 Log Path: 3 attrs (char + value + CCC)
- BHI360 Data1: 3 attrs (char + value + CCC)
- BHI360 Data2: 3 attrs (char + value + CCC)
- BHI360 Data3: 3 attrs (char + value + CCC)
- Total Step Count: 3 attrs (char + value + CCC)
- Activity Step Count: 3 attrs (char + value + CCC)
- FOTA Progress: 3 attrs (char + value + CCC)
- Activity Log Available: 3 attrs (char + value + CCC)
- Activity Log Path: 3 attrs (char + value + CCC)
- Secondary Manufacturer: 2 attrs (char + value)
- Secondary Model: 2 attrs (char + value)
- Secondary Serial: 2 attrs (char + value)
- Secondary HW Rev: 2 attrs (char + value)
- Secondary FW Rev: 2 attrs (char + value)
- Secondary FOTA Progress: 3 attrs (char + value + CCC)
- Secondary Foot Log Available: 3 attrs (char + value + CCC)
- Secondary Foot Log Path: 3 attrs (char + value + CCC)
- Secondary BHI360 Log Available: 3 attrs (char + value + CCC)
- Secondary BHI360 Log Path: 3 attrs (char + value + CCC)
- Secondary Activity Log Available: 3 attrs (char + value + CCC)
- Secondary Activity Log Path: 3 attrs (char + value + CCC)
**Total: 1 + 77 = 78 attributes**

### 2. Control Service (Primary Device Only)
- 1 Primary Service declaration
- Set Time: 2 attrs (char + value)
- Delete Foot Log: 2 attrs (char + value)
- Delete BHI360 Log: 2 attrs (char + value)
- Start Activity: 2 attrs (char + value)
- Stop Activity: 2 attrs (char + value)
- Trigger BHI360 Calibration: 3 attrs (char + value + CCC)
- Delete Activity Log: 2 attrs (char + value)
- Delete Secondary Foot Log: 2 attrs (char + value)
- Delete Secondary BHI360 Log: 2 attrs (char + value)
- Delete Secondary Activity Log: 2 attrs (char + value)
- Connection Parameter Control: 2 attrs (char + value)
**Total: 1 + 23 = 24 attributes**

### 3. Device Information Service
- 1 Primary Service declaration
- Manufacturer Name: 2 attrs (char + value)
- Model Number: 2 attrs (char + value)
- Serial Number: 2 attrs (char + value)
- Hardware Revision: 2 attrs (char + value)
- Firmware Revision: 2 attrs (char + value)
**Total: 1 + 10 = 11 attributes**

### 4. Battery Service
- 1 Primary Service declaration
- Battery Level: 3 attrs (char + value + CCC)
**Total: 1 + 3 = 4 attributes**

### 5. FOTA Proxy Service (Primary Device Only)
- 1 Primary Service declaration
- Target Selection: 2 attrs (char + value)
- Command: 2 attrs (char + value)
- Data: 2 attrs (char + value)
- Status: 3 attrs (char + value + CCC)
**Total: 1 + 9 = 10 attributes**

### 6. File Proxy Service (Primary Device Only)
- 1 Primary Service declaration
- Target Device: 2 attrs (char + value)
- File Command: 2 attrs (char + value)
- File Data: 3 attrs (char + value + CCC)
- File Status: 3 attrs (char + value + CCC)
**Total: 1 + 10 = 11 attributes**

### 7. SMP Proxy Service (Primary Device Only)
- 1 Primary Service declaration
- Target Select: 2 attrs (char + value)
- SMP Data: 3 attrs (char + value + CCC)
**Total: 1 + 5 = 6 attributes**

### 8. 3D Orientation Service (Primary Device Only)
- 1 Primary Service declaration
- 3D Orientation: 3 attrs (char + value + CCC)
**Total: 1 + 3 = 4 attributes**

### 9. D2D RX Service (Primary Device Only)
- 1 Primary Service declaration
- Set Time: 2 attrs (char + value)
- Delete Foot Log: 2 attrs (char + value)
- Delete BHI360 Log: 2 attrs (char + value)
- Start Activity: 2 attrs (char + value)
- Stop Activity: 2 attrs (char + value)
- Trigger BHI360 Calibration: 2 attrs (char + value)
- FOTA Status: 2 attrs (char + value)
- Delete Activity Log: 2 attrs (char + value)
**Total: 1 + 16 = 17 attributes**

### 10. D2D TX Service (Secondary Device Only)
- 1 Primary Service declaration
- Foot Sensor: 3 attrs (char + value + CCC)
- BHI360 Data1: 3 attrs (char + value + CCC)
- BHI360 Data2: 3 attrs (char + value + CCC)
- BHI360 Data3: 3 attrs (char + value + CCC)
- Status: 3 attrs (char + value + CCC)
- Charge Status: 3 attrs (char + value + CCC)
- Foot Log Available: 3 attrs (char + value + CCC)
- BHI360 Log Available: 3 attrs (char + value + CCC)
- Device Info: 3 attrs (char + value + CCC)
- FOTA Progress: 3 attrs (char + value + CCC)
- Foot Log Path: 3 attrs (char + value + CCC)
- BHI360 Log Path: 3 attrs (char + value + CCC)
- Activity Log Available: 3 attrs (char + value + CCC)
- Activity Log Path: 3 attrs (char + value + CCC)
- Activity Step Count: 3 attrs (char + value + CCC)
**Total: 1 + 45 = 46 attributes**

### 11. D2D File Transfer Service (Secondary Device Only)
- 1 Primary Service declaration
- Command: 2 attrs (char + value)
- Data: 3 attrs (char + value + CCC)
- Status: 3 attrs (char + value + CCC)
**Total: 1 + 8 = 9 attributes**

## Total Attribute Count

### Primary Device:
- Information Service: 78
- Control Service: 24
- Device Information Service: 11
- Battery Service: 4
- FOTA Proxy Service: 10
- File Proxy Service: 11
- SMP Proxy Service: 6
- 3D Orientation Service: 4
- D2D RX Service: 17
- GAP Service (built-in): ~6
- GATT Service (built-in): ~4
**Total: ~175 attributes**

### Secondary Device:
- Device Information Service: 11
- Battery Service: 4
- D2D TX Service: 46
- D2D File Transfer Service: 9
- SMP Service (standard): ~6
- GAP Service (built-in): ~6
- GATT Service (built-in): ~4
**Total: ~86 attributes**

## Analysis

The primary device uses approximately 175 GATT attributes, which is quite high. The default GATT database size in Zephyr might not be sufficient for this many attributes, especially when combined with the subscription requirements.

## Recommendations

1. Check if there's a Kconfig option to increase the GATT database size
2. Consider optimizing services by removing unused characteristics
3. Ensure the heap size is adequate for dynamic GATT allocations
4. Consider using dynamic GATT registration if static registration hits limits
# BHI360 Driver Integration Test Plan

## Objective
Verify that the driver integration maintains 100% data compatibility with the original implementation.

## Test Setup
1. Flash the updated firmware
2. Connect debugging tools (RTT viewer, logic analyzer)
3. Prepare test movements for sensor validation

## Test Cases

### 1. Driver Initialization Test
**Purpose**: Verify driver initializes correctly
**Steps**:
1. Power on device
2. Monitor logs for:
   - "BHI360 device driver ready"
   - "BHI360 driver ready, BHY2 handle obtained"
   - "BHI360: Boot successful, kernel version"
**Expected**: All initialization messages appear without errors

### 2. Data Value Verification
**Purpose**: Verify sensor data values are identical
**Test Method**: Compare raw values before and after integration

#### 2.1 Quaternion Data Test
- **Expected Values**: Range [-1.0, 1.0] for x,y,z,w
- **Verify**: Values = raw_data / 16384.0f
- **Log Check**: Look for quaternion values in parse_all_sensors

#### 2.2 Linear Acceleration Test
- **Expected Values**: m/s² units
- **Verify**: Values = raw_data / 100.0f
- **Test**: Move device, verify gravity is removed

#### 2.3 Gyroscope Test
- **Expected Values**: rad/s units
- **Verify**: Values = raw_data / 16384.0f
- **Test**: Rotate device, verify angular velocity

#### 2.4 Step Counter Test
- **Expected Values**: Incrementing count
- **Test**: Walk 10 steps, verify count increases

### 3. Message Queue Verification
**Purpose**: Verify message formats unchanged

#### 3.1 Data Queue Messages
```
Monitor: k_msgq_put(&data_msgq, &msg, K_NO_WAIT)
Verify: MSG_TYPE_BHI360_LOG_RECORD structure
Check: All float values present and scaled correctly
```

#### 3.2 Bluetooth Queue Messages
```
Monitor: k_msgq_put(&bluetooth_msgq, ...)
Verify: Three separate messages sent
- MSG_TYPE_BHI360_3D_MAPPING
- MSG_TYPE_BHI360_LINEAR_ACCEL  
- MSG_TYPE_BHI360_STEP_COUNT
```

### 4. Timing Verification
**Purpose**: Verify data synchronization unchanged
- **Test**: Monitor motion_update_mask behavior
- **Verify**: Data sent only when all motion sensors updated
- **Check**: 50Hz rate for motion, 5Hz for steps

### 5. Protobuf Encoding Test
**Purpose**: Verify data.cpp processes data identically
- **Monitor**: Log output in data.cpp
- **Verify**: Fixed-point conversions match
  - Quaternion: ×10000
  - Linear Accel: ×1000
  - Gyroscope: ×10000

### 6. Bluetooth Notification Test
**Purpose**: Verify BLE characteristics unchanged
- **Tool**: nRF Connect app
- **Check**: Notification values match expected format
- **Verify**: Data rate and packaging unchanged

### 7. Interrupt Performance Test
**Purpose**: Verify interrupt handling performance
- **Measure**: Time between interrupt and FIFO processing
- **Compare**: Original vs driver implementation
- **Expected**: <1ms additional latency

### 8. Error Handling Test
**Purpose**: Verify error conditions handled properly
- **Test Cases**:
  1. Disconnect SPI (if possible)
  2. Corrupt firmware upload
  3. Invalid sensor configuration
- **Verify**: Appropriate error messages logged

### 9. Long Duration Test
**Purpose**: Verify stability over time
- **Duration**: 24 hours minimum
- **Monitor**: 
  - Memory usage
  - Stack usage
  - Error counts
  - Data consistency

### 10. Data Logging Verification
**Purpose**: Verify external flash logging unchanged
- **Check**: Log files created with same format
- **Verify**: Protobuf messages decode correctly
- **Tool**: Parse log files with existing tools

## Regression Testing

### Before Integration (Baseline)
1. Capture 1000 samples of each sensor
2. Save to reference file
3. Note timing characteristics

### After Integration
1. Capture 1000 samples of each sensor
2. Compare with reference:
   - Value ranges
   - Statistical distribution
   - Timing intervals
3. Difference should be <0.01%

## Acceptance Criteria

1. **No Data Changes**: All sensor values identical
2. **No Format Changes**: Message structures unchanged
3. **No Timing Changes**: Same update rates
4. **No Protocol Changes**: BLE/protobuf identical
5. **Performance**: <1ms additional latency
6. **Stability**: 24hr test passes

## Tools Required

1. **Logging**: Segger RTT Viewer
2. **BLE**: nRF Connect mobile app
3. **Analysis**: Python scripts for data comparison
4. **Logic Analyzer**: For SPI/interrupt timing

## Sign-off

- [ ] Developer Testing Complete
- [ ] Data Compatibility Verified
- [ ] Performance Acceptable
- [ ] Documentation Updated
- [ ] Code Review Passed
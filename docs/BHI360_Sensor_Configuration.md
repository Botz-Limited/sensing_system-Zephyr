# BHI360 Sensor Configuration Guide

## Overview
This document describes the BHI360 6-axis IMU sensor configuration used in the sensing firmware, including available sensors, maximum sampling rates, power consumption considerations, and troubleshooting findings.

## Current Firmware
**Firmware Used**: `BHI360.fw.h` (Basic firmware)
- General purpose firmware with standard sensor support
- No magnetometer required (6-axis IMU only)
- Provides Low Power variants of step counter/detector
- Successfully tested with step counting functionality

## Available Sensors and Maximum Frequencies

### Core Motion Sensors (Hardware)

| Sensor | ID | Max Frequency | Current Setting | Status | Description |
|--------|-----|--------------|-----------------|--------|-------------|
| **Quaternion (Game Rotation Vector)** | 28 | 400 Hz | 100 Hz | ✅ Working | Orientation without magnetometer |
| **Gyroscope** | 13 | 1600 Hz | 100 Hz | ✅ Working | Angular velocity (rad/s) |
| **Accelerometer** | 4 | 1600 Hz | 100 Hz | ✅ Working | Acceleration with gravity (m/s²) |

### Virtual/Computed Sensors

| Sensor | ID | Max Frequency | Current Setting | Status | Description |
|--------|-----|--------------|-----------------|--------|-------------|
| **Linear Acceleration** | 31 | 400 Hz | 100 Hz | ✅ Working | Acceleration without gravity (m/s²) |
| **Gravity Vector** | 28 | 100 Hz | 100 Hz | ✅ Working | Gravity direction (m/s²) |
| **Step Counter (Standard)** | 52 | 50 Hz | - | ❌ Not Available | Standard step counter |
| **Step Counter (Low Power)** | 136 | 50 Hz | 20 Hz | ✅ Working | Low power step counter (used) |
| **Step Detector (Standard)** | 50 | 50 Hz | - | ❌ Not Available | Standard step detector |
| **Step Detector (Low Power)** | 137 | 50 Hz | 20 Hz | ✅ Working | Low power step detector (used) |
| **Activity Recognition** | 63 | 10 Hz | 5 Hz | ❌ Not Available | Activity classification |

### Physical Sensors (Required for Virtual Sensors)

| Sensor | ID | Description | Status |
|--------|-----|-------------|--------|
| **Physical Step Counter** | 32 | Hardware step counting support | ✅ Present |
| **Physical Step Detector** | 33 | Hardware step detection support | ✅ Present |

## Activity Recognition Types
With HWActivity firmware, the following activities can be detected:
- 0: Unknown
- 1: Still
- 2: Walking
- 3: Running
- 4: On Bicycle
- 5: In Vehicle
- 6: Tilting

## Power Consumption by Sampling Rate

| Sampling Rate | Typical Current Draw | Use Case |
|---------------|---------------------|----------|
| 25 Hz | ~0.2-0.3 mA | Low power monitoring |
| 100 Hz | ~0.5-1.0 mA | **Current configuration** - Optimal for biomechanics |
| 400 Hz | ~1.5-2.0 mA | High-speed motion capture |
| 1600 Hz | ~3.0-4.0 mA | Impact/vibration analysis |

## Why 100 Hz Configuration?

The current 100 Hz sampling rate is optimal for biomechanics and sports applications because:

1. **Sufficient for Human Motion**: Captures all relevant dynamics of human movement
2. **Power Efficient**: Balances accuracy with battery life
3. **Data Management**: Generates manageable data rates for storage and transmission
4. **Bluetooth Compatible**: Can stream over BLE at reduced rates (5 Hz)
5. **FIFO Management**: Prevents buffer overflow with 10ms polling interval

## Technical Limitations

### FIFO Buffer
- Size: 2048 bytes
- At 100 Hz with all sensors: ~200ms before overflow
- Current polling rate: 10ms (prevents overflow)

### Bluetooth Streaming
- Maximum practical rate: 5-10 Hz for quaternion data
- Current implementation: 5 Hz when streaming enabled

### SD Card Logging
- All sensors logged at full 100 Hz during activities
- Lab version: Continuous logging to SD card

## Sensor Data Format

### Data Scaling (Critical for Correct Values)

| Sensor Type | Scaling Factor | Units | Notes |
|-------------|---------------|-------|-------|
| **Accelerometer/Gravity/Linear Acc** | ÷ 1000 | m/s² | LSB = 1/1000 m/s² |
| **Gyroscope** | ÷ 2900 × π/180 | rad/s | LSB = 1/2900 dps, convert to radians |
| **Quaternion** | ÷ 16384 | normalized [-1,1] | Fixed point to float |
| **Step Counter** | Raw value | steps | Direct count, handle wrap-around |

### 1-Second Debug Log Output
When activity or streaming is active, the following data is logged every second:

```
BHI360 Quat: x=-0.0563, y=0.0241, z=0.0024, w=0.9981, accuracy=0.0
BHI360 Gyro: x=0.0038, y=-0.0096, z=0.0000 rad/s
BHI360 Accel: x=1.2630, y=0.4030, z=3.8670 m/s² (with gravity)
BHI360 LinAcc: x=0.0360, y=0.2830, z=-0.0380 m/s² (gravity removed)
BHI360 Gravity: x=1.2260, y=0.1190, z=3.9060 m/s²
BHI360 Steps: total=17, activity=0
BHI360 Timestamp: 27138 ms
```

### Step Detection Events
```
Step detected (ID 137)! Total detector events: 1
Step counter raw value changed: 0 -> 4
```

## Recommendations for Different Use Cases

### Running/Gait Analysis (Current Use)
- **Quaternion**: 100 Hz ✓
- **Accelerometer**: 100 Hz ✓
- **Gyroscope**: 100 Hz ✓
- **Step Counter**: 20 Hz ✓

### High-Impact Sports
- Consider increasing accelerometer to 400 Hz for impact detection
- Gyroscope at 200-400 Hz for rapid rotations

### Long-Duration Monitoring
- Reduce all sensors to 25-50 Hz
- Disable unnecessary virtual sensors
- Focus on step counter and activity recognition

## Implementation Notes

### Sensor Availability Check
The firmware performs comprehensive checks at initialization:

1. **Physical Sensor Check**: Verifies hardware support
```cpp
bhy2_get_parameter(BHY2_PARAM_SYS_PHYS_SENSOR_PRESENT, phys_present, ...);
```

2. **Virtual Sensor Check**: Tests each variant
```cpp
if (bhy2_is_sensor_available(STEP_COUNTER_SENSOR_ID, bhy2_ptr)) {
    // Standard version
} else if (bhy2_is_sensor_available(STEP_COUNTER_LP_ID, bhy2_ptr)) {
    // Low Power version (currently used)
}
```

### Step Counter Implementation Details

#### Initialization Requirements
- Physical sensors (ID 32, 33) must be present
- Virtual sensors are checked in order: Standard → Wake Up → Low Power
- The first available variant is used

#### Step Detection Algorithm
- **Minimum Steps**: Requires 4-7 consecutive steps before counting starts
- **Initial Count**: May start at non-zero value (e.g., 4) after validation
- **Batch Updates**: Counter may jump by 2+ steps (e.g., 5→7, 14→15)
- **Detection Delay**: First few movements may not register

#### Testing Step Counter
1. **Physical Walking**: Most reliable method
2. **Manual Simulation**: 
   - Rhythmic up/down movement at 1-2 Hz
   - 20-30 cm amplitude
   - Continue for 10+ seconds

### Backward Compatibility
- If Linear Acceleration (ID 31) is not available, the system falls back to using regular Accelerometer (ID 4)
- Multiple step counter variants are checked to ensure functionality

### Calibration
The system supports runtime calibration for:
- Accelerometer: Fast Offset Compensation (FOC)
- Gyroscope: Fast Offset Compensation (FOC)
- Calibration profiles are saved and restored automatically

## Troubleshooting

### Step Counter Shows Zero
1. **Check Physical Sensors**: Verify IDs 32 & 33 are present
2. **Check Virtual Sensors**: Ensure at least one variant (52, 53, or 136) is available
3. **Movement Pattern**: Ensure rhythmic, walking-like motion
4. **Wait for Initialization**: Algorithm needs 4-7 steps to start

### Incorrect Sensor Values
1. **Gravity Not ~9.8 m/s²**: Check scaling factor (should be ÷1000, not ÷100)
2. **Gyroscope Values Too High**: Ensure conversion from dps to rad/s
3. **Step Counter Not Updating**: May be using Low Power variant with delayed updates

### Firmware Selection Guide

| Firmware | Best For | Step Counter | Notes |
|----------|----------|--------------|-------|
| **BHI360.fw.h** | General use | ✅ Low Power variants | Currently used, working well |
| **BHI360_HWActivity.fw.h** | Activity tracking | ❌ Not available in our tests | Better activity recognition |
| **BHI360_Turbo.fw.h** | High performance | Unknown | Higher power consumption |

## References
- BHI360 Datasheet: 6-axis Smart Sensor Hub
- BHY2 Sensor API Documentation
- Firmware: BHI360.fw.h (Bosch Sensortec)
- BHY2 Physical Sensor IDs: bhy2_defs.h


# Weight Calibration Complete Guide

## Overview

The weight measurement system uses 16 pressure sensors (8 per foot) to calculate a person's total weight. This guide explains the complete calibration procedure and how the system works.

## System Architecture

### 1. Primary Device Triggers Calibration
- Mobile app writes `0x01` to the weight measurement characteristic (`...b68c`)
- Control service sends `"MEASURE_WEIGHT"` command to activity metrics queue
- Primary device forwards command to secondary via D2D TX

### 2. Secondary Device Performs Measurement
- Activity metrics module receives `"MEASURE_WEIGHT"` command
- Checks if person is standing still using BHI360 motion data
- Collects pressure samples for 3 seconds while stationary
- Calculates weight using calibration parameters
- Sends result back to primary via D2D TX

### 3. Primary Device Aggregates and Reports
- Receives weight measurement from secondary
- Forwards to phone via BLE notification
- Weight appears in Information Service weight measurement characteristic

## Calibration Procedure

### What You Need
- **Known reference weight** (your actual body weight measured on a calibrated scale)
- **The sensing shoes** (both feet)
- **Mobile app** connected to primary device

### Step-by-Step Calibration

#### Step 1: Zero Calibration (No Load)
1. Remove shoes or ensure no weight is on the sensors
2. Enable debug mode: Send command `"WEIGHT_DEBUG_ON"` to activity metrics
3. Trigger measurement: Write `0x01` to weight measurement characteristic
4. Note the ADC reading from logs (e.g., "CALIBRATION DEBUG: Total ADC = 1024.0")
5. Update zero offset: Send command `"CALIBRATE_WEIGHT:ZERO:1024.0"`

#### Step 2: Scale Factor Calibration (With Known Weight)
1. Put on both shoes and stand still
2. Trigger measurement: Write `0x01` to weight measurement characteristic  
3. Note the ADC reading from logs (e.g., "CALIBRATION DEBUG: Total ADC = 5200.0")
4. Calculate scale factor:
   ```
   Scale Factor = (Your Weight in kg × 9.81) / (ADC Reading - Zero Offset)
   Example: (70 kg × 9.81) / (5200 - 1024) = 686.7 / 4176 = 0.1644
   ```
5. Update scale factor: Send command `"CALIBRATE_WEIGHT:SCALE:0.1644"`

#### Step 3: Test Calibration
1. Disable debug mode: Send command `"WEIGHT_DEBUG_OFF"`
2. Trigger measurement: Write `0x01` to weight measurement characteristic
3. Check if reported weight matches your known weight
4. Repeat calibration if needed

### Calibration Commands

Send these commands to the activity metrics message queue:

```c
// Enable/disable debug mode (shows raw ADC values)
"WEIGHT_DEBUG_ON"
"WEIGHT_DEBUG_OFF"

// Set calibration parameters
"CALIBRATE_WEIGHT:ZERO:1024.0"        // Zero offset (ADC with no load)
"CALIBRATE_WEIGHT:SCALE:0.1644"       // Scale factor (N per ADC unit)

// Advanced calibration (for non-linear sensors)
"CALIBRATE_WEIGHT:NONLINEAR_A:1.0"    // Polynomial coefficient A
"CALIBRATE_WEIGHT:NONLINEAR_B:0.0"    // Polynomial coefficient B  
"CALIBRATE_WEIGHT:NONLINEAR_C:0.0"    // Polynomial coefficient C

// View current calibration
"GET_WEIGHT_CAL"
```

## Weight Calculation Formula

### Linear Model (Default)
```c
calibrated_adc = total_adc - zero_offset
total_force_n = calibrated_adc × scale_factor
weight_kg = total_force_n / 9.81
```

### Non-Linear Model (For FSR Sensors)
```c
calibrated_adc = total_adc - zero_offset
total_force_n = A × calibrated_adc² + B × calibrated_adc + C
weight_kg = total_force_n / 9.81
```

## Calibration Data Storage

### Current Implementation
- Calibration parameters are stored in RAM in the activity metrics module
- Values are lost on power cycle
- Default values are loaded on startup

### Recommended Enhancement
- Save calibration data to LittleFS at `/lfs1/calibration/weight_calib.bin`
- Load calibration on startup
- Implement `MSG_TYPE_SAVE_WEIGHT_CALIBRATION` and `MSG_TYPE_LOAD_WEIGHT_CALIBRATION`

## Troubleshooting

### "Invalid weight calculation"
- **Cause**: Person moved during measurement or ADC reading out of range
- **Solution**: Stand perfectly still during measurement, check sensor connections

### Weight reading is 0 or negative
- **Cause**: Zero offset too high or scale factor incorrect
- **Solution**: Recalibrate zero offset with no load

### Weight reading too high/low
- **Cause**: Incorrect scale factor
- **Solution**: Recalibrate with known reference weight

### "Motion detected - resetting stable sample count"
- **Cause**: Person is moving during measurement
- **Solution**: Stand completely still for 3+ seconds

## Default Calibration Values

The system ships with these default values (suitable for FSR sensors):

```c
zero_offset = 1024.0f     // 12-bit ADC baseline
scale_factor = 0.0168f    // ~100N max per sensor (16 sensors total)
nonlinear_a = 1.0f        // Linear model
nonlinear_b = 0.0f
nonlinear_c = 0.0f
```

## Sensor Specifications

- **16 pressure sensors total** (8 per foot)
- **ADC resolution**: 12-bit (0-4095)
- **Expected range**: 20-300 kg person weight
- **Accuracy target**: ±2% after calibration
- **Measurement time**: 3 seconds while standing still

## Implementation Notes

### Message Flow
1. `Control Service` → `Activity Metrics`: `"MEASURE_WEIGHT"`
2. `Activity Metrics` → `BHI360`: Check motion status
3. `Activity Metrics` → `Foot Sensors`: Collect pressure data
4. `Activity Metrics` → `Bluetooth`: Send weight result
5. `Primary` → `Secondary`: Forward command via D2D
6. `Secondary` → `Primary`: Send weight result via D2D

### Motion Detection
- Uses BHI360 linear acceleration and gyroscope
- Acceleration threshold: 0.5 m/s²
- Gyroscope threshold: 0.1 rad/s
- Must be still for 30 consecutive samples (3 seconds at 10Hz)

### Data Averaging
- Collects 20 pressure samples while person is stationary
- Uses rolling average to reduce noise
- Rejects measurements if motion detected

## Future Enhancements

1. **Persistent Calibration Storage**
   - Save/load calibration from filesystem
   - Backup/restore calibration data

2. **Multi-Point Calibration**
   - Use multiple known weights for better accuracy
   - Automatic polynomial fitting

3. **Temperature Compensation**
   - Compensate for sensor drift with temperature
   - Requires temperature sensor integration

4. **User-Specific Calibration**
   - Store calibration per user profile
   - Automatic calibration using known user weight

5. **Calibration Validation**
   - Automatic validation with known weights
   - Warning if calibration appears incorrect

## Conclusion

Proper calibration is essential for accurate weight measurement. Follow this guide carefully and always validate your calibration with known weights. The system provides both simple linear calibration and advanced non-linear calibration for different sensor types.
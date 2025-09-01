# Weight Measurement Feature

## Overview

The activity metrics module now includes a weight measurement feature that calculates a person's weight using the 16 pressure sensors (8 in each shoe). This feature requires the person to stand still on both feet for accurate measurement.

**Module Communication**: Following the system architecture, this module communicates exclusively through message queues and events. No global functions are exposed, ensuring proper module isolation and thread safety.

## How It Works

1. **Pressure Collection**: The system collects pressure data from all 16 sensors (8 per foot)
2. **Motion Detection**: Uses BHI360 IMU data to ensure the person is standing still
3. **Data Averaging**: Collects samples over 3 seconds while the person is stationary
4. **Weight Calculation**: Converts the total pressure reading to weight in kilograms

## Implementation Details

### Key Components

- **Weight Measurement State**: Tracks the measurement process
  - `measurement_in_progress`: Boolean flag
  - `pressure_sum_buffer`: Rolling buffer of 20 samples
  - `stable_samples_required`: 30 samples (3 seconds at 10Hz)
  - `motion_threshold`: 0.5 m/s² for stillness detection

- **Motion Detection**: Uses linear acceleration and gyroscope data
  - Linear acceleration magnitude < 0.5 m/s²
  - Angular velocity magnitude < 0.1 rad/s

- **Calibration Macros**: Easy to replace after calibration
  ```c
  #define WEIGHT_CAL_ZERO_OFFSET      0.0f    // ADC reading with no load
  #define WEIGHT_CAL_SCALE_FACTOR     0.1f    // Newtons per ADC unit
  #define WEIGHT_CAL_NONLINEAR_A      1.0f    // Nonlinear coefficient A
  #define WEIGHT_CAL_NONLINEAR_B      0.0f    // Nonlinear coefficient B
  #define WEIGHT_CAL_NONLINEAR_C      0.0f    // Nonlinear coefficient C
  #define WEIGHT_CAL_TEMP_COEFF       0.0f    // Temperature coefficient (%/°C)
  #define WEIGHT_CAL_TEMP_REF         25.0f   // Reference temperature (°C)
  ```

### Message Queue Interface

The activity metrics module communicates exclusively through message queues. No global functions are exposed.

To trigger weight measurement, send a command message:
```c
generic_message_t msg = {
    .sender = SENDER_NONE,
    .type = MSG_TYPE_COMMAND,
};
strcpy(msg.data.command_str, "MEASURE_WEIGHT");
k_msgq_put(&activity_metrics_msgq, &msg, K_NO_WAIT);
```

## Usage Example

### BLE Integration

The weight measurement feature is integrated with BLE through two characteristics:

#### Control Service - Trigger Measurement
- **UUID**: `4fd5b68c-9d89-4061-92aa-319ca786baae`
- **Properties**: Write only
- **Usage**: Write `0x01` to trigger weight measurement

#### Information Service - Weight Result
- **UUID**: `0c372ec6-27eb-437e-bef4-775aefaf3c97`
- **Properties**: Read, Notify
- **Format**: `uint16_t` (weight in kg × 10 for 0.1kg precision)
- **Example**: Value `752` = 75.2 kg

The flow is:
1. Mobile app writes `0x01` to the control service weight measurement characteristic
2. Control service sends `"MEASURE_WEIGHT"` command to activity metrics queue
3. Activity metrics performs the measurement (person must stand still)
4. Result is sent as `"WEIGHT:XX.X"` command to Bluetooth queue
5. Bluetooth module calls `jis_weight_measurement_notify()` in information service
6. Mobile app receives notification with weight as `uint16_t`

### Module-to-Module Communication

Any module can request a weight measurement by sending a message:

```c
// Example from any module
generic_message_t msg = {};
msg.sender = SENDER_BTH;  // Identify the sender
msg.type = MSG_TYPE_COMMAND;
strcpy(msg.data.command_str, "MEASURE_WEIGHT");

if (k_msgq_put(&activity_metrics_msgq, &msg, K_NO_WAIT) != 0) {
    LOG_WRN("Failed to send weight measurement request");
}
```

### Receiving Weight Measurement Result

The requesting module receives the result via its message queue:

```c
// In message handler
case MSG_TYPE_COMMAND:
    if (strncmp(msg.data.command_str, "WEIGHT:", 7) == 0) {
        float weight_kg;
        if (sscanf(msg.data.command_str, "WEIGHT:%f", &weight_kg) == 1) {
            LOG_INF("Received weight measurement: %.1f kg", weight_kg);
            // Process the weight value
        }
    }
    break;
```

### During Activity Start

The measured weight is automatically used when starting an activity session:
```c
SessionHeader header = {
    // ... other fields ...
    .user_weight_kg = sensor_data.weight_measurement_valid ? 
                     (uint16_t)(sensor_data.calculated_weight_kg * 10) : 700,
    // ... other fields ...
};
```

## Calibration Requirements

For accurate weight measurement, the system needs proper calibration. The calibration values are defined as macros in `activity_metrics.cpp` for easy replacement:

### Calibration Procedure

1. **Zero Calibration** (no load on sensors)
   ```c
   // Read total ADC value with no weight
   // Update: WEIGHT_CAL_ZERO_OFFSET = measured_adc_value
   ```

2. **Linear Calibration** (single known weight)
   ```c
   // Apply known weight (e.g., 70 kg = 686.7 N)
   // Read ADC value
   // Calculate: WEIGHT_CAL_SCALE_FACTOR = 686.7 / (adc_reading - WEIGHT_CAL_ZERO_OFFSET)
   ```

3. **Multi-point Calibration** (optional, for better accuracy)
   - Apply multiple known weights (10kg, 30kg, 50kg, 70kg, 90kg)
   - Record ADC values for each
   - Use linear regression or polynomial fitting
   - Update `WEIGHT_CAL_SCALE_FACTOR` for linear fit
   - Or update `WEIGHT_CAL_NONLINEAR_A/B/C` for polynomial fit

4. **Temperature Compensation** (optional)
   - Measure at different temperatures
   - Calculate drift coefficient
   - Update `WEIGHT_CAL_TEMP_COEFF`

### Example Calibration Values

After calibration with real sensors, replace the macros:
```c
// Example for FSR sensors with 14-bit ADC
#define WEIGHT_CAL_ZERO_OFFSET      512.0f   // Typical baseline
#define WEIGHT_CAL_SCALE_FACTOR     0.0168f  // ~100N max per sensor
// For 16 sensors: 1600N max = ~163kg max weight
```

## Limitations

1. **Calibration**: Current implementation uses placeholder calibration values
2. **Sensor Distribution**: Assumes even weight distribution across sensors
3. **Temperature**: No temperature compensation implemented
4. **Sensor Type**: Calibration depends on the specific pressure sensor type (FSR, load cell, etc.)

## Future Improvements

1. **Non-linear Calibration**: Implement proper calibration curves for FSR sensors
2. **Temperature Compensation**: Add temperature sensor for drift compensation
3. **Dynamic Calibration**: Auto-calibrate using known user weight
4. **Weight History**: Store weight measurements over time
5. **BLE Characteristic**: Add dedicated BLE characteristic for weight measurement
6. **Visual Feedback**: Add LED or haptic feedback during measurement
# Pressure Sensor Weight Calibration Guide

## Overview

This guide provides detailed instructions for calibrating the pressure sensors in the sensing firmware to accurately measure weight. The system uses 16 pressure sensors (8 per foot) to calculate a person's total weight when standing still.

## Table of Contents

1. [Understanding the Calibration System](#understanding-the-calibration-system)
2. [Required Equipment](#required-equipment)
3. [Calibration Procedure](#calibration-procedure)
4. [Implementing Calibration Values](#implementing-calibration-values)
5. [Testing and Validation](#testing-and-validation)
6. [Troubleshooting](#troubleshooting)
7. [Advanced Calibration Techniques](#advanced-calibration-techniques)

## Understanding the Calibration System

### How Pressure Sensors Work

The system uses Force Sensitive Resistors (FSRs) or similar pressure sensors that change resistance based on applied force. The ADC (Analog-to-Digital Converter) reads these resistance changes as digital values.

### Calibration Parameters

The firmware uses several calibration parameters defined in `src/activity_metrics/activity_metrics.cpp`:

```c
#define WEIGHT_CAL_ZERO_OFFSET      0.0f    // ADC reading with no load
#define WEIGHT_CAL_SCALE_FACTOR     0.1f    // Newtons per ADC unit
#define WEIGHT_CAL_NONLINEAR_A      1.0f    // Nonlinear coefficient A
#define WEIGHT_CAL_NONLINEAR_B      0.0f    // Nonlinear coefficient B
#define WEIGHT_CAL_NONLINEAR_C      0.0f    // Nonlinear coefficient C
#define WEIGHT_CAL_TEMP_COEFF       0.0f    // Temperature coefficient (%/°C)
#define WEIGHT_CAL_TEMP_REF         25.0f   // Reference temperature (°C)
```

### Weight Calculation Formula

The system converts ADC readings to weight using:

1. **Linear Model**: `Force = (ADC - ZERO_OFFSET) × SCALE_FACTOR`
2. **Nonlinear Model**: `Force = A × ADC² + B × ADC + C`
3. **Weight**: `Weight_kg = Total_Force_N / 9.81`

## Required Equipment

1. **Calibration Weights**: 
   - Minimum: 10kg, 30kg, 50kg, 70kg
   - Recommended: 10kg, 20kg, 30kg, 40kg, 50kg, 60kg, 70kg, 80kg, 90kg
   - Accuracy: ±0.1kg or better

2. **Test Setup**:
   - Flat, rigid surface
   - Both sensing shoes properly positioned
   - Serial/debug connection to device
   - Temperature sensor (optional, for temperature compensation)

3. **Software Tools**:
   - Terminal for serial communication
   - Spreadsheet software for data analysis
   - Python/MATLAB for curve fitting (optional)

## Calibration Procedure

### Step 1: Zero Calibration

1. **Prepare the System**:
   ```bash
   # Ensure no weight on sensors
   # Place shoes on flat surface
   # Power on the system
   ```

2. **Create Debug Code** (temporary modification):
   ```c
   // Add to process_weight_measurement() in activity_metrics.cpp
   static bool calibration_mode = true;  // Enable for calibration
   
   if (calibration_mode) {
       // Calculate raw ADC sum
       float total_adc = 0;
       for (int i = 0; i < 8; i++) {
           total_adc += sensor_data.left_pressure[i];
           total_adc += sensor_data.right_pressure[i];
       }
       LOG_INF("CALIBRATION: Raw ADC sum = %.2f", total_adc);
   }
   ```

3. **Record Zero Reading**:
   - Let system stabilize for 30 seconds
   - Record average ADC reading with no load
   - This becomes your `WEIGHT_CAL_ZERO_OFFSET`

### Step 2: Single-Point Linear Calibration

1. **Apply Known Weight**:
   - Place a 70kg weight evenly distributed on both shoes
   - Ensure person/weight is standing still
   - Wait for stable readings

2. **Calculate Scale Factor**:
   ```
   Known_Force_N = Weight_kg × 9.81
   Scale_Factor = Known_Force_N / (ADC_reading - ZERO_OFFSET)
   
   Example:
   70kg × 9.81 = 686.7N
   If ADC_reading = 41000, ZERO_OFFSET = 1000
   Scale_Factor = 686.7 / (41000 - 1000) = 0.0172
   ```

### Step 3: Multi-Point Calibration

1. **Collect Data Points**:
   ```
   Weight(kg) | ADC_Reading | Force(N)
   -----------|-------------|----------
   0          | 1000        | 0
   10         | 6800        | 98.1
   30         | 18400       | 294.3
   50         | 29600       | 490.5
   70         | 41000       | 686.7
   90         | 52200       | 882.9
   ```

2. **Perform Linear Regression**:
   - Plot Force(N) vs (ADC - ZERO_OFFSET)
   - Calculate best-fit line
   - Slope = SCALE_FACTOR

3. **Check Linearity**:
   - Calculate R² value
   - If R² < 0.98, consider nonlinear model

### Step 4: Nonlinear Calibration (if needed)

For FSR sensors with nonlinear response:

1. **Fit Polynomial**:
   ```python
   import numpy as np
   
   # Example data
   adc_values = np.array([1000, 6800, 18400, 29600, 41000, 52200])
   force_values = np.array([0, 98.1, 294.3, 490.5, 686.7, 882.9])
   
   # Subtract zero offset
   adc_adjusted = adc_values - adc_values[0]
   
   # Fit 2nd order polynomial
   coeffs = np.polyfit(adc_adjusted, force_values, 2)
   # coeffs[0] = A, coeffs[1] = B, coeffs[2] = C
   ```

2. **Update Calibration Constants**:
   ```c
   #define WEIGHT_CAL_NONLINEAR_A  0.0000012f  // Example value
   #define WEIGHT_CAL_NONLINEAR_B  0.0168f     // Example value
   #define WEIGHT_CAL_NONLINEAR_C  0.0f        // Example value
   ```

### Step 5: Temperature Compensation (Optional)

1. **Collect Temperature Data**:
   - Measure at 15°C, 25°C, 35°C
   - Use same reference weight (e.g., 50kg)
   - Record ADC values at each temperature

2. **Calculate Temperature Coefficient**:
   ```
   Temp_Coeff = ((ADC_35C - ADC_15C) / ADC_25C) / 20°C × 100%
   
   Example:
   ADC_15C = 29000, ADC_25C = 29600, ADC_35C = 30200
   Temp_Coeff = ((30200 - 29000) / 29600) / 20 × 100 = 0.2%/°C
   ```

## Implementing Calibration Values

1. **Update Constants** in `activity_metrics.cpp`:
   ```c
   // Replace with your calibrated values
   #define WEIGHT_CAL_ZERO_OFFSET      1000.0f   // Your zero reading
   #define WEIGHT_CAL_SCALE_FACTOR     0.0172f   // Your scale factor
   #define WEIGHT_CAL_NONLINEAR_A      0.0f      // Set if using nonlinear
   #define WEIGHT_CAL_NONLINEAR_B      0.0f      // Set if using nonlinear
   #define WEIGHT_CAL_NONLINEAR_C      0.0f      // Set if using nonlinear
   #define WEIGHT_CAL_TEMP_COEFF       0.2f      // Your temp coefficient
   #define WEIGHT_CAL_TEMP_REF         25.0f     // Reference temperature
   ```

2. **Rebuild Firmware**:
   ```bash
   cd /home/ee/sensing_fw
   ./tools/build_primary.sh
   ```

3. **Flash Updated Firmware**:
   ```bash
   ./tools/flash_primary.sh
   ```

## Testing and Validation

### Accuracy Test

1. **Test Multiple Weights**:
   - Use weights not used in calibration
   - Test: 25kg, 45kg, 65kg, 85kg
   - Record measured vs actual

2. **Calculate Error**:
   ```
   Error% = |Measured - Actual| / Actual × 100%
   Target: < 2% error across range
   ```

### Repeatability Test

1. **Multiple Measurements**:
   - Same weight, 10 measurements
   - Calculate standard deviation
   - Target: σ < 0.5kg

### Distribution Test

1. **Test Uneven Loading**:
   - Weight shifted to left foot
   - Weight shifted to right foot
   - Weight on toes/heels
   - Verify consistent total

## Troubleshooting

### Common Issues

1. **Drift Over Time**:
   - Cause: Sensor aging, temperature
   - Solution: Periodic recalibration
   - Add auto-zero when no weight detected

2. **Nonlinear at Low Weights**:
   - Cause: FSR dead zone
   - Solution: Add minimum threshold
   ```c
   if (total_force_n < 10.0f) {
       total_force_n = 0.0f;  // Below detection threshold
   }
   ```

3. **Saturation at High Weights**:
   - Cause: Sensor/ADC limits
   - Solution: Check sensor specifications
   - May need different sensors for >100kg

4. **Inconsistent Readings**:
   - Cause: Poor sensor contact
   - Solution: Check mechanical mounting
   - Ensure even pressure distribution

### Debug Output

Enable detailed logging:
```c
// In calculate_weight_from_pressure()
LOG_DBG("Raw ADC sum: %.2f", avg_pressure);
LOG_DBG("After zero offset: %.2f", calibrated_adc);
LOG_DBG("Calculated force: %.2f N", total_force_n);
LOG_DBG("Final weight: %.2f kg", weight_kg);
```

## Advanced Calibration Techniques

### Per-Sensor Calibration

For highest accuracy, calibrate each sensor individually:

```c
// Define per-sensor calibration
typedef struct {
    float zero_offset;
    float scale_factor;
} sensor_cal_t;

static const sensor_cal_t left_sensor_cal[8] = {
    {1020.0f, 0.0168f},  // Sensor 0
    {1015.0f, 0.0171f},  // Sensor 1
    // ... etc
};
```

### Machine Learning Approach

For complex sensor arrangements:

1. Collect large dataset with various weights and positions
2. Train neural network or regression model
3. Export model coefficients to firmware
4. Implement inference in `calculate_weight_from_pressure()`

### Dynamic Calibration

Use known user weight for continuous calibration:

```c
// If user enters their weight in app
void update_calibration_from_known_weight(float known_weight_kg) {
    float measured_weight = calculate_weight_from_pressure();
    float correction_factor = known_weight_kg / measured_weight;
    
    // Apply exponential moving average
    static float avg_correction = 1.0f;
    avg_correction = 0.9f * avg_correction + 0.1f * correction_factor;
    
    // Update scale factor
    WEIGHT_CAL_SCALE_FACTOR *= avg_correction;
}
```

## Best Practices

1. **Document Everything**:
   - Record all calibration data
   - Note environmental conditions
   - Keep calibration certificates

2. **Regular Validation**:
   - Weekly: Quick check with known weight
   - Monthly: Full validation test
   - Yearly: Complete recalibration

3. **Production Calibration**:
   - Automate calibration process
   - Store calibration data in NVM
   - Implement per-device calibration

4. **Safety Margins**:
   - Set maximum weight limit
   - Add sanity checks
   - Implement error reporting

## Example Calibration Report

```
Calibration Report
Date: 2024-01-15
Device: SensingGR-001
Temperature: 25°C

Zero Offset: 1024.0 ADC units
Scale Factor: 0.0169 N/ADC

Validation Results:
Weight(kg) | Measured(kg) | Error(%)
-----------|--------------|----------
10.0       | 10.2         | 2.0
30.0       | 29.8         | -0.7
50.0       | 50.3         | 0.6
70.0       | 69.9         | -0.1
90.0       | 90.5         | 0.6

Average Error: 0.68%
Max Error: 2.0%
Status: PASS
```

## Conclusion

Proper calibration is essential for accurate weight measurement. Follow this guide carefully, and always validate your calibration with known weights across the expected measurement range. For production devices, consider implementing automated calibration procedures and storing calibration data persistently.
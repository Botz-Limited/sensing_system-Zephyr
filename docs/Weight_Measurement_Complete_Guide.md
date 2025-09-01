# Complete Weight Measurement Guide

## Quick Start - Simplest Way to Calibrate

### What You Need
- A known weight (yourself, 10kg weight, water bottles, etc.)
- The sensing shoes
- Serial terminal to see debug output
- 10 minutes

### PRACTICAL EXAMPLE: Using a 10kg Weight

#### Step 1: Enable Debug Output
First, temporarily modify `src/activity_metrics/activity_metrics.cpp` to see ADC values:

```c
// Add this at line ~720 in process_weight_measurement() function
// Right after "if (data_valid) {" 
if (data_valid) {
    // Sum all pressure values from both feet
    for (int i = 0; i < 8; i++) {
        total_pressure += sensor_data.left_pressure[i];
        total_pressure += sensor_data.right_pressure[i];
    }
    
    // ADD THIS DEBUG LINE:
    LOG_INF("CALIBRATION DEBUG: Total ADC = %.2f", total_pressure);
```

#### Step 2: Get Zero Offset Value
1. **Build and flash with debug enabled:**
   ```bash
   cd /home/ee/sensing_fw
   ./tools/build_primary.sh
   ./tools/flash_primary.sh
   ```

2. **Connect serial terminal:**
   ```bash
   minicom -D /dev/ttyACM0 -b 115200
   # or
   screen /dev/ttyACM0 115200
   ```

3. **Trigger measurement with NO weight:**
   - Make sure shoes are empty
   - Send weight measurement command (via BLE app or test command)
   - Look for: `CALIBRATION DEBUG: Total ADC = 1024.00`
   - **This is your ZERO_OFFSET value: 1024.0**

#### Step 3: Get Loaded ADC Value
1. **Place your 10kg weight on the shoes:**
   - Distribute evenly across both shoes
   - Make sure weight is stable

2. **Trigger measurement again:**
   - Look for: `CALIBRATION DEBUG: Total ADC = 6824.00`
   - **This is your loaded ADC value: 6824.0**

#### Step 4: Calculate Scale Factor
```
Weight in Newtons = Weight in kg × 9.81
Scale Factor = Weight in Newtons / (Loaded ADC - Zero ADC)

For 10kg weight:
Force = 10 × 9.81 = 98.1 N
Scale Factor = 98.1 / (6824 - 1024) = 98.1 / 5800 = 0.0169
```

#### Step 5: Update the Macros
Edit `src/activity_metrics/activity_metrics.cpp` (around line 780):

```c
// BEFORE:
#define WEIGHT_CAL_ZERO_OFFSET      0.0f    // ADC reading with no load
#define WEIGHT_CAL_SCALE_FACTOR     0.1f    // Newtons per ADC unit

// AFTER (with your calculated values):
#define WEIGHT_CAL_ZERO_OFFSET      1024.0f  // Your zero ADC value
#define WEIGHT_CAL_SCALE_FACTOR     0.0169f  // Your calculated scale factor
```

#### Step 6: Remove Debug Code and Rebuild
1. **Remove the debug line you added**
2. **Rebuild and flash:**
   ```bash
   ./tools/build_primary.sh
   ./tools/flash_primary.sh
   ```

#### Step 7: Test Your Calibration
- Place the 10kg weight back on - should read ~10kg
- Try yourself - should match your bathroom scale
- Try other weights to verify

### COMPLETE EXAMPLE WITH MULTIPLE WEIGHTS

If you have multiple weights (better accuracy):

#### Data Collection:
```
Weight | ADC Reading | Calculation
-------|-------------|-------------
0 kg   | 1024        | (zero offset)
10 kg  | 6824        | 98.1 / (6824-1024) = 0.0169
20 kg  | 12624       | 196.2 / (12624-1024) = 0.0169
30 kg  | 18424       | 294.3 / (18424-1024) = 0.0169
```

Average scale factor = 0.0169

### EXAMPLE: Using Your Body Weight (70kg)

1. **Get zero reading:** 
   - No weight → ADC = 1024

2. **Get your reading:**
   - You standing → ADC = 42824

3. **Calculate:**
   ```
   Your force = 70 × 9.81 = 686.7 N
   Scale = 686.7 / (42824 - 1024) = 686.7 / 41800 = 0.0164
   ```

4. **Update macros:**
   ```c
   #define WEIGHT_CAL_ZERO_OFFSET      1024.0f
   #define WEIGHT_CAL_SCALE_FACTOR     0.0164f
   ```

### EXAMPLE: Using Water Bottles (5L = 5kg)

1. **Collect data:**
   ```
   Bottles | Weight | ADC Reading
   --------|--------|------------
   0       | 0 kg   | 1024
   1       | 5 kg   | 3924
   2       | 10 kg  | 6824
   3       | 15 kg  | 9724
   4       | 20 kg  | 12624
   ```

2. **Calculate scale factor for each:**
   ```
   5kg:  49.05 / (3924-1024) = 0.0169
   10kg: 98.1 / (6824-1024) = 0.0169
   15kg: 147.15 / (9724-1024) = 0.0169
   20kg: 196.2 / (12624-1024) = 0.0169
   ```

3. **Use average: 0.0169**

### WHERE TO FIND ADC VALUES

The ADC values come from the pressure sensors. To see them:

1. **Option 1 - Temporary Debug in calculate_weight_from_pressure():**
   ```c
   // Around line 800 in activity_metrics.cpp
   float avg_pressure = 0;
   // ... existing code ...
   avg_pressure /= valid_samples;
   
   // ADD THIS:
   LOG_INF("DEBUG: Average ADC = %.2f", avg_pressure);
   ```

2. **Option 2 - Use existing logs:**
   Look for any existing pressure sensor logs that show raw values

3. **Option 3 - Create a debug command:**
   ```c
   // Add to command handler
   else if (strcmp(msg.data.command_str, "DEBUG_ADC") == 0) {
       float total = 0;
       for (int i = 0; i < 8; i++) {
           total += sensor_data.left_pressure[i];
           total += sensor_data.right_pressure[i];
       }
       LOG_INF("Current total ADC: %.2f", total);
   }
   ```

### TROUBLESHOOTING

**"I don't see any ADC values"**
- Make sure you're triggering weight measurement
- Check that serial terminal is connected
- Verify LOG_LEVEL is set to INFO or DEBUG

**"My calculated weight is way off"**
- Double-check your zero offset (should be ~500-2000 for 12-bit ADC)
- Verify weight is evenly distributed on both shoes
- Make sure you're standing still during measurement

**"Values keep changing"**
- Normal variation is ±50 ADC units
- If more, check sensor connections
- Ensure stable surface under shoes

---

## All Calibration Macros

Located in `src/activity_metrics/activity_metrics.cpp`:

```c
// Basic calibration (these are the only 2 you NEED to change)
#define WEIGHT_CAL_ZERO_OFFSET      0.0f    // ADC reading with no load
#define WEIGHT_CAL_SCALE_FACTOR     0.1f    // Newtons per ADC unit

// Advanced calibration (optional - leave as-is for linear sensors)
#define WEIGHT_CAL_NONLINEAR_A      1.0f    // Nonlinear coefficient A
#define WEIGHT_CAL_NONLINEAR_B      0.0f    // Nonlinear coefficient B  
#define WEIGHT_CAL_NONLINEAR_C      0.0f    // Nonlinear coefficient C

// Temperature compensation (optional - leave as-is if no temp sensor)
#define WEIGHT_CAL_TEMP_COEFF       0.0f    // Temperature coefficient (%/°C)
#define WEIGHT_CAL_TEMP_REF         25.0f   // Reference temperature (°C)
```

---

## How Weight Measurement Works

### System Overview
- **16 pressure sensors** (8 per foot) measure force distribution
- **Motion detection** ensures person is standing still
- **3-second averaging** for stable measurement
- **Result in kg** with 0.1kg precision

### The Process
1. User triggers measurement (via BLE or command)
2. System checks if person is standing still (using IMU data)
3. Collects pressure data for 3 seconds
4. Converts total pressure to weight using calibration
5. Sends result via BLE notification

### Weight Calculation Formula

**Simple Linear Model** (default):
```
Total ADC = Sum of all 16 sensor readings
Calibrated ADC = Total ADC - ZERO_OFFSET
Force (Newtons) = Calibrated ADC × SCALE_FACTOR
Weight (kg) = Force / 9.81
```

**Advanced Nonlinear Model** (for FSR sensors):
```
Force = A × ADC² + B × ADC + C
```

---

## Usage in the System

### Triggering Weight Measurement

**Via BLE (from mobile app):**
1. Write `0x01` to Control Service weight measurement characteristic
2. UUID: `4fd5b68c-9d89-4061-92aa-319ca786baae`

**Via Internal Message:**
```c
generic_message_t msg = {
    .sender = SENDER_NONE,
    .type = MSG_TYPE_COMMAND,
};
strcpy(msg.data.command_str, "MEASURE_WEIGHT");
k_msgq_put(&activity_metrics_msgq, &msg, K_NO_WAIT);
```

### Getting the Result

**Via BLE:**
- Information Service weight characteristic
- UUID: `0c372ec6-27eb-437e-bef4-775aefaf3c97`
- Format: `uint16_t` (weight in kg × 10)
- Example: 752 = 75.2 kg

**Via Message Queue:**
```c
// Weight result comes as "WEIGHT:XX.X" command
if (strncmp(msg.data.command_str, "WEIGHT:", 7) == 0) {
    float weight_kg;
    sscanf(msg.data.command_str, "WEIGHT:%f", &weight_kg);
    // Use weight_kg
}
```

---

## Calibration Without Weights - Alternative Methods

### Method 1: Using People (Recommended)
```
1. Find 3 people who know their weight
2. Have each stand on the sensors
3. Record: Person A: 65kg = 38000 ADC
          Person B: 75kg = 43000 ADC  
          Person C: 85kg = 48000 ADC
4. Calculate average scale factor
```

### Method 2: Water Bottles
```
1L water = 1kg
Use 5L bottles (5kg each)
Stack them: 5kg, 10kg, 15kg, 20kg...
```

### Method 3: Single Point (Quickest)
```
Just use yourself:
1. Zero cal: No load = 1000 ADC
2. Your weight: 70kg = 41000 ADC
3. Scale = 686.7N / 40000 = 0.0172
Done!
```

---

## Debug and Testing

### Enable Debug Output
Add to `process_weight_measurement()`:
```c
LOG_INF("Raw ADC sum: %.2f", avg_pressure);
LOG_INF("After zero: %.2f", calibrated_adc);
LOG_INF("Force: %.2f N", total_force_n);
LOG_INF("Weight: %.2f kg", weight_kg);
```

### Test Commands
```bash
# Monitor logs
minicom -D /dev/ttyACM0 -b 115200

# Look for:
# "Weight measurement complete: XX.X kg"
```

### Validation Checklist
- [ ] Zero reading stable (±50 ADC)
- [ ] Weight within ±3kg of actual
- [ ] Consistent readings (±0.5kg)
- [ ] Works with weight on one foot
- [ ] Rejects measurement if moving

---

## Common Issues and Solutions

### "Invalid weight calculation"
- Person moved during measurement
- Solution: Stand still for 5 seconds

### Reading too high/low
- Wrong scale factor
- Solution: Recalibrate with known weight

### Drift over time
- Sensor aging or temperature
- Solution: Re-zero when no load detected

### Different readings each time
- Poor sensor contact
- Solution: Check mechanical mounting

---

## Production Recommendations

### Basic Setup
```c
// Reasonable defaults for FSR sensors
#define WEIGHT_CAL_ZERO_OFFSET    1024.0f   // Typical 12-bit ADC baseline
#define WEIGHT_CAL_SCALE_FACTOR   0.0168f   // ~100N max per sensor
```

### User Calibration
Let users calibrate in the app:
1. User enters their known weight
2. App triggers measurement
3. Calculate personal scale factor
4. Store in device

### Accuracy Expectations
- With proper calibration: ±2%
- With single-point cal: ±5%
- With defaults only: ±10-15%

---

## Example Values for Different Sensors

### FSR (Force Sensitive Resistor)
```c
#define WEIGHT_CAL_ZERO_OFFSET    1000.0f
#define WEIGHT_CAL_SCALE_FACTOR   0.017f
// Nonlinear usually needed
#define WEIGHT_CAL_NONLINEAR_A    0.0000012f
#define WEIGHT_CAL_NONLINEAR_B    0.0168f
```

### Load Cell
```c
#define WEIGHT_CAL_ZERO_OFFSET    2048.0f  // Centered at mid-range
#define WEIGHT_CAL_SCALE_FACTOR   0.025f   // More linear
// Linear model usually sufficient
```

### Capacitive
```c
#define WEIGHT_CAL_ZERO_OFFSET    500.0f
#define WEIGHT_CAL_SCALE_FACTOR   0.012f
```

---

## Summary - Minimum Steps to Get Working

1. **Find your zero offset:**
   - No weight on shoes
   - Check logs for ADC value
   - Update `WEIGHT_CAL_ZERO_OFFSET`

2. **Find your scale factor:**
   - Stand on shoes (know your weight)
   - Calculate: Scale = (Weight×9.81) / (ADC-Zero)
   - Update `WEIGHT_CAL_SCALE_FACTOR`

3. **Rebuild and flash:**
   ```bash
   ./tools/build_primary.sh
   ./tools/flash_primary.sh
   ```

4. **Test:**
   - Trigger measurement via BLE
   - Check if result is within ±3kg

That's all you need for basic functionality!
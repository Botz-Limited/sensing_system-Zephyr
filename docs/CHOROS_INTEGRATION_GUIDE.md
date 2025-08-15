# Choros Algorithm Integration Guide

## Architecture Overview

Your Choros algorithms are integrated into our system at the **analytics module** level, receiving consolidated sensor data at 100-200Hz:

```
Sensors (80-100Hz) → sensor_data → analytics (+ CHOROS BUFFER) → BLE Updates
                                           ↓
                                    [Your algorithms here]
```

## Buffer Implementation

### Structure (Choros-Compatible)
Location: `include/choros_buffer.h`

```c
typedef struct {
    // Your IMUData format exactly
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
    float timestamp;                // Seconds as float
    float insole_pressures[8];      // NOW WITH REAL RAW SENSOR DATA
    float battery;
    uint8_t foot;                   // 0=Left, 1=Right
    // ... rest of your fields
} choros_imu_data_t;

// Ring buffer: 600 samples (3 sec @ 200Hz)
choros_imu_data_t buffer[600];
```

### CRITICAL: Buffer Contains BOTH Feet (Primary Device)

**On PRIMARY device, the buffer stores samples from BOTH feet in mixed sequence:**
```
Buffer[0]: LEFT foot (from D2D),  timestamp 1.000, foot=0, pressure=[120,145,89...]
Buffer[1]: RIGHT foot (local),    timestamp 1.001, foot=1, pressure=[98,167,134...]
Buffer[2]: LEFT foot (from D2D),  timestamp 1.010, foot=0, pressure=[125,148,91...]
Buffer[3]: RIGHT foot (local),    timestamp 1.011, foot=1, pressure=[101,170,138...]
```

**Data Quality:** The buffer now contains:
- ✅ **REAL raw pressure sensor values** (not synthesized)
- ✅ **REAL timestamps** in milliseconds
- ✅ **REAL IMU data** (accelerometer, gyroscope)
- ✅ **Both feet data** on primary device

## Currently Integrated Functions

We replaced these placeholder functions with your algorithms:

| Function | Location | Your Algorithm Used |
|----------|----------|-------------------|
| Ground Contact Time | `realtime_metrics.cpp:290` | IC/TO event detection |
| Stride Length | `analytics.cpp:320` | Velocity integration |
| Pronation Angle | `analytics.cpp:324` | Gyro integration at IC |

## How to Add New Algorithms

### Step 1: Add Function Declaration
In `include/choros_buffer.h`:
```c
float choros_your_new_metric(void);
```

### Step 2: Implement in choros_buffer_impl.c
```c
float choros_your_new_metric(void) {
    // Access the ring buffer
    for (int i = 0; i < choros_buffer.count; i++) {
        choros_imu_data_t* sample = &choros_buffer.samples[i];
        
        // Your algorithm using standard IMUData fields
        float pressure_sum = 0;
        for (int j = 0; j < 8; j++) {
            pressure_sum += sample->insole_pressures[j];
        }
        
        // Use timestamp for timing
        float dt = sample[i+1].timestamp - sample[i].timestamp;
        
        // Return your calculated metric
    }
}
```

### Step 3: Replace Our Calculation
Find where we calculate the metric and replace:

```c
// OLD: Our placeholder
float metric = calculate_placeholder();

// NEW: Your algorithm with fallback
float metric = choros_your_new_metric();
if (metric == 0) metric = calculate_placeholder(); // Safety fallback
```

## Key Functions You Can Replace

| Our Function | Location | Replace With |
|--------------|----------|-------------|
| `calculate_fatigue_index_placeholder()` | `analytics.cpp:414` | Your fatigue algorithm |
| `calculate_injury_risk_placeholder()` | `analytics.cpp:425` | Your injury detection |
| `calculate_vertical_stiffness_placeholder()` | `analytics.cpp:464` | Your stiffness calc |
| `calculate_recovery_score_placeholder()` | `analytics.cpp:475` | Your recovery metric |

## Important Notes

1. **Buffer is continuously updated** - Always has last 3 seconds of data
2. **Real raw data available** - Pressure arrays contain actual sensor ADC values
3. **Use fallbacks** - Return 0 or -1 if calculation fails
4. **BOTH feet in buffer on Primary** - Buffer contains mixed L/R samples with `foot` field
5. **Secondary has only Left** - Secondary device buffer only contains left foot
6. **Minimum samples required** - Most algorithms need 100+ samples before calculating

## Direct Algorithm Replacement

Your colleague can directly replace any function because:
1. **Exact buffer structure** - Matches `choros::core::IMUData` format
2. **Real sensor data** - Raw pressure values, not processed metrics
3. **Proper timestamps** - Millisecond precision for timing calculations
4. **Bilateral support** - Both feet in same buffer on primary
5. **Safe wrappers** - Handle buffer state checking automatically

Example replacement:
```c
// OLD placeholder:
float calculate_stride_length() {
    return 1.2f; // Fixed placeholder
}

// NEW Choros replacement (no other changes needed):
float calculate_stride_length() {
    // Direct access to choros_buffer with real data
    return choros_get_stride_length_safe(1.2f);
}
```

## Data Access Pattern

```c
// Get most recent samples - Now handles BOTH feet
int start = (choros_buffer.write_idx - 100 + 600) % 600;
for (int i = 0; i < 100; i++) {
    int idx = (start + i) % 600;
    choros_imu_data_t* sample = &choros_buffer.samples[idx];
    
    // Check which foot this sample is from
    if (sample->foot == 0) {  // LEFT foot
        // Process left foot data
    } else if (sample->foot == 1) {  // RIGHT foot
        // Process right foot data
    }
}
```

## Bilateral Algorithm Example

```c
// Calculate symmetry between feet
float calculate_bilateral_symmetry() {
    float left_gct_sum = 0, right_gct_sum = 0;
    int left_count = 0, right_count = 0;
    
    for (int i = 0; i < choros_buffer.count; i++) {
        choros_imu_data_t* sample = &choros_buffer.samples[i];
        
        if (sample->foot == 0) {  // LEFT
            // Process left foot metrics
            left_count++;
        } else if (sample->foot == 1) {  // RIGHT
            // Process right foot metrics
            right_count++;
        }
    }
    
    // Both feet data available in same buffer!
    return calculate_symmetry(left_metrics, right_metrics);
}
```

## Testing Your Algorithms

1. Enable debug logging:
```c
LOG_INF("Your metric: %.2f", result);
```

2. Check buffer health:
```c
if (choros_buffer_is_healthy()) {
    // Buffer has valid data
}
```

3. Monitor sample rate:
```c
choros_buffer_print_stats(); // Logs buffer statistics
```

## Contact

Buffer implementation: `src/analytics/choros_buffer_impl.c`
Questions: Check existing implementations for examples
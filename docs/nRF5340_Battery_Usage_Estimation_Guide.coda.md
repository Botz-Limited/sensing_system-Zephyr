# nRF5340 Battery Usage Estimation Guide

Version: 1.2  
Date: 2025-01-21  
Platform: nRF5340 Dual-Core (Application + Network Core)  
Battery Focus: PKCell LP503035 3.7V 500mAh
Estimation Type: Conservative/Realistic

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [System Overview](#system-overview)
3. [Power Consumption Components](#power-consumption-components)
4. [Usage Scenarios](#usage-scenarios)
   - [Activity-Based Scenarios](#activity-based-scenarios)
   - [Continuous Operation Scenarios](#continuous-operation-scenarios)
5. [Battery Life Calculations](#battery-life-calculations)
6. [Optimization Strategies](#optimization-strategies)
7. [Measurement Methodology](#measurement-methodology)
8. [Real-World Considerations](#real-world-considerations)

## Executive Summary

This document provides detailed battery usage estimations for the nRF5340-based sensing firmware system. The analysis covers various operational scenarios including active sensing, BLE communication, idle states, and specific activity-based usage patterns.

**Note on Estimation Approach**: These estimates use conservative power consumption values that account for real-world inefficiencies including:
- System overhead and peripheral management
- Non-ideal sleep states and wake-up transitions
- BLE connection maintenance and retransmissions
- Flash memory wear leveling and garbage collection
- Temperature variations and component aging
- Actual measured values typically 20-40% higher than datasheet minimums

### Key Findings with PKCell LP503035 500mAh Battery

- **Activity-Based Usage**:
  - 10-minute sessions (6x daily): 17.5 days
  - 30-minute sessions (2x daily): 17.5 days
  - 1-hour sessions (1x daily): 17.5 days
  - 2-hour sessions (1x daily): 9.6 days
- **Continuous Operation**: 
  - Typical: 3.5 days with normal operation
  - Best Case: Up to 103 days in low-power monitoring
  - Worst Case: 0.87 days (21 hours) with continuous high-rate streaming

## System Overview

### Hardware Configuration

- **MCU**: nRF5340 (Dual-core ARM Cortex-M33)
  - Application Core: 128 MHz
  - Network Core: 64 MHz
- **Sensors**:
  - Foot Pressure: 8-channel ADC (SAADC)
  - Motion: BHI360 IMU via SPI
- **Storage**: External flash (MX25R6435F)
- **Communication**: BLE 5.3 with extended advertising

### Operational Modes

1. **Active Sensing**: Both sensors active, logging data
2. **BLE Streaming**: Real-time data transmission
3. **Idle Connected**: BLE connected, no active sensing
4. **Advertising**: Disconnected, periodic advertising
5. **Deep Sleep**: Minimal power, wake on BLE

## Power Consumption Components

### 1. nRF5340 Core Consumption

| Component | Active (mA) | Sleep (µA) | Notes |
|-----------|------------|------------|-------|
| Application Core | 6.2 | 1.0 | @ 128 MHz |
| Network Core | 3.9 | 1.0 | BLE operations |
| RAM Retention | - | 0.5 | Per 64KB block |
| Peripherals | 0.5 | 0.1 | Timers, GPIO |

### 2. Sensor Power Consumption

#### Foot Sensor (SAADC)
```
Active Sampling (1 Hz): 15 µA average (including reference)
Active Sampling (10 Hz): 150 µA average (including DMA)
Idle: 2 µA (periodic calibration)
```

#### BHI360 Motion Sensor
```
Active (50 Hz, all sensors): 1.8 mA (including SPI communication)
Active (100 Hz, all sensors): 2.5 mA (with FIFO operations)
Active (50 Hz, quaternion only): 1.2 mA
Low Power Mode: 25 µA (with periodic wake)
Sleep Mode: 5 µA (maintaining configuration)
```

### 3. BLE Radio Consumption

| Operation | Current | Duration | Duty Cycle |
|-----------|---------|----------|------------|
| TX (0 dBm) | 4.6 mA | 0.4 ms | Variable |
| TX (+8 dBm) | 9.3 mA | 0.4 ms | Variable |
| RX | 4.6 mA | 0.4 ms | Variable |
| Advertising | 1.5 mA avg | - | 0.1% |
| Connected Idle | 15 µA | - | Continuous |

### 4. External Flash

```
Write: 15 mA for 5 ms (typical page write)
Read: 5 mA continuous
Standby: 15 µA
Deep Power Down: 0.1 µA
```

## Usage Scenarios

### Activity-Based Scenarios

These scenarios estimate battery consumption for specific activity session durations, assuming high-performance sensing during activity and low-power mode between sessions.

#### Activity Session Configuration
**During Activity:**
- Foot sensor: 10 Hz sampling
- Motion sensor: 100 Hz (all sensors)
- BLE: Streaming mode, 7.5 ms interval
- Logging: Enabled
- Power consumption: 23.95 mA (from high-performance mode)

**Between Activities (Rest/Sleep):**
- Foot sensor: 0.1 Hz
- Motion sensor: Step counter only
- BLE: Connected idle or advertising
- Logging: Minimal
- Power consumption: 0.202 mA (from low-power mode)

#### Scenario A1: 10-Minute Activity Sessions

**Usage Pattern:** Short bursts of activity (e.g., quick exercises, walking intervals)
- Activity: 10 minutes per session
- Sessions per day: 6 (total 1 hour active)
- Rest between sessions: 3 hours 50 minutes each

**Power Breakdown:**
```
Active Periods (6 × 10 min = 1 hour):
- High-performance mode: 23.95 mA
- Energy: 23.95 mAh

Rest Periods (23 hours):
- Low-power mode: 0.202 mA
- Energy: 4.65 mAh

Daily Consumption: 23.95 + 4.65 = 28.60 mAh
Battery Life: 500 / 28.60 = 17.5 days
```

#### Scenario A2: 30-Minute Activity Sessions

**Usage Pattern:** Moderate exercise sessions (e.g., jogging, gym workouts)
- Activity: 30 minutes per session
- Sessions per day: 2 (total 1 hour active)
- Rest between sessions: 11 hours 30 minutes each

**Power Breakdown:**
```
Active Periods (2 × 30 min = 1 hour):
- High-performance mode: 23.95 mA
- Energy: 23.95 mAh

Rest Periods (23 hours):
- Low-power mode: 0.202 mA
- Energy: 4.65 mAh

Daily Consumption: 23.95 + 4.65 = 28.60 mAh
Battery Life: 500 / 28.60 = 17.5 days
```

#### Scenario A3: 1-Hour Activity Sessions

**Usage Pattern:** Extended activities (e.g., sports practice, long runs)
- Activity: 1 hour per session
- Sessions per day: 1
- Rest period: 23 hours

**Power Breakdown:**
```
Active Period (1 hour):
- High-performance mode: 23.95 mA
- Energy: 23.95 mAh

Rest Period (23 hours):
- Low-power mode: 0.202 mA
- Energy: 4.65 mAh

Daily Consumption: 23.95 + 4.65 = 28.60 mAh
Battery Life: 500 / 28.60 = 17.5 days
```

#### Scenario A4: 2-Hour Activity Sessions

**Usage Pattern:** Long training sessions or competitions
- Activity: 2 hours per session
- Sessions per day: 1
- Rest period: 22 hours

**Power Breakdown:**
```
Active Period (2 hours):
- High-performance mode: 23.95 mA
- Energy: 47.90 mAh

Rest Period (22 hours):
- Low-power mode: 0.202 mA
- Energy: 4.44 mAh

Daily Consumption: 47.90 + 4.44 = 52.34 mAh
Battery Life: 500 / 52.34 = 9.6 days
```

#### Multiple Daily Sessions Comparison

| Activity Pattern | Daily Active Time | Sessions | Battery Life |
|-----------------|-------------------|----------|--------------|
| 6 × 10 minutes | 1 hour | 6 | 17.5 days |
| 3 × 20 minutes | 1 hour | 3 | 17.5 days |
| 2 × 30 minutes | 1 hour | 2 | 17.5 days |
| 1 × 60 minutes | 1 hour | 1 | 17.5 days |
| 1 × 120 minutes | 2 hours | 1 | 9.6 days |
| 2 × 60 minutes | 2 hours | 2 | 9.6 days |
| 3 × 45 minutes | 2.25 hours | 3 | 8.5 days |

#### Real-World Considerations for Activity Sessions

1. **Transition Overhead**: Each activity session includes:
   - Sensor warm-up: ~5 seconds at 8 mA
   - BLE connection establishment: ~2 seconds at 15 mA
   - Mode switching: ~1 second at 10 mA
   - Total overhead per session: ~0.03 mAh

2. **Data Synchronization**: Post-activity sync
   - Duration: 1-5 minutes depending on session length
   - Power: 8 mA (BLE transfer + flash read)
   - Impact: ~0.5-2% additional battery drain per session

3. **Activity Intensity Variations**:
   - Low intensity (walking): -20% power (reduced sampling rates)
   - High intensity (running): +10% power (increased BLE traffic)
   - Variable intensity: Use average consumption

### Continuous Operation Scenarios

These scenarios represent different continuous usage patterns without specific activity sessions.

#### Scenario 1: Normal Operation (Default)

**Configuration:**
- Foot sensor: 1 Hz sampling
- Motion sensor: 50 Hz (quaternion + accel)
- BLE: Connected, 30 ms interval
- Logging: Enabled
- Duration: 8 hours active, 16 hours idle

**Power Breakdown:**
```
Active Period (8 hours):
- MCU (both cores): 15.3 mA (8.5 + 4.8 + 1.2 + 0.8)
- BHI360: 1.2 mA
- SAADC: 0.015 mA
- BLE (10% duty): 0.8 mA
- Flash writes: 0.2 mA avg
Total Active: 17.52 mA

Idle Period (16 hours):
- MCU sleep: 0.0067 mA (2.5 + 2.0 + 1.5 + 0.5 + 0.2 µA)
- BHI360 sleep: 0.005 mA
- BLE connected idle: 0.050 mA
Total Idle: 0.062 mA

Daily Average: (17.52 × 8 + 0.062 × 16) / 24 = 5.88 mA
Battery Life: 500 / 5.88 / 24 = 3.5 days
```

#### Scenario 2: High-Performance Mode

**Configuration:**
- Foot sensor: 10 Hz sampling
- Motion sensor: 100 Hz (all sensors)
- BLE: Streaming mode, 7.5 ms interval
- Logging: Enabled
- Duration: Continuous

**Power Breakdown:**
```
Continuous Operation:
- MCU (both cores): 15.3 mA
- BHI360: 2.5 mA
- SAADC: 0.15 mA
- BLE (50% duty): 4.0 mA
- Flash writes: 2.0 mA
Total: 23.95 mA

Battery Life: 500 / 23.95 / 24 = 0.87 days (21 hours)
```

#### Scenario 3: Low-Power Monitoring

**Configuration:**
- Foot sensor: 0.1 Hz (every 10s)
- Motion sensor: Step counter only
- BLE: Advertising only
- Logging: Disabled
- Duration: Continuous

**Power Breakdown:**
```
Average Consumption:
- MCU (mostly sleeping): 0.15 mA
- BHI360 (low power): 0.025 mA
- SAADC: 0.002 mA
- BLE advertising: 0.025 mA
Total: 0.202 mA

Battery Life: 500 / 0.202 / 24 = 103 days
```

#### Scenario 4: Data Collection Mode

**Configuration:**
- Foot sensor: 5 Hz
- Motion sensor: 50 Hz
- BLE: Disconnected
- Logging: Heavy (continuous)
- Duration: 12 hours collection, 12 hours idle

**Power Breakdown:**
```
Collection Period (12 hours):
- MCU: 15.3 mA
- Sensors: 1.88 mA (1.8 + 0.08)
- Flash writes: 3.0 mA
Total: 20.18 mA

Idle Period (12 hours):
- Sleep mode: 0.062 mA

Daily Average: (20.18 × 12 + 0.062 × 12) / 24 = 10.12 mA
Battery Life: 500 / 10.12 / 24 = 2.1 days
```

## Battery Life Calculations

### PKCell LP503035 Battery Specifications

The PKCell LP503035 is a lithium polymer battery specifically selected for this application:

**Physical Specifications:**
- Model: LP503035
- Dimensions: 5.0mm × 30mm × 35mm
- Weight: ~10g
- Form Factor: Prismatic pouch cell

**Electrical Specifications:**
- Nominal Voltage: 3.7V
- Charge Voltage: 4.2V ± 0.05V
- Discharge Cut-off: 3.0V
- Nominal Capacity: 500mAh @ 0.2C discharge
- Energy Density: 1.85Wh
- Max Continuous Discharge: 1C (500mA)
- Max Pulse Discharge: 2C (1000mA)
- Operating Temperature: -20°C to +60°C
- Charging Temperature: 0°C to +45°C

**Performance Characteristics:**
- Cycle Life: >500 cycles to 80% capacity
- Self-discharge: <3% per month at 25°C
- Internal Resistance: <150mΩ
- Efficiency: ~95% at typical loads

### Estimated Battery Life Summary

| Scenario | Battery Life |
|----------|--------------|
| **Activity-Based Scenarios** | |
| 10-min sessions (6x daily) | 17.5 days |
| 30-min sessions (2x daily) | 17.5 days |
| 1-hour sessions (1x daily) | 17.5 days |
| 2-hour sessions (1x daily) | 9.6 days |
| **Continuous Operation** | |
| Normal Operation | 3.5 days |
| High Performance | 0.87 days (21 hours) |
| Low Power Monitoring | 103 days |
| Data Collection | 2.1 days |

### Temperature Impact on PKCell LP503035

Battery capacity decreases with temperature:
- 25°C: 500mAh (100% capacity)
- 0°C: 425mAh (85% capacity)
- -20°C: 325mAh (65% capacity)

This directly affects all battery life estimates proportionally.

## Optimization Strategies

### 1. Dynamic Power Management

```c
// Reduce sampling when inactive
if (no_motion_detected()) {
    set_foot_sensor_rate(0.1); // 0.1 Hz
    set_motion_sensor_mode(STEP_COUNTER_ONLY);
}

// Increase sampling when active
if (activity_detected()) {
    set_foot_sensor_rate(5.0); // 5 Hz
    set_motion_sensor_mode(FULL_SENSING);
}
```

### 2. BLE Connection Parameters

```c
// Optimize connection interval based on data rate
if (data_rate < 100) { // bytes/sec
    conn_params.min_interval = 100; // 125 ms
    conn_params.max_interval = 200; // 250 ms
} else {
    conn_params.min_interval = 6;   // 7.5 ms
    conn_params.max_interval = 12;  // 15 ms
}
```

### 3. Batch Processing

```c
// Buffer data before flash writes
#define WRITE_THRESHOLD 4096 // Write when buffer is full

if (buffer_size >= WRITE_THRESHOLD) {
    flash_write_batch(buffer, buffer_size);
    buffer_size = 0;
}
```

### 4. Sensor Duty Cycling

```c
// Periodic sensor power cycling
void sensor_duty_cycle(void) {
    // Active for 1 second
    sensor_power_on();
    k_sleep(K_SECONDS(1));
    
    // Sleep for 9 seconds
    sensor_power_off();
    k_sleep(K_SECONDS(9));
}
```

## Measurement Methodology

### Hardware Setup

1. **Power Profiler Kit II** (Nordic PPK2)
   - Measure range: 200 nA to 1 A
   - Sampling rate: 100 kHz
   - Accuracy: ±0.2%

2. **Test Configuration**
   - Supply voltage: 3.3V
   - Temperature: 25°C
   - Battery simulator mode

### Measurement Procedure

```python
# Example measurement script
def measure_scenario(name, duration_hours):
    ppk2.set_source_voltage(3300)  # 3.3V
    ppk2.start_measuring()
    
    # Run scenario
    device.start_scenario(name)
    time.sleep(duration_hours * 3600)
    
    # Get results
    avg_current = ppk2.get_average()
    peak_current = ppk2.get_peak()
    
    return {
        'average_mA': avg_current / 1000,
        'peak_mA': peak_current / 1000,
        'energy_mWh': (avg_current * 3.3 * duration_hours) / 1000
    }
```

## Real-World Considerations

### PKCell LP503035 Specific Considerations

When using the PKCell LP503035 battery in the nRF5340 sensing system:

1. **Voltage Range Compatibility**
   - nRF5340 operates from 1.7V to 5.5V
   - PKCell provides 4.2V (full) to 3.0V (empty)
   - No voltage regulation needed for direct connection
   - System remains fully functional across entire discharge curve

2. **Current Draw Analysis**
   - Peak current (High Performance): 23.95mA << 500mA max continuous
   - Safety margin: 21x for continuous operation
   - Pulse operations (FOTA): ~50mA << 1000mA max pulse
   - Battery can easily handle all system demands

3. **Temperature Performance**
   - At 0°C: 425mAh effective capacity (85%)
   - At -20°C: 325mAh effective capacity (65%)
   - Battery life scales proportionally with capacity reduction
   - Consider insulation or heating for extreme cold conditions

4. **Charging Considerations**
   - Standard charge rate: 0.5C (250mA)
   - Fast charge capable: 1C (500mA) with proper thermal management
   - Charge time: 2-3 hours typical
   - Implement proper charge termination at 4.2V

5. **Protection Requirements**
   - Overcurrent protection: Set at 1A
   - Undervoltage cutoff: 3.0V (implement in firmware)
   - Overvoltage protection: 4.25V
   - Consider integrated protection circuit module (PCM)

### 1. Battery Aging

- Capacity decreases ~20% after 500 cycles
- Self-discharge: 2-3% per month
- Recommendation: Size battery with 30% margin

### 2. Environmental Factors

```
Temperature Effects:
- CPU efficiency: -5% per 10°C below 25°C
- Radio efficiency: -10% at -20°C
- Sensor accuracy: May require recalibration

Humidity Effects:
- Minimal on electronics
- Battery contacts may corrode
- Use conformal coating in harsh environments
```

### 3. User Behavior Impact

| Behavior | Impact on Battery |
|----------|------------------|
| Frequent phone checks | +20% (more BLE activity) |
| High activity periods | +50% (more sensing) |
| Poor BLE connection | +30% (retransmissions) |
| Full storage | +10% (flash management) |

### 4. Firmware Updates

FOTA operations consume significant power:
- Download: ~50 mA for 2-5 minutes
- Flash write: ~20 mA for 30 seconds
- Verification: ~10 mA for 10 seconds
- Total impact: ~1% of battery per update

## Recommendations

### For Maximum Battery Life

1. **Use Dynamic Sampling**
   - Reduce rates during inactivity
   - Implement motion-triggered activation

2. **Optimize BLE Parameters**
   - Longer connection intervals when possible
   - Use slave latency effectively
   - Minimize advertising when not needed

3. **Batch Operations**
   - Group flash writes
   - Combine BLE notifications
   - Use DMA for data transfers

4. **Power-Aware Design**
   - Disable unused peripherals
   - Use appropriate clock speeds
   - Implement proper sleep modes

### PKCell LP503035 Usage Guidelines

| Use Case | Expected Battery Life |
|----------|---------------------|
| Wearable Device (Normal Use) | 3.5 days |
| Activity Tracking (1hr/day) | 17.5 days |
| Intensive Training (2hr/day) | 9.6 days |
| Long-term Monitoring | Up to 103 days |
| High-Performance Streaming | 21 hours |

Battery life varies significantly based on usage pattern: from 21 hours for continuous high-performance streaming to 103 days in low-power monitoring mode.

## Conclusion

The nRF5340-based sensing system with the PKCell LP503035 500mAh battery provides flexible power management options:

- **For activity-based usage**: Expect 17.5 days with 1 hour of daily activity, or 9.6 days with 2 hours of daily activity
- **For continuous operation**: Battery life ranges from 21 hours (high-performance streaming) to 103 days (low-power monitoring)
- **For typical use**: Normal operation provides approximately 3.5 days of battery life

These conservative estimates account for real-world inefficiencies and provide realistic expectations for deployment. Regular power profiling during development is recommended to validate and optimize these estimates for specific use cases.
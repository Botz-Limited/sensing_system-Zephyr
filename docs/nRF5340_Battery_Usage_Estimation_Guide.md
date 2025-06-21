# nRF5340 Battery Usage Estimation Guide

Version: 1.0  
Date: 2025-01-21  
Platform: nRF5340 Dual-Core (Application + Network Core)

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

### Key Findings

- **Activity-Based Usage (500mAh battery)**:
  - 10-minute sessions (6x daily): 32.4 days
  - 30-minute sessions (2x daily): 32.4 days
  - 1-hour sessions (1x daily): 32.4 days
  - 2-hour sessions (1x daily): 17.9 days
- **Continuous Operation**: 
  - Typical: 7-10 days with normal operation
  - Best Case: Up to 165 days in low-power monitoring
  - Worst Case: 1.7 days with continuous high-rate streaming

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
| Application Core | 4.6 | 1.0 | @ 64 MHz |
| Network Core | 3.9 | 1.0 | BLE operations |
| RAM Retention | - | 0.5 | Per 64KB block |
| Peripherals | 0.5 | 0.1 | Timers, GPIO |

### 2. Sensor Power Consumption

#### Foot Sensor (SAADC)
```
Active Sampling (1 Hz): 5 µA average
Active Sampling (10 Hz): 50 µA average
Idle: 0.5 µA
```

#### BHI360 Motion Sensor
```
Active (50 Hz, all sensors): 1.2 mA
Active (50 Hz, quaternion only): 0.8 mA
Low Power Mode: 10 µA
Sleep Mode: 1 µA
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
- Power consumption: 12.55 mA (from high-performance mode)

**Between Activities (Rest/Sleep):**
- Foot sensor: 0.1 Hz
- Motion sensor: Step counter only
- BLE: Connected idle or advertising
- Logging: Minimal
- Power consumption: 0.126 mA (from low-power mode)

#### Scenario A1: 10-Minute Activity Sessions

**Usage Pattern:** Short bursts of activity (e.g., quick exercises, walking intervals)
- Activity: 10 minutes per session
- Sessions per day: 6 (total 1 hour active)
- Rest between sessions: 3 hours 50 minutes each

**Power Breakdown:**
```
Active Periods (6 × 10 min = 1 hour):
- High-performance mode: 12.55 mA
- Energy: 12.55 mAh

Rest Periods (23 hours):
- Low-power mode: 0.126 mA
- Energy: 2.90 mAh

Daily Consumption: 12.55 + 2.90 = 15.45 mAh
Battery Life (500mAh): 500 / 15.45 = 32.4 days
Battery Life (1800mAh): 1800 / 15.45 = 116.5 days
```

#### Scenario A2: 30-Minute Activity Sessions

**Usage Pattern:** Moderate exercise sessions (e.g., jogging, gym workouts)
- Activity: 30 minutes per session
- Sessions per day: 2 (total 1 hour active)
- Rest between sessions: 11 hours 30 minutes each

**Power Breakdown:**
```
Active Periods (2 × 30 min = 1 hour):
- High-performance mode: 12.55 mA
- Energy: 12.55 mAh

Rest Periods (23 hours):
- Low-power mode: 0.126 mA
- Energy: 2.90 mAh

Daily Consumption: 12.55 + 2.90 = 15.45 mAh
Battery Life (500mAh): 500 / 15.45 = 32.4 days
Battery Life (1800mAh): 1800 / 15.45 = 116.5 days
```

#### Scenario A3: 1-Hour Activity Sessions

**Usage Pattern:** Extended activities (e.g., sports practice, long runs)
- Activity: 1 hour per session
- Sessions per day: 1
- Rest period: 23 hours

**Power Breakdown:**
```
Active Period (1 hour):
- High-performance mode: 12.55 mA
- Energy: 12.55 mAh

Rest Period (23 hours):
- Low-power mode: 0.126 mA
- Energy: 2.90 mAh

Daily Consumption: 12.55 + 2.90 = 15.45 mAh
Battery Life (500mAh): 500 / 15.45 = 32.4 days
Battery Life (1800mAh): 1800 / 15.45 = 116.5 days
```

#### Scenario A4: 2-Hour Activity Sessions

**Usage Pattern:** Long training sessions or competitions
- Activity: 2 hours per session
- Sessions per day: 1
- Rest period: 22 hours

**Power Breakdown:**
```
Active Period (2 hours):
- High-performance mode: 12.55 mA
- Energy: 25.10 mAh

Rest Period (22 hours):
- Low-power mode: 0.126 mA
- Energy: 2.77 mAh

Daily Consumption: 25.10 + 2.77 = 27.87 mAh
Battery Life (500mAh): 500 / 27.87 = 17.9 days
Battery Life (1800mAh): 1800 / 27.87 = 64.6 days
```

#### Multiple Daily Sessions Comparison

| Activity Pattern | Daily Active Time | Sessions | 500mAh Battery | 1800mAh Battery |
|-----------------|-------------------|----------|----------------|-----------------|
| 6 × 10 minutes | 1 hour | 6 | 32.4 days | 116.5 days |
| 3 × 20 minutes | 1 hour | 3 | 32.4 days | 116.5 days |
| 2 × 30 minutes | 1 hour | 2 | 32.4 days | 116.5 days |
| 1 × 60 minutes | 1 hour | 1 | 32.4 days | 116.5 days |
| 1 × 120 minutes | 2 hours | 1 | 17.9 days | 64.6 days |
| 2 × 60 minutes | 2 hours | 2 | 17.9 days | 64.6 days |
| 3 × 45 minutes | 2.25 hours | 3 | 15.9 days | 57.2 days |

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
- MCU (both cores): 8.5 mA
- BHI360: 0.8 mA
- SAADC: 0.005 mA
- BLE (10% duty): 0.46 mA
- Flash writes: 0.1 mA avg
Total Active: 9.87 mA

Idle Period (16 hours):
- MCU sleep: 0.002 mA
- BHI360 sleep: 0.001 mA
- BLE connected idle: 0.015 mA
Total Idle: 0.018 mA

Daily Average: (9.87 × 8 + 0.018 × 16) / 24 = 3.30 mA
Battery Life (500mAh): 500 / 3.30 / 24 = 6.3 days
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
- MCU (both cores): 8.5 mA
- BHI360: 1.2 mA
- SAADC: 0.05 mA
- BLE (50% duty): 2.3 mA
- Flash writes: 0.5 mA
Total: 12.55 mA

Battery Life (500mAh): 500 / 12.55 / 24 = 1.7 days
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
- MCU (mostly sleeping): 0.1 mA
- BHI360 (low power): 0.01 mA
- SAADC: 0.0005 mA
- BLE advertising: 0.015 mA
Total: 0.126 mA

Battery Life (500mAh): 500 / 0.126 / 24 = 165 days
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
- MCU: 8.5 mA
- Sensors: 0.83 mA
- Flash writes: 1.0 mA
Total: 10.33 mA

Idle Period (12 hours):
- Sleep mode: 0.02 mA

Daily Average: (10.33 × 12 + 0.02 × 12) / 24 = 5.18 mA
Battery Life (500mAh): 500 / 5.18 / 24 = 4.0 days
```

## Battery Life Calculations

### Battery Capacity Considerations

| Battery Type | Capacity | Voltage | Energy (Wh) |
|--------------|----------|---------|-------------|
| CR2032 | 225 mAh | 3.0V | 0.675 |
| LiPo 401230 | 150 mAh | 3.7V | 0.555 |
| LiPo 502540 | 500 mAh | 3.7V | 1.850 |
| LiPo 103450 | 1800 mAh | 3.7V | 6.660 |

### Estimated Battery Life by Scenario

| Scenario | 150mAh | 500mAh | 1800mAh |
|----------|--------|--------|---------|
| **Activity-Based Scenarios** | | | |
| 10-min sessions (6x daily) | 9.7 days | 32.4 days | 116.5 days |
| 30-min sessions (2x daily) | 9.7 days | 32.4 days | 116.5 days |
| 1-hour sessions (1x daily) | 9.7 days | 32.4 days | 116.5 days |
| 2-hour sessions (1x daily) | 5.4 days | 17.9 days | 64.6 days |
| **Continuous Operation** | | | |
| Normal Operation | 1.9 days | 6.3 days | 22.7 days |
| High Performance | 0.5 days | 1.7 days | 6.0 days |
| Low Power | 49.6 days | 165 days | 595 days |
| Data Collection | 1.2 days | 4.0 days | 14.5 days |

### Temperature Impact

Battery capacity decreases with temperature:
- 25°C: 100% capacity
- 0°C: 85% capacity
- -20°C: 65% capacity

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

### Battery Selection Guide

| Use Case | Recommended Battery | Expected Life |
|----------|-------------------|---------------|
| Wearable Device | 500mAh LiPo | 5-7 days |
| Research Study | 1800mAh LiPo | 2-3 weeks |
| Long-term Monitoring | 2x 1800mAh | 1-2 months |
| Disposable Unit | CR2450 | 3-5 days |

## Conclusion

The nRF5340-based sensing system offers flexible power management with battery life ranging from days to months depending on configuration. Proper optimization can achieve the target use case requirements while maintaining functionality. Regular power profiling during development is recommended to validate these estimates.
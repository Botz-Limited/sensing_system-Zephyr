# nRF5340 Battery Usage Estimation Guide

Version: 2.0  
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
  - 10-minute sessions (6x daily): 14.8 days
  - 30-minute sessions (2x daily): 14.8 days
  - 1-hour sessions (1x daily): 14.8 days
  - 2-hour sessions (1x daily): 8.2 days
- **Continuous Operation**: 
  - Typical: 2.8 days with normal operation
  - Best Case: Up to 132 days in low-power monitoring
  - Worst Case: 0.8 days with continuous high-rate streaming

## System Overview

### Hardware Configuration

- **MCU**: nRF5340 (Dual-core ARM Cortex-M33)
  - Application Core: 128 MHz
  - Network Core: 64 MHz
- **Sensors**:
  - Foot Pressure: 8-channel ADC (SAADC) - 1 Hz normal, 10 Hz during activity
  - Motion: BHI360 IMU via SPI - 50 Hz normal, 100 Hz during activity
- **Storage**: External flash (MX25R6435F)
- **Communication**: BLE 5.3 with extended advertising
  - Connection interval: 24-40 ms (30 ms typical)
  - Multiple connections supported (up to 4)
  - D2D communication between primary/secondary devices

**Important Note**: The application core is configured to run at 128 MHz for optimal performance. All calculations in this document reflect this configuration.

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
| Application Core | 8.2 | 1.5 | @ 128 MHz |
| Network Core | 4.5 | 1.0 | BLE operations @ 64 MHz |
| RAM Retention | - | 0.8 | Per 64KB block (512KB total) |
| Peripherals | 1.5 | 0.3 | SPI, I2C, Timers, GPIO, DPPI |
| System overhead | 1.0 | 0.2 | Clocks, regulators |

### 2. Sensor Power Consumption

#### Foot Sensor (SAADC)
```
Active Sampling (1 Hz): 5 µA average
Active Sampling (10 Hz): 50 µA average
Idle: 0.5 µA
```

#### BHI360 Motion Sensor
```
Active (50 Hz, quaternion + accel): 0.8 mA
Active (100 Hz, all sensors): 1.2 mA
Low Power Mode: 10 µA
Sleep Mode: 1 µA
```

### 3. BLE Radio Consumption

| Operation | Current | Duration | Duty Cycle |
|-----------|---------|----------|------------|
| TX (0 dBm) | 5.4 mA | 0.5-2 ms | Variable |
| TX (+8 dBm) | 11.2 mA | 0.5-2 ms | Variable |
| RX | 5.8 mA | 0.5-2 ms | Variable |
| Advertising | 3.2 mA avg | - | 0.8% typical |
| Connected Idle | 30 µA | - | Continuous |
| Connection Event | 9.0 mA | 3-6 ms | Per 30ms interval |

### 4. External Flash

```
Write: 15 mA for 5 ms (typical page write)
Read: 5 mA continuous
Standby: 15 µA
Deep Power Down: 0.1 µA
```

### 5. Additional Components

```
Power Management (PM enabled): 0.2 mA overhead
Multiple threads (6+ active): 0.5 mA
Logging system: 0.3 mA when active
```

## Usage Scenarios

### Activity-Based Scenarios

These scenarios estimate battery consumption for specific activity session durations, assuming high-performance sensing during activity and low-power mode between sessions.

#### Activity Session Configuration
**During Activity:**
- Foot sensor: 10 Hz sampling
- Motion sensor: 100 Hz (all sensors)
- BLE: Streaming mode, 30 ms interval
- Logging: Enabled with buffering
- Power consumption: 25.5 mA (realistic high-performance mode)

**Between Activities (Rest/Sleep):**
- Foot sensor: 0.1 Hz
- Motion sensor: Step counter only
- BLE: Connected idle or advertising
- Logging: Minimal
- Power consumption: 0.45 mA (realistic low-power mode with PM)

#### Scenario A1: 10-Minute Activity Sessions

**Usage Pattern:** Short bursts of activity (e.g., quick exercises, walking intervals)
- Activity: 10 minutes per session
- Sessions per day: 6 (total 1 hour active)
- Rest between sessions: 3 hours 50 minutes each

**Power Breakdown:**
```
Active Periods (6 × 10 min = 1 hour):
- High-performance mode: 25.5 mA
- Energy: 25.5 mAh

Rest Periods (23 hours):
- Low-power mode: 0.45 mA
- Energy: 10.35 mAh

Daily Consumption: 25.5 + 10.35 = 35.85 mAh
Battery Life (500mAh): 500 / 35.85 = 13.9 days
```

#### Scenario A2: 30-Minute Activity Sessions

**Usage Pattern:** Moderate exercise sessions (e.g., jogging, gym workouts)
- Activity: 30 minutes per session
- Sessions per day: 2 (total 1 hour active)
- Rest between sessions: 11 hours 30 minutes each

**Power Breakdown:**
```
Active Periods (2 × 30 min = 1 hour):
- High-performance mode: 25.5 mA
- Energy: 25.5 mAh

Rest Periods (23 hours):
- Low-power mode: 0.45 mA
- Energy: 10.35 mAh

Daily Consumption: 25.5 + 10.35 = 35.85 mAh
Battery Life (500mAh): 500 / 35.85 = 13.9 days
```

#### Scenario A3: 1-Hour Activity Sessions

**Usage Pattern:** Extended activities (e.g., sports practice, long runs)
- Activity: 1 hour per session
- Sessions per day: 1
- Rest period: 23 hours

**Power Breakdown:**
```
Active Period (1 hour):
- High-performance mode: 25.5 mA
- Energy: 25.5 mAh

Rest Period (23 hours):
- Low-power mode: 0.45 mA
- Energy: 10.35 mAh

Daily Consumption: 25.5 + 10.35 = 35.85 mAh
Battery Life (500mAh): 500 / 35.85 = 13.9 days
```

#### Scenario A4: 2-Hour Activity Sessions

**Usage Pattern:** Long training sessions or competitions
- Activity: 2 hours per session
- Sessions per day: 1
- Rest period: 22 hours

**Power Breakdown:**
```
Active Period (2 hours):
- High-performance mode: 25.5 mA
- Energy: 51.0 mAh

Rest Period (22 hours):
- Low-power mode: 0.45 mA
- Energy: 9.9 mAh

Daily Consumption: 51.0 + 9.9 = 60.9 mAh
Battery Life (500mAh): 500 / 60.9 = 8.2 days
```

#### Multiple Daily Sessions Comparison

| Activity Pattern | Daily Active Time | Sessions | Battery Life (500mAh) |
|-----------------|-------------------|----------|----------------------|
| 6 × 10 minutes | 1 hour | 6 | 13.9 days |
| 3 × 20 minutes | 1 hour | 3 | 13.9 days |
| 2 × 30 minutes | 1 hour | 2 | 13.9 days |
| 1 × 60 minutes | 1 hour | 1 | 13.9 days |
| 1 × 120 minutes | 2 hours | 1 | 8.2 days |
| 2 × 60 minutes | 2 hours | 2 | 8.2 days |
| 3 × 45 minutes | 2.25 hours | 3 | 7.3 days |

#### Real-World Considerations for Activity Sessions

1. **Transition Overhead**: Each activity session includes:
   - Sensor warm-up: ~5 seconds at 10 mA
   - BLE connection establishment: ~2 seconds at 15 mA
   - Mode switching: ~1 second at 12 mA
   - Total overhead per session: ~0.04 mAh

2. **Data Synchronization**: Post-activity sync
   - Duration: 1-5 minutes depending on session length
   - Power: 10 mA (BLE transfer + flash read)
   - Impact: ~1-3% additional battery drain per session

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
- MCU (both cores): 15.2 mA
- BHI360: 0.8 mA
- SAADC: 0.005 mA
- BLE (20% duty): 1.8 mA
- Flash writes: 0.3 mA avg
- System overhead: 0.5 mA
Total Active: 18.6 mA

Idle Period (16 hours):
- MCU sleep: 0.004 mA
- BHI360 sleep: 0.001 mA
- BLE connected idle: 0.030 mA
- RAM retention: 0.005 mA
- PM overhead: 0.010 mA
Total Idle: 0.050 mA

Daily Average: (18.6 × 8 + 0.050 × 16) / 24 = 6.23 mA
Battery Life (500mAh): 500 / (6.23 × 24) = 3.3 days
```

#### Scenario 2: High-Performance Mode

**Configuration:**
- Foot sensor: 10 Hz sampling
- Motion sensor: 100 Hz (all sensors)
- BLE: Streaming mode, 30 ms interval
- Logging: Enabled
- Duration: Continuous

**Power Breakdown:**
```
Continuous Operation:
- MCU (both cores @ 128MHz + 64MHz): 15.2 mA
- BHI360: 1.2 mA
- SAADC: 0.05 mA
- BLE (60% duty): 5.4 mA
- Flash writes: 0.6 mA
- System overhead: 0.8 mA
Total: 23.25 mA

Battery Life (500mAh): 500 / (23.25 × 24) = 0.9 days
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
- MCU (mostly sleeping): 0.12 mA
- BHI360 (low power): 0.01 mA
- SAADC: 0.0005 mA
- BLE advertising: 0.025 mA
- PM overhead: 0.002 mA
Total: 0.158 mA

Battery Life (500mAh): 500 / (0.158 × 24) = 132 days
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
- MCU (app core only @ 128MHz): 8.2 mA
- Sensors: 0.83 mA
- Flash writes: 1.2 mA
- System overhead: 0.5 mA
Total: 10.73 mA

Idle Period (12 hours):
- Sleep mode: 0.025 mA

Daily Average: (10.73 × 12 + 0.025 × 12) / 24 = 5.38 mA
Battery Life (500mAh): 500 / (5.38 × 24) = 3.9 days
```

## Battery Life Calculations

### Battery Capacity Considerations

| Battery Type | Capacity | Voltage | Energy (Wh) |
|--------------|----------|---------|-------------|
| LiPo 502540 | 500 mAh | 3.7V | 1.850 |

### Estimated Battery Life by Scenario

| Scenario | Battery Life (500mAh) |
|----------|----------------------|
| **Activity-Based Scenarios** | |
| 10-min sessions (6x daily) | 13.9 days |
| 30-min sessions (2x daily) | 13.9 days |
| 1-hour sessions (1x daily) | 13.9 days |
| 2-hour sessions (1x daily) | 8.2 days |
| **Continuous Operation** | |
| Normal Operation | 3.3 days |
| High Performance | 0.9 days |
| Low Power | 132 days |
| Data Collection | 3.9 days |

### Temperature Impact

Battery capacity decreases with temperature:
- 25°C: 100% capacity
- 0°C: 85% capacity
- -20°C: 65% capacity

## Optimization Strategies

### 1. Dynamic Sensor Configuration

```c
// Optimize sensor sampling rates based on activity
void optimize_sensor_rates(void) {
    if (low_activity_detected()) {
        // Reduce sensor sampling rates
        set_foot_sensor_rate(0.5); // 0.5 Hz
        set_motion_sensor_mode(LOW_POWER);
        // This can save ~2-3 mA during operation
    }
}
```

### 2. Dynamic Power Management

```c
// Reduce sampling when inactive
if (no_motion_detected()) {
    set_foot_sensor_rate(0.1); // 0.1 Hz
    set_motion_sensor_mode(STEP_COUNTER_ONLY);
    // Enter low power mode
}

// Increase sampling when active
if (activity_detected()) {
    set_foot_sensor_rate(5.0); // 5 Hz
    set_motion_sensor_mode(FULL_SENSING);
    // Full performance mode
}
```

### 3. BLE Connection Parameters

```c
// Optimize connection interval based on data rate
if (data_rate < 100) { // bytes/sec
    conn_params.min_interval = 100; // 125 ms
    conn_params.max_interval = 200; // 250 ms
} else {
    conn_params.min_interval = 24;  // 30 ms (current setting)
    conn_params.max_interval = 40;  // 50 ms
}
```

### 4. Batch Processing

```c
// Buffer data before flash writes (already implemented)
#define WRITE_THRESHOLD 256 // Current page size

if (buffer_size >= WRITE_THRESHOLD) {
    flash_write_batch(buffer, buffer_size);
    buffer_size = 0;
}
```

### 5. Sensor Duty Cycling

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

### 6. Thread Optimization

- Reduce thread stack sizes where possible
- Consolidate threads to reduce context switching
- Use event-driven architecture instead of polling

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
| Multiple device connections | +15% per additional connection |

### 4. Firmware Updates

FOTA operations consume significant power:
- Download: ~50 mA for 2-5 minutes
- Flash write: ~20 mA for 30 seconds
- Verification: ~10 mA for 10 seconds
- Total impact: ~1% of battery per update

### 5. System-Specific Factors

Based on actual implementation:
- Multiple message queues: +0.3 mA overhead
- Deferred logging: +0.2 mA when active
- Event manager: +0.1 mA overhead
- File system operations: +0.5 mA during writes
- D2D communication: +2 mA when active

## Recommendations

### For Maximum Battery Life

1. **Use Dynamic Sampling**
   - Reduce rates during inactivity
   - Implement motion-triggered activation

2. **Optimize BLE Parameters**
   - Increase connection intervals when possible
   - Use slave latency effectively
   - Minimize advertising when not needed

3. **Batch Operations**
   - Group flash writes (already implemented)
   - Combine BLE notifications
   - Use DMA for data transfers

4. **Power-Aware Design**
   - Disable unused peripherals
   - Implement proper sleep modes
   - Use hardware accelerators when available

5. **Code Optimization**
   - Reduce thread count and stack sizes
   - Minimize logging in production
   - Use event-driven instead of polling

### Battery Selection Guide

With the 500mAh LiPo battery:

| Use Case | Expected Life | Notes |
|----------|---------------|-------|
| Daily Activity Sessions (1hr) | 13.9 days | Ideal for fitness tracking |
| Intensive Training (2hr daily) | 8.2 days | Requires weekly charging |
| Continuous Monitoring | 3.3 days | Normal operation mode |
| Low-Power Standby | 132 days | Minimal sensing, advertising only |

## Conclusion

The nRF5340-based sensing system is optimized for performance with the application core running at 128 MHz. With proper optimization strategies, the system achieves reasonable battery life for most use cases. The key factors affecting battery life are:

1. Sensor sampling rates and configurations
2. BLE connection parameters and data throughput
3. Activity duration and frequency
4. Environmental conditions
5. Power management implementation

Regular power profiling during development is recommended to validate these estimates and identify optimization opportunities.
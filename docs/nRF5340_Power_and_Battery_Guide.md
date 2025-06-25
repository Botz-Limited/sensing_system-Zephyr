# nRF5340 Power and Battery Management Guide

**Version:** 1.0  
**Date:** January 2025  
**Platform:** nRF5340 Dual-Core (Application + Network Core)  
**Scope:** Complete guide for power management, battery usage estimation, and optimization strategies

---

## Table of Contents

1. [Overview](#overview)
2. [nRF5340 Power Architecture](#nrf5340-power-architecture)
3. [Power Consumption Analysis](#power-consumption-analysis)
4. [Battery Life Calculations](#battery-life-calculations)
5. [Power Management Implementation](#power-management-implementation)
6. [Optimization Strategies](#optimization-strategies)
7. [Testing and Validation](#testing-and-validation)
8. [Quick Reference](#quick-reference)

---

## Overview

This guide consolidates all power and battery management information for the nRF5340-based sensing firmware. It covers:

- Hardware power architecture and capabilities
- Current consumption in various states
- Battery life estimations for different usage scenarios
- Implementation of power-saving features
- Optimization strategies and best practices

### Key Metrics

- **Activity-Based Usage (500mAh battery)**:
  - 10-minute sessions (6x daily): 14.8 days
  - Typical continuous use: 2.8 days
  - Low-power monitoring: Up to 132 days
  - High-performance streaming: 0.8 days

---

## nRF5340 Power Architecture

### Dual-Core Power Domains

```
nRF5340:
├── Application Core (M33)
│   ├── CPU Power Domain
│   ├── RAM Power Domains (8 sections, 64KB each)
│   └── Peripheral Power Domains
└── Network Core (M33)
    ├── CPU Power Domain
    ├── RAM Power Domain
    └── Radio Power Domain
```

### Power States

| State | App Core | Net Core | Current | Wake Time | Use Case |
|-------|----------|----------|---------|-----------|----------|
| System ON | Active | Active | 3-10 mA | - | Normal operation |
| System ON Idle | WFI | WFI | 1-3 mA | <1 µs | Between operations |
| System OFF | OFF | OFF | 0.4 µA | 5-10 ms | Deep sleep |
| Network Core OFF | Active | OFF | 2-5 mA | 1-2 ms | No BLE needed |

---

## Power Consumption Analysis

### Component Breakdown

#### 1. MCU Power
```
Application Core @ 128 MHz: 3-5 mA
Network Core (BLE active): 5-8 mA
Both cores idle: 1-2 mA
System OFF: 0.4 µA
```

#### 2. Sensor Power
```
BHI360 IMU:
- Active (100 Hz): 1.2 mA
- Low Power Mode: 10 µA
- Sleep Mode: 1 µA

Foot Pressure (SAADC):
- Active (100 Hz): 0.5 mA
- Standby: 15 µA
- Deep Power Down: 0.1 µA
```

#### 3. BLE Power
```
Advertising: 1-2 mA average
Connected (1s interval): 1.5 mA
Connected (50ms interval): 8-10 mA
Data streaming: 10-15 mA
```

#### 4. System Overhead
```
Logging enabled: 0.5-1 mA
Power Management: 0.2 mA
Multiple threads: 0.5 mA
```

### Usage Scenarios

| Scenario | Total Current | Battery Life (500mAh) |
|----------|--------------|----------------------|
| High Performance | 23.25 mA | 0.9 days |
| Normal Operation | 6.23 mA | 3.3 days |
| Low Power Monitoring | 0.158 mA | 132 days |
| Activity Sessions (1hr/day) | 1.49 mA | 14 days |

---

## Battery Life Calculations

### Battery Capacity Considerations

| Battery Type | Capacity | Voltage | Energy (Wh) | Notes |
|--------------|----------|---------|-------------|-------|
| CR2032 | 225 mAh | 3.0V | 0.675 | Not suitable |
| LiPo | 500 mAh | 3.7V | 1.85 | Recommended |
| Li-ion 18650 | 2600 mAh | 3.7V | 9.62 | Lab use only |

### Activity-Based Calculations

#### Running Sessions (1 hour/day)
```
Active Period (1 hour):
- Full sensing: 25.5 mA
- Energy: 25.5 mAh

Rest Period (23 hours):
- Low-power mode: 0.45 mA
- Energy: 10.35 mAh

Daily Total: 35.85 mAh
Battery Life: 500 / 35.85 = 13.9 days
```

#### Mixed Activity (6x 10-min sessions)
```
Active Periods (60 minutes total):
- High performance: 25.5 mA
- Energy: 25.5 mAh

Rest Periods (23 hours):
- Low-power mode: 0.45 mA
- Energy: 10.35 mAh

Daily Total: 35.85 mAh
Battery Life: 500 / 35.85 = 13.9 days
```

### Environmental Factors

Temperature effects on battery capacity:
- 25°C: 100% capacity
- 0°C: 80% capacity
- -20°C: 50% capacity

---

## Power Management Implementation

### 1. Activity-Based Power Modes

```c
typedef enum {
    POWER_MODE_SLEEP,      // Everything off except RTC
    POWER_MODE_IDLE,       // Low-rate motion detection only
    POWER_MODE_WALKING,    // Reduced sampling rates
    POWER_MODE_RUNNING,    // Full sampling rates
    POWER_MODE_ANALYSIS    // Post-activity processing
} power_mode_t;

void update_power_mode(void) {
    static uint32_t no_motion_counter = 0;
    
    if (is_motion_detected()) {
        no_motion_counter = 0;
        if (activity_level > RUNNING_THRESHOLD) {
            set_power_mode(POWER_MODE_RUNNING);
        } else if (activity_level > WALKING_THRESHOLD) {
            set_power_mode(POWER_MODE_WALKING);
        } else {
            set_power_mode(POWER_MODE_IDLE);
        }
    } else {
        no_motion_counter++;
        if (no_motion_counter > 300) {  // 5 minutes
            set_power_mode(POWER_MODE_SLEEP);
        }
    }
}
```

### 2. Sensor Configuration by Mode

```c
void configure_sampling_for_power_mode(power_mode_t mode) {
    switch (mode) {
        case POWER_MODE_SLEEP:
            disable_all_sensors();
            break;
            
        case POWER_MODE_IDLE:
            set_imu_rate(1.0);      // 1 Hz
            set_pressure_rate(0.0);  // Disabled
            break;
            
        case POWER_MODE_WALKING:
            set_imu_rate(25.0);     // 25 Hz
            set_pressure_rate(10.0); // 10 Hz
            break;
            
        case POWER_MODE_RUNNING:
            set_imu_rate(50.0);     // 50 Hz
            set_pressure_rate(50.0); // 50 Hz
            break;
    }
}
```

### 3. BLE Connection Parameters

```c
void optimize_ble_for_power_mode(power_mode_t mode) {
    struct bt_le_conn_param params;
    
    switch (mode) {
        case POWER_MODE_SLEEP:
            bt_le_adv_stop();
            break;
            
        case POWER_MODE_IDLE:
            params.interval_min = 800;  // 1000ms
            params.interval_max = 800;
            params.latency = 4;
            break;
            
        case POWER_MODE_RUNNING:
            params.interval_min = 40;   // 50ms
            params.interval_max = 40;
            params.latency = 0;
            break;
    }
    
    if (current_conn && mode != POWER_MODE_SLEEP) {
        bt_conn_le_param_update(current_conn, &params);
    }
}
```

### 4. RAM Power Management

```c
void configure_ram_retention(bool entering_sleep) {
    if (entering_sleep) {
        // Power down unused RAM sections (keep 0-1 for critical data)
        nrf_vmc_ram_block_retention_mask_set(NRF_VMC, 0x0003);
        LOG_INF("RAM sections 2-7 powered down");
    } else {
        // Power up all RAM sections
        nrf_vmc_ram_block_retention_mask_set(NRF_VMC, 0xFFFF);
        LOG_INF("All RAM sections powered");
    }
}
```

### 5. Wake-Up Configuration

```c
void configure_wake_gpio(void) {
    // Configure GPIO for wake from System OFF
    nrf_gpio_cfg_sense_input(MOTION_INT_PIN,
                            NRF_GPIO_PIN_PULLDOWN,
                            NRF_GPIO_PIN_SENSE_HIGH);
    
    // Check reset reason
    uint32_t reset_reason = nrf_power_resetreas_get(NRF_POWER);
    if (reset_reason & NRF_POWER_RESETREAS_OFF_MASK) {
        LOG_INF("Woke from System OFF via GPIO");
        fast_sensor_init();
    }
}
```

---

## Optimization Strategies

### 1. Adaptive Sampling
- Reduce sensor rates based on activity
- Disable unused sensors
- Use motion interrupts instead of polling

### 2. Smart Data Batching
- Buffer sensor data to reduce BLE transmissions
- Flush based on activity level
- Use larger MTU for efficiency

### 3. Thread Management
```c
void suspend_non_critical_threads(void) {
    k_thread_suspend(logging_thread);
    k_thread_suspend(ui_thread);
    k_thread_suspend(analytics_thread);
}
```

### 4. Peripheral Power Gating
```c
void disable_unused_peripherals(void) {
    // Disable unused UARTs
    nrf_uarte_disable(NRF_UARTE0);
    
    // Disable unused timers
    nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_SHUTDOWN);
    
    // Disable unused PWM
    nrf_pwm_disable(NRF_PWM0);
}
```

### 5. Network Core Management
```c
void manage_network_core_power(bool ble_needed) {
    if (!ble_needed) {
        // Request network core to enter low power
        nrf_ipc_task_trigger(NRF_IPC, NRF_IPC_TASK_SEND_2);
        LOG_INF("Network core entering low power");
    }
}
```

---

## Testing and Validation

### 1. Power Profiling Setup
```bash
# Using Nordic Power Profiler Kit II
# Connect to DUT, set voltage to 3.7V
# Enable logging mode for detailed analysis
```

### 2. Test Scenarios
- Idle current measurement
- Activity detection response time
- BLE connection current profile
- Sensor duty cycling validation

### 3. Validation Metrics
```c
typedef struct {
    power_mode_t mode;
    float avg_current_ma;
    uint32_t wake_latency_ms;
    uint32_t mode_duration_s;
} power_metrics_t;

void log_power_metrics(void) {
    LOG_INF("Mode: %d, Current: %.1f mA, Wake: %d ms",
            current_mode, measured_current, wake_latency);
}
```

---

## Quick Reference

### Power Modes Summary
| Mode | IMU Rate | Pressure Rate | BLE Interval | Current |
|------|----------|---------------|--------------|---------|
| SLEEP | Off | Off | Disconnected | 0.4 µA |
| IDLE | 1 Hz | Off | 1000 ms | 0.45 mA |
| WALKING | 25 Hz | 10 Hz | 200 ms | 8 mA |
| RUNNING | 50 Hz | 50 Hz | 50 ms | 25 mA |

### Key Commands
```c
// Enter System OFF
nrf_power_system_off(NRF_POWER);

// Configure RAM retention
nrf_vmc_ram_block_retention_mask_set(NRF_VMC, mask);

// Check reset reason
uint32_t reason = nrf_power_resetreas_get(NRF_POWER);

// Suspend thread
k_thread_suspend(thread_id);
```

### Best Practices
1. Always profile actual current consumption
2. Use motion interrupts for wake-up
3. Batch operations to minimize wake time
4. Power down unused peripherals and RAM
5. Optimize BLE connection parameters
6. Consider temperature effects on battery

---

## Summary

The nRF5340 provides excellent power management capabilities through:
- Independent power domains for cores and peripherals
- Flexible RAM retention options
- Multiple low-power states
- Fast wake-up mechanisms

With proper implementation, battery life can be extended from hours to weeks depending on usage patterns. The key is matching power modes to activity levels and aggressively powering down unused components.
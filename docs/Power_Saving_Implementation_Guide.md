# Power Saving Implementation Guide

Hey team! Here's how we can implement power saving features in our firmware to maximize battery life.

## Current Power Consumption Analysis

### Major Power Consumers:
1. **BHI360 IMU** - ~3-5mA active, <10µA sleep
2. **Pressure Sensors (ADC)** - ~1-2mA during sampling
3. **BLE Radio** - ~8-15mA during TX/RX, ~1µA in idle
4. **MCU (nRF5340)** - ~3-10mA active, <5µA in sleep
5. **Flash Operations** - ~10-15mA during writes

### Current Issues:
- Sensors running continuously at 100Hz (pressure) and 50Hz (IMU)
- BLE always advertising/connected
- No adaptive sampling based on activity
- CPU always running at full speed

## nRF5340 Power Architecture

The nRF5340 has excellent power management features we can leverage:

### Dual-Core Power Domains
```
nRF5340:
├── Application Core (M33)
│   ├── CPU Power Domain
│   ├── RAM Power Domains (8 sections, 512KB total)
│   └── Peripheral Power Domains
└── Network Core (M33) 
    ├── CPU Power Domain
    ├── RAM Power Domain
    └── Radio Power Domain
```

### Power States Overview

| State | App Core | Net Core | Current | Wake Time | Use Case |
|-------|----------|----------|---------|-----------|----------|
| Active | Running | Running | 3-10mA | - | Full operation |
| Idle | WFI | WFI | 1-3mA | <1µs | Between tasks |
| System ON Idle | Sleep | Sleep | 7.5µA | 3.5µs | RTC + RAM retention |
| System OFF | Off | Off | 0.4µA | 15µs | GPIO wake only |

## Power Saving Strategies

### 1. Activity-Based Power Modes

```c
typedef enum {
    POWER_MODE_SLEEP,      // Everything off except RTC
    POWER_MODE_IDLE,       // Low-rate motion detection only
    POWER_MODE_WALKING,    // Reduced sampling rates
    POWER_MODE_RUNNING,    // Full sampling rates
    POWER_MODE_ANALYSIS    // Post-activity processing
} power_mode_t;

static power_mode_t current_power_mode = POWER_MODE_IDLE;

void update_power_mode(void) {
    static uint32_t no_motion_counter = 0;
    
    // Check for motion using low-power accelerometer interrupt
    if (is_motion_detected()) {
        no_motion_counter = 0;
        
        // Determine activity level
        float activity_level = get_activity_intensity();
        
        if (activity_level > RUNNING_THRESHOLD) {
            set_power_mode(POWER_MODE_RUNNING);
        } else if (activity_level > WALKING_THRESHOLD) {
            set_power_mode(POWER_MODE_WALKING);
        } else {
            set_power_mode(POWER_MODE_IDLE);
        }
    } else {
        no_motion_counter++;
        
        // Go to sleep after 5 minutes of no motion
        if (no_motion_counter > 300) {
            set_power_mode(POWER_MODE_SLEEP);
        }
    }
}
```

### 2. Adaptive Sampling Rates

```c
// In motion_sensor.cpp
void configure_sampling_for_power_mode(power_mode_t mode) {
    float imu_rate, pressure_rate;
    uint32_t report_latency_ms;
    
    switch (mode) {
        case POWER_MODE_SLEEP:
            // Disable all sensors
            bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, 0, 0, bhy2_ptr);
            bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, 0, 0, bhy2_ptr);
            disable_pressure_sensors();
            break;
            
        case POWER_MODE_IDLE:
            // Minimal sampling for motion detection
            imu_rate = 5.0f;   // 5Hz for basic motion
            pressure_rate = 0;  // Pressure sensors off
            report_latency_ms = 1000;  // Batch for 1 second
            break;
            
        case POWER_MODE_WALKING:
            // Reduced rates for walking
            imu_rate = 25.0f;  // 25Hz
            pressure_rate = 50.0f;  // 50Hz
            report_latency_ms = 200;
            break;
            
        case POWER_MODE_RUNNING:
            // Full rates for running
            imu_rate = 50.0f;  // 50Hz
            pressure_rate = 100.0f;  // 100Hz
            report_latency_ms = 0;  // Real-time
            break;
    }
    
    // Apply configuration
    if (mode != POWER_MODE_SLEEP) {
        bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, imu_rate, report_latency_ms, bhy2_ptr);
        configure_pressure_sampling(pressure_rate);
    }
}
```

### 3. BLE Connection Optimization

```c
// In bluetooth.cpp
void optimize_ble_for_power_mode(power_mode_t mode) {
    struct bt_le_conn_param params;
    
    switch (mode) {
        case POWER_MODE_SLEEP:
            // Disconnect and stop advertising
            if (current_conn) {
                bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            }
            bt_le_adv_stop();
            break;
            
        case POWER_MODE_IDLE:
            // Slow advertising, long connection intervals
            params.interval_min = BT_GAP_INIT_CONN_INT_MIN;  // 30ms
            params.interval_max = 400;  // 500ms (400 * 1.25ms)
            params.latency = 4;
            params.timeout = 400;  // 4 seconds
            start_slow_advertising();
            break;
            
        case POWER_MODE_WALKING:
            // Medium connection parameters
            params.interval_min = 24;   // 30ms
            params.interval_max = 40;   // 50ms
            params.latency = 2;
            params.timeout = 400;
            break;
            
        case POWER_MODE_RUNNING:
            // Fast connection for real-time data
            params.interval_min = 12;   // 15ms
            params.interval_max = 24;   // 30ms
            params.latency = 0;
            params.timeout = 400;
            break;
    }
    
    if (current_conn && mode != POWER_MODE_SLEEP) {
        bt_conn_le_param_update(current_conn, &params);
    }
}
```

### 4. Smart Sensor Wake-Up with nRF5340 GPIO

```c
// Configure BHI360 motion-triggered wake-up
void setup_motion_wakeup(void) {
    // Configure any-motion interrupt
    struct bhy2_any_motion_config config = {
        .threshold = 10,      // mg units
        .duration = 2,        // samples
        .axes_enabled = 0x07  // X, Y, Z
    };
    
    bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ANY_MOTION, 
                            1.0f,  // 1Hz checking
                            0, 
                            bhy2_ptr);
    
    // Configure interrupt to wake MCU
    bhy2_set_host_interrupt_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, bhy2_ptr);
}

// Configure nRF5340 GPIO for wake from System OFF
void configure_wake_gpio(void) {
    // Get GPIO device
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    
    // Configure BHI360 interrupt pin for wake
    gpio_pin_configure(gpio_dev, BHI360_INT_PIN, 
                      GPIO_INPUT | GPIO_PULL_DOWN);
    
    // Configure interrupt with wake capability
    gpio_pin_interrupt_configure(gpio_dev, BHI360_INT_PIN,
                                GPIO_INT_EDGE_TO_ACTIVE);
    
    // IMPORTANT: Configure as SENSE for System OFF wake
    nrf_gpio_cfg_sense_input(NRF_GPIO_PIN_MAP(0, BHI360_INT_PIN),
                            NRF_GPIO_PIN_PULLDOWN,
                            NRF_GPIO_PIN_SENSE_HIGH);
    
    // Set up callback
    static struct gpio_callback gpio_cb;
    gpio_init_callback(&gpio_cb, wake_interrupt_handler, BIT(BHI360_INT_PIN));
    gpio_add_callback(gpio_dev, &gpio_cb);
}

// Wake interrupt handler
void wake_interrupt_handler(const struct device *dev, 
                          struct gpio_callback *cb, 
                          uint32_t pins) {
    // Check reset reason to detect wake from System OFF
    uint32_t reset_reason = nrf_power_resetreas_get(NRF_POWER);
    
    if (reset_reason & NRF_POWER_RESETREAS_OFF_MASK) {
        LOG_INF("Woke from System OFF via GPIO");
        // System restarted - need full initialization
        handle_system_off_wake();
    } else {
        LOG_INF("GPIO interrupt in normal operation");
        // Normal interrupt handling
        k_sem_give(&motion_detected_sem);
    }
}
```

### 5. Intelligent Data Batching

```c
// Buffer data instead of sending immediately
typedef struct {
    uint8_t buffer[1024];
    size_t write_idx;
    size_t read_idx;
    k_timer flush_timer;
} data_batcher_t;

void batch_sensor_data(const void* data, size_t len) {
    // Add to buffer instead of immediate transmission
    if (batcher.write_idx + len < sizeof(batcher.buffer)) {
        memcpy(&batcher.buffer[batcher.write_idx], data, len);
        batcher.write_idx += len;
    }
    
    // Flush based on power mode
    uint32_t flush_interval = get_flush_interval_for_mode(current_power_mode);
    k_timer_start(&batcher.flush_timer, K_MSEC(flush_interval), K_NO_WAIT);
}
```

### 6. Advanced nRF5340 Power Management

```c
// Thread suspension for power saving
typedef struct {
    k_tid_t tid;
    const char *name;
    bool is_critical;
    bool suspended;
} thread_info_t;

static thread_info_t managed_threads[] = {
    {NULL, "ble_handler", true, false},      // Critical - keep running
    {NULL, "motion_sensor", false, false},   // Suspend when idle
    {NULL, "data_logger", false, false},     // Suspend when idle
    {NULL, "ui_handler", false, false},      // Suspend when idle
};

void suspend_non_critical_threads(void) {
    LOG_INF("Suspending non-critical threads for power saving");
    
    for (int i = 0; i < ARRAY_SIZE(managed_threads); i++) {
        if (!managed_threads[i].is_critical && 
            managed_threads[i].tid != NULL &&
            !managed_threads[i].suspended) {
            
            k_thread_suspend(managed_threads[i].tid);
            managed_threads[i].suspended = true;
            LOG_DBG("Suspended thread: %s", managed_threads[i].name);
        }
    }
}

// RAM power management - nRF5340 has 8 sections
void configure_ram_retention(bool entering_sleep) {
    if (entering_sleep) {
        // Power down unused RAM sections
        // Keep only section 0-1 for critical data (64KB)
        nrf_vmc_ram_block_retention_mask_set(NRF_VMC, 
            NRF_VMC_RAM_BLOCK_RETENTION_MASK_S0RETENTION_MASK |
            NRF_VMC_RAM_BLOCK_RETENTION_MASK_S1RETENTION_MASK);
        
        LOG_INF("RAM retention set to 64KB (sections 0-1 only)");
    } else {
        // Power up all RAM sections
        nrf_vmc_ram_block_retention_mask_set(NRF_VMC, 0xFFFF);
        LOG_INF("All RAM sections powered");
    }
}

// Peripheral power management
void disable_unused_peripherals(void) {
    // Disable UART when not debugging
    #ifndef CONFIG_DEBUG_UART
    NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
    NRF_UARTE1->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
    #endif
    
    // Disable unused timers
    NRF_TIMER1->TASKS_STOP = 1;
    NRF_TIMER2->TASKS_STOP = 1;
    
    // Disable PWM if not used
    NRF_PWM0->ENABLE = PWM_ENABLE_ENABLE_Disabled;
    
    // Disable unused I2C/SPI
    if (!spi_in_use) {
        NRF_SPIM1->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
    }
}

// Network core power management
void manage_network_core_power(bool ble_needed) {
    if (!ble_needed) {
        // Stop network core to save ~5mA
        LOG_INF("Stopping network core");
        bt_disable();
        nrf_reset_network_force_off(NRF_RESET, true);
    } else {
        // Wake network core
        LOG_INF("Starting network core");
        nrf_reset_network_force_off(NRF_RESET, false);
        bt_enable(NULL);
    }
}

// Enter System OFF mode (0.4µA)
void enter_system_off_mode(void) {
    LOG_INF("Entering System OFF mode");
    
    // Configure wake sources
    configure_wake_gpio();
    
    // Disable all peripherals
    disable_unused_peripherals();
    
    // Clear any pending events
    __SEV();
    __WFE();
    __WFE();
    
    // Enter System OFF
    nrf_power_system_off(NRF_POWER);
    
    // CPU will reset on wake - code never reaches here
}

// Use Zephyr's power management
void configure_cpu_power_saving(void) {
    // Enable automatic CPU sleep
    pm_policy_state_lock_get(PM_STATE_ACTIVE, PM_ALL_SUBSTATES);
    
    // Set constraints based on mode
    switch (current_power_mode) {
        case POWER_MODE_SLEEP:
            // Allow deep sleep
            pm_policy_state_lock_put(PM_STATE_ACTIVE, PM_ALL_SUBSTATES);
            suspend_non_critical_threads();
            configure_ram_retention(true);
            manage_network_core_power(false);
            break;
            
        case POWER_MODE_IDLE:
            // Allow light sleep between samples
            pm_constraint_set(PM_STATE_STANDBY);
            suspend_non_critical_threads();
            break;
            
        case POWER_MODE_RUNNING:
            // Stay active for responsiveness
            pm_constraint_set(PM_STATE_ACTIVE);
            configure_ram_retention(false);
            resume_all_threads();
            break;
    }
}
```

### 7. Flash Write Optimization

```c
// Batch flash writes to reduce power
typedef struct {
    uint8_t write_buffer[4096];  // Full page
    size_t buffer_used;
    k_timer write_timer;
} flash_batcher_t;

void queue_flash_write(const void* data, size_t len) {
    // Don't write immediately - batch it
    if (flash_batcher.buffer_used + len <= sizeof(flash_batcher.write_buffer)) {
        memcpy(&flash_batcher.write_buffer[flash_batcher.buffer_used], data, len);
        flash_batcher.buffer_used += len;
        
        // Write when buffer is full or after timeout
        if (flash_batcher.buffer_used >= 3072) {  // 75% full
            flush_flash_buffer();
        } else {
            k_timer_start(&flash_batcher.write_timer, K_SECONDS(5), K_NO_WAIT);
        }
    }
}
```

## Implementation Priority

### Phase 1: Quick Wins (1-2 weeks)
1. **Motion-based wake/sleep**
   - Add any-motion detection
   - Sleep after 5 minutes idle
   - ~50% power savings when not worn

2. **BLE connection parameters**
   - Adjust based on activity
   - Slow advertising when idle
   - ~30% BLE power reduction

### Phase 2: Adaptive Sampling (2-3 weeks)
1. **Dynamic sensor rates**
   - Reduce rates when walking
   - Disable pressure sensors when idle
   - ~40% sensor power reduction

2. **Data batching**
   - Buffer before transmission
   - Batch flash writes
   - ~20% I/O power reduction

### Phase 3: Advanced Features (3-4 weeks)
1. **Predictive power management**
   - Learn user patterns
   - Pre-emptive mode switching
   - Additional 10-15% savings

2. **Component shutdown**
   - Power gate unused peripherals
   - Selective sensor enabling
   - Maximum power efficiency

## Expected Battery Life Improvements

### Current Consumption (No Power Saving):
- Active: ~25mA average
- Battery: 100mAh
- **Life: ~4 hours**

### With Basic Power Saving:
- Active running: 25mA (1 hour/day)
- Active walking: 15mA (2 hours/day)
- Idle: 5mA (8 hours/day)
- Sleep: 0.05mA (13 hours/day)
- **Average: ~5.5mA**
- **Life: ~18 hours**

### With Full nRF5340 Optimization:
- Active running: 20mA (1 hour/day)
- Active walking: 10mA (2 hours/day)
- Idle: 2mA (8 hours/day)
- System ON Idle: 7.5µA (8 hours/day)
- System OFF: 0.4µA (5 hours/day)
- **Average: ~2.5mA**
- **Life: ~40 hours**

### Power Consumption by State (nRF5340-specific)

| Mode | Configuration | Current |
|------|--------------|---------|
| System OFF | GPIO wake only | 0.4µA |
| Deep Sleep | RTC + 1 GPIO | 1.5µA |
| System ON Idle | RTC + RAM retention | 7.5µA |
| BLE Advertising | 1s interval | 20µA avg |
| Idle (sensors off) | Thread suspended | 2mA |
| Walking | 25Hz IMU, 50Hz pressure | 10mA |
| Running | 50Hz IMU, 100Hz pressure | 20mA |
| Full Operation | All features | 25mA |

## Testing Power Consumption

```c
// Add power profiling mode
#ifdef CONFIG_POWER_PROFILING
void log_power_metrics(void) {
    power_metrics_t metrics = {
        .mode = current_power_mode,
        .cpu_active_time = get_cpu_active_percentage(),
        .ble_tx_time = get_ble_active_percentage(),
        .sensor_active_time = get_sensor_active_percentage(),
        .estimated_current_ma = calculate_current_consumption()
    };
    
    LOG_INF("Power: Mode=%d, CPU=%d%%, BLE=%d%%, Sensors=%d%%, Current=%.1fmA",
            metrics.mode,
            metrics.cpu_active_time,
            metrics.ble_tx_time,
            metrics.sensor_active_time,
            metrics.estimated_current_ma);
}
#endif
```

## Configuration Options

Add to Kconfig:
```kconfig
menu "Power Management"

config ENABLE_ADAPTIVE_SAMPLING
    bool "Enable adaptive sensor sampling"
    default y
    help
      Automatically adjust sampling rates based on activity

config IDLE_TIMEOUT_SECONDS
    int "Seconds before entering idle mode"
    default 30
    range 10 300

config SLEEP_TIMEOUT_SECONDS
    int "Seconds before entering sleep mode"
    default 300
    range 60 3600

config MIN_BATTERY_PERCENT
    int "Minimum battery for full features"
    default 20
    help
      Below this level, some features are disabled

endmenu
```

## nRF5340-Specific Implementation Summary

### Key Features to Leverage:

1. **Dual-Core Architecture**
   - Shut down network core when BLE not needed (saves ~5mA)
   - Independent power domains for flexibility

2. **Thread Suspension**
   - Suspend non-critical threads during idle
   - Prevents unnecessary CPU wake-ups
   - Easy to implement with `k_thread_suspend()`

3. **RAM Power Management**
   - 8 independent RAM sections (64KB each)
   - Power down unused sections in sleep
   - Keep only critical data in retained RAM

4. **GPIO Wake Configuration**
   - Use `nrf_gpio_cfg_sense_input()` for System OFF wake
   - Multiple wake sources supported
   - 15µs wake time from System OFF

5. **System Power States**
   - System OFF: 0.4µA (deepest sleep, GPIO wake only)
   - System ON Idle: 7.5µA (RTC + RAM retention)
   - Active with optimizations: 2-25mA

### Implementation Checklist:

- [ ] Configure GPIO SENSE for wake interrupts
- [ ] Implement thread suspension for idle modes
- [ ] Set up RAM retention for sleep modes
- [ ] Disable unused peripherals
- [ ] Manage network core based on BLE needs
- [ ] Use Zephyr PM hooks for automatic management
- [ ] Test wake latency and reliability
- [ ] Profile power consumption in each state

With these nRF5340-specific optimizations, we can extend battery life from 4 hours to over 40 hours with smart activity detection and adaptive sampling!
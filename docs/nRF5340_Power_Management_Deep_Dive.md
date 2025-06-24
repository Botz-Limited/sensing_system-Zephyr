# nRF5340 Power Management Deep Dive

Hey team! The nRF5340 is actually amazing for power management - it has dual cores, multiple power domains, and very flexible wake mechanisms. Here's how to squeeze every bit of battery life out of it.

## nRF5340 Power Architecture

### Dual-Core Power Domains
```
nRF5340:
├── Application Core (M33)
│   ├── CPU Power Domain
│   ├── RAM Power Domains (8 sections)
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

## 1. GPIO Wake-Up Configuration

### Setting Up Wake Interrupts

```c
#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>
#include <drivers/gpio.h>

// Best practice: Use GPIOTE for wake interrupts
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

### Multiple Wake Sources

```c
// Configure multiple GPIO pins for wake
void configure_multiple_wake_sources(void) {
    // Array of wake-capable pins
    const struct wake_pin {
        uint8_t pin;
        nrf_gpio_pin_pull_t pull;
        nrf_gpio_pin_sense_t sense;
    } wake_pins[] = {
        {BHI360_INT_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH},
        {CHARGER_DETECT_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW},
        {BUTTON_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW},
    };
    
    // Configure each pin
    for (int i = 0; i < ARRAY_SIZE(wake_pins); i++) {
        nrf_gpio_cfg_sense_input(wake_pins[i].pin,
                                wake_pins[i].pull,
                                wake_pins[i].sense);
    }
    
    // Check which pin caused wake after reset
    for (int i = 0; i < ARRAY_SIZE(wake_pins); i++) {
        if (nrf_gpio_pin_latch_get(wake_pins[i].pin)) {
            LOG_INF("Wake triggered by pin %d", wake_pins[i].pin);
            nrf_gpio_pin_latch_clear(wake_pins[i].pin);
        }
    }
}
```

## 2. Thread Suspension for Power Saving

### Suspend Non-Critical Threads

```c
// Thread management for power saving
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

void register_managed_thread(const char *name, k_tid_t tid, bool is_critical) {
    for (int i = 0; i < ARRAY_SIZE(managed_threads); i++) {
        if (strcmp(managed_threads[i].name, name) == 0) {
            managed_threads[i].tid = tid;
            managed_threads[i].is_critical = is_critical;
            break;
        }
    }
}

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

void resume_all_threads(void) {
    LOG_INF("Resuming all threads");
    
    for (int i = 0; i < ARRAY_SIZE(managed_threads); i++) {
        if (managed_threads[i].suspended && 
            managed_threads[i].tid != NULL) {
            
            k_thread_resume(managed_threads[i].tid);
            managed_threads[i].suspended = false;
            LOG_DBG("Resumed thread: %s", managed_threads[i].name);
        }
    }
}
```

### Dynamic Thread Priority Adjustment

```c
// Adjust thread priorities based on power mode
void adjust_thread_priorities_for_power_mode(power_mode_t mode) {
    switch (mode) {
        case POWER_MODE_SLEEP:
            // Suspend all non-critical
            suspend_non_critical_threads();
            break;
            
        case POWER_MODE_IDLE:
            // Lower priority for sensor threads
            if (motion_sensor_tid) {
                k_thread_priority_set(motion_sensor_tid, 10);  // Lower priority
            }
            break;
            
        case POWER_MODE_RUNNING:
            // Normal priorities
            if (motion_sensor_tid) {
                k_thread_priority_set(motion_sensor_tid, 5);   // Normal priority
            }
            resume_all_threads();
            break;
    }
}
```

## 3. RAM Power Management

```c
// nRF5340 has 8 RAM sections that can be powered independently
void configure_ram_retention(bool entering_sleep) {
    if (entering_sleep) {
        // Power down unused RAM sections
        // Keep only section 0-1 for critical data
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

// Mark critical data for retention
__attribute__((section(".retained_data")))
static struct {
    uint32_t wake_count;
    uint32_t last_activity_time;
    device_context_t last_context;
    uint8_t calibration_data[256];
} retained_data;
```

## 4. Peripheral Power Management

```c
// Disable unused peripherals
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

// Clock management
void configure_clocks_for_power_mode(power_mode_t mode) {
    switch (mode) {
        case POWER_MODE_SLEEP:
            // Use only 32kHz crystal
            nrf_clock_lf_src_set(NRF_CLOCK, NRF_CLOCK_LFCLK_XTAL);
            // Stop HFCLK
            nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTOP);
            break;
            
        case POWER_MODE_IDLE:
            // Use internal RC for HFCLK (less accurate but lower power)
            nrf_clock_hf_src_set(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY);
            break;
            
        case POWER_MODE_RUNNING:
            // Use external crystal for accuracy
            nrf_clock_hf_src_set(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY);
            nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
            break;
    }
}
```

## 5. Network Core Power Management

```c
// Control network core for BLE
void manage_network_core_power(bool ble_needed) {
    if (!ble_needed) {
        // Stop network core to save power
        LOG_INF("Stopping network core");
        
        // Disable BLE
        bt_disable();
        
        // Force network core to sleep
        nrf_reset_network_force_off(NRF_RESET, true);
        
        // Saves ~5mA when BLE not needed
    } else {
        // Wake network core
        LOG_INF("Starting network core");
        
        nrf_reset_network_force_off(NRF_RESET, false);
        
        // Re-initialize BLE
        bt_enable(NULL);
    }
}
```

## 6. System OFF Mode Implementation

```c
// Enter deepest sleep mode (0.4µA)
void enter_system_off_mode(void) {
    LOG_INF("Entering System OFF mode");
    
    // Save critical data to retained RAM
    retained_data.wake_count++;
    retained_data.last_activity_time = k_uptime_get_32();
    
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

// Check wake reason on boot
void check_wake_reason(void) {
    uint32_t reset_reason = nrf_power_resetreas_get(NRF_POWER);
    nrf_power_resetreas_clear(NRF_POWER, reset_reason);
    
    if (reset_reason & NRF_POWER_RESETREAS_OFF_MASK) {
        LOG_INF("Woke from System OFF, count: %d", retained_data.wake_count);
        // Fast path initialization
        quick_wake_init();
    } else if (reset_reason & NRF_POWER_RESETREAS_RESETPIN_MASK) {
        LOG_INF("Reset pin pressed");
    } else {
        LOG_INF("Normal power on");
        // Full initialization
    }
}
```

## 7. Zephyr Power Management Integration

```c
// Implement Zephyr PM hooks
static int pm_suspend_hook(const struct device *dev) {
    ARG_UNUSED(dev);
    
    // Called before entering low power state
    suspend_non_critical_threads();
    disable_unused_peripherals();
    
    return 0;
}

static int pm_resume_hook(const struct device *dev) {
    ARG_UNUSED(dev);
    
    // Called after waking
    resume_all_threads();
    
    return 0;
}

// Register PM hooks
SYS_INIT(pm_suspend_hook, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
SYS_INIT(pm_resume_hook, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

// Configure PM policy
void configure_pm_policy(void) {
    // Allow deep sleep states
    pm_policy_state_lock_put(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
    
    // Set latency constraints
    struct pm_policy_latency_request req = {
        .state = PM_STATE_RUNTIME_IDLE,
        .value_us = 100,  // Max 100µs latency
    };
    pm_policy_latency_request_add(&req);
}
```

## 8. Advanced Wake Strategies

```c
// Use PPI/DPPI for hardware-triggered wake
void configure_hardware_wake_chain(void) {
    // Configure DPPI to wake on multiple conditions
    nrfx_dppi_channel_t dppi_channel;
    nrfx_dppi_channel_alloc(&dppi_channel);
    
    // Link timer compare to wake event
    nrf_timer_publish_set(NRF_TIMER0, 
                         NRF_TIMER_EVENT_COMPARE0, 
                         dppi_channel);
    
    // Link to power management
    nrf_power_subscribe_set(NRF_POWER,
                           NRF_POWER_TASK_CONSTLAT,
                           dppi_channel);
    
    nrfx_dppi_channel_enable(dppi_channel);
}

// RTC wake for periodic tasks
void configure_rtc_wake(uint32_t seconds) {
    // Use RTC for ultra-low power timing
    NRF_RTC0->PRESCALER = 32;  // 1kHz tick
    NRF_RTC0->CC[0] = seconds * 1000;
    NRF_RTC0->INTENSET = RTC_INTENSET_COMPARE0_Msk;
    NRF_RTC0->TASKS_START = 1;
    
    // RTC runs in all power modes
    NVIC_EnableIRQ(RTC0_IRQn);
}
```

## Power Optimization Checklist

### 1. **GPIO Configuration**
- [ ] All unused pins set to disconnected
- [ ] Wake pins configured with SENSE
- [ ] Pull resistors optimized

### 2. **Clock Management**
- [ ] HFCLK stopped when not needed
- [ ] Using 32kHz crystal for timing
- [ ] Clock sources optimized per mode

### 3. **RAM Management**
- [ ] Unused RAM sections powered down
- [ ] Critical data in retained section
- [ ] Stack sizes optimized

### 4. **Thread Management**
- [ ] Non-critical threads suspended
- [ ] Priorities adjusted for power
- [ ] Idle threads use k_sleep()

### 5. **Peripheral Control**
- [ ] Unused peripherals disabled
- [ ] DMA used where possible
- [ ] Interrupts instead of polling

### 6. **Network Core**
- [ ] Disabled when BLE not needed
- [ ] Optimized connection parameters
- [ ] Advertising intervals adjusted

## Expected Power Consumption

| Mode | Configuration | Current |
|------|--------------|---------|
| System OFF | GPIO wake only | 0.4µA |
| Deep Sleep | RTC + 1 GPIO | 1.5µA |
| Idle | RTC + RAM retention | 7.5µA |
| BLE Advertising | 1s interval | 20µA avg |
| Active Sensing | 50Hz IMU | 3-5mA |
| Full Operation | All features | 10-15mA |

With proper implementation, the nRF5340 can achieve incredibly low power consumption while maintaining responsive wake capabilities!
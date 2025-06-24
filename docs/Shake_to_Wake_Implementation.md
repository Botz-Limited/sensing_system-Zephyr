# Shake-to-Wake Implementation Guide

Hey team! Here's how we can implement a shake-to-wake feature for our buttonless shoe device.

## Overview

Since we don't have buttons, we need a reliable way to wake the device from deep sleep. The solution: use the BHI360's built-in activity detection in ultra-low power mode to detect when someone picks up or shakes the shoe.

Additionally, the device needs to automatically enter deep sleep after being idle for a reasonable time (20 minutes) to conserve battery.

## Auto-Sleep Implementation

### 1. Idle Detection and Sleep Timer

```c
// Global idle tracking
typedef struct {
    uint32_t last_activity_time;
    uint32_t idle_start_time;
    bool is_idle;
    k_timer sleep_timer;
    uint8_t consecutive_idle_checks;
} idle_tracker_t;

static idle_tracker_t idle_tracker;

// Initialize idle tracking
void init_idle_detection(void) {
    idle_tracker.last_activity_time = k_uptime_get_32();
    idle_tracker.is_idle = false;
    idle_tracker.consecutive_idle_checks = 0;
    
    // Initialize sleep timer
    k_timer_init(&idle_tracker.sleep_timer, auto_sleep_timeout, NULL);
}

// Called from sensor processing
void update_activity_status(float motion_magnitude, uint16_t pressure_sum) {
    uint32_t current_time = k_uptime_get_32();
    
    // Define what constitutes "activity"
    bool has_motion = (motion_magnitude > 50.0f);  // 50mg threshold
    bool has_pressure = (pressure_sum > 100);       // Foot in shoe
    
    if (has_motion || has_pressure) {
        // Activity detected
        idle_tracker.last_activity_time = current_time;
        idle_tracker.consecutive_idle_checks = 0;
        
        if (idle_tracker.is_idle) {
            LOG_INF("Activity resumed, canceling sleep timer");
            k_timer_stop(&idle_tracker.sleep_timer);
            idle_tracker.is_idle = false;
        }
    } else {
        // No activity
        uint32_t idle_duration = current_time - idle_tracker.last_activity_time;
        
        if (!idle_tracker.is_idle && idle_duration > 30000) {  // 30 seconds
            idle_tracker.consecutive_idle_checks++;
            
            // Need multiple consecutive checks to confirm idle
            if (idle_tracker.consecutive_idle_checks >= 3) {
                LOG_INF("Device idle for %d seconds, starting sleep timer", 
                       idle_duration / 1000);
                idle_tracker.is_idle = true;
                idle_tracker.idle_start_time = current_time;
                
                // Start 20-minute timer (minus time already idle)
                uint32_t remaining_time = (20 * 60 * 1000) - idle_duration;
                if (remaining_time > 0) {
                    k_timer_start(&idle_tracker.sleep_timer, 
                                 K_MSEC(remaining_time), K_NO_WAIT);
                } else {
                    // Already past 20 minutes, sleep immediately
                    auto_sleep_timeout(&idle_tracker.sleep_timer);
                }
            }
        }
    }
}

// Timer callback for auto-sleep
void auto_sleep_timeout(struct k_timer *timer) {
    LOG_INF("Auto-sleep timeout reached, entering deep sleep");
    
    // Final check - make sure we're still idle
    if (idle_tracker.is_idle) {
        uint32_t total_idle_time = k_uptime_get_32() - idle_tracker.idle_start_time;
        LOG_INF("Device idle for %d minutes, entering deep sleep", 
               total_idle_time / 60000);
        
        // Notify any connected devices
        if (is_ble_connected()) {
            send_sleep_notification();
            k_msleep(500);  // Give time for notification to send
        }
        
        enter_deep_sleep_mode();
    }
}
```

### 2. Progressive Power Reduction While Idle

```c
// Gradually reduce power consumption as idle time increases
void progressive_idle_power_management(void) {
    uint32_t idle_duration = k_uptime_get_32() - idle_tracker.last_activity_time;
    
    if (idle_duration < 30000) {  // < 30 seconds
        // Normal operation
        return;
    } else if (idle_duration < 300000) {  // 30s - 5 minutes
        // Stage 1: Reduce sampling rates
        if (current_power_mode != POWER_MODE_IDLE) {
            LOG_INF("Idle Stage 1: Reducing sampling rates");
            set_power_mode(POWER_MODE_IDLE);
            
            // Reduce BLE advertising
            set_slow_advertising();
        }
    } else if (idle_duration < 600000) {  // 5-10 minutes
        // Stage 2: Disable non-essential sensors
        LOG_INF("Idle Stage 2: Disabling pressure sensors");
        disable_pressure_sensors();
        
        // Further reduce IMU rate
        bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, 5.0f, 1000, bhy2_ptr);
        
    } else if (idle_duration < 1200000) {  // 10-20 minutes
        // Stage 3: Minimal operation
        LOG_INF("Idle Stage 3: Minimal sensors only");
        
        // Keep only motion detection active
        bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, 0, 0, bhy2_ptr);
        bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, 0, 0, bhy2_ptr);
        bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ANY_MOTION, 5.0f, 0, bhy2_ptr);
        
        // Stop BLE advertising
        bt_le_adv_stop();
    }
    // At 20 minutes: auto_sleep_timeout() triggers deep sleep
}
```

### 3. Smart Idle Detection

```c
// More sophisticated idle detection considering context
typedef enum {
    CONTEXT_UNKNOWN,
    CONTEXT_WORN_ACTIVE,     // Shoe on foot, user active
    CONTEXT_WORN_RESTING,    // Shoe on foot, user sitting/standing
    CONTEXT_NOT_WORN,        // Shoe off foot
    CONTEXT_CHARGING         // Connected to charger
} device_context_t;

device_context_t determine_device_context(void) {
    // Check if charging
    if (is_charging()) {
        return CONTEXT_CHARGING;
    }
    
    // Check pressure to see if worn
    uint16_t avg_pressure = get_average_pressure();
    if (avg_pressure < 50) {
        return CONTEXT_NOT_WORN;
    }
    
    // Check motion to determine if active
    float motion_variance = get_motion_variance();
    if (motion_variance > 100.0f) {
        return CONTEXT_WORN_ACTIVE;
    } else {
        return CONTEXT_WORN_RESTING;
    }
}

// Adjust idle timeout based on context
uint32_t get_idle_timeout_for_context(device_context_t context) {
    switch (context) {
        case CONTEXT_NOT_WORN:
            return 20 * 60 * 1000;  // 20 minutes
            
        case CONTEXT_WORN_RESTING:
            return 60 * 60 * 1000;  // 1 hour (user might be sitting)
            
        case CONTEXT_WORN_ACTIVE:
            return 0;  // Don't sleep while active
            
        case CONTEXT_CHARGING:
            return 0;  // Don't sleep while charging
            
        default:
            return 20 * 60 * 1000;  // Default 20 minutes
    }
}
```

### 4. Integration with Main Loop

```c
// In motion_sensor_process() or main loop
void motion_sensor_process(void *, void *, void *) {
    // ... existing code ...
    
    while (true) {
        // Process sensor data
        process_sensor_data();
        
        // Update activity tracking
        float motion_mag = get_current_motion_magnitude();
        uint16_t pressure = get_current_pressure_sum();
        update_activity_status(motion_mag, pressure);
        
        // Progressive power management
        progressive_idle_power_management();
        
        // Check if we should adjust idle timeout based on context
        static uint32_t last_context_check = 0;
        if (k_uptime_get_32() - last_context_check > 30000) {  // Every 30s
            device_context_t context = determine_device_context();
            uint32_t timeout = get_idle_timeout_for_context(context);
            
            if (timeout > 0 && idle_tracker.is_idle) {
                // Restart timer with context-appropriate timeout
                k_timer_start(&idle_tracker.sleep_timer, K_MSEC(timeout), K_NO_WAIT);
            }
            last_context_check = k_uptime_get_32();
        }
        
        k_msleep(100);  // Main loop delay
    }
}
```

## Wake-Up Detection Strategy

### 1. Configure BHI360 for Ultra-Low Power Motion Detection

```c
// In motion_sensor.cpp - Configure before entering deep sleep
void configure_shake_wake_detection(void) {
    int8_t rslt;
    
    // First, disable all high-power sensors
    bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, 0, 0, bhy2_ptr);
    bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, 0, 0, bhy2_ptr);
    bhy2_set_virt_sensor_cfg(GYRO_SENSOR_ID, 0, 0, bhy2_ptr);
    
    // Configure activity recognition in low power mode
    struct bhy2_sensor_config config = {
        .sensor_id = BHY2_SENSOR_ID_ANY_MOTION,
        .sample_rate = 25.0f,  // 25Hz is enough for shake detection
        .latency = 0
    };
    
    // Set up any-motion detection with higher threshold for shake
    struct bhy2_any_motion_config shake_config = {
        .threshold = 50,        // ~50mg - requires deliberate movement
        .duration = 3,          // 3 samples @ 25Hz = 120ms
        .axes_enabled = 0x07    // Monitor all axes
    };
    
    rslt = bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_ANY_MOTION, 
                                   config.sample_rate, 
                                   config.latency, 
                                   bhy2_ptr);
    
    // Also enable significant motion for backup
    rslt |= bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_SIG_MOTION, 
                                     1.0f,  // 1Hz check rate
                                     0, 
                                     bhy2_ptr);
    
    // Configure interrupt to wake MCU
    uint8_t hintr_ctrl = BHY2_HINTR_CTRL_ASYNC_STATUS_CHANNEL;
    rslt |= bhy2_set_host_interrupt_ctrl(hintr_ctrl, bhy2_ptr);
    
    LOG_INF("Shake-wake detection configured, entering deep sleep mode");
}
```

### 2. Implement Shake Pattern Recognition

```c
// More sophisticated shake detection to avoid false wakes
typedef struct {
    uint32_t shake_start_time;
    uint8_t shake_count;
    bool shake_detected;
    float last_magnitude;
    uint32_t last_motion_time;
} shake_detector_t;

static shake_detector_t shake_detector = {0};

bool detect_wake_shake(float x, float y, float z) {
    // Calculate magnitude of acceleration change
    float magnitude = sqrtf(x*x + y*y + z*z);
    float delta = fabsf(magnitude - shake_detector.last_magnitude);
    shake_detector.last_magnitude = magnitude;
    
    uint32_t current_time = k_uptime_get_32();
    
    // Detect significant acceleration change (shake)
    if (delta > 100.0f) {  // 100mg change
        uint32_t time_since_last = current_time - shake_detector.last_motion_time;
        
        // If motion within 500ms of last, it's part of shake pattern
        if (time_since_last < 500) {
            shake_detector.shake_count++;
            
            // 3 shakes within 2 seconds = wake up
            if (shake_detector.shake_count >= 3) {
                uint32_t shake_duration = current_time - shake_detector.shake_start_time;
                if (shake_duration < 2000) {
                    shake_detector.shake_detected = true;
                    LOG_INF("Wake shake detected! Count: %d, Duration: %dms", 
                           shake_detector.shake_count, shake_duration);
                    return true;
                }
            }
        } else {
            // New shake sequence
            shake_detector.shake_count = 1;
            shake_detector.shake_start_time = current_time;
        }
        
        shake_detector.last_motion_time = current_time;
    }
    
    // Reset if too much time passed
    if (current_time - shake_detector.last_motion_time > 2000) {
        shake_detector.shake_count = 0;
        shake_detector.shake_detected = false;
    }
    
    return false;
}
```

### 3. Deep Sleep Entry with Wake Configuration

```c
// Modified deep sleep entry
void enter_deep_sleep_mode(void) {
    LOG_INF("Entering deep sleep after 20 minutes idle");
    
    // Save current state
    save_device_state();
    
    // Configure shake-wake detection
    configure_shake_wake_detection();
    
    // Disable pressure sensors completely
    disable_pressure_sensors();
    nrfx_saadc_uninit();
    
    // Configure GPIO for lowest power
    configure_gpios_for_sleep();
    
    // Set up wake interrupt from BHI360
    nrfx_gpiote_in_config_t config = {
        .sense = NRF_GPIOTE_POLARITY_LOTOHI,
        .pull = NRF_GPIO_PIN_PULLDOWN,
        .is_watcher = false,
        .hi_accuracy = false,  // Low power mode
        .skip_gpio_setup = false
    };
    
    nrfx_gpiote_in_init(BHI360_INT_PIN, &config, bhi360_wake_handler);
    nrfx_gpiote_in_event_enable(BHI360_INT_PIN, true);
    
    // Enter system off mode (lowest power)
    // MCU will restart on interrupt
    sys_poweroff();
}
```

### 4. Wake-Up Handler

```c
// This runs after MCU wakes from deep sleep
void handle_wake_from_sleep(void) {
    LOG_INF("Device waking from deep sleep");
    
    // Quick check if this is a real wake event
    uint8_t work_buffer[256];
    int8_t rslt = bhy2_get_and_process_fifo(work_buffer, sizeof(work_buffer), bhy2_ptr);
    
    // Check if we got valid shake data
    if (!shake_detector.shake_detected) {
        LOG_WRN("False wake, going back to sleep");
        enter_deep_sleep_mode();
        return;
    }
    
    // Real wake - start initialization sequence
    LOG_INF("Valid shake detected, initializing device");
    
    // Visual/haptic feedback that device is waking
    indicate_wake_up();
    
    // Restore normal operation
    restore_normal_operation();
}

void restore_normal_operation(void) {
    // Re-initialize sensors
    init_saadc();  // Pressure sensors
    
    // Restore BHI360 to normal mode
    bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, 50.0f, 0, bhy2_ptr);
    bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, 50.0f, 0, bhy2_ptr);
    bhy2_set_virt_sensor_cfg(GYRO_SENSOR_ID, 50.0f, 0, bhy2_ptr);
    
    // Start in idle mode
    set_power_mode(POWER_MODE_IDLE);
    
    // Re-enable BLE advertising
    start_advertising();
    
    LOG_INF("Device fully operational");
}
```

### 5. Alternative Wake Methods

```c
// Multiple wake triggers for reliability
typedef enum {
    WAKE_TRIGGER_SHAKE = 0x01,
    WAKE_TRIGGER_PICKUP = 0x02,
    WAKE_TRIGGER_STEP = 0x04,
    WAKE_TRIGGER_BLE_SCAN = 0x08
} wake_trigger_t;

// Pickup detection (orientation change)
bool detect_pickup(float qw, float qx, float qy, float qz) {
    static float last_qw = 1.0f, last_qx = 0, last_qy = 0, last_qz = 0;
    
    // Calculate quaternion difference
    float dot = qw*last_qw + qx*last_qx + qy*last_qy + qz*last_qz;
    float angle = acosf(fabsf(dot)) * 2.0f * 180.0f / M_PI;
    
    // Store current as last
    last_qw = qw; last_qx = qx; last_qy = qy; last_qz = qz;
    
    // Significant orientation change = picked up
    return (angle > 30.0f);  // 30 degree change
}

// Step detection for "put on and walk" scenario
bool detect_step_pattern(float acc_magnitude) {
    static uint8_t potential_steps = 0;
    static uint32_t last_peak_time = 0;
    
    // Simple peak detection
    if (acc_magnitude > 1200.0f) {  // 1.2g threshold
        uint32_t current_time = k_uptime_get_32();
        uint32_t time_diff = current_time - last_peak_time;
        
        // Steps typically 300-1000ms apart
        if (time_diff > 300 && time_diff < 1000) {
            potential_steps++;
            if (potential_steps >= 3) {
                LOG_INF("Step pattern detected, waking device");
                return true;
            }
        } else {
            potential_steps = 1;  // Reset counter
        }
        last_peak_time = current_time;
    }
    
    return false;
}
```

### 6. Power Consumption in Sleep

```c
// Ultra-low power configuration
void configure_system_for_minimum_power(void) {
    // Disable all unnecessary peripherals
    NRF_UARTE0->ENABLE = 0;
    NRF_SPIM0->ENABLE = 0;
    NRF_TWIM0->ENABLE = 0;
    
    // Configure all unused GPIOs as disconnected
    for (int i = 0; i < 32; i++) {
        if (i != BHI360_INT_PIN) {
            nrf_gpio_cfg_default(i);
        }
    }
    
    // Disable RAM retention for unused sections
    // Keep only essential data
    NRF_VMC->RAM[0].POWER = 0x0000FFFF;  // Only first 64KB
    
    // Expected current in deep sleep:
    // - MCU: ~1µA
    // - BHI360 in motion detect: ~10µA  
    // - Total system: ~15-20µA
    // 
    // With 100mAh battery: 
    // Sleep life = 100mAh / 0.02mA = 5000 hours = 208 days
}
```

### 7. User Experience Considerations

```c
// Provide feedback when waking
void indicate_wake_up(void) {
    // If we have an LED
    #ifdef CONFIG_HAS_LED
    for (int i = 0; i < 3; i++) {
        gpio_pin_set(led_dev, LED_PIN, 1);
        k_msleep(100);
        gpio_pin_set(led_dev, LED_PIN, 0);
        k_msleep(100);
    }
    #endif
    
    // If we have a vibration motor
    #ifdef CONFIG_HAS_VIBRATION
    vibrate_pattern(PATTERN_WAKE_UP);
    #endif
    
    // Send BLE advertisement with "WAKING" flag
    set_device_name("Shoe-L-WAKE");
    start_fast_advertising();  // 20ms interval for 5 seconds
}

// Smooth transition from sleep to active
void graduated_wake_sequence(void) {
    // Stage 1: Minimal sensors (1 second)
    set_power_mode(POWER_MODE_IDLE);
    k_msleep(1000);
    
    // Stage 2: Check if still moving
    if (is_motion_continuing()) {
        // Stage 3: Full sensors
        set_power_mode(POWER_MODE_WALKING);
    } else {
        // False wake - go back to sleep
        LOG_INF("No continued motion, returning to sleep");
        enter_deep_sleep_mode();
    }
}
```

## Testing the Wake Mechanism

```c
// Test mode for shake detection tuning
#ifdef CONFIG_SHAKE_WAKE_TEST_MODE
void test_shake_sensitivity(void) {
    LOG_INF("=== Shake Wake Test Mode ===");
    LOG_INF("Shake the device 3 times to trigger wake");
    
    // Log all motion events for tuning
    while (1) {
        float x, y, z;
        if (get_accel_data(&x, &y, &z)) {
            float magnitude = sqrtf(x*x + y*y + z*z);
            if (magnitude > 1100.0f) {  // Log significant motion
                LOG_INF("Motion: mag=%.1f, x=%.1f, y=%.1f, z=%.1f", 
                       magnitude, x, y, z);
            }
        }
        k_msleep(40);  // 25Hz
    }
}
#endif
```

## Configuration Options

```kconfig
menu "Wake Detection"

config SHAKE_WAKE_ENABLED
    bool "Enable shake to wake"
    default y

config SHAKE_COUNT_THRESHOLD
    int "Number of shakes to wake"
    default 3
    range 2 5

config SHAKE_TIMEOUT_MS
    int "Maximum time for shake sequence (ms)"
    default 2000
    range 1000 3000

config PICKUP_WAKE_ENABLED
    bool "Enable pickup detection"
    default y
    help
      Wake when orientation changes significantly

config STEP_WAKE_ENABLED  
    bool "Enable step pattern wake"
    default y
    help
      Wake when user starts walking

endmenu
```

## Complete Sleep/Wake Cycle

### Typical User Journey:

1. **Morning**: User puts on shoes
   - Device wakes from shake/pickup
   - Enters idle mode, starts monitoring
   
2. **Activity**: User runs/walks
   - Full sensor operation
   - Real-time data transmission
   
3. **Rest**: User sits at desk
   - Progressive power reduction
   - After 20 minutes idle → deep sleep
   
4. **Evening**: Shoes removed
   - Quick transition to deep sleep
   - ~20µA power consumption
   
5. **Next Day**: Pick up shoes
   - Shake to wake
   - Ready for new day

### Power Budget Summary:

| State | Duration/Day | Current | Energy |
|-------|--------------|---------|---------|
| Deep Sleep | 20 hours | 20µA | 0.4mAh |
| Idle | 2 hours | 2mA | 4mAh |
| Walking | 1 hour | 10mA | 10mAh |
| Running | 1 hour | 20mA | 20mAh |
| **Total** | **24 hours** | **Avg: 1.4mA** | **34.4mAh** |

With a 100mAh battery, this gives approximately **3 days** of typical use!

### Key Features:

1. **Automatic Sleep**:
   - 20 minutes idle → deep sleep
   - Progressive power reduction
   - Context-aware timeouts

2. **Easy Wake**:
   - Shake 3 times
   - Pick up (orientation change)
   - Start walking

3. **Smart Detection**:
   - Validates patterns to prevent false wakes
   - Adjusts timeouts based on context
   - Smooth transitions between states

This implementation provides a seamless, buttonless experience while maximizing battery life!
# Lab vs User Build Configuration

Hey team! Here's how we can handle the dual-purpose firmware for lab testing vs user/production builds.

## Overview

We'll use Kconfig (Zephyr's configuration system) to create build variants from the same codebase. This lets us maintain one firmware but compile different versions for different use cases.

## Proposed Configuration Structure

### 1. Main Build Type Selection

In `Kconfig` (root or app-specific):

```kconfig
choice DEVICE_BUILD_TYPE
    prompt "Device Build Type"
    default BUILD_TYPE_LAB
    help
      Select the build type for the device firmware

    config BUILD_TYPE_LAB
        bool "Lab/Debug Build"
        help
          Full featured build for lab testing and development.
          Includes raw data logging, debug output, and test commands.

    config BUILD_TYPE_USER
        bool "User/Production Build"
        help
          Optimized build for end users.
          Calculated metrics only, no raw data logging.

endchoice
```

### 2. Feature Flags Based on Build Type

```kconfig
# Automatically set features based on build type
config ENABLE_RAW_DATA_LOGGING
    bool
    default y if BUILD_TYPE_LAB
    default n if BUILD_TYPE_USER

config ENABLE_DEBUG_COMMANDS
    bool
    default y if BUILD_TYPE_LAB
    default n if BUILD_TYPE_USER

config ENABLE_PROTOBUF_LOGGING
    bool
    default y if BUILD_TYPE_LAB
    default n if BUILD_TYPE_USER

config ENABLE_CALCULATED_METRICS_ONLY
    bool
    default n if BUILD_TYPE_LAB
    default y if BUILD_TYPE_USER

config ENABLE_POWER_OPTIMIZATION
    bool
    default n if BUILD_TYPE_LAB
    default y if BUILD_TYPE_USER

config LOG_LEVEL_DEFAULT
    int
    default 4 if BUILD_TYPE_LAB  # Debug level
    default 2 if BUILD_TYPE_USER # Warning level
```

## Implementation in Code

### 1. In Data Module (`data.cpp`)

```c
void handle_bhi360_log_record(const bhi360_log_record_t* record) {
    #if CONFIG_ENABLE_RAW_DATA_LOGGING
        // Lab version: Log raw sensor data with protobuf
        log_raw_bhi360_protobuf(record);
    #endif
    
    #if CONFIG_ENABLE_CALCULATED_METRICS_ONLY
        // User version: Calculate and log metrics only
        calculate_and_log_activity_metrics(record);
    #endif
}
```

### 2. In Motion Sensor (`motion_sensor.cpp`)

```c
static void parse_all_sensors(...) {
    // Always calculate basic values
    process_sensor_data();
    
    #if CONFIG_ENABLE_RAW_DATA_LOGGING
        // Lab: Send everything
        send_raw_data_to_logger();
    #else
        // User: Send only if activity is active
        if (atomic_get(&activity_logging_active)) {
            send_calculated_metrics();
        }
    #endif
}
```

### 3. In BLE Services

```c
void init_ble_services() {
    // Always initialize core services
    init_device_info_service();
    init_control_service();
    
    #if CONFIG_BUILD_TYPE_LAB
        // Lab-only services
        init_raw_data_streaming_service();
        init_debug_command_service();
    #endif
    
    #if CONFIG_BUILD_TYPE_USER
        // User-only services
        init_activity_metrics_service();
        init_coaching_alerts_service();
    #endif
}
```

## Build Commands

### For Lab Build:
```bash
west build -b nrf5340dk_nrf5340_cpuapp -- -DCONFIG_BUILD_TYPE_LAB=y
```

### For User Build:
```bash
west build -b nrf5340dk_nrf5340_cpuapp -- -DCONFIG_BUILD_TYPE_USER=y
```

### Or use build configurations:
Create `build_lab.conf`:
```
CONFIG_BUILD_TYPE_LAB=y
```

Create `build_user.conf`:
```
CONFIG_BUILD_TYPE_USER=y
```

Then build with:
```bash
west build -b nrf5340dk_nrf5340_cpuapp -- -DOVERLAY_CONFIG=build_user.conf
```

## Key Differences Between Builds

### Lab Build:
- **Raw sensor data logging** (31.7 MB/hour)
- **Protobuf format** for flexibility
- **All debug commands** available
- **Verbose logging** (LOG_LEVEL_DBG)
- **No power optimization** (always on)
- **Test modes** (calibration, sensor test)
- **Larger flash usage** (~500KB)

### User Build:
- **Calculated metrics only** (45 KB/hour)
- **Binary struct format** for efficiency
- **Limited commands** (start/stop activity)
- **Minimal logging** (LOG_LEVEL_WRN)
- **Power optimized** (adaptive sampling)
- **No test modes**
- **Smaller flash usage** (~300KB)

## Memory Impact

```c
// Conditional compilation saves memory
#if CONFIG_ENABLE_RAW_DATA_LOGGING
    static uint8_t protobuf_buffer[PROTOBUF_BUFFER_SIZE];  // 32KB
    static RawDataLogger raw_logger;                       // 8KB
#endif

#if CONFIG_ENABLE_CALCULATED_METRICS_ONLY
    static uint8_t metrics_buffer[METRICS_BUFFER_SIZE];    // 4KB
    static MetricsCalculator calculator;                   // 2KB
#endif
```

## Testing Both Builds

We should set up CI to build and test both variants:

```yaml
# .github/workflows/build.yml
strategy:
  matrix:
    build_type: [lab, user]
    
steps:
  - name: Build ${{ matrix.build_type }}
    run: |
      west build -b nrf5340dk_nrf5340_cpuapp -- \
        -DCONFIG_BUILD_TYPE_${{ matrix.build_type | upper }}=y
```

## Migration Path

1. **Phase 1**: Add Kconfig options but keep current behavior
2. **Phase 2**: Start using flags in new code
3. **Phase 3**: Refactor existing code to respect flags
4. **Phase 4**: Create separate build configs
5. **Phase 5**: Test and validate both builds

## Additional Considerations

### Version Identification
```c
const char* get_build_type(void) {
    #if CONFIG_BUILD_TYPE_LAB
        return "LAB";
    #else
        return "USER";
    #endif
}
```

### BLE Advertising
The device should advertise its build type so the mobile app knows what features are available:
```c
// In manufacturer data or device name
"SensingFW-LAB" vs "SensingFW-USER"
```

### Feature Discovery
Mobile app can query available features:
```c
uint32_t get_feature_flags(void) {
    uint32_t flags = 0;
    #if CONFIG_ENABLE_RAW_DATA_LOGGING
        flags |= FEATURE_RAW_DATA;
    #endif
    #if CONFIG_ENABLE_CALCULATED_METRICS_ONLY
        flags |= FEATURE_ACTIVITY_METRICS;
    #endif
    return flags;
}
```

This approach gives us maximum flexibility while maintaining a single codebase. We can easily add more build types later (like FACTORY_TEST or DEMO) if needed!
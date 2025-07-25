# Firmware Requirements for Mobile Background Execution Support

## Overview

This document outlines the firmware changes needed to support mobile apps running in background mode during long fitness activities. The firmware must adapt to different connection scenarios to optimize power consumption while maintaining reliable data collection.

## Current Firmware Configuration

### Connection Parameters (from prj.conf)
```
CONFIG_BT_PERIPHERAL_PREF_MIN_INT=24  # 30ms (24 * 1.25ms)
CONFIG_BT_PERIPHERAL_PREF_MAX_INT=40  # 50ms (40 * 1.25ms)
CONFIG_BT_PERIPHERAL_PREF_LATENCY=0   # No latency
CONFIG_BT_PERIPHERAL_PREF_TIMEOUT=42  # 420ms supervision timeout
```

These parameters are suitable for foreground operation but not optimized for background scenarios.

## Required Firmware Changes

### 1. Dynamic Connection Parameter Updates

#### Implementation: Connection Parameter Request Handler

```c
// In bluetooth.cpp or new file: ble_conn_params.cpp

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

// Connection parameter profiles
typedef enum {
    CONN_PROFILE_FOREGROUND,      // High performance
    CONN_PROFILE_BACKGROUND,      // Power saving
    CONN_PROFILE_BACKGROUND_IDLE  // Maximum power saving
} conn_profile_t;

// Connection parameter sets
static const struct bt_le_conn_param conn_params[] = {
    [CONN_PROFILE_FOREGROUND] = {
        .interval_min = 12,  // 15ms
        .interval_max = 24,  // 30ms
        .latency = 0,
        .timeout = 400      // 4s
    },
    [CONN_PROFILE_BACKGROUND] = {
        .interval_min = 40,  // 50ms
        .interval_max = 80,  // 100ms
        .latency = 4,       // Allow 4 missed intervals
        .timeout = 600      // 6s
    },
    [CONN_PROFILE_BACKGROUND_IDLE] = {
        .interval_min = 160, // 200ms
        .interval_max = 400, // 500ms
        .latency = 10,      // Allow 10 missed intervals
        .timeout = 1000     // 10s
    }
};

static conn_profile_t current_profile = CONN_PROFILE_FOREGROUND;

int ble_update_conn_params(struct bt_conn *conn, conn_profile_t profile)
{
    if (profile >= ARRAY_SIZE(conn_params)) {
        return -EINVAL;
    }
    
    int err = bt_conn_le_param_update(conn, &conn_params[profile]);
    if (err) {
        LOG_ERR("Failed to update connection parameters: %d", err);
        return err;
    }
    
    current_profile = profile;
    LOG_INF("Connection parameters updated to profile: %d", profile);
    return 0;
}
```

### 2. Control Service Extension

Add a new characteristic to the Control Service for mobile app to request connection parameter changes:

```c
// In control_service.cpp

// New UUID for connection parameter control
static struct bt_uuid_128 conn_param_control_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x4fd5b68b, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Handler for connection parameter control writes
static ssize_t conn_param_control_write(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       const void *buf, uint16_t len,
                                       uint16_t offset, uint8_t flags)
{
    if (len != 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint8_t profile = *((uint8_t *)buf);
    
    // Validate profile value
    if (profile > CONN_PROFILE_BACKGROUND_IDLE) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    // Update connection parameters
    int err = ble_update_conn_params(conn, (conn_profile_t)profile);
    if (err) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY_ERROR);
    }
    
    return len;
}

// Add to service definition
BT_GATT_CHARACTERISTIC(&conn_param_control_uuid.uuid,
                      BT_GATT_CHRC_WRITE,
                      BT_GATT_PERM_WRITE_ENCRYPT,
                      NULL, conn_param_control_write, NULL),
```

### 3. Adaptive Data Rate Control

Implement data rate adaptation based on connection profile:

```c
// In information_service.cpp or sensor modules

typedef struct {
    uint32_t foot_sensor_interval_ms;
    uint32_t motion_sensor_interval_ms;
    bool aggregate_data;
} data_rate_config_t;

static const data_rate_config_t data_configs[] = {
    [CONN_PROFILE_FOREGROUND] = {
        .foot_sensor_interval_ms = 15,
        .motion_sensor_interval_ms = 30,
        .aggregate_data = false
    },
    [CONN_PROFILE_BACKGROUND] = {
        .foot_sensor_interval_ms = 50,
        .motion_sensor_interval_ms = 100,
        .aggregate_data = true
    },
    [CONN_PROFILE_BACKGROUND_IDLE] = {
        .foot_sensor_interval_ms = 200,
        .motion_sensor_interval_ms = 500,
        .aggregate_data = true
    }
};

void update_sensor_data_rates(conn_profile_t profile)
{
    const data_rate_config_t *config = &data_configs[profile];
    
    // Update foot sensor sampling rate
    foot_sensor_set_interval(config->foot_sensor_interval_ms);
    
    // Update motion sensor sampling rate
    motion_sensor_set_interval(config->motion_sensor_interval_ms);
    
    // Enable/disable data aggregation
    set_data_aggregation(config->aggregate_data);
    
    LOG_INF("Sensor data rates updated for profile %d", profile);
}
```

### 4. Data Aggregation for Background Mode

Implement data aggregation to reduce BLE traffic in background:

```c
// In ble_data_aggregator.cpp (new file)

typedef struct {
    foot_samples_t samples[4];  // Aggregate up to 4 samples
    uint8_t count;
    uint32_t timestamp;
} aggregated_foot_data_t;

typedef struct {
    bhi360_3d_mapping_t samples[2];  // Aggregate up to 2 samples
    uint8_t count;
    uint32_t timestamp;
} aggregated_motion_data_t;

static aggregated_foot_data_t foot_buffer;
static aggregated_motion_data_t motion_buffer;

void aggregate_foot_sample(const foot_samples_t *sample)
{
    if (!is_aggregation_enabled()) {
        // Direct notification in foreground mode
        jis_foot_sensor_notify(sample);
        return;
    }
    
    // Add to buffer
    memcpy(&foot_buffer.samples[foot_buffer.count++], sample, sizeof(foot_samples_t));
    
    // Send when buffer is full or timeout
    if (foot_buffer.count >= 4 || 
        (k_uptime_get_32() - foot_buffer.timestamp) > 1000) {
        send_aggregated_foot_data();
    }
}

void send_aggregated_foot_data(void)
{
    if (foot_buffer.count == 0) return;
    
    // Create aggregated packet with multiple samples
    // This reduces BLE overhead in background mode
    for (int i = 0; i < foot_buffer.count; i++) {
        jis_foot_sensor_notify(&foot_buffer.samples[i]);
    }
    
    foot_buffer.count = 0;
    foot_buffer.timestamp = k_uptime_get_32();
}
```

### 5. Connection Event Callbacks

Add callbacks to detect connection state changes:

```c
// In bluetooth.cpp

static void on_conn_param_updated(struct bt_conn *conn, uint16_t interval,
                                 uint16_t latency, uint16_t timeout)
{
    LOG_INF("Connection parameters updated: interval=%d, latency=%d, timeout=%d",
            interval, latency, timeout);
    
    // Adjust sensor rates based on new parameters
    if (interval > 80) {  // > 100ms interval
        update_sensor_data_rates(CONN_PROFILE_BACKGROUND_IDLE);
    } else if (interval > 30) {  // > 37.5ms interval
        update_sensor_data_rates(CONN_PROFILE_BACKGROUND);
    } else {
        update_sensor_data_rates(CONN_PROFILE_FOREGROUND);
    }
}

// Register callback
static struct bt_conn_cb conn_callbacks = {
    .le_param_updated = on_conn_param_updated,
    // ... other callbacks
};
```

### 6. Kconfig Updates

Add configuration options in `Kconfig`:

```kconfig
menu "Background Execution Support"

config BLE_BACKGROUND_SUPPORT
    bool "Enable background execution support"
    default y
    help
      Enable dynamic connection parameter updates and data rate
      adaptation for mobile app background execution.

config BLE_BACKGROUND_CONN_INTERVAL_MIN
    int "Background mode minimum connection interval (ms)"
    default 50
    range 30 200
    depends on BLE_BACKGROUND_SUPPORT

config BLE_BACKGROUND_CONN_INTERVAL_MAX
    int "Background mode maximum connection interval (ms)"
    default 100
    range 50 500
    depends on BLE_BACKGROUND_SUPPORT

config BLE_BACKGROUND_LATENCY
    int "Background mode slave latency"
    default 4
    range 0 10
    depends on BLE_BACKGROUND_SUPPORT

config BLE_DATA_AGGREGATION
    bool "Enable data aggregation in background mode"
    default y
    depends on BLE_BACKGROUND_SUPPORT
    help
      Aggregate multiple sensor samples into single BLE packets
      to reduce overhead in background mode.

endmenu
```

### 7. Update prj.conf

Add to `prj.conf`:

```conf
# Background execution support
CONFIG_BLE_BACKGROUND_SUPPORT=y
CONFIG_BLE_BACKGROUND_CONN_INTERVAL_MIN=50
CONFIG_BLE_BACKGROUND_CONN_INTERVAL_MAX=100
CONFIG_BLE_BACKGROUND_LATENCY=4
CONFIG_BLE_DATA_AGGREGATION=y

# Allow connection parameter updates from central
CONFIG_BT_GAP_AUTO_UPDATE_CONN_PARAMS=y
CONFIG_BT_GAP_PERIPHERAL_PREF_PARAMS=y

# Increase buffers for aggregated data
CONFIG_BT_L2CAP_TX_BUF_COUNT=16
CONFIG_BT_CONN_TX_MAX=32
```

## Testing Requirements

### 1. Connection Parameter Switching
- Test transitions between foreground/background/idle modes
- Verify data integrity during parameter changes
- Measure power consumption in each mode

### 2. Data Aggregation
- Verify aggregated packets are correctly parsed by mobile app
- Test buffer overflow handling
- Ensure sequence numbers work with aggregated data

### 3. Long Duration Testing
- Run 2+ hour activities with app in background
- Monitor for connection drops
- Verify data completeness

### 4. Platform-Specific Testing
- iOS: Test with Core Bluetooth background mode
- Android: Test with foreground service and Doze mode

## Implementation Priority

1. **Phase 1**: Connection parameter control characteristic
2. **Phase 2**: Dynamic sensor rate adjustment
3. **Phase 3**: Data aggregation for background mode
4. **Phase 4**: Automatic profile detection and switching

## Summary

The firmware needs to support three connection profiles:
- **Foreground**: 15-30ms interval, no latency (current)
- **Background Active**: 50-100ms interval, latency 4
- **Background Idle**: 200-500ms interval, latency 10

Mobile apps can request profile changes via a new control characteristic, and the firmware will automatically adjust sensor sampling rates and data aggregation to optimize for each mode.

This ensures reliable data collection during long activities while respecting mobile OS background execution constraints and optimizing battery life on both devices.
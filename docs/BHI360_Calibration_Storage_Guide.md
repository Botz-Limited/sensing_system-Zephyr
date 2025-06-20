# BHI360 Calibration Storage Guide

This guide explains how to save and load BHI360 calibration data using the littlefs filesystem.

## Overview

The BHI360 calibration data is stored in the littlefs partition under `/lfs1/calibration/`. Each sensor type (accelerometer, gyroscope, magnetometer) has its own calibration file.

## File Structure

```
/lfs1/calibration/
├── bhi360_calib_accel.bin   # Accelerometer calibration
└── bhi360_calib_gyro.bin    # Gyroscope calibration
```

## API Functions

### Storing Calibration Data

```c
#include <data.hpp>

// After performing calibration, get the profile from BHI360
uint8_t profile_data[512];
size_t actual_size;
int ret = bhi360_get_calibration_profile(bhi360_dev, BHI360_SENSOR_GYRO, 
                                        profile_data, sizeof(profile_data), 
                                        &actual_size);
if (ret == 0) {
    // Store the calibration data to flash
    err_t err = store_bhi360_calibration_data(1, // 1 = gyro
                                             profile_data, 
                                             actual_size);
    if (err == err_t::NO_ERROR) {
        LOG_INF("Calibration saved successfully");
    }
}
```

### Loading Calibration Data

```c
#include <data.hpp>

// During initialization, try to load saved calibration
uint8_t profile_data[512];
size_t actual_size;
err_t err = get_bhi360_calibration_data(1, // 1 = gyro
                                        profile_data, 
                                        sizeof(profile_data), 
                                        &actual_size);
if (err == err_t::NO_ERROR) {
    // Apply the calibration to BHI360
    int ret = bhi360_set_calibration_profile(bhi360_dev, BHI360_SENSOR_GYRO,
                                            profile_data, actual_size);
    if (ret == 0) {
        LOG_INF("Calibration loaded successfully");
    }
} else if (err == err_t::FILE_SYSTEM_NO_FILES) {
    LOG_INF("No saved calibration found, performing new calibration");
    // Perform calibration...
}
```

## Integration with Motion Sensor Module

The motion sensor module should:

1. **On Startup**: Try to load saved calibration data
2. **After Calibration**: Save the calibration data
3. **Periodically**: Update calibration if accuracy improves

### Example Integration

```c
// In motion_sensor.cpp initialization
static void load_saved_calibrations(const struct device *bhi360_dev)
{
    struct {
        enum bhi360_sensor_type sensor;
        uint8_t type_id;
        const char *name;
    } sensors[] = {
        {BHI360_SENSOR_ACCEL, 0, "accel"},
        {BHI360_SENSOR_GYRO, 1, "gyro"}
    };

    for (int i = 0; i < ARRAY_SIZE(sensors); i++) {
        uint8_t profile_data[512];
        size_t actual_size;
        
        err_t err = get_bhi360_calibration_data(sensors[i].type_id, 
                                               profile_data, 
                                               sizeof(profile_data), 
                                               &actual_size);
        if (err == err_t::NO_ERROR) {
            int ret = bhi360_set_calibration_profile(bhi360_dev, 
                                                    sensors[i].sensor,
                                                    profile_data, 
                                                    actual_size);
            if (ret == 0) {
                LOG_INF("Loaded %s calibration from storage", sensors[i].name);
            } else {
                LOG_WRN("Failed to apply %s calibration: %d", sensors[i].name, ret);
            }
        } else {
            LOG_INF("No saved %s calibration found", sensors[i].name);
        }
    }
}

// After successful calibration
static void save_calibration_profile(const struct device *bhi360_dev, 
                                   enum bhi360_sensor_type sensor,
                                   uint8_t type_id)
{
    uint8_t profile_data[512];
    size_t actual_size;
    
    int ret = bhi360_get_calibration_profile(bhi360_dev, sensor, 
                                            profile_data, 
                                            sizeof(profile_data), 
                                            &actual_size);
    if (ret == 0) {
        err_t err = store_bhi360_calibration_data(type_id, 
                                                 profile_data, 
                                                 actual_size);
        if (err == err_t::NO_ERROR) {
            LOG_INF("Saved calibration for sensor type %d", type_id);
        } else {
            LOG_ERR("Failed to save calibration: %d", (int)err);
        }
    }
}
```

## Message-Based Calibration Commands

The data module also supports message-based calibration save/load commands:

### Save Calibration via Message

```c
generic_message_t msg;
msg.sender = SENDER_BHI360_THREAD;
msg.type = MSG_TYPE_SAVE_BHI360_CALIBRATION;
msg.data.bhi360_calibration.sensor_type = 1; // gyro
msg.data.bhi360_calibration.profile_size = actual_size;
memcpy(msg.data.bhi360_calibration.profile_data, profile_data, actual_size);

k_msgq_put(&data_msgq, &msg, K_NO_WAIT);
```

### Load Calibration via Message

```c
generic_message_t msg;
msg.sender = SENDER_BHI360_THREAD;
msg.type = MSG_TYPE_LOAD_BHI360_CALIBRATION;
msg.data.bhi360_calibration.sensor_type = 1; // gyro

k_msgq_put(&data_msgq, &msg, K_NO_WAIT);
```

## Error Handling

The functions return the following error codes:

- `NO_ERROR`: Success
- `INVALID_PARAMETER`: Invalid sensor type or null pointers
- `FILE_SYSTEM_ERROR`: Failed to access filesystem
- `FILE_SYSTEM_NO_FILES`: Calibration file not found
- `DATA_ERROR`: Invalid data size or format

## How It Works

### First Time Operation

When the BHI360 starts for the first time:

1. **Load attempt**: The system tries to load calibration files from `/lfs1/calibration/`
2. **Not found**: Since it's the first time, no files exist (returns `FILE_SYSTEM_NO_FILES`)
3. **Calibration check**: The system checks the current calibration status
4. **Perform calibration**: If sensors are not calibrated, it performs FOC (Fast Offset Compensation)
5. **Save calibration**: After successful calibration, the profiles are saved to files

### Subsequent Operations

On subsequent startups:

1. **Load calibration**: The system loads saved calibration profiles from files
2. **Apply to BHI360**: The profiles are applied using `bhi360_set_calibration_profile()`
3. **Check status**: The calibration status is checked
4. **Skip calibration**: If sensors are already calibrated (from loaded profiles), calibration is skipped
5. **Update if needed**: Only performs calibration if the loaded profiles are insufficient

### Note on Sensors

The BHI360 only contains:
- **Accelerometer**: 3-axis accelerometer with FOC calibration
- **Gyroscope**: 3-axis gyroscope with FOC calibration

There is no magnetometer in the BHI360 or on the PCB.

## Integration Steps

To integrate this into your motion sensor module:

1. **Include header**: Add `#include <data.hpp>` to access the storage API
2. **Add load function**: Call `load_saved_calibrations()` before checking calibration status
3. **Add save function**: Call `save_calibration_profile()` after successful calibration
4. **Periodic check**: Add periodic calibration status check (optional for monitoring improvements)

## Notes

1. The calibration files are binary format with a 2-byte header containing the profile size
2. Maximum profile size is 512 bytes
3. The calibration directory is created automatically if it doesn't exist
4. Calibration data persists across power cycles
5. Each sensor type has its own calibration file
6. The system gracefully handles missing files (first time operation)
7. Failed loads don't prevent the system from operating
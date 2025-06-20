# BHI360 Zephyr Driver Usage Guide

## Overview

The BHI360 driver has been implemented as a proper Zephyr sensor driver with the following features:

1. **Standard Zephyr Sensor API**: Implements `sensor_sample_fetch()` and `sensor_channel_get()`
2. **SPI Communication**: Uses Zephyr's SPI API instead of direct nrfx calls
3. **GPIO Handling**: Proper interrupt and reset GPIO management
4. **Extended API**: Additional functions for direct BHY2 access

## Key Changes from Original Implementation

### 1. SPI Communication
- **Old**: Used `nrfx_spim` directly with `NRFX_SPIM_INSTANCE(1)`
- **New**: Uses Zephyr's SPI API with device tree configuration

### 2. Device Initialization
- **Old**: Manual initialization in `motion_sensor.cpp`
- **New**: Automatic initialization through Zephyr's device driver framework

### 3. Interrupt Handling
- **Old**: Manual GPIO configuration and callback setup
- **New**: Integrated interrupt handling with semaphore signaling

## Migration Guide for motion_sensor.cpp

### Step 1: Remove Direct Hardware Access

Replace:
```cpp
static const struct device *const bhi360_dev = DEVICE_DT_GET(BHI360_NODE);
```

With:
```cpp
#include <drivers/sensor/bosch/bhi360/bhi360.h>
static const struct device *const bhi360_dev = DEVICE_DT_GET(DT_NODELABEL(bhi360));
```

### Step 2: Use Driver API for Initialization

The driver automatically initializes during boot, so remove manual initialization code:

```cpp
// Remove all of this:
// - GPIO configuration
// - SPI setup
// - bhy2_init() calls
// - Product ID verification

// Just check if device is ready:
if (!device_is_ready(bhi360_dev)) {
    LOG_ERR("BHI360 device not ready");
    return;
}
```

### Step 3: Access BHY2 Device

To access the BHY2 device for advanced operations:

```cpp
struct bhy2_dev *bhy2 = bhi360_get_bhy2_dev(bhi360_dev);
if (!bhy2) {
    LOG_ERR("Failed to get BHY2 device handle");
    return;
}

// Now you can use bhy2 for firmware upload, sensor configuration, etc.
int8_t rslt = upload_firmware(bhy2);
```

### Step 4: Handle Interrupts

Replace manual interrupt handling with driver API:

```cpp
// In your processing thread:
while (true) {
    // Wait for data ready
    int ret = bhi360_wait_for_data(bhi360_dev, K_FOREVER);
    if (ret == 0) {
        // Process FIFO
        uint8_t work_buffer[2048];
        ret = bhi360_process_fifo(bhi360_dev, work_buffer, sizeof(work_buffer));
        if (ret < 0) {
            LOG_ERR("FIFO processing failed: %d", ret);
        }
    }
}
```

## Device Tree Configuration

The driver uses the following device tree properties:

```dts
&spi1 {
    bhi360: bhi360@0 {
        compatible = "bosch,bhi360";
        reg = <0>;
        spi-max-frequency = <8000000>;
        int-gpios = <&gpio1 2 GPIO_ACTIVE_HIGH>;
        reset-gpios = <&gpio1 3 GPIO_ACTIVE_LOW>;
        label = "BHI360";
    };
};
```

## Benefits of the New Implementation

1. **Better Integration**: Works with Zephyr's device model and power management
2. **Cleaner Code**: Separates driver logic from application logic
3. **Reusability**: Driver can be used by multiple applications
4. **Standard API**: Follows Zephyr conventions for sensor drivers
5. **Error Handling**: Consistent error codes and logging

## Future Enhancements

1. Implement full sensor channel support for standard sensor types
2. Add power management support
3. Implement sensor trigger API for interrupt-driven operation
4. Add support for multiple BHI360 instances
5. Implement sensor attribute configuration

## Example Usage

```cpp
// Basic usage with standard sensor API
struct sensor_value accel[3];
int ret;

// Fetch all sensor data
ret = sensor_sample_fetch(bhi360_dev, SENSOR_CHAN_ALL);
if (ret < 0) {
    LOG_ERR("Failed to fetch sample: %d", ret);
    return;
}

// Get accelerometer data (when implemented)
ret = sensor_channel_get(bhi360_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
if (ret < 0) {
    LOG_ERR("Failed to get accel data: %d", ret);
    return;
}
```
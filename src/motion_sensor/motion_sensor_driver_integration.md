# Motion Sensor Driver Integration Analysis

## Current State Analysis

### What's Already There:
1. **Device Tree Integration**: The code already gets the BHI360 device from device tree
2. **Interrupt Handling**: Manual GPIO configuration for interrupts with semaphore
3. **BHY2 Direct Usage**: Direct calls to BHY2 API functions
4. **SPI Communication**: Uses `bhi360_spi_read/write` from common.c

### What the New Driver Provides:
1. **Automatic Initialization**: Driver initializes during boot
2. **Managed SPI Communication**: Uses Zephyr's SPI API
3. **Interrupt Management**: Built-in interrupt handling
4. **Extended API**: Access to BHY2 device, FIFO processing, wait for data

## Integration Steps

### Step 1: Remove Manual SPI Initialization ✓ READY
The current code uses `bhi360_spi_read/write` from common.c. The driver now handles this internally.

**Current Code to Remove:**
```cpp
// Line 165-170: Direct BHY2 init with manual SPI functions
rslt = bhy2_init(BHY2_SPI_INTERFACE, bhi360_spi_read, bhi360_spi_write, 
                 bhi360_delay_us, BHY2_RD_WR_LEN, NULL, &bhy2);
```

**Replace With:**
```cpp
// Get BHY2 device from driver
struct bhy2_dev *bhy2_ptr = bhi360_get_bhy2_dev(bhi360_dev);
if (!bhy2_ptr) {
    LOG_ERR("Failed to get BHY2 device from driver");
    return;
}
// Use the pointer for subsequent operations
bhy2 = *bhy2_ptr;  // Copy to local if needed, or use pointer directly
```

### Step 2: Use Driver's Interrupt Handling ✓ READY
The driver manages interrupts internally and provides a wait function.

**Current Code to Remove:**
```cpp
// Lines 147-157: Manual GPIO interrupt configuration
gpio_pin_configure_dt(&bhi360_int, GPIO_INPUT);
gpio_pin_interrupt_configure_dt(&bhi360_int, GPIO_INT_EDGE_TO_ACTIVE);
gpio_init_callback(&bhi360_int_cb, bhi360_int_handler, BIT(bhi360_int.pin));
gpio_add_callback(bhi360_int.port, &bhi360_int_cb);

// Lines 279-285: Manual interrupt handler
static void bhi360_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&bhi360_int_sem);
}
```

**Replace With:**
```cpp
// In motion_sensor_process thread:
while (true) {
    // Wait for data ready from driver
    int ret = bhi360_wait_for_data(bhi360_dev, K_FOREVER);
    if (ret == 0) {
        // Process FIFO using driver API
        uint8_t work_buffer[WORK_BUFFER_SIZE];
        ret = bhi360_process_fifo(bhi360_dev, work_buffer, sizeof(work_buffer));
        if (ret < 0) {
            LOG_WRN("BHI360: FIFO processing error: %d", ret);
        }
    }
}
```

### Step 3: Access BHY2 Through Driver API ✓ READY
All BHY2 operations can continue using the device handle from the driver.

**Implementation Pattern:**
```cpp
static void motion_sensor_init()
{
    // Check if driver is ready
    if (!device_is_ready(bhi360_dev)) {
        LOG_ERR("BHI360 device not ready");
        return;
    }
    
    // Get BHY2 device handle
    struct bhy2_dev *bhy2_ptr = bhi360_get_bhy2_dev(bhi360_dev);
    if (!bhy2_ptr) {
        LOG_ERR("Failed to get BHY2 device from driver");
        return;
    }
    
    // Driver already did initialization, soft reset, and product ID check
    // We can proceed directly to firmware upload and sensor configuration
    
    // Check boot status and upload firmware
    uint8_t boot_status;
    int8_t rslt = bhy2_get_boot_status(&boot_status, bhy2_ptr);
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
        // Continue with firmware upload and sensor configuration
        rslt = upload_firmware(bhy2_ptr);
        // ... rest of initialization
    }
}
```

### Step 4: Standard Sensor Channels ⚠️ PARTIAL
The driver provides the framework but doesn't implement specific channels yet.

**Future Enhancement:**
```cpp
// In bhi360_zephyr.c, implement channel support:
static int bhi360_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{
    struct bhi360_data *data = dev->data;
    
    switch (chan) {
    case SENSOR_CHAN_ACCEL_XYZ:
        // Return linear acceleration data
        break;
    case SENSOR_CHAN_GYRO_XYZ:
        // Return gyroscope data
        break;
    case SENSOR_CHAN_ROTATION:
        // Return quaternion data
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}
```

## What Can Be Implemented Now

### 1. Minimal Integration (Recommended First Step)
Keep most of the existing logic but use the driver for device management:

```cpp
static void motion_sensor_init()
{
    // 1. Check driver readiness
    if (!device_is_ready(bhi360_dev)) {
        LOG_ERR("BHI360 device not ready");
        return;
    }
    
    // 2. Get BHY2 handle from driver
    struct bhy2_dev *bhy2_ptr = bhi360_get_bhy2_dev(bhi360_dev);
    if (!bhy2_ptr) {
        LOG_ERR("Failed to get BHY2 device");
        return;
    }
    
    // 3. Continue with existing firmware upload and configuration
    // Just use bhy2_ptr instead of &bhy2
    
    // 4. Start motion sensor thread
    motion_sensor_tid = k_thread_create(...);
}

void motion_sensor_process(void *, void *, void *)
{
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    struct bhy2_dev *bhy2_ptr = bhi360_get_bhy2_dev(bhi360_dev);
    
    while (true) {
        // Use driver's wait function
        int ret = bhi360_wait_for_data(bhi360_dev, K_FOREVER);
        if (ret == 0) {
            // Process FIFO directly with BHY2 API
            int8_t rslt = bhy2_get_and_process_fifo(work_buffer, 
                                                    sizeof(work_buffer), 
                                                    bhy2_ptr);
            if (rslt != BHY2_OK) {
                LOG_WRN("BHI360: FIFO processing error");
            }
        }
    }
}
```

### 2. Full Integration (Future Enhancement)
Move all BHY2 operations into the driver and expose high-level APIs:

```cpp
// In driver, add:
int bhi360_configure_sensors(const struct device *dev, 
                           const struct bhi360_sensor_config *configs,
                           size_t num_configs);

int bhi360_upload_firmware(const struct device *dev,
                          const uint8_t *firmware,
                          size_t firmware_size);

// In motion_sensor.cpp, use simplified API:
static void motion_sensor_init()
{
    struct bhi360_sensor_config configs[] = {
        { .id = BHY2_SENSOR_ID_RV, .rate = 50.0f },
        { .id = BHY2_SENSOR_ID_LACC, .rate = 50.0f },
        { .id = BHY2_SENSOR_ID_GYRO, .rate = 50.0f },
        { .id = BHY2_SENSOR_ID_STC, .rate = 5.0f },
    };
    
    bhi360_configure_sensors(bhi360_dev, configs, ARRAY_SIZE(configs));
}
```

## Benefits of Integration

1. **Cleaner Code**: Remove manual GPIO and SPI handling
2. **Better Error Handling**: Driver manages device state
3. **Thread Safety**: Driver handles synchronization
4. **Power Management**: Driver can implement PM callbacks
5. **Reusability**: Other modules can use the same driver

## Risks and Considerations

1. **Minimal Risk**: The driver is designed to be compatible
2. **Testing Required**: Verify interrupt timing and FIFO processing
3. **Performance**: Driver adds minimal overhead
4. **Backward Compatibility**: Can be done incrementally

## Recommended Approach

1. **Phase 1**: Minimal integration - Use driver for device management only
2. **Phase 2**: Move interrupt handling to driver
3. **Phase 3**: Implement standard sensor channels
4. **Phase 4**: Full API abstraction

This allows gradual migration while maintaining functionality.
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>

#include "BHY2-Sensor-API/bhy2.h"
#include "BHY2-Sensor-API/bhy2_defs.h"
#include "bhi360.h"

LOG_MODULE_REGISTER(bhi360, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT bosch_bhi360

struct bhi360_config {
    struct spi_dt_spec bus;
#if DT_INST_NODE_HAS_PROP(0, int_gpios)
    struct gpio_dt_spec int_gpio;
#endif
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
    struct gpio_dt_spec reset_gpio;
#endif
};

struct bhi360_data {
    struct bhy2_dev bhy2_dev;
    bool initialized;
    struct k_sem data_ready_sem;
    struct gpio_callback int_cb;
    
    /* Latest sensor readings */
    struct bhi360_sensor_data sensor_data;
    struct k_mutex data_mutex;
};

/* Forward declarations */
static int8_t bhi360_spi_read_wrapper(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
static int8_t bhi360_spi_write_wrapper(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
static void bhi360_delay_us(uint32_t period_us, void *intf_ptr);
static void bhi360_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

static int bhi360_init(const struct device *dev)
{
    const struct bhi360_config *config = dev->config;
    struct bhi360_data *data = dev->data;
    int ret;
    
    LOG_INF("Initializing BHI360 sensor driver");

    
    /* Initialize semaphore for interrupt handling */
    k_sem_init(&data->data_ready_sem, 0, 1);
    
    /* Initialize mutex for data protection */
    k_mutex_init(&data->data_mutex);
    
    /* Check if SPI bus is ready */
    if (!spi_is_ready_dt(&config->bus)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }
    
    /* Configure reset GPIO if available */
#if DT_INST_NODE_HAS_PROP(0, reset_gpios)
    if (gpio_is_ready_dt(&config->reset_gpio)) {
        ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }
        
        /* Perform hardware reset */
        gpio_pin_set_dt(&config->reset_gpio, 1); // Assert reset (active low)
        k_msleep(10); // Hold reset for 10ms
        gpio_pin_set_dt(&config->reset_gpio, 0); // Deassert reset
        k_msleep(100); // Wait 100ms for device to boot, increased to ensure readiness
        LOG_DBG("Hardware reset performed with extended wait time");
    }
#endif
    
    /* Configure interrupt GPIO if available */
#if DT_INST_NODE_HAS_PROP(0, int_gpios)
    if (gpio_is_ready_dt(&config->int_gpio)) {
        ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure INT GPIO: %d", ret);
            return ret;
        }
        
        ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure INT GPIO interrupt: %d", ret);
            return ret;
        }
        
        /* Setup interrupt callback */
        gpio_init_callback(&data->int_cb, bhi360_int_handler, BIT(config->int_gpio.pin));
        ret = gpio_add_callback(config->int_gpio.port, &data->int_cb);
        if (ret < 0) {
            LOG_ERR("Failed to add INT GPIO callback: %d", ret);
            return ret;
        }
        
        LOG_DBG("Interrupt GPIO configured");
    }
#endif
    
    /* Initialize the BHY2 device structure */
    memset(&data->bhy2_dev, 0, sizeof(data->bhy2_dev));
    
    /* Initialize BHY2 with SPI interface */
    int8_t rslt = bhy2_init(BHY2_SPI_INTERFACE, 
                           bhi360_spi_read_wrapper, 
                           bhi360_spi_write_wrapper, 
                           bhi360_delay_us, 
                           256, /* Read/write length */
                           (void *)dev, 
                           &data->bhy2_dev);
    
    if (rslt != BHY2_OK) {
        LOG_ERR("BHY2 initialization failed: %d", rslt);
        return -EIO;
    }
    
    /* Soft reset the device with retry mechanism */
    int retry_count = 0;
    int max_retries = 2;
    do {
        rslt = bhy2_soft_reset(&data->bhy2_dev);
        if (rslt != BHY2_OK) {
            LOG_ERR("BHY2 soft reset attempt %d failed: %d", retry_count + 1, rslt);
            k_msleep(50); // Wait before retry
            retry_count++;
        }
    } while (rslt != BHY2_OK && retry_count < max_retries);

    if (rslt != BHY2_OK) {
        LOG_ERR("BHY2 soft reset failed after %d attempts", max_retries);
        return -EIO;
    }
    LOG_INF("BHY2 soft reset successful after %d attempt(s)", retry_count + 1);
    
    /* Wait for device to be ready and verify product ID */
    uint8_t product_id = 0;
    retry_count = 0;
    max_retries = 20;
    
    do {
        k_msleep(10);
        rslt = bhy2_get_product_id(&product_id, &data->bhy2_dev);
        retry_count++;
    } while ((rslt != BHY2_OK || product_id != BHY2_PRODUCT_ID) && retry_count < max_retries);
    
    if (rslt != BHY2_OK || product_id != BHY2_PRODUCT_ID) {
        LOG_ERR("Failed to verify product ID after %d attempts (rslt=%d, id=0x%02X)", 
                retry_count, rslt, product_id);
        return -EIO;
    }
    
    LOG_INF("BHI360 initialized successfully (Product ID: 0x%02X)", product_id);
    data->initialized = true;
    
    return 0;
}

/* SPI read wrapper function */
static int8_t bhi360_spi_read_wrapper(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    const struct device *dev = (const struct device *)intf_ptr;
    const struct bhi360_config *config = dev->config;
    int ret;
    
    /* BHI360 uses bit 7 set for read operations */
    uint8_t tx_buf[1] = {reg_addr | 0x80};
    
    const struct spi_buf tx_bufs[] = {
        {
            .buf = tx_buf,
            .len = 1,
        },
    };
    
    const struct spi_buf rx_bufs[] = {
        {
            .buf = NULL,
            .len = 1,
        },
        {
            .buf = reg_data,
            .len = length,
        },
    };
    
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = ARRAY_SIZE(tx_bufs),
    };
    
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = ARRAY_SIZE(rx_bufs),
    };
    
    ret = spi_transceive_dt(&config->bus, &tx, &rx);
    if (ret < 0) {
        LOG_ERR("SPI read failed: %d", ret);
        return BHY2_E_IO;
    }
    
    return BHY2_OK;
}

/* SPI write wrapper function */
static int8_t bhi360_spi_write_wrapper(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    const struct device *dev = (const struct device *)intf_ptr;
    const struct bhi360_config *config = dev->config;
    int ret;
    
    /* Allocate buffer for address + data */
    uint8_t *tx_buf = k_malloc(length + 1);
    if (!tx_buf) {
        LOG_ERR("Failed to allocate TX buffer");
        return BHY2_E_BUFFER;
    }
    
    /* BHI360 uses bit 7 clear for write operations */
    tx_buf[0] = reg_addr & 0x7F;
    memcpy(&tx_buf[1], reg_data, length);
    
    const struct spi_buf tx_bufs[] = {
        {
            .buf = tx_buf,
            .len = length + 1,
        },
    };
    
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = ARRAY_SIZE(tx_bufs),
    };
    
    ret = spi_write_dt(&config->bus, &tx);
    k_free(tx_buf);
    
    if (ret < 0) {
        LOG_ERR("SPI write failed: %d", ret);
        return BHY2_E_IO;
    }
    
    return BHY2_OK;
}

/* Delay function for BHY2 driver */
static void bhi360_delay_us(uint32_t period_us, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);
    k_usleep(period_us);
}

/* Interrupt handler for data ready */
static void bhi360_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct bhi360_data *data = CONTAINER_OF(cb, struct bhi360_data, int_cb);
    
    k_sem_give(&data->data_ready_sem);
}

/* Sensor API implementation */
static int bhi360_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct bhi360_data *data = dev->data;
    
    if (!data->initialized) {
        LOG_ERR("Device not initialized");
        return -ENODEV;
    }
    
    if (chan != SENSOR_CHAN_ALL) {
        LOG_ERR("Unsupported channel %d", chan);
        return -ENOTSUP;
    }
    
    /* In a real implementation, this would read FIFO data */
    /* For now, just return success */
    return 0;
}

static int bhi360_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{
    struct bhi360_data *data = dev->data;
    int ret = 0;
    
    if (!data->initialized) {
        LOG_ERR("Device not initialized");
        return -ENODEV;
    }
    
    /* Lock mutex to ensure data consistency */
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    
    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
        sensor_value_from_float(val, data->sensor_data.lacc_x);
        break;
    case SENSOR_CHAN_ACCEL_Y:
        sensor_value_from_float(val, data->sensor_data.lacc_y);
        break;
    case SENSOR_CHAN_ACCEL_Z:
        sensor_value_from_float(val, data->sensor_data.lacc_z);
        break;
    case SENSOR_CHAN_ACCEL_XYZ:
        /* Return X value, caller should get Y and Z separately */
        sensor_value_from_float(val, data->sensor_data.lacc_x);
        sensor_value_from_float(val + 1, data->sensor_data.lacc_y);
        sensor_value_from_float(val + 2, data->sensor_data.lacc_z);
        break;
        
    case SENSOR_CHAN_GYRO_X:
        sensor_value_from_float(val, data->sensor_data.gyro_x);
        break;
    case SENSOR_CHAN_GYRO_Y:
        sensor_value_from_float(val, data->sensor_data.gyro_y);
        break;
    case SENSOR_CHAN_GYRO_Z:
        sensor_value_from_float(val, data->sensor_data.gyro_z);
        break;
    case SENSOR_CHAN_GYRO_XYZ:
        /* Return all three values */
        sensor_value_from_float(val, data->sensor_data.gyro_x);
        sensor_value_from_float(val + 1, data->sensor_data.gyro_y);
        sensor_value_from_float(val + 2, data->sensor_data.gyro_z);
        break;
        
    case SENSOR_CHAN_ROTATION:
        /* Return quaternion as X, Y, Z, W */
        sensor_value_from_float(val, data->sensor_data.quat_x);
        sensor_value_from_float(val + 1, data->sensor_data.quat_y);
        sensor_value_from_float(val + 2, data->sensor_data.quat_z);
        sensor_value_from_float(val + 3, data->sensor_data.quat_w);
        break;
        
    /* Step counter - using custom channel */
    case BHI360_SENSOR_CHAN_STEPS:
        val->val1 = data->sensor_data.step_count;
        val->val2 = 0;
        break;
        
    default:
        LOG_ERR("Unsupported channel %d", chan);
        ret = -ENOTSUP;
        break;
    }
    
    k_mutex_unlock(&data->data_mutex);
    
    return ret;
}

static const struct sensor_driver_api bhi360_api = {
    .sample_fetch = bhi360_sample_fetch,
    .channel_get = bhi360_channel_get,
};

/* Additional API functions */
struct bhy2_dev *bhi360_get_bhy2_dev(const struct device *dev)
{
    struct bhi360_data *data = dev->data;
    
    if (!data->initialized) {
        return NULL;
    }
    
    return &data->bhy2_dev;
}

int bhi360_wait_for_data(const struct device *dev, k_timeout_t timeout)
{
    struct bhi360_data *data = dev->data;
    
    if (!data->initialized) {
        return -ENODEV;
    }
    
    return k_sem_take(&data->data_ready_sem, timeout);
}

int bhi360_process_fifo(const struct device *dev, uint8_t *work_buffer, size_t buffer_size)
{
    struct bhi360_data *data = dev->data;
    int8_t rslt;
    
    if (!data->initialized) {
        return -ENODEV;
    }
    
    if (!work_buffer || buffer_size == 0) {
        return -EINVAL;
    }
    
    rslt = bhy2_get_and_process_fifo(work_buffer, buffer_size, &data->bhy2_dev);
    if (rslt != BHY2_OK) {
        LOG_ERR("FIFO processing failed: %d", rslt);
        return -EIO;
    }
    
    return 0;
}

/* Function to update sensor data - called by application after parsing FIFO */
int bhi360_update_sensor_data(const struct device *dev, 
                             const struct bhi360_sensor_data *new_data)
{
    struct bhi360_data *data = dev->data;
    
    if (!data->initialized) {
        return -ENODEV;
    }
    
    if (!new_data) {
        return -EINVAL;
    }
    
    /* Lock mutex and update data */
    k_mutex_lock(&data->data_mutex, K_FOREVER);
    memcpy(&data->sensor_data, new_data, sizeof(struct bhi360_sensor_data));
    k_mutex_unlock(&data->data_mutex);
    
    return 0;
}

/* Device instantiation macro */
#define BHI360_INIT(n) \
    static struct bhi360_config bhi360_config_##n = { \
        .bus = SPI_DT_SPEC_INST_GET(n, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0), \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(n, int_gpios), (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),)) \
        IF_ENABLED(DT_INST_NODE_HAS_PROP(n, reset_gpios), (.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),)) \
    }; \
    static struct bhi360_data bhi360_data_##n; \
    DEVICE_DT_INST_DEFINE(n, bhi360_init, NULL, \
        &bhi360_data_##n, &bhi360_config_##n, \
        POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
        &bhi360_api);

DT_INST_FOREACH_STATUS_OKAY(BHI360_INIT)

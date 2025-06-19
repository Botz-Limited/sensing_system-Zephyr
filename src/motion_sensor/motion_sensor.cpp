/**
 * @file motion_sensor.cpp
 * @author
 * @brief
 * @version 1.0.0
 * @date 2025-05-12
 *
 * @copyright Botz Innovation 2025
 *
 */

#include <hal/nrf_spim.h>
#include <zephyr/arch/arm/arch.h>
#define MODULE motion_sensor

/*************************** INCLUDE HEADERS ********************************/
#include <cstring>
#include <time.h>

#include <cstdio>
#include <cstring>

#include <app.hpp>
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <events/app_state_event.h>
#include <events/motion_sensor_event.h>
#include <motion_sensor.hpp>
#include <status_codes.h>
#include <ble_services.hpp>

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "../bluetooth/ble_d2d_tx.hpp"
#endif
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "/home/ee/sensing_fw/drivers/sensor/bosch/bhi360/BHY2-Sensor-API/bhy2.h"
#include "/home/ee/sensing_fw/drivers/sensor/bosch/bhi360/BHY2-Sensor-API/bhy2_parse.h"
#include "/home/ee/sensing_fw/drivers/sensor/bosch/bhi360/BHY2-Sensor-API/firmware/bhi360/BHI360_Aux_BMM150.fw.h"
extern "C"
{
#include "/home/ee/sensing_fw/drivers/sensor/bosch/bhi360/common.h"
}

#include <errors.hpp>

static constexpr uint16_t BHY2_RD_WR_LEN = 256;
static constexpr uint16_t WORK_BUFFER_SIZE = 2048;

LOG_MODULE_REGISTER(MODULE, CONFIG_LOG_DEFAULT_LEVEL); // NOLINT

// Get BHI360 device from device tree
#define BHI360_NODE DT_NODELABEL(bhi360)
static const struct device *const bhi360_dev = DEVICE_DT_GET(BHI360_NODE);

// SPI config
// Currently unused - kept for future SPI configuration
__attribute__((unused))
static struct spi_config bhi360_spi_cfg = {
    .frequency = 8000000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | NRF_SPIM_MODE_0,
    .slave = 0,
    .cs = NULL // We'll use Zephyr's GPIO API for manual CS if needed
};

static bool logging_active = false; // Tracks current logging state
static float configured_sample_rate = 50.0f; // Tracks the configured sample rate (Hz) for motion sensors

// INT and RESET GPIOs from device tree
#if DT_NODE_HAS_PROP(BHI360_NODE, int_gpios)
static const struct gpio_dt_spec bhi360_int = GPIO_DT_SPEC_GET(BHI360_NODE, int_gpios);
#endif
#if DT_NODE_HAS_PROP(BHI360_NODE, reset_gpios)
static const struct gpio_dt_spec bhi360_reset = GPIO_DT_SPEC_GET(BHI360_NODE, reset_gpios);
#endif

// Sensor IDs for BHI360 virtual sensors
static constexpr uint8_t QUAT_SENSOR_ID = BHY2_SENSOR_ID_RV;          // Rotation Vector (quaternion) - ID 34
static constexpr uint8_t LACC_SENSOR_ID = BHY2_SENSOR_ID_LACC;        // Linear Acceleration (gravity removed) - ID 31
static constexpr uint8_t GYRO_SENSOR_ID = BHY2_SENSOR_ID_GYRO;        // Gyroscope (angular velocity) - ID 13
static constexpr uint8_t STEP_COUNTER_SENSOR_ID = BHY2_SENSOR_ID_STC; // Step Counter - ID 52

// Note: The key improvement here is using LACC (ID 31) instead of ACC (ID 4).
// LACC provides linear acceleration with gravity already removed by the BHI360's
// sensor fusion algorithms, which is better for motion tracking applications.

static struct bhy2_dev bhy2;

// Semaphore for interrupt-driven FIFO processing
static struct k_sem bhi360_int_sem;

// Forward declarations
static void parse_all_sensors(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void print_api_error(int8_t rslt, struct bhy2_dev *dev);
static int8_t upload_firmware(struct bhy2_dev *dev);
static void bhi360_delay_us(uint32_t period_us, void *intf_ptr);
static void bhi360_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

/********************************** Motion Sensor THREAD ********************************/
static constexpr int motion_sensor_stack_size = CONFIG_MOTION_SENSOR_MODULE_STACK_SIZE;
static constexpr int motion_sensor_priority = CONFIG_MOTION_SENSOR_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(motion_sensor_stack_area, motion_sensor_stack_size);
static struct k_thread motion_sensor_thread_data;
static k_tid_t motion_sensor_tid;
void motion_sensor_process(void *, void *, void *);

// GPIO callback struct
#if DT_NODE_HAS_PROP(BHI360_NODE, int_gpios)
static struct gpio_callback bhi360_int_cb;
#endif

void motion_sensor_initializing_entry()
{
    LOG_INF("Motion Sensor initializing_entry done ");
}

static void motion_sensor_init()
{
    motion_sensor_initializing_entry();
    LOG_INF("Starting BHI360 full initialization sequence");
    if (!device_is_ready(bhi360_dev))
    {
        LOG_ERR("BHI360 device not ready");
        return;
    }
    // Additional check: try to read product ID to verify device presence
    uint8_t probe_product_id = 0;
    int8_t probe_rslt = bhy2_get_product_id(&probe_product_id, &bhy2);
    if (probe_rslt != BHY2_OK || probe_product_id != BHY2_PRODUCT_ID) {
        LOG_ERR("BHI360 hardware not detected or not responding (probe_rslt=%d, id=0x%02X)", probe_rslt, probe_product_id);
        return;
    }
#if DT_NODE_HAS_PROP(BHI360_NODE, reset_gpios)
    if (gpio_is_ready_dt(&bhi360_reset))
    {
        gpio_pin_configure_dt(&bhi360_reset, GPIO_OUTPUT_ACTIVE);
        gpio_pin_set_dt(&bhi360_reset, 1);
        k_msleep(1);
        gpio_pin_set_dt(&bhi360_reset, 0);
        k_msleep(10);
        gpio_pin_set_dt(&bhi360_reset, 1);
        k_msleep(10);
    }
#endif
#if DT_NODE_HAS_PROP(BHI360_NODE, int_gpios)
    if (gpio_is_ready_dt(&bhi360_int))
    {
        gpio_pin_configure_dt(&bhi360_int, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&bhi360_int, GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&bhi360_int_cb, bhi360_int_handler, BIT(bhi360_int.pin));
        gpio_add_callback(bhi360_int.port, &bhi360_int_cb);
        LOG_INF("BHI360 INT pin interrupt configured");
    }
#endif
    k_sem_init(&bhi360_int_sem, 0, 1);
    int8_t rslt;
    uint8_t product_id = 0;
    uint16_t version = 0;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;
    // 1. Initialize BHI360 driver
    rslt =
        bhy2_init(BHY2_SPI_INTERFACE, bhi360_spi_read, bhi360_spi_write, bhi360_delay_us, BHY2_RD_WR_LEN, NULL, &bhy2);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("BHI360: Initialization failed");
        return;
    }
    // 2. Soft reset
    rslt = bhy2_soft_reset(&bhy2);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("BHI360: Soft reset failed");
        return;
    }
    // 3. Product ID check with retries
    bool id_read_success = false;
    for (int retry = 0; retry < 20; retry++)
    {
        rslt = bhy2_get_product_id(&product_id, &bhy2);
        if (rslt == BHY2_OK && product_id == BHY2_PRODUCT_ID)
        {
            LOG_INF("BHI360: Product ID verified on attempt %d", retry + 1);
            id_read_success = true;
            break;
        }
        k_msleep(10);
    }
    if (!id_read_success)
    {
        LOG_ERR("BHI360: Failed to verify product ID");
        return;
    }
    // 4. Configure FIFO and interrupts (enable INT output)
    hintr_ctrl = 0; // Enable all INT outputs
    rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, &bhy2);
    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, &bhy2);
    // 5. Check boot status and upload firmware
    rslt = bhy2_get_boot_status(&boot_status, &bhy2);
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY)
    {
        LOG_INF("BHI360: Uploading firmware");
        rslt = upload_firmware(&bhy2);
        if (rslt != BHY2_OK)
        {
            LOG_ERR("BHI360: Firmware upload failed");
            return;
        }
        // 6. Boot from RAM and verify
        rslt = bhy2_boot_from_ram(&bhy2);
        rslt = bhy2_get_kernel_version(&version, &bhy2);
        if (rslt != BHY2_OK || version == 0)
        {
            LOG_ERR("BHI360: Boot failed");
            return;
        }
        LOG_INF("BHI360: Boot successful, kernel version %u", version);
        // 7. Update virtual sensor list
        LOG_INF("BHI360: Updating virtual sensor list");
        rslt = bhy2_update_virtual_sensor_list(&bhy2);
        print_api_error(rslt, &bhy2);
        // 8. Register callbacks
        LOG_INF("BHI360: Registering callbacks");
        
        // Configure different rates for motion sensors vs step counter
        float motion_sensor_rate = 50.0f;  // 50 Hz for motion sensors (20ms interval)
        float step_counter_rate = 5.0f;    // 5 Hz for step counter (200ms interval)
        configured_sample_rate = motion_sensor_rate; // Store the motion sensor rate for logging
        uint32_t report_latency_ms = 0;
        
        // Configure high-rate motion sensors
        LOG_INF("BHI360: Configuring motion sensors at %.1f Hz", motion_sensor_rate);
        
        // Configure quaternion sensor
        LOG_INF("BHI360: Configuring quaternion sensor (ID %d)...", QUAT_SENSOR_ID);
        rslt = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, motion_sensor_rate, report_latency_ms, &bhy2);
        print_api_error(rslt, &bhy2);
        
        // Configure linear acceleration sensor (gravity removed)
        LOG_INF("BHI360: Configuring linear acceleration sensor (ID %d)...", LACC_SENSOR_ID);
        rslt = bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, motion_sensor_rate, report_latency_ms, &bhy2);
        print_api_error(rslt, &bhy2);
        
        // Configure gyroscope sensor
        LOG_INF("BHI360: Configuring gyroscope sensor (ID %d)...", GYRO_SENSOR_ID);
        rslt = bhy2_set_virt_sensor_cfg(GYRO_SENSOR_ID, motion_sensor_rate, report_latency_ms, &bhy2);
        print_api_error(rslt, &bhy2);
        
        // Configure step counter sensor at lower rate
        LOG_INF("BHI360: Configuring step counter sensor (ID %d) at %.1f Hz...", STEP_COUNTER_SENSOR_ID, step_counter_rate);
        rslt = bhy2_set_virt_sensor_cfg(STEP_COUNTER_SENSOR_ID, step_counter_rate, report_latency_ms, &bhy2);
        print_api_error(rslt, &bhy2);
        // Register a single callback for all sensors
        rslt = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_all_sensors, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(LACC_SENSOR_ID, parse_all_sensors, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(GYRO_SENSOR_ID, parse_all_sensors, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(STEP_COUNTER_SENSOR_ID, parse_all_sensors, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        rslt = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, &bhy2);
        print_api_error(rslt, &bhy2);
        LOG_INF("BHI360: Initialization complete");
    }
    else
    {
        LOG_ERR("BHI360: Host interface not ready");
        return;
    }
    // Start the motion sensor thread
    motion_sensor_tid = k_thread_create(&motion_sensor_thread_data, motion_sensor_stack_area,
                                        K_THREAD_STACK_SIZEOF(motion_sensor_stack_area), motion_sensor_process, nullptr,
                                        nullptr, nullptr, motion_sensor_priority, 0, K_NO_WAIT);
    LOG_INF("Motion Sensor Module Initialised");
}

void motion_sensor_process(void *, void *, void *)
{
    k_thread_name_set(motion_sensor_tid, "Motion_Sensor");
    module_set_state(MODULE_STATE_READY);
    uint8_t work_buffer[WORK_BUFFER_SIZE];
    while (true)
    {
        // Wait for interrupt
        k_sem_take(&bhi360_int_sem, K_FOREVER);
        // Process FIFO
        int8_t rslt = bhy2_get_and_process_fifo(work_buffer, sizeof(work_buffer), &bhy2);
        if (rslt != BHY2_OK)
        {
            LOG_WRN("BHI360: FIFO processing error");
        }
    }
}

static void parse_all_sensors(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    ARG_UNUSED(callback_ref);
    // Static variables to hold the latest values
    static float latest_quat_x = 0, latest_quat_y = 0, latest_quat_z = 0, latest_quat_w = 0, latest_quat_accuracy = 0;
    static float latest_lacc_x = 0, latest_lacc_y = 0, latest_lacc_z = 0;
    static float latest_gyro_x = 0, latest_gyro_y = 0, latest_gyro_z = 0;
    static uint32_t latest_step_count = 0;
    static uint64_t latest_timestamp = 0;

    // Synchronization tracking
    static uint8_t motion_update_mask = 0;
    static const uint8_t QUAT_UPDATED = 0x01;
    static const uint8_t LACC_UPDATED = 0x02;
    static const uint8_t GYRO_UPDATED = 0x04;
    static const uint8_t ALL_MOTION_SENSORS = (QUAT_UPDATED | LACC_UPDATED | GYRO_UPDATED);
    
    bool send_data = false;

    switch (callback_info->sensor_id)
    {
        case QUAT_SENSOR_ID: {
            struct bhy2_data_quaternion data;
            if (callback_info->data_size != 11)
            {
                LOG_ERR("Quaternion: invalid data size: %d", callback_info->data_size);
                return;
            }
            bhy2_parse_quaternion(callback_info->data_ptr, &data);
            // Quaternion components are normalized to [-1, 1] range
            latest_quat_x = (float)data.x / 16384.0f;
            latest_quat_y = (float)data.y / 16384.0f;
            latest_quat_z = (float)data.z / 16384.0f;
            latest_quat_w = (float)data.w / 16384.0f;
            latest_quat_accuracy = (float)data.accuracy;
            latest_timestamp = *callback_info->time_stamp * 15625; // Convert to nanoseconds
            motion_update_mask |= QUAT_UPDATED;
            break;
        }
        case LACC_SENSOR_ID: {
            struct bhy2_data_xyz data;
            bhy2_parse_xyz(callback_info->data_ptr, &data);
            // Linear acceleration is in m/s^2, scaled by 100
            latest_lacc_x = (float)data.x / 100.0f;
            latest_lacc_y = (float)data.y / 100.0f;
            latest_lacc_z = (float)data.z / 100.0f;
            motion_update_mask |= LACC_UPDATED;
            break;
        }
        case GYRO_SENSOR_ID: {
            struct bhy2_data_xyz data;
            bhy2_parse_xyz(callback_info->data_ptr, &data);
            // Gyroscope data is in rad/s, scaled by 2^14
            latest_gyro_x = (float)data.x / 16384.0f;
            latest_gyro_y = (float)data.y / 16384.0f;
            latest_gyro_z = (float)data.z / 16384.0f;
            motion_update_mask |= GYRO_UPDATED;
            break;
        }
        case STEP_COUNTER_SENSOR_ID: {
            if (callback_info->data_size < 4)
            {
                LOG_ERR("Step counter: invalid data size: %d", callback_info->data_size);
                return;
            }
            latest_step_count = (callback_info->data_ptr[0]) | (callback_info->data_ptr[1] << 8) |
                                (callback_info->data_ptr[2] << 16) | (callback_info->data_ptr[3] << 24);
            // Step counter updates independently, don't wait for it
            // It will be included with whatever value it has when motion sensors sync
            break;
        }
        default:
            LOG_WRN("Unknown sensor ID: %u", callback_info->sensor_id);
            break;
    }

    // Check if all motion sensors have been updated
    if ((motion_update_mask & ALL_MOTION_SENSORS) == ALL_MOTION_SENSORS)
    {
        send_data = true;
        motion_update_mask = 0;  // Reset for next cycle
    }
    
    // Send data when we have a complete set or step counter update
    if (send_data)
    {
        // Send combined log record to data module
        bhi360_log_record_t record{};
        record.quat_x = latest_quat_x;
        record.quat_y = latest_quat_y;
        record.quat_z = latest_quat_z;
        record.quat_w = latest_quat_w;
        record.quat_accuracy = latest_quat_accuracy;
        record.lacc_x = latest_lacc_x;
        record.lacc_y = latest_lacc_y;
        record.lacc_z = latest_lacc_z;
        record.gyro_x = latest_gyro_x;
        record.gyro_y = latest_gyro_y;
        record.gyro_z = latest_gyro_z;
        record.step_count = latest_step_count;
        record.timestamp = latest_timestamp;
        if (logging_active == true)
        {
            generic_message_t msg{};
            msg.sender = SENDER_BHI360_THREAD;
            msg.type = MSG_TYPE_BHI360_LOG_RECORD;
            msg.data.bhi360_log_record = record;
            k_msgq_put(&data_msgq, &msg, K_NO_WAIT);

            // Send individual values to Bluetooth module
            // 3D mapping with actual gyro data and quaternion
            generic_message_t qmsg{};
            qmsg.sender = SENDER_BHI360_THREAD;
            qmsg.type = MSG_TYPE_BHI360_3D_MAPPING;
            qmsg.data.bhi360_3d_mapping.gyro_x = latest_gyro_x;
            qmsg.data.bhi360_3d_mapping.gyro_y = latest_gyro_y;
            qmsg.data.bhi360_3d_mapping.gyro_z = latest_gyro_z;
            // Using accel fields for quaternion x,y,z components
            qmsg.data.bhi360_3d_mapping.accel_x = latest_quat_x;
            qmsg.data.bhi360_3d_mapping.accel_y = latest_quat_y;
            qmsg.data.bhi360_3d_mapping.accel_z = latest_quat_z;
            qmsg.data.bhi360_3d_mapping.quat_w = latest_quat_w;  // Now including W component
            #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            k_msgq_put(&bluetooth_msgq, &qmsg, K_NO_WAIT);
#else
            // Secondary device: Send to primary via D2D
            ble_d2d_tx_send_bhi360_data1(&qmsg.data.bhi360_3d_mapping);
#endif

            // Linear acceleration
            generic_message_t lmsg{};
            lmsg.sender = SENDER_BHI360_THREAD;
            lmsg.type = MSG_TYPE_BHI360_LINEAR_ACCEL;
            lmsg.data.bhi360_linear_accel.x = latest_lacc_x;
            lmsg.data.bhi360_linear_accel.y = latest_lacc_y;
            lmsg.data.bhi360_linear_accel.z = latest_lacc_z;
            #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            k_msgq_put(&bluetooth_msgq, &lmsg, K_NO_WAIT);
#else
            // Secondary device: Send to primary via D2D
            ble_d2d_tx_send_bhi360_data3(&lmsg.data.bhi360_linear_accel);
#endif

            // Step count
            generic_message_t smsg{};
            smsg.sender = SENDER_BHI360_THREAD;
            smsg.type = MSG_TYPE_BHI360_STEP_COUNT;
            smsg.data.bhi360_step_count.step_count = latest_step_count;
            smsg.data.bhi360_step_count.activity_duration_s = 0; // Fill if available
            #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            k_msgq_put(&bluetooth_msgq, &smsg, K_NO_WAIT);
#else
            // Secondary device: Send to primary via D2D
            ble_d2d_tx_send_bhi360_data2(&smsg.data.bhi360_step_count);
#endif
        }
    }
}

static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    if (rslt != BHY2_OK)
    {
        LOG_ERR("API error: %d", rslt);
        if ((rslt == BHY2_E_IO) && (dev != NULL))
        {
            LOG_ERR("Interface error: %d", dev->hif.intf_rslt);
            dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
        }
    }
}

static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    char *event_text;
    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
    {
        event_text = (char *)"[META EVENT]";
    }
    else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
    {
        event_text = (char *)"[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }
    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            LOG_INF("%s Flush complete for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            LOG_INF("%s Sample rate changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_POWER_MODE_CHANGED:
            LOG_INF("%s Power mode changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_ALGORITHM_EVENTS:
            LOG_INF("%s Algorithm event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            LOG_INF("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
            LOG_INF("%s BSX event (do steps main)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
            LOG_INF("%s BSX event (do steps calib)\r\n", event_text);
            break;
        case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            LOG_INF("%s BSX event (get output signal)\r\n", event_text);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            LOG_INF("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            LOG_INF("%s FIFO overflow\r\n", event_text);
            break;
        case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
            LOG_INF("%s Dynamic range changed for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_FIFO_WATERMARK:
            LOG_INF("%s FIFO watermark reached\r\n", event_text);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            LOG_INF("%s Firmware initialized. Firmware version %u\r\n", event_text, ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_TRANSFER_CAUSE:
            LOG_INF("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_SENSOR_FRAMEWORK:
            LOG_INF("%s Sensor framework event for sensor id %u\r\n", event_text, byte1);
            break;
        case BHY2_META_EVENT_RESET:
            LOG_INF("%s Reset event\r\n", event_text);
            break;
        case BHY2_META_EVENT_SPACER:
            break;
        default:
            LOG_INF("%s Unknown meta event with id: %u\r\n", event_text, meta_event_type);
            break;
    }
}

static int8_t upload_firmware(struct bhy2_dev *dev)
{
    uint32_t incr = 256; /* Max command packet size */
    uint32_t len = sizeof(bhy2_firmware_image);
    int8_t rslt = BHY2_OK;
    if ((incr % 4) != 0) /* Round off to higher 4 bytes */
    {
        incr = ((incr >> 2) + 1) << 2;
    }
    for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
    {
        if (incr > (len - i)) /* If last payload */
        {
            incr = len - i;
            if ((incr % 4) != 0) /* Round off to higher 4 bytes */
            {
                incr = ((incr >> 2) + 1) << 2;
            }
        }
#ifdef UPLOAD_FIRMWARE_TO_FLASH
        rslt = bhy2_upload_firmware_to_flash_partly(&bhy2_firmware_image[i], i, incr, dev);
#else
        rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i, incr, dev);
#endif
        LOG_INF("%.2f%% complete", (double)((float)(i + incr) / (float)len * 100.0f));
    }
    return rslt;
}

// Delay function for BHY2 driver
static void bhi360_delay_us(uint32_t period_us, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);
    k_usleep(period_us);
}

// GPIO interrupt handler for BHI360 INT pin
static void bhi360_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    k_sem_give(&bhi360_int_sem);
}

// Return type dictates if event is consumed. False = Not Consumed, True = Consumed.

static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);
        if (check_state(event, MODULE_ID(foot_sensor), MODULE_STATE_READY))
        {
            motion_sensor_init();
        }
        return false;
    }
    if (is_motion_sensor_start_activity_event(aeh))
    {
        // Trigger the same logic as auto-start logging for BHI360
        if (!logging_active)
        {
            generic_message_t cmd_msg = {};
            cmd_msg.sender = SENDER_BHI360_THREAD;
            cmd_msg.type = MSG_TYPE_COMMAND;
            strncpy(cmd_msg.data.command_str, "START_LOGGING_BHI360", sizeof(cmd_msg.data.command_str) - 1);
            cmd_msg.data.command_str[sizeof(cmd_msg.data.command_str) - 1] = '\0';
            // Use the actual BHI360 sample rate here
            cmd_msg.sampling_frequency = configured_sample_rate;
            strncpy(cmd_msg.fw_version, CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION, sizeof(cmd_msg.fw_version) - 1);
            cmd_msg.fw_version[sizeof(cmd_msg.fw_version) - 1] = '\0';

            LOG_INF("Motion sensor sending stop command: '%s', type: %d", cmd_msg.data.command_str, cmd_msg.type);
            int cmd_ret = k_msgq_put(&data_msgq, &cmd_msg, K_NO_WAIT);
            if (cmd_ret == 0)
            {
                LOG_INF("Sent START_LOGGING_BHI360 command (via event)");
                logging_active = true;
            }
            else
            {
                LOG_ERR("Failed to send START_LOGGING_BHI360 command: %d", cmd_ret);
            }
        }
        return false;
    }
    if (is_motion_sensor_stop_activity_event(aeh))
    {
        LOG_INF("Received motion_sensor_stop_activity_event");
        if (logging_active)
        {
            generic_message_t cmd_msg = {};
            cmd_msg.sender = SENDER_BHI360_THREAD;
            cmd_msg.type = MSG_TYPE_COMMAND;
            strncpy(cmd_msg.data.command_str, "STOP_LOGGING_BHI360", sizeof(cmd_msg.data.command_str) - 1);
            cmd_msg.data.command_str[sizeof(cmd_msg.data.command_str) - 1] = '\0';
            int cmd_ret = k_msgq_put(&data_msgq, &cmd_msg, K_NO_WAIT);
            if (cmd_ret == 0)
            {
                LOG_INF("Sent STOP_LOGGING_BHI360 command (via event)");
                logging_active = false;
            }
            else
            {
                LOG_ERR("Failed to send STOP_LOGGING_BHI360 command: %d", cmd_ret);
            }
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_sensor_stop_activity_event);

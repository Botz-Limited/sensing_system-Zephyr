/**
 * @file motion_sensor_simple.cpp
 * @brief Simplified motion sensor module using BHI360 with polling mode
 * @version 2.1.0
 * @date 2025-01-09
 *
 * @copyright Botz Innovation 2025
 *
 * This is a simplified version using polling instead of interrupts,
 * similar to foot_sensor_simple.cpp architecture
 */

#define MODULE motion_sensor

#include <cmath>
#include <cstdio>
#include <cstring>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <app.hpp>
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/idle_event.h>
#include <events/motion_sensor_event.h>
#include <events/streaming_control_event.h>
#include <status_codes.h>

// BHI360 includes
#include "BHY2-Sensor-API/bhy2.h"
#include "BHY2-Sensor-API/bhy2_parse.h"
#include "BHY2-Sensor-API/firmware/bhi360/BHI360_Aux_BMM150.fw.h"
#include <bhi360.h>

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "../bluetooth/ble_d2d_tx.hpp"
#endif

LOG_MODULE_REGISTER(MODULE, CONFIG_MOTION_SENSOR_MODULE_LOG_LEVEL);

// External message queues
extern struct k_msgq sensor_data_msgq;
extern struct k_msgq bluetooth_msgq;
extern struct k_msgq data_msgq;
extern struct k_msgq motion_sensor_msgq;
#if IS_ENABLED(CONFIG_LAB_VERSION)
extern struct k_msgq data_sd_msgq; // For sending to data_sd module (lab version)
#endif

// Constants
static constexpr uint16_t WORK_BUFFER_SIZE = 2048;
static constexpr uint32_t SAMPLE_PERIOD_MS = 10;             // 10ms for 100Hz polling
static constexpr uint8_t BLUETOOTH_RATE_DIVIDER = 20;        // Send to BLE at 5Hz (100Hz / 20)
static constexpr uint16_t LOG_RATE_DIVIDER = 100;            // Log at 1Hz (100Hz / 100)
static constexpr uint32_t CALIBRATION_CHECK_INTERVAL = 1000; // Check every 1000 iterations

// Thread stack and priority
K_THREAD_STACK_DEFINE(motion_sensor_stack, CONFIG_MOTION_SENSOR_MODULE_STACK_SIZE);
static struct k_thread motion_sensor_thread_data;
static k_tid_t motion_sensor_tid = NULL;

// Get BHI360 device from device tree
#define BHI360_NODE DT_NODELABEL(bhi360)
static const struct device *const bhi360_dev = DEVICE_DT_GET(BHI360_NODE);
static struct bhy2_dev *bhy2_ptr = nullptr;

// Use atomic for thread-safe access to logging state
static atomic_t logging_active = ATOMIC_INIT(0);

// Local streaming state
static bool quaternion_streaming_enabled = false;

// Step count tracking variables
static uint32_t latest_step_count = 0;
static uint32_t activity_start_step_count = 0;
static uint32_t latest_activity_step_count = 0;
static bool activity_logging_active = false;

#include <zephyr/drivers/sensor.h> // For standard sensor value types

// Sensor IDs for BHI360 virtual sensors
static constexpr uint8_t QUAT_SENSOR_ID = BHY2_SENSOR_ID_GAMERV;      // 28: Game Rotation Vector (no mag)
static constexpr uint8_t LACC_SENSOR_ID = BHY2_SENSOR_ID_ACC;         // Linear Acceleration (direct ID if not defined)
static constexpr uint8_t GYRO_SENSOR_ID = BHY2_SENSOR_ID_GYRO;        // 3: Raw Gyroscope
static constexpr uint8_t ACCEL_SENSOR_ID = BHY2_SENSOR_ID_ACC;        // 1: Raw Accelerometer (dependency)
static constexpr uint8_t STEP_COUNTER_SENSOR_ID = BHY2_SENSOR_ID_STC; // 43: Step Counter (corrected back)

// Static variables to hold the latest sensor values (file scope for access across functions)
static float latest_quat_x = 0, latest_quat_y = 0, latest_quat_z = 0, latest_quat_w = 0, latest_quat_accuracy = 0;
static float latest_lacc_x = 0, latest_lacc_y = 0, latest_lacc_z = 0;
static float latest_gyro_x = 0, latest_gyro_y = 0, latest_gyro_z = 0;
static uint64_t latest_timestamp = 0;

// Forward declarations
static void motion_sensor_thread(void *p1, void *p2, void *p3);
static err_t init_bhi360(void);
static int8_t upload_firmware(struct bhy2_dev *dev);
static void parse_all_sensors(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void process_sensor_data(void);
static void handle_messages(void);
static void perform_bhi360_calibration(void);
static void apply_calibration_data(const bhi360_calibration_data_t *calib_data);
static void save_calibration_profile(const struct device *bhi360_dev, enum bhi360_sensor_type sensor,
                                     const char *sensor_name);
static void check_and_save_calibration_updates(const struct device *bhi360_dev);

/**
 * @brief Initialize BHI360 sensor
 */
static err_t init_bhi360(void)
{
    // Check if driver is ready
    if (!device_is_ready(bhi360_dev))
    {
        LOG_ERR("BHI360 device driver not ready");
        return err_t::MOTION_ERROR;
    }

    // Get BHY2 device handle from driver
    bhy2_ptr = bhi360_get_bhy2_dev(bhi360_dev);
    if (!bhy2_ptr)
    {
        LOG_ERR("Failed to get BHY2 device handle from driver");
        return err_t::MOTION_ERROR;
    }

    LOG_INF("BHI360 driver ready, BHY2 handle obtained");

    int8_t rslt;
    uint16_t version = 0;
    uint8_t hintr_ctrl, hif_ctrl, boot_status;

    // Configure FIFO and interrupts - disable interrupts for polling mode
    hintr_ctrl = BHY2_ICTL_DISABLE_STATUS_FIFO | BHY2_ICTL_DISABLE_DEBUG;
    rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to set host interrupt control: %d", rslt);
        return err_t::MOTION_ERROR;
    }

    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to set host interface control: %d", rslt);
        return err_t::MOTION_ERROR;
    }

    // Check boot status
    rslt = bhy2_get_boot_status(&boot_status, bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to get boot status: %d", rslt);
        return err_t::MOTION_ERROR;
    }

    if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY))
    {
        LOG_ERR("BHI360: Host interface not ready");
        return err_t::MOTION_ERROR;
    }

    // Upload firmware
    LOG_INF("BHI360: Uploading firmware");
    rslt = upload_firmware(bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("BHI360: Firmware upload failed");
        return err_t::MOTION_ERROR;
    }

    // Boot from RAM
    rslt = bhy2_boot_from_ram(bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("BHI360: Boot from RAM failed: %d", rslt);
        return err_t::MOTION_ERROR;
    }

    // Verify boot
    rslt = bhy2_get_kernel_version(&version, bhy2_ptr);
    if (rslt != BHY2_OK || version == 0)
    {
        LOG_ERR("BHI360: Boot verification failed");
        return err_t::MOTION_ERROR;
    }

    LOG_INF("BHI360: Boot successful, kernel version %u", version);

    // Update virtual sensor list
    LOG_INF("BHI360: Updating virtual sensor list");
    rslt = bhy2_update_virtual_sensor_list(bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to update virtual sensor list: %d", rslt);
        return err_t::MOTION_ERROR;
    }

    // Configure sensors at 100Hz
    float motion_sensor_rate = 100.0f;
    float step_counter_rate = 20.0f;
    uint32_t report_latency_ms = 0;

    LOG_INF("BHI360: Configuring sensors");

    rslt = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, motion_sensor_rate, report_latency_ms, bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to configure quaternion sensor: %d", rslt);
        return err_t::MOTION_ERROR;
    }

    rslt = bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, motion_sensor_rate, report_latency_ms, bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to configure linear acceleration sensor: %d", rslt);
        return err_t::MOTION_ERROR;
    }

    rslt = bhy2_set_virt_sensor_cfg(GYRO_SENSOR_ID, motion_sensor_rate, report_latency_ms, bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to configure gyroscope sensor: %d", rslt);
        return err_t::MOTION_ERROR;
    }

    rslt = bhy2_set_virt_sensor_cfg(STEP_COUNTER_SENSOR_ID, step_counter_rate, report_latency_ms, bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to configure step counter sensor: %d", rslt);
        return err_t::MOTION_ERROR;
    }

    // Register callbacks - ESSENTIAL for data parsing
    LOG_INF("BHI360: Registering callbacks");
    rslt |= bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
    rslt |= bhy2_register_fifo_parse_callback(LACC_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
    rslt |= bhy2_register_fifo_parse_callback(GYRO_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
    rslt |= bhy2_register_fifo_parse_callback(STEP_COUNTER_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
    rslt |= bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, bhy2_ptr);

    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to register callbacks: %d", rslt);
        return err_t::MOTION_ERROR;
    }

    LOG_INF("BHI360: Initialization complete");

    // Request calibration data from data module
    LOG_INF("BHI360: Requesting calibration data from data module");
    generic_message_t request_msg = {};
    request_msg.sender = SENDER_MOTION_SENSOR;
    request_msg.type = MSG_TYPE_REQUEST_BHI360_CALIBRATION;
    k_msgq_put(&data_msgq, &request_msg, K_NO_WAIT);

    return err_t::NO_ERROR;
}

/**
 * @brief Upload firmware to BHI360
 */
static int8_t upload_firmware(struct bhy2_dev *dev)
{
    uint32_t incr = 256; // Max command packet size
    uint32_t len = sizeof(bhy2_firmware_image);
    int8_t rslt = BHY2_OK;

    if ((incr % 4) != 0)
    {
        incr = ((incr >> 2) + 1) << 2;
    }

    for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr)
    {
        if (incr > (len - i))
        {
            incr = len - i;
            if ((incr % 4) != 0)
            {
                incr = ((incr >> 2) + 1) << 2;
            }
        }

        rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i, incr, dev);
        LOG_INF("%.2f%% complete", (double)((float)(i + incr) / (float)len * 100.0f));
    }

    return rslt;
}

/**
 * @brief Parse sensor data callback
 */
static void parse_all_sensors(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    ARG_UNUSED(callback_ref);

    // For motion activity detection (idle module)
    static float last_reported_quat_w = 0;
    static uint32_t last_reported_steps = 0;
    static int64_t last_activity_report_time = 0;
    const int64_t ACTIVITY_REPORT_INTERVAL_MS = 5000;

    // Synchronization tracking
    static uint8_t motion_update_mask = 0;
    static const uint8_t QUAT_UPDATED = 0x01;
    static const uint8_t LACC_UPDATED = 0x02;
    static const uint8_t GYRO_UPDATED = 0x04;
    static const uint8_t ALL_MOTION_SENSORS = (QUAT_UPDATED | LACC_UPDATED | GYRO_UPDATED);

    bool send_data = false;
    bool motion_detected = false;

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

            // Detect motion for idle module
            float motion_threshold = CONFIG_IDLE_MOTION_THRESHOLD_QUATERNION / 1000.0f;
            float quat_delta = fabs(latest_quat_w - last_reported_quat_w);
            if (quat_delta > motion_threshold)
            {
                motion_detected = true;
                last_reported_quat_w = latest_quat_w;
                LOG_DBG("Motion detected: quaternion delta %.3f > threshold %.3f", (double)quat_delta,
                        (double)motion_threshold);
            }
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

            // Handle potential wrap-around
            static uint32_t previous_step_count = 0;
            static uint32_t step_accumulator = 0;

            uint32_t new_step_count = (callback_info->data_ptr[0]) | (callback_info->data_ptr[1] << 8) |
                                      (callback_info->data_ptr[2] << 16) | (callback_info->data_ptr[3] << 24);

            if (new_step_count < previous_step_count)
            {
                LOG_WRN("Step count wrap detected");
                step_accumulator += UINT32_MAX + 1;
            }

            latest_step_count = step_accumulator + new_step_count;
            previous_step_count = new_step_count;

            // Calculate activity step count if activity is active
            if (activity_logging_active)
            {
                latest_activity_step_count = latest_step_count - activity_start_step_count;

                // Send activity step count update during logging
                if (atomic_get(&logging_active) == 1)
                {
                    generic_message_t activity_step_msg{};
                    activity_step_msg.sender = SENDER_MOTION_SENSOR;
                    activity_step_msg.type = MSG_TYPE_BHI360_STEP_COUNT;
                    activity_step_msg.data.bhi360_step_count.step_count = latest_activity_step_count;
                    activity_step_msg.data.bhi360_step_count.activity_duration_s = 0;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    k_msgq_put(&bluetooth_msgq, &activity_step_msg, K_NO_WAIT);
#else
                    ble_d2d_tx_send_activity_step_count(&activity_step_msg.data.bhi360_step_count);
#endif
                }
            }

            // Detect motion via step count changes
            if (latest_step_count > last_reported_steps)
            {
                motion_detected = true;
                last_reported_steps = latest_step_count;
            }
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
        motion_update_mask = 0; // Reset for next cycle
                                //  LOG_DBG("All motion sensors updated, processing data");
    }
    else
    {
        // Debug: Log which sensors are still missing
    }

    // Process data when we have a complete set
    if (send_data)
    {
        process_sensor_data();
    }

    // Send motion activity event to idle module if motion detected
    if (motion_detected)
    {
        int64_t current_time = k_uptime_get();
        if ((current_time - last_activity_report_time) > ACTIVITY_REPORT_INTERVAL_MS)
        {
            struct motion_activity_event *evt = new_motion_activity_event();
            if (evt)
            {
                evt->timestamp_ms = (uint32_t)current_time;
                APP_EVENT_SUBMIT(evt);
                LOG_DBG("Sent motion activity event to idle module");
            }
            last_activity_report_time = current_time;
        }
    }
}

/**
 * @brief Parse meta events callback
 */
static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    ARG_UNUSED(callback_ref);

    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];

    switch (meta_event_type)
    {
        case BHY2_META_EVENT_FLUSH_COMPLETE:
            LOG_DBG("Flush complete for sensor id %u", byte1);
            break;
        case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
            LOG_INF("Sample rate changed for sensor id %u", byte1);
            break;
        case BHY2_META_EVENT_SENSOR_STATUS:
            LOG_DBG("Accuracy for sensor id %u changed to %u", byte1, byte2);
            break;
        case BHY2_META_EVENT_INITIALIZED:
            LOG_INF("Firmware initialized. Version %u", ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHY2_META_EVENT_SENSOR_ERROR:
            LOG_ERR("Sensor id %u reported error 0x%02X", byte1, byte2);
            break;
        case BHY2_META_EVENT_FIFO_OVERFLOW:
            LOG_WRN("FIFO overflow");
            break;
        default:
            // Ignore other meta events
            break;
    }
}

/**
 * @brief Process sensor data and send to appropriate queues
 */
static void process_sensor_data(void)
{
    static uint8_t ble_sample_counter = 0;
    static uint16_t log_sample_counter = 0;

    // Update driver with latest sensor data
    struct bhi360_sensor_data driver_data = {.quat_x = latest_quat_x,
                                             .quat_y = latest_quat_y,
                                             .quat_z = latest_quat_z,
                                             .quat_w = latest_quat_w,
                                             .quat_accuracy = (uint8_t)latest_quat_accuracy,
                                             .lacc_x = latest_lacc_x,
                                             .lacc_y = latest_lacc_y,
                                             .lacc_z = latest_lacc_z,
                                             .gyro_x = latest_gyro_x,
                                             .gyro_y = latest_gyro_y,
                                             .gyro_z = latest_gyro_z,
                                             .step_count = latest_step_count,
                                             .timestamp = latest_timestamp};
    bhi360_update_sensor_data(bhi360_dev, &driver_data);

    // Prepare message for logging
    generic_message_t msg{};
    msg.sender = SENDER_BHI360_THREAD;
    msg.type = MSG_TYPE_BHI360_LOG_RECORD;
    msg.data.bhi360_log_record.quat_x = latest_quat_x;
    msg.data.bhi360_log_record.quat_y = latest_quat_y;
    msg.data.bhi360_log_record.quat_z = latest_quat_z;
    msg.data.bhi360_log_record.quat_w = latest_quat_w;
    msg.data.bhi360_log_record.quat_accuracy = latest_quat_accuracy;
    msg.data.bhi360_log_record.lacc_x = latest_lacc_x;
    msg.data.bhi360_log_record.lacc_y = latest_lacc_y;
    msg.data.bhi360_log_record.lacc_z = latest_lacc_z;
    msg.data.bhi360_log_record.gyro_x = latest_gyro_x;
    msg.data.bhi360_log_record.gyro_y = latest_gyro_y;
    msg.data.bhi360_log_record.gyro_z = latest_gyro_z;
    msg.data.bhi360_log_record.step_count = latest_step_count;
    msg.data.bhi360_log_record.timestamp = latest_timestamp;

    // Send to sensor_data module if activity is active
    if (atomic_get(&logging_active) == 1)
    {
#if IS_ENABLED(CONFIG_SENSOR_DATA_MODULE)
        k_msgq_put(&sensor_data_msgq, &msg, K_NO_WAIT);
#endif
    }

#if IS_ENABLED(CONFIG_LAB_VERSION)
    // For lab version, send raw IMU data to SD card during activity OR streaming
    if ((atomic_get(&logging_active) == 0) || (quaternion_streaming_enabled == true))
    {
        if (k_msgq_put(&data_sd_msgq, &msg, K_NO_WAIT) != 0)
        {
            LOG_WRN("Failed to send BHI360 data to data_sd module");
        }
    }
#endif

    // Debug logging at 1Hz when activity or streaming is active
    if ((atomic_get(&logging_active) == 1) || (quaternion_streaming_enabled == true))
    {
        if (++log_sample_counter >= LOG_RATE_DIVIDER)
        {
            log_sample_counter = 0;
            LOG_INF("BHI360 3D Mapping: quat_x=%.6f, quat_y=%.6f, quat_z=%.6f, quat_w=%.6f, "
                    "gyro_x=%.6f, gyro_y=%.6f, gyro_z=%.6f",
                    (double)latest_quat_x, (double)latest_quat_y, (double)latest_quat_z, (double)latest_quat_w,
                    (double)latest_gyro_x, (double)latest_gyro_y, (double)latest_gyro_z);
        }
    }

    // Send to Bluetooth at 5Hz when streaming is enabled AND no activity is running
    if (quaternion_streaming_enabled == true && atomic_get(&logging_active) == 0)
    {
        if (++ble_sample_counter >= BLUETOOTH_RATE_DIVIDER)
        {
            ble_sample_counter = 0;

            // Send quaternion data
            generic_message_t qmsg{};
            qmsg.sender = SENDER_BHI360_THREAD;
            qmsg.type = MSG_TYPE_BHI360_3D_MAPPING;
            qmsg.data.bhi360_3d_mapping.gyro_x = 0; // Zeroed to reduce BLE bandwidth
            qmsg.data.bhi360_3d_mapping.gyro_y = 0;
            qmsg.data.bhi360_3d_mapping.gyro_z = 0;
            qmsg.data.bhi360_3d_mapping.accel_x = latest_quat_x;
            qmsg.data.bhi360_3d_mapping.accel_y = latest_quat_y;
            qmsg.data.bhi360_3d_mapping.accel_z = latest_quat_z;
            qmsg.data.bhi360_3d_mapping.quat_w = latest_quat_w;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            k_msgq_put(&bluetooth_msgq, &qmsg, K_NO_WAIT);
#else
            ble_d2d_tx_send_bhi360_data1(&qmsg.data.bhi360_3d_mapping);
#endif

            // Send step count
            generic_message_t smsg{};
            smsg.sender = SENDER_BHI360_THREAD;
            smsg.type = MSG_TYPE_BHI360_STEP_COUNT;
            smsg.data.bhi360_step_count.step_count = latest_step_count;
            smsg.data.bhi360_step_count.activity_duration_s = 0;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            k_msgq_put(&bluetooth_msgq, &smsg, K_NO_WAIT);
#else
            ble_d2d_tx_send_bhi360_data2(&smsg.data.bhi360_step_count);
#endif
        }
    }
    else
    {
        // Reset counter when not streaming
        ble_sample_counter = 0;
    }
}

/**
 * @brief Handle incoming messages
 */
static void handle_messages(void)
{
    generic_message_t msg;

    // Check for messages (non-blocking)
    while (k_msgq_get(&motion_sensor_msgq, &msg, K_NO_WAIT) == 0)
    {
        switch (msg.type)
        {
            case MSG_TYPE_TRIGGER_BHI360_CALIBRATION:
                LOG_INF("Received calibration trigger from %s", get_sender_name(msg.sender));
                perform_bhi360_calibration();
                break;

            case MSG_TYPE_BHI360_CALIBRATION_DATA:
                LOG_INF("Received calibration data from %s", get_sender_name(msg.sender));
                apply_calibration_data(&msg.data.bhi360_calibration);
                break;

            default:
                LOG_WRN("Unknown message type %d from %s", msg.type, get_sender_name(msg.sender));
                break;
        }
    }
}

/**
 * @brief Main motion sensor thread
 */
static void motion_sensor_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Motion sensor thread started (polling mode)");

    // Initialize BHI360
    err_t err = init_bhi360();
    if (err != err_t::NO_ERROR)
    {
        LOG_ERR("Failed to initialize BHI360");

        // Report error to Bluetooth module
        generic_message_t err_msg;
        err_msg.sender = SENDER_MOTION_SENSOR;
        err_msg.type = MSG_TYPE_ERROR_STATUS;
        err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
        err_msg.data.error_status.is_set = true;
        k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT);

#if IS_ENABLED(CONFIG_MOTION_SENSOR_OPTIONAL)
        LOG_WRN("Motion sensor initialization failed but continuing (non-critical)");
        module_set_state(MODULE_STATE_READY);
#else
        LOG_ERR("Motion sensor initialization failed (critical)");
        module_set_state(MODULE_STATE_ERROR);
        return;
#endif
    }
    else
    {
        module_set_state(MODULE_STATE_READY);
    }

    uint8_t work_buffer[WORK_BUFFER_SIZE];
    uint32_t calibration_check_counter = 0;
    uint32_t poll_counter = 0;

    // Main polling loop
    while (true)
    {
        // Handle incoming messages
        //    handle_messages();

        // Periodically check and save calibration improvements
        if (++calibration_check_counter >= CALIBRATION_CHECK_INTERVAL)
        {
            calibration_check_counter = 0;
            check_and_save_calibration_updates(bhi360_dev);
        }

        // Poll BHI360 FIFO (this is the key difference from interrupt mode)
        if (bhy2_ptr)
        {
            int8_t rslt = bhy2_get_and_process_fifo(work_buffer, sizeof(work_buffer), bhy2_ptr);
            // BHY2_OK (0) means success, any negative value is an error
            // Empty FIFO is not an error - the function just processes what's available
            if (rslt != BHY2_OK)
            {
                LOG_WRN("BHI360: FIFO processing error: %d", rslt);
            }

            // Debug: Log polling status periodically
            if (++poll_counter % 1000 == 0)
            { // Every 10 seconds at 100Hz
                LOG_DBG("Motion sensor polling active - %u polls completed", poll_counter);
            }
        }

        // Sleep for 10ms to achieve 100Hz polling rate
        k_msleep(SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief Perform BHI360 calibration
 */
static void perform_bhi360_calibration(void)
{
    LOG_INF("Starting BHI360 calibration sequence...");

    struct bhi360_calibration_status calib_status;
    int ret = bhi360_get_calibration_status(bhi360_dev, &calib_status);
    if (ret == 0)
    {
        LOG_INF("Current calibration status - Accel: %d, Gyro: %d", calib_status.accel_calib_status,
                calib_status.gyro_calib_status);
    }

    // Perform gyroscope calibration
    LOG_INF("Performing gyroscope calibration...");
    LOG_INF("Please keep the device stationary");
    k_sleep(K_SECONDS(2));

    struct bhi360_foc_result foc_result;
    ret = bhi360_perform_gyro_foc(bhi360_dev, &foc_result);
    if (ret == 0 && foc_result.success)
    {
        LOG_INF("Gyro calibration successful! Offsets: X=%d, Y=%d, Z=%d", foc_result.x_offset, foc_result.y_offset,
                foc_result.z_offset);
        save_calibration_profile(bhi360_dev, BHI360_SENSOR_GYRO, "gyro");
    }
    else
    {
        LOG_ERR("Gyro calibration failed");
    }

    // Perform accelerometer calibration
    LOG_INF("Performing accelerometer calibration...");
    LOG_INF("Please place device on flat surface with Z-axis up");
    k_sleep(K_SECONDS(3));

    ret = bhi360_perform_accel_foc(bhi360_dev, &foc_result);
    if (ret == 0 && foc_result.success)
    {
        LOG_INF("Accel calibration successful! Offsets: X=%d, Y=%d, Z=%d", foc_result.x_offset, foc_result.y_offset,
                foc_result.z_offset);
        save_calibration_profile(bhi360_dev, BHI360_SENSOR_ACCEL, "accel");
    }
    else
    {
        LOG_ERR("Accel calibration failed");
    }

    LOG_INF("BHI360 calibration sequence complete");
}

/**
 * @brief Apply calibration data received from data module
 */
static void apply_calibration_data(const bhi360_calibration_data_t *calib_data)
{
    if (!calib_data)
    {
        LOG_ERR("Invalid calibration data");
        return;
    }

    enum bhi360_sensor_type sensor_type;
    const char *sensor_name;

    switch (calib_data->sensor_type)
    {
        case 0:
            sensor_type = BHI360_SENSOR_ACCEL;
            sensor_name = "accel";
            break;
        case 1:
            sensor_type = BHI360_SENSOR_GYRO;
            sensor_name = "gyro";
            break;
        default:
            LOG_ERR("Unknown sensor type: %u", calib_data->sensor_type);
            return;
    }

    if (calib_data->profile_size > 0)
    {
        int ret =
            bhi360_set_calibration_profile(bhi360_dev, sensor_type, calib_data->profile_data, calib_data->profile_size);
        if (ret == 0)
        {
            LOG_INF("Applied %s calibration (%u bytes)", sensor_name, calib_data->profile_size);
        }
        else
        {
            LOG_ERR("Failed to apply %s calibration: %d", sensor_name, ret);
        }
    }
    else
    {
        LOG_INF("No %s calibration data available", sensor_name);
    }
}

/**
 * @brief Save calibration profile after successful calibration
 */
static void save_calibration_profile(const struct device *bhi360_dev, enum bhi360_sensor_type sensor,
                                     const char *sensor_name)
{
    uint8_t type_id;

    switch (sensor)
    {
        case BHI360_SENSOR_ACCEL:
            type_id = 0;
            break;
        case BHI360_SENSOR_GYRO:
            type_id = 1;
            break;
        default:
            return;
    }

    generic_message_t save_msg = {};
    save_msg.sender = SENDER_MOTION_SENSOR;
    save_msg.type = MSG_TYPE_SAVE_BHI360_CALIBRATION;
    save_msg.data.bhi360_calibration.sensor_type = type_id;

    size_t actual_size;
    int ret = bhi360_get_calibration_profile(bhi360_dev, sensor, save_msg.data.bhi360_calibration.profile_data,
                                             sizeof(save_msg.data.bhi360_calibration.profile_data), &actual_size);
    save_msg.data.bhi360_calibration.profile_size = (uint16_t)actual_size;

    if (ret == 0)
    {
        if (k_msgq_put(&data_msgq, &save_msg, K_NO_WAIT) != 0)
        {
            LOG_ERR("Failed to send %s calibration to data module", sensor_name);
        }
        else
        {
            LOG_INF("Sent %s calibration to data module (%u bytes)", sensor_name,
                    save_msg.data.bhi360_calibration.profile_size);
        }
    }
    else
    {
        LOG_ERR("Failed to get %s calibration profile: %d", sensor_name, ret);
    }
}

/**
 * @brief Periodic calibration check and save
 */
static void check_and_save_calibration_updates(const struct device *bhi360_dev)
{
    static struct bhi360_calibration_status last_status = {0, 0, 0};
    struct bhi360_calibration_status current_status;

    int ret = bhi360_get_calibration_status(bhi360_dev, &current_status);
    if (ret != 0)
    {
        return;
    }

    if (current_status.accel_calib_status > last_status.accel_calib_status)
    {
        LOG_INF("Accel calibration improved: %d -> %d", last_status.accel_calib_status,
                current_status.accel_calib_status);
        save_calibration_profile(bhi360_dev, BHI360_SENSOR_ACCEL, "accel");
    }

    if (current_status.gyro_calib_status > last_status.gyro_calib_status)
    {
        LOG_INF("Gyro calibration improved: %d -> %d", last_status.gyro_calib_status, current_status.gyro_calib_status);
        save_calibration_profile(bhi360_dev, BHI360_SENSOR_GYRO, "gyro");
    }

    last_status = current_status;
}

/**
 * @brief Initialize motion sensor module
 */
static void motion_sensor_init(void)
{
    LOG_INF("Initializing motion sensor module (simplified polling version)");

    // Create and start the sampling thread
    motion_sensor_tid =
        k_thread_create(&motion_sensor_thread_data, motion_sensor_stack, K_THREAD_STACK_SIZEOF(motion_sensor_stack),
                        motion_sensor_thread, NULL, NULL, NULL, CONFIG_MOTION_SENSOR_MODULE_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(motion_sensor_tid, "motion_sensor");
    LOG_INF("Motion sensor module initialized successfully");
}

/**
 * @brief Event handler for motion sensor module
 */
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        // Initialize after foot sensor is ready
        if (check_state(event, MODULE_ID(foot_sensor), MODULE_STATE_READY))
        {
            motion_sensor_init();
        }
        return false;
    }

    if (is_motion_sensor_start_activity_event(aeh))
    {
        LOG_INF("Received start activity event - enabling motion sensor logging");
        if (atomic_get(&logging_active) == 0)
        {
            activity_logging_active = true;
            quaternion_streaming_enabled = false;
            activity_start_step_count = latest_step_count;
            atomic_set(&logging_active, 1);
        }
        return false;
    }

    if (is_motion_sensor_stop_activity_event(aeh))
    {
        LOG_INF("Received stop activity event - disabling motion sensor logging");
        if (atomic_get(&logging_active) == 1)
        {
            activity_logging_active = false;
            atomic_set(&logging_active, 0);
        }
        return false;
    }

    if (is_streaming_control_event(aeh))
    {
        auto *event = cast_streaming_control_event(aeh);
        quaternion_streaming_enabled = event->quaternion_streaming_enabled;
        LOG_INF("Quaternion streaming %s", quaternion_streaming_enabled ? "enabled" : "disabled");
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
APP_EVENT_SUBSCRIBE(MODULE, streaming_control_event);
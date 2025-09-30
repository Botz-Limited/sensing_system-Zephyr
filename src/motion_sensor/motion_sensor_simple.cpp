/**
 * @file motion_sensor_simple.cpp
 * @brief Simplified motion sensor module using BHI360 with polling mode
 * @version 2.2.0
 * @date 2025-01-09
 *
 * @copyright Botz Innovation 2025
 *
 * This is a simplified version using polling instead of interrupts,
 * similar to foot_sensor_simple.cpp architecture.
 * Version 2.2.0 adds a software calibration state machine for orientation-agnostic zeroing.
 */

#define MODULE motion_sensor

#include <cmath>
#include <cstdio>
#include <cstring>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
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
#include <zephyr/sys/reboot.h>

// BHI360 includes
#include "BHY2-Sensor-API/bhy2.h"
#include "BHY2-Sensor-API/bhy2_parse.h"
// Try basic firmware which should have all standard sensors including step counter
#include "BHY2-Sensor-API/firmware/bhi360/BHI360.fw.h" // Basic firmware with standard sensors
#include <bhi360.h>

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "../bluetooth/ble_d2d_tx.hpp"
#endif

LOG_MODULE_REGISTER(MODULE, CONFIG_MOTION_SENSOR_MODULE_LOG_LEVEL);

static const struct gpio_dt_spec int_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(bhi360), int_gpios);
static struct gpio_callback bhi360_int_cb_data;

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

static void bhi360_work_handler(struct k_work *work);
K_WORK_DEFINE(bhi360_work, bhi360_work_handler);

// Get BHI360 device from device tree
#define BHI360_NODE DT_NODELABEL(bhi360)
static const struct device *const bhi360_dev = DEVICE_DT_GET(BHI360_NODE);
static struct bhy2_dev *bhy2_ptr = nullptr;

// Use atomic for thread-safe access to logging state
static atomic_t logging_active = ATOMIC_INIT(0);
static atomic_t is_in_sleep_mode = ATOMIC_INIT(0);

// Local streaming state
static bool quaternion_streaming_enabled = false;

// Step count tracking variables
static uint32_t latest_step_count = 0;
static uint32_t activity_start_step_count = 0;
static uint32_t latest_activity_step_count = 0;
static bool activity_logging_active = false;

#include <zephyr/drivers/sensor.h> // For standard sensor value types

// Sensor IDs for BHI360 virtual sensors - Core sensors
static constexpr uint8_t QUAT_SENSOR_ID = BHY2_SENSOR_ID_GAMERV; // 28: Game Rotation Vector (no mag)
static constexpr uint8_t ACCEL_SENSOR_ID = BHY2_SENSOR_ID_ACC;   // 4: Calibrated Accelerometer (with gravity)
static constexpr uint8_t GYRO_SENSOR_ID = BHY2_SENSOR_ID_GYRO;   // 13: Calibrated Gyroscope

// Step counter variants - try multiple versions to find which one is available
static constexpr uint8_t STEP_COUNTER_SENSOR_ID = BHY2_SENSOR_ID_STC;  // 52: Step Counter (standard)
static constexpr uint8_t STEP_DETECTOR_SENSOR_ID = BHY2_SENSOR_ID_STD; // 50: Step Detector (individual steps)
static constexpr uint8_t STEP_COUNTER_WU_ID = BHY2_SENSOR_ID_STC_WU;   // 53: Step Counter Wake Up
static constexpr uint8_t STEP_DETECTOR_WU_ID = BHY2_SENSOR_ID_STD_WU;  // 94: Step Detector Wake Up
static constexpr uint8_t STEP_COUNTER_LP_ID = BHY2_SENSOR_ID_STC_LP;   // 136: Step Counter Low Power
static constexpr uint8_t STEP_DETECTOR_LP_ID = BHY2_SENSOR_ID_STD_LP;  // 137: Step Detector Low Power

static constexpr uint8_t ANY_MOTION_SENSOR_ID = BHY2_SENSOR_ID_ANY_MOTION_LP_WU; // Wake-up version

// Track which step counter variant is actually available
static uint8_t active_step_counter_id = 0;
static uint8_t active_step_detector_id = 0;

// Additional sensors available with HWActivity firmware
static constexpr uint8_t LACC_SENSOR_ID = BHY2_SENSOR_ID_LACC;   // 31: Linear Acceleration (gravity removed)
static constexpr uint8_t GRAVITY_SENSOR_ID = BHY2_SENSOR_ID_GRA; // 28: Gravity vector
static constexpr uint8_t ACTIVITY_SENSOR_ID = BHY2_SENSOR_ID_AR; // 63: Activity Recognition

// Track which sensors are actually available
static bool lacc_available = false;
static bool gravity_available = false;
static bool activity_available = false;

// Static variables to hold the latest sensor values (file scope for access across functions)
static float latest_quat_x = 0, latest_quat_y = 0, latest_quat_z = 0, latest_quat_w = 0, latest_quat_accuracy = 0;
static float latest_acc_x = 0, latest_acc_y = 0, latest_acc_z = 0;    // Accelerometer with gravity
static float latest_lacc_x = 0, latest_lacc_y = 0, latest_lacc_z = 0; // Linear acceleration (gravity removed)
static float latest_gyro_x = 0, latest_gyro_y = 0, latest_gyro_z = 0;
static float latest_gravity_x = 0, latest_gravity_y = 0, latest_gravity_z = 0; // Gravity vector
static uint8_t latest_activity_type = 0;                                       // Activity recognition result
static uint64_t latest_timestamp = 0;

// =================== STATE MACHINE AND CALIBRATION START ===================
enum class MotionSensorState
{
    POLLING,
    CALIBRATING_START_GYRO,
    CALIBRATING_ACCUMULATE_SAMPLES,
    CALIBRATING_FINISH
};

static MotionSensorState current_state = MotionSensorState::POLLING;
static int64_t state_start_time = 0;

// Variables for software offset calculation
static float accel_offset_x = 0.0f, accel_offset_y = 0.0f, accel_offset_z = 0.0f;
static float gyro_offset_x = 0.0f, gyro_offset_y = 0.0f, gyro_offset_z = 0.0f;

// Variables for accumulating data during calibration
static double acc_sum_x = 0, acc_sum_y = 0, acc_sum_z = 0;
static double gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
static uint32_t calibration_sample_count = 0;
static constexpr uint32_t CALIBRATION_DURATION_MS = 3000; // Calibrate for 3 seconds
// =================== STATE MACHINE AND CALIBRATION END =====================

// Forward declarations
static void motion_sensor_thread(void *p1, void *p2, void *p3);
static err_t init_bhi360(void);
static int8_t upload_firmware(struct bhy2_dev *dev);
static void parse_all_sensors(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
static void process_sensor_data(void);
static void handle_messages(void);

// perform_bhi360_calibration() is now removed and replaced by the state machine
static void apply_calibration_data(const bhi360_calibration_data_t *calib_data);
static void save_calibration_profile(const struct device *bhi360_dev, enum bhi360_sensor_type sensor,
                                     const char *sensor_name);
static void check_and_save_calibration_updates(const struct device *bhi360_dev);
static void bhi360_isr_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void bhi360_work_handler(struct k_work *work);
static void motion_sensor_enter_sleep_mode();

/**
 * @brief Initialize BHI360 sensor
 */
static err_t init_bhi360(void)
{
    uint32_t actual_len;
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

    // Check sensor availability before configuration
    LOG_INF("BHI360: Checking sensor availability with firmware");

    // First check if physical step counter/detector are present
    LOG_INF("  Checking physical sensors:");
    uint8_t phys_present[32] = {0};
    rslt = bhy2_get_parameter(BHY2_PARAM_SYS_PHYS_SENSOR_PRESENT, phys_present, sizeof(phys_present), &actual_len,
                              bhy2_ptr);
    if (rslt == BHY2_OK)
    {
        // Check for physical step counter (ID 32) and detector (ID 33)
        if (phys_present[32 / 8] & (1 << (32 % 8)))
        {
            LOG_INF("    Physical Step Counter (32) - Present");
        }
        else
        {
            LOG_WRN("    Physical Step Counter (32) - Not Present");
        }
        if (phys_present[33 / 8] & (1 << (33 % 8)))
        {
            LOG_INF("    Physical Step Detector (33) - Present");
        }
        else
        {
            LOG_WRN("    Physical Step Detector (33) - Not Present");
        }
    }

    // Check core sensors (these should always be available)
    if (bhy2_is_sensor_available(QUAT_SENSOR_ID, bhy2_ptr))
    {
        LOG_INF("  Quaternion (Game Rotation Vector) - Available");
    }
    else
    {
        LOG_ERR("  Quaternion sensor not available!");
        return err_t::MOTION_ERROR;
    }

    if (bhy2_is_sensor_available(ACCEL_SENSOR_ID, bhy2_ptr))
    {
        LOG_INF("  Accelerometer (with gravity) - Available");
    }
    else
    {
        LOG_WRN("  Accelerometer sensor not available");
    }

    if (bhy2_is_sensor_available(GYRO_SENSOR_ID, bhy2_ptr))
    {
        LOG_INF("  Gyroscope - Available");
    }
    else
    {
        LOG_ERR("  Gyroscope sensor not available!");
        return err_t::MOTION_ERROR;
    }

    // Check all step counter variants to find which one is available
    LOG_INF("  Checking step counter variants:");

    if (bhy2_is_sensor_available(STEP_COUNTER_SENSOR_ID, bhy2_ptr))
    {
        LOG_INF("    Step Counter (52) - Available");
        active_step_counter_id = STEP_COUNTER_SENSOR_ID;
    }
    else if (bhy2_is_sensor_available(STEP_COUNTER_WU_ID, bhy2_ptr))
    {
        LOG_INF("    Step Counter Wake Up (53) - Available");
        active_step_counter_id = STEP_COUNTER_WU_ID;
    }
    else if (bhy2_is_sensor_available(STEP_COUNTER_LP_ID, bhy2_ptr))
    {
        LOG_INF("    Step Counter Low Power (136) - Available");
        active_step_counter_id = STEP_COUNTER_LP_ID;
    }
    else
    {
        LOG_WRN("    No step counter variant available");
    }

    if (bhy2_is_sensor_available(STEP_DETECTOR_SENSOR_ID, bhy2_ptr))
    {
        LOG_INF("    Step Detector (50) - Available");
        active_step_detector_id = STEP_DETECTOR_SENSOR_ID;
    }
    else if (bhy2_is_sensor_available(STEP_DETECTOR_WU_ID, bhy2_ptr))
    {
        LOG_INF("    Step Detector Wake Up (94) - Available");
        active_step_detector_id = STEP_DETECTOR_WU_ID;
    }
    else if (bhy2_is_sensor_available(STEP_DETECTOR_LP_ID, bhy2_ptr))
    {
        LOG_INF("    Step Detector Low Power (137) - Available");
        active_step_detector_id = STEP_DETECTOR_LP_ID;
    }
    else
    {
        LOG_WRN("    No step detector variant available");
    }

    // Check additional sensors from HWActivity firmware
    lacc_available = bhy2_is_sensor_available(LACC_SENSOR_ID, bhy2_ptr);
    if (lacc_available)
    {
        LOG_INF("  Linear Acceleration (gravity removed) - Available");
    }
    else
    {
        LOG_WRN("  Linear Acceleration not available, will use regular accelerometer");
    }

    gravity_available = bhy2_is_sensor_available(GRAVITY_SENSOR_ID, bhy2_ptr);
    if (gravity_available)
    {
        LOG_INF("  Gravity Vector - Available");
    }
    else
    {
        LOG_DBG("  Gravity Vector not available");
    }

    activity_available = bhy2_is_sensor_available(ACTIVITY_SENSOR_ID, bhy2_ptr);
    if (activity_available)
    {
        LOG_INF("  Activity Recognition - Available");
    }
    else
    {
        LOG_DBG("  Activity Recognition not available");
    }

    // Configure sensors at 100Hz
    float motion_sensor_rate = 100.0f;
    float step_counter_rate = 20.0f;
    float activity_rate = 5.0f; // Activity recognition at 5Hz
    uint32_t report_latency_ms = 0;

    LOG_INF("BHI360: Configuring sensors");
    k_msleep(SAMPLE_PERIOD_MS);
    // Configure core sensors (required)
    rslt = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, motion_sensor_rate, report_latency_ms, bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to configure quaternion sensor: %d", rslt);
        return err_t::MOTION_ERROR;
    }
    k_msleep(SAMPLE_PERIOD_MS);
    // Configure accelerometer (we'll use this for now, even if LACC is available)
    rslt = bhy2_set_virt_sensor_cfg(ACCEL_SENSOR_ID, motion_sensor_rate, report_latency_ms, bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to configure accelerometer sensor: %d", rslt);
        return err_t::MOTION_ERROR;
    }
    k_msleep(SAMPLE_PERIOD_MS);
    rslt = bhy2_set_virt_sensor_cfg(GYRO_SENSOR_ID, motion_sensor_rate, report_latency_ms, bhy2_ptr);
    if (rslt != BHY2_OK)
    {
        LOG_ERR("Failed to configure gyroscope sensor: %d", rslt);
        return err_t::MOTION_ERROR;
    }
    k_msleep(SAMPLE_PERIOD_MS);

    // Configure whichever step counter variant is available
    if (active_step_counter_id != 0)
    {
        rslt = bhy2_set_virt_sensor_cfg(active_step_counter_id, step_counter_rate, report_latency_ms, bhy2_ptr);
        if (rslt != BHY2_OK)
        {
            LOG_ERR("Failed to configure step counter sensor (ID %u): %d", active_step_counter_id, rslt);
            active_step_counter_id = 0; // Mark as unavailable
        }
        else
        {
            LOG_INF("Step counter configured successfully (ID %u)", active_step_counter_id);
        }
    }

    k_msleep(SAMPLE_PERIOD_MS);

    // Configure whichever step detector variant is available
    if (active_step_detector_id != 0)
    {
        rslt = bhy2_set_virt_sensor_cfg(active_step_detector_id, step_counter_rate, report_latency_ms, bhy2_ptr);
        if (rslt != BHY2_OK)
        {
            LOG_WRN("Failed to configure step detector sensor (ID %u): %d", active_step_detector_id, rslt);
            active_step_detector_id = 0; // Mark as unavailable
        }
        else
        {
            LOG_INF("Step detector configured successfully (ID %u)", active_step_detector_id);
        }
    }
    k_msleep(SAMPLE_PERIOD_MS);
    // Configure optional sensors if available
    if (lacc_available)
    {
        rslt = bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, motion_sensor_rate, report_latency_ms, bhy2_ptr);
        if (rslt != BHY2_OK)
        {
            LOG_WRN("Failed to configure linear acceleration sensor: %d", rslt);
            lacc_available = false;
        }
    }
    k_msleep(SAMPLE_PERIOD_MS);
    if (gravity_available)
    {
        rslt = bhy2_set_virt_sensor_cfg(GRAVITY_SENSOR_ID, motion_sensor_rate, report_latency_ms, bhy2_ptr);
        if (rslt != BHY2_OK)
        {
            LOG_WRN("Failed to configure gravity sensor: %d", rslt);
            gravity_available = false;
        }
    }
    k_msleep(SAMPLE_PERIOD_MS);
    if (activity_available)
    {
        rslt = bhy2_set_virt_sensor_cfg(ACTIVITY_SENSOR_ID, activity_rate, report_latency_ms, bhy2_ptr);
        if (rslt != BHY2_OK)
        {
            LOG_WRN("Failed to configure activity recognition: %d", rslt);
            activity_available = false;
        }
    }
    k_msleep(SAMPLE_PERIOD_MS);
    // Register callbacks - ESSENTIAL for data parsing
    LOG_INF("BHI360: Registering callbacks");

    // Register core sensor callbacks (always register these)
    rslt = bhy2_register_fifo_parse_callback(QUAT_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
    rslt |= bhy2_register_fifo_parse_callback(ACCEL_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
    rslt |= bhy2_register_fifo_parse_callback(GYRO_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);

    // Register callbacks for whichever step counter/detector variants are available
    if (active_step_counter_id != 0)
    {
        rslt |= bhy2_register_fifo_parse_callback(active_step_counter_id, parse_all_sensors, NULL, bhy2_ptr);
    }
    if (active_step_detector_id != 0)
    {
        rslt |= bhy2_register_fifo_parse_callback(active_step_detector_id, parse_all_sensors, NULL, bhy2_ptr);
    }

    // Register optional sensor callbacks if available
    if (lacc_available)
    {
        rslt |= bhy2_register_fifo_parse_callback(LACC_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
    }
    if (gravity_available)
    {
        rslt |= bhy2_register_fifo_parse_callback(GRAVITY_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
    }
    if (activity_available)
    {
        rslt |= bhy2_register_fifo_parse_callback(ACTIVITY_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
    }

    // Always register meta event callback
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

    // For waking up
    if (bhy2_is_sensor_available(ANY_MOTION_SENSOR_ID, bhy2_ptr))
    {
        LOG_INF("Any Motion (Wake Up) sensor available.");
        bhy2_set_virt_sensor_cfg(ANY_MOTION_SENSOR_ID, 0, 0, bhy2_ptr);
    }
    else
    {
        LOG_WRN("Any Motion (Wake Up) sensor NOT available. Wake-from-sleep will not work!");
    }

    if (!gpio_is_ready_dt(&int_gpio))
    {
        LOG_ERR("BHI360 interrupt GPIO not ready");
        return err_t::MOTION_ERROR;
    }
    gpio_pin_configure_dt(&int_gpio, GPIO_INPUT);
    gpio_init_callback(&bhi360_int_cb_data, bhi360_isr_handler, BIT(int_gpio.pin));
    int ret = gpio_add_callback(int_gpio.port, &bhi360_int_cb_data);
    if (ret != 0)
    {
        LOG_ERR("Failed to add GPIO callback for BHI360 INT: %d", ret);
        return err_t::MOTION_ERROR;
    }
    gpio_pin_interrupt_configure_dt(&int_gpio, GPIO_INT_DISABLE);
    LOG_INF("BHI360 GPIO interrupt handler configured.");

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

        case ACCEL_SENSOR_ID: {
            struct bhy2_data_xyz data;
            bhy2_parse_xyz(callback_info->data_ptr, &data);

            // Accelerometer data (with gravity) is in m/s^2
            // BHI360 HWActivity firmware uses LSB = 1/1000 m/s^2
            latest_acc_x = (float)data.x / 1000.0f;
            latest_acc_y = (float)data.y / 1000.0f;
            latest_acc_z = (float)data.z / 1000.0f;

            // For backward compatibility, also update lacc variables if true LACC not available
            if (!lacc_available)
            {
                latest_lacc_x = latest_acc_x;
                latest_lacc_y = latest_acc_y;
                latest_lacc_z = latest_acc_z;
                motion_update_mask |= LACC_UPDATED;
            }
            break;
        }

        case LACC_SENSOR_ID: {
            if (lacc_available)
            {
                struct bhy2_data_xyz data;
                bhy2_parse_xyz(callback_info->data_ptr, &data);

                // Linear acceleration (gravity removed) is in m/s^2
                // BHI360 HWActivity firmware uses LSB = 1/1000 m/s^2
                latest_lacc_x = (float)data.x / 1000.0f;
                latest_lacc_y = (float)data.y / 1000.0f;
                latest_lacc_z = (float)data.z / 1000.0f;
                motion_update_mask |= LACC_UPDATED;
            }
            break;
        }

        case GRAVITY_SENSOR_ID: {
            if (gravity_available)
            {
                struct bhy2_data_xyz data;
                bhy2_parse_xyz(callback_info->data_ptr, &data);

                // Gravity vector is in m/s^2
                // BHI360 HWActivity firmware uses LSB = 1/1000 m/s^2
                latest_gravity_x = (float)data.x / 1000.0f;
                latest_gravity_y = (float)data.y / 1000.0f;
                latest_gravity_z = (float)data.z / 1000.0f;
            }
            break;
        }

        case ACTIVITY_SENSOR_ID: {
            if (activity_available)
            {
                // Activity recognition data format varies by firmware
                // Typically first byte is activity type
                if (callback_info->data_size >= 1)
                {
                    latest_activity_type = callback_info->data_ptr[0];
                    // Activity types (firmware dependent):
                    // 0: Unknown, 1: Still, 2: Walking, 3: Running, 4: On Bicycle, 5: In Vehicle, 6: Tilting
                }
            }
            break;
        }

        case GYRO_SENSOR_ID: {
            struct bhy2_data_xyz data;
            bhy2_parse_xyz(callback_info->data_ptr, &data);

            // Gyroscope data: BHI360 HWActivity firmware uses LSB = 1/2900 dps
            // Convert from degrees/s to rad/s: multiply by PI/180
            const float DEG_TO_RAD = 3.14159265359f / 180.0f;
            latest_gyro_x = ((float)data.x / 2900.0f) * DEG_TO_RAD;
            latest_gyro_y = ((float)data.y / 2900.0f) * DEG_TO_RAD;
            latest_gyro_z = ((float)data.z / 2900.0f) * DEG_TO_RAD;
            motion_update_mask |= GYRO_UPDATED;
            break;
        }

        case BHY2_SENSOR_ID_STD:    // 50: Step Detector
        case BHY2_SENSOR_ID_STD_WU: // 94: Step Detector Wake Up
        case BHY2_SENSOR_ID_STD_LP: // 137: Step Detector Low Power
        {
            // Step detector fires for each individual step detected
            // This helps ensure the step counter is working
            static uint32_t detector_count = 0;
            detector_count++;
            LOG_INF("Step detected (ID %u)! Total detector events: %u", callback_info->sensor_id, detector_count);
            break;
        }

        case BHY2_SENSOR_ID_STC:    // 52: Step Counter
        case BHY2_SENSOR_ID_STC_WU: // 53: Step Counter Wake Up
        case BHY2_SENSOR_ID_STC_LP: // 136: Step Counter Low Power
        {
            if (callback_info->data_size < 4)
            {
                LOG_ERR("Step counter: invalid data size: %d", callback_info->data_size);
                return;
            }

            // Parse the step count value
            uint32_t new_step_count = (callback_info->data_ptr[0]) | (callback_info->data_ptr[1] << 8) |
                                      (callback_info->data_ptr[2] << 16) | (callback_info->data_ptr[3] << 24);

            // Log raw step counter data for debugging
            static uint32_t last_logged_count = 0;
            if (new_step_count != last_logged_count)
            {
                LOG_INF("Step counter raw value changed: %u -> %u", last_logged_count, new_step_count);
                last_logged_count = new_step_count;
            }

            // Handle potential wrap-around
            static uint32_t previous_step_count = 0;
            static uint32_t step_accumulator = 0;

            if (new_step_count < previous_step_count && previous_step_count > 0)
            {
                LOG_WRN("Step count wrap detected: %u -> %u", previous_step_count, new_step_count);
                step_accumulator += (UINT32_MAX - previous_step_count) + new_step_count + 1;
            }
            else if (new_step_count > previous_step_count)
            {
                step_accumulator += (new_step_count - previous_step_count);
            }

            latest_step_count = step_accumulator;
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

    // Apply the software offsets
    // NOTE: Quaternion is a rotation vector and should not be offset like linear sensors.
    // If a "zero" rotation is needed, that logic should be handled at a higher level.
    float calibrated_lacc_x = latest_lacc_x - accel_offset_x;
    float calibrated_lacc_y = latest_lacc_y - accel_offset_y;
    float calibrated_lacc_z = latest_lacc_z - accel_offset_z;

    float calibrated_gyro_x = latest_gyro_x - gyro_offset_x;
    float calibrated_gyro_y = latest_gyro_y - gyro_offset_y;
    float calibrated_gyro_z = latest_gyro_z - gyro_offset_z;

    // Also apply to the raw accelerometer data
    float calibrated_acc_x = latest_acc_x - accel_offset_x;
    float calibrated_acc_y = latest_acc_y - accel_offset_y;
    float calibrated_acc_z = latest_acc_z - accel_offset_z;

    // Update driver with latest sensor data
    struct bhi360_sensor_data driver_data = {.quat_x = latest_quat_x,
                                             .quat_y = latest_quat_y,
                                             .quat_z = latest_quat_z,
                                             .quat_w = latest_quat_w,
                                             .quat_accuracy = (uint8_t)latest_quat_accuracy,
                                             .lacc_x = calibrated_lacc_x,
                                             .lacc_y = calibrated_lacc_y,
                                             .lacc_z = calibrated_lacc_z,
                                             .gyro_x = calibrated_gyro_x,
                                             .gyro_y = calibrated_gyro_y,
                                             .gyro_z = calibrated_gyro_z,
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
    msg.data.bhi360_log_record.lacc_x = calibrated_lacc_x;
    msg.data.bhi360_log_record.lacc_y = calibrated_lacc_y;
    msg.data.bhi360_log_record.lacc_z = calibrated_lacc_z;
    msg.data.bhi360_log_record.gyro_x = calibrated_gyro_x;
    msg.data.bhi360_log_record.gyro_y = calibrated_gyro_y;
    msg.data.bhi360_log_record.gyro_z = calibrated_gyro_z;
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

            // Log Quaternion data
            LOG_INF("BHI360 Quat: x=%.4f, y=%.4f, z=%.4f, w=%.4f, accuracy=%.1f", (double)latest_quat_x,
                    (double)latest_quat_y, (double)latest_quat_z, (double)latest_quat_w, (double)latest_quat_accuracy);

            // Log Gyroscope data (rad/s)
            LOG_INF("BHI360 Gyro (cal): x=%.4f, y=%.4f, z=%.4f rad/s", (double)calibrated_gyro_x,
                    (double)calibrated_gyro_y, (double)calibrated_gyro_z);

            // Log Accelerometer data (with gravity) - always available
            LOG_INF("BHI360 Accel (cal): x=%.4f, y=%.4f, z=%.4f m/s² (with gravity)", (double)calibrated_acc_x,
                    (double)calibrated_acc_y, (double)calibrated_acc_z);

            // Log Linear Acceleration data (gravity removed) - if different from accel
            if (lacc_available)
            {
                LOG_INF("BHI360 LinAcc (cal): x=%.4f, y=%.4f, z=%.4f m/s² (gravity removed)", (double)calibrated_lacc_x,
                        (double)calibrated_lacc_y, (double)calibrated_lacc_z);
            }

            // Log Gravity Vector - if available
            if (gravity_available)
            {
                LOG_INF("BHI360 Gravity: x=%.4f, y=%.4f, z=%.4f m/s²", (double)latest_gravity_x,
                        (double)latest_gravity_y, (double)latest_gravity_z);
            }

            // Log Activity Recognition - if available
            if (activity_available && latest_activity_type != 0)
            {
                const char *activity_name = "Unknown";
                switch (latest_activity_type)
                {
                    case 1:
                        activity_name = "Still";
                        break;
                    case 2:
                        activity_name = "Walking";
                        break;
                    case 3:
                        activity_name = "Running";
                        break;
                    case 4:
                        activity_name = "On Bicycle";
                        break;
                    case 5:
                        activity_name = "In Vehicle";
                        break;
                    case 6:
                        activity_name = "Tilting";
                        break;
                }
                LOG_INF("BHI360 Activity: %s (type=%u)", activity_name, latest_activity_type);
            }

            // Log Step Count data
            LOG_INF("BHI360 Steps: total=%u, activity=%u", latest_step_count, latest_activity_step_count);

            // Log timestamp (convert from nanoseconds to milliseconds for readability)
            uint32_t timestamp_ms = (uint32_t)(latest_timestamp / 1000000);
            LOG_INF("BHI360 Timestamp: %u ms", timestamp_ms);
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
                if (current_state == MotionSensorState::POLLING)
                {
                    LOG_INF("Starting calibration state machine.");
                    current_state = MotionSensorState::CALIBRATING_START_GYRO;
                }
                else
                {
                    LOG_WRN("Ignoring calibration trigger, another operation is in progress.");
                }
                break;

            case MSG_TYPE_BHI360_CALIBRATION_DATA:
                LOG_INF("Received calibration data from %s", get_sender_name(msg.sender));
                apply_calibration_data(&msg.data.bhi360_calibration);
                break;

            case MSG_TYPE_COMMAND:
                if (strncmp(msg.data.command_str, "ENTER_SLEEP", MAX_COMMAND_STRING_LEN) == 0)
                {
                    motion_sensor_enter_sleep_mode();
                }
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

    uint8_t work_buffer[WORK_BUFFER_SIZE];
    uint32_t calibration_check_counter = 0;
    uint32_t poll_counter = 0;

    // Main polling loop
    while (true)
    {
        // Handle incoming messages (e.g., to trigger calibration)
        handle_messages();

        // Always poll BHI360 FIFO to keep data fresh for all states
        if (bhy2_ptr)
        {
            int8_t rslt = bhy2_get_and_process_fifo(work_buffer, sizeof(work_buffer), bhy2_ptr);
            if (rslt != BHY2_OK)
            {
                LOG_WRN("BHI360: FIFO processing error: %d", rslt);
            }
        }

        // --- State Machine Logic ---
        switch (current_state)
        {
            case MotionSensorState::POLLING:
                // Periodically check and save hardware auto-calibration improvements
                if (++calibration_check_counter >= CALIBRATION_CHECK_INTERVAL)
                {
                    calibration_check_counter = 0;
                    check_and_save_calibration_updates(bhi360_dev);
                }

                // Debug: Log polling status periodically
                if (++poll_counter % 1000 == 0)
                { // Every 10 seconds at 100Hz
                    LOG_DBG("Motion sensor polling active - %u polls completed", poll_counter);
                }
                break;

            case MotionSensorState::CALIBRATING_START_GYRO:
                LOG_INF("Step 1: Gyro FOC. Keep device still...");
                struct bhi360_foc_result foc_result;
                bhi360_perform_gyro_foc(bhi360_dev, &foc_result);
                if (foc_result.success)
                {
                    LOG_INF("Gyro FOC successful.");
                }
                else
                {
                    LOG_WRN("Gyro FOC failed.");
                }

                LOG_INF("Step 2: Software Accel/Gyro offset capture. Keep still for %u ms...", CALIBRATION_DURATION_MS);
                // Reset accumulators
                acc_sum_x = 0;
                acc_sum_y = 0;
                acc_sum_z = 0;
                gyro_sum_x = 0;
                gyro_sum_y = 0;
                gyro_sum_z = 0;
                calibration_sample_count = 0;

                state_start_time = k_uptime_get();
                current_state = MotionSensorState::CALIBRATING_ACCUMULATE_SAMPLES;
                break;

            case MotionSensorState::CALIBRATING_ACCUMULATE_SAMPLES:
                // Data is collected via the FIFO processing that now runs every loop.
                // Here, we just add the latest values to our sums.
                acc_sum_x += latest_lacc_x; // Use linear accel if available
                acc_sum_y += latest_lacc_y;
                acc_sum_z += latest_lacc_z;
                gyro_sum_x += latest_gyro_x;
                gyro_sum_y += latest_gyro_y;
                gyro_sum_z += latest_gyro_z;
                calibration_sample_count++;

                if (k_uptime_get() - state_start_time >= CALIBRATION_DURATION_MS)
                {
                    current_state = MotionSensorState::CALIBRATING_FINISH;
                }
                break;

            case MotionSensorState::CALIBRATING_FINISH:
                if (calibration_sample_count > 0)
                {
                    // Calculate the average to find the offset
                    accel_offset_x = (float)(acc_sum_x / calibration_sample_count);
                    accel_offset_y = (float)(acc_sum_y / calibration_sample_count);
                    accel_offset_z = (float)(acc_sum_z / calibration_sample_count);
                    gyro_offset_x = (float)(gyro_sum_x / calibration_sample_count);
                    gyro_offset_y = (float)(gyro_sum_y / calibration_sample_count);
                    gyro_offset_z = (float)(gyro_sum_z / calibration_sample_count);

                    LOG_INF("Calibration complete. Samples: %u", calibration_sample_count);
                    LOG_INF("--> Accel Offsets: x=%.4f, y=%.4f, z=%.4f", (double)accel_offset_x, (double)accel_offset_y,
                            (double)accel_offset_z);
                    LOG_INF("--> Gyro Offsets: x=%.4f, y=%.4f, z=%.4f", (double)gyro_offset_x, (double)gyro_offset_y,
                            (double)gyro_offset_z);
                }
                else
                {
                    LOG_ERR("Calibration failed: no samples collected.");
                }
                // Return to normal polling mode
                current_state = MotionSensorState::POLLING;
                break;
        }

        // Sleep for 10ms to maintain the desired loop rate
        k_msleep(SAMPLE_PERIOD_MS);
    }
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

        LOG_WRN("Motion sensor initialization failed but continuing (non-critical)");
        module_set_state(MODULE_STATE_READY);
    }
    else
    {
        LOG_WRN("Motion sensor initialization ok");
        module_set_state(MODULE_STATE_READY);
    }

    // Create and start the sampling thread
    motion_sensor_tid =
        k_thread_create(&motion_sensor_thread_data, motion_sensor_stack, K_THREAD_STACK_SIZEOF(motion_sensor_stack),
                        motion_sensor_thread, NULL, NULL, NULL, CONFIG_MOTION_SENSOR_MODULE_PRIORITY, 0, K_NO_WAIT);

    //  k_thread_suspend(motion_sensor_tid);
    k_thread_name_set(motion_sensor_tid, "motion_sensor");
    LOG_INF("Motion sensor module initialized successfully");
}

// =================== Interrupt and Mode-Switching Logic ===================
static void bhi360_isr_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    k_work_submit(&bhi360_work);
}

static void bhi360_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    // ONLY reboot if the interrupt happens while we are in sleep mode
    if (atomic_get(&is_in_sleep_mode) == 1)
    {
        LOG_WRN("Motion detected during sleep, triggering system reboot!");
        k_msleep(100); // Allow log to flush
        sys_reboot(SYS_REBOOT_COLD);
    }
}

static void motion_sensor_enter_sleep_mode()
{
    LOG_INF("Switching BHI360 to low-power interrupt mode for wake-up.");
    atomic_set(&is_in_sleep_mode, 1); // Set the flag

    // Turn off high-frequency sensors
    bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, 0, 0, bhy2_ptr);
    bhy2_set_virt_sensor_cfg(GYRO_SENSOR_ID, 0, 0, bhy2_ptr);
    bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, 0, 0, bhy2_ptr);

    // Turn on the wake-up sensor
    bhy2_set_virt_sensor_cfg(ANY_MOTION_SENSOR_ID, 1.0f, 0, bhy2_ptr);

    // Enable the BHI360's interrupt line
    bhy2_set_host_interrupt_ctrl(0, bhy2_ptr);

    // Enable the MCU's GPIO interrupt
    gpio_pin_interrupt_configure_dt(&int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

// =================== Event Handling ===================

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
            k_thread_resume(motion_sensor_tid);
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
            k_thread_suspend(motion_sensor_tid);
        }
        return false;
    }

    if (is_streaming_control_event(aeh))
    {
        auto *event = cast_streaming_control_event(aeh);
        quaternion_streaming_enabled = event->quaternion_streaming_enabled;
        if (quaternion_streaming_enabled)
        {
            k_thread_resume(motion_sensor_tid);
        }
        else if (!quaternion_streaming_enabled)
        {
            k_thread_suspend(motion_sensor_tid);
        }
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
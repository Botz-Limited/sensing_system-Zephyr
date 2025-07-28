/**
 * @file motion_sensor_refactored.cpp
 * @brief Refactored motion sensor module using BHI360 driver
 * @version 2.0.0
 * @date 2024-12-19
 *
 * This is a refactored version showing integration with the new BHI360 driver.
 * Changes marked with // DRIVER_INTEGRATION:
 */

#include <hal/nrf_spim.h>
#include <zephyr/arch/arm/arch.h>
#define MODULE motion_sensor

/*************************** INCLUDE HEADERS ********************************/
#include <cstdio>
#include <cstring>
#include <time.h>

#include <app.hpp>
#include <app_event_manager.h>
#include <app_version.h>
#include <caf/events/module_state_event.h>
#include <events/app_state_event.h>
#include <events/motion_sensor_event.h>
#if defined(CONFIG_WIFI_MODULE)
#include <events/wifi_event.h>
#endif
#include <ble_services.hpp>
#include <motion_sensor.hpp>
#include <status_codes.h>

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "../bluetooth/ble_d2d_tx.hpp"
#endif /* CONFIG_WIFI_NRF70 */

// Removed: 3D orientation service no longer needed
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

// DRIVER_INTEGRATION: Include driver header instead of direct paths
#include "BHY2-Sensor-API/bhy2.h"
#include "BHY2-Sensor-API/bhy2_parse.h"
#include "BHY2-Sensor-API/firmware/bhi360/BHI360_Aux_BMM150.fw.h"
#include <bhi360.h>

#include <errors.hpp>

static constexpr uint16_t BHY2_RD_WR_LEN = 256;
static constexpr uint16_t WORK_BUFFER_SIZE = 2048;

LOG_MODULE_REGISTER(MODULE, CONFIG_LOG_DEFAULT_LEVEL);

// Get BHI360 device from device tree
#define BHI360_NODE DT_NODELABEL(bhi360)
static const struct device *const bhi360_dev = DEVICE_DT_GET(BHI360_NODE);

// DRIVER_INTEGRATION: Remove manual SPI config - driver handles this
// DRIVER_INTEGRATION: Remove manual GPIO specs - driver handles this

// Use atomic for thread-safe access to logging state
static atomic_t logging_active = ATOMIC_INIT(0);
static atomic_t wifi_active = ATOMIC_INIT(0);
static float configured_sample_rate = 50.0f;

// Step count tracking variables (file scope for access across functions)
static uint32_t latest_step_count = 0; // Global step count from sensor
static uint32_t activity_start_step_count =
    0; // Step count when activity started
static uint32_t latest_activity_step_count = 0; // Steps during current activity
static bool activity_logging_active = false;

// Sensor IDs for BHI360 virtual sensors
static constexpr uint8_t QUAT_SENSOR_ID = BHY2_SENSOR_ID_RV;
static constexpr uint8_t LACC_SENSOR_ID = BHY2_SENSOR_ID_LACC;
static constexpr uint8_t GYRO_SENSOR_ID = BHY2_SENSOR_ID_GYRO;
static constexpr uint8_t STEP_COUNTER_SENSOR_ID = BHY2_SENSOR_ID_STC;

// DRIVER_INTEGRATION: Remove local bhy2 struct - will get from driver
static struct bhy2_dev *bhy2_ptr = nullptr;

// DRIVER_INTEGRATION: Remove manual semaphore - driver handles interrupts

// Forward declarations
static void
parse_all_sensors(const struct bhy2_fifo_parse_data_info *callback_info,
                  void *callback_ref);
static void
parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info,
                 void *callback_ref);
#if 0
static void print_api_error(int8_t rslt, struct bhy2_dev *dev);
#endif
static int8_t upload_firmware(struct bhy2_dev *dev);
static void save_calibration_profile(const struct device *bhi360_dev,
                                     enum bhi360_sensor_type sensor,
                                     const char *sensor_name);
static void check_and_save_calibration_updates(const struct device *bhi360_dev);
static void perform_bhi360_calibration(void);
static void apply_calibration_data(const bhi360_calibration_data_t *calib_data);

/********************************** Motion Sensor THREAD
 * ********************************/
static constexpr int motion_sensor_stack_size =
    CONFIG_MOTION_SENSOR_MODULE_STACK_SIZE;
static constexpr int motion_sensor_priority =
    CONFIG_MOTION_SENSOR_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(motion_sensor_stack_area, motion_sensor_stack_size);
static struct k_thread motion_sensor_thread_data;
static k_tid_t motion_sensor_tid;
void motion_sensor_process(void *, void *, void *);

// DRIVER_INTEGRATION: Remove GPIO callback struct - driver handles this

void motion_sensor_initializing_entry() {
  LOG_INF("Motion Sensor initializing_entry done");
}

static void motion_sensor_init() {
  bool init_failed = false;
  motion_sensor_initializing_entry();
  LOG_INF("Starting BHI360 initialization using driver");

  // DRIVER_INTEGRATION: Check if driver is ready
  if (!device_is_ready(bhi360_dev)) {
    LOG_ERR("BHI360 device driver not ready");
    generic_message_t err_msg;
    err_msg.sender = SENDER_MOTION_SENSOR;
    err_msg.type = MSG_TYPE_ERROR_STATUS;
    err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
    err_msg.data.error_status.is_set = true;
    if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
      LOG_WRN("Failed to send error status to Bluetooth module");
    } else {
      LOG_INF("Sent error status to Bluetooth module");
    }
    init_failed = true;
  }

  // DRIVER_INTEGRATION: Get BHY2 device handle from driver
  if (!init_failed) {
    bhy2_ptr = bhi360_get_bhy2_dev(bhi360_dev);
    if (!bhy2_ptr) {
      LOG_ERR("Failed to get BHY2 device handle from driver");
      generic_message_t err_msg;
      err_msg.sender = SENDER_MOTION_SENSOR;
      err_msg.type = MSG_TYPE_ERROR_STATUS;
      err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
      err_msg.data.error_status.is_set = true;
      if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send error status to Bluetooth module");
      } else {
        LOG_INF("Sent error status to Bluetooth module");
      }
      init_failed = true;
    }
  }

  if (!init_failed) {
    LOG_INF("BHI360 driver ready, BHY2 handle obtained");
  }

  // DRIVER_INTEGRATION: Driver already performed:
  // - Hardware reset
  // - SPI initialization
  // - Product ID verification
  // - Interrupt configuration

  int8_t rslt;
  uint16_t version = 0;
  uint8_t hintr_ctrl, hif_ctrl, boot_status;

  if (!init_failed) {
    // Configure FIFO and interrupts (enable INT output)
    hintr_ctrl = 0; // Enable all INT outputs
    rslt = bhy2_set_host_interrupt_ctrl(hintr_ctrl, bhy2_ptr);
    if (rslt != BHY2_OK) {
      LOG_ERR("Failed to set host interrupt control: %d", rslt);
      generic_message_t err_msg;
      err_msg.sender = SENDER_MOTION_SENSOR;
      err_msg.type = MSG_TYPE_ERROR_STATUS;
      err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
      err_msg.data.error_status.is_set = true;
      if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send error status to Bluetooth module");
      } else {
        LOG_INF("Sent error status to Bluetooth module");
      }
      init_failed = true;
    }

    hif_ctrl = 0;
    rslt = bhy2_set_host_intf_ctrl(hif_ctrl, bhy2_ptr);
    if (rslt != BHY2_OK) {
      LOG_ERR("Failed to set host interface control: %d", rslt);
      generic_message_t err_msg;
      err_msg.sender = SENDER_MOTION_SENSOR;
      err_msg.type = MSG_TYPE_ERROR_STATUS;
      err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
      err_msg.data.error_status.is_set = true;
      if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send error status to Bluetooth module");
      } else {
        LOG_INF("Sent error status to Bluetooth module");
      }
      init_failed = true;
    }
  }

  // Check boot status and upload firmware
  if (!init_failed) {
    rslt = bhy2_get_boot_status(&boot_status, bhy2_ptr);
    if (rslt != BHY2_OK) {
      LOG_ERR("Failed to get boot status: %d", rslt);
      generic_message_t err_msg;
      err_msg.sender = SENDER_MOTION_SENSOR;
      err_msg.type = MSG_TYPE_ERROR_STATUS;
      err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
      err_msg.data.error_status.is_set = true;
      if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send error status to Bluetooth module");
      } else {
        LOG_INF("Sent error status to Bluetooth module");
      }
      init_failed = true;
    } else if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY)) {
      LOG_ERR("BHI360: Host interface not ready");
      generic_message_t err_msg;
      err_msg.sender = SENDER_MOTION_SENSOR;
      err_msg.type = MSG_TYPE_ERROR_STATUS;
      err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
      err_msg.data.error_status.is_set = true;
      if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send error status to Bluetooth module");
      } else {
        LOG_INF("Sent error status to Bluetooth module");
      }
      init_failed = true;
    }
  }

  if (!init_failed) {
    LOG_INF("BHI360: Uploading firmware");
    rslt = upload_firmware(bhy2_ptr);
    if (rslt != BHY2_OK) {
      LOG_ERR("BHI360: Firmware upload failed");
      generic_message_t err_msg;
      err_msg.sender = SENDER_MOTION_SENSOR;
      err_msg.type = MSG_TYPE_ERROR_STATUS;
      err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
      err_msg.data.error_status.is_set = true;
      if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send error status to Bluetooth module");
      } else {
        LOG_INF("Sent error status to Bluetooth module");
      }
      init_failed = true;
    }

    // Boot from RAM and verify
    if (!init_failed) {
      rslt = bhy2_boot_from_ram(bhy2_ptr);
      if (rslt != BHY2_OK) {
        LOG_ERR("BHI360: Boot from RAM failed: %d", rslt);
        generic_message_t err_msg;
        err_msg.sender = SENDER_MOTION_SENSOR;
        err_msg.type = MSG_TYPE_ERROR_STATUS;
        err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
        err_msg.data.error_status.is_set = true;
        if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send error status to Bluetooth module");
        } else {
          LOG_INF("Sent error status to Bluetooth module");
        }
        init_failed = true;
      }
    }

    if (!init_failed) {
      rslt = bhy2_get_kernel_version(&version, bhy2_ptr);
      if (rslt != BHY2_OK || version == 0) {
        LOG_ERR("BHI360: Boot verification failed");
        generic_message_t err_msg;
        err_msg.sender = SENDER_MOTION_SENSOR;
        err_msg.type = MSG_TYPE_ERROR_STATUS;
        err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
        err_msg.data.error_status.is_set = true;
        if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send error status to Bluetooth module");
        } else {
          LOG_INF("Sent error status to Bluetooth module");
        }
        init_failed = true;
      } else {
        LOG_INF("BHI360: Boot successful, kernel version %u", version);
      }
    }

    // Update virtual sensor list
    if (!init_failed) {
      LOG_INF("BHI360: Updating virtual sensor list");
      rslt = bhy2_update_virtual_sensor_list(bhy2_ptr);
      if (rslt != BHY2_OK) {
        LOG_ERR("Failed to update virtual sensor list: %d", rslt);
        generic_message_t err_msg;
        err_msg.sender = SENDER_MOTION_SENSOR;
        err_msg.type = MSG_TYPE_ERROR_STATUS;
        err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
        err_msg.data.error_status.is_set = true;
        if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send error status to Bluetooth module");
        } else {
          LOG_INF("Sent error status to Bluetooth module");
        }
        init_failed = true;
      }
    }

    // Configure sensor rates
    float motion_sensor_rate = 100.0f;
    float step_counter_rate = 20.0f;
    configured_sample_rate = motion_sensor_rate;
    uint32_t report_latency_ms = 0;

    // Configure sensors
    if (!init_failed) {
      LOG_INF("BHI360: Configuring sensors");
      rslt = bhy2_set_virt_sensor_cfg(QUAT_SENSOR_ID, motion_sensor_rate,
                                      report_latency_ms, bhy2_ptr);
      if (rslt != BHY2_OK) {
        LOG_ERR("Failed to configure quaternion sensor: %d", rslt);
        generic_message_t err_msg;
        err_msg.sender = SENDER_MOTION_SENSOR;
        err_msg.type = MSG_TYPE_ERROR_STATUS;
        err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
        err_msg.data.error_status.is_set = true;
        if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send error status to Bluetooth module");
        } else {
          LOG_INF("Sent error status to Bluetooth module");
        }
        init_failed = true;
      }

      rslt = bhy2_set_virt_sensor_cfg(LACC_SENSOR_ID, motion_sensor_rate,
                                      report_latency_ms, bhy2_ptr);
      if (rslt != BHY2_OK) {
        LOG_ERR("Failed to configure linear acceleration sensor: %d", rslt);
        generic_message_t err_msg;
        err_msg.sender = SENDER_MOTION_SENSOR;
        err_msg.type = MSG_TYPE_ERROR_STATUS;
        err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
        err_msg.data.error_status.is_set = true;
        if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send error status to Bluetooth module");
        } else {
          LOG_INF("Sent error status to Bluetooth module");
        }
        init_failed = true;
      }

      rslt = bhy2_set_virt_sensor_cfg(GYRO_SENSOR_ID, motion_sensor_rate,
                                      report_latency_ms, bhy2_ptr);
      if (rslt != BHY2_OK) {
        LOG_ERR("Failed to configure gyroscope sensor: %d", rslt);
        generic_message_t err_msg;
        err_msg.sender = SENDER_MOTION_SENSOR;
        err_msg.type = MSG_TYPE_ERROR_STATUS;
        err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
        err_msg.data.error_status.is_set = true;
        if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send error status to Bluetooth module");
        } else {
          LOG_INF("Sent error status to Bluetooth module");
        }
        init_failed = true;
      }

      rslt = bhy2_set_virt_sensor_cfg(STEP_COUNTER_SENSOR_ID, step_counter_rate,
                                      report_latency_ms, bhy2_ptr);
      if (rslt != BHY2_OK) {
        LOG_ERR("Failed to configure step counter sensor: %d", rslt);
        generic_message_t err_msg;
        err_msg.sender = SENDER_MOTION_SENSOR;
        err_msg.type = MSG_TYPE_ERROR_STATUS;
        err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
        err_msg.data.error_status.is_set = true;
        if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send error status to Bluetooth module");
        } else {
          LOG_INF("Sent error status to Bluetooth module");
        }
        init_failed = true;
      }
    }

    // Register callbacks
    if (!init_failed) {
      LOG_INF("BHI360: Registering callbacks");
      rslt = bhy2_register_fifo_parse_callback(
          QUAT_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
      rslt |= bhy2_register_fifo_parse_callback(
          LACC_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
      rslt |= bhy2_register_fifo_parse_callback(
          GYRO_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
      rslt |= bhy2_register_fifo_parse_callback(
          STEP_COUNTER_SENSOR_ID, parse_all_sensors, NULL, bhy2_ptr);
      rslt |= bhy2_register_fifo_parse_callback(
          BHY2_SYS_ID_META_EVENT, parse_meta_event, NULL, bhy2_ptr);

      if (rslt != BHY2_OK) {
        LOG_ERR("Failed to register callbacks: %d", rslt);
        generic_message_t err_msg;
        err_msg.sender = SENDER_MOTION_SENSOR;
        err_msg.type = MSG_TYPE_ERROR_STATUS;
        err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
        err_msg.data.error_status.is_set = true;
        if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send error status to Bluetooth module");
        } else {
          LOG_INF("Sent error status to Bluetooth module");
        }
        init_failed = true;
      }
    }

    if (!init_failed) {
      LOG_INF("BHI360: Initialization complete");

      // Request calibration data from data module
      LOG_INF("BHI360: Requesting calibration data from data module");
      generic_message_t request_msg = {};
      request_msg.sender = SENDER_MOTION_SENSOR;
      request_msg.type = MSG_TYPE_REQUEST_BHI360_CALIBRATION;

      if (k_msgq_put(&data_msgq, &request_msg, K_NO_WAIT) != 0) {
        LOG_ERR("Failed to request calibration data from data module");
      }
    }
  }

  // Handle initialization failure
  if (init_failed) {
    // Report error to Bluetooth module
    generic_message_t err_msg;
    err_msg.sender = SENDER_MOTION_SENSOR;
    err_msg.type = MSG_TYPE_ERROR_STATUS;
    err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
    err_msg.data.error_status.is_set = true;
    if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
      LOG_WRN("Failed to send error status to Bluetooth module");
    } else {
      LOG_INF("Sent error status to Bluetooth module");
    }

#if IS_ENABLED(CONFIG_MOTION_SENSOR_OPTIONAL)
    LOG_WRN(
        "Motion sensor initialization failed but continuing (non-critical)");
    module_set_state(MODULE_STATE_READY);
#else
    LOG_ERR("Motion sensor initialization failed (critical)");
    module_set_state(MODULE_STATE_ERROR);
    return;
#endif
  }

  // Start the motion sensor thread only if initialization succeeded
  if (!init_failed) {
    motion_sensor_tid = k_thread_create(
        &motion_sensor_thread_data, motion_sensor_stack_area,
        K_THREAD_STACK_SIZEOF(motion_sensor_stack_area), motion_sensor_process,
        nullptr, nullptr, nullptr, motion_sensor_priority, 0, K_NO_WAIT);
    LOG_INF("Motion Sensor Module Initialised");
  }
}

void motion_sensor_process(void *, void *, void *) {
  k_thread_name_set(motion_sensor_tid, "Motion_Sensor");
  module_set_state(MODULE_STATE_READY);
  uint8_t work_buffer[WORK_BUFFER_SIZE];
  uint32_t calibration_check_counter = 0;
  const uint32_t CALIBRATION_CHECK_INTERVAL =
      1000; // Check every 1000 iterations
  generic_message_t msg;

  // Check if BHY2 pointer is valid (in case we're in degraded mode)
  if (!bhy2_ptr) {
    LOG_ERR("BHY2 pointer is null, cannot process sensor data");
    generic_message_t err_msg;
    err_msg.sender = SENDER_MOTION_SENSOR;
    err_msg.type = MSG_TYPE_ERROR_STATUS;
    err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
    err_msg.data.error_status.is_set = true;
    if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
      LOG_WRN("Failed to send error status to Bluetooth module");
    } else {
      LOG_INF("Sent error status to Bluetooth module");
    }
    return;
  }

  while (true) {
    // Check for messages with a short timeout
    int msg_ret = k_msgq_get(&motion_sensor_msgq, &msg, K_MSEC(10));
    if (msg_ret == 0) {
      // Handle messages
      switch (msg.type) {
      case MSG_TYPE_TRIGGER_BHI360_CALIBRATION:
        LOG_INF("Received calibration trigger from %s",
                get_sender_name(msg.sender));
        perform_bhi360_calibration();
        break;

      case MSG_TYPE_BHI360_CALIBRATION_DATA:
        LOG_INF("Received calibration data from %s",
                get_sender_name(msg.sender));
        apply_calibration_data(&msg.data.bhi360_calibration);
        break;

      default:
        LOG_WRN("Unknown message type %d from %s", msg.type,
                get_sender_name(msg.sender));
        break;
      }
    }

    // Periodically check and save calibration improvements
    if (++calibration_check_counter >= CALIBRATION_CHECK_INTERVAL) {
      calibration_check_counter = 0;
      check_and_save_calibration_updates(bhi360_dev);
    }

    // DRIVER_INTEGRATION: Use driver's wait function instead of manual
    // semaphore
    int ret = bhi360_wait_for_data(bhi360_dev, K_MSEC(10));
    if (ret == 0) {
      // Process FIFO using BHY2 API directly
      int8_t rslt =
          bhy2_get_and_process_fifo(work_buffer, sizeof(work_buffer), bhy2_ptr);
      if (rslt != BHY2_OK) {
        LOG_WRN("BHI360: FIFO processing error: %d", rslt);
      }
    } else if (ret == -EAGAIN) {
      // Timeout is expected with short wait time
    } else if (ret != -EAGAIN) {
      LOG_ERR("BHI360: Wait for data error: %d", ret);
      generic_message_t err_msg;
      err_msg.sender = SENDER_MOTION_SENSOR;
      err_msg.type = MSG_TYPE_ERROR_STATUS;
      err_msg.data.error_status.error_code = err_t::MOTION_ERROR;
      err_msg.data.error_status.is_set = true;
      if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send error status to Bluetooth module");
      } else {
        LOG_INF("Sent error status to Bluetooth module");
      }
    }
  }
}

static void
parse_all_sensors(const struct bhy2_fifo_parse_data_info *callback_info,
                  void *callback_ref) {
  ARG_UNUSED(callback_ref);
  // Static variables to hold the latest values
  static float latest_quat_x = 0, latest_quat_y = 0, latest_quat_z = 0,
               latest_quat_w = 0, latest_quat_accuracy = 0;
  static float latest_lacc_x = 0, latest_lacc_y = 0, latest_lacc_z = 0;
  static float latest_gyro_x = 0, latest_gyro_y = 0, latest_gyro_z = 0;
  static uint64_t latest_timestamp = 0;

  // Synchronization tracking
  static uint8_t motion_update_mask = 0;
  static const uint8_t QUAT_UPDATED = 0x01;
  static const uint8_t LACC_UPDATED = 0x02;
  static const uint8_t GYRO_UPDATED = 0x04;
  static const uint8_t ALL_MOTION_SENSORS =
      (QUAT_UPDATED | LACC_UPDATED | GYRO_UPDATED);

  bool send_data = false;

  switch (callback_info->sensor_id) {
  case QUAT_SENSOR_ID: {
    struct bhy2_data_quaternion data;
    if (callback_info->data_size != 11) {
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
    latest_timestamp =
        *callback_info->time_stamp * 15625; // Convert to nanoseconds
    motion_update_mask |= QUAT_UPDATED;

    // Removed: No longer send orientation data before logging starts

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
    if (callback_info->data_size < 4) {
      LOG_ERR("Step counter: invalid data size: %d", callback_info->data_size);
      return;
    }
    latest_step_count =
        (callback_info->data_ptr[0]) | (callback_info->data_ptr[1] << 8) |
        (callback_info->data_ptr[2] << 16) | (callback_info->data_ptr[3] << 24);

    // Calculate activity step count if activity is active
    if (activity_logging_active) {
      latest_activity_step_count =
          latest_step_count - activity_start_step_count;

      // Send activity step count update to bluetooth (only during logging)
      if (atomic_get(&logging_active) == 1) {
        generic_message_t activity_step_msg{};
        activity_step_msg.sender =
            SENDER_MOTION_SENSOR; // Different sender to indicate activity steps
        activity_step_msg.type =
            MSG_TYPE_BHI360_STEP_COUNT; // Reuse same message type
        activity_step_msg.data.bhi360_step_count.step_count =
            latest_activity_step_count;
        activity_step_msg.data.bhi360_step_count.activity_duration_s = 0;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        k_msgq_put(&bluetooth_msgq, &activity_step_msg, K_NO_WAIT);
#else
        // Secondary device: Send activity steps to primary via D2D using
        // dedicated function
        ble_d2d_tx_send_activity_step_count(
            &activity_step_msg.data.bhi360_step_count);
#endif
      }
    }

    // Step counter updates independently, don't wait for it
    // It will be included with whatever value it has when motion sensors sync
    break;
  }
  default:
    LOG_WRN("Unknown sensor ID: %u", callback_info->sensor_id);
    break;
  }

  // Check if all motion sensors have been updated
  if ((motion_update_mask & ALL_MOTION_SENSORS) == ALL_MOTION_SENSORS) {
    send_data = true;
    motion_update_mask = 0; // Reset for next cycle
  }

  // Send data when we have a complete set or step counter update
  if (send_data) {
    // Update driver with latest sensor data for standard sensor API
    struct bhi360_sensor_data driver_data = {.quat_x = latest_quat_x,
                                             .quat_y = latest_quat_y,
                                             .quat_z = latest_quat_z,
                                             .quat_w = latest_quat_w,
                                             .quat_accuracy =
                                                 (uint8_t)latest_quat_accuracy,
                                             .lacc_x = latest_lacc_x,
                                             .lacc_y = latest_lacc_y,
                                             .lacc_z = latest_lacc_z,
                                             .gyro_x = latest_gyro_x,
                                             .gyro_y = latest_gyro_y,
                                             .gyro_z = latest_gyro_z,
                                             .step_count = latest_step_count,
                                             .timestamp = latest_timestamp};
    bhi360_update_sensor_data(bhi360_dev, &driver_data);

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
    if (atomic_get(&logging_active) == 1) {
      generic_message_t msg{};
      msg.sender = SENDER_BHI360_THREAD;
      msg.type = MSG_TYPE_BHI360_LOG_RECORD;
      msg.data.bhi360_log_record = record;
      // Don't log bhi360 data anymore at the moment
      // k_msgq_put(&data_msgq, &msg, K_NO_WAIT);

// Send to sensor data module (new multi-thread architecture)
#if IS_ENABLED(CONFIG_SENSOR_DATA_MODULE)
      k_msgq_put(&sensor_data_msgq, &msg, K_NO_WAIT);
#endif

      // Also send to WiFi module if WiFi is active
#if defined(CONFIG_WIFI_MODULE)
      if (atomic_get(&wifi_active) == 1) {
        k_msgq_put(&wifi_msgq, &msg, K_NO_WAIT);
      }
#endif

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
      qmsg.data.bhi360_3d_mapping.quat_w =
          latest_quat_w; // Now including W component
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
      k_msgq_put(&bluetooth_msgq, &qmsg, K_NO_WAIT);
#if IS_ENABLED(CONFIG_SENSOR_DATA_MODULE)
// k_msgq_put(&sensor_data_msgq, &qmsg, K_NO_WAIT);
#endif
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

      // Step count (global count, no duration)
      generic_message_t smsg{};
      smsg.sender = SENDER_BHI360_THREAD;
      smsg.type = MSG_TYPE_BHI360_STEP_COUNT;
      smsg.data.bhi360_step_count.step_count = latest_step_count;
      smsg.data.bhi360_step_count.activity_duration_s =
          0; // Deprecated - always 0
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
      k_msgq_put(&bluetooth_msgq, &smsg, K_NO_WAIT);
#if IS_ENABLED(CONFIG_SENSOR_DATA_MODULE)
      // k_msgq_put(&sensor_data_msgq, &smsg, K_NO_WAIT);
#endif
#else
      // Secondary device: Send to primary via D2D
      ble_d2d_tx_send_bhi360_data2(&smsg.data.bhi360_step_count);
#endif

      // Also send activity step count to data module for activity file
      generic_message_t activity_msg{};
      activity_msg.sender = SENDER_MOTION_SENSOR;
      activity_msg.type = MSG_TYPE_ACTIVITY_STEP_COUNT;
// Send activity-specific step count (not global count)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
      activity_msg.data.activity_step_count.left_step_count =
          latest_activity_step_count;
      activity_msg.data.activity_step_count.right_step_count = 0;
#else
      activity_msg.data.activity_step_count.left_step_count = 0;
      activity_msg.data.activity_step_count.right_step_count =
          latest_activity_step_count;
#endif
      k_msgq_put(&data_msgq, &activity_msg, K_NO_WAIT);
    }
  }
}

// Currently unused - kept for future debugging
#if 0
static void print_api_error(int8_t rslt, struct bhy2_dev *dev)
{
    // [Implementation remains unchanged]
    if (rslt != BHY2_OK) {
        LOG_ERR("API error: %d", rslt);
        if ((rslt == BHY2_E_IO) && (dev != NULL)) {
            LOG_ERR("Interface error: %d", dev->hif.intf_rslt);
            dev->hif.intf_rslt = BHY2_INTF_RET_SUCCESS;
        }
    }
}
#endif

static void
parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info,
                 void *callback_ref) {
  (void)callback_ref;
  uint8_t meta_event_type = callback_info->data_ptr[0];
  uint8_t byte1 = callback_info->data_ptr[1];
  uint8_t byte2 = callback_info->data_ptr[2];
  char *event_text;
  if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT) {
    event_text = (char *)"[META EVENT]";
  } else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU) {
    event_text = (char *)"[META EVENT WAKE UP]";
  } else {
    return;
  }
  switch (meta_event_type) {
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
    LOG_INF("%s Accuracy for sensor id %u changed to %u\r\n", event_text, byte1,
            byte2);
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
    LOG_INF("%s Sensor id %u reported error 0x%02X\r\n", event_text, byte1,
            byte2);
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
    LOG_INF("%s Firmware initialized. Firmware version %u\r\n", event_text,
            ((uint16_t)byte2 << 8) | byte1);
    break;
  case BHY2_META_TRANSFER_CAUSE:
    LOG_INF("%s Transfer cause for sensor id %u\r\n", event_text, byte1);
    break;
  case BHY2_META_EVENT_SENSOR_FRAMEWORK:
    LOG_INF("%s Sensor framework event for sensor id %u\r\n", event_text,
            byte1);
    break;
  case BHY2_META_EVENT_RESET:
    LOG_INF("%s Reset event\r\n", event_text);
    break;
  case BHY2_META_EVENT_SPACER:
    break;
  default:
    LOG_INF("%s Unknown meta event with id: %u\r\n", event_text,
            meta_event_type);
    break;
  }
}

static int8_t upload_firmware(struct bhy2_dev *dev) {
  uint32_t incr = 256; /* Max command packet size */
  uint32_t len = sizeof(bhy2_firmware_image);
  int8_t rslt = BHY2_OK;
  if ((incr % 4) != 0) /* Round off to higher 4 bytes */
  {
    incr = ((incr >> 2) + 1) << 2;
  }
  for (uint32_t i = 0; (i < len) && (rslt == BHY2_OK); i += incr) {
    if (incr > (len - i)) /* If last payload */
    {
      incr = len - i;
      if ((incr % 4) != 0) /* Round off to higher 4 bytes */
      {
        incr = ((incr >> 2) + 1) << 2;
      }
    }
#ifdef UPLOAD_FIRMWARE_TO_FLASH
    rslt = bhy2_upload_firmware_to_flash_partly(&bhy2_firmware_image[i], i,
                                                incr, dev);
#else
    rslt = bhy2_upload_firmware_to_ram_partly(&bhy2_firmware_image[i], len, i,
                                              incr, dev);
#endif
    LOG_INF("%.2f%% complete",
            (double)((float)(i + incr) / (float)len * 100.0f));
  }
  return rslt;
}

// DRIVER_INTEGRATION: Remove bhi360_delay_us - driver provides this
// DRIVER_INTEGRATION: Remove bhi360_int_handler - driver handles interrupts

/**
 * @brief Apply calibration data received from data module
 */
static void
apply_calibration_data(const bhi360_calibration_data_t *calib_data) {
  if (!calib_data) {
    LOG_ERR("Invalid calibration data");
    return;
  }

  enum bhi360_sensor_type sensor_type;
  const char *sensor_name;

  switch (calib_data->sensor_type) {
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

  if (calib_data->profile_size > 0) {
    int ret = bhi360_set_calibration_profile(bhi360_dev, sensor_type,
                                             calib_data->profile_data,
                                             calib_data->profile_size);
    if (ret == 0) {
      LOG_INF("Applied %s calibration (%u bytes)", sensor_name,
              calib_data->profile_size);
    } else {
      LOG_ERR("Failed to apply %s calibration: %d", sensor_name, ret);
    }
  } else {
    LOG_INF("No %s calibration data available", sensor_name);
  }
}

/**
 * @brief Perform BHI360 calibration when triggered
 */
static void perform_bhi360_calibration(void) {
  LOG_INF("Starting BHI360 calibration sequence...");

  // Check current calibration status
  struct bhi360_calibration_status calib_status;
  int ret = bhi360_get_calibration_status(bhi360_dev, &calib_status);
  if (ret == 0) {
    LOG_INF("Current calibration status - Accel: %d, Gyro: %d",
            calib_status.accel_calib_status, calib_status.gyro_calib_status);
  }

  // Perform gyroscope calibration
  LOG_INF("Performing gyroscope calibration...");
  LOG_INF("Please keep the device stationary");

  // Give user time to ensure device is stationary
  k_sleep(K_SECONDS(2));

  struct bhi360_foc_result foc_result;
  ret = bhi360_perform_gyro_foc(bhi360_dev, &foc_result);
  if (ret == 0 && foc_result.success) {
    LOG_INF("Gyro calibration successful! Offsets: X=%d, Y=%d, Z=%d",
            foc_result.x_offset, foc_result.y_offset, foc_result.z_offset);

    // Save the new calibration
    save_calibration_profile(bhi360_dev, BHI360_SENSOR_GYRO, "gyro");
  } else {
    LOG_ERR("Gyro calibration failed");
  }

  // Perform accelerometer calibration
  LOG_INF("Performing accelerometer calibration...");
  LOG_INF("Please place device on flat surface with Z-axis up");

  // Give user time to position device
  k_sleep(K_SECONDS(3));

  ret = bhi360_perform_accel_foc(bhi360_dev, &foc_result);
  if (ret == 0 && foc_result.success) {
    LOG_INF("Accel calibration successful! Offsets: X=%d, Y=%d, Z=%d",
            foc_result.x_offset, foc_result.y_offset, foc_result.z_offset);

    // Save the new calibration
    save_calibration_profile(bhi360_dev, BHI360_SENSOR_ACCEL, "accel");
  } else {
    LOG_ERR("Accel calibration failed");
  }

  // Check final calibration status
  ret = bhi360_get_calibration_status(bhi360_dev, &calib_status);
  if (ret == 0) {
    LOG_INF("Final calibration status - Accel: %d, Gyro: %d",
            calib_status.accel_calib_status, calib_status.gyro_calib_status);
  }

  LOG_INF("BHI360 calibration sequence complete");
}

/**
 * @brief Save calibration profile after successful calibration
 */
static void save_calibration_profile(const struct device *bhi360_dev,
                                     enum bhi360_sensor_type sensor,
                                     const char *sensor_name) {
  uint8_t type_id;

  // Map sensor type to storage ID
  switch (sensor) {
  case BHI360_SENSOR_ACCEL:
    type_id = 0;
    break;
  case BHI360_SENSOR_GYRO:
    type_id = 1;
    break;
  default:
    return;
  }

  // Get current calibration profile from BHI360
  generic_message_t save_msg = {};
  save_msg.sender = SENDER_MOTION_SENSOR;
  save_msg.type = MSG_TYPE_SAVE_BHI360_CALIBRATION;
  save_msg.data.bhi360_calibration.sensor_type = type_id;

  size_t actual_size;
  int ret = bhi360_get_calibration_profile(
      bhi360_dev, sensor, save_msg.data.bhi360_calibration.profile_data,
      sizeof(save_msg.data.bhi360_calibration.profile_data), &actual_size);
  save_msg.data.bhi360_calibration.profile_size = (uint16_t)actual_size;
  if (ret == 0) {
    // Send to data module for storage
    if (k_msgq_put(&data_msgq, &save_msg, K_NO_WAIT) != 0) {
      LOG_ERR("Failed to send %s calibration to data module", sensor_name);
    } else {
      LOG_INF("Sent %s calibration to data module (%u bytes)", sensor_name,
              save_msg.data.bhi360_calibration.profile_size);
    }
  } else {
    LOG_ERR("Failed to get %s calibration profile: %d", sensor_name, ret);
  }
}

/**
 * @brief Periodic calibration check and save
 *
 * This function is called periodically to check if calibration
 * has improved and save it. Particularly useful for magnetometer.
 */
static void
check_and_save_calibration_updates(const struct device *bhi360_dev) {
  static struct bhi360_calibration_status last_status = {0, 0, 0};
  struct bhi360_calibration_status current_status;

  int ret = bhi360_get_calibration_status(bhi360_dev, &current_status);
  if (ret != 0) {
    return;
  }

  // Check if any calibration status has improved
  if (current_status.accel_calib_status > last_status.accel_calib_status) {
    LOG_INF("Accel calibration improved: %d -> %d",
            last_status.accel_calib_status, current_status.accel_calib_status);
    save_calibration_profile(bhi360_dev, BHI360_SENSOR_ACCEL, "accel");
  }

  if (current_status.gyro_calib_status > last_status.gyro_calib_status) {
    LOG_INF("Gyro calibration improved: %d -> %d",
            last_status.gyro_calib_status, current_status.gyro_calib_status);
    save_calibration_profile(bhi360_dev, BHI360_SENSOR_GYRO, "gyro");
  }

  last_status = current_status;
}

static bool app_event_handler(const struct app_event_header *aeh) {
  if (is_module_state_event(aeh)) {
    auto *event = cast_module_state_event(aeh);
    if (check_state(event, MODULE_ID(foot_sensor), MODULE_STATE_READY)) {
      motion_sensor_init();
    }
    return false;
  }
  if (is_motion_sensor_start_activity_event(aeh)) {
    if (atomic_get(&logging_active) == 0) {
      // Mark activity as started and capture current step count
      activity_logging_active = true;
      atomic_set(&logging_active, 1);

      // Also start activity logging
    }
    return false;
  }
  if (is_motion_sensor_stop_activity_event(aeh)) {
    LOG_INF("Received motion_sensor_stop_activity_event");
    if (atomic_get(&logging_active) == 1) {
      // Mark activity as stopped
      activity_logging_active = false;
      
      atomic_set(&logging_active, 0);
    }
    return false;
  }
#if defined(CONFIG_WIFI_MODULE)
  if (is_wifi_connected_event(aeh)) {
    LOG_INF("Motion sensor: WiFi connected - enabling WiFi data transmission");
    atomic_set(&wifi_active, 1);
    return false;
  }
  if (is_wifi_disconnected_event(aeh)) {
    LOG_INF(
        "Motion sensor: WiFi disconnected - disabling WiFi data transmission");
    atomic_set(&wifi_active, 0);
    return false;
  }
#endif
  return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, motion_sensor_stop_activity_event);
#if defined(CONFIG_WIFI_MODULE)
APP_EVENT_SUBSCRIBE(MODULE, wifi_connected_event);
APP_EVENT_SUBSCRIBE(MODULE, wifi_disconnected_event);
#endif

/*
 * DRIVER_INTEGRATION SUMMARY:
 *
 * 1. Removed manual SPI initialization - driver handles this
 * 2. Removed GPIO configuration - driver handles interrupts
 * 3. Get BHY2 device from driver API
 * 4. Use driver's wait_for_data function
 * 5. Kept all application logic unchanged
 *
 * Benefits:
 * - Cleaner code with less hardware-specific details
 * - Driver manages device lifecycle
 * - Better error handling and recovery
 * - Thread-safe interrupt handling
 *
 * Next steps:
 * - Test the refactored code
 * - Consider moving firmware upload to driver
 * - Implement standard sensor channels in driver
 */
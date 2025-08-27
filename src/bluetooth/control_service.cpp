/**
 * @file control_service.cpp
 * @author
 * @brief
 * @version 2.4.1
 * @date 2025-05-12
 *
 * @copyright Botz Innovation 2025
 *
 */
#define MODULE bluetooth

#include <array>
#include <cstdint>
#include <cstring>
#include <errno.h>
#include <iomanip>
#include <sstream>
#include <stddef.h>
#include <string.h>
#include <string_view>

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#include "ble_conn_params.hpp"
#include "ble_services.hpp"
#include "ccc_callback_fix.hpp"
#include <app.hpp>
#include <app_version.h>

#include <app_event_manager.h>
#include <events/foot_sensor_event.h>
#include <events/motion_sensor_event.h>

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "ble_d2d_tx.hpp"
#include "ble_d2d_tx_queue.hpp"
#endif

LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Constants
#define VND_MAX_LEN 128 // Maximum length for vendor characteristic data

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
// Helper macro to forward D2D commands with automatic queuing if not ready
#define FORWARD_D2D_COMMAND(func, cmd_type, value, desc)                       \
  do {                                                                         \
    int err = func(value);                                                     \
    if (err == -EINVAL && !ble_d2d_tx_is_ready()) {                            \
      LOG_INF("D2D TX not ready, queuing " desc);                              \
      err = ble_d2d_tx_queue_command(cmd_type, &value);                        \
      if (err) {                                                               \
        LOG_ERR("Failed to queue " desc ": %d", err);                          \
      }                                                                        \
    } else if (err) {                                                          \
      LOG_ERR("Failed to forward " desc " to secondary: %d", err);             \
    } else {                                                                   \
      LOG_INF("Successfully forwarded " desc " to secondary");                 \
    }                                                                          \
  } while (0)
#endif

// External function declarations
extern "C" void set_current_time_from_epoch(uint32_t new_epoch_time_s);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE) 
extern err_t ble_reset_bonds(void);
#endif

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static constexpr uint8_t is_little_endian = 1;
#else
static constexpr uint8_t is_little_endian = 0;
#endif

static uint32_t set_time_control = 0;
static bool status_subscribed = false;
static uint8_t external_flash_erase_status = 0; // 0=idle, 1=erasing, 2=complete

static uint32_t swap_to_little_endian(uint32_t value);

static ssize_t write_set_time_control_vnd(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          const void *buf, uint16_t len,
                                          uint16_t offset, uint8_t flags);

/**
 * @brief UUID for the control service. This UUID identifies the control service
 * used for handling various commands over Bluetooth communication.
 */
static struct bt_uuid_128 control_service_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b67f, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for the start activity characteristic. ---
static struct bt_uuid_128 start_activity_command_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b684, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for the stop activity characteristic. ---
static struct bt_uuid_128 stop_activity_command_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b685, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for the trigger BHI360 calibration characteristic. ---
static struct bt_uuid_128 trigger_bhi360_calibration_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b686, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for the delete activity log command characteristic. ---
static struct bt_uuid_128 delete_activity_log_command_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b687, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

static struct bt_uuid_128 delete_secondary_activity_log_command_uuid =
    BT_UUID_INIT_128(
        BT_UUID_128_ENCODE(0x4fd5b68a, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for connection parameter control characteristic ---
static struct bt_uuid_128 conn_param_control_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b68b, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for weight calibration characteristic ---
static struct bt_uuid_128 weight_calibration_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b68d, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for GPS update characteristic ---
static struct bt_uuid_128 gps_update_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b68e, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for reset bonds characteristic ---
static struct bt_uuid_128 reset_bonds_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b68f, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for user info characteristic ---
static struct bt_uuid_128 user_info_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b690, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for erase external flash characteristic ---
static struct bt_uuid_128 erase_external_flash_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b691, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Forward declarations for start/stop activity handlers
static ssize_t write_start_activity_command_vnd(struct bt_conn *conn,
                                                const struct bt_gatt_attr *attr,
                                                const void *buf, uint16_t len,
                                                uint16_t offset, uint8_t flags);
static void cs_start_activity_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                              uint16_t value);
static ssize_t write_stop_activity_command_vnd(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               const void *buf, uint16_t len,
                                               uint16_t offset, uint8_t flags);
static void cs_stop_activity_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                             uint16_t value);
static ssize_t write_trigger_bhi360_calibration_vnd(
    struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
    uint16_t len, uint16_t offset, uint8_t flags);
static void
cs_trigger_bhi360_calibration_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                              uint16_t value);
static ssize_t write_delete_activity_log_command_vnd(
    struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
    uint16_t len, uint16_t offset, uint8_t flags);
static void
cs_delete_activity_log_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value);

// Secondary device delete command handlers

static ssize_t write_delete_secondary_activity_log_command_vnd(
    struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
    uint16_t len, uint16_t offset, uint8_t flags);
static void cs_delete_secondary_activity_log_ccc_cfg_changed(
    const struct bt_gatt_attr *attr, uint16_t value);

// Connection parameter control handlers
static ssize_t write_conn_param_control_vnd(struct bt_conn *conn,
                                            const struct bt_gatt_attr *attr,
                                            const void *buf, uint16_t len,
                                            uint16_t offset, uint8_t flags);
static ssize_t read_conn_param_control_vnd(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset);

// Weight calibration handlers
static ssize_t write_weight_calibration_trigger_vnd(
    struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
    uint16_t len, uint16_t offset, uint8_t flags);
static void
cs_weight_calibration_trigger_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                              uint16_t value);

// GPS update handlers
static ssize_t write_gps_update_vnd(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len,
                                    uint16_t offset, uint8_t flags);

// Reset bonds handlers
static ssize_t write_reset_bonds_vnd(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset, uint8_t flags);

// User info handlers
static ssize_t write_user_info_vnd(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len,
                                   uint16_t offset, uint8_t flags);

// Erase external flash handlers
static ssize_t write_erase_external_flash_vnd(struct bt_conn *conn,
                                              const struct bt_gatt_attr *attr,
                                              const void *buf, uint16_t len,
                                              uint16_t offset, uint8_t flags);
static ssize_t read_erase_external_flash_vnd(struct bt_conn *conn,
                                             const struct bt_gatt_attr *attr,
                                             void *buf, uint16_t len,
                                             uint16_t offset);

/**
 * @brief Structure for user information data
 * Age in years (1 byte), Gender (1 byte), Height in mm (2 bytes), Weight in decigrams (2 bytes)
 */
struct __attribute__((packed)) UserInfoData {
    uint8_t age_years;       // Age in years
    uint8_t gender;          // Gender identifier
    uint16_t height_mm;      // Height in millimeters
    uint16_t weight_dg;      // Weight in decigrams (1 dg = 0.1 g = 0.0001 kg)
};

/**
 * @brief UUID for the set time command characteristic.
 * This UUID identifies the characteristic for setting the device's time.
 */
static struct bt_uuid_128 set_time_command_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x4fd5b681, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

/**
 * @brief Control Service GATT Declaration
 *
 * This GATT declaration defines the control service and its characteristics
 * for controlling various functionalities over Bluetooth communication.
 *
 * Characteristics:
 * - Command Point Characteristic for controlling bounce and amplitude.
 * - Time Characteristic for setting the device's time.
 * - Shipping Mode Characteristic for enabling/disabling shipping mode.
 * - Delete Log Characteristics for deleting hardware and metadata logs.
 * - Debug Command Characteristic for executing debug commands.
 */

BT_GATT_SERVICE_DEFINE(
    control_service, BT_GATT_PRIMARY_SERVICE(&control_service_uuid),

    // Command Point Characteristic

    // time characteristics
    BT_GATT_CHARACTERISTIC(&set_time_command_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE, nullptr,
                           write_set_time_control_vnd,
                           static_cast<void *>(&set_time_control)),


    // New: Start Activity Characteristic
    BT_GATT_CHARACTERISTIC(&start_activity_command_uuid.uuid,
                           BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, nullptr,
                           write_start_activity_command_vnd, nullptr),
    BT_GATT_CCC(cs_start_activity_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // New: Stop Activity Characteristic
    BT_GATT_CHARACTERISTIC(&stop_activity_command_uuid.uuid, BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE, nullptr,
                           write_stop_activity_command_vnd, nullptr),
    BT_GATT_CCC(cs_stop_activity_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // New: Trigger BHI360 Calibration Characteristic
    BT_GATT_CHARACTERISTIC(&trigger_bhi360_calibration_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_WRITE, nullptr,
                           write_trigger_bhi360_calibration_vnd, nullptr),
    BT_GATT_CCC(cs_trigger_bhi360_calibration_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // New: Delete Activity Log Characteristic
    BT_GATT_CHARACTERISTIC(&delete_activity_log_command_uuid.uuid,
                           BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, nullptr,
                           write_delete_activity_log_command_vnd, nullptr),
    BT_GATT_CCC(cs_delete_activity_log_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Secondary device delete commands - only on primary

    BT_GATT_CHARACTERISTIC(&delete_secondary_activity_log_command_uuid.uuid,
                           BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, nullptr,
                           write_delete_secondary_activity_log_command_vnd,
                           nullptr),
    BT_GATT_CCC(cs_delete_secondary_activity_log_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif

    // Connection Parameter Control Characteristic
    BT_GATT_CHARACTERISTIC(&conn_param_control_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_conn_param_control_vnd,
                           write_conn_param_control_vnd, nullptr),

    // Weight Calibration Trigger Characteristic
    BT_GATT_CHARACTERISTIC(&weight_calibration_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_WRITE, nullptr,
                           write_weight_calibration_trigger_vnd, nullptr),
    BT_GATT_CCC(cs_weight_calibration_trigger_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // GPS Update Characteristic
    BT_GATT_CHARACTERISTIC(&gps_update_uuid.uuid, BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE, nullptr, write_gps_update_vnd,
                           nullptr),

    // Reset Bonds Characteristic
    BT_GATT_CHARACTERISTIC(&reset_bonds_uuid.uuid, BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE, nullptr, write_reset_bonds_vnd,
                           nullptr),

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // User Info Characteristic (primary device only)
    BT_GATT_CHARACTERISTIC(&user_info_uuid.uuid, BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE, nullptr, write_user_info_vnd,
                           nullptr),
#endif

    // Erase External Flash Characteristic
    BT_GATT_CHARACTERISTIC(&erase_external_flash_uuid.uuid,
                           BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE,
                           nullptr,
                           write_erase_external_flash_vnd,
                           &external_flash_erase_status),
    );

/**
 * @brief Notifies subscribed devices about the metadata log deletion command.
 * This function notifies subscribed devices about the metadata log deletion
 * command.
 *
 * @param stu The metadata log deletion command value.
 */





/**
 * @brief Converts a 32-bit value from big endian to little endian.
 *
 * This function converts a 32-bit value from big endian to little endian.
 *
 * @param value The value to be converted.
 * @return The converted value in little endian format.
 */
static uint32_t swap_to_little_endian(uint32_t value) {
  return ((value << 24) & 0xFF000000) | ((value << 8) & 0x00FF0000) |
         ((value >> 8) & 0x0000FF00) | ((value >> 24) & 0x000000FF);
}

// --- Start Activity Characteristic Handlers ---
static bool start_activity_status_subscribed = false;
static bool stop_activity_status_subscribed = false;



static ssize_t write_start_activity_command_vnd(struct bt_conn *conn,
                                                const struct bt_gatt_attr *attr,
                                                const void *buf, uint16_t len,
                                                uint16_t offset,
                                                uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  if (len != sizeof(uint8_t)) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  uint8_t value = *((const uint8_t *)buf);

  if (value == 1) {
    struct foot_sensor_start_activity_event *foot_evt =
        new_foot_sensor_start_activity_event();
    APP_EVENT_SUBMIT(foot_evt);

    struct motion_sensor_start_activity_event *motion_evt =
        new_motion_sensor_start_activity_event();
    APP_EVENT_SUBMIT(motion_evt);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Forward the command to secondary device
    FORWARD_D2D_COMMAND(ble_d2d_tx_send_start_activity_command,
                        D2D_TX_CMD_START_ACTIVITY, value,
                        "start activity command");
#endif

    // Start Logging Activity file
    generic_message_t start_logging_msg = {};

    start_logging_msg.sender = SENDER_BTH;
    start_logging_msg.type = MSG_TYPE_COMMAND;
    strncpy(start_logging_msg.data.command_str, "START_LOGGING_ACTIVITY",
            sizeof(start_logging_msg.data.command_str) - 1);
    start_logging_msg.data.command_str[sizeof(start_logging_msg.data.command_str) - 1] = '\0';

    // Start logging Activity file
    if (k_msgq_put(&data_msgq, &start_logging_msg, K_NO_WAIT) != 0) {
      LOG_WRN("Failed to send START_LOGGING_ACTIVITY command to DATA module");
    } else {
      LOG_INF("Sent START_LOGGING_ACTIVITY command to DATA module");
    }

    LOG_INF("Submitted start activity events for foot sensor and motion sensor "
            "(input=1).");
  } else {
    LOG_WRN("Start activity characteristic write ignored (input=%u).", value);
  }

  return len;
}

// --- GPS Update Handler ---
static ssize_t write_gps_update_vnd(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len,
                                    uint16_t offset, uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  // Check if we received the correct size for GPSUpdateCommand
  if (len != sizeof(GPSUpdateCommand)) {
    LOG_ERR("GPS update characteristic write: invalid length %u (expected %u)",
            len, sizeof(GPSUpdateCommand));
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  GPSUpdateCommand gps_data;
  memcpy(&gps_data, buf, sizeof(GPSUpdateCommand));

  LOG_INF(
      "GPS update received: lat=%d, lon=%d, speed=%u cm/s, dist=%u m, acc=%u m",
      gps_data.latitude_e7, gps_data.longitude_e7, gps_data.speed_cms,
      gps_data.distance_m, gps_data.accuracy_m);

  // Send GPS update to activity metrics module
  generic_message_t gps_msg = {};
  gps_msg.sender = SENDER_BTH;
  gps_msg.type = MSG_TYPE_GPS_UPDATE;
  memcpy(&gps_msg.data.gps_update, &gps_data, sizeof(GPSUpdateCommand));

  if (k_msgq_put(&activity_metrics_msgq, &gps_msg, K_NO_WAIT) != 0) {
    LOG_ERR("Failed to send GPS update to activity metrics");
    return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
  }

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
  // Forward the GPS update to secondary device
  int err = ble_d2d_tx_send_gps_update(&gps_data);
  if (err) {
    LOG_ERR("Failed to forward GPS update to secondary: %d", err);
  } else {
    LOG_DBG("Successfully forwarded GPS update to secondary");
  }
#endif

  return len;
}

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
// --- User Info Handler ---
static ssize_t write_user_info_vnd(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   const void *buf, uint16_t len,
                                   uint16_t offset, uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  // Check if we received the correct size for UserInfoData
  if (len != sizeof(UserInfoData)) {
    LOG_ERR("User info characteristic write: invalid length %u (expected %u)",
            len, sizeof(UserInfoData));
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  UserInfoData user_info;
  memcpy(&user_info, buf, sizeof(UserInfoData));

  // Convert weight from decigrams to kilograms for logging
  float weight_kg = user_info.weight_dg / 100.0f;
  
  // Convert height from millimeters to centimeters for logging
  float height_cm = user_info.height_mm / 10.0f;
  
  // Log the received user information
  LOG_INF("User info received:");
  LOG_INF("  Age: %u years", user_info.age_years);
  LOG_INF("  Gender: %u", user_info.gender);
  LOG_INF("  Height: %u mm (%.1f cm)", user_info.height_mm, (double)height_cm);
  LOG_INF("  Weight: %u dg (%.1f kg)", user_info.weight_dg, (double)weight_kg);

  // TODO: Forward user info to secondary device if needed
  // Currently just logging as requested - implementation of data usage to follow
  
  // TODO: Store user info in persistent storage or send to relevant modules
  // This will be implemented based on future requirements
  
  return len;
}
#endif // CONFIG_PRIMARY_DEVICE


#if IS_ENABLED(CONFIG_PRIMARY_DEVICE) 
// --- Reset Bonds Handler ---
static ssize_t write_reset_bonds_vnd(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset, uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  if (len != sizeof(uint8_t)) {
    LOG_ERR("Reset bonds characteristic write: invalid length %u (expected %u)",
            len, sizeof(uint8_t));
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  uint8_t value = *((const uint8_t *)buf);

  if (value == 1) {
    LOG_INF("Reset bonds command received, resetting all BLE bonds");
    
    // Call the ble_reset_bonds function
    err_t err = ble_reset_bonds();
    if (err != err_t::NO_ERROR) {
      LOG_ERR("Failed to reset BLE bonds: %d", (int)err);
      return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    
    LOG_INF("Successfully reset all BLE bonds");


  } else {
    LOG_WRN("Reset bonds characteristic write ignored (value=%u, expected 1)",
            value);
  }

  return len;
}

#endif // CONFIG_PRIMARY_DEVICE

// --- Weight Calibration Trigger Handler ---
static bool weight_calibration_trigger_subscribed = false;

static ssize_t write_weight_calibration_trigger_vnd(
    struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
    uint16_t len, uint16_t offset, uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  // Check if this is a simple trigger (1 byte) or calibration data (4 bytes for
  // float)
  if (len == 1) {
    // Legacy support: single byte trigger for measurement without known weight
    uint8_t value = *((const uint8_t *)buf);
    LOG_INF("Weight calibration trigger characteristic write (legacy): %u",
            value);

    if (value == 1) {
      // Send weight calibration trigger without known weight
      generic_message_t calib_msg = {};
      calib_msg.sender = SENDER_BTH;
      calib_msg.type = MSG_TYPE_START_WEIGHT_CALIBRATION;

      if (k_msgq_put(&activity_metrics_msgq, &calib_msg, K_NO_WAIT) != 0) {
        LOG_ERR(
            "Failed to send weight calibration trigger to activity metrics");
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
      }

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
      // Forward the command to secondary device
      FORWARD_D2D_COMMAND(ble_d2d_tx_send_weight_calibration_trigger_command,
                          D2D_TX_CMD_WEIGHT_CALIBRATION_TRIGGER, value,
                          "weight calibration trigger command");
#endif
    }
  } else if (len == sizeof(weight_calibration_step_t)) {
    // New format: calibration with known weight
    weight_calibration_step_t *calib_data = (weight_calibration_step_t *)buf;
    LOG_INF("Weight calibration trigger with known weight: %.1f kg",
            (double)calib_data->known_weight_kg);

    // Validate weight range
    if (calib_data->known_weight_kg < 20.0f ||
        calib_data->known_weight_kg > 300.0f) {
      LOG_ERR("Invalid weight for calibration: %.1f kg",
              (double)calib_data->known_weight_kg);
      return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }

    // Send calibration data to activity metrics
    generic_message_t calib_msg = {};
    calib_msg.sender = SENDER_BTH;
    calib_msg.type = MSG_TYPE_START_WEIGHT_CALIBRATION;
    memcpy(&calib_msg.data.weight_calibration_step, calib_data,
           sizeof(weight_calibration_step_t));

    if (k_msgq_put(&activity_metrics_msgq, &calib_msg, K_NO_WAIT) != 0) {
      LOG_ERR("Failed to send weight calibration data to activity metrics");
      return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Forward calibration data to secondary device
    int err = ble_d2d_tx_send_weight_calibration_with_weight(calib_data);
    if (err) {
      LOG_ERR(
          "Failed to forward weight calibration with weight to secondary: %d",
          err);
    } else {
      LOG_INF(
          "Successfully forwarded weight calibration with weight to secondary");
    }
#endif
  } else {
    LOG_ERR(
        "Weight calibration trigger characteristic write: invalid length %u",
        len);
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  return len;
}

static void
cs_weight_calibration_trigger_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                              uint16_t value) {
  if (!attr) {
    LOG_ERR("cs_weight_calibration_trigger_ccc_cfg_changed: attr is NULL");
    return;
  }
  weight_calibration_trigger_subscribed = value == BT_GATT_CCC_NOTIFY;
  LOG_DBG("Weight Calibration Trigger CCC changed: %u", value);
}

// --- Connection Parameter Control Handlers ---

/**
 * @brief Read handler for connection parameter control characteristic
 * Returns the current connection profile (0=FOREGROUND, 1=BACKGROUND,
 * 2=BACKGROUND_IDLE)
 */
static ssize_t read_conn_param_control_vnd(struct bt_conn *conn,
                                           const struct bt_gatt_attr *attr,
                                           void *buf, uint16_t len,
                                           uint16_t offset) {
  uint8_t current_profile = (uint8_t)ble_conn_params_get_profile();
  return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_profile,
                           sizeof(current_profile));
}

/**
 * @brief Write handler for connection parameter control characteristic
 * Allows mobile app to request different connection profiles for background
 * execution
 *
 * @param buf Should contain 1 byte: 0=FOREGROUND, 1=BACKGROUND,
 * 2=BACKGROUND_IDLE
 */
static ssize_t write_conn_param_control_vnd(struct bt_conn *conn,
                                            const struct bt_gatt_attr *attr,
                                            const void *buf, uint16_t len,
                                            uint16_t offset, uint8_t flags) {
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  if (len != sizeof(uint8_t)) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  uint8_t requested_profile = *((const uint8_t *)buf);

  // Validate profile value
  if (requested_profile >= CONN_PROFILE_MAX) {
    LOG_ERR("Invalid connection profile requested: %d", requested_profile);
    return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
  }

  LOG_INF("Mobile app requested connection profile change to: %d",
          requested_profile);

  // Update connection parameters for primary device
  int err = ble_conn_params_update(conn, (conn_profile_t)requested_profile);
  if (err) {
    LOG_ERR("Failed to update connection parameters: %d", err);
    return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
  }

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
  // Forward the command to secondary device
  FORWARD_D2D_COMMAND(ble_d2d_tx_send_conn_param_control_command,
                      D2D_TX_CMD_CONN_PARAM_CONTROL, requested_profile,
                      "connection parameter control command");
#endif

  LOG_INF("Connection parameter update initiated successfully");
  return len;
}

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
/**
 * @brief Write callback for Delete Secondary Foot Log Command Characteristic.
 * Only forwards the command to secondary device via D2D.
 */



/**
 * @brief Write callback for Delete Secondary Activity Log Command
 * Characteristic. Only forwards the command to secondary device via D2D.
 */
static ssize_t write_delete_secondary_activity_log_command_vnd(
    struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
    uint16_t len, uint16_t offset, uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  if (len != sizeof(uint8_t)) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  uint8_t id_to_delete;
  memcpy(&id_to_delete, buf, sizeof(uint8_t));

  // Only forward to secondary device
  FORWARD_D2D_COMMAND(ble_d2d_tx_send_delete_activity_log_command,
                      D2D_TX_CMD_DELETE_ACTIVITY_LOG, id_to_delete,
                      "delete activity log command");
  return len;
}

static void cs_delete_secondary_activity_log_ccc_cfg_changed(
    const struct bt_gatt_attr *attr, uint16_t value) {
  if (!attr) {
    LOG_ERR("cs_delete_secondary_activity_log_ccc_cfg_changed: attr is NULL");
    return;
  }
  LOG_DBG("Delete Secondary Activity Log CCC changed: %u", value);
}
#endif // CONFIG_PRIMARY_DEVICE

/**
 * @brief Write callback for Delete Activity Log Command Characteristic.
 * Receives the ID of the activity log file to delete and sends a message to the
 * data module.
 */
static ssize_t write_delete_activity_log_command_vnd(
    struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
    uint16_t len, uint16_t offset, uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  if (len != sizeof(uint8_t)) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  uint8_t id_to_delete;
  memcpy(&id_to_delete, buf, sizeof(uint8_t));

  generic_message_t delete_msg;
  delete_msg.sender = SENDER_BTH;
  delete_msg.type = MSG_TYPE_DELETE_ACTIVITY_LOG;
  delete_msg.data.delete_cmd.type = RECORD_HARDWARE_ACTIVITY;
  delete_msg.data.delete_cmd.id = id_to_delete;

  if (k_msgq_put(&data_msgq, &delete_msg, K_NO_WAIT) != 0) {
    LOG_ERR("Failed to send delete activity log message for ID %u.",
            id_to_delete);
    return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
  }

  LOG_INF("Delete activity log message for ID %u sent to data_msgq.",
          id_to_delete);

  // Note: This only deletes from primary device storage
  // Use delete_secondary_activity_log_command to delete from secondary

  return len;
}

/**
 * @brief Callback for CCC configuration change of delete activity log command
 * characteristic.
 */
static void
cs_delete_activity_log_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                       uint16_t value) {
  if (!attr) {
    LOG_ERR("cs_delete_activity_log_ccc_cfg_changed: attr is NULL");
    return;
  }
  // Could use a separate flag, but reusing status_subscribed for simplicity
  LOG_DBG("Delete Activity Log CCC changed: %u", value);
}

// --- Trigger BHI360 Calibration Characteristic Handlers ---
static bool trigger_bhi360_calibration_subscribed = false;

static ssize_t write_trigger_bhi360_calibration_vnd(
    struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
    uint16_t len, uint16_t offset, uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  if (len != sizeof(uint8_t)) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  uint8_t value = *((const uint8_t *)buf);

  if (value == 1) {
    // Send message to motion sensor to trigger calibration
    generic_message_t calib_msg = {};
    calib_msg.sender = SENDER_BTH;
    calib_msg.type = MSG_TYPE_TRIGGER_BHI360_CALIBRATION;

    if (k_msgq_put(&motion_sensor_msgq, &calib_msg, K_NO_WAIT) != 0) {
      LOG_ERR("Failed to send calibration trigger to motion sensor");
      return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Forward the command to secondary device
    FORWARD_D2D_COMMAND(ble_d2d_tx_send_trigger_bhi360_calibration_command,
                        D2D_TX_CMD_TRIGGER_CALIBRATION, value,
                        "trigger calibration command");
#endif

    LOG_INF("Triggered BHI360 calibration (input=1).");
  } else {
    LOG_WRN(
        "Trigger BHI360 calibration characteristic write ignored (input=%u).",
        value);
  }

  return len;
}

static void
cs_trigger_bhi360_calibration_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                              uint16_t value) {
  if (!attr) {
    LOG_ERR("cs_trigger_bhi360_calibration_ccc_cfg_changed: attr is NULL");
    return;
  }
  trigger_bhi360_calibration_subscribed = value == BT_GATT_CCC_NOTIFY;
  LOG_DBG("Trigger BHI360 Calibration CCC changed: %u", value);
}

static void cs_start_activity_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                              uint16_t value) {
  if (!attr) {
    LOG_ERR("cs_start_activity_ccc_cfg_changed: attr is NULL");
    return;
  }
  start_activity_status_subscribed = value == BT_GATT_CCC_NOTIFY;
  LOG_DBG("Start Activity CCC changed: %u", value);
}

// --- Stop Activity Characteristic Handlers ---
static ssize_t write_stop_activity_command_vnd(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               const void *buf, uint16_t len,
                                               uint16_t offset, uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(attr);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  if (len != sizeof(uint8_t)) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  uint8_t value = *((const uint8_t *)buf);

  if (value == 1) {
    struct foot_sensor_stop_activity_event *foot_evt =
        new_foot_sensor_stop_activity_event();
    APP_EVENT_SUBMIT(foot_evt);

    struct motion_sensor_stop_activity_event *motion_evt =
        new_motion_sensor_stop_activity_event();
    APP_EVENT_SUBMIT(motion_evt);

    LOG_INF("Sending STOP_ACTIVITY message");

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Forward the command to secondary device
    FORWARD_D2D_COMMAND(ble_d2d_tx_send_stop_activity_command,
                        D2D_TX_CMD_STOP_ACTIVITY, value,
                        "stop activity command");
#endif

    // Stop  Logging Activity file
    generic_message_t stop_logging_msg = {};

    stop_logging_msg.sender = SENDER_BTH;
    stop_logging_msg.type = MSG_TYPE_COMMAND;
    strncpy(stop_logging_msg.data.command_str, "STOP_LOGGING_ACTIVITY",
            sizeof(stop_logging_msg.data.command_str));
    stop_logging_msg.data.command_str[sizeof(stop_logging_msg.data.command_str) - 1] = '\0';

    // Stop logging Activity file
    if (k_msgq_put(&data_msgq, &stop_logging_msg, K_NO_WAIT) != 0) {
      LOG_WRN("Failed to send STOP_LOGGING_ACTIVITY command to DATA module");
    } else {
      LOG_INF("Sent STOP_LOGGING_ACTIVITY command to DATA module");
    }

    LOG_INF("Submitted stop activity events for foot sensor and motion sensor "
            "(input=1).");
  } else {
    LOG_WRN("Stop activity characteristic write ignored (input=%u).", value);
  }

  return len;
}

// --- Erase External Flash Handlers ---
// Store the attribute pointer for notifications
static const struct bt_gatt_attr *erase_flash_attr = NULL;

static ssize_t write_erase_external_flash_vnd(struct bt_conn *conn,
                                              const struct bt_gatt_attr *attr,
                                              const void *buf, uint16_t len,
                                              uint16_t offset, uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(offset);
  ARG_UNUSED(flags);

  // Save the attribute pointer for notifications
  erase_flash_attr = attr;

  if (len != sizeof(uint8_t)) {
    LOG_ERR("Erase external flash characteristic write: invalid length %u (expected %u)",
            len, sizeof(uint8_t));
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
  }

  uint8_t value = *((const uint8_t *)buf);

  if (value == 1) {
    // Only proceed if not already erasing
    if (external_flash_erase_status == 0) {
      LOG_INF("Erase external flash command received, sending to data module");
      
      // Update status to erasing
      external_flash_erase_status = 1;
      
      // Send message to data module to trigger erase
      generic_message_t erase_msg = {};
      erase_msg.sender = SENDER_BTH;
      erase_msg.type = MSG_TYPE_ERASE_EXTERNAL_FLASH;
      
      if (k_msgq_put(&data_msgq, &erase_msg, K_NO_WAIT) != 0) {
        LOG_ERR("Failed to send erase external flash command to DATA module");
        external_flash_erase_status = 0; // Reset status on failure
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
      }
      
      LOG_INF("Sent ERASE_EXTERNAL_FLASH command to DATA module");
      
      // Notify that erasing has started
      if (erase_flash_attr) {
        bt_gatt_notify(NULL, erase_flash_attr,
            &external_flash_erase_status, sizeof(external_flash_erase_status));
      }
      
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
      // Forward the command to secondary device
      FORWARD_D2D_COMMAND(ble_d2d_tx_send_erase_flash_command,
                          D2D_TX_CMD_ERASE_FLASH, value,
                          "erase flash");
#endif
      
    } else {
      LOG_WRN("Erase external flash already in progress (status=%u)",
              external_flash_erase_status);
    }
  } else {
    LOG_WRN("Erase external flash characteristic write ignored (value=%u, expected 1)",
            value);
  }

  return len;
}

static ssize_t read_erase_external_flash_vnd(struct bt_conn *conn,
                                             const struct bt_gatt_attr *attr,
                                             void *buf, uint16_t len,
                                             uint16_t offset) {
  return bt_gatt_attr_read(conn, attr, buf, len, offset,
                           &external_flash_erase_status,
                           sizeof(external_flash_erase_status));
}

// Function to be called by data module when erase is complete
void cs_external_flash_erase_complete_notify(void) {
  external_flash_erase_status = 2; // Set status to complete
  
  // Notify using the saved attribute pointer
  if (erase_flash_attr) {
    bt_gatt_notify(NULL, erase_flash_attr,
        &external_flash_erase_status, sizeof(external_flash_erase_status));
    LOG_INF("External flash erase complete notification sent (status=2)");
  } else {
    LOG_WRN("Cannot send erase complete notification - attribute not initialized");
  }
  
  // Note: Status remains at 2 until next erase operation
  // Client can read the characteristic to check status at any time
}


static void cs_stop_activity_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                             uint16_t value) {
  if (!attr) {
    LOG_ERR("cs_stop_activity_ccc_cfg_changed: attr is NULL");
    return;
  }
  stop_activity_status_subscribed = value == BT_GATT_CCC_NOTIFY;
  LOG_DBG("Stop Activity CCC changed: %u", value);
}

static ssize_t write_set_time_control_vnd(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          const void *buf, uint16_t len,
                                          uint16_t offset, uint8_t flags) {
  ARG_UNUSED(conn);
  ARG_UNUSED(flags);
  uint32_t *value = static_cast<uint32_t *>(attr->user_data);
  uint32_t temp_value = 0;

  if ((offset + len) > VND_MAX_LEN) {
    return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
  }

  // This line copies the contents of the buffer buf of length len to the memory
  // pointed to by value starting at the offset offset.
  std::memcpy(value + offset, buf, len);

  if (is_little_endian) {
    temp_value = swap_to_little_endian(*value);
  } else {
    // the system is big endian, no need to convert
    temp_value = *value;
  }

  // Save epoch time to RTC
  set_current_time_from_epoch((temp_value));

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
  // Forward the command to secondary device
  int err = ble_d2d_tx_send_set_time_command(temp_value);
  if (err == -EINVAL && !ble_d2d_tx_is_ready()) {
    // D2D TX not ready yet, queue the command
    LOG_INF("D2D TX not ready, queuing set time command");
    err = ble_d2d_tx_queue_command(D2D_TX_CMD_SET_TIME, &temp_value);
    if (err) {
      LOG_ERR("Failed to queue set time command: %d", err);
    }
  } else if (err) {
    LOG_ERR("Failed to forward set time command to secondary: %d", err);
  } else {
    LOG_INF("Successfully forwarded set time command to secondary");
  }
#endif

  return len;
}

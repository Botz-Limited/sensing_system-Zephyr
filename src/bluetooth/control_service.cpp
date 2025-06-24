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

#include "ble_services.hpp"
#include <app.hpp>
#include <app_version.h>

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "ble_d2d_tx.hpp"
#endif

LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Constants
#define VND_MAX_LEN 128  // Maximum length for vendor characteristic data

// External function declarations
extern "C" void set_current_time_from_epoch(uint32_t new_epoch_time_s);

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static constexpr uint8_t is_little_endian = 1;
#else
static constexpr uint8_t is_little_endian = 0;
#endif

static uint32_t set_time_control = 0;
static bool status_subscribed = false;

static uint32_t swap_to_little_endian(uint32_t value);

static ssize_t write_set_time_control_vnd(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          const void *buf, uint16_t len,
                                          uint16_t offset, uint8_t flags);

                                          // For foot log deletion characteristic
ssize_t read_delete_foot_log_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                         uint16_t len, uint16_t offset);
ssize_t write_delete_foot_log_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                          uint16_t len, uint16_t offset, uint8_t flags);
static void cs_delete_foot_log_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

// For BHI360 log deletion characteristic
ssize_t read_delete_bhi360_log_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                          uint16_t len, uint16_t offset);
ssize_t write_delete_bhi360_log_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                           uint16_t len, uint16_t offset, uint8_t flags);
static void cs_delete_bhi360_log_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);





/**
 * @file
 * @brief Bluetooth GATT Service and Characteristic UUIDs
 */

/**
 * @brief UUID for the control service. This UUID identifies the control service
 * used for handling various commands over Bluetooth communication.
 */
static struct bt_uuid_128 control_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x4fd5b67f, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for the delete foot log command characteristic. ---
static struct bt_uuid_128 delete_foot_log_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x4fd5b682, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae)); // Incrementing from old meta uuid

// --- New: UUID for the delete BHI360 log command characteristic. ---
static struct bt_uuid_128 delete_bhi360_log_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x4fd5b683, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae)); // Incrementing further

// --- New: UUID for the start activity characteristic. ---
static struct bt_uuid_128 start_activity_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x4fd5b684, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for the stop activity characteristic. ---
static struct bt_uuid_128 stop_activity_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x4fd5b685, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// --- New: UUID for the trigger BHI360 calibration characteristic. ---
static struct bt_uuid_128 trigger_bhi360_calibration_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x4fd5b686, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

// Forward declarations for start/stop activity handlers
static ssize_t write_start_activity_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static void cs_start_activity_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t write_stop_activity_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static void cs_stop_activity_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t write_trigger_bhi360_calibration_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static void cs_trigger_bhi360_calibration_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

/**
 * @brief UUID for the set time command characteristic.
 * This UUID identifies the characteristic for setting the device's time.
 */
static struct bt_uuid_128 set_time_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x4fd5b681, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));
    

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
    BT_GATT_CHARACTERISTIC(&set_time_command_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE_ENCRYPT, nullptr,
                           write_set_time_control_vnd, static_cast<void *>(&set_time_control)),

BT_GATT_CHARACTERISTIC(&delete_foot_log_command_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, // Added NOTIFY for status feedback
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_delete_foot_log_command_vnd, // Read callback for current deletion ID
                           write_delete_foot_log_command_vnd, static_cast<void *>(nullptr)), // No specific data pointer needed if id is read from buffer
    BT_GATT_CCC(cs_delete_foot_log_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // New: Delete BHI360 Log Characteristic
    BT_GATT_CHARACTERISTIC(&delete_bhi360_log_command_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, // Added NOTIFY for status feedback
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_delete_bhi360_log_command_vnd, // Read callback for current deletion ID
                           write_delete_bhi360_log_command_vnd, static_cast<void *>(nullptr)), // No specific data pointer needed if id is read from buffer
    BT_GATT_CCC(cs_delete_bhi360_log_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // New: Start Activity Characteristic
    BT_GATT_CHARACTERISTIC(&start_activity_command_uuid.uuid, BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE_ENCRYPT,
                           nullptr, write_start_activity_command_vnd, nullptr),
    BT_GATT_CCC(cs_start_activity_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // New: Stop Activity Characteristic
    BT_GATT_CHARACTERISTIC(&stop_activity_command_uuid.uuid, BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE_ENCRYPT,
                           nullptr, write_stop_activity_command_vnd, nullptr),
    BT_GATT_CCC(cs_stop_activity_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // New: Trigger BHI360 Calibration Characteristic
    BT_GATT_CHARACTERISTIC(&trigger_bhi360_calibration_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_WRITE_ENCRYPT,
                           nullptr, write_trigger_bhi360_calibration_vnd, nullptr),
    BT_GATT_CCC(cs_trigger_bhi360_calibration_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT)
);

/**
 * @brief Notifies subscribed devices about the metadata log deletion command.
 * This function notifies subscribed devices about the metadata log deletion
 * command.
 *
 * @param stu The metadata log deletion command value.
 */
ssize_t read_delete_foot_log_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                         uint16_t len, uint16_t offset)
{
    // This could return the ID of the last requested deletion or a status.
    // For now, let's return a dummy value or a flag indicating readiness.
    // If you want to return the last ID, you'd need a static variable to store it.
    uint8_t current_id = 0; // Replace with actual state if tracking
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_id, sizeof(current_id));
}

/**
 * @brief Write callback for Delete Foot Log Command Characteristic.
 * Receives the ID of the foot sensor log file to delete and sends a message to the data module.
 */
ssize_t write_delete_foot_log_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                          uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != sizeof(uint8_t) || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t id_to_delete;
    memcpy(&id_to_delete, buf, sizeof(uint8_t));

    generic_message_t delete_msg;
    delete_msg.sender = SENDER_BTH;
    delete_msg.type = MSG_TYPE_DELETE_FOOT_LOG; // New specific message type
    delete_msg.data.delete_cmd.type = RECORD_HARDWARE_FOOT_SENSOR; // Specify record type
    delete_msg.data.delete_cmd.id = id_to_delete; // The ID of the log file

    LOG_WRN("Sent delete foot log message for ID %u to data_msgq.", id_to_delete);

    if (k_msgq_put(&data_msgq, &delete_msg, K_NO_WAIT) != 0) {
        LOG_ERR("Failed to send delete foot log message for ID %u.", id_to_delete);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY); // Or a more specific error
    }

    #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Forward the command to secondary device
    ble_d2d_tx_send_delete_foot_log_command(id_to_delete);
    #endif

    // Optionally notify the client of success/failure or a status update via CCC
    // This would typically be done after data module has processed the deletion
    // For direct notification, you'd need a way to store the result and notify later.
    // For simplicity, for now, we'll assume the client is checking the read value or
    // we'll add a separate notification mechanism if required for actual deletion status.
    // cs_delete_foot_log_command_notify(id_to_delete, true); // Example, assuming such a notify exists

    return len; // Indicate successful write
}

/**
 * @brief Callback for CCC configuration change of delete foot log command characteristic.
 */
static void cs_delete_foot_log_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    status_subscribed = value == BT_GATT_CCC_NOTIFY; // Assuming a common status_subscribed flag
    LOG_DBG("Delete Foot Log CCC changed: %u", value);
}

/**
 * @brief Read callback for Delete BHI360 Log Command Characteristic.
 * Returns the last requested deletion ID for the BHI360 sensor.
 */
ssize_t read_delete_bhi360_log_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                          uint16_t len, uint16_t offset)
{
    uint8_t current_id = 0; // Replace with actual state if tracking
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_id, sizeof(current_id));
}

/**
 * @brief Write callback for Delete BHI360 Log Command Characteristic.
 * Receives the ID of the BHI360 log file to delete and sends a message to the data module.
 */
ssize_t write_delete_bhi360_log_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                           uint16_t len, uint16_t offset, uint8_t flags)
{
    if (len != sizeof(uint8_t) || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t id_to_delete;
    memcpy(&id_to_delete, buf, sizeof(uint8_t));

    generic_message_t delete_msg;
    delete_msg.sender = SENDER_BTH;
    delete_msg.type = MSG_TYPE_DELETE_BHI360_LOG; // New specific message type
    delete_msg.data.delete_cmd.type = RECORD_HARDWARE_BHI360; // Specify record type
    delete_msg.data.delete_cmd.id = id_to_delete; // The ID of the log file

    LOG_WRN("Sent delete foot log message for ID %u to data_msgq.", id_to_delete);

    if (k_msgq_put(&data_msgq, &delete_msg, K_NO_WAIT) != 0) {
        LOG_ERR("Failed to send delete BHI360 log message for ID %u.", id_to_delete);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Forward the command to secondary device
    ble_d2d_tx_send_delete_bhi360_log_command(id_to_delete);
    #endif

    LOG_DBG("Sent delete BHI360 log message for ID %u to data_msgq.", id_to_delete);

    return len; // Indicate successful write
}

/**
 * @brief Callback for CCC configuration change of delete BHI360 log command characteristic.
 */
static void cs_delete_bhi360_log_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    status_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Delete BHI360 Log CCC changed: %u", value);
}

/**
 * @brief Converts a 32-bit value from big endian to little endian.
 *
 * This function converts a 32-bit value from big endian to little endian.
 *
 * @param value The value to be converted.
 * @return The converted value in little endian format.
 */
static uint32_t swap_to_little_endian(uint32_t value)
{
    return ((value << 24) & 0xFF000000) | ((value << 8) & 0x00FF0000) | ((value >> 8) & 0x0000FF00) |
           ((value >> 24) & 0x000000FF);
}

// --- Start Activity Characteristic Handlers ---
static bool start_activity_status_subscribed = false;
static bool stop_activity_status_subscribed = false;

#include <app_event_manager.h>
#include <events/foot_sensor_event.h>
#include <events/motion_sensor_event.h>

static ssize_t write_start_activity_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t value = *((const uint8_t *)buf);

    if (value == 1) {
        struct foot_sensor_start_activity_event *foot_evt = new_foot_sensor_start_activity_event();
        APP_EVENT_SUBMIT(foot_evt);

        struct motion_sensor_start_activity_event *motion_evt = new_motion_sensor_start_activity_event();
        APP_EVENT_SUBMIT(motion_evt);

        #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Forward the command to secondary device
        ble_d2d_tx_send_start_activity_command(value);
        #endif

        LOG_INF("Submitted start activity events for foot sensor and motion sensor (input=1).");
    } else {
        LOG_WRN("Start activity characteristic write ignored (input=%u).", value);
    }

    return len;
}

// --- Trigger BHI360 Calibration Characteristic Handlers ---
static bool trigger_bhi360_calibration_subscribed = false;

static ssize_t write_trigger_bhi360_calibration_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
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
        ble_d2d_tx_send_trigger_bhi360_calibration_command(value);
        #endif

        LOG_INF("Triggered BHI360 calibration (input=1).");
    } else {
        LOG_WRN("Trigger BHI360 calibration characteristic write ignored (input=%u).", value);
    }

    return len;
}

static void cs_trigger_bhi360_calibration_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    trigger_bhi360_calibration_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Trigger BHI360 Calibration CCC changed: %u", value);
}

static void cs_start_activity_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    start_activity_status_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Start Activity CCC changed: %u", value);
}

// --- Stop Activity Characteristic Handlers ---
static ssize_t write_stop_activity_command_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t value = *((const uint8_t *)buf);

    if (value == 1) {
        struct foot_sensor_stop_activity_event *foot_evt = new_foot_sensor_stop_activity_event();
        APP_EVENT_SUBMIT(foot_evt);

        struct motion_sensor_stop_activity_event *motion_evt = new_motion_sensor_stop_activity_event();
        APP_EVENT_SUBMIT(motion_evt);

        #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Forward the command to secondary device
        ble_d2d_tx_send_stop_activity_command(value);
        #endif

        LOG_INF("Submitted stop activity events for foot sensor and motion sensor (input=1).");
    } else {
        LOG_WRN("Stop activity characteristic write ignored (input=%u).", value);
    }

    return len;
}

static void cs_stop_activity_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
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

  if (is_little_endian) 
  { 
    temp_value = swap_to_little_endian(*value);
  } 
  else 
  { 
    // the system is big endian, no need to convert
    temp_value = *value;
  }

  // Save epoch time to RTC
  set_current_time_from_epoch((temp_value));

  #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
  // Forward the command to secondary device
  ble_d2d_tx_send_set_time_command(temp_value);
  #endif

  return len;
}

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
#include <app_version.h>

LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
static constexpr uint8_t is_little_endian = 1;
#else
static constexpr uint8_t is_little_endian = 0;
#endif

static uint32_t set_time_control = 0;
static uint32_t delete_log_hw_control = 0;
static uint32_t delete_log_meta_control = 0;
static bool status_subscribed = false;

static uint32_t swap_to_little_endian(uint32_t value);

static void cs_delete_log_hw_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void cs_delete_log_meta_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t write_set_time_control_vnd(struct bt_conn *conn,
                                          const struct bt_gatt_attr *attr,
                                          const void *buf, uint16_t len,
                                          uint16_t offset, uint8_t flags);

void cs_delete_log_meta_control_command_notify(uint32_t stu);

/**
 * @brief Read debug command for control.
 *
 * This function is a callback for reading deleting meta log command
 * over a Bluetooth connection.
 *
 * @param conn Pointer to the Bluetooth connection structure.
 * @param attr Pointer to the GATT attribute structure.
 * @param buf Pointer to the buffer where the read data will be stored.
 * @param len Length of the buffer.
 * @param offset Offset within the attribute value.
 * @return ssize_t Returns the length of the data read.
 */
static ssize_t read_delete_meta_log_control_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                                uint16_t len, uint16_t offset)
{
    // This line declares a pointer variable named value and assigns it the
    // address of the user data stored in the attr structure. The (uint32_t *)
    // cast is used to explicitly convert the value of attr->user_data to a
    // pointer to a uint32_t type.
    auto *value = static_cast<uint32_t *>(attr->user_data);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(delete_log_meta_control));
}

/**
 * @brief Write delete meta data log command for control.
 *
 * This function is a callback for writing delete meta data control
 * over a Bluetooth connection.
 *
 * @param conn Pointer to the Bluetooth connection structure.
 * @param attr Pointer to the GATT attribute structure.
 * @param buf Pointer to the data buffer containing the command.
 * @param len Length of the data buffer.
 * @param offset Offset within the attribute value.
 * @param flags Flags indicating GATT write operation.
 * @return ssize_t Returns the length of the data written.
 */

// We cast the void * pointer attr->user_data to uint32_t because the
// characteristic send to the FW an unsigned big endian 32 bit bit number
// representing the file id to be deleted.
static ssize_t write_delete_meta_log_control_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                                 uint16_t len, uint16_t offset, uint8_t flags)
{

    ARG_UNUSED(conn);
    ARG_UNUSED(flags);
    uint32_t *value = static_cast<uint32_t *>(attr->user_data);

    if ((offset + len) > VND_MAX_LEN)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    // This line copies the contents of the buffer buf of length len to the memory
    // pointed to by value starting at the offset offset.
    memcpy(value + offset, buf, len);

    cs_delete_log_meta_control_command_notify(*value);

    return len;
}

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

/**
 * @brief UUID for the delete log metadata command characteristic.
 * This UUID identifies the characteristic for deleting metadata logs.
 */
static struct bt_uuid_128 delete_log_meta_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x4fd5b680, 0x9d89, 0x4061, 0x92aa, 0x319ca786baae));

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



    // deleting the meta data logs
    BT_GATT_CHARACTERISTIC(&delete_log_meta_command_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, read_delete_meta_log_control_vnd,
                           write_delete_meta_log_control_vnd, static_cast<void *>(&delete_log_meta_control)),
    BT_GATT_CCC(cs_delete_log_meta_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT)
);

/**
 * @brief Notifies subscribed devices about the metadata log deletion command.
 * This function notifies subscribed devices about the metadata log deletion
 * command.
 *
 * @param stu The metadata log deletion command value.
 */
void cs_delete_log_meta_control_command_notify(uint32_t stu)
{

    delete_log_meta_control = stu;

    // Cast a delete file event for the data module
    // auto *event = new_record_deletion_event();
    // event->record_type = RECORD_METADATA;
    // event->id = stu;
    // APP_EVENT_SUBMIT(event);

    if (status_subscribed)
    {
        auto *status_gatt =
            bt_gatt_find_by_uuid(control_service.attrs, control_service.attr_count, &delete_log_meta_command_uuid.uuid);
        bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&delete_log_meta_control),
                       sizeof(delete_log_meta_control));
    }
}

/**
 * @brief Callback for CCC configuration change of hardware log control
 * characteristic. This function is called when the CCC descriptor associated
 * with the hardware log control characteristic is changed.
 *
 * @param attr Pointer to the GATT attribute structure.
 * @param value New value of the CCC descriptor.
 */
static void cs_delete_log_hw_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    status_subscribed = value == BT_GATT_CCC_NOTIFY;
}

/**
 * @brief Callback for CCC configuration change of metadata log control
 * characteristic.
 *
 * This function is called when the CCC descriptor associated with
 *  the metadata log control characteristic is changed.
 *
 * @param attr Pointer to the GATT attribute structure.
 * @param value New value of the CCC descriptor.
 */
static void cs_delete_log_meta_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    status_subscribed = value == BT_GATT_CCC_NOTIFY;
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

  return len;
}

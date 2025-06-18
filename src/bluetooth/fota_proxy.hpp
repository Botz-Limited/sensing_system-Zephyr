/**
 * @file fota_proxy.hpp
 * @brief FOTA Proxy Service for relaying firmware updates to secondary devices
 * 
 * This service runs on the primary device and acts as a proxy between the phone
 * and secondary devices for FOTA operations.
 */

#ifndef FOTA_PROXY_HPP
#define FOTA_PROXY_HPP

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#ifdef __cplusplus
extern "C" {
#endif

// FOTA Proxy Service UUID: 6e400001-b5a3-f393-e0a9-e50e24dcca9e
#define FOTA_PROXY_SERVICE_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6e400001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))

// Characteristics UUIDs
#define FOTA_PROXY_TARGET_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6e400002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))
#define FOTA_PROXY_COMMAND_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6e400003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))
#define FOTA_PROXY_DATA_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6e400004, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))
#define FOTA_PROXY_STATUS_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x6e400005, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))

// FOTA Proxy Commands
enum fota_proxy_cmd {
    FOTA_PROXY_CMD_START = 0x01,
    FOTA_PROXY_CMD_DATA = 0x02,
    FOTA_PROXY_CMD_END = 0x03,
    FOTA_PROXY_CMD_ABORT = 0x04,
    FOTA_PROXY_CMD_STATUS = 0x05,
    FOTA_PROXY_CMD_RESET = 0x06,
};

// FOTA Proxy Status
enum fota_proxy_status {
    FOTA_PROXY_STATUS_IDLE = 0x00,
    FOTA_PROXY_STATUS_IN_PROGRESS = 0x01,
    FOTA_PROXY_STATUS_SUCCESS = 0x02,
    FOTA_PROXY_STATUS_ERROR = 0x03,
    FOTA_PROXY_STATUS_NO_TARGET = 0x04,
};

// Target device selection
enum fota_proxy_target {
    FOTA_TARGET_PRIMARY = 0x00,
    FOTA_TARGET_SECONDARY = 0x01,
    FOTA_TARGET_ALL = 0xFF,
};

/**
 * @brief Initialize the FOTA proxy service
 * @return 0 on success, negative error code on failure
 */
int fota_proxy_init(void);

/**
 * @brief Set the connection handle for the secondary device
 * @param conn Connection handle to the secondary device
 */
void fota_proxy_set_secondary_conn(struct bt_conn *conn);

/**
 * @brief Get the current FOTA proxy status
 * @return Current status
 */
enum fota_proxy_status fota_proxy_get_status(void);

/**
 * @brief Notify status change to connected phone
 * @param status New status
 * @return 0 on success, negative error code on failure
 */
int fota_proxy_notify_status(enum fota_proxy_status status);

#ifdef __cplusplus
}
#endif

#endif /* FOTA_PROXY_HPP */
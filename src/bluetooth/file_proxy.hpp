/**
 * @file file_proxy.hpp
 * @brief File Access Proxy Service for relaying file operations to secondary devices
 * 
 * This service runs on the primary device and acts as a proxy between the phone
 * and secondary devices for file operations (list, read, delete).
 */

#ifndef FILE_PROXY_HPP
#define FILE_PROXY_HPP

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#ifdef __cplusplus
extern "C" {
#endif

// File Proxy Service UUID: 7e500001-b5a3-f393-e0a9-e50e24dcca9e
#define FILE_PROXY_SERVICE_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x7e500001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))

// Characteristics UUIDs
#define FILE_PROXY_TARGET_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x7e500002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))
#define FILE_PROXY_COMMAND_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x7e500003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))
#define FILE_PROXY_DATA_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x7e500004, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))
#define FILE_PROXY_STATUS_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x7e500005, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))

// File Proxy Commands
enum file_proxy_cmd {
    FILE_PROXY_CMD_LIST_LOGS = 0x01,      // List available log files
    FILE_PROXY_CMD_READ_FILE = 0x02,      // Read file by ID
    FILE_PROXY_CMD_DELETE_FILE = 0x03,    // Delete file by ID
    FILE_PROXY_CMD_GET_FILE_INFO = 0x04,  // Get file info (size, timestamp)
    FILE_PROXY_CMD_ABORT = 0x05,          // Abort current operation
};

// File Proxy Status
enum file_proxy_status {
    FILE_PROXY_STATUS_IDLE = 0x00,
    FILE_PROXY_STATUS_BUSY = 0x01,
    FILE_PROXY_STATUS_SUCCESS = 0x02,
    FILE_PROXY_STATUS_ERROR = 0x03,
    FILE_PROXY_STATUS_NO_TARGET = 0x04,
    FILE_PROXY_STATUS_FILE_NOT_FOUND = 0x05,
    FILE_PROXY_STATUS_TRANSFER_IN_PROGRESS = 0x06,
};

// Target device selection (same as FOTA proxy)
enum file_proxy_target {
    FILE_TARGET_PRIMARY = 0x00,
    FILE_TARGET_SECONDARY = 0x01,
};

// File types
enum file_type {
    FILE_TYPE_FOOT_SENSOR = 0x01,
    FILE_TYPE_BHI360 = 0x02,
    FILE_TYPE_ACTIVITY = 0x03,
    FILE_TYPE_ALL = 0xFF,
};

// File list entry structure
struct file_list_entry {
    uint8_t id;
    uint8_t type;  // FILE_TYPE_FOOT_SENSOR or FILE_TYPE_BHI360
    uint32_t size;
    uint32_t timestamp;
    char name[32];
} __packed;

// File info response
struct file_info_response {
    uint8_t status;
    uint8_t id;
    uint8_t type;
    uint32_t size;
    uint32_t timestamp;
    uint32_t crc32;
} __packed;

/**
 * @brief Initialize the file proxy service
 * @return 0 on success, negative error code on failure
 */
int file_proxy_init(void);

/**
 * @brief Set the connection handle for the secondary device
 * @param conn Connection handle to the secondary device
 */
void file_proxy_set_secondary_conn(struct bt_conn *conn);

/**
 * @brief Get the current file proxy status
 * @return Current status
 */
enum file_proxy_status file_proxy_get_status(void);

/**
 * @brief Notify status change to connected phone
 * @param status New status
 * @return 0 on success, negative error code on failure
 */
int file_proxy_notify_status(enum file_proxy_status status);

/**
 * @brief Notify file data to connected phone
 * @param data File data
 * @param len Data length
 * @return 0 on success, negative error code on failure
 */
int file_proxy_notify_data(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* FILE_PROXY_HPP */
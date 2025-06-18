/**
 * @file ble_d2d_file_transfer.hpp
 * @brief Device-to-Device file transfer protocol for secondary device file access
 */

#pragma once

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

// D2D File Transfer Service UUID: 8e600001-b5a3-f393-e0a9-e50e24dcca9e
#define D2D_FILE_SERVICE_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8e600001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))

// Characteristics UUIDs
#define D2D_FILE_COMMAND_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8e600002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))
#define D2D_FILE_DATA_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8e600003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))
#define D2D_FILE_STATUS_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8e600004, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e))

// D2D File Commands
enum d2d_file_cmd {
    D2D_FILE_CMD_LIST = 0x01,
    D2D_FILE_CMD_READ = 0x02,
    D2D_FILE_CMD_DELETE = 0x03,
    D2D_FILE_CMD_INFO = 0x04,
    D2D_FILE_CMD_ABORT = 0x05,
};

// D2D File Status
enum d2d_file_status {
    D2D_FILE_STATUS_OK = 0x00,
    D2D_FILE_STATUS_ERROR = 0x01,
    D2D_FILE_STATUS_NOT_FOUND = 0x02,
    D2D_FILE_STATUS_BUSY = 0x03,
    D2D_FILE_STATUS_END_OF_DATA = 0x04,
};

// File transfer packet header
struct d2d_file_packet {
    uint8_t cmd;
    uint8_t status;
    uint16_t sequence;
    uint16_t length;
    uint8_t data[];
} __packed;

// Callback for receiving file data from secondary device
typedef void (*d2d_file_data_cb_t)(const uint8_t *data, size_t len);

// Callback for receiving status updates
typedef void (*d2d_file_status_cb_t)(enum d2d_file_status status);

/**
 * @brief Initialize D2D file transfer service (secondary device)
 */
void ble_d2d_file_transfer_init(void);

/**
 * @brief Initialize D2D file transfer client (primary device)
 * @param conn Connection to secondary device
 */
void ble_d2d_file_client_init(struct bt_conn *conn);

/**
 * @brief Set callbacks for file data and status
 * @param data_cb Callback for file data
 * @param status_cb Callback for status updates
 */
void ble_d2d_file_set_callbacks(d2d_file_data_cb_t data_cb, d2d_file_status_cb_t status_cb);

/**
 * @brief Send file command to secondary device (primary only)
 * @param cmd Command type
 * @param data Command data
 * @param len Data length
 * @return 0 on success, negative error code on failure
 */
int ble_d2d_file_send_command(enum d2d_file_cmd cmd, const uint8_t *data, size_t len);

/**
 * @brief Send file data to primary device (secondary only)
 * @param data File data
 * @param len Data length
 * @param sequence Packet sequence number
 * @return 0 on success, negative error code on failure
 */
int ble_d2d_file_send_data(const uint8_t *data, size_t len, uint16_t sequence);

/**
 * @brief Send status to primary device (secondary only)
 * @param status Status code
 * @return 0 on success, negative error code on failure
 */
int ble_d2d_file_send_status(enum d2d_file_status status);

#ifdef __cplusplus
}
#endif
/**
 * @file file_proxy.cpp
 * @brief File Access Proxy Service implementation
 */

#include "file_proxy.hpp"
#include "ble_d2d_file_transfer.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(file_proxy, LOG_LEVEL_INF);

// Buffer sizes
#define FILE_PROXY_BUF_SIZE 512
#define FILE_PROXY_MTU_SIZE 244
#define MAX_FILE_LIST_ENTRIES 20

// State management
static struct {
    struct bt_conn *secondary_conn;
    enum file_proxy_target current_target;
    enum file_proxy_status status;
    bool is_active;
    uint8_t current_operation;
    uint8_t current_file_id;
    uint8_t current_file_type;
    uint32_t total_size;
    uint32_t transferred_size;
    uint8_t data_buffer[FILE_PROXY_BUF_SIZE];
    size_t data_len;
    struct file_list_entry file_list[MAX_FILE_LIST_ENTRIES];
    uint8_t file_count;
} file_proxy_state = {
    .secondary_conn = NULL,
    .current_target = FILE_TARGET_PRIMARY,
    .status = FILE_PROXY_STATUS_IDLE,
    .is_active = false,
    .current_operation = 0,
    .current_file_id = 0,
    .current_file_type = 0,
    .total_size = 0,
    .transferred_size = 0,
    .data_buffer = {0},
    .data_len = 0,
    .file_list = {},
    .file_count = 0,
};

// Work queue for async operations
static struct k_work file_work;
static struct k_work_delayable file_timeout_work;

// Forward declarations
static void file_proxy_work_handler(struct k_work *work);
static void file_proxy_timeout_handler(struct k_work *work);
static int forward_to_secondary(uint8_t cmd, const uint8_t *data, size_t len);
static int handle_primary_file_operation(uint8_t cmd, const uint8_t *data, size_t len);

// D2D callbacks
static void d2d_file_data_callback(const uint8_t *data, size_t len);
static void d2d_file_status_callback(enum d2d_file_status status);

// GATT characteristic values
static uint8_t file_target_value = FILE_TARGET_PRIMARY;
static uint8_t file_status_value = FILE_PROXY_STATUS_IDLE;

// GATT callbacks
static ssize_t file_proxy_target_write(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       const void *buf, uint16_t len,
                                       uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    
    if (offset + len > sizeof(file_target_value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(&file_target_value + offset, buf, len);
    file_proxy_state.current_target = (enum file_proxy_target)file_target_value;
    
    LOG_INF("File proxy target set to: %s", 
            file_target_value == FILE_TARGET_PRIMARY ? "PRIMARY" : "SECONDARY");

    return len;
}

static ssize_t file_proxy_command_write(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        const void *buf, uint16_t len,
                                        uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    
    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    if (len > FILE_PROXY_BUF_SIZE) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    const uint8_t *data = (const uint8_t *)buf;
    uint8_t cmd = data[0];

    LOG_INF("File proxy command received: 0x%02x", cmd);

    // Check if we're already busy
    if (file_proxy_state.is_active && cmd != FILE_PROXY_CMD_ABORT) {
        LOG_WRN("File proxy busy, rejecting command");
        file_proxy_state.status = FILE_PROXY_STATUS_BUSY;
        file_proxy_notify_status(file_proxy_state.status);
        return BT_GATT_ERR(BT_ATT_ERR_PROCEDURE_IN_PROGRESS);
    }

    switch (cmd) {
        case FILE_PROXY_CMD_LIST_LOGS:
            file_proxy_state.is_active = true;
            file_proxy_state.current_operation = cmd;
            file_proxy_state.status = FILE_PROXY_STATUS_BUSY;
            file_proxy_state.file_count = 0;
            
            LOG_INF("Listing log files on %s device", 
                    file_proxy_state.current_target == FILE_TARGET_PRIMARY ? "primary" : "secondary");
            
            // Start timeout timer
            k_work_schedule(&file_timeout_work, K_SECONDS(CONFIG_FILE_PROXY_TIMEOUT_SEC / 6)); // Shorter for list operation
            
            // Submit work to handle the operation
            k_work_submit(&file_work);
            break;

        case FILE_PROXY_CMD_READ_FILE:
            if (len < 3) { // cmd + file_id + file_type
                return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
            }
            
            file_proxy_state.is_active = true;
            file_proxy_state.current_operation = cmd;
            file_proxy_state.current_file_id = data[1];
            file_proxy_state.current_file_type = data[2];
            file_proxy_state.status = FILE_PROXY_STATUS_TRANSFER_IN_PROGRESS;
            file_proxy_state.transferred_size = 0;
            
            LOG_INF("Reading file ID %d, type %d from %s device", 
                    file_proxy_state.current_file_id,
                    file_proxy_state.current_file_type,
                    file_proxy_state.current_target == FILE_TARGET_PRIMARY ? "primary" : "secondary");
            
            // Start timeout timer
            k_work_schedule(&file_timeout_work, K_SECONDS(CONFIG_FILE_PROXY_TIMEOUT_SEC));
            
            // Submit work to handle the operation
            k_work_submit(&file_work);
            break;

        case FILE_PROXY_CMD_DELETE_FILE:
            if (len < 3) { // cmd + file_id + file_type
                return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
            }
            
            file_proxy_state.is_active = true;
            file_proxy_state.current_operation = cmd;
            file_proxy_state.current_file_id = data[1];
            file_proxy_state.current_file_type = data[2];
            file_proxy_state.status = FILE_PROXY_STATUS_BUSY;
            
            LOG_INF("Deleting file ID %d, type %d from %s device", 
                    file_proxy_state.current_file_id,
                    file_proxy_state.current_file_type,
                    file_proxy_state.current_target == FILE_TARGET_PRIMARY ? "primary" : "secondary");
            
            // Start timeout timer
            k_work_schedule(&file_timeout_work, K_SECONDS(10));
            
            // Submit work to handle the operation
            k_work_submit(&file_work);
            break;

        case FILE_PROXY_CMD_GET_FILE_INFO:
            if (len < 3) { // cmd + file_id + file_type
                return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
            }
            
            file_proxy_state.is_active = true;
            file_proxy_state.current_operation = cmd;
            file_proxy_state.current_file_id = data[1];
            file_proxy_state.current_file_type = data[2];
            file_proxy_state.status = FILE_PROXY_STATUS_BUSY;
            
            LOG_INF("Getting info for file ID %d, type %d from %s device", 
                    file_proxy_state.current_file_id,
                    file_proxy_state.current_file_type,
                    file_proxy_state.current_target == FILE_TARGET_PRIMARY ? "primary" : "secondary");
            
            // Start timeout timer
            k_work_schedule(&file_timeout_work, K_SECONDS(10));
            
            // Submit work to handle the operation
            k_work_submit(&file_work);
            break;

        case FILE_PROXY_CMD_ABORT:
            LOG_WRN("File operation aborted");
            file_proxy_state.is_active = false;
            file_proxy_state.status = FILE_PROXY_STATUS_IDLE;
            k_work_cancel_delayable(&file_timeout_work);
            break;

        default:
            LOG_WRN("Unknown file proxy command: 0x%02x", cmd);
            return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    // Update status
    file_proxy_notify_status(file_proxy_state.status);

    return len;
}

static ssize_t file_proxy_status_read(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &file_status_value, sizeof(file_status_value));
}

// GATT Service Definition
BT_GATT_SERVICE_DEFINE(file_proxy_svc,
    BT_GATT_PRIMARY_SERVICE(FILE_PROXY_SERVICE_UUID),
    
    // Target selection characteristic (write)
    BT_GATT_CHARACTERISTIC(FILE_PROXY_TARGET_UUID,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, file_proxy_target_write, &file_target_value),
    
    // Command characteristic (write)
    BT_GATT_CHARACTERISTIC(FILE_PROXY_COMMAND_UUID,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, file_proxy_command_write, NULL),
    
    // Data characteristic (notify only)
    BT_GATT_CHARACTERISTIC(FILE_PROXY_DATA_UUID,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC_MANAGED(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Status characteristic (read/notify)
    BT_GATT_CHARACTERISTIC(FILE_PROXY_STATUS_UUID,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          file_proxy_status_read, NULL, &file_status_value),
    BT_GATT_CCC_MANAGED(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// Forward operation to secondary device
static int forward_to_secondary(uint8_t cmd, const uint8_t *data, size_t len)
{
    if (!file_proxy_state.secondary_conn) {
        LOG_ERR("No secondary device connected");
        file_proxy_state.status = FILE_PROXY_STATUS_NO_TARGET;
        return -ENOTCONN;
    }

    // Build command packet
    uint8_t cmd_packet[16];
    size_t packet_len = 1;
    cmd_packet[0] = cmd;
    
    // Add command-specific data
    switch (cmd) {
        case FILE_PROXY_CMD_LIST_LOGS:
            // No additional data needed
            break;
            
        case FILE_PROXY_CMD_READ_FILE:
        case FILE_PROXY_CMD_DELETE_FILE:
        case FILE_PROXY_CMD_GET_FILE_INFO:
            // Add file ID and type
            if (data && len >= 2) {
                cmd_packet[1] = data[0]; // file_id
                cmd_packet[2] = data[1]; // file_type
                packet_len = 3;
            }
            break;
            
        case FILE_PROXY_CMD_ABORT:
            // No additional data needed
            break;
    }
    
    // Send command via D2D
    int rc = ble_d2d_file_send_command((enum d2d_file_cmd)cmd, cmd_packet + 1, packet_len - 1);
    if (rc != 0) {
        LOG_ERR("Failed to send D2D command: %d", rc);
        file_proxy_state.status = FILE_PROXY_STATUS_ERROR;
        return rc;
    }
    
    return 0;
}

// Handle file operations on primary device
static int handle_primary_file_operation(uint8_t cmd, const uint8_t *data, size_t len)
{
    ARG_UNUSED(data);
    ARG_UNUSED(len);
    
    switch (cmd) {
        case FILE_PROXY_CMD_LIST_LOGS: {
            // For primary device, we would enumerate files from the file system
            // This is a simplified example
            file_proxy_state.file_count = 0;
            
            // Add dummy entries for demonstration
            // In real implementation, scan /lfs directory
            if (file_proxy_state.file_count < MAX_FILE_LIST_ENTRIES) {
                struct file_list_entry *entry = &file_proxy_state.file_list[file_proxy_state.file_count++];
                entry->id = 1;
                entry->type = FILE_TYPE_FOOT_SENSOR;
                entry->size = 1024;
                entry->timestamp = 1234567890;
                strncpy(entry->name, "foot_1.bin", sizeof(entry->name) - 1);
                entry->name[sizeof(entry->name) - 1] = '\0';
            }
            
            if (file_proxy_state.file_count < MAX_FILE_LIST_ENTRIES) {
                struct file_list_entry *entry = &file_proxy_state.file_list[file_proxy_state.file_count++];
                entry->id = 2;
                entry->type = FILE_TYPE_BHI360;
                entry->size = 2048;
                entry->timestamp = 1234567900;
                strncpy(entry->name, "bhi360_2.bin", sizeof(entry->name) - 1);
                entry->name[sizeof(entry->name) - 1] = '\0';
            }
            
            // Send file list as data notification
            uint8_t list_header[2] = {FILE_PROXY_CMD_LIST_LOGS, file_proxy_state.file_count};
            file_proxy_notify_data(list_header, sizeof(list_header));
            
            // Send each file entry
            for (int i = 0; i < file_proxy_state.file_count; i++) {
                file_proxy_notify_data((uint8_t *)&file_proxy_state.file_list[i], 
                                     sizeof(struct file_list_entry));
            }
            
            file_proxy_state.status = FILE_PROXY_STATUS_SUCCESS;
            break;
        }
            
        case FILE_PROXY_CMD_READ_FILE:
            // For primary device, this would use the existing SMP file transfer
            // The phone should use SMP directly for primary files
            LOG_INF("Use SMP for primary device file transfer");
            file_proxy_state.status = FILE_PROXY_STATUS_SUCCESS;
            break;
            
        case FILE_PROXY_CMD_DELETE_FILE:
            // For primary device, use existing delete mechanisms
            LOG_INF("Deleting file ID %d, type %d", 
                    file_proxy_state.current_file_id,
                    file_proxy_state.current_file_type);
            // TODO: Send delete command to data module
            file_proxy_state.status = FILE_PROXY_STATUS_SUCCESS;
            break;
            
        case FILE_PROXY_CMD_GET_FILE_INFO: {
            // Return file information
            struct file_info_response info = {
                .status = FILE_PROXY_STATUS_SUCCESS,
                .id = file_proxy_state.current_file_id,
                .type = file_proxy_state.current_file_type,
                .size = 1024, // Dummy size
                .timestamp = 1234567890,
                .crc32 = 0x12345678,
            };
            
            uint8_t info_header[1] = {FILE_PROXY_CMD_GET_FILE_INFO};
            file_proxy_notify_data(info_header, sizeof(info_header));
            file_proxy_notify_data((uint8_t *)&info, sizeof(info));
            
            file_proxy_state.status = FILE_PROXY_STATUS_SUCCESS;
            break;
        }
    }
    
    return 0;
}

// Work handler for processing file operations
static void file_proxy_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    int rc = 0;
    
    // Route to appropriate handler based on target
    if (file_proxy_state.current_target == FILE_TARGET_PRIMARY) {
        rc = handle_primary_file_operation(file_proxy_state.current_operation, NULL, 0);
    } else {
        rc = forward_to_secondary(file_proxy_state.current_operation, NULL, 0);
    }
    
    if (rc != 0) {
        file_proxy_state.status = FILE_PROXY_STATUS_ERROR;
    }
    
    // Operation complete
    file_proxy_state.is_active = false;
    k_work_cancel_delayable(&file_timeout_work);
    file_proxy_notify_status(file_proxy_state.status);
}

// Timeout handler
static void file_proxy_timeout_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    LOG_ERR("File proxy timeout");
    file_proxy_state.is_active = false;
    file_proxy_state.status = FILE_PROXY_STATUS_ERROR;
    file_proxy_notify_status(file_proxy_state.status);
}

// D2D callback implementations
static void d2d_file_data_callback(const uint8_t *data, size_t len)
{
    // Forward data to phone via file proxy data characteristic
    if (file_proxy_state.is_active) {
        file_proxy_notify_data(data, len);
    }
}

static void d2d_file_status_callback(enum d2d_file_status status)
{
    // Map D2D status to file proxy status
    switch (status) {
        case D2D_FILE_STATUS_OK:
            file_proxy_state.status = FILE_PROXY_STATUS_SUCCESS;
            break;
        case D2D_FILE_STATUS_ERROR:
            file_proxy_state.status = FILE_PROXY_STATUS_ERROR;
            break;
        case D2D_FILE_STATUS_NOT_FOUND:
            file_proxy_state.status = FILE_PROXY_STATUS_FILE_NOT_FOUND;
            break;
        case D2D_FILE_STATUS_BUSY:
            file_proxy_state.status = FILE_PROXY_STATUS_BUSY;
            break;
        case D2D_FILE_STATUS_END_OF_DATA:
            file_proxy_state.status = FILE_PROXY_STATUS_SUCCESS;
            file_proxy_state.is_active = false;
            break;
    }
    
    file_proxy_notify_status(file_proxy_state.status);
}

// Public functions
int file_proxy_init(void)
{
    // Initialize work items
    k_work_init(&file_work, file_proxy_work_handler);
    k_work_init_delayable(&file_timeout_work, file_proxy_timeout_handler);

    // Set D2D callbacks
    ble_d2d_file_set_callbacks(d2d_file_data_callback, d2d_file_status_callback);

    LOG_INF("File proxy service initialized");
    return 0;
}

void file_proxy_set_secondary_conn(struct bt_conn *conn)
{
    file_proxy_state.secondary_conn = conn;
    
    if (conn) {
        LOG_INF("Secondary device connection set for file proxy");
    } else {
        LOG_INF("Secondary device disconnected from file proxy");
        
        if (file_proxy_state.is_active && 
            file_proxy_state.current_target == FILE_TARGET_SECONDARY) {
            file_proxy_state.status = FILE_PROXY_STATUS_NO_TARGET;
            file_proxy_notify_status(file_proxy_state.status);
        }
    }
}

enum file_proxy_status file_proxy_get_status(void)
{
    return file_proxy_state.status;
}

int file_proxy_notify_status(enum file_proxy_status status)
{
    file_status_value = (uint8_t)status;
    
    // Send notification if anyone is subscribed
    // Status characteristic is at index 8
    return bt_gatt_notify(NULL, &file_proxy_svc.attrs[8], 
                         &file_status_value, sizeof(file_status_value));
}

int file_proxy_notify_data(const uint8_t *data, size_t len)
{
    // Data characteristic is at index 5
    return bt_gatt_notify(NULL, &file_proxy_svc.attrs[5], data, len);
}
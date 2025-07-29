/**
 * @file ble_d2d_file_transfer.cpp
 * @brief Device-to-Device file transfer protocol implementation
 */

#include "ble_d2d_file_transfer.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <string.h>
#include <stdio.h>
#include "ccc_callback_fix.hpp"

LOG_MODULE_REGISTER(d2d_file_transfer, LOG_LEVEL_INF);

// File transfer state
static struct {
    bool is_active;
    enum d2d_file_cmd current_cmd;
    struct fs_file_t file;
    bool file_open;
    uint16_t sequence;
    d2d_file_data_cb_t data_cb;
    d2d_file_status_cb_t status_cb;
    struct bt_conn *conn;
} d2d_file_state = {
    .is_active = false,
    .current_cmd = (enum d2d_file_cmd)0,
    .file = {},
    .file_open = false,
    .sequence = 0,
    .data_cb = NULL,
    .status_cb = NULL,
    .conn = NULL,
};

// Buffer for file operations
#define FILE_BUFFER_SIZE 240
static uint8_t file_buffer[FILE_BUFFER_SIZE];

// Forward declarations
static ssize_t d2d_file_command_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static void d2d_file_data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void d2d_file_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value);

// For secondary device - handle incoming commands
static int handle_list_command(void);
static int handle_read_command(uint8_t file_id, uint8_t file_type);
static int handle_delete_command(uint8_t file_id, uint8_t file_type);
static int handle_info_command(uint8_t file_id, uint8_t file_type);

// GATT Service Definition for Secondary Device
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
BT_GATT_SERVICE_DEFINE(d2d_file_svc,
    BT_GATT_PRIMARY_SERVICE(D2D_FILE_SERVICE_UUID),
    
    // Command characteristic (write from primary)
    BT_GATT_CHARACTERISTIC(D2D_FILE_COMMAND_UUID,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, d2d_file_command_write, NULL),
    
    // Data characteristic (notify to primary)
    BT_GATT_CHARACTERISTIC(D2D_FILE_DATA_UUID,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(d2d_file_data_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Status characteristic (notify to primary)
    BT_GATT_CHARACTERISTIC(D2D_FILE_STATUS_UUID,
                          BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_NONE,
                          NULL, NULL, NULL),
    BT_GATT_CCC(d2d_file_status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);
#endif

// Command write handler for secondary device
static ssize_t d2d_file_command_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    
    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    const uint8_t *data = (const uint8_t *)buf;
    enum d2d_file_cmd cmd = (enum d2d_file_cmd)data[0];
    
    LOG_INF("D2D File command received: 0x%02x", cmd);
    
    // Store connection for responses
    d2d_file_state.conn = conn;
    
    // Handle command
    int rc = 0;
    switch (cmd) {
        case D2D_FILE_CMD_LIST:
            rc = handle_list_command();
            break;
            
        case D2D_FILE_CMD_READ:
            if (len >= 3) {
                rc = handle_read_command(data[1], data[2]);
            } else {
                rc = -EINVAL;
            }
            break;
            
        case D2D_FILE_CMD_DELETE:
            if (len >= 3) {
                rc = handle_delete_command(data[1], data[2]);
            } else {
                rc = -EINVAL;
            }
            break;
            
        case D2D_FILE_CMD_INFO:
            if (len >= 3) {
                rc = handle_info_command(data[1], data[2]);
            } else {
                rc = -EINVAL;
            }
            break;
            
        case D2D_FILE_CMD_ABORT:
            if (d2d_file_state.file_open) {
                fs_close(&d2d_file_state.file);
                d2d_file_state.file_open = false;
            }
            d2d_file_state.is_active = false;
            ble_d2d_file_send_status(D2D_FILE_STATUS_OK);
            break;
            
        default:
            LOG_WRN("Unknown D2D file command: 0x%02x", cmd);
            rc = -ENOTSUP;
    }
    
    if (rc != 0) {
        ble_d2d_file_send_status(D2D_FILE_STATUS_ERROR);
    }
    
    return len;
}

// CCC changed callbacks
static void d2d_file_data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("d2d_file_data_ccc_changed: attr is NULL");
        return;
    }
    LOG_DBG("D2D File data CCC changed: %u", value);
}

static void d2d_file_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("d2d_file_status_ccc_changed: attr is NULL");
        return;
    }
    LOG_DBG("D2D File status CCC changed: %u", value);
}

// Get file path based on ID and type
static void get_file_path(char *path, size_t path_size, uint8_t file_id, uint8_t file_type)
{
    const char *prefix = (file_type == 0x01) ? "foot" : "bhi360";
    int ret = snprintf(path, path_size, "/lfs/%s_%d.bin", prefix, file_id);
    if (ret < 0 || (size_t)ret >= path_size) {
        LOG_ERR("Path buffer too small for file path");
        path[0] = '\0';
    }
}

// Handle list command - enumerate files in /lfs directory
static int handle_list_command(void)
{
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int rc;
    
    // Open directory
    fs_dir_t_init(&dir);
    rc = fs_opendir(&dir, "/lfs");
    if (rc != 0) {
        LOG_ERR("Failed to open /lfs directory: %d", rc);
        return rc;
    }
    
    // Count files and build list
    uint8_t file_count = 0;
    
    // Define file entry structure
    struct file_entry_t {
        uint8_t id;
        uint8_t type;
        uint32_t size;
        uint32_t timestamp;
        char name[32];
    } __packed;
    
    // Allocate packet with enough space for file entry
    uint8_t packet_buffer[sizeof(struct d2d_file_packet) + sizeof(struct file_entry_t)];
    struct d2d_file_packet *packet = (struct d2d_file_packet *)packet_buffer;
    packet->cmd = D2D_FILE_CMD_LIST;
    packet->status = D2D_FILE_STATUS_OK;
    packet->sequence = 0;
    
    // First pass - count files
    while (fs_readdir(&dir, &entry) == 0) {
        if (entry.type == FS_DIR_ENTRY_FILE) {
            file_count++;
        }
    }
    
    // Send count
    packet->length = 1;
    packet->data[0] = file_count;
    ble_d2d_file_send_data((uint8_t *)packet, sizeof(struct d2d_file_packet) + 1, 0);
    
    // Reset directory
    fs_closedir(&dir);
    rc = fs_opendir(&dir, "/lfs");
    if (rc != 0) {
        return rc;
    }
    
    // Second pass - send file entries
    uint16_t seq = 1;
    while (fs_readdir(&dir, &entry) == 0) {
        if (entry.type == FS_DIR_ENTRY_FILE) {
            // Parse filename to extract ID and type
            uint8_t file_id = 0;
            uint8_t file_type = 0;
            
            if (strncmp(entry.name, "foot_", 5) == 0) {
                file_type = 0x01;
                if (sscanf(entry.name + 5, "%3hhu", &file_id) != 1) {
                    LOG_WRN("Failed to parse foot sensor file ID from: %s", entry.name);
                    continue;
                }
            } else if (strncmp(entry.name, "bhi360_", 7) == 0) {
                file_type = 0x02;
                if (sscanf(entry.name + 7, "%3hhu", &file_id) != 1) {
                    LOG_WRN("Failed to parse BHI360 file ID from: %s", entry.name);
                    continue;
                }
            }
            
            // Build file entry
            struct file_entry_t file_entry;
            
            file_entry.id = file_id;
            file_entry.type = file_type;
            file_entry.size = entry.size;
            file_entry.timestamp = 0; // TODO: Get actual timestamp
            strncpy(file_entry.name, entry.name, sizeof(file_entry.name) - 1);
            file_entry.name[sizeof(file_entry.name) - 1] = '\0';
            
            // Send entry
            // Boundary check and rolling mechanism for sequence
            // Risk: Overflows after 65,535 packets
            // Strategy: Reset to 0 on overflow; log event; notify receiver if needed
            if (seq >= UINT16_MAX - 1) {
                LOG_WRN("sequence rolling over - resetting to 0");
                seq = 0; // Reset to 0 as rolling mechanism
                // TODO: Send reset packet to notify receiver
            }
            packet->sequence = seq++;
            packet->length = sizeof(file_entry);
            memcpy(packet->data, &file_entry, sizeof(file_entry));
            ble_d2d_file_send_data((uint8_t *)packet, sizeof(struct d2d_file_packet) + sizeof(file_entry), seq);
        }
    }
    
    fs_closedir(&dir);
    
    // Send end of data
    ble_d2d_file_send_status(D2D_FILE_STATUS_END_OF_DATA);
    
    return 0;
}

// Handle read command - stream file data
static int handle_read_command(uint8_t file_id, uint8_t file_type)
{
    char path[64];
    int rc;
    
    get_file_path(path, sizeof(path), file_id, file_type);
    
    LOG_INF("Reading file: %s", path);
    
    // Open file
    fs_file_t_init(&d2d_file_state.file);
    rc = fs_open(&d2d_file_state.file, path, FS_O_READ);
    if (rc != 0) {
        LOG_ERR("Failed to open file %s: %d", path, rc);
        ble_d2d_file_send_status(D2D_FILE_STATUS_NOT_FOUND);
        return rc;
    }
    
    d2d_file_state.file_open = true;
    d2d_file_state.sequence = 0;
    
    // Read and send file in chunks
    uint8_t packet_buffer[sizeof(struct d2d_file_packet) + FILE_BUFFER_SIZE];
    struct d2d_file_packet *packet = (struct d2d_file_packet *)packet_buffer;
    packet->cmd = D2D_FILE_CMD_READ;
    packet->status = D2D_FILE_STATUS_OK;
    
    while (true) {
        ssize_t bytes_read = fs_read(&d2d_file_state.file, file_buffer, FILE_BUFFER_SIZE);
        if (bytes_read <= 0) {
            break;
        }
        
        // Boundary check and rolling mechanism for sequence
        // Risk: Overflows after 65,535 packets
        // Strategy: Reset to 0 on overflow; log event; notify receiver if needed
        if (d2d_file_state.sequence >= UINT16_MAX - 1) {
            LOG_WRN("sequence rolling over - resetting to 0");
            d2d_file_state.sequence = 0; // Reset to 0 as rolling mechanism
            // TODO: Send reset packet to notify receiver
        }
        packet->sequence = d2d_file_state.sequence++;
        packet->length = bytes_read;
        memcpy(packet->data, file_buffer, bytes_read);
        
        // Send data packet
        ble_d2d_file_send_data((uint8_t *)packet, sizeof(struct d2d_file_packet) + bytes_read, packet->sequence);
        
        // Small delay to avoid overwhelming the connection
        k_msleep(10);
    }
    
    // Close file
    fs_close(&d2d_file_state.file);
    d2d_file_state.file_open = false;
    
    // Send end of data
    ble_d2d_file_send_status(D2D_FILE_STATUS_END_OF_DATA);
    
    return 0;
}

// Handle delete command
static int handle_delete_command(uint8_t file_id, uint8_t file_type)
{
    char path[64];
    int rc;
    
    get_file_path(path, sizeof(path), file_id, file_type);
    
    LOG_INF("Deleting file: %s", path);
    
    rc = fs_unlink(path);
    if (rc != 0) {
        LOG_ERR("Failed to delete file %s: %d", path, rc);
        ble_d2d_file_send_status(D2D_FILE_STATUS_NOT_FOUND);
        return rc;
    }
    
    ble_d2d_file_send_status(D2D_FILE_STATUS_OK);
    
    return 0;
}

// Handle info command
static int handle_info_command(uint8_t file_id, uint8_t file_type)
{
    char path[64];
    int rc;
    
    get_file_path(path, sizeof(path), file_id, file_type);
    
    // Use fs_stat alternative - check if file exists and get size
    struct fs_file_t tmp_file;
    fs_file_t_init(&tmp_file);
    rc = fs_open(&tmp_file, path, FS_O_READ);
    if (rc != 0) {
        LOG_ERR("Failed to open file %s: %d", path, rc);
        ble_d2d_file_send_status(D2D_FILE_STATUS_NOT_FOUND);
        return rc;
    }
    
    // Get file size
    off_t file_size = fs_tell(&tmp_file);
    fs_seek(&tmp_file, 0, FS_SEEK_END);
    file_size = fs_tell(&tmp_file);
    fs_close(&tmp_file);
    
    // Build info response
    struct {
        uint8_t cmd;
        uint8_t status;
        uint8_t id;
        uint8_t type;
        uint32_t size;
        uint32_t timestamp;
        uint32_t crc32;
    } __packed info_response;
    
    info_response.cmd = D2D_FILE_CMD_INFO;
    info_response.status = D2D_FILE_STATUS_OK;
    info_response.id = file_id;
    info_response.type = file_type;
    info_response.size = (uint32_t)file_size;
    info_response.timestamp = 0; // TODO: Get actual timestamp
    info_response.crc32 = 0; // TODO: Calculate CRC32
    
    // Send info
    ble_d2d_file_send_data((uint8_t *)&info_response, sizeof(info_response), 0);
    
    return 0;
}

// Public functions

void ble_d2d_file_transfer_init(void)
{
    LOG_INF("D2D File Transfer service initialized");
}

void ble_d2d_file_client_init(struct bt_conn *conn)
{
    d2d_file_state.conn = conn;
    if (conn) {
        LOG_INF("D2D File Transfer client initialized");
    } else {
        LOG_INF("D2D File Transfer client cleared");
        // Clear any active state
        if (d2d_file_state.file_open) {
            fs_close(&d2d_file_state.file);
            d2d_file_state.file_open = false;
        }
        d2d_file_state.is_active = false;
    }
}

void ble_d2d_file_set_callbacks(d2d_file_data_cb_t data_cb, d2d_file_status_cb_t status_cb)
{
    d2d_file_state.data_cb = data_cb;
    d2d_file_state.status_cb = status_cb;
}

int ble_d2d_file_send_command(enum d2d_file_cmd cmd, const uint8_t *data, size_t len)
{
    ARG_UNUSED(cmd);
    ARG_UNUSED(data);
    ARG_UNUSED(len);
    
    // This would be implemented on the primary device to send commands to secondary
    // For now, return not supported
    return -ENOTSUP;
}

int ble_d2d_file_send_data(const uint8_t *data, size_t len, uint16_t sequence)
{
    ARG_UNUSED(sequence);
    
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!d2d_file_state.conn) {
        return -ENOTCONN;
    }
    
    // Send via data characteristic (index 2)
    return bt_gatt_notify(d2d_file_state.conn, &d2d_file_svc.attrs[2], data, len);
#else
    ARG_UNUSED(data);
    ARG_UNUSED(len);
    return -ENOTSUP;
#endif
}

int ble_d2d_file_send_status(enum d2d_file_status status)
{
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!d2d_file_state.conn) {
        return -ENOTCONN;
    }
    
    uint8_t status_byte = (uint8_t)status;
    
    // Send via status characteristic (index 5)
    return bt_gatt_notify(d2d_file_state.conn, &d2d_file_svc.attrs[5], &status_byte, sizeof(status_byte));
#else
    return -ENOTSUP;
#endif
}
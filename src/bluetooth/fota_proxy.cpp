/**
 * @file fota_proxy.cpp
 * @brief FOTA Proxy Service implementation
 */

#include "fota_proxy.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <string.h>
#include <dfu/dfu_target.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(fota_proxy, LOG_LEVEL_WRN);

// Buffer sizes
#define FOTA_PROXY_BUF_SIZE 512
#define FOTA_PROXY_MTU_SIZE 244

// Global flag for D2D secondary FOTA completion
bool d2d_secondary_fota_complete = false;

// State management
static struct {
    struct bt_conn *secondary_conn;
    enum fota_proxy_target current_target;
    enum fota_proxy_status status;
    bool is_active;
    uint32_t total_size;
    uint32_t transferred_size;
    uint8_t data_buffer[FOTA_PROXY_BUF_SIZE];
    size_t data_len;
    bool dfu_initialized;
    // Synchronization fields for coordinated updates
    bool primary_complete;
    bool secondary_complete;
    bool waiting_for_secondary;
} fota_proxy_state = {
    .secondary_conn = NULL,
    .current_target = FOTA_TARGET_PRIMARY,
    .status = FOTA_PROXY_STATUS_IDLE,
    .is_active = false,
    .total_size = 0,
    .transferred_size = 0,
    .data_buffer = {0},
    .data_len = 0,
    .dfu_initialized = false,
    .primary_complete = false,
    .secondary_complete = false,
    .waiting_for_secondary = false,
};

// Work queue for async operations
static struct k_work fota_work;
static struct k_work_delayable fota_timeout_work;
static struct k_work_delayable fota_reset_work;

// Forward declarations
static void fota_proxy_work_handler(struct k_work *work);
static void fota_proxy_timeout_handler(struct k_work *work);
static void fota_proxy_reset_handler(struct k_work *work);
static int forward_to_secondary(uint8_t cmd, const uint8_t *data, size_t len);

// GATT characteristic values
static uint8_t fota_target_value = FOTA_TARGET_PRIMARY;
static uint8_t fota_status_value = FOTA_PROXY_STATUS_IDLE;

// GATT callbacks
static ssize_t fota_proxy_target_write(struct bt_conn *conn,
                                       const struct bt_gatt_attr *attr,
                                       const void *buf, uint16_t len,
                                       uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    if (offset + len > sizeof(fota_target_value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(&fota_target_value + offset, buf, len);
    fota_proxy_state.current_target = (enum fota_proxy_target)fota_target_value;
    
    LOG_INF("FOTA target set to: %s", 
            fota_target_value == FOTA_TARGET_PRIMARY ? "PRIMARY" :
            fota_target_value == FOTA_TARGET_SECONDARY ? "SECONDARY" : "ALL");

    return len;
}

static ssize_t fota_proxy_command_write(struct bt_conn *conn,
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

    const uint8_t *data = (const uint8_t *)buf;
    uint8_t cmd = data[0];

    LOG_INF("FOTA proxy command received: 0x%02x", cmd);

    switch (cmd) {
        case FOTA_PROXY_CMD_START:
            if (len >= 5) {
                // Extract total size from command
                fota_proxy_state.total_size = sys_get_le32(&data[1]);
                fota_proxy_state.transferred_size = 0;
                fota_proxy_state.is_active = true;
                fota_proxy_state.status = FOTA_PROXY_STATUS_IN_PROGRESS;
                
                // Reset completion flags
                fota_proxy_state.primary_complete = false;
                fota_proxy_state.secondary_complete = false;
                fota_proxy_state.waiting_for_secondary = false;
                
                LOG_INF("Starting FOTA update, total size: %u bytes", fota_proxy_state.total_size);
                
                // Start timeout timer
                k_work_schedule(&fota_timeout_work, K_SECONDS(CONFIG_FOTA_PROXY_TIMEOUT_SEC));
                
                // Forward to secondary if needed
                if (fota_proxy_state.current_target == FOTA_TARGET_SECONDARY ||
                    fota_proxy_state.current_target == FOTA_TARGET_ALL) {
                    forward_to_secondary(cmd, &data[1], len - 1);
                }
            }
            break;

        case FOTA_PROXY_CMD_DATA:
            if (fota_proxy_state.is_active && len > 1) {
                size_t data_size = len - 1;
                
                // Buffer the data
                if (fota_proxy_state.data_len + data_size <= FOTA_PROXY_BUF_SIZE) {
                    memcpy(&fota_proxy_state.data_buffer[fota_proxy_state.data_len], 
                           &data[1], data_size);
                    fota_proxy_state.data_len += data_size;
                    fota_proxy_state.transferred_size += data_size;
                    
                    // Submit work to process the data
                    k_work_submit(&fota_work);
                }
            }
            break;

        case FOTA_PROXY_CMD_END:
            if (fota_proxy_state.is_active) {
                LOG_INF("FOTA update completed, transferred %u bytes", 
                        fota_proxy_state.transferred_size);
                fota_proxy_state.is_active = false;
                fota_proxy_state.status = FOTA_PROXY_STATUS_SUCCESS;
                k_work_cancel_delayable(&fota_timeout_work);
                
                // Forward to secondary if needed
                if (fota_proxy_state.current_target == FOTA_TARGET_SECONDARY ||
                    fota_proxy_state.current_target == FOTA_TARGET_ALL) {
                    forward_to_secondary(cmd, NULL, 0);
                }
            }
            break;

        case FOTA_PROXY_CMD_ABORT:
            LOG_WRN("FOTA update aborted");
            fota_proxy_state.is_active = false;
            fota_proxy_state.status = FOTA_PROXY_STATUS_ERROR;
            k_work_cancel_delayable(&fota_timeout_work);
            
            // Forward abort to secondary
            if (fota_proxy_state.current_target == FOTA_TARGET_SECONDARY ||
                fota_proxy_state.current_target == FOTA_TARGET_ALL) {
                forward_to_secondary(cmd, NULL, 0);
            }
            break;

        case FOTA_PROXY_CMD_STATUS:
            // Status query - update will be sent via notification
            fota_proxy_notify_status(fota_proxy_state.status);
            break;

        case FOTA_PROXY_CMD_RESET:
            LOG_INF("Reset command received for target: %d", fota_proxy_state.current_target);
            
            if (fota_proxy_state.current_target == FOTA_TARGET_ALL) {
                // For ALL target, need to coordinate reset
                fota_proxy_state.primary_complete = true;
                
                // Check if secondary already completed via D2D flag
                if (d2d_secondary_fota_complete) {
                    fota_proxy_state.secondary_complete = true;
                    d2d_secondary_fota_complete = false; // Reset flag
                }
                
                if (fota_proxy_state.secondary_complete) {
                    // Both devices are ready, safe to reset
                    LOG_INF("Both devices complete, scheduling reset");
                    fota_proxy_state.status = FOTA_PROXY_STATUS_BOTH_COMPLETE;
                    k_work_schedule(&fota_reset_work, K_SECONDS(2));
                } else {
                    // Wait for secondary to complete
                    fota_proxy_state.waiting_for_secondary = true;
                    fota_proxy_state.status = FOTA_PROXY_STATUS_WAITING_SECONDARY;
                    LOG_INF("Waiting for secondary device to complete FOTA...");
                    
                    // Set a timeout in case secondary fails
                    k_work_schedule(&fota_reset_work, K_SECONDS(CONFIG_FOTA_SYNC_TIMEOUT_SEC));
                }
            } else if (fota_proxy_state.current_target == FOTA_TARGET_PRIMARY) {
                // Primary only, reset after delay
                LOG_INF("Primary-only update, scheduling reset");
                k_work_schedule(&fota_reset_work, K_SECONDS(2));
            } else if (fota_proxy_state.current_target == FOTA_TARGET_SECONDARY) {
                // Secondary only, no reset needed on primary
                LOG_INF("Secondary-only update, no primary reset needed");
            }
            break;

        case FOTA_PROXY_CMD_SECONDARY_COMPLETE:
            LOG_INF("Secondary device reported FOTA complete");
            fota_proxy_state.secondary_complete = true;
            
            // Check if primary is waiting for secondary
            if (fota_proxy_state.waiting_for_secondary && fota_proxy_state.primary_complete) {
                // Both are now complete, proceed with reset
                LOG_INF("Both devices now complete, scheduling reset");
                fota_proxy_state.status = FOTA_PROXY_STATUS_BOTH_COMPLETE;
                k_work_cancel_delayable(&fota_reset_work); // Cancel timeout
                k_work_schedule(&fota_reset_work, K_SECONDS(2));
            }
            break;

        default:
            LOG_WRN("Unknown FOTA proxy command: 0x%02x", cmd);
            return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
    }

    // Update status
    fota_proxy_notify_status(fota_proxy_state.status);

    return len;
}

static ssize_t fota_proxy_data_write(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len,
                                     uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    if (!fota_proxy_state.is_active) {
        return BT_GATT_ERR(BT_ATT_ERR_WRITE_NOT_PERMITTED);
    }

    // This is for direct data transfer without command byte
    if (fota_proxy_state.data_len + len <= FOTA_PROXY_BUF_SIZE) {
        memcpy(&fota_proxy_state.data_buffer[fota_proxy_state.data_len], buf, len);
        fota_proxy_state.data_len += len;
        fota_proxy_state.transferred_size += len;
        
        // Submit work to process the data
        k_work_submit(&fota_work);
    }

    return len;
}

static ssize_t fota_proxy_status_read(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &fota_status_value, sizeof(fota_status_value));
}

// CCC changed callback
static void fota_proxy_status_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("fota_proxy_status_ccc_changed: attr is NULL");
        return;
    }
    LOG_DBG("FOTA proxy status CCC changed: %u", value);
}

// GATT Service Definition
BT_GATT_SERVICE_DEFINE(fota_proxy_svc,
    BT_GATT_PRIMARY_SERVICE(FOTA_PROXY_SERVICE_UUID),
    
    // Target selection characteristic (write)
    BT_GATT_CHARACTERISTIC(FOTA_PROXY_TARGET_UUID,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, fota_proxy_target_write, &fota_target_value),
    
    // Command characteristic (write)
    BT_GATT_CHARACTERISTIC(FOTA_PROXY_COMMAND_UUID,
                          BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                          BT_GATT_PERM_WRITE,
                          NULL, fota_proxy_command_write, NULL),
    
    // Data characteristic (write)
    BT_GATT_CHARACTERISTIC(FOTA_PROXY_DATA_UUID,
                          BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                          BT_GATT_PERM_WRITE,
                          NULL, fota_proxy_data_write, NULL),
    
    // Status characteristic (read/notify)
    BT_GATT_CHARACTERISTIC(FOTA_PROXY_STATUS_UUID,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          fota_proxy_status_read, NULL, &fota_status_value),
    BT_GATT_CCC(fota_proxy_status_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// Forward data to secondary device
static int forward_to_secondary(uint8_t cmd, const uint8_t *data, size_t len)
{
    ARG_UNUSED(data);
    ARG_UNUSED(len);
    if (!fota_proxy_state.secondary_conn) {
        LOG_ERR("No secondary device connected");
        fota_proxy_state.status = FOTA_PROXY_STATUS_NO_TARGET;
        return -ENOTCONN;
    }

    int rc = 0;

    switch (cmd) {
        case FOTA_PROXY_CMD_START:
            // Initialize DFU target for SMP if not already done
            if (!fota_proxy_state.dfu_initialized) {
                rc = dfu_target_init(DFU_TARGET_IMAGE_TYPE_SMP, 0, fota_proxy_state.total_size, NULL);
                if (rc != 0) {
                    LOG_ERR("Failed to initialize DFU target: %d", rc);
                    fota_proxy_state.status = FOTA_PROXY_STATUS_ERROR;
                    return rc;
                }
                fota_proxy_state.dfu_initialized = true;
            }
            LOG_INF("Starting image upload to secondary device");
            break;

        case FOTA_PROXY_CMD_DATA:
            // This will be handled in the work handler
            break;

        case FOTA_PROXY_CMD_END:
            // Finalize the DFU
            if (fota_proxy_state.dfu_initialized) {
                rc = dfu_target_done(true);
                if (rc != 0) {
                    LOG_ERR("Failed to finalize DFU: %d", rc);
                    fota_proxy_state.status = FOTA_PROXY_STATUS_ERROR;
                }
                fota_proxy_state.dfu_initialized = false;
            }
            LOG_INF("Finalizing image on secondary device");
            break;

        case FOTA_PROXY_CMD_ABORT:
            // Abort the DFU
            if (fota_proxy_state.dfu_initialized) {
                rc = dfu_target_done(false);
                fota_proxy_state.dfu_initialized = false;
            }
            LOG_INF("Aborting image on secondary device");
            break;
    }

    return rc;
}

// Work handler for processing buffered data
static void fota_proxy_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (fota_proxy_state.data_len == 0) {
        return;
    }

    // Forward data to appropriate target
    if (fota_proxy_state.current_target == FOTA_TARGET_SECONDARY ||
        fota_proxy_state.current_target == FOTA_TARGET_ALL) {
        
        if (fota_proxy_state.secondary_conn && fota_proxy_state.dfu_initialized) {
            // Write data using DFU target
            int rc = dfu_target_write(fota_proxy_state.data_buffer, fota_proxy_state.data_len);
            if (rc < 0) {
                LOG_ERR("Failed to write data to DFU target: %d", rc);
                fota_proxy_state.status = FOTA_PROXY_STATUS_ERROR;
            } else {
                LOG_DBG("Wrote %d bytes to DFU target", fota_proxy_state.data_len);
            }
        }
    }

    if (fota_proxy_state.current_target == FOTA_TARGET_PRIMARY ||
        fota_proxy_state.current_target == FOTA_TARGET_ALL) {
        // For primary device, the standard MCUmgr handles it
        // This proxy just monitors the progress
    }

    // Clear buffer
    fota_proxy_state.data_len = 0;
}

// Timeout handler
static void fota_proxy_timeout_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    // Check if secondary completed while we were waiting
    if (d2d_secondary_fota_complete && fota_proxy_state.waiting_for_secondary) {
        LOG_INF("Secondary FOTA completed during timeout period");
        fota_proxy_state.secondary_complete = true;
        d2d_secondary_fota_complete = false; // Reset flag
        
        if (fota_proxy_state.primary_complete) {
            // Both are now complete, proceed with reset
            LOG_INF("Both devices now complete, scheduling reset");
            fota_proxy_state.status = FOTA_PROXY_STATUS_BOTH_COMPLETE;
            fota_proxy_notify_status(fota_proxy_state.status);
            k_work_schedule(&fota_reset_work, K_SECONDS(2));
            return;
        }
    }
    
    LOG_ERR("FOTA proxy timeout");
    fota_proxy_state.is_active = false;
    fota_proxy_state.status = FOTA_PROXY_STATUS_ERROR;
    
    // Clean up DFU if initialized
    if (fota_proxy_state.dfu_initialized) {
        dfu_target_done(false);
        fota_proxy_state.dfu_initialized = false;
    }
    
    fota_proxy_notify_status(fota_proxy_state.status);
}

// Reset handler - performs the actual system reset
static void fota_proxy_reset_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (fota_proxy_state.waiting_for_secondary && !fota_proxy_state.secondary_complete) {
        LOG_WRN("Timeout waiting for secondary device, resetting anyway");
    }
    
    LOG_INF("Performing system reset for FOTA completion");
    
    // Give time for final log messages
    k_sleep(K_MSEC(100));
    
    // Perform warm reset
    sys_reboot(SYS_REBOOT_WARM);
}

// Public functions
int fota_proxy_init(void)
{
    // Initialize work items
    k_work_init(&fota_work, fota_proxy_work_handler);
    k_work_init_delayable(&fota_timeout_work, fota_proxy_timeout_handler);
    k_work_init_delayable(&fota_reset_work, fota_proxy_reset_handler);

    LOG_INF("FOTA proxy service initialized");
    return 0;
}

void fota_proxy_set_secondary_conn(struct bt_conn *conn)
{
    fota_proxy_state.secondary_conn = conn;
    
    if (conn) {
        LOG_INF("Secondary device connection set for FOTA proxy");
    } else {
        LOG_INF("Secondary device disconnected from FOTA proxy");
        
        // Clean up any ongoing DFU
        if (fota_proxy_state.dfu_initialized) {
            dfu_target_done(false);
            fota_proxy_state.dfu_initialized = false;
        }
        
        if (fota_proxy_state.is_active && 
            fota_proxy_state.current_target == FOTA_TARGET_SECONDARY) {
            fota_proxy_state.status = FOTA_PROXY_STATUS_NO_TARGET;
            fota_proxy_notify_status(fota_proxy_state.status);
        }
    }
}

enum fota_proxy_status fota_proxy_get_status(void)
{
    return fota_proxy_state.status;
}

int fota_proxy_notify_status(enum fota_proxy_status status)
{
    fota_status_value = (uint8_t)status;
    
    // Send notification if anyone is subscribed
    // Status characteristic is at index 7 (0-based)
    // 0: Primary Service
    // 1-2: Target characteristic + value
    // 3-4: Command characteristic + value  
    // 5-6: Data characteristic + value
    // 7-8: Status characteristic + value
    // 9: CCC
    if (fota_proxy_svc.attrs && fota_proxy_svc.attr_count > 7) {
        return bt_gatt_notify(NULL, &fota_proxy_svc.attrs[7], 
                             &fota_status_value, sizeof(fota_status_value));
    } else {
        LOG_WRN("FOTA proxy service not ready for status notification");
        return -EINVAL;
    }
}

int fota_proxy_handle_secondary_complete(void)
{
    LOG_INF("Secondary device FOTA completion received");
    fota_proxy_state.secondary_complete = true;
    
    // Check if primary is waiting for secondary
    if (fota_proxy_state.waiting_for_secondary && fota_proxy_state.primary_complete) {
        // Both are now complete, proceed with reset
        LOG_INF("Both devices now complete, scheduling reset");
        fota_proxy_state.status = FOTA_PROXY_STATUS_BOTH_COMPLETE;
        fota_proxy_notify_status(fota_proxy_state.status);
        
        k_work_cancel_delayable(&fota_reset_work); // Cancel timeout
        k_work_schedule(&fota_reset_work, K_SECONDS(2));
    }
    
    return 0;
}
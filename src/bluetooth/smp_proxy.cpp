/**
 * @file smp_proxy.cpp
 * @brief SMP Proxy Service implementation for transparent MCUmgr forwarding
 */

#include "smp_proxy.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include "ble_d2d_smp_client.hpp"

// MCUmgr includes
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt_defines.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>

// Define mgmt_hdr structure
struct mgmt_hdr {
    uint8_t  nh_op;
    uint8_t  nh_flags;
    uint16_t nh_len;
    uint16_t nh_group;
    uint8_t  nh_seq;
    uint8_t  nh_id;
} __attribute__((packed));

// MCUmgr constants if not defined
#ifndef MGMT_OP_WRITE_RSP
#define MGMT_OP_WRITE_RSP 3
#endif

#ifndef MGMT_GROUP_ID_OS
#define MGMT_GROUP_ID_OS 0
#endif

#ifndef MGMT_ERR_ENOENT
#define MGMT_ERR_ENOENT 2
#endif

#ifndef MGMT_ERR_ENOTSUP
#define MGMT_ERR_ENOTSUP 8
#endif

LOG_MODULE_REGISTER(smp_proxy, LOG_LEVEL_INF);

// Buffer sizes
#define SMP_PROXY_BUF_SIZE 512
#define SMP_PROXY_MTU_SIZE 498  // Aligned with standard SMP

// State management
static struct {
    struct bt_conn *phone_conn;      // Connection to phone
    struct bt_conn *secondary_conn;  // Connection to secondary device
    enum smp_proxy_target target;    // Current target device
    uint8_t rx_buffer[SMP_PROXY_BUF_SIZE];
    size_t rx_len;
    bool rx_in_progress;
    struct k_work_delayable timeout_work;
    struct k_sem tx_sem;
    // Response handling
    uint8_t response_buffer[SMP_PROXY_BUF_SIZE];
    size_t response_len;
    bool response_ready;
} smp_proxy_state = {
    .phone_conn = NULL,
    .secondary_conn = NULL,
    .target = SMP_TARGET_PRIMARY,
    .rx_buffer = {0},
    .rx_len = 0,
    .rx_in_progress = false,
    .response_buffer = {0},
    .response_len = 0,
    .response_ready = false,
};

// Forward declarations
static void smp_proxy_timeout_handler(struct k_work *work);
static int forward_to_secondary(const uint8_t *data, size_t len);
static int process_primary_smp(const uint8_t *data, size_t len);
static void handle_secondary_response(const uint8_t *data, size_t len);

// GATT characteristic values
static uint8_t smp_target_value = SMP_TARGET_PRIMARY;

// Callback for D2D SMP responses from secondary
static void d2d_smp_response_cb(const uint8_t *data, size_t len)
{
    LOG_DBG("Received SMP response from secondary: %zu bytes", len);
    handle_secondary_response(data, len);
}

// GATT callbacks
static ssize_t smp_proxy_target_write(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len,
                                      uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    
    if (offset + len > sizeof(smp_target_value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(&smp_target_value + offset, buf, len);
    smp_proxy_state.target = (enum smp_proxy_target)smp_target_value;
    smp_proxy_state.phone_conn = conn;  // Remember phone connection
    
    LOG_INF("SMP proxy target set to: %s", 
            smp_target_value == SMP_TARGET_PRIMARY ? "PRIMARY" : "SECONDARY");

    // Clear any pending responses when switching targets
    smp_proxy_state.response_ready = false;
    smp_proxy_state.response_len = 0;

    return len;
}

static ssize_t smp_proxy_target_read(struct bt_conn *conn,
                                     const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &smp_target_value, sizeof(smp_target_value));
}

static ssize_t smp_proxy_data_write(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len,
                                    uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(attr);
    
    LOG_DBG("SMP data write: %u bytes, offset %u, flags 0x%02x", len, offset, flags);
    
    // Handle fragmented writes
    if (offset == 0) {
        // New SMP frame starting
        smp_proxy_state.rx_len = 0;
        smp_proxy_state.rx_in_progress = true;
        k_work_reschedule(&smp_proxy_state.timeout_work, K_SECONDS(5));
    }
    
    if (!smp_proxy_state.rx_in_progress) {
        LOG_ERR("Unexpected SMP data write");
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    
    // Append data to buffer
    if (smp_proxy_state.rx_len + len > SMP_PROXY_BUF_SIZE) {
        LOG_ERR("SMP data too large");
        smp_proxy_state.rx_in_progress = false;
        return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
    }
    
    memcpy(&smp_proxy_state.rx_buffer[smp_proxy_state.rx_len], buf, len);
    smp_proxy_state.rx_len += len;
    
    // Check if this is the last fragment
    if (!(flags & BT_GATT_WRITE_FLAG_PREPARE)) {
        // Complete frame received
        smp_proxy_state.rx_in_progress = false;
        k_work_cancel_delayable(&smp_proxy_state.timeout_work);
        
        LOG_DBG("Complete SMP frame received: %zu bytes", smp_proxy_state.rx_len);
        
        // Process based on target
        if (smp_proxy_state.target == SMP_TARGET_PRIMARY) {
            process_primary_smp(smp_proxy_state.rx_buffer, smp_proxy_state.rx_len);
        } else {
            forward_to_secondary(smp_proxy_state.rx_buffer, smp_proxy_state.rx_len);
        }
    }
    
    return len;
}

// CCC changed callback
static void smp_proxy_data_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("smp_proxy_data_ccc_changed: attr is NULL");
        return;
    }
    LOG_DBG("SMP proxy data CCC changed: %u", value);
}

// GATT Service Definition
BT_GATT_SERVICE_DEFINE(smp_proxy_svc,
    BT_GATT_PRIMARY_SERVICE(SMP_PROXY_SERVICE_UUID),
    
    // Target selection characteristic (read/write)
    BT_GATT_CHARACTERISTIC(SMP_PROXY_TARGET_UUID,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          smp_proxy_target_read, smp_proxy_target_write, &smp_target_value),
    
    // SMP data characteristic (write without response + notify)
    BT_GATT_CHARACTERISTIC(SMP_PROXY_DATA_UUID,
                          BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_WRITE,
                          NULL, smp_proxy_data_write, NULL),
    BT_GATT_CCC(smp_proxy_data_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// Forward SMP frame to secondary device via D2D
static int forward_to_secondary(const uint8_t *data, size_t len)
{
    if (!smp_proxy_state.secondary_conn) {
        LOG_ERR("No secondary device connected");
        // Send error response back to phone
        struct mgmt_hdr *hdr = (struct mgmt_hdr *)smp_proxy_state.response_buffer;
        hdr->nh_len = sys_cpu_to_be16(0);
        hdr->nh_flags = 0;
        hdr->nh_op = MGMT_OP_WRITE_RSP;
        hdr->nh_group = sys_cpu_to_be16(MGMT_GROUP_ID_OS);
        hdr->nh_seq = 0;
        hdr->nh_id = MGMT_ERR_ENOENT;
        
        smp_proxy_state.response_len = sizeof(struct mgmt_hdr);
        smp_proxy_state.response_ready = true;
        
        // Notify phone
        if (smp_proxy_state.phone_conn) {
            bt_gatt_notify(smp_proxy_state.phone_conn, &smp_proxy_svc.attrs[3], 
                          smp_proxy_state.response_buffer, smp_proxy_state.response_len);
        } else {
            LOG_WRN("No phone connection to send SMP error response");
        }
        return -ENOTCONN;
    }
    
    LOG_INF("Forwarding SMP frame to secondary device: %zu bytes", len);
    
    // Log first few bytes of the SMP frame for debugging
    if (len >= sizeof(struct mgmt_hdr)) {
        struct mgmt_hdr *hdr = (struct mgmt_hdr *)data;
        LOG_INF("SMP frame: op=%d, group=0x%04x, id=%d", 
                hdr->nh_op, sys_be16_to_cpu(hdr->nh_group), hdr->nh_id);
    }
    
    // Use D2D SMP client to forward
    int rc = ble_d2d_smp_client_send(data, len);
    if (rc != 0) {
        LOG_ERR("Failed to forward SMP to secondary: %d", rc);
        return rc;
    }
    
    LOG_INF("SMP frame forwarded successfully");
    return 0;
}

// Process SMP frame for primary device (local handling)
static int process_primary_smp(const uint8_t *data, size_t len)
{
    LOG_DBG("Processing SMP frame for primary device: %zu bytes", len);
    
    // For primary device, we need to pass it to the local SMP processor
    // This would typically be handled by the existing MCUmgr SMP transport
    // For now, we'll just log that we would process it locally
    
    LOG_WRN("Primary device SMP processing not implemented in proxy");
    LOG_WRN("Use standard SMP service for primary device operations");
    
    // Send error response
    struct mgmt_hdr *hdr = (struct mgmt_hdr *)smp_proxy_state.response_buffer;
    hdr->nh_len = sys_cpu_to_be16(0);
    hdr->nh_flags = 0;
    hdr->nh_op = MGMT_OP_WRITE_RSP;
    hdr->nh_group = sys_cpu_to_be16(MGMT_GROUP_ID_OS);
    hdr->nh_seq = 0;
    hdr->nh_id = MGMT_ERR_ENOTSUP;
    
    smp_proxy_state.response_len = sizeof(struct mgmt_hdr);
    smp_proxy_state.response_ready = true;
    
    // Notify phone
    if (smp_proxy_state.phone_conn) {
        bt_gatt_notify(smp_proxy_state.phone_conn, &smp_proxy_svc.attrs[3], 
                      smp_proxy_state.response_buffer, smp_proxy_state.response_len);
    } else {
        LOG_WRN("No phone connection to send primary SMP error response");
    }
    
    return -ENOTSUP;
}

// Handle SMP response from secondary device
static void handle_secondary_response(const uint8_t *data, size_t len)
{
    if (len > SMP_PROXY_BUF_SIZE) {
        LOG_ERR("Secondary response too large: %zu bytes", len);
        return;
    }
    
    // Copy response to buffer
    memcpy(smp_proxy_state.response_buffer, data, len);
    smp_proxy_state.response_len = len;
    smp_proxy_state.response_ready = true;
    
    // Forward response to phone via notification
    if (smp_proxy_state.phone_conn) {
        LOG_DBG("Forwarding secondary response to phone: %zu bytes", len);
        
        // Send via notification on the SMP data characteristic
        int rc = bt_gatt_notify(smp_proxy_state.phone_conn, &smp_proxy_svc.attrs[3], 
                               smp_proxy_state.response_buffer, smp_proxy_state.response_len);
        if (rc != 0) {
            LOG_ERR("Failed to notify SMP response: %d", rc);
        }
    } else {
        LOG_WRN("No phone connection to forward secondary response");
    }
}

// Timeout handler
static void smp_proxy_timeout_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    LOG_ERR("SMP proxy timeout - incomplete frame");
    smp_proxy_state.rx_in_progress = false;
    smp_proxy_state.rx_len = 0;
}

// Public functions
int smp_proxy_init(void)
{
    // Initialize work items
    k_work_init_delayable(&smp_proxy_state.timeout_work, smp_proxy_timeout_handler);
    k_sem_init(&smp_proxy_state.tx_sem, 1, 1);
    
    // Initialize D2D SMP client with our callback
    ble_d2d_smp_client_init(d2d_smp_response_cb);
    
    LOG_INF("SMP proxy service initialized");
    LOG_INF("Mobile apps can now use standard MCUmgr for both devices");
    LOG_INF("Just set target characteristic before operations:");
    LOG_INF("  0x00 = Primary device");
    LOG_INF("  0x01 = Secondary device");
    
    return 0;
}

void smp_proxy_set_secondary_conn(struct bt_conn *conn)
{
    smp_proxy_state.secondary_conn = conn;
    
    if (conn) {
        LOG_INF("Secondary device connection set for SMP proxy");
        // Set connection in D2D SMP client
        ble_d2d_smp_client_set_connection(conn);
    } else {
        LOG_INF("Secondary device disconnected from SMP proxy");
        ble_d2d_smp_client_set_connection(NULL);
        
        // If we were targeting secondary, switch back to primary
        if (smp_proxy_state.target == SMP_TARGET_SECONDARY) {
            smp_proxy_state.target = SMP_TARGET_PRIMARY;
            smp_target_value = SMP_TARGET_PRIMARY;
            LOG_INF("Switched target back to primary due to secondary disconnect");
        }
    }
}

enum smp_proxy_target smp_proxy_get_target(void)
{
    return smp_proxy_state.target;
}

bool smp_proxy_is_secondary_ready(void)
{
    return (smp_proxy_state.secondary_conn != NULL) && 
           ble_d2d_smp_client_is_ready();
}

void smp_proxy_clear_phone_conn(struct bt_conn *conn)
{
    if (smp_proxy_state.phone_conn == conn) {
        LOG_INF("Clearing phone connection from SMP proxy");
        smp_proxy_state.phone_conn = NULL;
    }
}
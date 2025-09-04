/**
 * @file ble_d2d_smp_client.cpp
 * @brief D2D SMP Client implementation for secondary device communication
 */

#include "ble_d2d_smp_client.hpp"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <string.h>

LOG_MODULE_REGISTER(d2d_smp_client, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Standard MCUmgr SMP Service UUID
static struct bt_uuid_128 smp_service_uuid = 
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x8D53DC1D, 0x1DB7, 0x4CD3, 0x868B, 0x8A527460AA84));

// SMP characteristic UUID
static struct bt_uuid_128 smp_char_uuid = 
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xDA2E7828, 0xFBCE, 0x4E01, 0xAE9E, 0x261174997C48));

// Client state
static struct {
    struct bt_conn *conn;
    uint16_t smp_handle;
    uint16_t smp_ccc_handle;
    bool discovery_complete;
    d2d_smp_response_cb_t response_callback;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_subscribe_params subscribe_params;
    struct k_sem discovery_sem;
    struct k_work_delayable discovery_timeout;
} d2d_smp_state = {
    .conn = NULL,
    .smp_handle = 0,
    .smp_ccc_handle = 0,
    .discovery_complete = false,
    .response_callback = NULL,
};

// Forward declarations
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            struct bt_gatt_discover_params *params);
static uint8_t notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                          const void *data, uint16_t length);
static void discovery_timeout_handler(struct k_work *work);

// Discovery callback
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_INF("SMP discovery complete - no more attributes");
        k_sem_give(&d2d_smp_state.discovery_sem);
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);

    // Check if this is the SMP service
    if (params->type == BT_GATT_DISCOVER_PRIMARY) {
        if (bt_uuid_cmp(params->uuid, &smp_service_uuid.uuid) == 0) {
            LOG_INF("Found SMP service on secondary device, handle: %u", attr->handle);
            
            // Now discover the SMP characteristic
            d2d_smp_state.discover_params.uuid = &smp_char_uuid.uuid;
            d2d_smp_state.discover_params.start_handle = attr->handle + 1;
            d2d_smp_state.discover_params.end_handle = 0xffff;
            d2d_smp_state.discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            
            int err = bt_gatt_discover(conn, &d2d_smp_state.discover_params);
            if (err) {
                LOG_ERR("Failed to discover SMP characteristic: %d", err);
                k_sem_give(&d2d_smp_state.discovery_sem);
            }
            return BT_GATT_ITER_STOP;
        }
    } else if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        if (bt_uuid_cmp(params->uuid, &smp_char_uuid.uuid) == 0) {
            d2d_smp_state.smp_handle = bt_gatt_attr_value_handle(attr);
            LOG_INF("Found SMP characteristic, handle: %u", d2d_smp_state.smp_handle);
            
            // Now discover the CCC descriptor
            d2d_smp_state.discover_params.uuid = BT_UUID_GATT_CCC;
            d2d_smp_state.discover_params.start_handle = attr->handle + 1;
            d2d_smp_state.discover_params.end_handle = attr->handle + 2;
            d2d_smp_state.discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
            
            int err = bt_gatt_discover(conn, &d2d_smp_state.discover_params);
            if (err) {
                LOG_ERR("Failed to discover CCC: %d", err);
                k_sem_give(&d2d_smp_state.discovery_sem);
            }
            return BT_GATT_ITER_STOP;
        }
    } else if (params->type == BT_GATT_DISCOVER_DESCRIPTOR) {
        d2d_smp_state.smp_ccc_handle = attr->handle;
        LOG_INF("Found SMP CCC descriptor, handle: %u", d2d_smp_state.smp_ccc_handle);
        
        // Subscribe to notifications
        d2d_smp_state.subscribe_params.notify = notify_func;
        d2d_smp_state.subscribe_params.value = BT_GATT_CCC_NOTIFY;
        d2d_smp_state.subscribe_params.value_handle = d2d_smp_state.smp_handle;
        d2d_smp_state.subscribe_params.ccc_handle = d2d_smp_state.smp_ccc_handle;
        
        int err = bt_gatt_subscribe(conn, &d2d_smp_state.subscribe_params);
        if (err) {
            LOG_ERR("Failed to subscribe to SMP notifications: %d", err);
        } else {
            LOG_INF("Subscribed to SMP notifications from secondary device");
            d2d_smp_state.discovery_complete = true;
            // Cancel the timeout since we succeeded
            k_work_cancel_delayable(&d2d_smp_state.discovery_timeout);
        }
        
        k_sem_give(&d2d_smp_state.discovery_sem);
        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_CONTINUE;
}

// Notification callback for SMP responses
static uint8_t notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                          const void *data, uint16_t length)
{
    ARG_UNUSED(params);
    
    // Check if we still have a valid connection
    if (!d2d_smp_state.conn || conn != d2d_smp_state.conn) {
        LOG_WRN("SMP notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    if (!data) {
        LOG_INF("SMP notification subscription removed");
        return BT_GATT_ITER_CONTINUE;
    }

    LOG_DBG("Received SMP notification from secondary: %u bytes", length);
    
    // Forward to callback
    if (d2d_smp_state.response_callback) {
        d2d_smp_state.response_callback((const uint8_t *)data, length);
    }
    
    return BT_GATT_ITER_CONTINUE;
}

// Discovery timeout handler
static void discovery_timeout_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    // Only warn if discovery didn't complete
    if (!d2d_smp_state.discovery_complete) {
        LOG_WRN("D2D SMP discovery timeout - secondary may not have SMP service");
    }
    
    // Signal completion even on timeout
    k_sem_give(&d2d_smp_state.discovery_sem);
}

// Public functions
int ble_d2d_smp_client_init(d2d_smp_response_cb_t response_cb)
{
    LOG_INF("ble_d2d_smp_client_init called with callback: %p", response_cb);
    
    d2d_smp_state.response_callback = response_cb;
    k_sem_init(&d2d_smp_state.discovery_sem, 0, 1);
    k_work_init_delayable(&d2d_smp_state.discovery_timeout, discovery_timeout_handler);
    
    LOG_INF("D2D SMP client initialized successfully");
    return 0;
}

void ble_d2d_smp_client_set_connection(struct bt_conn *conn)
{
    if (d2d_smp_state.conn && d2d_smp_state.conn != conn) {
        // Unsubscribe from old connection
        if (d2d_smp_state.discovery_complete) {
            bt_gatt_unsubscribe(d2d_smp_state.conn, &d2d_smp_state.subscribe_params);
        }
    }
    
    d2d_smp_state.conn = conn;
    d2d_smp_state.discovery_complete = false;
    d2d_smp_state.smp_handle = 0;
    d2d_smp_state.smp_ccc_handle = 0;
    
    if (conn) {
        LOG_INF("D2D SMP client connection set");
        // Don't start discovery immediately - wait for explicit discover call
        // This prevents discovery from starting before D2D RX is ready
    } else {
        LOG_INF("D2D SMP client connection cleared");
    }
}

int ble_d2d_smp_client_send(const uint8_t *data, size_t len)
{
    if (!d2d_smp_state.conn) {
        LOG_ERR("No D2D connection for SMP");
        return -ENOTCONN;
    }
    
    if (!d2d_smp_state.discovery_complete || d2d_smp_state.smp_handle == 0) {
        LOG_ERR("D2D SMP discovery not complete");
        return -EINVAL;
    }
    
    LOG_DBG("Sending SMP frame to secondary: %zu bytes", len);
    
    // For large frames, we may need to split into multiple writes
    size_t offset = 0;
    while (offset < len) {
        size_t chunk_len = MIN(len - offset, bt_gatt_get_mtu(d2d_smp_state.conn) - 3);
        
        int err = bt_gatt_write_without_response(d2d_smp_state.conn,
                                                d2d_smp_state.smp_handle,
                                                data + offset,
                                                chunk_len,
                                                false);
        if (err) {
            LOG_ERR("Failed to send SMP chunk: %d", err);
            return err;
        }
        
        offset += chunk_len;
        
        // Small delay between chunks to avoid congestion
        if (offset < len) {
            k_sleep(K_MSEC(10));
        }
    }
    
    LOG_DBG("SMP frame sent successfully");
    return 0;
}

bool ble_d2d_smp_client_is_ready(void)
{
    return d2d_smp_state.conn && d2d_smp_state.discovery_complete;
}

int ble_d2d_smp_client_discover(struct bt_conn *conn)
{
    if (!conn) {
        return -EINVAL;
    }
    
    LOG_INF("Starting D2D SMP service discovery");
    
    // Reset discovery state
    d2d_smp_state.discovery_complete = false;
    k_sem_reset(&d2d_smp_state.discovery_sem);
    
    // Clear the discover params structure first
    memset(&d2d_smp_state.discover_params, 0, sizeof(d2d_smp_state.discover_params));
    
    // Start discovery
    d2d_smp_state.discover_params.uuid = &smp_service_uuid.uuid;
    d2d_smp_state.discover_params.func = discover_func;
    d2d_smp_state.discover_params.start_handle = 0x0001;
    d2d_smp_state.discover_params.end_handle = 0xffff;
    d2d_smp_state.discover_params.type = BT_GATT_DISCOVER_PRIMARY;
    
    LOG_INF("Calling bt_gatt_discover with params: start=0x%04x, end=0x%04x", 
            d2d_smp_state.discover_params.start_handle,
            d2d_smp_state.discover_params.end_handle);
    
    int err = bt_gatt_discover(conn, &d2d_smp_state.discover_params);
    if (err) {
        LOG_ERR("Failed to start D2D SMP discovery: %d", err);
        return err;
    }
    
    LOG_INF("D2D SMP discovery started - will complete asynchronously");
    
    // Start a timeout to log if discovery takes too long
    k_work_schedule(&d2d_smp_state.discovery_timeout, K_SECONDS(10));
    
    // Return immediately - discovery will complete asynchronously
    return 0;
}
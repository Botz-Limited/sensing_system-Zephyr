#include "ble_d2d_tx.hpp"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "ble_d2d_tx_service.hpp"
#endif

LOG_MODULE_REGISTER(ble_d2d_tx, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

/*
 * D2D TX Module - Used by both Primary and Secondary devices
 * - Secondary: Sends sensor data to primary
 * - Primary: Sends control commands to secondary
 */

// D2D TX Service UUID: 75ad68d6-200c-437d-98b5-061862076c5f
// static struct bt_uuid_128 d2d_tx_service_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x75ad68d6, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// D2D TX Characteristics (increment first byte from information_service.cpp)
// These are currently unused but may be needed for future TX service implementation
// static struct bt_uuid_128 d2d_status_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d6, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
// static struct bt_uuid_128 d2d_foot_sensor_log_available_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d7, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
// static struct bt_uuid_128 d2d_charge_status_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d8, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
// static struct bt_uuid_128 d2d_foot_sensor_req_id_path_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d9, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
// static struct bt_uuid_128 d2d_bhi360_log_available_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68da, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
// static struct bt_uuid_128 d2d_bhi360_req_id_path_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68db, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
// static struct bt_uuid_128 d2d_foot_sensor_samples_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68dc, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
// static struct bt_uuid_128 d2d_bhi360_data1_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68dd, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
// static struct bt_uuid_128 d2d_bhi360_data2_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68de, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
// static struct bt_uuid_128 d2d_bhi360_data3_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68df, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
// static struct bt_uuid_128 d2d_current_time_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e0, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Connection handle
static struct bt_conn *d2d_conn = NULL;

// D2D RX Service and Characteristic UUIDs (must match ble_d2d_rx.cpp)
static struct bt_uuid_128 d2d_rx_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe060ca1f, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// D2D RX Characteristic UUIDs for commands
static struct bt_uuid_128 d2d_rx_set_time_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca1f, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_rx_delete_foot_log_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca82, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_rx_delete_bhi360_log_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca83, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_rx_start_activity_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca84, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_rx_stop_activity_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca85, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_rx_fota_status_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca86, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// Discovered characteristic handles
struct d2d_discovered_handles {
    uint16_t set_time_handle;
    uint16_t delete_foot_log_handle;
    uint16_t delete_bhi360_log_handle;
    uint16_t start_activity_handle;
    uint16_t stop_activity_handle;
    uint16_t fota_status_handle;
    bool discovery_complete;
};

static struct d2d_discovered_handles d2d_handles = {
    .set_time_handle = 0,
    .delete_foot_log_handle = 0,
    .delete_bhi360_log_handle = 0,
    .start_activity_handle = 0,
    .stop_activity_handle = 0,
    .fota_status_handle = 0,
    .discovery_complete = false
};

// Discovery parameters
static struct bt_gatt_discover_params discover_params;
static uint16_t discover_start_handle = 0x0001;
static uint16_t discover_end_handle = 0xffff;

// Forward declarations
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            struct bt_gatt_discover_params *params);
static void continue_discovery(void);

// Discovery state machine states
enum discover_state {
    DISCOVER_SERVICE,
    DISCOVER_SET_TIME_CHAR,
    DISCOVER_DELETE_FOOT_LOG_CHAR,
    DISCOVER_DELETE_BHI360_LOG_CHAR,
    DISCOVER_START_ACTIVITY_CHAR,
    DISCOVER_STOP_ACTIVITY_CHAR,
    DISCOVER_FOTA_STATUS_CHAR,
    DISCOVER_COMPLETE
};

static enum discover_state discovery_state = DISCOVER_SERVICE;

// Service discovery callback
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            struct bt_gatt_discover_params *params)
{
    ARG_UNUSED(conn);
    if (!attr) {
        LOG_DBG("Discovery complete for state %d", discovery_state);
        if (discovery_state != DISCOVER_COMPLETE) {
            // Continue with next discovery step
            continue_discovery();
        }
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);

    switch (discovery_state) {
    case DISCOVER_SERVICE:
        if (bt_uuid_cmp(params->uuid, &d2d_rx_service_uuid.uuid) == 0) {
            LOG_INF("Found D2D RX service, handle: %u", attr->handle);
            discover_start_handle = attr->handle + 1;
            discover_end_handle = 0xffff;
            discovery_state = DISCOVER_SET_TIME_CHAR;
        }
        break;

    case DISCOVER_SET_TIME_CHAR:
        if (bt_uuid_cmp(params->uuid, &d2d_rx_set_time_uuid.uuid) == 0) {
            d2d_handles.set_time_handle = bt_gatt_attr_value_handle(attr);
            LOG_INF("Found set time characteristic, handle: %u", d2d_handles.set_time_handle);
            discovery_state = DISCOVER_DELETE_FOOT_LOG_CHAR;
        }
        break;

    case DISCOVER_DELETE_FOOT_LOG_CHAR:
        if (bt_uuid_cmp(params->uuid, &d2d_rx_delete_foot_log_uuid.uuid) == 0) {
            d2d_handles.delete_foot_log_handle = bt_gatt_attr_value_handle(attr);
            LOG_INF("Found delete foot log characteristic, handle: %u", d2d_handles.delete_foot_log_handle);
            discovery_state = DISCOVER_DELETE_BHI360_LOG_CHAR;
        }
        break;

    case DISCOVER_DELETE_BHI360_LOG_CHAR:
        if (bt_uuid_cmp(params->uuid, &d2d_rx_delete_bhi360_log_uuid.uuid) == 0) {
            d2d_handles.delete_bhi360_log_handle = bt_gatt_attr_value_handle(attr);
            LOG_INF("Found delete BHI360 log characteristic, handle: %u", d2d_handles.delete_bhi360_log_handle);
            discovery_state = DISCOVER_START_ACTIVITY_CHAR;
        }
        break;

    case DISCOVER_START_ACTIVITY_CHAR:
        if (bt_uuid_cmp(params->uuid, &d2d_rx_start_activity_uuid.uuid) == 0) {
            d2d_handles.start_activity_handle = bt_gatt_attr_value_handle(attr);
            LOG_INF("Found start activity characteristic, handle: %u", d2d_handles.start_activity_handle);
            discovery_state = DISCOVER_STOP_ACTIVITY_CHAR;
        }
        break;

    case DISCOVER_STOP_ACTIVITY_CHAR:
        if (bt_uuid_cmp(params->uuid, &d2d_rx_stop_activity_uuid.uuid) == 0) {
            d2d_handles.stop_activity_handle = bt_gatt_attr_value_handle(attr);
            LOG_INF("Found stop activity characteristic, handle: %u", d2d_handles.stop_activity_handle);
            discovery_state = DISCOVER_FOTA_STATUS_CHAR;
        }
        break;

    case DISCOVER_FOTA_STATUS_CHAR:
        if (bt_uuid_cmp(params->uuid, &d2d_rx_fota_status_uuid.uuid) == 0) {
            d2d_handles.fota_status_handle = bt_gatt_attr_value_handle(attr);
            LOG_INF("Found FOTA status characteristic, handle: %u", d2d_handles.fota_status_handle);
            discovery_state = DISCOVER_COMPLETE;
            d2d_handles.discovery_complete = true;
            LOG_INF("D2D service discovery complete!");
        }
        break;

    default:
        break;
    }

    return BT_GATT_ITER_CONTINUE;
}

// Start the discovery process
static void start_discovery(void)
{
    int err;
    
    if (!d2d_conn) {
        LOG_ERR("No D2D connection for discovery");
        return;
    }

    // Reset discovery state
    discovery_state = DISCOVER_SERVICE;
    memset(&d2d_handles, 0, sizeof(d2d_handles));

    // Start with service discovery
    discover_params.uuid = &d2d_rx_service_uuid.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = discover_start_handle;
    discover_params.end_handle = discover_end_handle;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    err = bt_gatt_discover(d2d_conn, &discover_params);
    if (err) {
        LOG_ERR("Discover failed (err %d)", err);
    } else {
        LOG_INF("Started D2D service discovery");
    }
}

// Continue discovery for next characteristic
static void continue_discovery(void)
{
    int err;
    
    if (!d2d_conn) {
        return;
    }

    switch (discovery_state) {
    case DISCOVER_SET_TIME_CHAR:
        discover_params.uuid = &d2d_rx_set_time_uuid.uuid;
        break;
    case DISCOVER_DELETE_FOOT_LOG_CHAR:
        discover_params.uuid = &d2d_rx_delete_foot_log_uuid.uuid;
        break;
    case DISCOVER_DELETE_BHI360_LOG_CHAR:
        discover_params.uuid = &d2d_rx_delete_bhi360_log_uuid.uuid;
        break;
    case DISCOVER_START_ACTIVITY_CHAR:
        discover_params.uuid = &d2d_rx_start_activity_uuid.uuid;
        break;
    case DISCOVER_STOP_ACTIVITY_CHAR:
        discover_params.uuid = &d2d_rx_stop_activity_uuid.uuid;
        break;
    case DISCOVER_FOTA_STATUS_CHAR:
        discover_params.uuid = &d2d_rx_fota_status_uuid.uuid;
        break;
    default:
        LOG_INF("Discovery sequence complete");
        return;
    }

    discover_params.start_handle = discover_start_handle;
    discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    err = bt_gatt_discover(d2d_conn, &discover_params);
    if (err) {
        LOG_ERR("Continue discover failed (err %d)", err);
    }
}

void ble_d2d_tx_init(void) {
    LOG_INF("D2D TX initialized");
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Initialize the GATT service for secondary device
    d2d_tx_service_init();
#endif
}

void ble_d2d_tx_set_connection(struct bt_conn *conn) {
    d2d_conn = conn;
    
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Secondary device: Update the GATT service connection
    d2d_tx_service_set_connection(conn);
#endif
    
    if (conn) {
        LOG_INF("D2D TX connection set");
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Primary device: Start service discovery when connection is established
        k_sleep(K_MSEC(100)); // Small delay to ensure connection is stable
        start_discovery();
#endif
    } else {
        LOG_INF("D2D TX connection cleared");
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Clear discovered handles
        memset(&d2d_handles, 0, sizeof(d2d_handles));
#endif
    }
}


int ble_d2d_tx_send_foot_sensor_data(const foot_samples_t *samples) {
    if (!d2d_conn) return -ENOTCONN;
    
    LOG_DBG("D2D TX: Sending foot sensor data");
    
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Secondary device: Send via GATT notification
    return d2d_tx_notify_foot_sensor_data(samples);
#else
    // Primary device shouldn't call this
    LOG_WRN("Primary device shouldn't send foot sensor data via D2D");
    return -EINVAL;
#endif
}

// Send FOTA completion status from secondary to primary
int ble_d2d_tx_send_fota_complete(void) {
    if (!d2d_conn) {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }
    
    if (!d2d_handles.discovery_complete || d2d_handles.fota_status_handle == 0) {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }
    
    uint8_t status = 1; // 1 = complete
    
    LOG_INF("D2D TX: Sending FOTA complete status to primary (handle: %u)", 
            d2d_handles.fota_status_handle);
    
    int err = bt_gatt_write_without_response(d2d_conn, 
                                            d2d_handles.fota_status_handle,
                                            &status, 
                                            sizeof(status), 
                                            false);
    if (err) {
        LOG_ERR("D2D TX: Failed to send FOTA complete status (err %d)", err);
        return err;
    }
    
    return 0;
}

int ble_d2d_tx_send_foot_sensor_log_available(uint8_t log_id) {
    if (!d2d_conn) return -ENOTCONN;
    LOG_DBG("D2D TX: Sending foot sensor log available: %u", log_id);
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_foot_log_available(log_id);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_foot_sensor_req_id_path(const char *path) {
    if (!d2d_conn) return -ENOTCONN;
    LOG_DBG("D2D TX: Sending foot sensor path: %s", path);
    // Path sending not implemented in GATT service yet
    return 0;
}

int ble_d2d_tx_send_bhi360_log_available(uint8_t log_id) {
    if (!d2d_conn) return -ENOTCONN;
    LOG_DBG("D2D TX: Sending BHI360 log available: %u", log_id);
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_bhi360_log_available(log_id);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_bhi360_req_id_path(const char *path) {
    if (!d2d_conn) return -ENOTCONN;
    LOG_DBG("D2D TX: Sending BHI360 path: %s", path);
    // Path sending not implemented in GATT service yet
    return 0;
}

int ble_d2d_tx_send_bhi360_data1(const bhi360_3d_mapping_t *data) {
    if (!d2d_conn) return -ENOTCONN;
    
    LOG_DBG("D2D TX: Sending BHI360 3D mapping data");
    
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_bhi360_data1(data);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_bhi360_data2(const bhi360_step_count_t *data) {
    if (!d2d_conn) return -ENOTCONN;
    
    LOG_DBG("D2D TX: Sending BHI360 step count data");
    
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_bhi360_data2(data);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_bhi360_data3(const bhi360_linear_accel_t *data) {
    if (!d2d_conn) return -ENOTCONN;
    
    LOG_DBG("D2D TX: Sending BHI360 linear accel data");
    
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_bhi360_data3(data);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_status(uint32_t status) {
    if (!d2d_conn) return -ENOTCONN;
    LOG_DBG("D2D TX: Sending status: 0x%08x", status);
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_status(status);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_charge_status(uint8_t status) {
    if (!d2d_conn) return -ENOTCONN;
    LOG_DBG("D2D TX: Sending charge status: %u", status);
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_charge_status(status);
#else
    return -EINVAL;
#endif
}

// Control command forwarding functions (primary -> secondary)
int ble_d2d_tx_send_set_time_command(uint32_t epoch_time) {
    if (!d2d_conn) {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }
    
    if (!d2d_handles.discovery_complete || d2d_handles.set_time_handle == 0) {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }
    
    LOG_INF("D2D TX: Forwarding set time command - epoch: %u to handle: %u", 
            epoch_time, d2d_handles.set_time_handle);
    
    // Write the epoch time to the secondary device
    int err = bt_gatt_write_without_response(d2d_conn, 
                                            d2d_handles.set_time_handle,
                                            &epoch_time, 
                                            sizeof(epoch_time), 
                                            false);
    if (err) {
        LOG_ERR("D2D TX: Failed to send set time command (err %d)", err);
        return err;
    }
    
    return 0;
}

int ble_d2d_tx_send_delete_foot_log_command(uint8_t log_id) {
    if (!d2d_conn) {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }
    
    if (!d2d_handles.discovery_complete || d2d_handles.delete_foot_log_handle == 0) {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }
    
    LOG_INF("D2D TX: Forwarding delete foot log command - ID: %u to handle: %u", 
            log_id, d2d_handles.delete_foot_log_handle);
    
    int err = bt_gatt_write_without_response(d2d_conn, 
                                            d2d_handles.delete_foot_log_handle,
                                            &log_id, 
                                            sizeof(log_id), 
                                            false);
    if (err) {
        LOG_ERR("D2D TX: Failed to send delete foot log command (err %d)", err);
        return err;
    }
    
    return 0;
}

int ble_d2d_tx_send_delete_bhi360_log_command(uint8_t log_id) {
    if (!d2d_conn) {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }
    
    if (!d2d_handles.discovery_complete || d2d_handles.delete_bhi360_log_handle == 0) {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }
    
    LOG_INF("D2D TX: Forwarding delete BHI360 log command - ID: %u to handle: %u", 
            log_id, d2d_handles.delete_bhi360_log_handle);
    
    int err = bt_gatt_write_without_response(d2d_conn, 
                                            d2d_handles.delete_bhi360_log_handle,
                                            &log_id, 
                                            sizeof(log_id), 
                                            false);
    if (err) {
        LOG_ERR("D2D TX: Failed to send delete BHI360 log command (err %d)", err);
        return err;
    }
    
    return 0;
}

int ble_d2d_tx_send_start_activity_command(uint8_t value) {
    if (!d2d_conn) {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }
    
    if (!d2d_handles.discovery_complete || d2d_handles.start_activity_handle == 0) {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }
    
    LOG_INF("D2D TX: Forwarding start activity command - value: %u to handle: %u", 
            value, d2d_handles.start_activity_handle);
    
    int err = bt_gatt_write_without_response(d2d_conn, 
                                            d2d_handles.start_activity_handle,
                                            &value, 
                                            sizeof(value), 
                                            false);
    if (err) {
        LOG_ERR("D2D TX: Failed to send start activity command (err %d)", err);
        return err;
    }
    
    return 0;
}

int ble_d2d_tx_send_stop_activity_command(uint8_t value) {
    if (!d2d_conn) {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }
    
    if (!d2d_handles.discovery_complete || d2d_handles.stop_activity_handle == 0) {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }
    
    LOG_INF("D2D TX: Forwarding stop activity command - value: %u to handle: %u", 
            value, d2d_handles.stop_activity_handle);
    
    int err = bt_gatt_write_without_response(d2d_conn, 
                                            d2d_handles.stop_activity_handle,
                                            &value, 
                                            sizeof(value), 
                                            false);
    if (err) {
        LOG_ERR("D2D TX: Failed to send stop activity command (err %d)", err);
        return err;
    }
    
    return 0;
}

#include "ble_d2d_tx.hpp"
#include <app_fixed_point.hpp>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "ble_d2d_tx_service.hpp"
#endif

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "ble_d2d_tx_queue.hpp"
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
static struct bt_uuid_128 d2d_rx_trigger_calibration_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca87, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// Weight measurement trigger command UUID (matches d2d_weight_measurement_trigger_uuid in ble_d2d_rx.cpp)
static struct bt_uuid_128 d2d_rx_weight_measurement_trigger_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca8a, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_rx_delete_activity_log_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca88, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_rx_request_device_info_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca89, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_rx_weight_calibration_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca8b, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// Discovered characteristic handles
struct d2d_discovered_handles
{
    uint16_t set_time_handle;
    uint16_t delete_foot_log_handle;
    uint16_t delete_bhi360_log_handle;
    uint16_t delete_activity_log_handle;
    uint16_t start_activity_handle;
    uint16_t stop_activity_handle;
    uint16_t fota_status_handle;
    uint16_t trigger_calibration_handle;
    uint16_t request_device_info_handle;
    uint16_t weight_measurement_trigger_handle;
    uint16_t weight_calibration_handle;
    bool discovery_complete;
};

static struct d2d_discovered_handles d2d_handles = {.set_time_handle = 0,
                                                    .delete_foot_log_handle = 0,
                                                    .delete_bhi360_log_handle = 0,
                                                    .delete_activity_log_handle = 0,
                                                    .start_activity_handle = 0,
                                                    .stop_activity_handle = 0,
                                                    .fota_status_handle = 0,
                                                    .trigger_calibration_handle = 0,
                                                    .request_device_info_handle = 0,
                                                    .weight_measurement_trigger_handle = 0,
                                                    .weight_calibration_handle = 0,
                                                    .discovery_complete = false};

// Discovery parameters
static struct bt_gatt_discover_params discover_params;
static uint16_t discover_start_handle = 0x0001;
static uint16_t discover_end_handle = 0xffff;

// Forward declarations
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params);
static void continue_discovery(void);

// Discovery state machine states
enum discover_state
{
    DISCOVER_SERVICE,
    DISCOVER_SET_TIME_CHAR,
    DISCOVER_DELETE_FOOT_LOG_CHAR,
    DISCOVER_DELETE_BHI360_LOG_CHAR,
    DISCOVER_DELETE_ACTIVITY_LOG_CHAR,
    DISCOVER_START_ACTIVITY_CHAR,
    DISCOVER_STOP_ACTIVITY_CHAR,
    DISCOVER_FOTA_STATUS_CHAR,
    DISCOVER_TRIGGER_CALIBRATION_CHAR,
    DISCOVER_REQUEST_DEVICE_INFO_CHAR,
    DISCOVER_WEIGHT_MEASUREMENT_TRIGGER_CHAR,
    DISCOVER_WEIGHT_CALIBRATION_CHAR,
    DISCOVER_COMPLETE
};

static enum discover_state discovery_state = DISCOVER_SERVICE;

// Service discovery callback
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    ARG_UNUSED(conn);
    if (!attr)
    {
        LOG_DBG("Discovery complete for state %d", discovery_state);
        if (discovery_state != DISCOVER_COMPLETE)
        {
            // Continue with next discovery step
            continue_discovery();
        }
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);

    switch (discovery_state)
    {
        case DISCOVER_SERVICE:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_service_uuid.uuid) == 0)
            {
                LOG_INF("Found D2D RX service, handle: %u", attr->handle);
                discover_start_handle = attr->handle + 1;
                discover_end_handle = 0xffff;
                discovery_state = DISCOVER_SET_TIME_CHAR;
            }
            break;

        case DISCOVER_SET_TIME_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_set_time_uuid.uuid) == 0)
            {
                d2d_handles.set_time_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found set time characteristic, handle: %u", d2d_handles.set_time_handle);
                discovery_state = DISCOVER_DELETE_FOOT_LOG_CHAR;
            }
            break;

        case DISCOVER_DELETE_FOOT_LOG_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_delete_foot_log_uuid.uuid) == 0)
            {
                d2d_handles.delete_foot_log_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found delete foot log characteristic, handle: %u", d2d_handles.delete_foot_log_handle);
                discovery_state = DISCOVER_DELETE_BHI360_LOG_CHAR;
            }
            break;

        case DISCOVER_DELETE_BHI360_LOG_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_delete_bhi360_log_uuid.uuid) == 0)
            {
                d2d_handles.delete_bhi360_log_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found delete BHI360 log characteristic, handle: %u", d2d_handles.delete_bhi360_log_handle);
                discovery_state = DISCOVER_DELETE_ACTIVITY_LOG_CHAR;
            }
            break;

        case DISCOVER_DELETE_ACTIVITY_LOG_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_delete_activity_log_uuid.uuid) == 0)
            {
                d2d_handles.delete_activity_log_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found delete activity log characteristic, handle: %u", d2d_handles.delete_activity_log_handle);
                discovery_state = DISCOVER_START_ACTIVITY_CHAR;
            }
            break;

        case DISCOVER_START_ACTIVITY_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_start_activity_uuid.uuid) == 0)
            {
                d2d_handles.start_activity_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found start activity characteristic, handle: %u", d2d_handles.start_activity_handle);
                discovery_state = DISCOVER_STOP_ACTIVITY_CHAR;
            }
            break;

        case DISCOVER_STOP_ACTIVITY_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_stop_activity_uuid.uuid) == 0)
            {
                d2d_handles.stop_activity_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found stop activity characteristic, handle: %u", d2d_handles.stop_activity_handle);
                discovery_state = DISCOVER_FOTA_STATUS_CHAR;
            }
            break;

        case DISCOVER_FOTA_STATUS_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_fota_status_uuid.uuid) == 0)
            {
                d2d_handles.fota_status_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found FOTA status characteristic, handle: %u", d2d_handles.fota_status_handle);
                discovery_state = DISCOVER_TRIGGER_CALIBRATION_CHAR;
            }
            break;

        case DISCOVER_TRIGGER_CALIBRATION_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_trigger_calibration_uuid.uuid) == 0)
            {
                d2d_handles.trigger_calibration_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found trigger calibration characteristic, handle: %u", d2d_handles.trigger_calibration_handle);
                discovery_state = DISCOVER_REQUEST_DEVICE_INFO_CHAR;
            }
            break;

        case DISCOVER_REQUEST_DEVICE_INFO_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_request_device_info_uuid.uuid) == 0)
            {
                d2d_handles.request_device_info_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found request device info characteristic, handle: %u", d2d_handles.request_device_info_handle);
                discovery_state = DISCOVER_WEIGHT_MEASUREMENT_TRIGGER_CHAR;
            }
            break;

        case DISCOVER_WEIGHT_MEASUREMENT_TRIGGER_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_weight_measurement_trigger_uuid.uuid) == 0)
            {
                d2d_handles.weight_measurement_trigger_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found weight measurement trigger characteristic, handle: %u", d2d_handles.weight_measurement_trigger_handle);
                discovery_state = DISCOVER_WEIGHT_CALIBRATION_CHAR;
            }
            break;

        case DISCOVER_WEIGHT_CALIBRATION_CHAR:
            if (bt_uuid_cmp(params->uuid, &d2d_rx_weight_calibration_uuid.uuid) == 0)
            {
                d2d_handles.weight_calibration_handle = bt_gatt_attr_value_handle(attr);
                LOG_INF("Found weight calibration characteristic, handle: %u", d2d_handles.weight_calibration_handle);
            }
            // Mark discovery as complete even if weight calibration is not found
            // This ensures basic functionality works even without weight calibration
            discovery_state = DISCOVER_COMPLETE;
            d2d_handles.discovery_complete = true;
            LOG_INF("D2D TX discovery complete - found %s characteristics", 
                    d2d_handles.weight_calibration_handle ? "all" : "most");
            
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            // Process any queued commands (only available on primary device)
            ble_d2d_tx_process_queued_commands();
#endif
            break;

        default:
            LOG_WRN("Unknown discovery state: %d", discovery_state);
            break;
    }

    return BT_GATT_ITER_CONTINUE;
}

// Start the discovery process
static void start_discovery(void)
{
    int err;

    if (!d2d_conn)
    {
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
    if (err)
    {
        LOG_ERR("Discover failed (err %d)", err);
    }
    else
    {
        LOG_INF("Started D2D service discovery");
    }
}

// Continue discovery for next characteristic
// Continue discovery for next characteristic
static void continue_discovery(void)
{
    int err;

    if (!d2d_conn)
    {
        return;
    }

    // Check if we've discovered the essential characteristics
    if (discovery_state == DISCOVER_WEIGHT_CALIBRATION_CHAR && 
        d2d_handles.start_activity_handle != 0 &&
        d2d_handles.stop_activity_handle != 0)
    {
        // We have the essential characteristics, mark as complete
        LOG_WRN("Weight calibration characteristic not found, but essential characteristics are available");
        discovery_state = DISCOVER_COMPLETE;
        d2d_handles.discovery_complete = true;
        LOG_INF("D2D TX discovery complete with essential characteristics");
        
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Process any queued commands
        ble_d2d_tx_process_queued_commands();
#endif
        return;
    }

    switch (discovery_state)
    {
        case DISCOVER_SET_TIME_CHAR:
            discover_params.uuid = &d2d_rx_set_time_uuid.uuid;
            break;
        case DISCOVER_DELETE_FOOT_LOG_CHAR:
            discover_params.uuid = &d2d_rx_delete_foot_log_uuid.uuid;
            break;
        case DISCOVER_DELETE_BHI360_LOG_CHAR:
            discover_params.uuid = &d2d_rx_delete_bhi360_log_uuid.uuid;
            break;
        case DISCOVER_DELETE_ACTIVITY_LOG_CHAR:
            discover_params.uuid = &d2d_rx_delete_activity_log_uuid.uuid;
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
        case DISCOVER_TRIGGER_CALIBRATION_CHAR:
            discover_params.uuid = &d2d_rx_trigger_calibration_uuid.uuid;
            break;
        case DISCOVER_REQUEST_DEVICE_INFO_CHAR:
            discover_params.uuid = &d2d_rx_request_device_info_uuid.uuid;
            break;
        case DISCOVER_WEIGHT_MEASUREMENT_TRIGGER_CHAR:
            discover_params.uuid = &d2d_rx_weight_measurement_trigger_uuid.uuid;
            break;
        case DISCOVER_WEIGHT_CALIBRATION_CHAR:
            discover_params.uuid = &d2d_rx_weight_calibration_uuid.uuid;
            break;
        default:
            LOG_INF("Discovery sequence complete");
            return;
    }

    discover_params.start_handle = discover_start_handle;
    discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    err = bt_gatt_discover(d2d_conn, &discover_params);
    if (err)
    {
        LOG_ERR("Continue discover failed (err %d)", err);
    }
}
void ble_d2d_tx_init(void)
{
    LOG_INF("D2D TX initialized");
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Initialize the GATT service for secondary device
    d2d_tx_service_init();
#else
    // Initialize the command queue for primary device
    ble_d2d_tx_queue_init();
#endif
}

void ble_d2d_tx_set_connection(struct bt_conn *conn)
{
    d2d_conn = conn;

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Secondary device: Update the GATT service connection
    LOG_INF("D2D TX: Setting connection for secondary device GATT service");
    d2d_tx_service_set_connection(conn);
#endif

    if (conn)
    {
        LOG_INF("D2D TX connection set (conn=%p)", (void *)conn);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Primary device: Start service discovery when connection is established
        k_sleep(K_MSEC(100)); // Small delay to ensure connection is stable
        start_discovery();
#endif
    }
    else
    {
        LOG_INF("D2D TX connection cleared");
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Clear discovered handles
        memset(&d2d_handles, 0, sizeof(d2d_handles));
#endif
    }
}

int ble_d2d_tx_send_foot_sensor_data(const foot_samples_t *samples)
{
    if (!d2d_conn)
        return -ENOTCONN;

    LOG_INF("D2D TX: Sending foot sensor data");

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
int ble_d2d_tx_send_fota_complete(void)
{
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete || d2d_handles.fota_status_handle == 0)
    {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }

    uint8_t status = 1; // 1 = complete

    LOG_INF("D2D TX: Sending FOTA complete status to primary (handle: %u)", d2d_handles.fota_status_handle);

    int err = bt_gatt_write_without_response(d2d_conn, d2d_handles.fota_status_handle, &status, sizeof(status), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send FOTA complete status (err %d)", err);
        return err;
    }

    return 0;
}

int ble_d2d_tx_send_trigger_bhi360_calibration_command(uint8_t value)
{
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete || d2d_handles.trigger_calibration_handle == 0)
    {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }

    LOG_INF("D2D TX: Forwarding trigger BHI360 calibration command - value: %u to handle: %u", value,
            d2d_handles.trigger_calibration_handle);

    int err =
        bt_gatt_write_without_response(d2d_conn, d2d_handles.trigger_calibration_handle, &value, sizeof(value), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send trigger calibration command (err %d)", err);
        return err;
    }

    return 0;
}

int ble_d2d_tx_send_foot_sensor_log_available(uint8_t log_id)
{
    if (!d2d_conn)
        return -ENOTCONN;
    LOG_DBG("D2D TX: Sending foot sensor log available: %u", log_id);
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_foot_log_available(log_id);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_foot_sensor_req_id_path(const char *path)
{
    if (!d2d_conn)
        return -ENOTCONN;
    LOG_DBG("D2D TX: Sending foot sensor path: %s", path);

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_foot_log_path(path);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_bhi360_log_available(uint8_t log_id)
{
    if (!d2d_conn)
        return -ENOTCONN;
    LOG_DBG("D2D TX: Sending BHI360 log available: %u", log_id);
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_bhi360_log_available(log_id);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_bhi360_req_id_path(const char *path)
{
    if (!d2d_conn)
        return -ENOTCONN;
    LOG_DBG("D2D TX: Sending BHI360 path: %s", path);

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_bhi360_log_path(path);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_bhi360_data1(const bhi360_3d_mapping_t *data)
{
    if (!d2d_conn)
        return -ENOTCONN;

    LOG_DBG("D2D TX: Sending BHI360 3D mapping data");

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_bhi360_data1(data);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_bhi360_data2(const bhi360_step_count_t *data)
{
    if (!d2d_conn)
        return -ENOTCONN;

    LOG_DBG("D2D TX: Sending BHI360 step count data");

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_bhi360_data2(data);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_activity_step_count(const bhi360_step_count_t *data)
{
    if (!d2d_conn)
        return -ENOTCONN;

    LOG_DBG("D2D TX: Sending activity step count data");

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_activity_step_count(data);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_bhi360_data3(const bhi360_linear_accel_t *data)
{
    if (!d2d_conn)
        return -ENOTCONN;

    LOG_DBG("D2D TX: Sending BHI360 linear accel data");

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_bhi360_data3(data);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_status(uint32_t status)
{
    if (!d2d_conn)
        return -ENOTCONN;
    LOG_DBG("D2D TX: Sending status: 0x%08x", status);
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_status(status);
#else
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_charge_status(uint8_t status)
{
    if (!d2d_conn)
        return -ENOTCONN;
    LOG_DBG("D2D TX: Sending charge status: %u", status);
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return d2d_tx_notify_charge_status(status);
#else
    return -EINVAL;
#endif
}

// Control command forwarding functions (primary -> secondary)
int ble_d2d_tx_send_set_time_command(uint32_t epoch_time)
{
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete || d2d_handles.set_time_handle == 0)
    {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }

    LOG_INF("D2D TX: Forwarding set time command - epoch: %u to handle: %u", epoch_time, d2d_handles.set_time_handle);

    // Write the epoch time to the secondary device
    int err =
        bt_gatt_write_without_response(d2d_conn, d2d_handles.set_time_handle, &epoch_time, sizeof(epoch_time), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send set time command (err %d)", err);
        return err;
    }

    return 0;
}

int ble_d2d_tx_send_delete_foot_log_command(uint8_t log_id)
{
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete || d2d_handles.delete_foot_log_handle == 0)
    {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }

    LOG_INF("D2D TX: Forwarding delete foot log command - ID: %u to handle: %u", log_id,
            d2d_handles.delete_foot_log_handle);

    int err =
        bt_gatt_write_without_response(d2d_conn, d2d_handles.delete_foot_log_handle, &log_id, sizeof(log_id), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send delete foot log command (err %d)", err);
        return err;
    }

    return 0;
}

int ble_d2d_tx_send_delete_bhi360_log_command(uint8_t log_id)
{
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete || d2d_handles.delete_bhi360_log_handle == 0)
    {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }

    LOG_INF("D2D TX: Forwarding delete BHI360 log command - ID: %u to handle: %u", log_id,
            d2d_handles.delete_bhi360_log_handle);

    int err =
        bt_gatt_write_without_response(d2d_conn, d2d_handles.delete_bhi360_log_handle, &log_id, sizeof(log_id), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send delete BHI360 log command (err %d)", err);
        return err;
    }

    return 0;
}

int ble_d2d_tx_send_delete_activity_log_command(uint8_t log_id)
{
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete || d2d_handles.delete_activity_log_handle == 0)
    {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }

    LOG_INF("D2D TX: Forwarding delete activity log command - ID: %u to handle: %u", log_id,
            d2d_handles.delete_activity_log_handle);

    int err = bt_gatt_write_without_response(d2d_conn, d2d_handles.delete_activity_log_handle, &log_id, sizeof(log_id),
                                             false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send delete activity log command (err %d)", err);
        return err;
    }

    return 0;
}

int ble_d2d_tx_send_start_activity_command(uint8_t value)
{
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete || d2d_handles.start_activity_handle == 0)
    {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }

    LOG_INF("D2D TX: Forwarding start activity command - value: %u to handle: %u", value,
            d2d_handles.start_activity_handle);

    int err = bt_gatt_write_without_response(d2d_conn, d2d_handles.start_activity_handle, &value, sizeof(value), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send start activity command (err %d)", err);
        return err;
    }

    return 0;
}

int ble_d2d_tx_send_stop_activity_command(uint8_t value)
{
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete || d2d_handles.stop_activity_handle == 0)
    {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }

    LOG_INF("D2D TX: Forwarding stop activity command - value: %u to handle: %u", value,
            d2d_handles.stop_activity_handle);

    int err = bt_gatt_write_without_response(d2d_conn, d2d_handles.stop_activity_handle, &value, sizeof(value), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send stop activity command (err %d)", err);
        return err;
    }

    return 0;
}

int ble_d2d_tx_send_device_info(const device_info_msg_t *info)
{
    if (!d2d_conn)
        return -ENOTCONN;

    LOG_INF("D2D TX: Sending device info");

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Secondary device: Send via GATT notification
    return d2d_tx_notify_device_info(info);
#else
    // Primary device shouldn't call this
    LOG_WRN("Primary device shouldn't send device info via D2D");
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_fota_progress(const fota_progress_msg_t *progress)
{
    if (!d2d_conn)
        return -ENOTCONN;

    LOG_DBG("D2D TX: Sending FOTA progress: active=%d, status=%d, percent=%d%%", progress->is_active, progress->status,
            progress->percent_complete);

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Secondary device: Send via GATT notification
    return d2d_tx_notify_fota_progress(progress);
#else
    // Primary device shouldn't send FOTA progress via D2D
    LOG_WRN("Primary device shouldn't send FOTA progress via D2D");
    return -EINVAL;
#endif
}

int ble_d2d_tx_request_device_info(void)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete || d2d_handles.request_device_info_handle == 0)
    {
        LOG_WRN("D2D TX: Service discovery not complete or handle not found");
        return -EINVAL;
    }

    uint8_t value = 1;
    LOG_INF("D2D TX: Requesting device info from secondary (handle: %u)", d2d_handles.request_device_info_handle);

    int err =
        bt_gatt_write_without_response(d2d_conn, d2d_handles.request_device_info_handle, &value, sizeof(value), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to request device info (err %d)", err);
        return err;
    }

    return 0;
#else
    // Secondary device doesn't request device info
    LOG_WRN("Secondary device shouldn't request device info");
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_weight_measurement(float weight_kg)
{
    if (!d2d_conn)
        return -ENOTCONN;

    LOG_INF("D2D TX: Sending weight measurement: %.1f kg", (double)weight_kg);

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Secondary device: Send via GATT notification
    return d2d_tx_notify_weight_measurement(weight_kg);
#else
    // Primary device shouldn't send weight measurements via D2D
    LOG_WRN("Primary device shouldn't send weight measurement via D2D");
    return -EINVAL;
#endif
}



int ble_d2d_tx_send_weight_calibration_trigger_command(uint8_t value)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete)
    {
        LOG_WRN("D2D TX: Service discovery not complete, queueing weight calibration trigger command");
        return ble_d2d_tx_queue_weight_calibration_trigger_command(value);
    }

    // Weight calibration trigger is sent to the same characteristic as BHI360 calibration
    // This is intentional as both trigger calibration procedures
    if (d2d_handles.trigger_calibration_handle == 0)
    {
        LOG_WRN("D2D TX: Trigger calibration handle not found");
        return -EINVAL;
    }

    LOG_INF("D2D TX: Forwarding weight calibration trigger command - value: %u", value);

    int err = bt_gatt_write_without_response(d2d_conn, d2d_handles.trigger_calibration_handle, &value, sizeof(value), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send weight calibration trigger command (err %d)", err);
        return err;
    }

    return 0;
#else
    // Secondary device doesn't send weight calibration trigger commands
    LOG_WRN("Secondary device shouldn't send weight calibration trigger commands");
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_weight_calibration_with_weight(const weight_calibration_step_t *calib_data)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete)
    {
        LOG_WRN("D2D TX: Service discovery not complete, cannot send weight calibration with weight");
        // TODO: Add queuing support for weight_calibration_step_t
        return -EBUSY;
    }

    // Weight calibration with known weight is sent to the same characteristic as BHI360 calibration
    if (!d2d_handles.trigger_calibration_handle)
    {
        LOG_ERR("D2D TX: Trigger calibration characteristic handle not found");
        return -EINVAL;
    }

    LOG_INF("D2D TX: Forwarding weight calibration with known weight: %.1f kg", 
            (double)calib_data->known_weight_kg);

    int err = bt_gatt_write_without_response(d2d_conn, d2d_handles.trigger_calibration_handle, 
                                            calib_data, sizeof(weight_calibration_step_t), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send weight calibration with weight (err %d)", err);
        return err;
    }

    return 0;
#else
    // Secondary device doesn't send weight calibration commands
    LOG_WRN("Secondary device shouldn't send weight calibration commands");
    return -EINVAL;
#endif
}

int ble_d2d_tx_send_weight_measurement_trigger_command(uint8_t value)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!d2d_conn)
    {
        LOG_WRN("D2D TX: No connection");
        return -ENOTCONN;
    }

    if (!d2d_handles.discovery_complete)
    {
        LOG_WRN("D2D TX: Service discovery not complete, queueing weight measurement trigger command");
        // TODO: Add queuing support for weight measurement trigger
        return -EBUSY;
    }

    // Use the dedicated weight measurement trigger handle
    if (d2d_handles.weight_measurement_trigger_handle == 0)
    {
        LOG_WRN("D2D TX: Weight measurement trigger handle not found");
        return -EINVAL;
    }

    LOG_INF("D2D TX: Forwarding weight measurement trigger command - value: %u to handle: %u", 
            value, d2d_handles.weight_measurement_trigger_handle);

    int err = bt_gatt_write_without_response(d2d_conn, d2d_handles.weight_measurement_trigger_handle, 
                                            &value, sizeof(value), false);
    if (err)
    {
        LOG_ERR("D2D TX: Failed to send weight measurement trigger command (err %d)", err);
        return err;
    }

    return 0;
#else
    // Secondary device doesn't send weight measurement trigger commands
    LOG_WRN("Secondary device shouldn't send weight measurement trigger commands");
    return -EINVAL;
#endif
}

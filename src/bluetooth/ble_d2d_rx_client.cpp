/**
 * @file ble_d2d_rx_client.cpp
 * @brief D2D RX Client for Primary Device
 * 
 * This client discovers and subscribes to the secondary device's D2D TX service
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "ble_d2d_rx_client.hpp"
#include "d2d_data_handler.hpp"
#include <app.hpp>

LOG_MODULE_REGISTER(d2d_rx_client, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Service UUID: 75ad68d6-200c-437d-98b5-061862076c5f (must match secondary's TX service)
static struct bt_uuid_128 d2d_tx_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x75ad68d6, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Characteristic UUIDs (must match secondary's TX service)
static struct bt_uuid_128 d2d_foot_sensor_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68dc, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_bhi360_data1_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68dd, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_bhi360_data2_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68de, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_bhi360_data3_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68df, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_status_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d6, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_charge_status_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d8, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_foot_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d7, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_bhi360_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68da, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Connection handle
static struct bt_conn *secondary_conn = NULL;

// Discovery parameters
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params[8]; // One for each characteristic

// Discovered handles
static uint16_t service_handle = 0;
static uint16_t foot_sensor_handle = 0;
static uint16_t bhi360_data1_handle = 0;
static uint16_t bhi360_data2_handle = 0;
static uint16_t bhi360_data3_handle = 0;
static uint16_t status_handle = 0;
static uint16_t charge_status_handle = 0;
static uint16_t foot_log_handle = 0;
static uint16_t bhi360_log_handle = 0;

// Discovery state
enum discover_state {
    DISCOVER_SERVICE,
    DISCOVER_CHARACTERISTICS,
    DISCOVER_COMPLETE
};

static enum discover_state discovery_state = DISCOVER_SERVICE;
static int characteristics_found = 0;

// Notification handlers
static uint8_t foot_sensor_notify_handler(struct bt_conn *conn,
                                         struct bt_gatt_subscribe_params *params,
                                         const void *data, uint16_t length)
{
    ARG_UNUSED(conn);
    
    if (!data) {
        LOG_WRN("Foot sensor unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(foot_samples_t)) {
        LOG_WRN("Invalid foot sensor data length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    const foot_samples_t *samples = (const foot_samples_t *)data;
    LOG_INF("=== RECEIVED FOOT SENSOR DATA FROM SECONDARY ===");
    LOG_INF("  Channel 0: %u", samples->values[0]);
    LOG_INF("  Channel 1: %u", samples->values[1]);
    LOG_INF("  Channel 2: %u", samples->values[2]);
    LOG_INF("  Channel 3: %u", samples->values[3]);
    LOG_INF("  Channel 4: %u", samples->values[4]);
    LOG_INF("  Channel 5: %u", samples->values[5]);
    LOG_INF("  Channel 6: %u", samples->values[6]);
    LOG_INF("  Channel 7: %u", samples->values[7]);

    // Process the data through the handler
    d2d_data_handler_process_foot_samples(samples);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t bhi360_data1_notify_handler(struct bt_conn *conn,
                                          struct bt_gatt_subscribe_params *params,
                                          const void *data, uint16_t length)
{
    ARG_UNUSED(conn);
    
    if (!data) {
        LOG_WRN("BHI360 data1 unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(bhi360_3d_mapping_t)) {
        LOG_WRN("Invalid BHI360 data1 length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    const bhi360_3d_mapping_t *mapping = (const bhi360_3d_mapping_t *)data;
    LOG_INF("=== RECEIVED BHI360 3D MAPPING FROM SECONDARY ===");
    LOG_INF("  Gyro: X=%.2f, Y=%.2f, Z=%.2f", 
            (double)mapping->gyro_x, (double)mapping->gyro_y, (double)mapping->gyro_z);
    LOG_INF("  Accel: X=%.2f, Y=%.2f, Z=%.2f",
            (double)mapping->accel_x, (double)mapping->accel_y, (double)mapping->accel_z);

    // Process the data through the handler
    d2d_data_handler_process_bhi360_3d_mapping(mapping);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t bhi360_data2_notify_handler(struct bt_conn *conn,
                                          struct bt_gatt_subscribe_params *params,
                                          const void *data, uint16_t length)
{
    ARG_UNUSED(conn);
    
    if (!data) {
        LOG_WRN("BHI360 data2 unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(bhi360_step_count_t)) {
        LOG_WRN("Invalid BHI360 data2 length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    const bhi360_step_count_t *steps = (const bhi360_step_count_t *)data;
    LOG_INF("=== RECEIVED BHI360 STEP COUNT FROM SECONDARY ===");
    LOG_INF("  Steps: %u", steps->step_count);
    LOG_INF("  Activity Duration: %u seconds", steps->activity_duration_s);

    // Process the data through the handler
    d2d_data_handler_process_bhi360_step_count(steps);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t bhi360_data3_notify_handler(struct bt_conn *conn,
                                          struct bt_gatt_subscribe_params *params,
                                          const void *data, uint16_t length)
{
    ARG_UNUSED(conn);
    
    if (!data) {
        LOG_WRN("BHI360 data3 unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(bhi360_linear_accel_t)) {
        LOG_WRN("Invalid BHI360 data3 length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    const bhi360_linear_accel_t *accel = (const bhi360_linear_accel_t *)data;
    LOG_INF("=== RECEIVED BHI360 LINEAR ACCEL FROM SECONDARY ===");
    LOG_INF("  Linear Accel: X=%.2f, Y=%.2f, Z=%.2f",
            (double)accel->x, (double)accel->y, (double)accel->z);

    // Process the data through the handler
    d2d_data_handler_process_bhi360_linear_accel(accel);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t status_notify_handler(struct bt_conn *conn,
                                    struct bt_gatt_subscribe_params *params,
                                    const void *data, uint16_t length)
{
    ARG_UNUSED(conn);
    if (!data) {
        LOG_WRN("Status unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(uint32_t)) {
        LOG_WRN("Invalid status length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    uint32_t status = *(const uint32_t *)data;
    LOG_INF("=== RECEIVED STATUS FROM SECONDARY: 0x%08x ===", status);

    // Process the data through the handler
    d2d_data_handler_process_status(status);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t charge_status_notify_handler(struct bt_conn *conn,
                                           struct bt_gatt_subscribe_params *params,
                                           const void *data, uint16_t length)
{
    ARG_UNUSED(conn);
    if (!data) {
        LOG_WRN("Charge status unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(uint8_t)) {
        LOG_WRN("Invalid charge status length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    uint8_t charge_status = *(const uint8_t *)data;
    LOG_INF("=== RECEIVED CHARGE STATUS FROM SECONDARY: %u ===", charge_status);

    // Process the data through the handler
    d2d_data_handler_process_charge_status(charge_status);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t foot_log_notify_handler(struct bt_conn *conn,
                                      struct bt_gatt_subscribe_params *params,
                                      const void *data, uint16_t length)
{
    ARG_UNUSED(conn);
    if (!data) {
        LOG_WRN("Foot log unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(uint8_t)) {
        LOG_WRN("Invalid foot log length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    uint8_t log_id = *(const uint8_t *)data;
    LOG_INF("=== RECEIVED FOOT LOG AVAILABLE FROM SECONDARY: ID=%u ===", log_id);

    // Process the data through the handler
    d2d_data_handler_process_log_available(log_id, 0); // 0 = foot sensor log type

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t bhi360_log_notify_handler(struct bt_conn *conn,
                                        struct bt_gatt_subscribe_params *params,
                                        const void *data, uint16_t length)
{
    ARG_UNUSED(conn);
    if (!data) {
        LOG_WRN("BHI360 log unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(uint8_t)) {
        LOG_WRN("Invalid BHI360 log length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    uint8_t log_id = *(const uint8_t *)data;
    LOG_INF("=== RECEIVED BHI360 LOG AVAILABLE FROM SECONDARY: ID=%u ===", log_id);

    // Process the data through the handler
    d2d_data_handler_process_log_available(log_id, 1); // 1 = BHI360 log type

    return BT_GATT_ITER_CONTINUE;
}

// Subscribe to a characteristic
static void subscribe_to_characteristic(uint16_t handle, bt_gatt_notify_func_t notify_func, int index)
{
    if (handle == 0) {
        return;
    }

    subscribe_params[index].notify = notify_func;
    subscribe_params[index].value = BT_GATT_CCC_NOTIFY;
    subscribe_params[index].value_handle = handle;
    subscribe_params[index].ccc_handle = 0; // Auto-discover

    int err = bt_gatt_subscribe(secondary_conn, &subscribe_params[index]);
    if (err) {
        LOG_ERR("Failed to subscribe to handle %u: %d", handle, err);
    } else {
        LOG_INF("Subscribed to handle %u", handle);
    }
}

// Discovery callback
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_DBG("Discovery complete for state %d", discovery_state);
        
        if (discovery_state == DISCOVER_CHARACTERISTICS) {
            LOG_INF("D2D TX service discovery complete, found %d characteristics", characteristics_found);
            discovery_state = DISCOVER_COMPLETE;
            
            // Subscribe to all discovered characteristics
            subscribe_to_characteristic(foot_sensor_handle, foot_sensor_notify_handler, 0);
            subscribe_to_characteristic(bhi360_data1_handle, bhi360_data1_notify_handler, 1);
            subscribe_to_characteristic(bhi360_data2_handle, bhi360_data2_notify_handler, 2);
            subscribe_to_characteristic(bhi360_data3_handle, bhi360_data3_notify_handler, 3);
            subscribe_to_characteristic(status_handle, status_notify_handler, 4);
            subscribe_to_characteristic(charge_status_handle, charge_status_notify_handler, 5);
            subscribe_to_characteristic(foot_log_handle, foot_log_notify_handler, 6);
            subscribe_to_characteristic(bhi360_log_handle, bhi360_log_notify_handler, 7);
        }
        
        return BT_GATT_ITER_STOP;
    }

    LOG_DBG("[ATTRIBUTE] handle %u", attr->handle);

    if (discovery_state == DISCOVER_SERVICE) {
        if (bt_uuid_cmp(params->uuid, &d2d_tx_service_uuid.uuid) == 0) {
            LOG_INF("Found D2D TX service on secondary, handle: %u", attr->handle);
            service_handle = attr->handle;
            
            // Now discover characteristics
            discovery_state = DISCOVER_CHARACTERISTICS;
            discover_params.uuid = NULL; // Discover all characteristics
            discover_params.start_handle = attr->handle + 1;
            discover_params.end_handle = 0xffff;
            discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            
            int err = bt_gatt_discover(conn, &discover_params);
            if (err) {
                LOG_ERR("Failed to start characteristic discovery: %d", err);
            }
            
            return BT_GATT_ITER_STOP;
        }
    } else if (discovery_state == DISCOVER_CHARACTERISTICS) {
        struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
        
        if (bt_uuid_cmp(chrc->uuid, &d2d_foot_sensor_uuid.uuid) == 0) {
            foot_sensor_handle = chrc->value_handle;
            LOG_INF("Found foot sensor characteristic, handle: %u", foot_sensor_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_bhi360_data1_uuid.uuid) == 0) {
            bhi360_data1_handle = chrc->value_handle;
            LOG_INF("Found BHI360 data1 characteristic, handle: %u", bhi360_data1_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_bhi360_data2_uuid.uuid) == 0) {
            bhi360_data2_handle = chrc->value_handle;
            LOG_INF("Found BHI360 data2 characteristic, handle: %u", bhi360_data2_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_bhi360_data3_uuid.uuid) == 0) {
            bhi360_data3_handle = chrc->value_handle;
            LOG_INF("Found BHI360 data3 characteristic, handle: %u", bhi360_data3_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_status_uuid.uuid) == 0) {
            status_handle = chrc->value_handle;
            LOG_INF("Found status characteristic, handle: %u", status_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_charge_status_uuid.uuid) == 0) {
            charge_status_handle = chrc->value_handle;
            LOG_INF("Found charge status characteristic, handle: %u", charge_status_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_foot_log_available_uuid.uuid) == 0) {
            foot_log_handle = chrc->value_handle;
            LOG_INF("Found foot log characteristic, handle: %u", foot_log_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_bhi360_log_available_uuid.uuid) == 0) {
            bhi360_log_handle = chrc->value_handle;
            LOG_INF("Found BHI360 log characteristic, handle: %u", bhi360_log_handle);
            characteristics_found++;
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

void d2d_rx_client_start_discovery(struct bt_conn *conn)
{
    if (!conn) {
        LOG_ERR("No connection for D2D RX client discovery");
        return;
    }

    secondary_conn = conn;
    
    // Reset discovery state
    discovery_state = DISCOVER_SERVICE;
    characteristics_found = 0;
    service_handle = 0;
    foot_sensor_handle = 0;
    bhi360_data1_handle = 0;
    bhi360_data2_handle = 0;
    bhi360_data3_handle = 0;
    status_handle = 0;
    charge_status_handle = 0;
    foot_log_handle = 0;
    bhi360_log_handle = 0;

    // Start service discovery
    discover_params.uuid = &d2d_tx_service_uuid.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = 0x0001;
    discover_params.end_handle = 0xffff;
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        LOG_ERR("Failed to start D2D RX client discovery: %d", err);
    } else {
        LOG_INF("Started D2D RX client discovery for secondary device TX service");
    }
}

void d2d_rx_client_disconnected(void)
{
    secondary_conn = NULL;
    discovery_state = DISCOVER_SERVICE;
    
    // Clear all handles
    service_handle = 0;
    foot_sensor_handle = 0;
    bhi360_data1_handle = 0;
    bhi360_data2_handle = 0;
    bhi360_data3_handle = 0;
    status_handle = 0;
    charge_status_handle = 0;
    foot_log_handle = 0;
    bhi360_log_handle = 0;
    
    // Clear subscribe params
    memset(subscribe_params, 0, sizeof(subscribe_params));
    
    LOG_INF("D2D RX client disconnected");
}
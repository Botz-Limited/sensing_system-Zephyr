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
#include <ble_services.hpp>

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "fota_proxy.hpp"
#include "file_proxy.hpp"
#include "ble_d2d_file_transfer.hpp"
#endif

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
static struct bt_gatt_subscribe_params subscribe_params[8] = {}; // One for each characteristic - zero initialized

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
    DISCOVER_DIS,  // Device Information Service discovery
    DISCOVER_DIS_CHARS,  // DIS characteristics discovery
    DISCOVER_COMPLETE
};

static enum discover_state discovery_state = DISCOVER_SERVICE;
static int characteristics_found = 0;

// DIS discovery variables
static struct bt_gatt_read_params read_params;
static char dis_manufacturer[32] = {0};
static char dis_model[16] = {0};
static char dis_serial[16] = {0};
static char dis_hw_rev[16] = {0};
static char dis_fw_rev[16] = {0};
static int dis_chars_read = 0;

// Forward declarations
static void subscribe_to_characteristic(uint16_t handle, bt_gatt_notify_func_t notify_func, int index);
static uint8_t foot_sensor_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t bhi360_data1_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t bhi360_data2_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t bhi360_data3_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t status_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t charge_status_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t foot_log_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t bhi360_log_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);

// Work items for deferred subscription
static void subscribe_work_handler(struct k_work *work);
K_WORK_DEFINE(subscribe_work, subscribe_work_handler);

// Individual work item for each subscription to avoid race conditions
struct subscribe_work_data {
    struct k_work work;
    uint16_t handle;
    bt_gatt_notify_func_t notify_func;
    int index;
};

static struct subscribe_work_data subscribe_work_items[8];

static void individual_subscribe_work_handler(struct k_work *work)
{
    struct subscribe_work_data *data = CONTAINER_OF(work, struct subscribe_work_data, work);
    
    if (!secondary_conn) {
        LOG_ERR("No connection for individual subscription work");
        return;
    }
    
    LOG_INF("Individual subscription work for handle %u, index %d", data->handle, data->index);
    subscribe_to_characteristic(data->handle, data->notify_func, data->index);
}

// Function to perform all subscriptions
static void perform_subscriptions(void)
{
    if (!secondary_conn) {
        LOG_ERR("No secondary connection for subscriptions");
        return;
    }
    
    // Verify connection is still valid
    struct bt_conn_info info;
    int err = bt_conn_get_info(secondary_conn, &info);
    if (err || info.state != BT_CONN_STATE_CONNECTED) {
        LOG_ERR("Connection not valid for subscriptions (err=%d, state=%d)", err, info.state);
        return;
    }
    
    LOG_INF("Starting characteristic subscriptions...");
    LOG_INF("Handles - foot:%u, bhi1:%u, bhi2:%u, bhi3:%u, status:%u, charge:%u, foot_log:%u, bhi_log:%u",
            foot_sensor_handle, bhi360_data1_handle, bhi360_data2_handle, bhi360_data3_handle,
            status_handle, charge_status_handle, foot_log_handle, bhi360_log_handle);
    
    // Clear all subscribe params before starting
    memset(subscribe_params, 0, sizeof(subscribe_params));
    
    // Subscribe to all characteristics
    int work_index = 0;
    
    if (foot_sensor_handle && work_index < 8) {
        LOG_INF("Scheduling subscription to foot sensor (handle %u)", foot_sensor_handle);
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = foot_sensor_handle;
        subscribe_work_items[work_index].notify_func = foot_sensor_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        k_work_submit(&subscribe_work_items[work_index].work);
        work_index++;
        k_msleep(200);
    }
    if (bhi360_data1_handle && work_index < 8) {
        LOG_INF("Scheduling subscription to BHI360 data1 (handle %u)", bhi360_data1_handle);
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = bhi360_data1_handle;
        subscribe_work_items[work_index].notify_func = bhi360_data1_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        k_work_submit(&subscribe_work_items[work_index].work);
        work_index++;
        k_msleep(200);
    }
    if (bhi360_data2_handle && work_index < 8) {
        LOG_INF("Scheduling subscription to BHI360 data2 (handle %u)", bhi360_data2_handle);
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = bhi360_data2_handle;
        subscribe_work_items[work_index].notify_func = bhi360_data2_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        k_work_submit(&subscribe_work_items[work_index].work);
        work_index++;
        k_msleep(200);
    }
    if (bhi360_data3_handle && work_index < 8) {
        LOG_INF("Scheduling subscription to BHI360 data3 (handle %u)", bhi360_data3_handle);
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = bhi360_data3_handle;
        subscribe_work_items[work_index].notify_func = bhi360_data3_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        k_work_submit(&subscribe_work_items[work_index].work);
        work_index++;
        k_msleep(200);
    }
    if (status_handle && work_index < 8) {
        LOG_INF("Scheduling subscription to status (handle %u)", status_handle);
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = status_handle;
        subscribe_work_items[work_index].notify_func = status_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        k_work_submit(&subscribe_work_items[work_index].work);
        work_index++;
        k_msleep(200);
    }
    if (charge_status_handle && work_index < 8) {
        LOG_INF("Scheduling subscription to charge status (handle %u)", charge_status_handle);
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = charge_status_handle;
        subscribe_work_items[work_index].notify_func = charge_status_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        k_work_submit(&subscribe_work_items[work_index].work);
        work_index++;
        k_msleep(200);
    }
    if (foot_log_handle && work_index < 8) {
        LOG_INF("Scheduling subscription to foot log (handle %u)", foot_log_handle);
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = foot_log_handle;
        subscribe_work_items[work_index].notify_func = foot_log_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        k_work_submit(&subscribe_work_items[work_index].work);
        work_index++;
        k_msleep(200);
    }
    if (bhi360_log_handle && work_index < 8) {
        LOG_INF("Scheduling subscription to BHI360 log (handle %u)", bhi360_log_handle);
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = bhi360_log_handle;
        subscribe_work_items[work_index].notify_func = bhi360_log_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        k_work_submit(&subscribe_work_items[work_index].work);
        work_index++;
    }
    LOG_INF("Scheduled %d characteristic subscriptions", work_index);
}

// Work handler for deferred subscription
static void subscribe_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    // Add a longer delay before starting subscriptions to ensure connection is fully stable
    k_msleep(2000); // Increased delay to 2 seconds
    
    // Double-check connection is still valid before performing subscriptions
    if (!secondary_conn) {
        LOG_ERR("Secondary connection lost before subscriptions could start");
        return;
    }
    
    struct bt_conn_info info;
    int err = bt_conn_get_info(secondary_conn, &info);
    if (err || info.state != BT_CONN_STATE_CONNECTED) {
        LOG_ERR("Connection not valid before subscriptions (err=%d, state=%d)", err, info.state);
        return;
    }
    
    // Check if connection parameters have been updated
    LOG_INF("Connection info before subscription:");
    LOG_INF("  Type: %u", info.type);
    LOG_INF("  Role: %u", info.role);
    LOG_INF("  ID: %u", info.id);
    LOG_INF("  Interval: %u", info.le.interval);
    LOG_INF("  Latency: %u", info.le.latency);
    LOG_INF("  Timeout: %u", info.le.timeout);
    
    perform_subscriptions();
}

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

// Alternative subscription method using write to CCC
static void write_ccc_alternative(struct bt_conn *conn, uint16_t handle, uint16_t ccc_handle)
{
    LOG_INF("Attempting alternative CCC write method for handle %u, CCC handle %u", handle, ccc_handle);
    
    uint8_t data[2];
    data[0] = BT_GATT_CCC_NOTIFY & 0xFF;
    data[1] = (BT_GATT_CCC_NOTIFY >> 8) & 0xFF;
    
    struct bt_gatt_write_params write_params;
    memset(&write_params, 0, sizeof(write_params));
    
    write_params.func = [](struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params) {
        ARG_UNUSED(conn);
        ARG_UNUSED(params);
        if (err) {
            LOG_ERR("CCC write failed (err %d)", err);
        } else {
            LOG_INF("CCC write successful");
        }
    };
    write_params.handle = ccc_handle;
    write_params.offset = 0;
    write_params.data = data;
    write_params.length = sizeof(data);
    
    int err = bt_gatt_write(conn, &write_params);
    if (err) {
        LOG_ERR("Failed to initiate CCC write: %d", err);
    }
}

// Subscribe to a characteristic
static void subscribe_to_characteristic(uint16_t handle, bt_gatt_notify_func_t notify_func, int index)
{
    if (handle == 0) {
        LOG_DBG("Skipping subscription for index %d - handle is 0", index);
        return;
    }
    
    // Validate handle is within reasonable range (handle is uint16_t, so > 0xFFFF check is unnecessary)
    if (handle < 0x0001) {
        LOG_ERR("Invalid handle value %u for index %d", handle, index);
        return;
    }

    if (index < 0 || index >= (int)ARRAY_SIZE(subscribe_params)) {
        LOG_ERR("Invalid subscription index %d", index);
        return;
    }

    if (!secondary_conn) {
        LOG_ERR("No secondary connection available for subscription");
        return;
    }

    // Check if connection is still valid
    struct bt_conn_info info;
    int err = bt_conn_get_info(secondary_conn, &info);
    if (err) {
        LOG_ERR("Failed to get connection info: %d", err);
        return;
    }

    if (info.state != BT_CONN_STATE_CONNECTED) {
        LOG_ERR("Connection not in connected state (state=%d), skipping subscription", info.state);
        return;
    }

    // Get pointer to the params structure
    struct bt_gatt_subscribe_params *params = &subscribe_params[index];
    
    // Clear the subscribe params for this index first
    memset(params, 0, sizeof(*params));
    
    // Set up subscription parameters
    params->notify = notify_func;
    params->value = BT_GATT_CCC_NOTIFY;
    params->value_handle = handle;
    // For D2D, the CCC handle is typically value_handle + 1
    // We'll try this instead of auto-discovery which might be causing issues
    params->ccc_handle = handle + 1;
    params->subscribe = [](struct bt_conn *conn, uint8_t err, struct bt_gatt_subscribe_params *params) {
        ARG_UNUSED(conn);
        if (err) {
            LOG_ERR("Subscribe failed (err %d) for handle %u", err, params->value_handle);
        } else {
            LOG_INF("Subscribed successfully to handle %u", params->value_handle);
        }
    };
    
    // Initialize the atomic flags field properly
    atomic_clear(params->flags);
    
#if defined(CONFIG_BT_GATT_AUTO_DISCOVER_CCC)
    params->end_handle = 0;
    params->disc_params = NULL;
#endif

#if defined(CONFIG_BT_SMP)
    params->min_security = BT_SECURITY_L1; // No security required for D2D
#endif

#if defined(CONFIG_BT_EATT)
    params->chan_opt = BT_ATT_CHAN_OPT_NONE;
#endif

    LOG_INF("Subscribing to handle %u at index %d with conn %p", handle, index, (void*)secondary_conn);
    
    // Double-check the connection is still valid right before subscribing
    if (!secondary_conn) {
        LOG_ERR("Connection became NULL before subscription!");
        return;
    }
    
    // Ensure the notify callback is set
    if (!params->notify) {
        LOG_ERR("Notify callback is NULL for index %d!", index);
        return;
    }
    
    // Log all parameters before subscribing
    LOG_INF("About to subscribe with params:");
    LOG_INF("  notify=%p", params->notify);
    LOG_INF("  value_handle=%u", params->value_handle);
    LOG_INF("  ccc_handle=%u", params->ccc_handle);
    LOG_INF("  value=0x%04x", params->value);
    LOG_INF("  conn=%p", (void*)secondary_conn);
    LOG_INF("  Current thread: %p", k_current_get());
    
    // Add a small delay to avoid race conditions
    k_msleep(50);
    
    // Try a simple subscription without retry first
    LOG_INF("Calling bt_gatt_subscribe...");
    err = bt_gatt_subscribe(secondary_conn, params);
    if (err == 0) {
        LOG_INF("Successfully subscribed to handle %u at index %d", handle, index);
        return;
    }
    
    LOG_ERR("Initial subscription failed with error %d", err);
    
    // If subscription failed, try the alternative CCC write method
    if (err == -EINVAL || err == -ENOMEM) {
        LOG_WRN("Trying alternative CCC write method due to subscription failure");
        write_ccc_alternative(secondary_conn, handle, params->ccc_handle);
        // Store the notify handler for when notifications arrive
        params->notify = notify_func;
        return;
    }
    
    // Add retry logic for subscription
    int retry_count = 2;
    while (retry_count > 0) {
        k_msleep(100); // Wait before retry
        
        err = bt_gatt_subscribe(secondary_conn, &subscribe_params[index]);
        if (err == 0) {
            LOG_INF("Successfully subscribed to handle %u at index %d on retry", handle, index);
            break;
        }
        
        LOG_WRN("Failed to subscribe to handle %u at index %d: %d (retries left: %d)", 
                handle, index, err, retry_count - 1);
        
        // Handle specific errors
        if (err == -EACCES || err == -EPERM) {
            LOG_WRN("Subscription failed due to security/permission issue - D2D doesn't require security");
            break; // Don't retry security errors
        } else if (err == -EINVAL) {
            LOG_ERR("Invalid parameters for subscription - check handle and connection");
            break; // Don't retry invalid params
        } else if (err == -ENOTCONN) {
            LOG_ERR("Connection lost during subscription");
            break; // Don't retry if disconnected
        } else if (err == -EALREADY) {
            LOG_WRN("Already subscribed to this characteristic");
            break; // Already subscribed, not an error
        } else if (err == -ENOMEM) {
            LOG_ERR("No memory for subscription");
            k_msleep(100); // Wait a bit before retry
        }
        
        retry_count--;
        if (retry_count > 0) {
            k_msleep(50); // Small delay before retry
        }
    }
    
    if (err != 0 && err != -EALREADY) {
        // Clear the params on failure
        memset(&subscribe_params[index], 0, sizeof(subscribe_params[index]));
    }
}

// Forward declaration for callback
extern void bluetooth_d2d_not_found(struct bt_conn *conn);

// DIS characteristic handles for tracking
static uint16_t dis_manufacturer_handle = 0;
static uint16_t dis_model_handle = 0;
static uint16_t dis_serial_handle = 0;
static uint16_t dis_hw_rev_handle = 0;
static uint16_t dis_fw_rev_handle = 0;

// DIS read callback
static uint8_t dis_read_callback(struct bt_conn *conn, uint8_t err,
                                struct bt_gatt_read_params *params,
                                const void *data, uint16_t length)
{
    ARG_UNUSED(conn);
    
    if (err) {
        LOG_ERR("DIS read failed (err %d)", err);
        return BT_GATT_ITER_STOP;
    }

    if (!data) {
        LOG_WRN("DIS read returned NULL data");
        return BT_GATT_ITER_STOP;
    }

    // Determine which characteristic was read based on the handle
    char *dest = NULL;
    size_t dest_size = 0;
    const char *char_name = "Unknown";
    uint16_t handle = params->single.handle;

    if (handle == dis_manufacturer_handle) {
        dest = dis_manufacturer;
        dest_size = sizeof(dis_manufacturer);
        char_name = "Manufacturer";
    } else if (handle == dis_model_handle) {
        dest = dis_model;
        dest_size = sizeof(dis_model);
        char_name = "Model";
    } else if (handle == dis_serial_handle) {
        dest = dis_serial;
        dest_size = sizeof(dis_serial);
        char_name = "Serial";
    } else if (handle == dis_hw_rev_handle) {
        dest = dis_hw_rev;
        dest_size = sizeof(dis_hw_rev);
        char_name = "HW Rev";
    } else if (handle == dis_fw_rev_handle) {
        dest = dis_fw_rev;
        dest_size = sizeof(dis_fw_rev);
        char_name = "FW Rev";
    }

    if (dest && dest_size > 0) {
        size_t copy_len = MIN(length, dest_size - 1);
        memcpy(dest, data, copy_len);
        dest[copy_len] = '\0';
        LOG_INF("DIS %s: %s", char_name, dest);
    }

    dis_chars_read++;

    // If we've read all DIS characteristics, update the information service
    if (dis_chars_read >= 5) {
        LOG_INF("All DIS characteristics read, updating secondary device info");
        jis_update_secondary_device_info(dis_manufacturer, dis_model, 
                                       dis_serial, dis_hw_rev, dis_fw_rev);
    }

    return BT_GATT_ITER_STOP;
}

// Discovery callback
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_DBG("Discovery complete for state %d", discovery_state);
        
        if (discovery_state == DISCOVER_SERVICE) {
            // No D2D service found - this is not a secondary device
            LOG_INF("No D2D TX service found - this is not a secondary device");
            bluetooth_d2d_not_found(conn);
            return BT_GATT_ITER_STOP;
        }
        else if (discovery_state == DISCOVER_CHARACTERISTICS) {
            LOG_INF("D2D TX service discovery complete, found %d characteristics", characteristics_found);
            
            // Skip DIS discovery for D2D connections since they don't use encryption
            // The secondary device will send its info via D2D messages instead
            LOG_INF("Skipping DIS discovery for D2D connection (no encryption)");
            discovery_state = DISCOVER_COMPLETE;
            
            // Verify connection is still valid before proceeding
            if (!secondary_conn) {
                LOG_ERR("Secondary connection lost during discovery");
                return BT_GATT_ITER_STOP;
            }
            
            struct bt_conn_info info;
            int err = bt_conn_get_info(secondary_conn, &info);
            if (err || info.state != BT_CONN_STATE_CONNECTED) {
                LOG_ERR("Connection not valid after discovery (err=%d, state=%d)", err, info.state);
                return BT_GATT_ITER_STOP;
            }
            
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            // Now we know this is a secondary device, set up D2D connection
            LOG_INF("Secondary device identified and D2D connection established!");
            
            // Notify that this is a confirmed D2D connection first
            extern void bluetooth_d2d_confirmed(struct bt_conn *conn);
            bluetooth_d2d_confirmed(secondary_conn);
            
            // Add delay after confirming D2D to let the connection stabilize
            k_msleep(500);
            
            // Set the secondary connection for FOTA proxy
            fota_proxy_set_secondary_conn(secondary_conn);

            // Set the secondary connection for file proxy
            file_proxy_set_secondary_conn(secondary_conn);

            // Initialize D2D file client for this connection
            ble_d2d_file_client_init(secondary_conn);
            
            // Another delay before subscribing
            k_msleep(500);
#endif
            
            // Schedule subscriptions to run in system workqueue with a delay
            LOG_INF("Scheduling characteristic subscriptions...");
            k_work_submit_to_queue(&k_sys_work_q, &subscribe_work);
        }
        else if (discovery_state == DISCOVER_DIS) {
            LOG_WRN("DIS not found on secondary device");
            discovery_state = DISCOVER_COMPLETE;
        }
        else if (discovery_state == DISCOVER_DIS_CHARS) {
            LOG_INF("DIS characteristic discovery complete");
            discovery_state = DISCOVER_COMPLETE;
            
            // Verify connection is still valid before subscribing
            if (!secondary_conn) {
                LOG_ERR("Secondary connection lost during discovery");
                return BT_GATT_ITER_STOP;
            }
            
            struct bt_conn_info info;
            int err = bt_conn_get_info(secondary_conn, &info);
            if (err || info.state != BT_CONN_STATE_CONNECTED) {
                LOG_ERR("Connection not valid after discovery (err=%d, state=%d)", err, info.state);
                return BT_GATT_ITER_STOP;
            }
            
            // Add a longer delay to ensure connection is stable after discovery
            k_msleep(500); // Increased from 300ms
            
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            // Now we know this is a secondary device, set up D2D connection
            // Note: secondary_conn is already set in d2d_rx_client_start_discovery
            
            LOG_INF("Secondary device identified and D2D connection established!");
            
            // Notify that this is a confirmed D2D connection first
            extern void bluetooth_d2d_confirmed(struct bt_conn *conn);
            bluetooth_d2d_confirmed(conn);
            
            // Add delay after confirming D2D to let the connection stabilize
            k_msleep(500); // Increased from 300ms
            
            // Set the secondary connection for FOTA proxy
            fota_proxy_set_secondary_conn(conn);

            // Set the secondary connection for file proxy
            file_proxy_set_secondary_conn(conn);

            // Initialize D2D file client for this connection
            ble_d2d_file_client_init(conn);
            
            // Another delay before subscribing
            k_msleep(500); // Increased from 200ms
#endif
            
            // Schedule subscriptions to run in system workqueue with a delay
            LOG_INF("Scheduling characteristic subscriptions...");
            k_work_submit_to_queue(&k_sys_work_q, &subscribe_work);
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
    } else if (discovery_state == DISCOVER_DIS) {
        if (bt_uuid_cmp(params->uuid, BT_UUID_DIS) == 0) {
            LOG_INF("Found DIS service, handle: %u", attr->handle);
            
            // Now discover DIS characteristics
            discovery_state = DISCOVER_DIS_CHARS;
            discover_params.uuid = NULL; // Discover all characteristics
            discover_params.start_handle = attr->handle + 1;
            discover_params.end_handle = 0xffff;
            discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            
            int err = bt_gatt_discover(conn, &discover_params);
            if (err) {
                LOG_ERR("Failed to start DIS characteristic discovery: %d", err);
                discovery_state = DISCOVER_COMPLETE;
            }
            
            return BT_GATT_ITER_STOP;
        }
    } else if (discovery_state == DISCOVER_DIS_CHARS) {
        struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
        
        // Check if this is a DIS characteristic we're interested in and store handle
        if (bt_uuid_cmp(chrc->uuid, BT_UUID_DIS_MANUFACTURER_NAME) == 0) {
            dis_manufacturer_handle = chrc->value_handle;
            LOG_DBG("Found DIS manufacturer handle: %u", dis_manufacturer_handle);
        } else if (bt_uuid_cmp(chrc->uuid, BT_UUID_DIS_MODEL_NUMBER) == 0) {
            dis_model_handle = chrc->value_handle;
            LOG_DBG("Found DIS model handle: %u", dis_model_handle);
        } else if (bt_uuid_cmp(chrc->uuid, BT_UUID_DIS_SERIAL_NUMBER) == 0) {
            dis_serial_handle = chrc->value_handle;
            LOG_DBG("Found DIS serial handle: %u", dis_serial_handle);
        } else if (bt_uuid_cmp(chrc->uuid, BT_UUID_DIS_HARDWARE_REVISION) == 0) {
            dis_hw_rev_handle = chrc->value_handle;
            LOG_DBG("Found DIS HW revision handle: %u", dis_hw_rev_handle);
        } else if (bt_uuid_cmp(chrc->uuid, BT_UUID_DIS_FIRMWARE_REVISION) == 0) {
            dis_fw_rev_handle = chrc->value_handle;
            LOG_DBG("Found DIS FW revision handle: %u", dis_fw_rev_handle);
        }
        
        // If this is one of the DIS characteristics we want, read it
        if (bt_uuid_cmp(chrc->uuid, BT_UUID_DIS_MANUFACTURER_NAME) == 0 ||
            bt_uuid_cmp(chrc->uuid, BT_UUID_DIS_MODEL_NUMBER) == 0 ||
            bt_uuid_cmp(chrc->uuid, BT_UUID_DIS_SERIAL_NUMBER) == 0 ||
            bt_uuid_cmp(chrc->uuid, BT_UUID_DIS_HARDWARE_REVISION) == 0 ||
            bt_uuid_cmp(chrc->uuid, BT_UUID_DIS_FIRMWARE_REVISION) == 0) {
            
            // Read this characteristic
            memset(&read_params, 0, sizeof(read_params));
            read_params.func = dis_read_callback;
            read_params.handle_count = 1;
            read_params.single.handle = chrc->value_handle;
            read_params.single.offset = 0;
            
            int err = bt_gatt_read(conn, &read_params);
            if (err) {
                LOG_ERR("Failed to read DIS characteristic: %d", err);
            } else {
                LOG_DBG("Reading DIS characteristic handle %u", chrc->value_handle);
            }
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

    // Cancel any pending subscription work from previous connections
    k_work_cancel(&subscribe_work);
    
    // Clear any existing connection reference
    if (secondary_conn) {
        bt_conn_unref(secondary_conn);
        secondary_conn = NULL;
    }
    
    // Take a reference to the connection
    secondary_conn = bt_conn_ref(conn);
    
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
    
    // Clear all subscribe params to ensure clean state
    memset(subscribe_params, 0, sizeof(subscribe_params));
    
    // Clear DIS data
    memset(dis_manufacturer, 0, sizeof(dis_manufacturer));
    memset(dis_model, 0, sizeof(dis_model));
    memset(dis_serial, 0, sizeof(dis_serial));
    memset(dis_hw_rev, 0, sizeof(dis_hw_rev));
    memset(dis_fw_rev, 0, sizeof(dis_fw_rev));
    dis_chars_read = 0;
    
    // Clear DIS handles
    dis_manufacturer_handle = 0;
    dis_model_handle = 0;
    dis_serial_handle = 0;
    dis_hw_rev_handle = 0;
    dis_fw_rev_handle = 0;

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
    // Cancel any pending subscription work
    k_work_cancel(&subscribe_work);
    
    // Release connection reference if we have one
    if (secondary_conn) {
        bt_conn_unref(secondary_conn);
        secondary_conn = NULL;
    }
    
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
    
    // Clear DIS data
    memset(dis_manufacturer, 0, sizeof(dis_manufacturer));
    memset(dis_model, 0, sizeof(dis_model));
    memset(dis_serial, 0, sizeof(dis_serial));
    memset(dis_hw_rev, 0, sizeof(dis_hw_rev));
    memset(dis_fw_rev, 0, sizeof(dis_fw_rev));
    dis_chars_read = 0;
    
    // Clear DIS handles
    dis_manufacturer_handle = 0;
    dis_model_handle = 0;
    dis_serial_handle = 0;
    dis_hw_rev_handle = 0;
    dis_fw_rev_handle = 0;
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Clear secondary device info in information service
    jis_clear_secondary_device_info();
#endif
    
    LOG_INF("D2D RX client disconnected");
}
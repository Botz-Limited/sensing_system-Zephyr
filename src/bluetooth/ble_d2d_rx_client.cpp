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
#include "smp_proxy.hpp"
#include "ble_d2d_file_transfer.hpp"
#include "ble_d2d_smp_client.hpp"
#include "ble_d2d_tx.hpp"
#endif

LOG_MODULE_REGISTER(d2d_rx_client, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)

// Timing constants for discovery and subscription process
static constexpr uint32_t DISCOVERY_INITIAL_DELAY_MS = 10;      // Delay before starting discovery
static constexpr uint32_t SUBSCRIPTION_INITIAL_DELAY_MS = 20;   // Delay before starting subscriptions
static constexpr uint32_t SUBSCRIPTION_BETWEEN_DELAY_MS = 2;    // Minimal delay between each subscription
static constexpr uint32_t SUBSCRIPTION_FINAL_DELAY_MS = 10;     // Delay after all subscriptions
static constexpr uint32_t CONNECTION_STABILIZE_DELAY_MS = 15;   // Delay for connection stabilization
static constexpr uint32_t RETRY_DELAY_MS = 10;                  // Delay before retrying operations
static constexpr uint32_t RACE_CONDITION_DELAY_MS = 5;          // Delay to avoid race conditions
static constexpr uint32_t RETRY_WAIT_MS = 100;                  // Longer wait before retry on errors

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

static struct bt_uuid_128 d2d_device_info_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e1, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

static struct bt_uuid_128 d2d_fota_progress_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e3, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Path UUIDs
static struct bt_uuid_128 d2d_foot_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d9, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_bhi360_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68db, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_activity_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e4, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_activity_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e5, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Activity step count UUID
static struct bt_uuid_128 d2d_activity_step_count_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e6, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Weight measurement UUID
static struct bt_uuid_128 d2d_weight_measurement_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e7, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// Connection handle
static struct bt_conn *secondary_conn = NULL;

// Discovery parameters
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params[16] = {}; // One for each characteristic - zero initialized

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
static uint16_t device_info_handle = 0;
static uint16_t fota_progress_handle = 0;
static uint16_t foot_log_path_handle = 0;
static uint16_t bhi360_log_path_handle = 0;
static uint16_t activity_log_available_handle = 0;
static uint16_t activity_log_path_handle = 0;
static uint16_t activity_step_count_handle = 0;
static uint16_t weight_measurement_handle = 0;

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
static uint8_t device_info_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t fota_progress_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t foot_log_path_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t bhi360_log_path_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t activity_log_available_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t activity_log_path_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t activity_step_count_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static uint8_t weight_measurement_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);

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

static struct subscribe_work_data subscribe_work_items[16];

// Delayed work for device info retry
static void device_info_retry_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(device_info_retry_work, device_info_retry_work_handler);
static int device_info_retry_count = 0;

// Delayed work for SMP discovery
static void smp_discovery_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(smp_discovery_work, smp_discovery_work_handler);

// Track subscription progress
static atomic_t subscriptions_pending = ATOMIC_INIT(0);
static atomic_t subscriptions_completed = ATOMIC_INIT(0);

static void smp_discovery_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (!secondary_conn) {
        LOG_WRN("No secondary connection for SMP discovery");
        return;
    }
    
    // Check if all subscriptions completed
    int pending = atomic_get(&subscriptions_pending);
    int completed = atomic_get(&subscriptions_completed);
    
    LOG_INF("SMP discovery work: %d subscriptions pending, %d completed", pending, completed);
    
    if (pending > 0 && completed < pending) {
        LOG_WRN("Not all subscriptions completed yet (%d/%d), rescheduling SMP discovery", 
                completed, pending);
        k_work_schedule(&smp_discovery_work, K_SECONDS(1));
        return;
    }
    
    LOG_INF("Starting SMP discovery after D2D RX subscriptions complete");
    
    LOG_INF("CONFIG_PRIMARY_DEVICE is enabled");
    
    if (!secondary_conn) {
        LOG_ERR("secondary_conn is NULL!");
        return;
    }
    
    LOG_INF("Setting SMP client connection...");
    // Make sure SMP client has the correct connection
    ble_d2d_smp_client_set_connection(secondary_conn);
    
    LOG_INF("Calling ble_d2d_smp_client_discover...");
    // Start SMP discovery - this is now non-blocking
    int err = ble_d2d_smp_client_discover(secondary_conn);
    LOG_INF("ble_d2d_smp_client_discover returned: %d", err);
    
    if (err) {
        LOG_ERR("Failed to discover SMP service: %d", err);
    } else {
        LOG_INF("SMP discovery initiated (will complete asynchronously)");
    }
}

static void device_info_retry_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (!secondary_conn || !device_info_handle) {
        return;
    }
    
    LOG_INF("Retrying device info subscription (attempt %d)", device_info_retry_count + 1);
    subscribe_to_characteristic(device_info_handle, device_info_notify_handler, 8);
}

static void individual_subscribe_work_handler(struct k_work *work)
{
    struct subscribe_work_data *data = CONTAINER_OF(work, struct subscribe_work_data, work);
    
    if (!secondary_conn) {
        LOG_ERR("No connection for individual subscription work");
        // Still increment completed count to avoid hanging
        atomic_inc(&subscriptions_completed);
        return;
    }
    
    LOG_INF("Individual subscription work for handle %u, index %d", data->handle, data->index);
    
    // Special handling for device info subscription
    if (data->handle == device_info_handle && data->index == 8) {
        // Add a small extra delay before device info subscription
        k_msleep(5);
    }
    
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
    LOG_INF("Handles - foot:%u, bhi1:%u, bhi2:%u, bhi3:%u, status:%u, charge:%u", 
            foot_sensor_handle, bhi360_data1_handle, bhi360_data2_handle, bhi360_data3_handle,
            status_handle, charge_status_handle);
    LOG_INF("         foot_log:%u, bhi_log:%u, dev_info:%u, fota:%u",
            foot_log_handle, bhi360_log_handle, device_info_handle, fota_progress_handle);
    LOG_INF("         foot_path:%u, bhi_path:%u, act_log:%u, act_path:%u",
            foot_log_path_handle, bhi360_log_path_handle, activity_log_available_handle, activity_log_path_handle);
    
    // Clear all subscribe params before starting
    memset(subscribe_params, 0, sizeof(subscribe_params));
    
    // Reset subscription tracking
    atomic_set(&subscriptions_pending, 0);
    atomic_set(&subscriptions_completed, 0);
    
    // Subscribe to all characteristics
    int work_index = 0;
    
    // Initialize all work items first
    if (foot_sensor_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = foot_sensor_handle;
        subscribe_work_items[work_index].notify_func = foot_sensor_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    if (bhi360_data1_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = bhi360_data1_handle;
        subscribe_work_items[work_index].notify_func = bhi360_data1_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    if (bhi360_data2_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = bhi360_data2_handle;
        subscribe_work_items[work_index].notify_func = bhi360_data2_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    if (bhi360_data3_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = bhi360_data3_handle;
        subscribe_work_items[work_index].notify_func = bhi360_data3_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    if (status_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = status_handle;
        subscribe_work_items[work_index].notify_func = status_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    if (charge_status_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = charge_status_handle;
        subscribe_work_items[work_index].notify_func = charge_status_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    if (foot_log_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = foot_log_handle;
        subscribe_work_items[work_index].notify_func = foot_log_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    if (bhi360_log_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = bhi360_log_handle;
        subscribe_work_items[work_index].notify_func = bhi360_log_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    
    // Add FOTA progress subscription
    if (fota_progress_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = fota_progress_handle;
        subscribe_work_items[work_index].notify_func = fota_progress_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    
    // Add path subscriptions
    if (foot_log_path_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = foot_log_path_handle;
        subscribe_work_items[work_index].notify_func = foot_log_path_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    
    if (bhi360_log_path_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = bhi360_log_path_handle;
        subscribe_work_items[work_index].notify_func = bhi360_log_path_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    
    if (activity_log_available_handle && work_index < 14) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = activity_log_available_handle;
        subscribe_work_items[work_index].notify_func = activity_log_available_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    
    if (activity_log_path_handle && work_index < 15) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = activity_log_path_handle;
        subscribe_work_items[work_index].notify_func = activity_log_path_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    
    if (activity_step_count_handle && work_index < 16) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = activity_step_count_handle;
        subscribe_work_items[work_index].notify_func = activity_step_count_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    
    if (weight_measurement_handle && work_index < 16) {
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = weight_measurement_handle;
        subscribe_work_items[work_index].notify_func = weight_measurement_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    
    // Try device info subscription as the last one with special handling
    if (device_info_handle && work_index < 16) {
        LOG_INF("Attempting device info subscription (handle %u) as last item", device_info_handle);
        k_work_init(&subscribe_work_items[work_index].work, individual_subscribe_work_handler);
        subscribe_work_items[work_index].handle = device_info_handle;
        subscribe_work_items[work_index].notify_func = device_info_notify_handler;
        subscribe_work_items[work_index].index = work_index;
        work_index++;
    }
    
    LOG_INF("Scheduling %d characteristic subscriptions", work_index);
    
    // Set the number of pending subscriptions
    atomic_set(&subscriptions_pending, work_index);
    
    // Now submit all work items with minimal delays
    for (int i = 0; i < work_index; i++) {
        LOG_INF("Scheduling subscription %d/%d (handle %u)", i+1, work_index, subscribe_work_items[i].handle);
        k_work_submit(&subscribe_work_items[i].work);
        if (i < work_index - 1) {
            k_msleep(SUBSCRIPTION_BETWEEN_DELAY_MS);
        }
    }
    LOG_INF("Scheduled %d characteristic subscriptions", work_index);
    
    // Small delay to ensure all subscriptions are processed
    k_msleep(SUBSCRIPTION_FINAL_DELAY_MS);
    LOG_INF("All subscriptions should now be active");
    
    // SMP discovery will be triggered automatically when all subscriptions complete
}

// Work handler for deferred subscription
static void subscribe_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    // Small delay before starting subscriptions to ensure connection is stable
    k_msleep(SUBSCRIPTION_INITIAL_DELAY_MS);
    
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
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("Foot sensor notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }
    
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

    // Process the data through the handler
    d2d_data_handler_process_foot_samples(samples);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t bhi360_data1_notify_handler(struct bt_conn *conn,
                                          struct bt_gatt_subscribe_params *params,
                                          const void *data, uint16_t length)
{
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("BHI360 data1 notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }
    
    if (!data) {
        LOG_WRN("BHI360 data1 unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    // Data is received as fixed-point format
    if (length != sizeof(bhi360_3d_mapping_fixed_t)) {
        LOG_WRN("Invalid BHI360 data1 length: %u (expected %u)", length, sizeof(bhi360_3d_mapping_fixed_t));
        return BT_GATT_ITER_CONTINUE;
    }

    const bhi360_3d_mapping_fixed_t *fixed_data = (const bhi360_3d_mapping_fixed_t *)data;
    
    // Convert fixed-point data back to float
    bhi360_3d_mapping_t mapping;
    // Note: In the current implementation, accel_x/y/z are actually quaternion x/y/z
    mapping.accel_x = fixed16_to_float(fixed_data->quat_x, FixedPoint::QUAT_SCALE);  // Quaternion X scaled by 10000
    mapping.accel_y = fixed16_to_float(fixed_data->quat_y, FixedPoint::QUAT_SCALE);  // Quaternion Y scaled by 10000
    mapping.accel_z = fixed16_to_float(fixed_data->quat_z, FixedPoint::QUAT_SCALE);  // Quaternion Z scaled by 10000
    mapping.quat_w = fixed16_to_float(fixed_data->quat_w, FixedPoint::QUAT_SCALE);   // Quaternion W scaled by 10000
    mapping.gyro_x = fixed16_to_float(fixed_data->gyro_x, FixedPoint::GYRO_SCALE);   // Gyro X scaled by 10000 (rad/s)
    mapping.gyro_y = fixed16_to_float(fixed_data->gyro_y, FixedPoint::GYRO_SCALE);   // Gyro Y scaled by 10000 (rad/s)
    mapping.gyro_z = fixed16_to_float(fixed_data->gyro_z, FixedPoint::GYRO_SCALE);   // Gyro Z scaled by 10000 (rad/s)
    
    LOG_INF("=== RECEIVED BHI360 3D MAPPING FROM SECONDARY ===");
    LOG_INF("  Gyro: X=%.4f, Y=%.4f, Z=%.4f rad/s", 
            (double)mapping.gyro_x, (double)mapping.gyro_y, (double)mapping.gyro_z);
    LOG_INF("  Quaternion: X=%.4f, Y=%.4f, Z=%.4f, W=%.4f",
            (double)mapping.accel_x, (double)mapping.accel_y, (double)mapping.accel_z, (double)mapping.quat_w);
    LOG_INF("  Quaternion Accuracy: %.2f", (double)fixed_data->quat_accuracy / FixedPoint::ACCURACY_SCALE);

    // Process the data through the handler
    d2d_data_handler_process_bhi360_3d_mapping(&mapping);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t bhi360_data2_notify_handler(struct bt_conn *conn,
                                          struct bt_gatt_subscribe_params *params,
                                          const void *data, uint16_t length)
{
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("BHI360 data2 notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }
    
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
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("BHI360 data3 notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }
    
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
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("Status notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }

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
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("Charge status notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }

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
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("Foot log notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }

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
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("BHI360 log notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }

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

static uint8_t device_info_notify_handler(struct bt_conn *conn,
                                         struct bt_gatt_subscribe_params *params,
                                         const void *data, uint16_t length)
{
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("Device info notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }

    if (!data) {
        LOG_WRN("Device info unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(device_info_msg_t)) {
        LOG_WRN("Invalid device info length: %u (expected %u)", length, sizeof(device_info_msg_t));
        return BT_GATT_ITER_CONTINUE;
    }

    const device_info_msg_t *dev_info = (const device_info_msg_t *)data;
    LOG_INF("=== RECEIVED DEVICE INFO FROM SECONDARY ===");
    LOG_INF("  Manufacturer: %s", dev_info->manufacturer);
    LOG_INF("  Model: %s", dev_info->model);
    LOG_INF("  Serial: %s", dev_info->serial);
    LOG_INF("  HW Rev: %s", dev_info->hw_rev);
    LOG_INF("  FW Rev: %s", dev_info->fw_rev);

// Update secondary device info in information service
    jis_update_secondary_device_info(dev_info->manufacturer, dev_info->model,
                                   dev_info->serial, dev_info->hw_rev, 
                                   dev_info->fw_rev);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t fota_progress_notify_handler(struct bt_conn *conn,
                                           struct bt_gatt_subscribe_params *params,
                                           const void *data, uint16_t length)
{
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("FOTA progress notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }

    if (!data) {
        LOG_WRN("FOTA progress unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(fota_progress_msg_t)) {
        LOG_WRN("Invalid FOTA progress length: %u (expected %u)", length, sizeof(fota_progress_msg_t));
        return BT_GATT_ITER_CONTINUE;
    }

    const fota_progress_msg_t *progress = (const fota_progress_msg_t *)data;
    LOG_INF("=== RECEIVED FOTA PROGRESS FROM SECONDARY ===");
    LOG_INF("  Active: %s", progress->is_active ? "YES" : "NO");
    LOG_INF("  Status: %u", progress->status);
    LOG_INF("  Progress: %u%%", progress->percent_complete);
    LOG_INF("  Bytes: %u/%u", progress->bytes_received, progress->total_size);
    if (progress->error_code != 0) {
        LOG_INF("  Error Code: %d", progress->error_code);
    }

// Forward the secondary FOTA progress to the phone via the secondary FOTA progress characteristic
    // Forward declaration to ensure it's available
    extern void jis_secondary_fota_progress_notify(const fota_progress_msg_t* progress);
    jis_secondary_fota_progress_notify(progress);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t foot_log_path_notify_handler(struct bt_conn *conn,
                                           struct bt_gatt_subscribe_params *params,
                                           const void *data, uint16_t length)
{
    ARG_UNUSED(length);
    
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("Foot log path notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }

    if (!data) {
        LOG_WRN("Foot log path unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    const char *path = (const char *)data;
    LOG_INF("=== RECEIVED FOOT LOG PATH FROM SECONDARY: %s ===", path);

    // Process the data through the handler
    d2d_data_handler_process_file_path(0, 0, path); // log_id=0, file_type=0 (foot)

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t bhi360_log_path_notify_handler(struct bt_conn *conn,
                                             struct bt_gatt_subscribe_params *params,
                                             const void *data, uint16_t length)
{
    ARG_UNUSED(length);
    
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("BHI360 log path notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }

    if (!data) {
        LOG_WRN("BHI360 log path unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    const char *path = (const char *)data;
    LOG_INF("=== RECEIVED BHI360 LOG PATH FROM SECONDARY: %s ===", path);

    // Process the data through the handler
    d2d_data_handler_process_file_path(0, 1, path); // log_id=0, file_type=1 (bhi360)

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t activity_log_available_notify_handler(struct bt_conn *conn,
                                                   struct bt_gatt_subscribe_params *params,
                                                   const void *data, uint16_t length)
{
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("Activity log available notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }

    if (!data) {
        LOG_WRN("Activity log available unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(uint8_t)) {
        LOG_WRN("Invalid activity log available length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    uint8_t log_id = *(const uint8_t *)data;
    LOG_INF("=== RECEIVED ACTIVITY LOG AVAILABLE FROM SECONDARY: ID=%u ===", log_id);

    // Process the data through the handler
    d2d_data_handler_process_log_available(log_id, 2); // 2 = activity log type

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t activity_log_path_notify_handler(struct bt_conn *conn,
                                               struct bt_gatt_subscribe_params *params,
                                               const void *data, uint16_t length)
{
    ARG_UNUSED(length);
    
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("Activity log path notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }

    if (!data) {
        LOG_WRN("Activity log path unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    const char *path = (const char *)data;
    LOG_INF("=== RECEIVED ACTIVITY LOG PATH FROM SECONDARY: %s ===", path);

    // Process the data through the handler
    d2d_data_handler_process_file_path(0, 2, path); // log_id=0, file_type=2 (activity)

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t activity_step_count_notify_handler(struct bt_conn *conn,
                                                 struct bt_gatt_subscribe_params *params,
                                                 const void *data, uint16_t length)
{
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("Activity step count notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }
    
    if (!data) {
        LOG_WRN("Activity step count unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(bhi360_step_count_t)) {
        LOG_WRN("Invalid activity step count length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    const bhi360_step_count_t *steps = (const bhi360_step_count_t *)data;
    LOG_INF("=== RECEIVED ACTIVITY STEP COUNT FROM SECONDARY ===");
    LOG_INF("  Activity Steps: %u", steps->step_count);

    // Process the data through the handler
    d2d_data_handler_process_activity_step_count(steps);

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t weight_measurement_notify_handler(struct bt_conn *conn,
                                               struct bt_gatt_subscribe_params *params,
                                               const void *data, uint16_t length)
{
    // Check if we still have a valid secondary connection
    if (!secondary_conn || conn != secondary_conn) {
        LOG_WRN("Weight measurement notification after disconnection, ignoring");
        return BT_GATT_ITER_STOP;
    }
    
    struct bt_conn_info info;
    if (!conn || bt_conn_get_info(conn, &info) != 0) {
        LOG_ERR("Invalid connection in callback");
        return BT_GATT_ITER_STOP;
    }
    
    if (!data) {
        LOG_WRN("Weight measurement unsubscribed");
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    if (length != sizeof(uint16_t)) {
        LOG_WRN("Invalid weight measurement length: %u", length);
        return BT_GATT_ITER_CONTINUE;
    }

    uint16_t weight_kg_x10 = *(const uint16_t *)data;
    float weight_kg = weight_kg_x10 / 10.0f;
    LOG_INF("=== RECEIVED WEIGHT MEASUREMENT FROM SECONDARY ===");
    LOG_INF("  Weight: %.1f kg (raw=%u)", (double)weight_kg, weight_kg_x10);

    // Forward to information service for aggregation with primary weight
    // The primary device should aggregate both feet weights
    extern void jis_secondary_weight_measurement_notify(float weight_kg);
    jis_secondary_weight_measurement_notify(weight_kg);

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
    // Check if we still have a valid secondary connection before subscribing
    if (!secondary_conn) {
        LOG_WRN("No secondary connection, skipping subscription for index %d", index);
        return;
    }
    
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
            
            // If device info subscription failed, schedule a retry
            if (params->value_handle == device_info_handle && device_info_retry_count < 3) {
                device_info_retry_count++;
                LOG_INF("Scheduling device info subscription retry in 500ms");
                k_work_schedule(&device_info_retry_work, K_MSEC(500));
            }
        } else {
            LOG_INF("Subscribed successfully to handle %u", params->value_handle);
            
            // Increment completed subscriptions
            atomic_inc(&subscriptions_completed);
            
            // Log progress
            int completed = atomic_get(&subscriptions_completed);
            int pending = atomic_get(&subscriptions_pending);
            LOG_INF("Subscription progress: %d/%d completed", completed, pending);
            
            // Reset retry count on success
            if (params->value_handle == device_info_handle) {
                device_info_retry_count = 0;
            }
            
// If all subscriptions are complete, request device info and start SMP discovery
            if (completed == pending && pending > 0) {
                LOG_INF("All D2D RX subscriptions complete");
                
                // Request device info from secondary
                // Note: This might fail if D2D TX discovery hasn't completed yet
                // In that case, we'll request it after TX discovery completes
                LOG_INF("Requesting device info from secondary");
                int err = ble_d2d_tx_request_device_info();
                if (err == -EINVAL) {
                    LOG_INF("D2D TX discovery not complete yet, will request device info later");
                } else if (err) {
                    LOG_WRN("Failed to request device info: %d", err);
                }
                
                // Schedule SMP discovery with a small delay to avoid stack issues
                LOG_INF("Scheduling SMP discovery");
                k_work_schedule(&smp_discovery_work, K_MSEC(100));
            }
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
    k_msleep(RACE_CONDITION_DELAY_MS);
    
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
        k_msleep(RETRY_WAIT_MS);
        
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
            k_msleep(RETRY_WAIT_MS);
        }
        
        retry_count--;
        if (retry_count > 0) {
            k_msleep(RETRY_DELAY_MS);
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
            
// Now we know this is a secondary device, set up D2D connection
            LOG_INF("Secondary device identified and D2D connection established!");
            
            // Notify that this is a confirmed D2D connection first
            extern void bluetooth_d2d_confirmed(struct bt_conn *conn);
            bluetooth_d2d_confirmed(secondary_conn);
            
            // Small delay after confirming D2D
            k_msleep(CONNECTION_STABILIZE_DELAY_MS);
            
            // Set the secondary connection for FOTA proxy
            fota_proxy_set_secondary_conn(secondary_conn);

            // Set the secondary connection for file proxy
            file_proxy_set_secondary_conn(secondary_conn);

            // Set the secondary connection for SMP proxy
            smp_proxy_set_secondary_conn(secondary_conn);

            // Initialize D2D file client for this connection
            ble_d2d_file_client_init(secondary_conn);
            
            // Small delay before subscribing
            k_msleep(CONNECTION_STABILIZE_DELAY_MS);
            
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
            
            // Small delay to ensure connection is stable after discovery
            k_msleep(CONNECTION_STABILIZE_DELAY_MS);
            
            // Now we know this is a secondary device, set up D2D connection
            // Note: secondary_conn is already set in d2d_rx_client_start_discovery
            
            LOG_INF("Secondary device identified and D2D connection established!");
            
            // Notify that this is a confirmed D2D connection first
            extern void bluetooth_d2d_confirmed(struct bt_conn *conn);
            bluetooth_d2d_confirmed(conn);
            
            // Small delay after confirming D2D
            k_msleep(CONNECTION_STABILIZE_DELAY_MS);
            
            // Set the secondary connection for FOTA proxy
            fota_proxy_set_secondary_conn(conn);

            // Set the secondary connection for file proxy
            file_proxy_set_secondary_conn(conn);

            // Set the secondary connection for SMP proxy
            smp_proxy_set_secondary_conn(conn);

            // Initialize D2D file client for this connection
            ble_d2d_file_client_init(conn);
            
            // Small delay before subscribing
            k_msleep(CONNECTION_STABILIZE_DELAY_MS);
            
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
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_device_info_uuid.uuid) == 0) {
            device_info_handle = chrc->value_handle;
            LOG_INF("Found device info characteristic, handle: %u", device_info_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_fota_progress_uuid.uuid) == 0) {
            fota_progress_handle = chrc->value_handle;
            LOG_INF("Found FOTA progress characteristic, handle: %u", fota_progress_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_foot_log_path_uuid.uuid) == 0) {
            foot_log_path_handle = chrc->value_handle;
            LOG_INF("Found foot log path characteristic, handle: %u", foot_log_path_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_bhi360_log_path_uuid.uuid) == 0) {
            bhi360_log_path_handle = chrc->value_handle;
            LOG_INF("Found BHI360 log path characteristic, handle: %u", bhi360_log_path_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_activity_log_available_uuid.uuid) == 0) {
            activity_log_available_handle = chrc->value_handle;
            LOG_INF("Found activity log available characteristic, handle: %u", activity_log_available_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_activity_log_path_uuid.uuid) == 0) {
            activity_log_path_handle = chrc->value_handle;
            LOG_INF("Found activity log path characteristic, handle: %u", activity_log_path_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_activity_step_count_uuid.uuid) == 0) {
            activity_step_count_handle = chrc->value_handle;
            LOG_INF("Found activity step count characteristic, handle: %u", activity_step_count_handle);
            characteristics_found++;
        } else if (bt_uuid_cmp(chrc->uuid, &d2d_weight_measurement_uuid.uuid) == 0) {
            weight_measurement_handle = chrc->value_handle;
            LOG_INF("Found weight measurement characteristic, handle: %u", weight_measurement_handle);
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
    device_info_handle = 0;
    fota_progress_handle = 0;
    foot_log_path_handle = 0;
    bhi360_log_path_handle = 0;
    activity_log_available_handle = 0;
    activity_log_path_handle = 0;
    activity_step_count_handle = 0;
    
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
    k_work_cancel_delayable(&device_info_retry_work);
    k_work_cancel_delayable(&smp_discovery_work);
    
    // Reset retry count
    device_info_retry_count = 0;
    
    // Clear subscribe params immediately to prevent callbacks
    memset(subscribe_params, 0, sizeof(subscribe_params));
    
    // Release connection reference if we have one
    // Use atomic operation to ensure thread safety
    struct bt_conn *temp_conn = secondary_conn;
    secondary_conn = NULL;
    
    // Memory barrier to ensure secondary_conn = NULL is visible to all threads
    __sync_synchronize();
    
    if (temp_conn) {
        bt_conn_unref(temp_conn);
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
    device_info_handle = 0;
    fota_progress_handle = 0;
    foot_log_path_handle = 0;
    bhi360_log_path_handle = 0;
    activity_log_available_handle = 0;
    activity_log_path_handle = 0;
    activity_step_count_handle = 0;
    
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
    
// Clear secondary device info in information service
    jis_clear_secondary_device_info();
    
    LOG_INF("D2D RX client disconnected");
}

#else // !CONFIG_PRIMARY_DEVICE

// Stub implementations for secondary device
void d2d_rx_client_start_discovery(struct bt_conn *conn) 
{
    ARG_UNUSED(conn);
}

void d2d_rx_client_disconnected(void) 
{
}

#endif // CONFIG_PRIMARY_DEVICE
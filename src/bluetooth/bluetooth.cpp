/**
 * @file bluetooth.cpp
 * @author
 * @brief
 * @version 2.4.1
 * @date 2025-05-12
 *
 * @copyright Botz Innovation 2025
 *
 */

#define MODULE bluetooth

#include <array>
#include <cctype>
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <string_view>

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "zephyr/bluetooth/services/bas.h"
#include "zephyr/settings/settings.h"

#include "ble_services.hpp"
#include <ble_services.hpp>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/controller.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/types.h>

#include <app.hpp>
#include <app_event_manager.h>
#include <app_version.h>
#include <app_fixed_point.hpp>
#include <ble_connection_state.h>
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/bluetooth_state_event.h>
#include <events/data_event.h>
#include <events/foot_sensor_event.h>
#include <events/motion_sensor_event.h>
#include <status_codes.h>

#include "ble_d2d_file_transfer.hpp"
#include "ble_d2d_rx.hpp"
#include "ble_d2d_tx.hpp"
#include "ble_d2d_tx_service.hpp"
#include "bluetooth_debug.hpp"
#include "d2d_data_handler.hpp"
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "ble_d2d_rx_client.hpp"
#include "ble_d2d_tx_queue.hpp"
#include "ble_recovery_handler.hpp"
#include "ble_seq_manager.hpp"
#include "file_proxy.hpp"
#include "fota_proxy.hpp"
#include "smp_proxy.hpp"
#endif
#include "ble_conn_params.hpp"
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "activity_metrics_service.h"
#endif

// External function declarations
// Step count functions moved to Activity Metrics Service

LOG_MODULE_REGISTER(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL); // NOLINT

void jis_update_d2d_connection_status(bool connected);

extern uint32_t device_status_bitfield;

extern void cs_external_flash_erase_complete_notify(void);
// FWD declarations
static bool app_event_handler(const struct app_event_header *aeh);
static void ble_timer_expiry_function(struct k_timer *timer_id);
static void ble_timer_handler_function(struct k_work *work);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
err_t bt_start_advertising(int err);
#endif
static err_t bt_stop_advertising(void);
static err_t ble_start_hidden(void);

// Constants
#define BLE_ADVERTISING_TIMEOUT_MS 30000                  // 30 seconds advertising timeout
static constexpr uint32_t DISCOVERY_INITIAL_DELAY_MS = 2; // Delay before starting discovery

/********************************** BTH THREAD ********************************/
static constexpr int bluetooth_stack_size = CONFIG_BLUETOOTH_MODULE_STACK_SIZE;
static constexpr int bluetooth_priority = CONFIG_BLUETOOTH_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(bluetooth_stack_area, bluetooth_stack_size);
static struct k_thread bluetooth_thread_data;
static k_tid_t bluetooth_tid;

/********************************** WORK QUEUE ********************************/
// Work queue configuration for asynchronous message processing
static constexpr int bluetooth_workq_stack_size = 8192; // Increased to 8KB to prevent stack overflow
K_THREAD_STACK_DEFINE(bluetooth_workq_stack, bluetooth_workq_stack_size);
static struct k_work_q bluetooth_work_q;

// Work items for different message types
static struct k_work foot_samples_work;
static struct k_work foot_samples_secondary_work; // Separate work item for secondary foot data
static struct k_work bhi360_3d_mapping_work;
static struct k_work bhi360_3d_mapping_secondary_work; // Separate work item for secondary BHI360 3D data
static struct k_work bhi360_linear_accel_work;
static struct k_work bhi360_linear_accel_secondary_work; // Separate work item for secondary linear accel
static struct k_work bhi360_step_count_work;
static struct k_work activity_step_count_work;
static struct k_work command_work;
static struct k_work fota_progress_work;
static struct k_work error_status_work;
static struct k_work new_activity_log_work;
static struct k_work activity_metrics_ble_work;
static struct k_work device_info_work;
static struct k_work weight_measurement_work;
static struct k_work battery_level_primary_work;
static struct k_work battery_level_secondary_work;

// Message buffers for work items (protected by mutex for thread safety)
K_MUTEX_DEFINE(bluetooth_msg_mutex);
static generic_message_t pending_foot_samples_msg;
static generic_message_t pending_foot_samples_secondary_msg; // Separate buffer for secondary foot data
static generic_message_t pending_bhi360_3d_msg;
static generic_message_t pending_bhi360_3d_secondary_msg; // Separate buffer for secondary BHI360 3D data
static generic_message_t pending_bhi360_linear_msg;
static generic_message_t pending_bhi360_linear_secondary_msg; // Separate buffer for secondary linear accel
static generic_message_t pending_bhi360_step_msg;
static generic_message_t pending_activity_step_msg;
static generic_message_t pending_command_msg;
static generic_message_t pending_fota_msg;
static generic_message_t pending_error_msg;
static generic_message_t pending_log_msg;
static generic_message_t pending_metrics_msg;
static generic_message_t pending_device_info_msg;
static generic_message_t pending_weight_msg;
static generic_message_t pending_battery_primary_msg;
static generic_message_t pending_battery_secondary_msg;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
// Static variables for step count aggregation (preserved from original)
static uint32_t primary_step_count = 0;
static uint32_t secondary_step_count = 0;
static uint32_t primary_activity_steps = 0;
static uint32_t secondary_activity_steps = 0;

// Weight measurement state variables (preserved from original)
static float primary_weight_kg = 0;
static float secondary_weight_kg = 0;
static bool waiting_for_secondary = false;
static int64_t weight_request_time = 0;
#endif

// --- D2D Connection Tracking and Callbacks ---
#include "ble_d2d_rx.hpp"
#include "ble_d2d_tx.hpp"

static struct bt_conn *d2d_conn = nullptr;
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static void start_scan(void);
#endif

// Track connections that failed pairing (primary only)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static struct bt_conn *pending_pairing_conn = nullptr;
static bool pairing_failed_for_pending = false;
#endif

// Helper callback for finding active connections
static void find_active_conn_cb(struct bt_conn *conn, void *data)
{
    struct bt_conn **active_conn = (struct bt_conn **)data;

    // If we haven't found a connection yet and this one is valid
    if (*active_conn == NULL && conn != NULL)
    {
        struct bt_conn_info info;
        if (bt_conn_get_info(conn, &info) == 0)
        {
            // Found an active connection, reference it
            *active_conn = bt_conn_ref(conn);
        }
    }
}

// Function to send device info from secondary to primary
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
void send_device_info_to_primary(void)
{
    generic_message_t msg;
    msg.sender = SENDER_BTH;
    msg.type = MSG_TYPE_DEVICE_INFO;

    // Get actual device info from the Device Information Service
    strncpy(msg.data.device_info.manufacturer, dis_get_manufacturer(), sizeof(msg.data.device_info.manufacturer) - 1);
    strncpy(msg.data.device_info.model, dis_get_model(), sizeof(msg.data.device_info.model) - 1);
    strncpy(msg.data.device_info.serial, dis_get_serial(), sizeof(msg.data.device_info.serial) - 1);
    strncpy(msg.data.device_info.hw_rev, dis_get_hw_revision(), sizeof(msg.data.device_info.hw_rev) - 1);
    strncpy(msg.data.device_info.fw_rev, dis_get_fw_revision(), sizeof(msg.data.device_info.fw_rev) - 1);

    // Ensure null termination
    msg.data.device_info.manufacturer[sizeof(msg.data.device_info.manufacturer) - 1] = '\0';
    msg.data.device_info.model[sizeof(msg.data.device_info.model) - 1] = '\0';
    msg.data.device_info.serial[sizeof(msg.data.device_info.serial) - 1] = '\0';
    msg.data.device_info.hw_rev[sizeof(msg.data.device_info.hw_rev) - 1] = '\0';
    msg.data.device_info.fw_rev[sizeof(msg.data.device_info.fw_rev) - 1] = '\0';

    // Send device info directly via D2D TX
    // Note: We need to implement a D2D TX function for device info
    // For now, log that we would send it
    LOG_INF("Device info ready to send via D2D:");
    LOG_INF("  Manufacturer: %s", msg.data.device_info.manufacturer);
    LOG_INF("  Model: %s", msg.data.device_info.model);
    LOG_INF("  Serial: %s", msg.data.device_info.serial);
    LOG_INF("  HW Rev: %s", msg.data.device_info.hw_rev);
    LOG_INF("  FW Rev: %s", msg.data.device_info.fw_rev);

    // Send the device info via D2D TX
    int err = ble_d2d_tx_send_device_info(&msg.data.device_info);
    if (err)
    {
        LOG_ERR("Failed to send device info via D2D: %d", err);
    }
    else
    {
        LOG_INF("Device info sent successfully via D2D");
    }
}
#endif

// Called by D2D RX client when secondary device is confirmed
void bluetooth_d2d_confirmed(struct bt_conn *conn)
{
    d2d_conn = conn;
    LOG_INF("D2D connection confirmed for secondary device");

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Clear the pending pairing failure since this is a valid D2D connection
    if (pending_pairing_conn == conn)
    {
        LOG_INF("Clearing pairing failure for confirmed D2D connection");
        pending_pairing_conn = nullptr;
        pairing_failed_for_pending = false;
    }

    // IMPORTANT: Set the connection in D2D TX module so it can forward commands
    ble_d2d_tx_set_connection(conn);
    LOG_INF("D2D TX connection set for command forwarding");

    // Notify Information Service that D2D is connected
    jis_update_d2d_connection_status(true);

#endif
}

// Called by D2D RX client when no D2D service is found (not a secondary device)
void bluetooth_d2d_not_found(struct bt_conn *conn)
{
    ARG_UNUSED(conn);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    LOG_INF("Device is not a secondary (no D2D TX service found) - this is a "
            "phone connection");

    // This is confirmed to be a phone connection
    ble_set_phone_connection_state(true);

    // Check if this connection had a pairing failure
    if (pending_pairing_conn == conn && pairing_failed_for_pending)
    {
        LOG_INF("Phone connection had pairing failure - this is expected for new devices");
        // Clear the pairing failure state since we now know it's a phone
        pending_pairing_conn = nullptr;
        pairing_failed_for_pending = false;
    }

    // Now we know it's a phone, set security
    int ret = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (ret != 0)
    {
        LOG_ERR("Failed to set security for phone connection (err %d)", ret);
        // For phones, we need security, so disconnect if it fails
        bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
    }
    else
    {
        LOG_INF("Security requested for phone connection");
    }
#endif
}

// For secondary (central) role: variables needed early
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static bt_addr_le_t target_addr;  // Store address of target device
static bool target_found = false; // Flag to indicate we found our target
#endif

static void d2d_connected(struct bt_conn *conn, uint8_t err)
{
    if (!err)
    {
        // Check if this is actually a D2D connection
        char addr_str[BT_ADDR_LE_STR_LEN];
        const bt_addr_le_t *addr = bt_conn_get_dst(conn);
        if (addr)
        {
            bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
        }

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // For primary device, check if the connecting device has secondary device
        // characteristics We'll identify it properly after service discovery For
        // now, we'll set d2d_conn when we discover D2D services
        LOG_INF("Connection established, will determine if D2D after service "
                "discovery");
        jis_update_d2d_connection_status(true);        
#else
        // For secondary device, any connection is to the primary (D2D)
        d2d_conn = conn;
        LOG_INF("Secondary device connected to primary (D2D connection)");

        // Set the D2D TX connection for the secondary device
        ble_d2d_tx_set_connection(conn);
        LOG_INF("D2D TX connection set for secondary device");

        // Schedule device info and status to be sent after primary has subscribed
        // Using a work item to avoid blocking the bluetooth thread
        static struct k_work_delayable device_info_work;
        k_work_init_delayable(&device_info_work, [](struct k_work *work) {
            ARG_UNUSED(work);
            LOG_INF("Sending device info and status after delay");

            // Send device information to primary
            send_device_info_to_primary();

            // Send a status update to indicate secondary is ready
            uint32_t connected_status = STATUS_READY;
            int err = ble_d2d_tx_send_status(connected_status);
            if (err)
            {
                LOG_ERR("Failed to send ready status: %d", err);
            }
            else
            {
                LOG_INF("Sent ready status to primary device");
            }
        });

        // Schedule to run after 3 seconds (time for primary to subscribe)
        k_work_schedule(&device_info_work, K_SECONDS(3));
        LOG_INF("Scheduled device info and status to be sent in 3 seconds");

        // MTU exchange will be handled in the main connected() callback
#endif

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Discovery is now started in the main connected() callback
        LOG_INF("D2D connection callback triggered");
#else
        LOG_INF("Connected to primary device!\n");

        // Stop scanning since we're now connected to the primary device
        int scan_err = bt_le_scan_stop();
        if (scan_err)
        {
            LOG_WRN("Failed to stop scan after connecting to primary: %d", scan_err);
        }
        else
        {
            LOG_INF("Stopped scanning after successful connection to primary device");
        }
#endif
    }
}

static void d2d_disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (conn == d2d_conn)
    {
        d2d_conn = nullptr;
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        LOG_INF("Secondary device disconnected! (reason 0x%02x)\n", reason);

        // Notify Information Service that D2D is disconnected
        jis_update_d2d_connection_status(false);

        // Clear the secondary connection for FOTA proxy
        fota_proxy_set_secondary_conn(NULL);

        // Clear the secondary connection for file proxy
        file_proxy_set_secondary_conn(NULL);

        // Clear the secondary connection for SMP proxy
        smp_proxy_set_secondary_conn(NULL);

        // Clear D2D RX client
        d2d_rx_client_disconnected();

        // Clear D2D TX connection
        ble_d2d_tx_set_connection(NULL);
        LOG_INF("D2D TX connection cleared");

        // Clear D2D file client connection
        ble_d2d_file_client_init(NULL);

        // Clear secondary device info
        jis_clear_secondary_device_info();

        // Restart advertising to allow secondary to reconnect
        LOG_INF("Secondary disconnected - restarting advertising immediately to allow reconnection");
        bt_start_advertising(0);
#else
        LOG_INF("Disconnected from primary device! (reason 0x%02x)\n", reason);

        // Clear D2D TX connection on secondary
        ble_d2d_tx_set_connection(NULL);
        LOG_INF("D2D TX connection cleared on secondary");

        // Reset target found flag and restart scanning after disconnection
        target_found = false;
        start_scan();
#endif
    }
}

// Target service UUID: 5cb36a12-ca69-4d97-89a8-003ffc9ec8cd (changed 002f to 003f for unique pairing)
// This is used by both primary (for advertising) and secondary (for scanning)
static const uint8_t target_service_uuid[16] = {0xcd, 0xc8, 0x9e, 0xfc, 0x3f, 0x00, 0xa8, 0x89,
                                                0x97, 0x4d, 0x69, 0xca, 0x14, 0x6a, 0xb3, 0x5c};

// Getter function to access the target service UUID from other modules
extern "C" const uint8_t* get_target_service_uuid(void)
{
    return target_service_uuid;
}

// For secondary (central) role: scanning and connecting
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)

static struct bt_conn *conn = NULL;

// Use more conservative connection parameters
static const struct bt_conn_le_create_param create_param = {
    .options = BT_CONN_LE_OPT_NONE,
    .interval = BT_GAP_SCAN_FAST_INTERVAL,
    .window = BT_GAP_SCAN_FAST_WINDOW,
    .interval_coded = 0,
    .window_coded = 0,
    .timeout = 0,
};

static const struct bt_le_conn_param conn_param = {
    .interval_min = BT_GAP_INIT_CONN_INT_MIN,
    .interval_max = BT_GAP_INIT_CONN_INT_MAX,
    .latency = 0,
    .timeout = 400,
};

// Helper function to check if advertisement contains our target UUID
static bool adv_data_has_uuid(struct net_buf_simple *ad)
{
    if (!ad)
        return false;

    size_t i = 0;
    while (i + 1 < ad->len)
    {
        uint8_t len = ad->data[i];
        if (len == 0 || (i + len >= ad->len))
            break;

        uint8_t type = ad->data[i + 1];

        // Check for 128-bit UUID
        if (type == BT_DATA_UUID128_ALL || type == BT_DATA_UUID128_SOME)
        {
            // Check all 128-bit UUIDs in this field
            for (size_t j = i + 2; j + 15 < i + 1 + len; j += 16)
            {
                if (memcmp(&ad->data[j], target_service_uuid, 16) == 0)
                {
                    return true;
                }
            }
        }
        i += len + 1;
    }
    return false;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("[SCAN] Device found: %s, RSSI: %d, type: %u, adv data len: %u", addr_str, rssi, type, ad ? ad->len : 0);

    // Debug: print raw advertising data
    if (ad && ad->len > 0)
    {
        LOG_HEXDUMP_INF(ad->data, ad->len, "[SCAN] Raw adv data:");
    }

    // Check if this is our target device by UUID only
    bool has_uuid = ad && adv_data_has_uuid(ad);
    LOG_INF("[SCAN] Has target UUID: %s", has_uuid ? "yes" : "no");

    // If we find a device with matching UUID in scan response, remember it
    if (has_uuid && (type == BT_GAP_ADV_TYPE_SCAN_RSP))
    {
        LOG_INF("[SCAN] Found device with matching UUID in scan response from %s", addr_str);

        // Remember this device
        memcpy(&target_addr, addr, sizeof(bt_addr_le_t));
        target_found = true;
        return;
    }

    // Check if this is a connectable advertisement from our target device
    if (target_found && bt_addr_le_cmp(addr, &target_addr) == 0)
    {
        LOG_INF("[SCAN] Found advertisement from target device %s, type %u", addr_str, type);
        if (type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND)
        {
            LOG_INF("[SCAN] Found connectable advertisement from target device!");

            // Check if we already have a connection object
            if (conn)
            {
                LOG_WRN("[SCAN] Connection already in progress, ignoring");
                return;
            }

            int err = bt_le_scan_stop();
            if (err)
            {
                LOG_ERR("[SCAN] Failed to stop scan: %d", err);
                return;
            }

            // Longer delay to ensure scan is fully stopped
            k_msleep(100);

            // Check if we're already connected to the primary device
            if (d2d_conn != nullptr)
            {
                LOG_WRN("[SCAN] Already connected to primary device, ignoring new "
                        "connection attempt to avoid duplicate connections");
                start_scan();
                return;
            }

            err = bt_conn_le_create(addr, &create_param, &conn_param, &conn);
            if (err)
            {
                if (err == -EINVAL)
                {
                    LOG_ERR("[SCAN] Failed to create connection: -EINVAL (invalid "
                            "argument). Possible address type or "
                            "advertising type mismatch.");
                }
                else if (err == -ENOTSUP)
                {
                    LOG_ERR("[SCAN] Failed to create connection: -ENOTSUP (operation not "
                            "supported). Check if "
                            "advertiser is "
                            "connectable.");
                }
                else if (err == -EIO)
                {
                    LOG_ERR("[SCAN] Failed to create connection: -EIO. Controller "
                            "rejected the request.");
                }
                else if (err == -EBUSY)
                {
                    LOG_ERR("[SCAN] Failed to create connection: -EBUSY. Connection "
                            "already in progress.");
                }
                else
                {
                    LOG_ERR("[SCAN] Failed to create connection: %d", err);
                }
                conn = NULL;
                target_found = false; // Reset target

                // Add delay before restarting scan to avoid rapid retries
                k_msleep(1000);
                start_scan();
            }
            else
            {
                LOG_INF("[SCAN] Connection initiated to %s", addr_str);
                target_found = false; // Reset for next time
            }
            return;
        }
    }

    // For devices that don't match or aren't our target
    if (!has_uuid)
    {
        LOG_DBG("[SCAN] Skipping device: UUID doesn't match");
    }
}

static void start_scan(void)
{
    // Clean up any existing connection reference before starting scan
    if (conn)
    {
        bt_conn_unref(conn);
        conn = NULL;
    }

    struct bt_le_scan_param scan_param = {
        .type = BT_HCI_LE_SCAN_ACTIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = 0x0010,
        .window = 0x0010,
        .timeout = 0,
        .interval_coded = 0,
        .window_coded = 0,
    };

    int err = bt_le_scan_start(&scan_param, device_found);
    if (err)
    {
        LOG_ERR("[SCAN] Failed to start scanning: %d", err);
    }
    else
    {
        LOG_INF("[SCAN] Scanning started successfully");
    }
}
#endif

void bluetooth_process(void * /*unused*/, void * /*unused*/, void * /*unused*/);

static uint8_t ble_status_connected = 0;
// Currently unused - kept for future BLE ID change handling
__attribute__((unused)) static bool ble_id_has_changed = false;

// Global phone connection state (atomic for thread safety)
static atomic_t phone_connected = ATOMIC_INIT(0);

// External queue declarations for clearing
extern struct k_msgq bluetooth_msgq;
extern struct k_msgq realtime_queue;
extern struct k_msgq data_msgq;

// Implementation of global BLE connection state functions
extern "C"
{
bool ble_phone_is_connected(void)
{
    return atomic_get(&phone_connected) == 1;
}

void ble_set_phone_connection_state(bool connected)
{
    atomic_set(&phone_connected, connected ? 1 : 0);
    LOG_INF("Phone connection state changed to: %s", connected ? "CONNECTED" : "DISCONNECTED");
}

void ble_clear_message_queues(void)
{
    generic_message_t dummy_msg;
    int cleared_count = 0;

    // Clear bluetooth queue
    while (k_msgq_get(&bluetooth_msgq, &dummy_msg, K_NO_WAIT) == 0)
    {
        cleared_count++;
    }
    if (cleared_count > 0)
    {
        LOG_INF("Cleared %d messages from bluetooth queue", cleared_count);
    }
}
}

struct check_bond_data
{
    const bt_addr_le_t *addr;
    bool found;
};

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static bt_le_adv_param bt_le_adv_conn_name = {
    .id = 0,
    .sid = 0,
    .secondary_max_skip = 0,
    .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    .peer = NULL,
};
#endif // CONFIG_PRIMARY_DEVICE

static const bt_le_adv_param *bt_le_adv_conn = BT_LE_ADV_CONN_FAST_1;

struct bt_conn *local_conn = NULL;

// Init timer
K_TIMER_DEFINE(ble_timer, ble_timer_expiry_function, NULL);

K_WORK_DEFINE(ble_handler, ble_timer_handler_function);

// Weight measurement timeout work
static void weight_measurement_timeout_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(weight_measurement_timeout_work, weight_measurement_timeout_work_handler);

/* Advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL), BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
};

/* Scan response data - put the 128-bit UUID here to avoid advertising packet
 * size issues */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0x5cb36a14, 0xca69, 0x4d97, 0x89a8, 0x003ffc9ec8cd)),
};

/**
 * @brief Callback function called when pairing is completed.
 *
 * This function is called when the pairing process is completed for a Bluetooth
 * connection. It logs an informational message indicating whether the pairing
 * was successful or not. Additionally, it stops the BLE timer and turns off the
 * Bluetooth LED.
 *
 * @param conn Pointer to the Bluetooth connection structure. This parameter is
 * not used in the implementation of this function.
 * @param bonded Boolean flag indicating whether the device is bonded after
 * pairing. 'true' indicates bonded, 'false' indicates not bonded.
 */
static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    local_conn = conn;
    LOG_INF("Pairing Complete (%sbonded)", bonded ? "" : "not ");
    k_timer_stop(&ble_timer);
}

/**
 * @brief Callback function called when pairing fails.
 *
 * This function is called when the pairing process fails for a Bluetooth
 * connection. It logs an error message indicating the reason for the pairing
 * failure. Additionally, it disconnects the Bluetooth connection and stops the
 * BLE timer. It also turns off the Bluetooth LED.
 *
 * @param conn Pointer to the Bluetooth connection structure associated with the
 * failed pairing.
 * @param reason An enum representing the reason for the pairing failure.
 *               Possible values include authentication failures and other
 * security-related errors.
 */
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    LOG_ERR("Pairing Failed (%d).", reason);

    // Check if this is a D2D connection that doesn't need pairing
    [[maybe_unused]] bool is_d2d = false;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // On primary, we need to determine if this is a D2D or phone connection
    // Mark this connection as having failed pairing but don't disconnect yet
    LOG_INF("Pairing failed - will determine connection type after service discovery");
    pending_pairing_conn = conn;
    pairing_failed_for_pending = true;
    // Don't disconnect here - let discovery complete first to identify device type
#else
    // On secondary, any connection is to the primary (D2D)
    // D2D connections don't require pairing/bonding
    is_d2d = true;
    LOG_INF("Ignoring pairing failure for D2D connection (expected behavior)");
    // Don't disconnect - D2D connections work without pairing
#endif

    k_timer_stop(&ble_timer);
}

/**
 * @brief Bluetooth connection authentication information callback structure.
 *
 * This structure defines callback functions for handling pairing events during
 * Bluetooth connection authentication. It includes callbacks for pairing
 * completion and pairing failure.
 */
static struct bt_conn_auth_info_cb bt_conn_auth_info = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed,
    .bond_deleted{},
    .node{},
};

static void check_bond(const struct bt_bond_info *info, void *data)
{
    struct check_bond_data *bond_data = static_cast<check_bond_data *>(data);

    if (bt_addr_le_cmp(&info->addr, bond_data->addr) == 0)
    {
        bond_data->found = true;
    }
}

static bool is_connection_bonded(struct bt_conn *conn)
{
    const bt_addr_le_t *addr = bt_conn_get_dst(conn);
    if (!addr)
    {
        LOG_ERR("Failed to get connection address");
        return false;
    }

    struct check_bond_data bond_data = {
        .addr = addr,
        .found = false,
    };

    bt_foreach_bond(BT_ID_DEFAULT, check_bond, &bond_data);

    return bond_data.found;
}

/**
 * @brief Callback function called when a Bluetooth connection is established.
 *
 * This function is called when a Bluetooth connection is successfully
 * established or when a connection attempt fails. It logs an error message if
 * the connection attempt fails with a non-zero error code. If the connection is
 * successful, it logs an informational message indicating the successful
 * connection, sets the security level of the connection to L2 (link layer), and
 * starts a timer for periodic BLE updates. Finally, it turns off the Bluetooth
 * LED.
 *
 * @param conn Pointer to the Bluetooth connection structure associated with the
 * connection attempt.
 * @param err Error code indicating the result of the connection attempt.
 * '0' indicates successful connection, non-zero indicates failure.
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err != 0)
    {
        LOG_ERR("Connection failed (err 0x%02x)", err);
        char addr_str[BT_ADDR_LE_STR_LEN];
        const bt_addr_le_t *addr = bt_conn_get_dst(conn);
        if (addr)
        {
            bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
            LOG_ERR("Failed to connect to: %s", addr_str);
        }
        else
        {
            LOG_ERR("Failed to connect: unable to get address");
        }

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // If we're in central mode and connection failed, restart scanning
        start_scan();
#endif
    }
    else
    {
        // Request MTU exchange to avoid small MTU issues
        static struct bt_gatt_exchange_params exchange_params;
        exchange_params.func = [](struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params) {
            ARG_UNUSED(params);
            if (!err)
            {
                uint16_t mtu = bt_gatt_get_mtu(conn);
                LOG_INF("MTU exchange successful: %u", mtu);
            }
            else
            {
                LOG_WRN("MTU exchange failed: %d", err);
            }
        };

        int mtu_ret = bt_gatt_exchange_mtu(conn, &exchange_params);
        if (mtu_ret)
        {
            LOG_WRN("Failed to initiate MTU exchange: %d", mtu_ret);
        }
        else
        {
            LOG_DBG("MTU exchange initiated");
        }

        // Only send a connected event if the connection is already bonded
        bool bonded = is_connection_bonded(conn);
        if (bonded)
        {
        }

        LOG_INF("Connected");

        ble_status_connected = 1;
        ble_set_phone_connection_state(1);

        // Clear any stale messages in BLE queues on reconnection
        ble_clear_message_queues();
        LOG_INF("BLE queues cleared on connection");

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Set phone connection state (only for non-D2D connections)
        // We'll determine this after discovery completes
        // For now, assume it might be a phone until proven otherwise

        // Update Activity Metrics Service with connection (primary only)
        ams_set_connection(conn);
#endif

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Check if this is a reconnection and start recovery if needed
        BleSequenceManager::getInstance().onReconnect();
        if (BleSequenceManager::getInstance().isInRecovery())
        {
            LOG_INF("Starting BLE packet recovery after reconnection");
            //  BleRecoveryHandler::getInstance().startRecovery();
        }
#endif

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Debug information service status
        bt_debug_info_service_status(conn);

        // Log that services are available
        LOG_INF("=== BLE SERVICES STATUS ===");
        LOG_INF("Information Service registered with UUID: "
                "0c372eaa-27eb-437e-bef4-775aefaf3c97");
        LOG_INF("Device Info Service (DIS) registered");
        LOG_INF("Control Service registered");
        LOG_INF("All primary device services are available for phone connections");
        LOG_INF("===========================");

        // For primary device, start discovery only if no D2D connection exists
        // Don't disrupt existing working D2D connections when phone connects
        if (d2d_conn == nullptr)
        {
            LOG_INF("PRIMARY: No D2D connection exists - starting discovery to identify device type");

            // Start discovery after a small delay to ensure connection is stable
            k_sleep(K_MSEC(DISCOVERY_INITIAL_DELAY_MS));
            LOG_INF("PRIMARY: Calling d2d_rx_client_start_discovery");
            d2d_rx_client_start_discovery(conn);
        }
        else
        {
            LOG_INF(
                "PRIMARY: D2D connection already established - skipping discovery for this connection (likely phone)");

            // This is likely a phone connection since we already have D2D
            // Set security immediately for phone connections
            int ret = bt_conn_set_security(conn, BT_SECURITY_L2);
            if (ret != 0)
            {
                LOG_ERR("Failed to set security for phone connection (err %d)", ret);
                bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
            }
            else
            {
                LOG_INF("Security requested for phone connection (D2D already established)");
            }
        }
#else
        // Secondary device doesn't set security for connections to primary
        LOG_INF("Secondary device: not setting security for connection to primary");
#endif

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Count active connections to determine advertising behavior
        int active_count = 0;

        // Helper to count connections
        auto count_conn_cb = [](struct bt_conn *conn, void *data) {
            int *count = (int *)data;
            struct bt_conn_info info;
            if (bt_conn_get_info(conn, &info) == 0 && info.state == BT_CONN_STATE_CONNECTED)
            {
                (*count)++;
            }
        };

        bt_conn_foreach(BT_CONN_TYPE_LE, count_conn_cb, &active_count);

        LOG_INF("Active connections after new connection: %d/%d", active_count, CONFIG_BT_MAX_CONN);

        // Continue advertising unless we're at maximum capacity
        // This allows both secondary and phone to connect simultaneously
        if (active_count >= CONFIG_BT_MAX_CONN)
        {
            LOG_INF("Maximum connections reached (%d/%d), stopping advertising", active_count, CONFIG_BT_MAX_CONN);
            int ret = bt_le_adv_stop();
            if (ret != 0 && ret != -EALREADY)
            {
                LOG_ERR("Failed to stop advertising (err 0x%02x)", ret);
            }
        }
        else
        {
            LOG_INF("Connection slots available (%d/%d), restarting advertising for additional connections",
                    active_count, CONFIG_BT_MAX_CONN);
            // Restart advertising to allow additional connections
            // BLE advertising stops when a connection is made, so we need to restart it
            bt_start_advertising(0);
        }
#endif
    }
}

static void count_bonds(const struct bt_bond_info *info, void *data)
{
    ARG_UNUSED(info);
    int *count_ptr = static_cast<int *>(data);
    (*count_ptr)++;
}

static bool bonds_present()
{
    int count = 0;
    bt_foreach_bond(BT_ID_DEFAULT, count_bonds, &count);
    LOG_INF("Bond Count %d", count);
    return count != 0;
}

/**
 * @brief Callback function called when a Bluetooth connection is disconnected.
 *
 * This function is called when a Bluetooth connection is disconnected. It logs
 * an message indicating the reason. If there was a bonding address associated
 * with the connection, it clears the bonding address and starts hidden
 * advertising to the last bonded device. It also stops the BLE update timer.
 *
 * @param conn Pointer to the Bluetooth connection structure
 * @param reason The reason for the disconnection, represented as a hexadecimal
 * code.
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Check if this was a phone connection (not D2D)
    if (conn != d2d_conn)
    {
        // This was a phone connection
        ble_set_phone_connection_state(false);

        // IMPORTANT: Clear message queues immediately on phone disconnect
        // This prevents queue overflow during activity when phone disconnects
        LOG_INF("Phone disconnected - clearing message queues to prevent overflow");
        ble_clear_message_queues();
    }

    // Clear Activity Metrics Service connection (primary only)
    ams_set_connection(NULL);
#endif

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Mark disconnection for potential recovery
    BleSequenceManager::getInstance().onDisconnect();
#endif

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Clear pending pairing state if this was the pending connection
    if (pending_pairing_conn == conn)
    {
        pending_pairing_conn = nullptr;
        pairing_failed_for_pending = false;
    }

    // Clear phone connection from SMP proxy if this was a phone connection
    if (conn != d2d_conn)
    {
        smp_proxy_clear_phone_conn(conn);
    }

    // Count remaining active connections after this disconnection
    int active_count = 0;
    auto count_conn_cb = [](struct bt_conn *conn_iter, void *data) {
        int *count = (int *)data;
        struct bt_conn *disconnected_conn = *((struct bt_conn **)data + 1);
        struct bt_conn_info info;

        // Skip the connection that just disconnected
        if (conn_iter == disconnected_conn)
        {
            return;
        }

        if (bt_conn_get_info(conn_iter, &info) == 0 && info.state == BT_CONN_STATE_CONNECTED)
        {
            (*count)++;
        }
    };

    // We need to pass both the count and the disconnected connection
    void *cb_data[2] = {&active_count, (void *)conn};
    bt_conn_foreach(
        BT_CONN_TYPE_LE,
        [](struct bt_conn *conn_iter, void *data) {
            void **params = (void **)data;
            int *count = (int *)params[0];
            struct bt_conn *disconnected_conn = (struct bt_conn *)params[1];
            struct bt_conn_info info;

            // Skip the connection that just disconnected
            if (conn_iter == disconnected_conn)
            {
                return;
            }

            if (bt_conn_get_info(conn_iter, &info) == 0 && info.state == BT_CONN_STATE_CONNECTED)
            {
                (*count)++;
            }
        },
        cb_data);

    // Always restart advertising when a connection disconnects, unless we're at max capacity
    if (active_count < CONFIG_BT_MAX_CONN)
    {
        LOG_INF("Connection disconnected - slots available (%d/%d), restarting advertising immediately", active_count,
                CONFIG_BT_MAX_CONN);
        bt_start_advertising(0);
    }
    else
    {
        LOG_INF("Connection disconnected but max connections still active (%d/%d), not restarting advertising",
                active_count, CONFIG_BT_MAX_CONN);
    }
#endif
}

/**
 * @brief Callback function called when the security level of a Bluetooth
 * connection changes.
 *
 * This function is called when the security level of a Bluetooth connection
 * changes. It logs an informational message indicating the new security level
 * if the change is successful, or an error message with the reason for the
 * security failure if the change fails.
 *
 * @param conn Pointer to the Bluetooth connection structure.
 * @param level The new security level of the connection.
 * @param err An enum representing the reason for the security change failure,
 * if any. '0' indicates successful security level change, non-zero indicates
 * failure.
 */
static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    const bt_addr_le_t *dst_addr = bt_conn_get_dst(conn);
    if (dst_addr)
    {
        bt_addr_le_to_str(dst_addr, addr, sizeof(addr));
    }
    else
    {
        strncpy(addr, "unknown", sizeof(addr) - 1);
        addr[sizeof(addr) - 1] = '\0';
    }

    if (err == 0)
    {
        LOG_INF("Security changed: %s level %u\n", addr, level);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Check if we now have proper security for encrypted characteristics
        if (level >= BT_SECURITY_L2)
        {
            LOG_INF("Security level sufficient for encrypted characteristics");
            bt_debug_info_service_status(conn);
        }
        else
        {
            LOG_WRN("Security level %u is insufficient for encrypted characteristics "
                    "(need >= L2)",
                    level);
        }
#endif
    }
    else
    {
        // Check if this is a D2D connection
        bool is_d2d = false;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        is_d2d = (conn == d2d_conn);
#else
        is_d2d = true;
#endif

        if (is_d2d && (err == BT_SECURITY_ERR_PIN_OR_KEY_MISSING || err == BT_SECURITY_ERR_AUTH_FAIL ||
                       err == BT_SECURITY_ERR_PAIR_NOT_SUPPORTED))
        {
            LOG_INF("Security failed for D2D connection (err %d) - this is expected for "
                    "device-to-device connections, continuing without security",
                    err);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            // For primary device, if this is the D2D connection and security failed
            // as expected, we can still proceed since D2D doesn't require security
            if (conn == d2d_conn)
            {
                LOG_INF("D2D connection security negotiation complete (no security required), "
                        "connection remains active");
            }
#endif
        }
        else if (!is_d2d)
        {
            // This is likely a phone connection that needs security
            LOG_ERR("Security failed for phone connection: %s level %u err %d", addr, level, err);
        }
        else
        {
            LOG_ERR("Unexpected security failure: %s level %u err %d", addr, level, err);
        }
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {.connected = connected,
                                     .disconnected = disconnected,
                                     .recycled = NULL,
                                     .le_param_req = NULL,
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                                     .le_param_updated = ble_conn_params_on_updated,
#else
                                     .le_param_updated = NULL,
#endif
#if defined(CONFIG_BT_SMP)
                                     .identity_resolved = NULL,
#endif
#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CLASSIC)
                                     .security_changed = security_changed,
#endif
                                     ._node = {}};

// Currently unused - kept for future power management
__attribute__((unused)) static void ble_set_power_off()
{}

int set_bluetooth_name()
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    const char *name = "BotzRightSh";
    int err = bt_set_name(name);
    if (err == 0)
    {
        LOG_INF("Bluetooth device name set to: %s", name);
    }
    else
    {
        LOG_ERR("Failed to set Bluetooth device name to %s, error: %d", name, err);
    }
#else
    const char *name = "BotzLeftSh";
    int err = bt_set_name(name);
    if (err == 0)
    {
        LOG_INF("Bluetooth device name set to: %s", name);
    }
    else
    {
        LOG_ERR("Failed to set Bluetooth device name to %s, error: %d", name, err);
    }
#endif
    return err;
}

/**
 * @brief Initializes the Bluetooth stack.
 *
 * This function initializes the Bluetooth stack by enabling Bluetooth,
 * loading settings, and registering a callback for authentication information.
 * If Bluetooth initialization fails, an error message is logged.
 * After initialization, the Bluetooth Low Energy (BLE) module is considered
 * initialized, and subsequent BLE operations can be performed.
 */
err_t bt_module_init(void)
{
    int err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)\n", err);
        return err_t::BLUETOOTH_ERROR;
    }

    // Load settings first to ensure ID address is available
    err = settings_load();
    if (err != 0)
    {
        LOG_ERR("Bluetooth settings_load() call failed (err %d)", err);
        return err_t::BLUETOOTH_ERROR;
    }

    // Longer delay to ensure settings and identity are fully loaded
    k_msleep(50);

    // Now set the Bluetooth name after settings are loaded
    int err_name = set_bluetooth_name();
    if (err_name != 0)
    {
        LOG_ERR("Failed to Set Name (err %d)", err_name);
        return err_t::BLUETOOTH_ERROR;
    }

    err = bt_conn_auth_info_cb_register(&bt_conn_auth_info);
    if (err)
    {
        LOG_INF("Can't register authentication information callbacks (err: %d)\n", err);
        return err_t::BLUETOOTH_ERROR;
    }

    static struct bt_conn_cb d2d_conn_callbacks = {.connected = d2d_connected,
                                                   .disconnected = d2d_disconnected,
                                                   .recycled = NULL,
                                                   .le_param_req = NULL,
                                                   .le_param_updated = NULL,
#if defined(CONFIG_BT_SMP)
                                                   .identity_resolved = NULL,
#endif
#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CLASSIC)
                                                   .security_changed = NULL,
#endif
                                                   ._node = {}};

    bt_conn_cb_register(&d2d_conn_callbacks);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Initialize BLE recovery handler (only for primary device)
    ble_recovery_handler_init();
    LOG_INF("BLE recovery handler initialized");

    // Initialize connection parameter manager
    int conn_params_err = ble_conn_params_init();
    if (conn_params_err != 0)
    {
        LOG_ERR("Failed to initialize connection parameter manager: %d", conn_params_err);
    }
    else
    {
        LOG_INF("Connection parameter manager initialized");
    }

    // Log that services are being initialized
    LOG_INF("Initializing primary device BLE services...");
    LOG_INF("Information Service UUID: 0c372eaa-27eb-437e-bef4-775aefaf3c97");

    // Clear any stale FOTA progress from Information Service
    fota_progress_msg_t clean_progress = {
        .is_active = false, .status = 0, .percent_complete = 0, .bytes_received = 0, .total_size = 0, .error_code = 0};
    jis_fota_progress_notify(&clean_progress);
    LOG_INF("Device Info Service, Control Service, and Information Service are "
            "registered via BT_GATT_SERVICE_DEFINE");

    ble_d2d_rx_init(); // Accept commands from secondary

    // Initialize FOTA proxy service for primary device
    int fota_err = fota_proxy_init();
    if (fota_err != 0)
    {
        LOG_ERR("Failed to initialize FOTA proxy: %d", fota_err);
    }
    else
    {
        LOG_INF("FOTA proxy service initialized");
    }

    // Initialize file proxy service for primary device
    int file_err = file_proxy_init();
    if (file_err != 0)
    {
        LOG_ERR("Failed to initialize file proxy: %d", file_err);
    }
    else
    {
        LOG_INF("File proxy service initialized");
    }

    // Initialize SMP proxy service for primary device
    int smp_err = smp_proxy_init();
    if (smp_err != 0)
    {
        LOG_ERR("Failed to initialize SMP proxy: %d", smp_err);
    }
    else
    {
        LOG_INF("SMP proxy service initialized - mobile can use standard MCUmgr!");
    }

    // Check if we have a valid identity address before advertising
    bt_addr_le_t addr;
    size_t count = 1;
    bt_id_get(&addr, &count);
    if (count > 0)
    {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
        LOG_INF("Bluetooth identity address: %s", addr_str);
    }
    else
    {
        LOG_WRN("No Bluetooth identity address available");
    }

    if (bt_start_advertising(0) == err_t::NO_ERROR)
    {
        LOG_INF("BLE Started Advertising");
    }
    else
    {
        LOG_ERR("BLE Failed to enter advertising mode");
    }
#else
    ble_d2d_tx_init();            // Prepare to send data to primary
    ble_d2d_file_transfer_init(); // Initialize D2D file transfer service

// MCUmgr SMP Bluetooth transport is automatically registered when enabled
#if defined(CONFIG_MCUMGR_TRANSPORT_BT)
    LOG_INF("MCUmgr SMP Bluetooth transport enabled for secondary device");
    LOG_INF("SMP service will be available for receiving commands from primary");
#else
    LOG_WRN("MCUmgr Bluetooth transport not enabled in configuration");
#endif

    // ADD: Start advertising so phones can discover the secondary device
    LOG_INF("Secondary device starting advertising as 'BotzLeftSh'");
    int adv_err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (adv_err)
    {
        LOG_ERR("Failed to start advertising: %d", adv_err);
        // Continue anyway - D2D functionality is more important
    }
    else
    {
        LOG_INF("Secondary device is now advertising and visible to phones");
    }

    start_scan(); // Start scanning for primary device
#endif

    bluetooth_tid =
        k_thread_create(&bluetooth_thread_data, bluetooth_stack_area, K_THREAD_STACK_SIZEOF(bluetooth_stack_area),
                        bluetooth_process, nullptr, nullptr, nullptr, bluetooth_priority, 0, K_NO_WAIT);

    return err_t::NO_ERROR;
}

// Forward declarations for work handlers
static void foot_samples_work_handler(struct k_work *work);
static void foot_samples_secondary_work_handler(struct k_work *work);
static void bhi360_3d_mapping_work_handler(struct k_work *work);
static void bhi360_3d_mapping_secondary_work_handler(struct k_work *work);
static void bhi360_linear_accel_work_handler(struct k_work *work);
static void bhi360_linear_accel_secondary_work_handler(struct k_work *work);
static void bhi360_step_count_work_handler(struct k_work *work);
static void activity_step_count_work_handler(struct k_work *work);
static void command_work_handler(struct k_work *work);
static void fota_progress_work_handler(struct k_work *work);
static void error_status_work_handler(struct k_work *work);
static void new_activity_log_work_handler(struct k_work *work);
static void activity_metrics_ble_work_handler(struct k_work *work);
static void device_info_work_handler(struct k_work *work);
static void weight_measurement_work_handler(struct k_work *work);
static void battery_level_primary_work_handler(struct k_work *work);
static void battery_level_secondary_work_handler(struct k_work *work);

void bluetooth_process(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{
    // Declare a generic_message_t structure to hold the received message
    generic_message_t msg;
    int ret;

    init_rtc_time();

    // Initialize work queue for asynchronous message processing
    k_work_queue_init(&bluetooth_work_q);
    k_work_queue_start(&bluetooth_work_q, bluetooth_workq_stack, K_THREAD_STACK_SIZEOF(bluetooth_workq_stack),
                       bluetooth_priority - 1, NULL);
    k_thread_name_set(&bluetooth_work_q.thread, "bluetooth_wq");

    // Initialize all work items with their handlers
    k_work_init(&foot_samples_work, foot_samples_work_handler);
    k_work_init(&foot_samples_secondary_work, foot_samples_secondary_work_handler);
    k_work_init(&bhi360_3d_mapping_work, bhi360_3d_mapping_work_handler);
    k_work_init(&bhi360_3d_mapping_secondary_work, bhi360_3d_mapping_secondary_work_handler);
    k_work_init(&bhi360_linear_accel_work, bhi360_linear_accel_work_handler);
    k_work_init(&bhi360_linear_accel_secondary_work, bhi360_linear_accel_secondary_work_handler);
    k_work_init(&bhi360_step_count_work, bhi360_step_count_work_handler);
    k_work_init(&activity_step_count_work, activity_step_count_work_handler);
    k_work_init(&command_work, command_work_handler);
    k_work_init(&fota_progress_work, fota_progress_work_handler);
    k_work_init(&error_status_work, error_status_work_handler);
    k_work_init(&new_activity_log_work, new_activity_log_work_handler);
    k_work_init(&activity_metrics_ble_work, activity_metrics_ble_work_handler);
    k_work_init(&device_info_work, device_info_work_handler);
    k_work_init(&weight_measurement_work, weight_measurement_work_handler);
    k_work_init(&battery_level_primary_work, battery_level_primary_work_handler);
    k_work_init(&battery_level_secondary_work, battery_level_secondary_work_handler);

    LOG_INF("Bluetooth work queue initialized");

    while (true)
    {
        // Wait indefinitely for a message to appear in the queue.
        // K_FOREVER makes the thread block until a message is available.
        ret = k_msgq_get(&bluetooth_msgq, &msg, K_FOREVER);

        if (ret == 0)
        { // Message successfully received

            // Copy message to appropriate buffer and submit work to queue
            // This allows the main thread to immediately return to waiting for next message
            switch (msg.type)
            {
                case MSG_TYPE_FOOT_SAMPLES:
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    // Primary device: Route based on sender
                    if (msg.sender == SENDER_D2D_SECONDARY)
                    {
                        // Secondary foot data from D2D - use separate work item
                        k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                        memcpy(&pending_foot_samples_secondary_msg, &msg, sizeof(msg));
                        k_mutex_unlock(&bluetooth_msg_mutex);
                        k_work_submit_to_queue(&bluetooth_work_q, &foot_samples_secondary_work);
                    }
                    else
                    {
                        // Primary's own foot data
                        k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                        memcpy(&pending_foot_samples_msg, &msg, sizeof(msg));
                        k_mutex_unlock(&bluetooth_msg_mutex);
                        k_work_submit_to_queue(&bluetooth_work_q, &foot_samples_work);
                    }
#else
                    // Secondary device: All foot data goes through normal handler to be sent via D2D
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_foot_samples_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);
                    k_work_submit_to_queue(&bluetooth_work_q, &foot_samples_work);
#endif
                    break;

                case MSG_TYPE_BHI360_3D_MAPPING:
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    // Primary device: Route based on sender
                    if (msg.sender == SENDER_D2D_SECONDARY)
                    {
                        // Secondary BHI360 3D data from D2D - use separate work item
                        k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                        memcpy(&pending_bhi360_3d_secondary_msg, &msg, sizeof(msg));
                        k_mutex_unlock(&bluetooth_msg_mutex);
                        k_work_submit_to_queue(&bluetooth_work_q, &bhi360_3d_mapping_secondary_work);
                    }
                    else
                    {
                        // Primary's own BHI360 3D data
                        k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                        memcpy(&pending_bhi360_3d_msg, &msg, sizeof(msg));
                        k_mutex_unlock(&bluetooth_msg_mutex);
                        k_work_submit_to_queue(&bluetooth_work_q, &bhi360_3d_mapping_work);
                    }
#else
                    // Secondary device: All BHI360 3D data goes through normal handler to be sent via D2D
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_bhi360_3d_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);
                    k_work_submit_to_queue(&bluetooth_work_q, &bhi360_3d_mapping_work);
#endif
                    break;

                case MSG_TYPE_BHI360_LINEAR_ACCEL:
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    // Primary device: Route based on sender
                    if (msg.sender == SENDER_D2D_SECONDARY)
                    {
                        // Secondary linear accel data from D2D - use separate work item
                        k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                        memcpy(&pending_bhi360_linear_secondary_msg, &msg, sizeof(msg));
                        k_mutex_unlock(&bluetooth_msg_mutex);
                        k_work_submit_to_queue(&bluetooth_work_q, &bhi360_linear_accel_secondary_work);
                    }
                    else
                    {
                        // Primary's own linear accel data
                        k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                        memcpy(&pending_bhi360_linear_msg, &msg, sizeof(msg));
                        k_mutex_unlock(&bluetooth_msg_mutex);
                        k_work_submit_to_queue(&bluetooth_work_q, &bhi360_linear_accel_work);
                    }
#else
                    // Secondary device: All linear accel data goes through normal handler to be sent via D2D
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_bhi360_linear_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);
                    k_work_submit_to_queue(&bluetooth_work_q, &bhi360_linear_accel_work);
#endif
                    break;

                case MSG_TYPE_BHI360_STEP_COUNT:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_bhi360_step_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);

                    k_work_submit_to_queue(&bluetooth_work_q, &bhi360_step_count_work);

                    break;

                case MSG_TYPE_ACTIVITY_STEP_COUNT:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_activity_step_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);

                    k_work_submit_to_queue(&bluetooth_work_q, &activity_step_count_work);

                    break;

                case MSG_TYPE_COMMAND:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_command_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);

                    k_work_submit_to_queue(&bluetooth_work_q, &command_work);

                    break;

                case MSG_TYPE_FOTA_PROGRESS:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_fota_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);

                    k_work_submit_to_queue(&bluetooth_work_q, &fota_progress_work);

                    break;

                case MSG_TYPE_ERROR_STATUS:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_error_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);

                    k_work_submit_to_queue(&bluetooth_work_q, &error_status_work);

                    break;

                case MSG_TYPE_NEW_ACTIVITY_LOG_FILE:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_log_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);

                    k_work_submit_to_queue(&bluetooth_work_q, &new_activity_log_work);

                    break;

                case MSG_TYPE_ACTIVITY_METRICS_BLE:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_metrics_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);

                    k_work_submit_to_queue(&bluetooth_work_q, &activity_metrics_ble_work);

                    break;

                case MSG_TYPE_DEVICE_INFO:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_device_info_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);
                    // Check if work is already pending before submitting

                    k_work_submit_to_queue(&bluetooth_work_q, &device_info_work);

                    break;

                case MSG_TYPE_WEIGHT_MEASUREMENT:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_weight_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);

                    k_work_submit_to_queue(&bluetooth_work_q, &weight_measurement_work);

                    break;

                case MSG_TYPE_BATTERY_LEVEL_PRIMARY:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_battery_primary_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);

                    k_work_submit_to_queue(&bluetooth_work_q, &battery_level_primary_work);

                    break;

                case MSG_TYPE_BATTERY_LEVEL_SECONDARY:
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_battery_secondary_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);

                    k_work_submit_to_queue(&bluetooth_work_q, &battery_level_secondary_work);

                    break;

                case MSG_TYPE_BIOMECHANICS_EXTENDED:
                    // Handle biomechanics extended data
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_metrics_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);
                    
                    // Process via existing activity metrics handler
                    k_work_submit_to_queue(&bluetooth_work_q, &activity_metrics_ble_work);
                    break;

                case MSG_TYPE_SESSION_SUMMARY:
                    // Handle session summary data
                    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
                    memcpy(&pending_metrics_msg, &msg, sizeof(msg));
                    k_mutex_unlock(&bluetooth_msg_mutex);
                    
                    // Process via existing activity metrics handler
                    k_work_submit_to_queue(&bluetooth_work_q, &activity_metrics_ble_work);
                    break;

                case MSG_TYPE_EXTERNAL_FLASH_ERASE_COMPLETE:
                    LOG_INF("Received external flash erase complete notification");
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    // Only primary device needs to notify via BLE characteristic
                    cs_external_flash_erase_complete_notify();
#else
                    // Secondary device: flash erase complete, no BLE notification needed
                    LOG_INF("Secondary device: Flash erase complete (no BLE notification)");
#endif
                    break;

                default:
                    LOG_WRN("Unknown message type received (%d) from %s", msg.type, get_sender_name(msg.sender));
                    break;
            }
        }
    }
}

/**
 * @brief Starts Bluetooth advertising.
 *
 * This function initiates Bluetooth Low Energy (BLE) advertising using the
 * specified advertising led_data. It stops any ongoing advertising before
 * starting a new one. After starting advertising, it logs an error message if
 * the operation fails and logs an informational message if advertising starts
 * successfully. Additionally, it starts a timer to timeout advertising after a
 * specified duration and publishes a Bluetooth state event indicating that the
 * device is in the advertising state.
 *
 * @param err Pointer to an integer variable to store the result of the
 * advertising start operation. This parameter is both an input and output
 * parameter. It is used to propagate the error code from the advertising start
 * operation to the caller.
 */
// Only used by primary device
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
err_t bt_start_advertising(int err)
{

    bt_stop_advertising();

    // Log advertising parameters for debugging
    LOG_INF("Starting advertising with params: options=0x%02x, interval_min=%d, "
            "interval_max=%d",
            bt_le_adv_conn_name.options, bt_le_adv_conn_name.interval_min, bt_le_adv_conn_name.interval_max);

    err = bt_le_adv_start(&bt_le_adv_conn_name, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

    if (err)
    {
        LOG_ERR("Advertising failed to start, error=%d", err);

        // If advertising fails due to identity issues, try without USE_IDENTITY
        if (err == -ENOENT || err == -EINVAL)
        {
            LOG_WRN("Retrying advertising without USE_IDENTITY option");

            // Create temporary advertising params without USE_IDENTITY
            struct bt_le_adv_param temp_adv_params = {
                .id = 0,
                .sid = 0,
                .secondary_max_skip = 0,
                .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_NAME,
                .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
                .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
                .peer = NULL,
            };

            err = bt_le_adv_start(&temp_adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
            if (err)
            {
                LOG_ERR("Advertising retry also failed, error=%d", err);
                return err_t::BLUETOOTH_ERROR;
            }
            else
            {
                LOG_WRN("Advertising started without USE_IDENTITY");
            }
        }
        else
        {
            return err_t::BLUETOOTH_ERROR;
        }
    }
    else
    {
        LOG_INF("Advertising successfully started");
        LOG_WRN("Firmware version: %s", APP_VERSION_STRING);
    }

    // Only start timer if we're at max connections - otherwise keep advertising continuously
    int active_count = 0;
    auto count_conn_cb = [](struct bt_conn *conn, void *data) {
        int *count = (int *)data;
        struct bt_conn_info info;
        if (bt_conn_get_info(conn, &info) == 0 && info.state == BT_CONN_STATE_CONNECTED)
        {
            (*count)++;
        }
    };

    bt_conn_foreach(BT_CONN_TYPE_LE, count_conn_cb, &active_count);

    if (active_count >= CONFIG_BT_MAX_CONN - 1)
    {
        // Close to max connections, start timeout timer
        LOG_INF("Near max connections (%d/%d), starting advertising timeout timer", active_count, CONFIG_BT_MAX_CONN);
        k_timer_start(&ble_timer, K_MSEC(BLE_ADVERTISING_TIMEOUT_MS), K_NO_WAIT);
    }
    else
    {
        // Stop any existing timer to keep advertising continuously
        k_timer_stop(&ble_timer);
        LOG_INF("Advertising continuously - connection slots available (%d/%d)", active_count, CONFIG_BT_MAX_CONN);
    }

    return err_t::NO_ERROR;
}
#endif // CONFIG_PRIMARY_DEVICE

/**
 * @brief Starts hidden Bluetooth Low Energy (BLE) advertising.
 *
 * This function initiates BLE advertising in a hidden mode, where the
 * advertising packets are not broadcasted openly. It logs an error message if
 * the advertising fails to start and logs an informational message if the
 * advertising starts successfully.
 */
err_t ble_start_hidden()
{
    bt_stop_advertising();

    int err = bt_le_adv_start(bt_le_adv_conn, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err)
    {
        LOG_ERR("Hidden Advertising failed to start, error=%d", err);
        return err_t::BLUETOOTH_ERROR;
    }

    LOG_INF("Hidden Advertising successfully started");
    return err_t::NO_ERROR;
}

/**
 * @brief The main timer expiry function for the BT,
 *
 * @param timer
 *
 */
static void ble_timer_expiry_function(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    int ret = k_work_submit(&ble_handler);
    if (ret < 0)
    {
        LOG_ERR("BLE submission error (err %d)", ret);
        k_timer_stop(&ble_timer);
    }
}

/**
 * @brief Handler function for the BLE timer.
 *
 * This function is called when the BLE timer expires. It stops the BLE timer,
 * turns off the BLE LED, stops BLE advertising, and starts the BLE in hidden
 * mode.
 *
 * @param work Pointer to the work item associated with the timer handler
 * (unused). This parameter is not used in the implementation of this function.
 */
static void ble_timer_handler_function(struct k_work *work)
{
    ARG_UNUSED(work);
    k_timer_stop(&ble_timer);
    if (ble_status_connected == 0)
    {
    }
    ble_status_connected = 0;

    LOG_INF("Bluetooth advertising timeout");

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Count active connections to determine if we should restart advertising
    int active_count = 0;
    auto count_conn_cb = [](struct bt_conn *conn, void *data) {
        int *count = (int *)data;
        struct bt_conn_info info;
        if (bt_conn_get_info(conn, &info) == 0 && info.state == BT_CONN_STATE_CONNECTED)
        {
            (*count)++;
        }
    };

    bt_conn_foreach(BT_CONN_TYPE_LE, count_conn_cb, &active_count);

    // Always restart advertising if we have slots available
    if (active_count < CONFIG_BT_MAX_CONN)
    {
        LOG_INF("Advertising timeout - connection slots available (%d/%d), restarting advertising immediately",
                active_count, CONFIG_BT_MAX_CONN);
        bt_start_advertising(0);
        return;
    }
    else
    {
        LOG_INF("Advertising timeout - maximum connections reached (%d/%d), switching to hidden mode", active_count,
                CONFIG_BT_MAX_CONN);
    }
#endif

    int ret = bt_le_adv_stop();
    if (ret != 0)
    {
        LOG_ERR("Failed to stop (err 0x%02x)", ret);
    }

    if (bonds_present())
    {
        err_t err = ble_start_hidden();
        if (err != err_t::NO_ERROR)
        {
            LOG_ERR("Failed to start %d", (int)err);
        }
    }
}

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
/**
 * @brief Handler for weight measurement timeout
 *
 * This function is called when we timeout waiting for secondary device weight
 * measurement
 */
static void weight_measurement_timeout_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (waiting_for_secondary)
    {
        LOG_WRN("Timeout waiting for secondary weight, using primary only: %.1f kg", (double)primary_weight_kg);
        jis_weight_measurement_notify(primary_weight_kg);

        // Reset state
        waiting_for_secondary = false;
        primary_weight_kg = 0;
        secondary_weight_kg = 0;
    }
}
#endif

/**
 * @brief Work handler for processing foot samples
 */
static void foot_samples_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_foot_samples_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    foot_samples_t *foot_data = &msg.data.foot_samples;
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Primary device: This handler now only processes primary foot data
    // Secondary data is handled by foot_samples_secondary_work_handler
    jis_foot_sensor_notify(foot_data);
#else
    // Secondary device: Send to primary via D2D
    LOG_ERR("Calling d2d");
    ble_d2d_tx_send_foot_sensor_data(foot_data);
#endif
}

/**
 * @brief Work handler for processing secondary foot samples
 */
static void foot_samples_secondary_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_foot_samples_secondary_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    foot_samples_t *foot_data = &msg.data.foot_samples;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Primary device: Update global secondary foot data for Information Service aggregation
    LOG_DBG("Processing secondary foot sensor data");

    // Update global secondary foot data for aggregation in Information Service
    memcpy(&g_secondary_foot_data, foot_data, sizeof(foot_samples_t));
    g_secondary_data_valid = true;
    g_secondary_last_timestamp = k_uptime_get_32();

    LOG_DBG("Updated global secondary foot data - values[0-3]: %u %u %u %u", g_secondary_foot_data.values[0],
            g_secondary_foot_data.values[1], g_secondary_foot_data.values[2], g_secondary_foot_data.values[3]);
    LOG_DBG("Updated global secondary foot data - values[4-7]: %u %u %u %u", g_secondary_foot_data.values[4],
            g_secondary_foot_data.values[5], g_secondary_foot_data.values[6], g_secondary_foot_data.values[7]);

    // Call the new update function which will check if we should send aggregated notification
    jis_foot_sensor_update_secondary(foot_data);

    // Forward to data module for logging (if needed)
    // extern struct k_msgq data_msgq;
    // generic_message_t data_msg;
    // data_msg.sender = SENDER_D2D_SECONDARY;
    // data_msg.type = MSG_TYPE_FOOT_SAMPLES;
    // memcpy(&data_msg.data.foot_samples, foot_data, sizeof(foot_samples_t));

    // if (k_msgq_put(&data_msgq, &data_msg, K_NO_WAIT) != 0) {
    //    LOG_WRN("Failed to forward secondary foot data to data module");
    // }
#else
    // This handler should not be called on secondary device
    LOG_ERR("Unexpected: Secondary foot handler called on secondary device");
#endif
}

/**
 * @brief Work handler for processing BHI360 3D mapping data
 */
static void bhi360_3d_mapping_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_bhi360_3d_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    bhi360_3d_mapping_t *mapping_data = &msg.data.bhi360_3d_mapping;
    LOG_INF("Received BHI360 3D Mapping from %s: Accel(%.2f,%.2f,%.2f), "
            "Gyro(%.2f,%.2f,%.2f)",
            get_sender_name(msg.sender), (double)mapping_data->accel_x, (double)mapping_data->accel_y,
            (double)mapping_data->accel_z, (double)mapping_data->gyro_x, (double)mapping_data->gyro_y,
            (double)mapping_data->gyro_z);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Primary device: This handler now only processes primary BHI360 data
    // Secondary data is handled by bhi360_3d_mapping_secondary_work_handler
    jis_bhi360_data1_notify(mapping_data);
#else
    // Secondary device: Send to primary via D2D
    ble_d2d_tx_send_bhi360_data1(mapping_data);
#endif
}

/**
 * @brief Work handler for processing secondary BHI360 3D mapping data
 */
static void bhi360_3d_mapping_secondary_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_bhi360_3d_secondary_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    bhi360_3d_mapping_t *mapping_data = &msg.data.bhi360_3d_mapping;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Primary device: Update global secondary quaternion data for Information Service aggregation
    LOG_DBG("Processing secondary BHI360 3D mapping data");

    // Update global secondary quaternion data for aggregation in Information Service
    // The BHI360 3D mapping uses accel_x/y/z fields for quaternion x/y/z and quat_w for w
    // Convert float quaternion to fixed-point format for storage
    g_secondary_quat_data.quat_x = float_to_fixed16(mapping_data->accel_x, FixedPoint::QUAT_SCALE);
    g_secondary_quat_data.quat_y = float_to_fixed16(mapping_data->accel_y, FixedPoint::QUAT_SCALE);
    g_secondary_quat_data.quat_z = float_to_fixed16(mapping_data->accel_z, FixedPoint::QUAT_SCALE);
    g_secondary_quat_data.quat_w = float_to_fixed16(mapping_data->quat_w, FixedPoint::QUAT_SCALE);
    g_secondary_data_valid = true;
    g_secondary_last_timestamp = k_uptime_get_32();

    LOG_DBG("Updated global secondary quaternion data - (%.2f,%.2f,%.2f,%.2f)", (double)mapping_data->accel_x,
            (double)mapping_data->accel_y, (double)mapping_data->accel_z, (double)mapping_data->quat_w);
    LOG_DBG("Quaternion (fixed): x=%d y=%d z=%d w=%d", g_secondary_quat_data.quat_x, g_secondary_quat_data.quat_y,
            g_secondary_quat_data.quat_z, g_secondary_quat_data.quat_w);

    // Call the new update function which will check if we should send aggregated notification
    jis_bhi360_data1_update_secondary(mapping_data);

    // Note: We don't forward raw BHI360 3D mapping to data module as it's not needed for storage
    // The calculated bilateral metrics are saved via MSG_TYPE_REALTIME_METRICS_DATA instead
#else
    // This handler should not be called on secondary device
    LOG_ERR("Unexpected: Secondary BHI360 3D handler called on secondary device");
#endif
}

/**
 * @brief Work handler for processing BHI360 linear acceleration data
 */
static void bhi360_linear_accel_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_bhi360_linear_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    bhi360_linear_accel_t *lacc_data = &msg.data.bhi360_linear_accel;
    LOG_INF("Received BHI360 Linear Accel from %s: (%.2f,%.2f,%.2f)", get_sender_name(msg.sender), (double)lacc_data->x,
            (double)lacc_data->y, (double)lacc_data->z);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Primary device: This handler now only processes primary linear accel data
    // Secondary data is handled by bhi360_linear_accel_secondary_work_handler
//    jis_bhi360_data3_notify(lacc_data);
#else
    // Secondary device: Send to primary via D2D
    ble_d2d_tx_send_bhi360_data3(lacc_data);
#endif
}

/**
 * @brief Work handler for processing secondary BHI360 linear acceleration data
 */
static void bhi360_linear_accel_secondary_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_bhi360_linear_secondary_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    bhi360_linear_accel_t *lacc_data = &msg.data.bhi360_linear_accel;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Primary device: Handle secondary linear accel data properly
    LOG_DBG("Processing secondary BHI360 linear acceleration data");
    LOG_INF("Secondary BHI360 Linear Accel: (%.2f,%.2f,%.2f)", (double)lacc_data->x, (double)lacc_data->y,
            (double)lacc_data->z);

    // TODO: In future, we could have a separate characteristic for secondary linear accel data
    // Note: We don't forward raw linear accel to data module as it's not needed for storage
    // The calculated bilateral metrics are saved via MSG_TYPE_REALTIME_METRICS_DATA instead

    // Note: We intentionally do NOT call jis_bhi360_data3_notify() here
    // as that would mix secondary data with primary data
#else
    // This handler should not be called on secondary device
    LOG_ERR("Unexpected: Secondary linear accel handler called on secondary device");
#endif
}
/**
 * @brief Work handler for processing BHI360 step count data
 */
static void bhi360_step_count_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_bhi360_step_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    bhi360_step_count_t *step_data = &msg.data.bhi360_step_count;
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Check if this is activity step count (from SENDER_MOTION_SENSOR)
    if (msg.sender == SENDER_MOTION_SENSOR)
    {
        primary_activity_steps = step_data->step_count;
        uint32_t total_activity_steps = primary_activity_steps + secondary_activity_steps;
        LOG_INF("Activity Step Count - Primary=%u, Secondary=%u, Total=%u", primary_activity_steps,
                secondary_activity_steps, total_activity_steps);
        ams_update_activity_step_count(total_activity_steps);
        return;
    }
    LOG_INF("Received BHI360 Step Count from %s: Steps=%u", get_sender_name(msg.sender), step_data->step_count);
    // Update step counts based on sender
    if (msg.sender == SENDER_BHI360_THREAD)
    {
        primary_step_count = step_data->step_count;
    }
    else if (msg.sender == SENDER_D2D_SECONDARY)
    {
        secondary_step_count = step_data->step_count;
    }
    // Only notify aggregated counts
    uint32_t total_steps = primary_step_count + secondary_step_count;
    LOG_DBG("Global step aggregation: Primary=%u, Secondary=%u, Total=%u", primary_step_count, secondary_step_count,
            total_steps);
    ams_update_total_step_count(total_steps);
#else
    LOG_INF("Secondary device sending step count: %u", step_data->step_count);
    // Secondary device: Send to primary via D2D batching logic
    ble_d2d_tx_send_bhi360_data2(step_data);
#endif
}

/**
 * @brief Work handler for processing activity step count
 */
static void activity_step_count_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_activity_step_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    // This is specifically for activity step counts
    bhi360_step_count_t *step_data = &msg.data.bhi360_step_count;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (msg.sender == SENDER_D2D_SECONDARY)
    {
        // Activity step count from secondary
        secondary_activity_steps = step_data->step_count;

        // Calculate total activity steps
        uint32_t total_activity_steps = primary_activity_steps + secondary_activity_steps;
        LOG_INF("Secondary Activity Step Count Update - Secondary=%u, Total=%u", secondary_activity_steps,
                total_activity_steps);

        ams_update_activity_step_count(total_activity_steps);
    }
#endif
}

/**
 * @brief Work handler for processing commands
 */
static void command_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_MSEC(100));
    generic_message_t msg = pending_command_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    char *command_str = msg.data.command_str;
    LOG_INF("Received Command from %s: '%s'", get_sender_name(msg.sender), command_str);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (strcmp(command_str, "RESET_ACTIVITY_STEPS") == 0)
    {
        // Reset activity step counts
        primary_activity_steps = 0;
        secondary_activity_steps = 0;
        ams_update_activity_step_count(0); // Notify 0 activity steps
        LOG_INF("Activity step counts reset to 0");
    }
    else if (strncmp(command_str, "WEIGHT:", 7) == 0)
    {
        // Handle weight measurement result from activity metrics
        float weight_kg;
        if (sscanf(command_str, "WEIGHT:%f", &weight_kg) == 1)
        {
            LOG_INF("Received weight measurement: %.1f kg", (double)weight_kg);
            // Send to information service for BLE notification
            jis_weight_measurement_notify(weight_kg);
        }
    }
#endif
}

/**
 * @brief Work handler for processing FOTA progress - this is a complex handler with significant state management
 */
static void fota_progress_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_fota_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    // Handle FOTA progress updates
    fota_progress_msg_t *progress = &msg.data.fota_progress;

    // Debug: Log stale messages
    if (!progress->is_active && progress->bytes_received > 0)
    {
        LOG_WRN("Stale FOTA progress detected: bytes=%u, percent=%d%% (ignoring)", progress->bytes_received,
                progress->percent_complete);
        return; // Ignore stale messages
    }

    LOG_INF("Received FOTA Progress: active=%d, status=%d, percent=%d%%, "
            "bytes=%u/%u",
            progress->is_active, progress->status, progress->percent_complete, progress->bytes_received,
            progress->total_size);

    // Detect completion between app core and net core
    static bool app_core_done = false;
    static bool fota_was_active = false;

    // App core complete: status=2 and large size
    if (progress->status == 2 && progress->bytes_received > 500000 && !app_core_done)
    {
        LOG_INF("===========================================");
        LOG_INF("=== APP CORE UPDATE COMPLETE ===");
        LOG_INF("Size: %u bytes", progress->bytes_received);
        LOG_INF("Waiting for network core update...");
        LOG_INF("===========================================");
        app_core_done = true;
    }

    // Net core complete: status=2 and small size
    if (progress->status == 2 && progress->bytes_received < 200000 && progress->bytes_received > 100000)
    {
        LOG_INF("===========================================");
        LOG_INF("=== NETWORK CORE UPDATE COMPLETE ===");
        LOG_INF("Size: %u bytes", progress->bytes_received);
        LOG_INF("=== ALL FOTA UPDATES COMPLETE ===");
        LOG_INF("===========================================");
        app_core_done = false;
    }

    if (progress->is_active && progress->status == 1 && !fota_was_active)
    {
        LOG_WRN("FOTA update starting - stopping advertising and reducing "
                "BLE activity");

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Stop advertising to prevent new connections during FOTA
        int adv_err = bt_le_adv_stop();
        if (adv_err && adv_err != -EALREADY)
        {
            LOG_ERR("Failed to stop advertising: %d", adv_err);
        }
        else
        {
            LOG_INF("Advertising stopped for FOTA update");
        }

        // Stop the BLE timer to prevent advertising restart
        k_timer_stop(&ble_timer);
        LOG_INF("BLE timer stopped for FOTA update");

        // Request connection parameter update to reduce BLE activity
        // Use BACKGROUND_IDLE profile for minimal interference
        struct bt_conn *active_conn = NULL;
        bt_conn_foreach(BT_CONN_TYPE_LE, find_active_conn_cb, &active_conn);

        if (active_conn)
        {
            // Check if connection is in a state where parameters can be updated
            struct bt_conn_info info;
            int info_err = bt_conn_get_info(active_conn, &info);
            if (info_err == 0 && info.state == BT_CONN_STATE_CONNECTED)
            {
                LOG_INF("Updating connection parameters for FOTA - switching to "
                        "BACKGROUND profile");
                int conn_err = ble_conn_params_update(active_conn, CONN_PROFILE_BACKGROUND);
                if (conn_err)
                {
                    if (conn_err == -EINVAL)
                    {
                        LOG_WRN("Connection parameter update failed: Invalid "
                                "parameters, trying "
                                "FOREGROUND");
                        // Try FOREGROUND profile as fallback
                        conn_err = ble_conn_params_update(active_conn, CONN_PROFILE_FOREGROUND);
                        if (conn_err == 0)
                        {
                            LOG_INF("Using FOREGROUND profile for FOTA instead");
                        }
                    }
                    else if (conn_err == -EALREADY)
                    {
                        LOG_WRN("Connection parameter update already in progress");
                    }
                    else
                    {
                        LOG_ERR("Failed to update connection parameters: %d", conn_err);
                    }
                    // Continue with FOTA even if parameter update fails
                }
            }
            else
            {
                LOG_WRN("Connection not in valid state for parameter update "
                        "(state: %d)",
                        info_err == 0 ? info.state : -1);
            }
            bt_conn_unref(active_conn);
        }
        else
        {
            LOG_DBG("No active connection found for parameter update");
        }
#else
        // For secondary device, stop scanning
        int scan_err = bt_le_scan_stop();
        if (scan_err && scan_err != -EALREADY)
        {
            LOG_ERR("Failed to stop scanning: %d", scan_err);
        }
        else
        {
            LOG_INF("Scanning stopped for FOTA update");
        }
#endif

        // Send message to stop all logging and reduce activity
        generic_message_t stop_logging_msg = {};
        stop_logging_msg.sender = SENDER_BTH;
        stop_logging_msg.type = MSG_TYPE_COMMAND;
        strncpy(stop_logging_msg.data.command_str, "STOP_LOGGING", sizeof(stop_logging_msg.data.command_str) - 1);

        // Stop all logging in data module (this will close all open files)
        if (k_msgq_put(&data_msgq, &stop_logging_msg, K_NO_WAIT) != 0)
        {
            LOG_WRN("Failed to send STOP_LOGGING command to data module");
        }
        else
        {
            LOG_INF("Sent STOP_LOGGING command to data module for FOTA");
        }

        // Send FOTA_IN_PROGRESS to other modules
        generic_message_t fota_msg = {};
        fota_msg.sender = SENDER_BTH;
        fota_msg.type = MSG_TYPE_COMMAND;
        strncpy(fota_msg.data.command_str, "FOTA_IN_PROGRESS", sizeof(fota_msg.data.command_str) - 1);

        // Notify motion sensor to reduce update rate
        if (k_msgq_put(&motion_sensor_msgq, &fota_msg, K_NO_WAIT) != 0)
        {
            LOG_WRN("Failed to notify motion sensor about FOTA");
        }

        // Also stop activity if it's running
        struct foot_sensor_stop_activity_event *foot_evt = new_foot_sensor_stop_activity_event();
        if (foot_evt)
        {
            APP_EVENT_SUBMIT(foot_evt);
            LOG_INF("Submitted stop activity event for foot sensor (FOTA "
                    "starting)");
        }

        struct motion_sensor_stop_activity_event *motion_evt = new_motion_sensor_stop_activity_event();
        if (motion_evt)
        {
            APP_EVENT_SUBMIT(motion_evt);
            LOG_INF("Submitted stop activity event for motion sensor (FOTA "
                    "starting)");
        }

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Send stop activity command to secondary device using the existing
        // mechanism
        if (d2d_conn != nullptr)
        {
            LOG_INF("Sending stop activity command to secondary device for FOTA");
            uint8_t stop_value = 1; // 1 = stop activity
            int d2d_err = ble_d2d_tx_send_stop_activity_command(stop_value);
            if (d2d_err != 0)
            {
                // If sending failed for any reason, try to queue it
                if (d2d_err == -EINVAL)
                {
                    LOG_INF("D2D service discovery not complete, queuing stop "
                            "activity command");
                }
                else
                {
                    LOG_WRN("Direct send failed (%d), attempting to queue stop "
                            "activity command",
                            d2d_err);
                }
                d2d_err = ble_d2d_tx_queue_command(D2D_TX_CMD_STOP_ACTIVITY, &stop_value);
                if (d2d_err)
                {
                    LOG_ERR("Failed to queue stop activity command: %d", d2d_err);
                }
                else
                {
                    LOG_INF("Stop activity command queued for secondary device "
                            "(will be sent after D2D "
                            "discovery completes)");
                }
            }
            else
            {
                LOG_INF("Stop activity command sent to secondary device");
            }
        }
        else
        {
            LOG_DBG("No secondary device connected, skipping D2D stop command");
        }
#endif

        fota_was_active = true;
    }

    // Check if FOTA is complete or failed
    if (fota_was_active && (!progress->is_active || progress->status == 3 || progress->status == 4))
    {
        LOG_INF("FOTA update complete/failed - resuming normal BLE operation");

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Restart advertising if no connections
        struct bt_conn *active_conn = NULL;
        bt_conn_foreach(BT_CONN_TYPE_LE, find_active_conn_cb, &active_conn);

        if (!active_conn)
        {
            LOG_INF("No active connections, restarting advertising");
            bt_start_advertising(0);
        }
        else
        {
            // Restore normal connection parameters
            LOG_INF("Restoring normal connection parameters - switching to "
                    "FOREGROUND profile");
            ble_conn_params_update(active_conn, CONN_PROFILE_FOREGROUND);
            bt_conn_unref(active_conn);
        }
#else
        // For secondary device, restart scanning if not connected
        if (d2d_conn == nullptr)
        {
            LOG_INF("Not connected to primary, restarting scanning");
            start_scan();
        }
#endif

        // Notify other modules that FOTA is complete
        generic_message_t fota_complete_msg = {};
        fota_complete_msg.sender = SENDER_BTH;
        fota_complete_msg.type = MSG_TYPE_COMMAND;
        strncpy(fota_complete_msg.data.command_str, "FOTA_COMPLETE", sizeof(fota_complete_msg.data.command_str) - 1);

        k_msgq_put(&data_msgq, &fota_complete_msg, K_NO_WAIT);
        k_msgq_put(&motion_sensor_msgq, &fota_complete_msg, K_NO_WAIT);

        fota_was_active = false;
    }

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    jis_fota_progress_notify(progress);
#else
    // Secondary device: Send progress to primary via D2D
    int err = ble_d2d_tx_send_fota_progress(progress);
    if (err)
    {
        LOG_WRN("Failed to send FOTA progress via D2D: %d", err);
    }
    else
    {
        LOG_DBG("FOTA progress sent to primary via D2D");
    }

    // Also send completion status when done
    if (!progress->is_active && progress->status == 0 && progress->percent_complete == 100)
    {
        ble_d2d_tx_send_fota_complete();
    }
#endif
}

/**
 * @brief Work handler for processing error status
 */
static void error_status_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_error_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    // Handle error status updates from other modules
    error_status_msg_t *error_status = &msg.data.error_status;
    LOG_INF("Received Error Status from %s: error_code=%d, is_set=%s", get_sender_name(msg.sender),
            (int)error_status->error_code, error_status->is_set ? "SET" : "CLEAR");
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (error_status->is_set)
    {
        jis_set_err_status_notify(error_status->error_code);
    }
    else
    {
        jis_clear_err_status_notify(error_status->error_code);
    }
#else
    // Secondary device: Send status to primary via D2D
    // Convert error status to a status bitmap for D2D transmission
    static uint32_t d2d_status = 0;
    uint32_t bit_mask = 1 << ((int)error_status->error_code & 0x1F);
    if (error_status->is_set)
    {
        d2d_status |= bit_mask;
    }
    else
    {
        d2d_status &= ~bit_mask;
    }
    ble_d2d_tx_send_status(d2d_status);
#endif
}

/**
 * @brief Work handler for processing new activity log file notifications
 */
static void new_activity_log_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_log_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    // Handle new activity log file notification
    new_log_info_msg_t *log_info = &msg.data.new_hardware_log_file;
    LOG_INF("Received NEW ACTIVITY LOG FILE notification from %s:", get_sender_name(msg.sender));
    LOG_INF("  File Path: %s", log_info->file_path);
    LOG_INF("  Sequence ID: %u", log_info->file_sequence_id);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Primary device: Send activity log notifications
    jis_activity_log_available_primary_notify(log_info->file_sequence_id);
    jis_activity_log_path_primary_notify(log_info->file_path);
#else
    // Secondary device: Send to primary via D2D
    int err = d2d_tx_notify_activity_log_available(log_info->file_sequence_id);
    if (err)
    {
        LOG_ERR("Failed to send activity log available via D2D: %d", err);
    }
    err = d2d_tx_notify_activity_log_path(log_info->file_path);
    if (err)
    {
        LOG_ERR("Failed to send activity log path via D2D: %d", err);
    }
#endif
}

/**
 * @brief Work handler for processing activity metrics BLE updates
 */
static void activity_metrics_ble_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_metrics_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Handle different types of activity metrics updates
    switch (msg.type) {
        case MSG_TYPE_ACTIVITY_METRICS_BLE:
            // Handle realtime metrics from activity_metrics or realtime_metrics module
            if (msg.sender == SENDER_REALTIME_METRICS || msg.sender == SENDER_ACTIVITY_METRICS)
            {
                // Extract metrics data from message
                realtime_metrics_t metrics;
                memcpy(&metrics, &msg.data.realtime_metrics, sizeof(metrics));

                // Update the Activity Metrics Service
                ams_update_realtime_metrics(&metrics);

                LOG_DBG("Updated activity metrics: cadence=%d, pace=%d, form=%d",
                        metrics.cadence_spm, metrics.pace_sec_km, metrics.form_score);
            }
            break;
            
        case MSG_TYPE_BIOMECHANICS_EXTENDED:
            // Handle biomechanics extended data
            {
                biomechanics_extended_msg_t *bio_data = &msg.data.biomechanics_extended;
                
                // Convert to BLE structure format
                biomechanics_extended_ble_t ble_bio_data;
                ble_bio_data.pronation_left = bio_data->pronation_left;
                ble_bio_data.pronation_right = bio_data->pronation_right;
                ble_bio_data.loading_rate_left = bio_data->loading_rate_left;
                ble_bio_data.loading_rate_right = bio_data->loading_rate_right;
                ble_bio_data.arch_collapse_left = bio_data->arch_collapse_left;
                ble_bio_data.arch_collapse_right = bio_data->arch_collapse_right;
                
                // Update Activity Metrics Service with biomechanics data
                ams_update_biomechanics_extended(&ble_bio_data);
                
                LOG_DBG("Updated biomechanics: pronation L/R=%d/%d, loading L/R=%u/%u",
                        bio_data->pronation_left, bio_data->pronation_right,
                        bio_data->loading_rate_left, bio_data->loading_rate_right);
            }
            break;
            
        case MSG_TYPE_SESSION_SUMMARY:
            // Handle session summary data
            {
                session_summary_msg_t *summary = &msg.data.session_summary;
                
                // Convert to BLE structure format
                session_summary_ble_t ble_summary;
                ble_summary.total_distance_m = summary->total_distance_m;
                ble_summary.avg_pace_sec_km = summary->avg_pace_sec_km;
                ble_summary.avg_cadence_spm = summary->avg_cadence_spm;
                ble_summary.total_steps = summary->total_steps;
                ble_summary.calories_kcal = summary->calories_kcal;
                ble_summary.avg_form_score = summary->avg_form_score;
                ble_summary.duration_sec = summary->duration_sec;
                
                // Update Activity Metrics Service with session summary
                ams_update_session_summary(&ble_summary);
                
                LOG_INF("Session summary: distance=%um, pace=%us/km, cadence=%uspm, steps=%u",
                        summary->total_distance_m, summary->avg_pace_sec_km,
                        summary->avg_cadence_spm, summary->total_steps);
            }
            break;
            
        default:
            LOG_WRN("Unexpected message type %d in activity_metrics_ble_work_handler", msg.type);
            break;
    }
#else
    // Secondary device doesn't have Activity Metrics Service
    LOG_DBG("Ignoring activity metrics update on secondary device");
#endif
}

/**
 * @brief Work handler for processing device info
 */
static void device_info_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_device_info_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    // Handle device information
    device_info_msg_t *dev_info = &msg.data.device_info;
    LOG_INF("Received DEVICE INFO from %s:", get_sender_name(msg.sender));
    LOG_INF("  Manufacturer: %s", dev_info->manufacturer);
    LOG_INF("  Model: %s", dev_info->model);
    LOG_INF("  Serial: %s", dev_info->serial);
    LOG_INF("  HW Rev: %s", dev_info->hw_rev);
    LOG_INF("  FW Rev: %s", dev_info->fw_rev);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Update secondary device info in information service
    jis_update_secondary_device_info(dev_info->manufacturer, dev_info->model, dev_info->serial, dev_info->hw_rev,
                                     dev_info->fw_rev);
#else
    // Secondary device: Send to primary via D2D
    // Device info is automatically sent when secondary connects via
    // send_device_info_to_primary()
    LOG_DBG("Device info from secondary is sent automatically on connection");
#endif
}

/**
 * @brief Work handler for processing weight measurement
 */
static void weight_measurement_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_weight_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

    // Handle weight measurement
    weight_measurement_msg_t *weight_data = &msg.data.weight_measurement;
    LOG_INF("Received WEIGHT MEASUREMENT from %s: %.1f kg", get_sender_name(msg.sender),
            (double)weight_data->weight_kg);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Primary device: Aggregate weights from both devices
    if (msg.sender == SENDER_ACTIVITY_METRICS)
    {
        // This is our own weight measurement
        primary_weight_kg = weight_data->weight_kg;
        weight_request_time = k_uptime_get();

        // Cancel any pending timeout
        k_work_cancel_delayable(&weight_measurement_timeout_work);

        // Check if secondary device is connected
        if (d2d_conn != nullptr)
        {
            // Request weight from secondary device
            LOG_INF("Primary weight measured: %.1f kg, requesting secondary weight", (double)primary_weight_kg);
        }
    }
    else if (msg.sender == SENDER_D2D_SECONDARY)
    {
        // This is weight from secondary device
        secondary_weight_kg = weight_data->weight_kg;
    }
#else
    // Secondary device: Send to primary via D2D
    int err = ble_d2d_tx_send_weight_measurement(weight_data->weight_kg);
    if (err)
    {
        LOG_ERR("Failed to send weight measurement via D2D: %d", err);
    }
    else
    {
        LOG_INF("Weight measurement sent to primary via D2D");
    }
#endif
}

/**
 * @brief Work handler for processing primary battery level
 */
static void battery_level_primary_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_battery_primary_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Handle primary battery level updates
    battery_level_msg_t battery_level = msg.data.battery_level;
    LOG_DBG("Received primary battery level: %d%% from %s", battery_level.level, get_sender_name(msg.sender));
    // Update the custom battery levels characteristic
    jis_update_primary_battery(battery_level.level);
#endif // CONFIG_PRIMARY_DEVICE
}

/**
 * @brief Work handler for processing secondary battery level
 */
static void battery_level_secondary_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&bluetooth_msg_mutex, K_FOREVER);
    generic_message_t msg = pending_battery_secondary_msg;
    k_mutex_unlock(&bluetooth_msg_mutex);

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Handle secondary battery level updates
    battery_level_msg_t battery_level = msg.data.battery_level;
    LOG_DBG("Received secondary battery level: %d%% from %s", battery_level.level, get_sender_name(msg.sender));
    d2d_tx_notify_battery_level(battery_level.level);
#endif // !CONFIG_PRIMARY_DEVICE
}

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)

/**
 * @brief Update D2D connection status
 * 
 * @param connected true if secondary device is connected, false otherwise
 */
void jis_update_d2d_connection_status(bool connected)
{
    uint32_t status = device_status_bitfield;
    
    if (connected) {
        status |= STATUS_D2D_CONNECTED;
        LOG_INF("D2D connection status: CONNECTED");
    } else {
        status &= ~STATUS_D2D_CONNECTED;
        LOG_INF("D2D connection status: DISCONNECTED");
    }
    
    set_device_status(status);
}

#endif

/**
 * @brief Resets Bluetooth bonding information.
 *
 * This function resets the bonding information for Bluetooth Low Energy (BLE)
 * devices. It stops the BLE timer and unpairs the device with any bonded BLE
 * devices. Logging information is provided to indicate that the bonds are being
 * reset.
 *
 * This function is called from control_service.cpp when the reset bonds
 * characteristic is written with a value of 1.
 */
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
err_t ble_reset_bonds()
{
    LOG_INF("Reset bonds");
    k_timer_stop(&ble_timer);
    int err = bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);
    if (err)
    {
        LOG_ERR("Failed to unpair connections");
        return err_t::BLUETOOTH_ERROR;
    }
    return err_t::NO_ERROR;
}
#endif // CONFIG_PRIMARY_DEVICE

/**
 * @brief Stops Bluetooth advertising.
 *
 * This function stops Bluetooth advertising by calling the appropriate API
 * function. It is responsible for halting the transmission of advertising
 * packets.
 */
err_t bt_stop_advertising()
{
    int err = bt_le_adv_stop();
    if (err)
    {
        LOG_ERR("Failed to unpair connections");
        return err_t::BLUETOOTH_ERROR;
    }
    return err_t::NO_ERROR;
}

static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(data), MODULE_STATE_READY))
        {
            err_t init = bt_module_init();
            if (init != err_t::NO_ERROR)
            {
                module_set_state(MODULE_STATE_ERROR);
            }
            else
            {
                module_set_state(MODULE_STATE_READY);
            }
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FINAL(MODULE, module_state_event);

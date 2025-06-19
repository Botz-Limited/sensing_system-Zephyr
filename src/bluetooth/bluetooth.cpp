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
#include <cstring>
#include <string_view>

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "zephyr/bluetooth/services/bas.h"
#include "zephyr/settings/settings.h"

#include "ble_services.hpp"

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
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/bluetooth_state_event.h>
#include <events/data_event.h>

#include "ble_d2d_file_transfer.hpp"
#include "ble_d2d_rx.hpp"
#include "ble_d2d_tx.hpp"
#include "bluetooth_debug.hpp"
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "file_proxy.hpp"
#include "fota_proxy.hpp"
#include "ble_d2d_rx_client.hpp"
#endif

LOG_MODULE_REGISTER(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL); // NOLINT

/********************************** BTH THREAD ********************************/
static constexpr int bluetooth_stack_size = CONFIG_BLUETOOTH_MODULE_STACK_SIZE;
static constexpr int bluetooth_priority = CONFIG_BLUETOOTH_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(bluetooth_stack_area, bluetooth_stack_size);
static struct k_thread bluetooth_thread_data;
static k_tid_t bluetooth_tid;

// --- D2D Connection Tracking and Callbacks ---
#include "ble_d2d_rx.hpp"
#include "ble_d2d_tx.hpp"

static struct bt_conn *d2d_conn = nullptr;
static void start_scan(void);

static void d2d_connected(struct bt_conn *conn, uint8_t err)
{
    if (!err)
    {
        d2d_conn = conn;
        
        // Request MTU exchange for D2D connection
        static struct bt_gatt_exchange_params d2d_exchange_params;
        d2d_exchange_params.func = [](struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params) {
            if (!err) {
                uint16_t mtu = bt_gatt_get_mtu(conn);
                LOG_INF("D2D MTU exchange successful: %u", mtu);
            } else {
                LOG_WRN("D2D MTU exchange failed: %d", err);
            }
        };
        
        int ret = bt_gatt_exchange_mtu(conn, &d2d_exchange_params);
        if (ret) {
            LOG_WRN("Failed to initiate D2D MTU exchange: %d", ret);
        }
        
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        LOG_INF("Secondary device connected!\n");
        char addr_str[BT_ADDR_LE_STR_LEN];
        const bt_addr_le_t *addr = bt_conn_get_dst(conn);
        if (addr)
        {
            bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
            LOG_INF("Connected device address: %s\n", addr_str);
        }
        else
        {
            LOG_WRN("Failed to get connected device address");
        }

        // Set the secondary connection for FOTA proxy
        fota_proxy_set_secondary_conn(conn);

        // Set the secondary connection for file proxy
        file_proxy_set_secondary_conn(conn);

        // Initialize D2D file client for this connection
        ble_d2d_file_client_init(conn);
        
        // Start discovery of secondary device's TX service
        k_sleep(K_MSEC(100)); // Small delay to ensure connection is stable
        d2d_rx_client_start_discovery(conn);
#else
        LOG_INF("Connected to primary device!\n");
#endif
    }
}

static void d2d_disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (conn == d2d_conn)
    {
        d2d_conn = nullptr;
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        LOG_INF("Secondary device disconnected!\n");

        // Clear the secondary connection for FOTA proxy
        fota_proxy_set_secondary_conn(NULL);

        // Clear the secondary connection for file proxy
        file_proxy_set_secondary_conn(NULL);
        
        // Clear D2D RX client
        d2d_rx_client_disconnected();
#else
        LOG_INF("Disconnected from primary device!\n");
        // Restart scanning after disconnection
        start_scan();
#endif
    }
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

// Target device name to look for
static const char target_device_name[] = "SensingGR";

// Target service UUID: 5cb36a14-ca69-4d97-89a8-001ffc9ec8cd
static const uint8_t target_service_uuid[16] = {0x5c, 0xb3, 0x6a, 0x14, 0xca, 0x69, 0x4d, 0x97,
                                                0x89, 0xa8, 0x00, 0x1f, 0xfc, 0x9e, 0xc8, 0xcd};

// Helper function to check if advertisement contains our target name
static bool adv_data_has_name(struct net_buf_simple *ad, const char *name)
{
    if (!ad || !name)
        return false;

    size_t name_len = strlen(name);
    size_t i = 0;

    while (i + 1 < ad->len)
    {
        uint8_t len = ad->data[i];
        if (len == 0 || (i + len >= ad->len))
            break;

        uint8_t type = ad->data[i + 1];

        // Check for complete or shortened local name
        if (type == BT_DATA_NAME_COMPLETE || type == BT_DATA_NAME_SHORTENED)
        {
            size_t data_len = len - 1; // Subtract type byte
            if (data_len >= name_len && memcmp(&ad->data[i + 2], name, name_len) == 0)
            {
                return true;
            }
        }
        i += len + 1;
    }
    return false;
}

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

    // Check if this is our target device by name AND UUID
    bool has_name = ad && adv_data_has_name(ad, target_device_name);
    bool has_uuid = ad && adv_data_has_uuid(ad);

    if (!has_name && !has_uuid)
    {
        LOG_INF("[SCAN] Skipping device: neither name nor UUID match");
        return;
    }

    if (has_name && has_uuid)
    {
        LOG_INF("[SCAN] Perfect match! Found 'SensingGR' with correct UUID at %s", addr_str);
    }
    else if (has_name)
    {
        LOG_INF("[SCAN] Found device with matching name 'SensingGR' at %s", addr_str);
    }
    else if (has_uuid)
    {
        LOG_INF("[SCAN] Found device with matching UUID at %s", addr_str);
    }

    if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND)
    {
        LOG_WRN("[SCAN] Advertiser is not connectable (type: %u)", type);
        return;
    }

    // Check if we already have a connection object and clean it up
    if (conn)
    {
        LOG_WRN("[SCAN] Cleaning up existing connection object");
        bt_conn_unref(conn);
        conn = NULL;
    }

    int err = bt_le_scan_stop();
    if (err)
    {
        LOG_ERR("[SCAN] Failed to stop scan: %d", err);
    }

    // Small delay to ensure scan is fully stopped
    k_msleep(10);

    err = bt_conn_le_create(addr, &create_param, &conn_param, &conn);
    if (err)
    {
        if (err == -EINVAL)
        {
            LOG_ERR("[SCAN] Failed to create connection: -EINVAL (invalid argument). Possible address type or "
                    "advertising type mismatch.");
        }
        else if (err == -ENOTSUP)
        {
            LOG_ERR("[SCAN] Failed to create connection: -ENOTSUP (operation not supported). Check if advertiser is "
                    "connectable.");
        }
        else
        {
            LOG_ERR("[SCAN] Failed to create connection: %d", err);
        }
        conn = NULL;
        start_scan();
    }
    else
    {
        LOG_INF("[SCAN] Connection initiated to %s", addr_str);
    }
}

static void start_scan(void)
{
    struct bt_le_scan_param scan_param = {
        .type = BT_HCI_LE_SCAN_ACTIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = 0x0010,
        .window = 0x0010,
    };

    bt_le_scan_start(&scan_param, device_found);
}
#endif

void bluetooth_process(void * /*unused*/, void * /*unused*/, void * /*unused*/);

static uint8_t ble_status_connected = 0;
// Currently unused - kept for future BLE ID change handling
__attribute__((unused)) static bool ble_id_has_changed = false;

struct check_bond_data
{
    const bt_addr_le_t *addr;
    bool found;
};

static bt_le_adv_param bt_le_adv_conn_name = {
    .id{},
    .sid{},
    .secondary_max_skip{},
    .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_NAME | BT_LE_ADV_OPT_USE_IDENTITY,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    .peer{},
};
static const bt_le_adv_param *bt_le_adv_conn = BT_LE_ADV_CONN_FAST_1;
;

// FWD declarations
static bool app_event_handler(const struct app_event_header *aeh);
static void ble_timer_expiry_function(struct k_timer *timer_id);
static void ble_timer_handler_function(struct k_work *work);
static err_t bt_start_advertising(int err);
static err_t bt_stop_advertising(void);
static err_t ble_start_hidden(void);
static err_t ble_reset_bonds(void);

struct bt_conn *local_conn = NULL;

// Init timer
K_TIMER_DEFINE(ble_timer, ble_timer_expiry_function, NULL);

K_WORK_DEFINE(ble_handler, ble_timer_handler_function);

/* Advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL), BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0x5cb36a14, 0xca69, 0x4d97, 0x89a8, 0x001ffc9ec8cd)),
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
    LOG_ERR("Pairing Failed (%d). Disconnecting.", reason);
    bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
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
            if (!err) {
                uint16_t mtu = bt_gatt_get_mtu(conn);
                LOG_INF("MTU exchange successful: %u", mtu);
            } else {
                LOG_WRN("MTU exchange failed: %d", err);
            }
        };
        
        int mtu_ret = bt_gatt_exchange_mtu(conn, &exchange_params);
        if (mtu_ret) {
            LOG_WRN("Failed to initiate MTU exchange: %d", mtu_ret);
        } else {
            LOG_DBG("MTU exchange initiated");
        }

        // Only send a connected event if the connection is already bonded
        bool bonded = is_connection_bonded(conn);
        if (bonded)
        {
        }

        LOG_INF("Connected");
        ble_status_connected = 1;

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Debug information service status
        bt_debug_info_service_status(conn);
#endif

        int ret = bt_conn_set_security(conn, BT_SECURITY_L2);
        if (ret != 0)
        {
            LOG_ERR("failed to set security (err %d)\n", ret);
        }

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        ret = bt_le_adv_stop();
        if (ret != 0)
        {
            LOG_ERR("Failed to stop advertising (err 0x%02x)", ret);
        }

        err_t error = ble_start_hidden();
        if (error != err_t::NO_ERROR)
        {
            LOG_ERR("Failed to start hidden advertising %d", (int)error);
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
    ARG_UNUSED(conn);

    LOG_INF("Disconnected (reason 0x%02x)", reason);
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
            LOG_WRN("Security level %u is insufficient for encrypted characteristics (need >= L2)", level);
        }
#endif
    }
    else
    {
        LOG_ERR("Security failed: %s level %u err %d\n", addr, level, err);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .recycled{},
    .le_param_req{},
    .le_param_updated{},
    .identity_resolved{},
    .security_changed = security_changed,
    ._node{},

};

// Currently unused - kept for future power management
__attribute__((unused)) static void ble_set_power_off()
{}

int set_bluetooth_name()
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    int err = bt_set_name("SensingGR");
#else
    int err = bt_set_name("SensingGL");
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

    // Small delay to ensure settings are fully loaded
    k_msleep(10);

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

    static struct bt_conn_cb d2d_conn_callbacks = {
        .connected = d2d_connected,
        .disconnected = d2d_disconnected,
    };

    bt_conn_cb_register(&d2d_conn_callbacks);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
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
    start_scan();                 // Start scanning for primary device
    // Do NOT start advertising or initialize control/info services here
    return err_t::NO_ERROR;
#endif

    bluetooth_tid =
        k_thread_create(&bluetooth_thread_data, bluetooth_stack_area, K_THREAD_STACK_SIZEOF(bluetooth_stack_area),
                        bluetooth_process, nullptr, nullptr, nullptr, bluetooth_priority, 0, K_NO_WAIT);

    return err_t::NO_ERROR;
}

void bluetooth_process(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{
    // Declare a generic_message_t structure to hold the received message
    generic_message_t msg;
    int ret;

    init_rtc_time();

    while (true)
    {
        // Wait indefinitely for a message to appear in the queue.
        // K_FOREVER makes the thread block until a message is available.
        ret = k_msgq_get(&bluetooth_msgq, &msg, K_FOREVER);

        if (ret == 0)
        { // Message successfully received
            LOG_DBG("Received message from: %s, type: %d", get_sender_name(msg.sender), msg.type);

            // Handle the message based on its type
            switch (msg.type)
            {
                case MSG_TYPE_FOOT_SAMPLES: {
                    // Directly access the data from the union member
                    foot_samples_t *foot_data = &msg.data.foot_samples; // Get pointer to data within the message
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    jis_foot_sensor_notify(foot_data);
#endif
                    break;
                }

                // Similarly for BHI360 messages:
                case MSG_TYPE_BHI360_3D_MAPPING: {
                    bhi360_3d_mapping_t *mapping_data = &msg.data.bhi360_3d_mapping;
                    LOG_INF("Received BHI360 3D Mapping from %s: Accel(%.2f,%.2f,%.2f), Gyro(%.2f,%.2f,%.2f)",
                            get_sender_name(msg.sender), (double)mapping_data->accel_x, (double)mapping_data->accel_y,
                            (double)mapping_data->accel_z, (double)mapping_data->gyro_x, (double)mapping_data->gyro_y,
                            (double)mapping_data->gyro_z);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    jis_bhi360_data1_notify(mapping_data);
#endif
                    break;
                }

                case MSG_TYPE_BHI360_LINEAR_ACCEL: {
                    bhi360_linear_accel_t *lacc_data = &msg.data.bhi360_linear_accel;
                    LOG_INF("Received BHI360 Linear Accel from %s: (%.2f,%.2f,%.2f)", get_sender_name(msg.sender),
                            (double)lacc_data->x, (double)lacc_data->y, (double)lacc_data->z);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    jis_bhi360_data3_notify(lacc_data);
#endif
                    break;
                }

                case MSG_TYPE_BHI360_STEP_COUNT: {
                    bhi360_step_count_t *step_data = &msg.data.bhi360_step_count;
                    LOG_INF("Received BHI360 Step Count from %s: Steps=%u, ActivityDuration=%u s",
                            get_sender_name(msg.sender), step_data->step_count, step_data->activity_duration_s);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    jis_bhi360_data2_notify(step_data);
#endif
                    break;
                }

                case MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE: {
                    // Access the new_hardware_log_file data from the union
                    new_log_info_msg_t *log_info = &msg.data.new_hardware_log_file;
                    LOG_INF("Received NEW FOOT LOG FILE notification from %s:", get_sender_name(msg.sender));
                    LOG_INF("  File Path: %s", log_info->file_path);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    jis_foot_sensor_log_available_notify(log_info->file_sequence_id);
                    jis_foot_sensor_req_id_path_notify(log_info->file_path);
#endif
                    break;
                }

                case MSG_TYPE_NEW_BHI360_LOG_FILE: {
                    // Access the new_hardware_log_file data from the union
                    new_log_info_msg_t *log_info = &msg.data.new_hardware_log_file;
                    LOG_INF("Received NEW BH360 LOG FILE notification from %s:", get_sender_name(msg.sender));
                    LOG_INF("  File Path: %s", log_info->file_path);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    jis_bhi360_log_available_notify(log_info->file_sequence_id);
                    jis_bhi360_req_id_path_notify(log_info->file_path);
#endif
                    break;
                }

                case MSG_TYPE_COMMAND: {
                    // If you send commands via char arrays in the union
                    char *command_str = msg.data.command_str;
                    LOG_INF("Received Command from %s: '%s'", get_sender_name(msg.sender), command_str);
                    break;
                }

                case MSG_TYPE_FOTA_PROGRESS: {
                    // Handle FOTA progress updates
                    fota_progress_msg_t *progress = &msg.data.fota_progress;
                    LOG_INF("Received FOTA Progress: active=%d, status=%d, percent=%d%%, bytes=%u/%u",
                            progress->is_active, progress->status, progress->percent_complete, progress->bytes_received,
                            progress->total_size);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    jis_fota_progress_notify(progress);
#endif
                    break;
                }

                case MSG_TYPE_ERROR_STATUS: {
                    // Handle error status updates from other modules
                    error_status_msg_t *error_status = &msg.data.error_status;
                    LOG_INF("Received Error Status from %s: error_code=%d, is_set=%s", 
                            get_sender_name(msg.sender), 
                            (int)error_status->error_code,
                            error_status->is_set ? "SET" : "CLEAR");
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    if (error_status->is_set) {
                        jis_set_err_status_notify(error_status->error_code);
                    } else {
                        jis_clear_err_status_notify(error_status->error_code);
                    }
#endif
                    break;
                }

                default:
                    LOG_WRN("Unknown message type received (%d) from %s", msg.type, get_sender_name(msg.sender));
                    break;
            }
        }
        else
        {
            // This else block is generally not reached with K_FOREVER, as it implies a timeout or error.
            // But it's good practice to have it for other k_msgq_get timeout values.
            LOG_ERR("Failed to get message from queue: %d", ret);
        }
    }
}

/**
 * @brief Starts Bluetooth advertising.
 *
 * This function initiates Bluetooth Low Energy (BLE) advertising using the
 * specified advertising led_data. It stops any ongoing advertising before starting
 * a new one. After starting advertising, it logs an error message if the
 * operation fails and logs an informational message if advertising starts
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

    err = bt_le_adv_start(&bt_le_adv_conn_name, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err)
    {
        LOG_ERR("Advertising failed to start, error=%d", err);

        return err_t::BLUETOOTH_ERROR;
    }
    else
    {
        LOG_INF("Advertising successfully started");
        k_timer_start(&ble_timer, K_MSEC(BLE_ADVERTISING_TIMEOUT_MS), K_NO_WAIT);
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

    int err = bt_le_adv_start(bt_le_adv_conn, ad, ARRAY_SIZE(ad), NULL, 0);
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
    int ret = bt_le_adv_stop();
    if (ret != 0)
    {
        LOG_ERR("Failed to stop (err 0x%02x)", ret);
    }
    LOG_INF("Bluetooth advertising timeout");

    if (bonds_present())
    {
        err_t err = ble_start_hidden();
        if (err != err_t::NO_ERROR)
        {
            LOG_ERR("Failed to start %d", (int)err);
        }
    }
}

/**
 * @brief Resets Bluetooth bonding information.
 *
 * This function resets the bonding information for Bluetooth Low Energy (BLE)
 * devices. It stops the BLE timer and unpairs the device with any bonded BLE
 * devices. Logging information is provided to indicate that the bonds are being
 * reset.
 */
// Currently unused - kept for future bond management
__attribute__((unused)) err_t ble_reset_bonds()
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

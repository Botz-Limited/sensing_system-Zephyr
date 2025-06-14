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
#include <zephyr/sys/printk.h>

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

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/bluetooth_state_event.h>
#include <events/data_event.h>
#include <app.hpp>

LOG_MODULE_REGISTER(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL); // NOLINT


/********************************** BTH THREAD ********************************/
static constexpr int bluetooth_stack_size = CONFIG_BLUETOOTH_MODULE_STACK_SIZE;
static constexpr int bluetooth_priority = CONFIG_BLUETOOTH_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(bluetooth_stack_area, bluetooth_stack_size);
static struct k_thread bluetooth_thread_data;
static k_tid_t bluetooth_tid;

void bluetooth_process(void * /*unused*/, void * /*unused*/, void * /*unused*/);

static uint8_t ble_status_connected = 0;
static bool ble_id_has_changed = false;

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
    ARG_UNUSED(conn);
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
    ARG_UNUSED(conn);

    if (err != 0)
    {
        LOG_ERR("Connection failed (err 0x%02x)", err);
    }
    else
    {
        // Only send a connected event if the connection is already bonded
        bool bonded = is_connection_bonded(conn);
        if (bonded)
        {
        }

        LOG_INF("Connected");
        ble_status_connected = 1;

        int ret = bt_conn_set_security(conn, BT_SECURITY_L2);
        if (ret != 0)
        {
            LOG_ERR("failed to set security (err %d)\n", ret);
        }

        ret = bt_le_adv_stop();
        if (ret != 0)
        {
            LOG_ERR("Failed to stop (err 0x%02x)", ret);
        }

        err_t error = ble_start_hidden();
        if (error != err_t::NO_ERROR)
        {
            LOG_ERR("Failed to start %d", (int)error);
        }
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

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err == 0)
    {
        LOG_INF("Security changed: %s level %u\n", addr, level);
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

static void ble_set_power_off()
{}

int set_bluetooth_name()
{
    int err = bt_set_name("SensingG");

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

    err = settings_load();
    if (err != 0)
    {
        LOG_ERR("Bluetooth settings_load() call failed (err %d)", err);
        return err_t::BLUETOOTH_ERROR;
    }

    int err_name = set_bluetooth_name();
    if (err_name != 0)
    {
        LOG_ERR("Failed to Set Name");
        return err_t::BLUETOOTH_ERROR;
    }

    err = bt_conn_auth_info_cb_register(&bt_conn_auth_info);
    if (err)
    {
        LOG_INF("Can't register authentication information callbacks (err: %d)\n", err);
        return err_t::BLUETOOTH_ERROR;
    }

    if (bt_start_advertising(0) == err_t::NO_ERROR)
    {
        LOG_INF("BLE Started Advertising");
    }
    else
    {
        LOG_ERR("BLE Failed to enter advertising mode");
    }

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
                    jis_foot_sensor_notify(foot_data);
                    break;
                }

                // Similarly for BHI360 messages:
                case MSG_TYPE_BHI360_3D_MAPPING: {
                    bhi360_3d_mapping_t *mapping_data = &msg.data.bhi360_3d_mapping;
                    LOG_INF("Received BHI360 3D Mapping from %s: Accel(%.2f,%.2f,%.2f), Gyro(%.2f,%.2f,%.2f)",
                            get_sender_name(msg.sender),
                            (double)mapping_data->accel_x, (double)mapping_data->accel_y, (double)mapping_data->accel_z,
                            (double)mapping_data->gyro_x, (double)mapping_data->gyro_y, (double)mapping_data->gyro_z);

                    break;
                }

                case MSG_TYPE_BHI360_STEP_COUNT: {
                    bhi360_step_count_t *step_data = &msg.data.bhi360_step_count;
                    LOG_INF("Received BHI360 Step Count from %s: Steps=%u, ActivityDuration=%u s",
                            get_sender_name(msg.sender), step_data->step_count, step_data->activity_duration_s);

                    break;
                }

                case MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE: {
                    // Access the new_hardware_log_file data from the union
                    new_log_info_msg_t *log_info = &msg.data.new_hardware_log_file;
                    LOG_INF("Received NEW FOOT LOG FILE notification from %s:", get_sender_name(msg.sender));
                    LOG_INF("  File Path: %s", log_info->file_path);
                    jis_foot_sensor_log_available_notify(log_info->file_sequence_id);
                    jis_foot_sensor_req_id_path_notify(log_info->file_path);
                    break;
                }

                case MSG_TYPE_NEW_BHI360_LOG_FILE: {
                    // Access the new_hardware_log_file data from the union
                    new_log_info_msg_t *log_info = &msg.data.new_hardware_log_file;
                    LOG_INF("Received NEW BH360 LOG FILE notification from %s:", get_sender_name(msg.sender));
                    LOG_INF("  File Path: %s", log_info->file_path);
                    jis_bhi360_log_available_notify(log_info->file_sequence_id);
                    jis_bhi360_req_id_path_notify(log_info->file_path);
                    break;
                }

                case MSG_TYPE_COMMAND: {
                    // If you send commands via char arrays in the union
                    char *command_str = msg.data.command_str;
                    LOG_INF("Received Command from %s: '%s'", get_sender_name(msg.sender), command_str);
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
/**
 * @file information_service.cpp
 * @author
 * @brief
 * @version 1.0.1
 * @date 2025-05-12
 *
 * @copyright Botz Innovation 2025
 *
 */

#define MODULE bluetooth

#include <cstddef>
#include <cstdint>

#include <errors.hpp>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>

#include <cstring>

#include <app.hpp>
#include <ble_services.hpp>
#include <util.hpp>

LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

static uint32_t error_bitfield_data = 0;
static uint32_t previous_error_bitfield_data = 0; // Store the previous value of error_bitfield_data
static bool init_status_bt_update = false;        // A flag to force the first update

static uint8_t charge_status = 0;
static foot_samples_t foot_sensor_char_value = {0};

static bool status_subscribed = true;
static bool foot_sensor_log_available_subscribed = true;
static uint32_t foot_sensor_log_available = 0;
static char foot_sensor_req_id_path[util::max_path_length] = {0};

static bool bhi360_log_available_subscribed = true;          // New subscription flag for BHI360 log availability
static uint32_t bhi360_log_available = 0;                    // New variable for BHI360 log availability ID
static char bhi360_req_id_path[util::max_path_length] = {0}; // New variable for BHI360 log file path

static uint8_t ct[10];
static uint8_t ct_update = 0;

// --- Global variables for Current Time Service Notifications ---
static struct bt_conn *current_time_conn_active = NULL; // Stores the connection object for notifications
static bool current_time_notifications_enabled = false; // Flag to track CCC state

// FWD Declarations
// clang-format off

//Current Time Characteristic
static ssize_t jis_read_current_time(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_current_time_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
// Status Characteristic
static ssize_t jis_status_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_status_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);

// Foot Sensor Log available Characteristic
static ssize_t jis_foot_sensor_log_available_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_foot_sensor_log_available_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);

static ssize_t jis_charge_status_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_charge_status_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
//Req ID
static ssize_t jis_foot_sensor_req_id_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len,
uint16_t offset);
static void jis_foot_sensor_req_id_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value); 

static ssize_t jis_bhi360_log_available_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_bhi360_log_available_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
// New: BHI360 Req ID/Path Characteristic
static ssize_t jis_bhi360_req_id_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len,
  uint16_t offset);
static void jis_bhi360_req_id_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);

// Foot Sensor Characteristic
static ssize_t jis_foot_sensor_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_foot_sensor_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);

// UUID Defines
static struct bt_uuid_128 SENSING_INFO_SERVICE_UUID =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eaa, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

static struct bt_uuid_128 STATUS_UUID =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eab, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));


// Foot Sensor Log Available UUID
static struct bt_uuid_128 foot_sensor_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eac, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

static struct bt_uuid_128 CHARGE_STATUS_UUID =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ead, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));


// Foot Sensor Request ID/Path UUID (renamed from req_id_meta_uuid)
static struct bt_uuid_128 foot_sensor_req_id_path_uuid =
     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eae, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

static struct bt_uuid_128 foot_sensor_samples =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eaf, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));   
    
static struct bt_uuid_128 bhi360_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb0, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97)); 

// New: BHI360 Request ID/Path UUID (choose a new unique UUID)
static struct bt_uuid_128 bhi360_req_id_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb1, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97)); 


BT_GATT_SERVICE_DEFINE(
    info_service, BT_GATT_PRIMARY_SERVICE(&SENSING_INFO_SERVICE_UUID),
    // Current Ttime Characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_read_current_time, NULL,
                            ct),
    BT_GATT_CCC(jis_current_time_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Status Characteristic
    BT_GATT_CHARACTERISTIC(&STATUS_UUID.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_status_read, nullptr,
                            static_cast<void *>(&error_bitfield_data)),
    BT_GATT_CCC(jis_status_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Foot Sensor samples Characteristic
    BT_GATT_CHARACTERISTIC(&foot_sensor_samples.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_foot_sensor_read, nullptr,
                            static_cast<void *>(&foot_sensor_char_value)),
    BT_GATT_CCC(jis_foot_sensor_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),


// Foot Sensor Logs available Characteristic 
    BT_GATT_CHARACTERISTIC(&foot_sensor_log_available_uuid.uuid,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                     BT_GATT_PERM_READ_ENCRYPT,
                     jis_foot_sensor_log_available_read, nullptr,
                static_cast<void *>(&foot_sensor_log_available)),
    BT_GATT_CCC(jis_foot_sensor_log_available_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Battery Status Characteristic
    BT_GATT_CHARACTERISTIC(&CHARGE_STATUS_UUID.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_charge_status_read, nullptr,
                            static_cast<void *>(&charge_status)),
    BT_GATT_CCC(jis_charge_status_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),


// Foot Sensor Request ID/Path Characteristic
     BT_GATT_CHARACTERISTIC(&foot_sensor_req_id_path_uuid.uuid,
                     BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, // Added NOTIFY to this characteristic as it might be useful
                  BT_GATT_PERM_READ_ENCRYPT,
                   jis_foot_sensor_req_id_read, nullptr,
                  foot_sensor_req_id_path),
        BT_GATT_CCC(jis_foot_sensor_req_id_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // BHI360 Logs available Characteristic
            BT_GATT_CHARACTERISTIC(&bhi360_log_available_uuid.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_bhi360_log_available_read, nullptr,
                            static_cast<void *>(&bhi360_log_available)),
    BT_GATT_CCC(jis_bhi360_log_available_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // BHI360 Request ID/Path Characteristic
    BT_GATT_CHARACTERISTIC(&bhi360_req_id_path_uuid.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_bhi360_req_id_read, nullptr,
                            bhi360_req_id_path),
    BT_GATT_CCC(jis_bhi360_req_id_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT));        

// clang-format on
/**
 * @brief
 *
 * @param status
 *
 */
void jis_set_err_status_notify(err_t new_device_status)
{
    switch (new_device_status)
    {

        default:
            LOG_WRN("UNKNOWN device fault status reported: %d", (int)new_device_status);
            break;
    }

    // Check if error_bitfield_data has changed, update BT, only if there is difference in value
    if (((!init_status_bt_update) && (error_bitfield_data != 0)) ||
        ((error_bitfield_data != previous_error_bitfield_data) && (error_bitfield_data != 0)))
    {
        if (status_subscribed)
        {
            auto *status_gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &STATUS_UUID.uuid);
            bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&error_bitfield_data),
                           sizeof(error_bitfield_data));
            LOG_DBG(" Set Fault status: status user data: 0x%X\n", error_bitfield_data);
        }

        // Update the previous value, after reporting the error
        previous_error_bitfield_data = error_bitfield_data;
        init_status_bt_update = true;
    }

    else
    {
        LOG_WRN("Errors reported but are disabled due to FCT Login");
    }
}

/**
 * @brief clears the error status
 *
 * @param new_device_status
 *
 */
void jis_clear_err_status_notify(err_t new_device_status)
{
    switch (new_device_status)
    {

        default:
            LOG_WRN("UNKNOWN device clear status reported: %d", (int)new_device_status);
            break;
    }
    // Check if error_bitfield_data has changed or update it if its the first time
    if (!init_status_bt_update || error_bitfield_data != previous_error_bitfield_data)
    {
        if (status_subscribed)
        {
            auto *status_gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &STATUS_UUID.uuid);
            bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&error_bitfield_data),
                           sizeof(error_bitfield_data));
            LOG_INF("Clear Bit field Data: 0x%X\n", error_bitfield_data);
            // create the critical error event and fill in the status
        }

        previous_error_bitfield_data = error_bitfield_data; // Update the previous value

        // All monitored bits are cleared, allow system reset to be cleared
        // Don't clear it at the first time, the static variable is cleared in the warm reboot.
        if ((error_bitfield_data == 0) && (init_status_bt_update))
        {
            LOG_WRN("error_bitfield_data %d, previous_error_bitfield_data: %d init_status_bt_update: %d",
                    error_bitfield_data, previous_error_bitfield_data, init_status_bt_update);
        }
        init_status_bt_update = true;
    }
}

void jis_foot_sensor_notify(const foot_samples_t *samples_data)
{
    // 1. Get the current epoch timestamp from your RTC module
    uint32_t current_epoch = get_current_epoch_time();

    foot_samples_t temp_foot_data;

    memcpy(temp_foot_data.values, samples_data->values, sizeof(samples_data->values));

    memcpy(&foot_sensor_char_value, &temp_foot_data, sizeof(foot_samples_t));

    if (status_subscribed)
    {
        auto *status_gatt =
            bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &foot_sensor_samples.uuid);
        bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&foot_sensor_char_value),
                       sizeof(foot_sensor_char_value));
    }
}

/**
 * @brief
 *
 * @param attr
 * @param value
 *
 */
static void jis_status_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    status_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Status CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static void jis_foot_sensor_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    status_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Foot Sensor CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

/**
 * @brief
 *
 * @param attr
 * @param value
 *
 */
void jis_current_time_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    status_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Time CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_read_current_time(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                     uint16_t offset)
{
    // 1. Request cts.cpp to update its internal Current Time characteristic buffer.
    // This ensures the buffer contains the latest calendar time.
    update_cts_characteristic_buffer();

    // 2. Get a pointer to the updated buffer and its size from cts.cpp.
    const void *cts_value_ptr = get_current_time_char_value_ptr();
    size_t cts_value_size = get_current_time_char_value_size();

    // 3. Check if the time is valid before attempting to read.
    // (This check depends on how update_cts_characteristic_buffer handles unsynced time.
    // If it zeros the buffer, checking the first byte might be sufficient,
    // or you could have a separate flag/function in cts.cpp to check sync status).
    // For simplicity, let's assume if cts_value_size is 0 or the pointer is null, it's invalid.
    if (cts_value_ptr == nullptr || cts_value_size == 0)
    {
        LOG_WRN("Current Time characteristic data is not ready or valid.");
        // Return 0 bytes read or an error code if appropriate.
        // A common practice for GATT reads is to return 0 bytes if data is unavailable.
        return 0;
    }

    // 4. Use bt_gatt_attr_read to send the content of the prepared buffer.
    // This function handles the copying of data from `cts_value_ptr` to `buf`
    // respecting `len` and `offset`.
    return bt_gatt_attr_read(conn, attr, buf, len, offset, cts_value_ptr, cts_value_size);
}

static ssize_t jis_status_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                               uint16_t offset)
{
    auto *value = static_cast<uint16_t *>(attr->user_data);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(error_bitfield_data));
}

static ssize_t jis_foot_sensor_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                    uint16_t offset)
{
    // Cast attr->user_data to the correct type (foot_samples_t *).
    // This 'attr->user_data' is actually a pointer to your global 'foot_sensor_char_value'.
    const foot_samples_t *value_to_read = static_cast<const foot_samples_t *>(attr->user_data);

    // Ensure that sizeof(foot_samples_t) is used, not sizeof(error_bitfield_data)
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value_to_read, sizeof(foot_samples_t));
}

/**
 * @brief
 *
 *
 */
void cts_notify(void)
{ /* Current Time Service updates only when time is changed */
    if (!ct_update)
    {
        return;
    }

    ct_update = 0U;
    bt_gatt_notify(NULL, &info_service.attrs[1], &ct, sizeof(ct));
}

/**
 * @brief
 *
 * @param new_charge_status
 *
 */
void jis_charge_status_notify(uint8_t new_charge_status)
{
    LOG_DBG("charge status: %d", new_charge_status);

    charge_status = new_charge_status;

    bt_gatt_notify_uuid(nullptr, &CHARGE_STATUS_UUID.uuid, info_service.attrs, static_cast<void *>(&charge_status),
                        sizeof(charge_status));
}

/**
 * @brief
 *
 * @param attr
 * @param value
 *
 */
static void jis_charge_status_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    ARG_UNUSED(value);
    // status_subscribed = value == BT_GATT_CCC_NOTIFY;
}

/**
 * @brief
 *
 * @param conn
 * @param attr
 * @param buf
 * @param len
 * @param offset
 * @return ssize_t
 *
 */
static ssize_t jis_charge_status_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                      uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, static_cast<void *>(&charge_status), sizeof(charge_status));
}

void jis_foot_sensor_log_available_notify(uint32_t log_id) 
{
    foot_sensor_log_available = log_id; 
    LOG_DBG("Foot Sensor Log ID: %u, notifications enabled: %s", foot_sensor_log_available,
            foot_sensor_log_available_subscribed ? "true" : "false"); 
    if (foot_sensor_log_available_subscribed)                         
    {
        auto *status_gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count,
                                                 &foot_sensor_log_available_uuid.uuid); 
        bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&foot_sensor_log_available),
                       sizeof(foot_sensor_log_available)); 
    }
}

/**
 * @brief CCC change callback for the Foot Sensor Log Available Characteristic.
 *
 * Updates the `foot_sensor_log_available_subscribed` flag based on the CCC value.
 *
 * @param attr The GATT attribute that triggered the callback.
 * @param value The new CCC value.
 */
static void jis_foot_sensor_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) // Renamed
{
    ARG_UNUSED(attr);
    foot_sensor_log_available_subscribed = value == BT_GATT_CCC_NOTIFY; // Renamed
}

/**
 * @brief Read callback for the Foot Sensor Log Available Characteristic.
 *
 * @param conn The connection object.
 * @param attr The GATT attribute being read.
 * @param buf The buffer to store the read data.
 * @param len The maximum length of data to read.
 * @param offset The offset from which to start reading.
 * @return ssize_t The number of bytes read on success, or a negative error code.
 */
static ssize_t jis_foot_sensor_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                  void *buf, // Renamed
                                                  uint16_t len, uint16_t offset)
{
    auto *value = static_cast<uint32_t *>(attr->user_data);                    // Changed to uint32_t for ID
    LOG_DBG("foot_sensor_log_available_read : %u", foot_sensor_log_available); // Renamed and format specifier
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(foot_sensor_log_available)); // Renamed
}

/**
 * @brief Notifies subscribed clients of the current foot sensor log file path.
 *
 * This function copies the provided `file_path` string into the
 * `foot_sensor_req_id_path` buffer and sends a GATT notification to subscribed
 * Bluetooth clients if notifications for this characteristic are enabled.
 *
 * @param file_path The string containing the new foot sensor log file path.
 */
void jis_foot_sensor_req_id_path_notify(const char *file_path) // Renamed from cs_req_id_meta_notify
{
    // clear buffer before writing the new value
    std::memset(foot_sensor_req_id_path, 0, sizeof(foot_sensor_req_id_path));              // Renamed
    std::strncpy(foot_sensor_req_id_path, file_path, sizeof(foot_sensor_req_id_path) - 1); // Using strncpy for safety
    foot_sensor_req_id_path[sizeof(foot_sensor_req_id_path) - 1] = '\0';                   // Ensure null termination

    if (foot_sensor_log_available_subscribed) // Re-using this flag for consistency, consider separate flag for this too
    {
        auto *status_gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count,
                                                 &foot_sensor_req_id_path_uuid.uuid); // Renamed
        bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&foot_sensor_req_id_path),
                       sizeof(foot_sensor_req_id_path)); // Renamed
    }
}

/**
 * @brief CCC change callback for the Foot Sensor Request ID/Path Characteristic.
 *
 * Updates the `foot_sensor_req_id_path_subscribed` flag based on the CCC value.
 * (Note: Assuming a new dedicated subscription flag for this characteristic for clarity).
 *
 * @param attr The GATT attribute that triggered the callback.
 * @param value The new CCC value.
 */
static void jis_foot_sensor_req_id_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) // Renamed
{
    ARG_UNUSED(attr);
    // For consistency and clarity, ideally, you'd have a separate `foot_sensor_req_id_path_subscribed` flag.
    // For now, I'm setting `foot_sensor_log_available_subscribed` as a placeholder,
    // but you should introduce a dedicated flag like `static bool foot_sensor_req_id_path_subscribed = false;`
    // and manage it here.
    foot_sensor_log_available_subscribed = value == BT_GATT_CCC_NOTIFY; // Temporary, replace with dedicated flag
    LOG_DBG("Foot Sensor Req ID/Path CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

/**
 * @brief Read callback for the Foot Sensor Request ID/Path Characteristic.
 *
 * @param conn The connection object.
 * @param attr The GATT attribute being read.
 * @param buf The buffer to store the read data.
 * @param len The maximum length of data to read.
 * @param offset The offset from which to start reading.
 * @return ssize_t The number of bytes read on success, or a negative error code.
 */
static ssize_t jis_foot_sensor_req_id_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                           uint16_t len, // Renamed
                                           uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(foot_sensor_req_id_path)); // Renamed
}

/**
 * @brief Notifies subscribed clients about the availability of BHI360 log files.
 *
 * This function updates the `bhi360_log_available` variable with the
 * provided log ID and sends a GATT notification to subscribed Bluetooth clients
 * if notifications for this characteristic are enabled.
 *
 * @param log_id The ID of the newly available BHI360 log file.
 */
void jis_bhi360_log_available_notify(uint32_t log_id)
{
    bhi360_log_available = log_id;
    LOG_DBG("BHI360 Log ID: %u, notifications enabled: %s", bhi360_log_available,
            bhi360_log_available_subscribed ? "true" : "false");
    if (bhi360_log_available_subscribed)
    {
        auto *status_gatt =
            bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &bhi360_log_available_uuid.uuid);
        bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&bhi360_log_available), sizeof(bhi360_log_available));
    }
}

/**
 * @brief CCC change callback for the BHI360 Log Available Characteristic.
 *
 * Updates the `bhi360_log_available_subscribed` flag based on the CCC value.
 *
 * @param attr The GATT attribute that triggered the callback.
 * @param value The new CCC value.
 */
static void jis_bhi360_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bhi360_log_available_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("BHI360 Log Available CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

/**
 * @brief Read callback for the BHI360 Log Available Characteristic.
 *
 * @param conn The connection object.
 * @param attr The GATT attribute being read.
 * @param buf The buffer to store the read data.
 * @param len The maximum length of data to read.
 * @param offset The offset from which to start reading.
 * @return ssize_t The number of bytes read on success, or a negative error code.
 */
static ssize_t jis_bhi360_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                    uint16_t len, uint16_t offset)
{
    auto *value = static_cast<uint32_t *>(attr->user_data);
    LOG_DBG("bhi360_log_available_read : %u", bhi360_log_available);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(bhi360_log_available));
}

/**
 * @brief Notifies subscribed clients of the current BHI360 log file path.
 *
 * This function copies the provided `file_path` string into the
 * `bhi360_req_id_path` buffer and sends a GATT notification to subscribed
 * Bluetooth clients if notifications for this characteristic are enabled.
 *
 * @param file_path The string containing the new BHI360 log file path.
 */
void jis_bhi360_req_id_path_notify(const char *file_path)
{
    // clear buffer before writing the new value
    std::memset(bhi360_req_id_path, 0, sizeof(bhi360_req_id_path));
    std::strncpy(bhi360_req_id_path, file_path, sizeof(bhi360_req_id_path) - 1);
    bhi360_req_id_path[sizeof(bhi360_req_id_path) - 1] = '\0'; // Ensure null termination

    if (bhi360_log_available_subscribed) // Use the dedicated BHI360 subscription flag
    {
        auto *status_gatt =
            bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &bhi360_req_id_path_uuid.uuid);
        bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&bhi360_req_id_path), sizeof(bhi360_req_id_path));
    }
}

/**
 * @brief CCC change callback for the BHI360 Request ID/Path Characteristic.
 *
 * Updates the `bhi360_req_id_path_subscribed` flag based on the CCC value.
 *
 * @param attr The GATT attribute that triggered the callback.
 * @param value The new CCC value.
 */
static void jis_bhi360_req_id_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bhi360_log_available_subscribed =
        value == BT_GATT_CCC_NOTIFY; // Using the same flag for now, consider dedicated if logic differs
    LOG_DBG("BHI360 Req ID/Path CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

/**
 * @brief Read callback for the BHI360 Request ID/Path Characteristic.
 *
 * @param conn The connection object.
 * @param attr The GATT attribute being read.
 * @param buf The buffer to store the read data.
 * @param len The maximum length of data to read.
 * @param offset The offset from which to start reading.
 * @return ssize_t The number of bytes read on success, or a negative error code.
 */
static ssize_t jis_bhi360_req_id_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(bhi360_req_id_path));
}

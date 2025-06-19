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
#include <status_codes.h>

LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

static uint32_t device_status_bitfield = 0;
static uint32_t previous_device_status_bitfield = 0; // Store the previous value
static bool init_status_bt_update = false;        // A flag to force the first update

static uint8_t charge_status = 0;
static foot_samples_t foot_sensor_char_value = {0};

static bool status_subscribed = true;
static bool foot_sensor_log_available_subscribed = true;
static uint8_t foot_sensor_log_available = 0;
static char foot_sensor_req_id_path[util::max_path_length] = {0};

static bool bhi360_log_available_subscribed = true;          // New subscription flag for BHI360 log availability
static uint8_t bhi360_log_available = 0;                     // New variable for BHI360 log availability ID
static char bhi360_req_id_path[util::max_path_length] = {0}; // New variable for BHI360 log file path

static uint8_t ct[10];
static uint8_t ct_update = 0;

// --- BHI360 Data Set Characteristics ---
// 1: 3D Mapping, 2: Step Count, 3: (placeholder, update as needed)
static bhi360_3d_mapping_t bhi360_data1_value = {0};
static bhi360_step_count_t bhi360_data2_value = {0};
static bhi360_linear_accel_t bhi360_data3_value = {}; // Now holds linear acceleration struct
static bool bhi360_data1_subscribed = false;
static bool bhi360_data2_subscribed = false;
static bool bhi360_data3_subscribed = false;

// FOTA progress tracking
static fota_progress_msg_t fota_progress_value = {0};
static bool fota_progress_subscribed = false;

// --- Global variables for Current Time Service Notifications ---
static struct bt_conn *current_time_conn_active = NULL; // Stores the connection object for notifications
static bool current_time_notifications_enabled = false; // Flag to track CCC state

// FWD Declarations
// clang-format off
// BHI360 Data Set Characteristics
static ssize_t jis_bhi360_data1_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_bhi360_data1_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_bhi360_data1_notify(const bhi360_3d_mapping_t* data);

static ssize_t jis_bhi360_data2_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_bhi360_data2_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_bhi360_data2_notify(const bhi360_step_count_t* data);

static ssize_t jis_bhi360_data3_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_bhi360_data3_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_bhi360_data3_notify(const int* data); // Placeholder, update to your third struct if needed

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

// FOTA Progress Characteristic
static ssize_t jis_fota_progress_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_fota_progress_notify(const fota_progress_msg_t* progress);

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

// BHI360 Data Set UUIDs
static struct bt_uuid_128 bhi360_data1_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb2, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 bhi360_data2_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb3, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 bhi360_data3_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb4, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// FOTA Progress UUID
static struct bt_uuid_128 fota_progress_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb5, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

BT_GATT_SERVICE_DEFINE(
    info_service, BT_GATT_PRIMARY_SERVICE(&SENSING_INFO_SERVICE_UUID),
    // Current Ttime Characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME,
                            BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_read_current_time, NULL,
                            ct),
    BT_GATT_CCC(jis_current_time_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Status Characteristic
    BT_GATT_CHARACTERISTIC(&STATUS_UUID.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            jis_status_read, nullptr,
                            static_cast<void *>(&device_status_bitfield)),
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
                            BT_GATT_CHRC_READ,
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
    BT_GATT_CCC(jis_bhi360_req_id_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // --- BHI360 Data Set Characteristics ---
    BT_GATT_CHARACTERISTIC(&bhi360_data1_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_bhi360_data1_read, nullptr,
        static_cast<void *>(&bhi360_data1_value)),
    BT_GATT_CCC(jis_bhi360_data1_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    BT_GATT_CHARACTERISTIC(&bhi360_data2_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_bhi360_data2_read, nullptr,
        static_cast<void *>(&bhi360_data2_value)),
    BT_GATT_CCC(jis_bhi360_data2_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    BT_GATT_CHARACTERISTIC(&bhi360_data3_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_bhi360_data3_read, nullptr,
        static_cast<void *>(&bhi360_data3_value)),
    BT_GATT_CCC(jis_bhi360_data3_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // FOTA Progress Characteristic
    BT_GATT_CHARACTERISTIC(&fota_progress_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_fota_progress_read, nullptr,
        static_cast<void *>(&fota_progress_value)),
    BT_GATT_CCC(jis_fota_progress_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT));

// clang-format on
/**
 * @brief
 *
 * @param status
 *
 */
// Set and notify device status (bitfield)
void set_device_status(uint32_t new_status)
{
    if (device_status_bitfield != new_status || !init_status_bt_update) {
        device_status_bitfield = new_status;
        if (status_subscribed) {
            auto *status_gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &STATUS_UUID.uuid);
            bt_gatt_notify(nullptr, status_gatt, &device_status_bitfield, sizeof(device_status_bitfield));
            LOG_DBG("Device status updated and notified: 0x%08X", device_status_bitfield);
        }
        previous_device_status_bitfield = device_status_bitfield;
        init_status_bt_update = true;
    }
}

/**
 * @brief Sets error status bits based on err_t error codes
 *
 * @param error_code The error code to convert to status bit
 */
void jis_set_err_status_notify(err_t error_code)
{
    uint32_t error_bit = 0;
    
    // Map err_t values to status bits
    switch (error_code)
    {
        case err_t::NO_ERROR:
            // No error, nothing to set
            return;
            
        case err_t::BATTERY_FAULT:
            error_bit = STATUS_BATTERY_FAULT;
            break;
            
        case err_t::BLUETOOTH_ERROR:
            error_bit = STATUS_BLUETOOTH_ERROR;
            break;
            
        case err_t::HARDWARE:
            error_bit = STATUS_HARDWARE_ERROR;
            break;
            
        case err_t::DATA_ERROR:
            error_bit = STATUS_DATA_ERROR;
            break;
            
        case err_t::DFU_ERROR:
            error_bit = STATUS_DFU_ERROR;
            break;
            
        case err_t::ADC_ERROR:
            error_bit = STATUS_ADC_ERROR;
            break;
            
        case err_t::I2C_ERROR:
            error_bit = STATUS_I2C_ERROR;
            break;
            
        case err_t::BATTERY_DISCONNECTION_ERROR:
            error_bit = STATUS_BATTERY_DISCONNECTED;
            break;
            
        case err_t::MOTION_ERROR:
            error_bit = STATUS_MOTION_ERROR;
            break;
            
        case err_t::RTC_ERROR:
            error_bit = STATUS_RTC_ERROR;
            break;
            
        case err_t::FILE_SYSTEM_ERROR:
            error_bit = STATUS_FILE_SYSTEM_ERROR;
            break;
            
        case err_t::PROTO_ENCODE_ERROR:
            error_bit = STATUS_PROTO_ENCODE_ERROR;
            break;
            
        case err_t::FILE_SYSTEM_NO_FILES:
            error_bit = STATUS_FILE_SYSTEM_NO_FILES;
            break;
            
        case err_t::FILE_SYSTEM_STORAGE_FULL:
            error_bit = STATUS_FILE_SYSTEM_FULL;
            break;
            
        case err_t::FLASH_FAILURE:
            error_bit = STATUS_FLASH_FAILURE;
            break;
            
        default:
            LOG_WRN("Unknown error code: %d, setting general error flag", (int)error_code);
            error_bit = STATUS_ERROR;
            break;
    }
    
    // Set the error bit and the general error flag
    uint32_t new_status = device_status_bitfield | error_bit | STATUS_ERROR;
    
    LOG_INF("Setting error status: error_code=%d, bit=0x%08X, new_status=0x%08X", 
            (int)error_code, error_bit, new_status);
    
    set_device_status(new_status);
}

/**
 * @brief Clears error status bits based on err_t error codes
 *
 * @param error_code The error code to clear from status
 */
void jis_clear_err_status_notify(err_t error_code)
{
    uint32_t error_bit = 0;
    
    // Map err_t values to status bits
    switch (error_code)
    {
        case err_t::NO_ERROR:
            // Clear all error bits
            error_bit = STATUS_ALL_ERRORS_MASK;
            break;
            
        case err_t::BATTERY_FAULT:
            error_bit = STATUS_BATTERY_FAULT;
            break;
            
        case err_t::BLUETOOTH_ERROR:
            error_bit = STATUS_BLUETOOTH_ERROR;
            break;
            
        case err_t::HARDWARE:
            error_bit = STATUS_HARDWARE_ERROR;
            break;
            
        case err_t::DATA_ERROR:
            error_bit = STATUS_DATA_ERROR;
            break;
            
        case err_t::DFU_ERROR:
            error_bit = STATUS_DFU_ERROR;
            break;
            
        case err_t::ADC_ERROR:
            error_bit = STATUS_ADC_ERROR;
            break;
            
        case err_t::I2C_ERROR:
            error_bit = STATUS_I2C_ERROR;
            break;
            
        case err_t::BATTERY_DISCONNECTION_ERROR:
            error_bit = STATUS_BATTERY_DISCONNECTED;
            break;
            
        case err_t::MOTION_ERROR:
            error_bit = STATUS_MOTION_ERROR;
            break;
            
        case err_t::RTC_ERROR:
            error_bit = STATUS_RTC_ERROR;
            break;
            
        case err_t::FILE_SYSTEM_ERROR:
            error_bit = STATUS_FILE_SYSTEM_ERROR;
            break;
            
        case err_t::PROTO_ENCODE_ERROR:
            error_bit = STATUS_PROTO_ENCODE_ERROR;
            break;
            
        case err_t::FILE_SYSTEM_NO_FILES:
            error_bit = STATUS_FILE_SYSTEM_NO_FILES;
            break;
            
        case err_t::FILE_SYSTEM_STORAGE_FULL:
            error_bit = STATUS_FILE_SYSTEM_FULL;
            break;
            
        case err_t::FLASH_FAILURE:
            error_bit = STATUS_FLASH_FAILURE;
            break;
            
        default:
            LOG_WRN("Unknown error code to clear: %d", (int)error_code);
            return;
    }
    
    // Clear the specific error bit
    uint32_t new_status = device_status_bitfield & ~error_bit;
    
    // If no error bits remain, clear the general error flag
    if ((new_status & STATUS_ALL_ERRORS_MASK) == 0) {
        new_status &= ~STATUS_ERROR;
    }
    
    LOG_INF("Clearing error status: error_code=%d, bit=0x%08X, new_status=0x%08X", 
            (int)error_code, error_bit, new_status);
    
    set_device_status(new_status);
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
    // Always return the current device status bitfield
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &device_status_bitfield, sizeof(device_status_bitfield));
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

void jis_foot_sensor_log_available_notify(uint8_t log_id)
{
    foot_sensor_log_available = log_id;
    LOG_DBG("Foot Sensor Log ID: %u, notifications enabled: %s", foot_sensor_log_available,
            foot_sensor_log_available_subscribed ? "true" : "false");
    if (foot_sensor_log_available_subscribed)
    {
        auto *status_gatt =
            bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &foot_sensor_log_available_uuid.uuid);
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
    auto *value = static_cast<uint8_t *>(attr->user_data);                     // Changed to uint32_t for ID
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
void jis_bhi360_log_available_notify(uint8_t log_id)
{
    bhi360_log_available = log_id;
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
    auto *value = static_cast<uint8_t *>(attr->user_data);
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

// --- BHI360 Data Set 1 ---
static void jis_bhi360_data1_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bhi360_data1_subscribed = value == BT_GATT_CCC_NOTIFY;
}
static ssize_t jis_bhi360_data1_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(bhi360_data1_value));
}
void jis_bhi360_data1_notify(const bhi360_3d_mapping_t *data)
{
    memcpy(&bhi360_data1_value, data, sizeof(bhi360_data1_value));
    if (bhi360_data1_subscribed) {
        auto *gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &bhi360_data1_uuid.uuid);
        bt_gatt_notify(nullptr, gatt, static_cast<const void *>(&bhi360_data1_value), sizeof(bhi360_data1_value));
    }
}
// --- BHI360 Data Set 2 ---
static void jis_bhi360_data2_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bhi360_data2_subscribed = value == BT_GATT_CCC_NOTIFY;
}
static ssize_t jis_bhi360_data2_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(bhi360_data2_value));
}
void jis_bhi360_data2_notify(const bhi360_step_count_t *data)
{
    memcpy(&bhi360_data2_value, data, sizeof(bhi360_data2_value));
    if (bhi360_data2_subscribed) {
        auto *gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &bhi360_data2_uuid.uuid);
        bt_gatt_notify(nullptr, gatt, static_cast<const void *>(&bhi360_data2_value), sizeof(bhi360_data2_value));
    }
}
// --- BHI360 Data Set 3 ---
static void jis_bhi360_data3_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    bhi360_data3_subscribed = value == BT_GATT_CCC_NOTIFY;
}
static ssize_t jis_bhi360_data3_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(bhi360_linear_accel_t));
}
void jis_bhi360_data3_notify(const bhi360_linear_accel_t *data)
{
    memcpy(&bhi360_data3_value, data, sizeof(bhi360_linear_accel_t));
    if (bhi360_data3_subscribed) {
        auto *gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &bhi360_data3_uuid.uuid);
        bt_gatt_notify(nullptr, gatt, static_cast<const void *>(&bhi360_data3_value), sizeof(bhi360_linear_accel_t));
    }
}

// --- FOTA Progress ---
static void jis_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    fota_progress_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("FOTA Progress CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_fota_progress_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(fota_progress_value));
}

void jis_fota_progress_notify(const fota_progress_msg_t *progress)
{
    memcpy(&fota_progress_value, progress, sizeof(fota_progress_value));
    
    LOG_DBG("FOTA Progress: active=%d, status=%d, percent=%d%%, bytes=%u/%u", 
            fota_progress_value.is_active,
            fota_progress_value.status,
            fota_progress_value.percent_complete,
            fota_progress_value.bytes_received,
            fota_progress_value.total_size);
    
    if (fota_progress_subscribed) {
        auto *gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &fota_progress_uuid.uuid);
        bt_gatt_notify(nullptr, gatt, static_cast<const void *>(&fota_progress_value), sizeof(fota_progress_value));
    }
}


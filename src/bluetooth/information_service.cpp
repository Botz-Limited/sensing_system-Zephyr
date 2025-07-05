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
#include <app_fixed_point.hpp>
#include <ble_services.hpp>
#include <util.hpp>
#include <status_codes.h>
#include "ccc_callback_fix.hpp"
#include "ble_seq_manager.hpp"
#include "ble_packed_structs.h"
#include "ble_d2d_tx.hpp"

LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

static uint32_t device_status_bitfield = 0;
static uint32_t previous_device_status_bitfield = 0; // Store the previous value
static bool init_status_bt_update = false;        // A flag to force the first update

// Packed device status for efficient BLE transmission
static device_status_packed_t device_status_packed = {0};
static bool device_status_packed_subscribed = false;

// Packed file notification for efficient BLE transmission
static file_notification_packed_t file_notification_packed = {0};
static bool file_notification_packed_subscribed = false;

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
// 1: 3D Mapping, 2: Step Count, 3: Linear Acceleration
// Using fixed-point versions for BLE transmission
static bhi360_3d_mapping_fixed_t bhi360_data1_value_fixed = {0, 0, 0, 0, 0, 0, 0, 0};
static bhi360_step_count_fixed_t bhi360_data2_value_fixed = {0, 0};
static bhi360_linear_accel_fixed_t bhi360_data3_value_fixed = {0, 0, 0};
static bool bhi360_data1_subscribed = false;
static bool bhi360_data2_subscribed = false;
static bool bhi360_data3_subscribed = false;

// FOTA progress tracking
static fota_progress_msg_t fota_progress_value = {false, 0, 0, 0, 0, 0};
static bool fota_progress_subscribed = false;

// Activity log tracking (primary device)
static uint8_t activity_log_available = 0;
static bool activity_log_available_subscribed = false;
static char activity_log_path[util::max_path_length] = {0};
static bool activity_log_path_subscribed = false;

// Secondary FOTA progress tracking (primary device only)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static fota_progress_msg_t secondary_fota_progress_value = {false, 0, 0, 0, 0, 0};
static bool secondary_fota_progress_subscribed = false;

// Secondary file management
static uint8_t secondary_foot_log_available = 0;
static bool secondary_foot_log_available_subscribed = false;
static char secondary_foot_log_path[util::max_path_length] = {0};
static bool secondary_foot_log_path_subscribed = false;

static uint8_t secondary_bhi360_log_available = 0;
static bool secondary_bhi360_log_available_subscribed = false;
static char secondary_bhi360_log_path[util::max_path_length] = {0};
static bool secondary_bhi360_log_path_subscribed = false;

static uint8_t secondary_activity_log_available = 0;
static bool secondary_activity_log_available_subscribed = false;
static char secondary_activity_log_path[util::max_path_length] = {0};
static bool secondary_activity_log_path_subscribed = false;
#endif

// Secondary device information storage (only used on primary)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static char secondary_manufacturer[32] = "Not Connected";
static char secondary_model[16] = "Not Connected";
static char secondary_serial[16] = "Not Connected";
static char secondary_hw_rev[16] = "Not Connected";
static char secondary_fw_rev[16] = "Not Connected";
#endif

// Step counts moved to Activity Metrics Service

// Weight measurement result
static uint16_t weight_kg_x10 = 0;  // Weight in kg * 10 (for 0.1kg precision)
static bool weight_subscribed = false;

// Weight calibration data
static float weight_calibration_offset = 0.0f;
static float weight_calibration_scale = 1.0f;
static bool weight_calibration_loaded = false;

// Weight request tracking
static bool weight_requested_by_phone = false;

// --- Global variables for Current Time Service Notifications ---
static struct bt_conn *current_time_conn_active = NULL; // Stores the connection object for notifications
static bool current_time_notifications_enabled = false; // Flag to track CCC state

// FWD Declarations
// clang-format off
// BHI360 Data Set Characteristics
static ssize_t jis_bhi360_data1_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_bhi360_data1_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_bhi360_data1_notify(const bhi360_3d_mapping_t* data);
extern "C" void jis_bhi360_data1_notify_ble(const bhi360_3d_mapping_ble_t* data);

static ssize_t jis_bhi360_data2_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_bhi360_data2_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_bhi360_data2_notify(const bhi360_step_count_t* data);

static ssize_t jis_bhi360_data3_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_bhi360_data3_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_bhi360_data3_notify_ble(const bhi360_linear_accel_ble_t* data);

// Step count characteristics moved to Activity Metrics Service

// Weight measurement characteristic
static ssize_t jis_weight_measurement_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_weight_measurement_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_weight_measurement_notify(float weight_kg);

// Weight calibration characteristics
static ssize_t jis_weight_calibration_write(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t jis_weight_request_write(struct bt_conn* conn, const struct bt_gatt_attr* attr, const void* buf, uint16_t len, uint16_t offset, uint8_t flags);

// Weight calibration helper functions
static void load_weight_calibration(void);
static float apply_weight_calibration(float raw_weight);
static void save_weight_calibration(void);

// Packed device status characteristic
static ssize_t jis_device_status_packed_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_device_status_packed_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_device_status_packed_notify(void);

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
extern "C" void jis_foot_sensor_notify_ble(const foot_samples_ble_t *data);

// FOTA Progress Characteristic
static ssize_t jis_fota_progress_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_fota_progress_notify(const fota_progress_msg_t* progress);

// Activity Log Characteristics
static ssize_t jis_activity_log_available_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_activity_log_available_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_activity_log_available_notify(uint8_t log_id);

static ssize_t jis_activity_log_path_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_activity_log_path_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_activity_log_path_notify(const char* path);

// Secondary Device Information Characteristics (Primary only)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static ssize_t jis_secondary_info_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);

// Secondary FOTA Progress Characteristic
static ssize_t jis_secondary_fota_progress_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_secondary_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_secondary_fota_progress_notify(const fota_progress_msg_t* progress);

// Secondary File Management Characteristics
static ssize_t jis_secondary_foot_log_available_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_secondary_foot_log_available_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_secondary_foot_log_available_notify(uint8_t log_id);

static ssize_t jis_secondary_foot_log_path_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_secondary_foot_log_path_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_secondary_foot_log_path_notify(const char* path);

static ssize_t jis_secondary_bhi360_log_available_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_secondary_bhi360_log_available_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_secondary_bhi360_log_available_notify(uint8_t log_id);

static ssize_t jis_secondary_bhi360_log_path_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_secondary_bhi360_log_path_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_secondary_bhi360_log_path_notify(const char* path);

static ssize_t jis_secondary_activity_log_available_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_secondary_activity_log_available_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_secondary_activity_log_available_notify(uint8_t log_id);

static ssize_t jis_secondary_activity_log_path_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_secondary_activity_log_path_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_secondary_activity_log_path_notify(const char* path);
#endif

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

// Activity Log UUIDs (primary device)
static struct bt_uuid_128 activity_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec2, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 activity_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec3, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// Step count UUIDs moved to Activity Metrics Service

// Weight Measurement UUID
static struct bt_uuid_128 weight_measurement_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec6, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// Weight Calibration UUIDs
static struct bt_uuid_128 weight_calibration_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec8, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 weight_request_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec9, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// Packed Device Status UUID (new)
static struct bt_uuid_128 device_status_packed_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec7, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));


// Secondary Device Information UUIDs
static struct bt_uuid_128 secondary_manufacturer_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb6, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 secondary_model_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb7, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 secondary_serial_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb8, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 secondary_hw_rev_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb9, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 secondary_fw_rev_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eba, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// Secondary FOTA Progress UUID
static struct bt_uuid_128 secondary_fota_progress_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ebb, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// Secondary File Management UUIDs
static struct bt_uuid_128 secondary_foot_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ebc, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 secondary_foot_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ebd, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 secondary_bhi360_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ebe, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 secondary_bhi360_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ebf, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 secondary_activity_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec0, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 secondary_activity_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec1, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

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
        static_cast<void *>(&bhi360_data1_value_fixed)),
    BT_GATT_CCC(jis_bhi360_data1_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Individual foot step count characteristic - DEPRECATED
    // Only aggregated step counts (total and activity) should be used by mobile apps
    BT_GATT_CHARACTERISTIC(&bhi360_data2_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_bhi360_data2_read, nullptr,
        static_cast<void *>(&bhi360_data2_value_fixed)),
    BT_GATT_CCC(jis_bhi360_data2_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    BT_GATT_CHARACTERISTIC(&bhi360_data3_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_bhi360_data3_read, nullptr,
        static_cast<void *>(&bhi360_data3_value_fixed)),
    BT_GATT_CCC(jis_bhi360_data3_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Step count characteristics moved to Activity Metrics Service

    // Weight Measurement Characteristic
    BT_GATT_CHARACTERISTIC(&weight_measurement_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_weight_measurement_read, nullptr,
        static_cast<void *>(&weight_kg_x10)),
    BT_GATT_CCC(jis_weight_measurement_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Weight Calibration Characteristic
    BT_GATT_CHARACTERISTIC(&weight_calibration_uuid.uuid,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE_ENCRYPT,
        nullptr, jis_weight_calibration_write, nullptr),

    // Weight Request Characteristic
    BT_GATT_CHARACTERISTIC(&weight_request_uuid.uuid,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE_ENCRYPT,
        nullptr, jis_weight_request_write, nullptr),

    // Packed Device Status Characteristic (new - combines multiple status fields)
    BT_GATT_CHARACTERISTIC(&device_status_packed_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_device_status_packed_read, nullptr,
        static_cast<void *>(&device_status_packed)),
    BT_GATT_CCC(jis_device_status_packed_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // FOTA Progress Characteristic
    BT_GATT_CHARACTERISTIC(&fota_progress_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_fota_progress_read, nullptr,
        static_cast<void *>(&fota_progress_value)),
    BT_GATT_CCC(jis_fota_progress_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Activity Log Available Characteristic
    BT_GATT_CHARACTERISTIC(&activity_log_available_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_activity_log_available_read, nullptr,
        static_cast<void *>(&activity_log_available)),
    BT_GATT_CCC(jis_activity_log_available_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Activity Log Path Characteristic
    BT_GATT_CHARACTERISTIC(&activity_log_path_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_activity_log_path_read, nullptr,
        activity_log_path),
    BT_GATT_CCC(jis_activity_log_path_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Secondary Device Information Characteristics
    BT_GATT_CHARACTERISTIC(&secondary_manufacturer_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_info_read, nullptr,
        secondary_manufacturer),

    BT_GATT_CHARACTERISTIC(&secondary_model_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_info_read, nullptr,
        secondary_model),

    BT_GATT_CHARACTERISTIC(&secondary_serial_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_info_read, nullptr,
        secondary_serial),

    BT_GATT_CHARACTERISTIC(&secondary_hw_rev_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_info_read, nullptr,
        secondary_hw_rev),

    BT_GATT_CHARACTERISTIC(&secondary_fw_rev_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_info_read, nullptr,
        secondary_fw_rev),

    // Secondary FOTA Progress Characteristic
    BT_GATT_CHARACTERISTIC(&secondary_fota_progress_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_fota_progress_read, nullptr,
        static_cast<void *>(&secondary_fota_progress_value)),
    BT_GATT_CCC(jis_secondary_fota_progress_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Secondary Foot Log Available
    BT_GATT_CHARACTERISTIC(&secondary_foot_log_available_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_foot_log_available_read, nullptr,
        static_cast<void *>(&secondary_foot_log_available)),
    BT_GATT_CCC(jis_secondary_foot_log_available_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Secondary Foot Log Path
    BT_GATT_CHARACTERISTIC(&secondary_foot_log_path_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_foot_log_path_read, nullptr,
        secondary_foot_log_path),
    BT_GATT_CCC(jis_secondary_foot_log_path_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Secondary BHI360 Log Available
    BT_GATT_CHARACTERISTIC(&secondary_bhi360_log_available_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_bhi360_log_available_read, nullptr,
        static_cast<void *>(&secondary_bhi360_log_available)),
    BT_GATT_CCC(jis_secondary_bhi360_log_available_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Secondary BHI360 Log Path
    BT_GATT_CHARACTERISTIC(&secondary_bhi360_log_path_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_bhi360_log_path_read, nullptr,
        secondary_bhi360_log_path),
    BT_GATT_CCC(jis_secondary_bhi360_log_path_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Secondary Activity Log Available
    BT_GATT_CHARACTERISTIC(&secondary_activity_log_available_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_activity_log_available_read, nullptr,
        static_cast<void *>(&secondary_activity_log_available)),
    BT_GATT_CCC(jis_secondary_activity_log_available_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),

    // Secondary Activity Log Path
    BT_GATT_CHARACTERISTIC(&secondary_activity_log_path_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ_ENCRYPT,
        jis_secondary_activity_log_path_read, nullptr,
        secondary_activity_log_path),
    BT_GATT_CCC(jis_secondary_activity_log_path_ccc_cfg_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT)
#endif
    );

// clang-format on

// Helper function to safely send notifications
static int safe_gatt_notify(const struct bt_uuid *uuid, const void *data, uint16_t len)
{
    auto *gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, uuid);
    if (!gatt) {
        LOG_WRN("GATT attribute not found for UUID, skipping notification");
        return -ENOENT;
    }
    return bt_gatt_notify(nullptr, gatt, data, len);
}

/**
 * @brief
 *
 * @param status
 *
 */
// Set and notify device status (bitfield)
extern "C" void set_device_status(uint32_t new_status)
{
    if (device_status_bitfield != new_status || !init_status_bt_update) {
        device_status_bitfield = new_status;
        if (status_subscribed) {
            auto *status_gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &STATUS_UUID.uuid);
            if (status_gatt) {
                bt_gatt_notify(nullptr, status_gatt, &device_status_bitfield, sizeof(device_status_bitfield));
                LOG_DBG("Device status updated and notified: 0x%08X", device_status_bitfield);
            } else {
                LOG_WRN("Status GATT attribute not found, skipping notification");
            }
        }
        
        // Also update packed device status
        jis_device_status_packed_notify();
        
        previous_device_status_bitfield = device_status_bitfield;
        init_status_bt_update = true;
    }
}

// Step count handlers moved to Activity Metrics Service

/**
 * @brief Sets error status bits based on err_t error codes
 *
 * @param error_code The error code to convert to status bit
 */
extern "C" void jis_set_err_status_notify(err_t error_code)
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
extern "C" void jis_clear_err_status_notify(err_t error_code)
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
        // Convert to BLE format with sequence number
        foot_samples_ble_t ble_data;
        BleSequenceManager::getInstance().addFootSample(samples_data, &ble_data);
        
        auto *status_gatt =
            bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &foot_sensor_samples.uuid);
        if (status_gatt) {
            // Send BLE format with sequence number
            bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&ble_data),
                           sizeof(ble_data));
        }
    }
}

// Helper function to send BLE format data directly (used by recovery)
extern "C" void jis_foot_sensor_notify_ble(const foot_samples_ble_t *data)
{
    if (status_subscribed && data) {
        auto *status_gatt =
            bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &foot_sensor_samples.uuid);
        if (status_gatt) {
            bt_gatt_notify(nullptr, status_gatt, static_cast<const void *>(data),
                           sizeof(*data));
        }
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
    if (!attr) {
        LOG_ERR("jis_status_ccc_cfg_changed: attr is NULL");
        return;
    }
    status_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Status CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static void jis_foot_sensor_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_foot_sensor_ccc_cfg_changed: attr is NULL");
        return;
    }
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
    if (!attr) {
        LOG_ERR("jis_current_time_ccc_cfg_changed: attr is NULL");
        return;
    }
    current_time_notifications_enabled = value == BT_GATT_CCC_NOTIFY;
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
    if (info_service.attrs && info_service.attr_count > 1) {
        bt_gatt_notify(NULL, &info_service.attrs[1], &ct, sizeof(ct));
    } else {
        LOG_WRN("Information service not ready for CTS notification");
    }
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

    if (info_service.attrs && info_service.attr_count > 0) {
        bt_gatt_notify_uuid(nullptr, &CHARGE_STATUS_UUID.uuid, info_service.attrs, static_cast<void *>(&charge_status),
                            sizeof(charge_status));
    } else {
        LOG_WRN("Information service not ready for charge status notification");
    }
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
    if (!attr) {
        LOG_ERR("jis_charge_status_ccc_cfg_changed: attr is NULL");
        return;
    }
    // Note: charge status doesn't use notifications currently
    LOG_DBG("Charge Status CCC changed to: %d", value);
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
        if (status_gatt) {
            bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&foot_sensor_log_available),
                           sizeof(foot_sensor_log_available));
        }
    }
    
    // Also send packed notification (will be combined with path notification)
    // Note: Path will be sent separately via jis_foot_sensor_req_id_path_notify
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
    if (!attr) {
        LOG_ERR("jis_foot_sensor_log_available_ccc_cfg_changed: attr is NULL");
        return;
    }
    foot_sensor_log_available_subscribed = value == BT_GATT_CCC_NOTIFY; // Renamed
    LOG_DBG("Foot Sensor Log Available CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
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
        safe_gatt_notify(&foot_sensor_req_id_path_uuid.uuid, 
                        static_cast<void *>(&foot_sensor_req_id_path),
                        sizeof(foot_sensor_req_id_path));
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
    if (!attr) {
        LOG_ERR("jis_foot_sensor_req_id_ccc_cfg_changed: attr is NULL");
        return;
    }
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
        safe_gatt_notify(&bhi360_log_available_uuid.uuid,
                        static_cast<void *>(&bhi360_log_available), 
                        sizeof(bhi360_log_available));
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
    if (!attr) {
        LOG_ERR("jis_bhi360_log_available_ccc_cfg_changed: attr is NULL");
        return;
    }
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
        safe_gatt_notify(&bhi360_req_id_path_uuid.uuid,
                        static_cast<void *>(&bhi360_req_id_path), 
                        sizeof(bhi360_req_id_path));
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
    if (!attr) {
        LOG_ERR("jis_bhi360_req_id_ccc_cfg_changed: attr is NULL");
        return;
    }
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
    if (!attr) {
        LOG_ERR("jis_bhi360_data1_ccc_cfg_changed: attr is NULL");
        return;
    }
    bhi360_data1_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("BHI360 Data1 CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}
static ssize_t jis_bhi360_data1_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(bhi360_3d_mapping_fixed_t));
}
extern "C" void jis_bhi360_data1_notify(const bhi360_3d_mapping_t *data)
{
    // Convert float data to fixed-point for BLE transmission
    convert_3d_mapping_to_fixed(*data, bhi360_data1_value_fixed);
    
    if (bhi360_data1_subscribed) {
        // Convert to BLE format with sequence number
        bhi360_3d_mapping_ble_t ble_data;
        BleSequenceManager::getInstance().addBhi3603D(data, &ble_data);
        
        safe_gatt_notify(&bhi360_data1_uuid.uuid,
                        static_cast<const void *>(&ble_data), 
                        sizeof(ble_data));
    }
}

// Helper function to send BLE format data directly (used by recovery)
extern "C" void jis_bhi360_data1_notify_ble(const bhi360_3d_mapping_ble_t *data)
{
    if (bhi360_data1_subscribed && data) {
        safe_gatt_notify(&bhi360_data1_uuid.uuid,
                        static_cast<const void *>(data), 
                        sizeof(*data));
    }
}
// --- BHI360 Data Set 2 ---
static void jis_bhi360_data2_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_bhi360_data2_ccc_cfg_changed: attr is NULL");
        return;
    }
    bhi360_data2_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("BHI360 Data2 CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}
static ssize_t jis_bhi360_data2_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(bhi360_step_count_fixed_t));
}
extern "C" void jis_bhi360_data2_notify(const bhi360_step_count_t *data)
{
    // Step count data is already integers, just copy
    bhi360_data2_value_fixed.step_count = data->step_count;
    bhi360_data2_value_fixed.activity_duration_s = 0;  // Deprecated - always 0
    
    if (bhi360_data2_subscribed) {
        safe_gatt_notify(&bhi360_data2_uuid.uuid,
                        static_cast<const void *>(&bhi360_data2_value_fixed), 
                        sizeof(bhi360_data2_value_fixed));
    }
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Also update secondary step count for aggregation
    // This function is called from d2d_data_handler when secondary data arrives
    // We need to tell the bluetooth module about the secondary update
    static uint32_t last_secondary_steps = 0;
    static uint32_t last_secondary_duration = 0;
    
    // Check if this is likely secondary data (different from last known secondary)
    // This is a bit of a hack - ideally we'd have a separate function for secondary
    if (data->step_count != last_secondary_steps || data->activity_duration_s != last_secondary_duration) {
        // Send a message to bluetooth module with secondary step count
        generic_message_t msg = {};
        msg.sender = SENDER_D2D_SECONDARY;  // Indicate it's from D2D
        msg.type = MSG_TYPE_BHI360_STEP_COUNT;
        msg.data.bhi360_step_count = *data;
        
        if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) == 0) {
            last_secondary_steps = data->step_count;
            last_secondary_duration = data->activity_duration_s;
            LOG_DBG("Sent secondary step count to bluetooth module for aggregation");
        }
    }
#endif
}
// --- BHI360 Data Set 3 ---
static void jis_bhi360_data3_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_bhi360_data3_ccc_cfg_changed: attr is NULL");
        return;
    }
    bhi360_data3_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("BHI360 Data3 CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}
static ssize_t jis_bhi360_data3_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(bhi360_linear_accel_fixed_t));
}
void jis_bhi360_data3_notify(const bhi360_linear_accel_t *data)
{
    // Convert float data to fixed-point for BLE transmission
    convert_linear_accel_to_fixed(*data, bhi360_data3_value_fixed);
    
    if (bhi360_data3_subscribed) {
        // Convert to BLE format with sequence number
        bhi360_linear_accel_ble_t ble_data;
        BleSequenceManager::getInstance().addBhi360Accel(data, &ble_data);
        
        safe_gatt_notify(&bhi360_data3_uuid.uuid,
                        static_cast<const void *>(&ble_data), 
                        sizeof(ble_data));
    }
}

// Helper function to send BLE format data directly (used by recovery)
extern "C" void jis_bhi360_data3_notify_ble(const bhi360_linear_accel_ble_t *data)
{
    if (bhi360_data3_subscribed && data) {
        safe_gatt_notify(&bhi360_data3_uuid.uuid,
                        static_cast<const void *>(data), 
                        sizeof(*data));
    }
}

// --- FOTA Progress ---
static void jis_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_fota_progress_ccc_cfg_changed: attr is NULL");
        return;
    }
    fota_progress_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("FOTA Progress CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_fota_progress_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(fota_progress_value));
}

void jis_fota_progress_notify(const fota_progress_msg_t *progress)
{
    // If FOTA is starting (transitioning to active with status 1), ensure clean values
    if (progress->is_active && progress->status == 1 && 
        progress->bytes_received == 0 && progress->percent_complete == 0) {
        // This is a fresh start, clear any stale data
        memset(&fota_progress_value, 0, sizeof(fota_progress_value));
    }
    
    memcpy(&fota_progress_value, progress, sizeof(fota_progress_value));
    
    LOG_DBG("FOTA Progress: active=%d, status=%d, percent=%d%%, bytes=%u/%u", 
            fota_progress_value.is_active,
            fota_progress_value.status,
            fota_progress_value.percent_complete,
            fota_progress_value.bytes_received,
            fota_progress_value.total_size);
    
    if (fota_progress_subscribed) {
        safe_gatt_notify(&fota_progress_uuid.uuid,
                        static_cast<const void *>(&fota_progress_value), 
                        sizeof(fota_progress_value));
    }
}

// --- Activity Log Handlers ---
static void jis_activity_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_activity_log_available_ccc_cfg_changed: attr is NULL");
        return;
    }
    activity_log_available_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Activity Log Available CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_activity_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(activity_log_available));
}

extern "C" void jis_activity_log_available_notify(uint8_t log_id)
{
    activity_log_available = log_id;
    LOG_INF("Activity Log Available: ID=%u", log_id);
    
    if (activity_log_available_subscribed) {
        safe_gatt_notify(&activity_log_available_uuid.uuid,
                        static_cast<const void *>(&activity_log_available), 
                        sizeof(activity_log_available));
    }
}

static void jis_activity_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_activity_log_path_ccc_cfg_changed: attr is NULL");
        return;
    }
    activity_log_path_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Activity Log Path CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_activity_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, strlen(activity_log_path));
}

extern "C" void jis_activity_log_path_notify(const char *path)
{
    memset(activity_log_path, 0, sizeof(activity_log_path));
    strncpy(activity_log_path, path, sizeof(activity_log_path) - 1);
    activity_log_path[sizeof(activity_log_path) - 1] = '\0';
    
    LOG_INF("Activity Log Path: %s", activity_log_path);
    
    if (activity_log_path_subscribed) {
        safe_gatt_notify(&activity_log_path_uuid.uuid,
                        static_cast<const void *>(&activity_log_path), 
                        strlen(activity_log_path) + 1);
    }
    
}

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
// Secondary device info read handler
static ssize_t jis_secondary_info_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    const char *value = static_cast<const char *>(attr->user_data);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

// Function to update secondary device information
void jis_update_secondary_device_info(const char *manufacturer, const char *model, 
                                     const char *serial, const char *hw_rev, const char *fw_rev)
{
    if (manufacturer) {
        strncpy(secondary_manufacturer, manufacturer, sizeof(secondary_manufacturer) - 1);
        secondary_manufacturer[sizeof(secondary_manufacturer) - 1] = '\0';
    }
    if (model) {
        strncpy(secondary_model, model, sizeof(secondary_model) - 1);
        secondary_model[sizeof(secondary_model) - 1] = '\0';
    }
    if (serial) {
        strncpy(secondary_serial, serial, sizeof(secondary_serial) - 1);
        secondary_serial[sizeof(secondary_serial) - 1] = '\0';
    }
    if (hw_rev) {
        strncpy(secondary_hw_rev, hw_rev, sizeof(secondary_hw_rev) - 1);
        secondary_hw_rev[sizeof(secondary_hw_rev) - 1] = '\0';
    }
    if (fw_rev) {
        strncpy(secondary_fw_rev, fw_rev, sizeof(secondary_fw_rev) - 1);
        secondary_fw_rev[sizeof(secondary_fw_rev) - 1] = '\0';
    }
    
    LOG_INF("Updated secondary device info: Mfg=%s, Model=%s, Serial=%s, HW=%s, FW=%s",
            secondary_manufacturer, secondary_model, secondary_serial, 
            secondary_hw_rev, secondary_fw_rev);
}

// Function to clear secondary device info when disconnected
void jis_clear_secondary_device_info(void)
{
    strcpy(secondary_manufacturer, "Not Connected");
    strcpy(secondary_model, "Not Connected");
    strcpy(secondary_serial, "Not Connected");
    strcpy(secondary_hw_rev, "Not Connected");
    strcpy(secondary_fw_rev, "Not Connected");
    
    LOG_INF("Cleared secondary device info");
}

// Secondary FOTA Progress handlers
static void jis_secondary_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_secondary_fota_progress_ccc_cfg_changed: attr is NULL");
        return;
    }
    secondary_fota_progress_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Secondary FOTA Progress CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_secondary_fota_progress_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(secondary_fota_progress_value));
}

void jis_secondary_fota_progress_notify(const fota_progress_msg_t *progress)
{
    memcpy(&secondary_fota_progress_value, progress, sizeof(secondary_fota_progress_value));
    
    LOG_INF("Secondary FOTA Progress: active=%d, status=%d, percent=%d%%, bytes=%u/%u", 
            secondary_fota_progress_value.is_active,
            secondary_fota_progress_value.status,
            secondary_fota_progress_value.percent_complete,
            secondary_fota_progress_value.bytes_received,
            secondary_fota_progress_value.total_size);
    
    if (secondary_fota_progress_subscribed) {
        safe_gatt_notify(&secondary_fota_progress_uuid.uuid,
                        static_cast<const void *>(&secondary_fota_progress_value), 
                        sizeof(secondary_fota_progress_value));
    }
}

// Secondary Foot Log Available handlers
static void jis_secondary_foot_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_secondary_foot_log_available_ccc_cfg_changed: attr is NULL");
        return;
    }
    secondary_foot_log_available_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Secondary Foot Log Available CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_secondary_foot_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(secondary_foot_log_available));
}

void jis_secondary_foot_log_available_notify(uint8_t log_id)
{
    secondary_foot_log_available = log_id;
    LOG_INF("Secondary Foot Log Available: ID=%u", log_id);
    
    if (secondary_foot_log_available_subscribed) {
        safe_gatt_notify(&secondary_foot_log_available_uuid.uuid,
                        static_cast<const void *>(&secondary_foot_log_available), 
                        sizeof(secondary_foot_log_available));
    }
}

// Secondary Foot Log Path handlers
static void jis_secondary_foot_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_secondary_foot_log_path_ccc_cfg_changed: attr is NULL");
        return;
    }
    secondary_foot_log_path_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Secondary Foot Log Path CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_secondary_foot_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, strlen(secondary_foot_log_path));
}

void jis_secondary_foot_log_path_notify(const char *path)
{
    memset(secondary_foot_log_path, 0, sizeof(secondary_foot_log_path));
    strncpy(secondary_foot_log_path, path, sizeof(secondary_foot_log_path) - 1);
    secondary_foot_log_path[sizeof(secondary_foot_log_path) - 1] = '\0';
    
    LOG_INF("Secondary Foot Log Path: %s", secondary_foot_log_path);
    
    if (secondary_foot_log_path_subscribed) {
        safe_gatt_notify(&secondary_foot_log_path_uuid.uuid,
                        static_cast<const void *>(&secondary_foot_log_path), 
                        strlen(secondary_foot_log_path) + 1);
    }
    
}

// Secondary BHI360 Log Available handlers
static void jis_secondary_bhi360_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_secondary_bhi360_log_available_ccc_cfg_changed: attr is NULL");
        return;
    }
    secondary_bhi360_log_available_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Secondary BHI360 Log Available CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_secondary_bhi360_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(secondary_bhi360_log_available));
}

void jis_secondary_bhi360_log_available_notify(uint8_t log_id)
{
    secondary_bhi360_log_available = log_id;
    LOG_INF("Secondary BHI360 Log Available: ID=%u", log_id);
    
    if (secondary_bhi360_log_available_subscribed) {
        safe_gatt_notify(&secondary_bhi360_log_available_uuid.uuid,
                        static_cast<const void *>(&secondary_bhi360_log_available), 
                        sizeof(secondary_bhi360_log_available));
    }
}

// Secondary BHI360 Log Path handlers
static void jis_secondary_bhi360_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_secondary_bhi360_log_path_ccc_cfg_changed: attr is NULL");
        return;
    }
    secondary_bhi360_log_path_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Secondary BHI360 Log Path CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_secondary_bhi360_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, strlen(secondary_bhi360_log_path));
}

void jis_secondary_bhi360_log_path_notify(const char *path)
{
    memset(secondary_bhi360_log_path, 0, sizeof(secondary_bhi360_log_path));
    strncpy(secondary_bhi360_log_path, path, sizeof(secondary_bhi360_log_path) - 1);
    secondary_bhi360_log_path[sizeof(secondary_bhi360_log_path) - 1] = '\0';
    
    LOG_INF("Secondary BHI360 Log Path: %s", secondary_bhi360_log_path);
    
    if (secondary_bhi360_log_path_subscribed) {
        safe_gatt_notify(&secondary_bhi360_log_path_uuid.uuid,
                        static_cast<const void *>(&secondary_bhi360_log_path), 
                        strlen(secondary_bhi360_log_path) + 1);
    }
    
}

// Secondary Activity Log Available handlers
static void jis_secondary_activity_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_secondary_activity_log_available_ccc_cfg_changed: attr is NULL");
        return;
    }
    secondary_activity_log_available_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Secondary Activity Log Available CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_secondary_activity_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(secondary_activity_log_available));
}

void jis_secondary_activity_log_available_notify(uint8_t log_id)
{
    secondary_activity_log_available = log_id;
    LOG_INF("Secondary Activity Log Available: ID=%u", log_id);
    
    if (secondary_activity_log_available_subscribed) {
        safe_gatt_notify(&secondary_activity_log_available_uuid.uuid,
                        static_cast<const void *>(&secondary_activity_log_available), 
                        sizeof(secondary_activity_log_available));
    }
}

// Secondary Activity Log Path handlers
static void jis_secondary_activity_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_secondary_activity_log_path_ccc_cfg_changed: attr is NULL");
        return;
    }
    secondary_activity_log_path_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Secondary Activity Log Path CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_secondary_activity_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, strlen(secondary_activity_log_path));
}

void jis_secondary_activity_log_path_notify(const char *path)
{
    memset(secondary_activity_log_path, 0, sizeof(secondary_activity_log_path));
    strncpy(secondary_activity_log_path, path, sizeof(secondary_activity_log_path) - 1);
    secondary_activity_log_path[sizeof(secondary_activity_log_path) - 1] = '\0';
    
    LOG_INF("Secondary Activity Log Path: %s", secondary_activity_log_path);
    
    if (secondary_activity_log_path_subscribed) {
        safe_gatt_notify(&secondary_activity_log_path_uuid.uuid,
                        static_cast<const void *>(&secondary_activity_log_path), 
                        strlen(secondary_activity_log_path) + 1);
    }
    
}
#endif

// Total step count handlers moved to Activity Metrics Service

// --- Weight Measurement Handlers ---
static void jis_weight_measurement_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_weight_measurement_ccc_cfg_changed: attr is NULL");
        return;
    }
    weight_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Weight Measurement CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_weight_measurement_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(weight_kg_x10));
}

extern "C" void jis_weight_measurement_notify(float weight_kg)
{
    // Ensure calibration is loaded
    load_weight_calibration();
    
    // Apply calibration
    float calibrated_weight = apply_weight_calibration(weight_kg);
    
    // Convert to uint16_t with 0.1kg precision
    weight_kg_x10 = (uint16_t)(calibrated_weight * 10);
    
    LOG_INF("Weight Measurement: raw=%.1f kg, calibrated=%.1f kg (raw=%u)", 
           weight_kg, calibrated_weight, weight_kg_x10);
    
    if (weight_subscribed) {
        safe_gatt_notify(&weight_measurement_uuid.uuid,
                        static_cast<const void *>(&weight_kg_x10), 
                        sizeof(weight_kg_x10));
    }
}

// Weight calibration structure for data persistence
struct weight_calibration_data {
    float offset;
    float scale;
    uint32_t checksum;
};

// Load weight calibration from data module
static void load_weight_calibration(void)
{
    if (weight_calibration_loaded) {
        return;
    }

    // Create message to request calibration data from data module
    generic_message_t msg = {};
    msg.sender = SENDER_BTH;
    msg.type = MSG_TYPE_REQUEST_WEIGHT_CALIBRATION;
    
    // Send message to data module
    if (k_msgq_put(&data_msgq, &msg, K_MSEC(100)) == 0) {
        LOG_DBG("Weight calibration request sent to data module");
    } else {
        LOG_ERR("Failed to send weight calibration request");
        // Use defaults
        weight_calibration_offset = 0.0f;
        weight_calibration_scale = 1.0f;
    }
    
    weight_calibration_loaded = true;
}

// Save weight calibration to data module
static void save_weight_calibration(void)
{
    // Create message to save calibration data to data module
    generic_message_t msg = {};
    msg.sender = SENDER_BTH;
    msg.type = MSG_TYPE_SAVE_WEIGHT_CALIBRATION;
    msg.data.weight_calibration.scale_factor = weight_calibration_scale;
    msg.data.weight_calibration.nonlinear_a = weight_calibration_offset;
    msg.data.weight_calibration.is_calibrated = true;
    msg.data.weight_calibration.timestamp = k_uptime_get_32();
    
    // Send message to data module
    if (k_msgq_put(&data_msgq, &msg, K_NO_WAIT) == 0) {
        LOG_INF("Weight calibration save request sent to data module");
    } else {
        LOG_ERR("Failed to send weight calibration save request");
    }
}

// Apply calibration to raw weight value
static float apply_weight_calibration(float raw_weight)
{
    return (raw_weight + weight_calibration_offset) * weight_calibration_scale;
}

// Weight calibration write handler
static ssize_t jis_weight_calibration_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, 
                                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    
    if (len != sizeof(weight_calibration_data)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    const weight_calibration_data* cal_data = static_cast<const weight_calibration_data*>(buf);
    
    // Verify checksum
    // Simple checksum calculation (sum of bytes)
    uint32_t calculated_checksum = 0;
    const uint8_t* data_ptr = reinterpret_cast<const uint8_t*>(&cal_data->offset);
    for (size_t i = 0; i < sizeof(float) * 2; i++) {
        calculated_checksum += data_ptr[i];
    }
    if (cal_data->checksum != calculated_checksum) {
        LOG_ERR("Weight calibration checksum verification failed");
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    // Validate calibration values
    if (cal_data->scale <= 0.0f || cal_data->scale > 10.0f) {
        LOG_ERR("Invalid weight calibration scale: %.3f", cal_data->scale);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    if (cal_data->offset < -100.0f || cal_data->offset > 100.0f) {
        LOG_ERR("Invalid weight calibration offset: %.3f", cal_data->offset);
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    // Update calibration values
    weight_calibration_offset = cal_data->offset;
    weight_calibration_scale = cal_data->scale;
    
    LOG_INF("Weight calibration updated: offset=%.3f, scale=%.3f", 
           weight_calibration_offset, weight_calibration_scale);
    
    // Save to data module
    save_weight_calibration();
    
    // Cascade calibration to secondary device if connected
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Send calibration update message to secondary device handler
    generic_message_t sec_msg = {};
    sec_msg.sender = SENDER_BTH;
    sec_msg.type = MSG_TYPE_SAVE_WEIGHT_CALIBRATION;
    sec_msg.data.weight_calibration.scale_factor = weight_calibration_scale;
    sec_msg.data.weight_calibration.nonlinear_a = weight_calibration_offset;
    sec_msg.data.weight_calibration.is_calibrated = true;
    sec_msg.data.weight_calibration.timestamp = k_uptime_get_32();
    
    // Send calibration to secondary device via D2D communication
    if (ble_d2d_tx_send_weight_calibration_command(&sec_msg.data.weight_calibration) == 0) {
        LOG_INF("Weight calibration cascaded to secondary device");
    } else {
        LOG_ERR("Failed to cascade weight calibration to secondary device");
    }
#endif
    
    return len;
}

// Weight request write handler
static ssize_t jis_weight_request_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint8_t request_value = *static_cast<const uint8_t*>(buf);
    weight_requested_by_phone = (request_value != 0);
    
    LOG_INF("Weight measurement requested by phone: %d", weight_requested_by_phone);
    
    if (weight_requested_by_phone) {
        // Ensure calibration is loaded
        load_weight_calibration();
        
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // Request weight measurement from secondary device
        // Send weight measurement request to secondary device via D2D communication
        if (ble_d2d_tx_send_measure_weight_command(1) == 0) {
            LOG_INF("Weight measurement request sent to secondary device");
        } else {
            LOG_ERR("Failed to send weight measurement request to secondary device");
        }
#endif
        
        // Also trigger local weight measurement
        generic_message_t local_msg = {};
        local_msg.sender = SENDER_BTH;
        local_msg.type = MSG_TYPE_WEIGHT_MEASUREMENT;
        
        if (k_msgq_put(&activity_metrics_msgq, &local_msg, K_NO_WAIT) == 0) {
            LOG_INF("Weight measurement request sent to activity metrics");
        } else {
            LOG_ERR("Failed to send weight measurement request to activity metrics");
        }
    }
    
    return len;
}

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
// Function to handle weight measurement from secondary device
extern "C" void jis_secondary_weight_measurement_notify(float weight_kg)
{
    LOG_INF("Received weight from secondary device: %.1f kg", weight_kg);
    
    // Only process if weight was requested by phone
    if (weight_requested_by_phone) {
        // Apply primary device calibration and notify
        jis_weight_measurement_notify(weight_kg);
        
        // Reset request flag
        weight_requested_by_phone = false;
    } else {
        LOG_DBG("Weight received but not requested by phone, ignoring");
    }
}
#endif

// Function to handle weight calibration data from data module
extern "C" void jis_handle_weight_calibration_data(const weight_calibration_data_t* cal_data)
{
    if (cal_data && cal_data->is_calibrated) {
        weight_calibration_scale = cal_data->scale_factor;
        weight_calibration_offset = cal_data->nonlinear_a;  // Using nonlinear_a as offset
        
        LOG_INF("Weight calibration loaded: offset=%.3f, scale=%.3f", 
               weight_calibration_offset, weight_calibration_scale);
    } else {
        LOG_INF("No weight calibration found, using defaults");
        weight_calibration_offset = 0.0f;
        weight_calibration_scale = 1.0f;
    }
    
    weight_calibration_loaded = true;
}

// Initialize weight measurement system
extern "C" void jis_weight_measurement_init(void)
{
    LOG_INF("Initializing weight measurement system");
    
    // Load calibration data
    load_weight_calibration();
    
    LOG_INF("Weight measurement system initialized");
}

// --- Packed Device Status Handlers ---
static void jis_device_status_packed_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr) {
        LOG_ERR("jis_device_status_packed_ccc_cfg_changed: attr is NULL");
        return;
    }
    device_status_packed_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Packed Device Status CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_device_status_packed_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    // Update packed structure before reading
    jis_device_status_packed_notify();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(device_status_packed_t));
}

extern "C" void jis_device_status_packed_notify(void)
{
    // Update the packed structure with current values
    device_status_packed.status_bitfield = device_status_bitfield;
    device_status_packed.battery_percent = 0; // TODO: Get from battery module
    device_status_packed.charge_status = charge_status;
    device_status_packed.temperature_c = 25; // TODO: Get from temperature sensor
    device_status_packed.activity_state = (device_status_bitfield & STATUS_LOGGING_ACTIVE) ? 1 : 0;
    device_status_packed.uptime_seconds = k_uptime_get_32() / 1000;
    device_status_packed.free_storage_mb = 0; // TODO: Get from filesystem
    device_status_packed.connected_devices = 0; // TODO: Count connected devices
    
    if (device_status_packed_subscribed) {
        safe_gatt_notify(&device_status_packed_uuid.uuid,
                        static_cast<const void *>(&device_status_packed), 
                        sizeof(device_status_packed));
    }
}


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

#include "ble_d2d_tx.hpp"
#include "ble_packed_structs.h"
#include "ble_seq_manager.hpp"
#include "ccc_callback_fix.hpp"
#include <app.hpp>
#include <app_fixed_point.hpp>
#include <ble_connection_state.h>
#include <ble_services.hpp>
#include <status_codes.h>
#include <util.hpp>
#include <zephyr/random/random.h>

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
#include "d2d_data_handler.hpp" // For accessing secondary data globals
#endif

LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Buffers for header, metrics, and footer
static ActivityFileHeaderV3 activity_header = {0};
static activity_metrics_binary_t activity_metrics_binary = {0};
static ActivityFileFooterV3 activity_footer = {0};


static bool activity_header_subscribed = false;
static bool activity_metrics_subscribed = false;
static bool activity_footer_subscribed = false;

uint32_t device_status_bitfield = 0;
static uint32_t previous_device_status_bitfield = 0; // Store the previous value
static bool init_status_bt_update = false;           // A flag to force the first update

static activity_state_t current_activity_state = ACTIVITY_STATE_IDLE;
static activity_state_t pre_pause_activity_state = ACTIVITY_STATE_IDLE; // State before pause

// Packed device status for efficient BLE transmission
static device_status_packed_t device_status_packed = {0};
static bool device_status_packed_subscribed = false;

// Packed file notification for efficient BLE transmission
static file_notification_packed_t file_notification_packed = {0};
static bool file_notification_packed_subscribed = false;

static uint8_t charge_status = 0;
static uint8_t battery_levels[2] = {80, 79}; // [0] = primary, [1] = secondary (0 = unknown)
static foot_samples_t foot_sensor_char_value = {0};

// Aggregated data storage for read callbacks
static foot_samples_aggregated_t foot_sensor_aggregated_value = {0};
static quaternion_aggregated_t quaternion_aggregated_value = {0};

static bool status_subscribed = false;
static bool foot_sensor_subscribed = false; // Separate flag for foot sensor
static bool bhi360_data1_subscribed = false;

static uint8_t ct[10];
static uint8_t ct_update = 0;

// --- BHI360 Data Set Characteristics ---
// 1: 3D Mapping, 2: Step Count, 3: Linear Acceleration
// Using fixed-point versions for BLE transmission
static bhi360_3d_mapping_fixed_t bhi360_data1_value_fixed = {0, 0, 0, 0, 0, 0, 0, 0};
// Commented out - only quaternion data sent to BLE at 5Hz
// static bhi360_step_count_fixed_t bhi360_data2_value_fixed = {0, 0};
// static bhi360_linear_accel_fixed_t bhi360_data3_value_fixed = {0, 0, 0};
// static bool bhi360_data2_subscribed = false;
// static bool bhi360_data3_subscribed = false;

// FOTA progress tracking
static fota_progress_msg_t fota_progress_value = {false, 0, 0, 0, 0, 0};
static bool fota_progress_subscribed = false;

// Activity log tracking (primary device)
static uint8_t activity_log_available_primary = 0;
static bool activity_log_available_primary_subscribed = false;
static char activity_log_path_primary[util::max_path_length] = {0};
static bool activity_log_path_primary_subscribed = false;

// Secondary FOTA progress tracking (primary device only)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static fota_progress_msg_t secondary_fota_progress_value = {false, 0, 0, 0, 0, 0};
static bool secondary_fota_progress_subscribed = false;

// Secondary file management

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
static uint16_t weight_kg_x10 = 0; // Weight in kg * 10 (for 0.1kg precision)
static bool weight_subscribed = false;

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

// Commented out - only quaternion data sent to BLE at 5Hz
// static ssize_t jis_bhi360_data2_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
// static void jis_bhi360_data2_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
// extern "C" void jis_bhi360_data2_notify(const bhi360_step_count_t* data);

// static ssize_t jis_bhi360_data3_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
// static void jis_bhi360_data3_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
// extern "C" void jis_bhi360_data3_notify_ble(const bhi360_linear_accel_ble_t* data);

// Step count characteristics moved to Activity Metrics Service

// Weight measurement characteristic
static ssize_t jis_weight_measurement_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_weight_measurement_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_weight_measurement_notify(float weight_kg);


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

static ssize_t jis_charge_status_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_charge_status_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);


// Foot Sensor Characteristic
static ssize_t jis_foot_sensor_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_foot_sensor_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_foot_sensor_notify_ble(const foot_samples_ble_t *data);

// FOTA Progress Characteristic
static ssize_t jis_fota_progress_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_fota_progress_notify(const fota_progress_msg_t* progress);

// Activity Log Characteristics
static ssize_t jis_activity_log_available_primary_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_activity_log_available_primary_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_activity_log_available_primary_notify(uint8_t log_id);

static ssize_t jis_activity_log_path_primary_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_activity_log_path_primary_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void jis_activity_log_path_primary_notify(const char* path);

static ssize_t jis_activity_metrics_header_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                                uint16_t len, uint16_t offset);

static ssize_t jis_activity_metrics_data_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                              uint16_t len, uint16_t offset);       
                                              
static ssize_t jis_activity_metrics_footer_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                                uint16_t len, uint16_t offset);      
                                                
static void jis_activity_metrics_header_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value); 
static void jis_activity_metrics_data_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);     
static void jis_activity_metrics_footer_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);                                          


// Secondary Device Information Characteristics (Primary only)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static ssize_t jis_secondary_info_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);

// Secondary FOTA Progress Characteristic
static ssize_t jis_secondary_fota_progress_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, void* buf, uint16_t len, uint16_t offset);
static void jis_secondary_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
void jis_secondary_fota_progress_notify(const fota_progress_msg_t* progress);


static ssize_t jis_battery_levels_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                       uint16_t offset);
static void jis_battery_levels_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
void jis_battery_levels_notify(uint8_t primary_level, uint8_t secondary_level);


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


static struct bt_uuid_128 CHARGE_STATUS_UUID =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ead, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// New UUID for packed battery levels characteristic
static struct bt_uuid_128 BATTERY_LEVELS_UUID =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec8, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));


static struct bt_uuid_128 foot_sensor_samples =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eaf, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));   
    
// BHI360 Data Set UUIDs
static struct bt_uuid_128 bhi360_data1_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb2, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
// Commented out - only quaternion data sent to BLE at 5Hz
// static struct bt_uuid_128 bhi360_data2_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb3, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
// static struct bt_uuid_128 bhi360_data3_uuid =
//     BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb4, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// FOTA Progress UUID
static struct bt_uuid_128 fota_progress_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372eb5, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// Activity Log UUIDs (primary device)
static struct bt_uuid_128 activity_log_available_primary_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec2, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 activity_log_path_primary_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec3, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// Step count UUIDs moved to Activity Metrics Service

// Weight Measurement UUID
static struct bt_uuid_128 weight_measurement_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec6, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

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

static struct bt_uuid_128 secondary_activity_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec0, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));
static struct bt_uuid_128 secondary_activity_log_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec1, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// Activity Metrics Streaming UUID
static struct bt_uuid_128 activity_metrics_header_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ecd, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

static struct bt_uuid_128 activity_metrics_data_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ece, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

static struct bt_uuid_128 activity_metrics_footer_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ecf, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

BT_GATT_SERVICE_DEFINE(
    info_service, BT_GATT_PRIMARY_SERVICE(&SENSING_INFO_SERVICE_UUID),
    // Current Ttime Characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME,
                            BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ,
                            jis_read_current_time, NULL,
                            ct),
    BT_GATT_CCC(jis_current_time_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Status Characteristic
    BT_GATT_CHARACTERISTIC(&STATUS_UUID.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ,
                            jis_status_read, nullptr,
                            static_cast<void *>(&device_status_bitfield)),
    BT_GATT_CCC(jis_status_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

        // Foot Sensor samples Characteristic
    BT_GATT_CHARACTERISTIC(&foot_sensor_samples.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ,
                            jis_foot_sensor_read, nullptr,
                            static_cast<void *>(&foot_sensor_aggregated_value)),
    BT_GATT_CCC(jis_foot_sensor_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),



     // Battery Status Characteristic
    BT_GATT_CHARACTERISTIC(&CHARGE_STATUS_UUID.uuid,
                            BT_GATT_CHRC_READ,
                            BT_GATT_PERM_READ,
                            jis_charge_status_read, nullptr,
                            static_cast<void *>(&charge_status)),
    BT_GATT_CCC(jis_charge_status_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // New characteristic for packed battery levels (primary and secondary)
    BT_GATT_CHARACTERISTIC(&BATTERY_LEVELS_UUID.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ,
                            jis_battery_levels_read, nullptr,
                            static_cast<void *>(&battery_levels)),
    BT_GATT_CCC(jis_battery_levels_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),


    // --- BHI360 Data Set Characteristics ---
    BT_GATT_CHARACTERISTIC(&bhi360_data1_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        jis_bhi360_data1_read, nullptr,
        static_cast<void *>(&quaternion_aggregated_value)),
    BT_GATT_CCC(jis_bhi360_data1_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Commented out - only quaternion data sent to BLE at 5Hz
    // Individual foot step count characteristic - DEPRECATED
    // Only aggregated step counts (total and activity) should be used by mobile apps
    // BT_GATT_CHARACTERISTIC(&bhi360_data2_uuid.uuid,
    //     BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
    //     BT_GATT_PERM_READ,
    //     jis_bhi360_data2_read, nullptr,
    //     static_cast<void *>(&bhi360_data2_value_fixed)),
    // BT_GATT_CCC(jis_bhi360_data2_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // BT_GATT_CHARACTERISTIC(&bhi360_data3_uuid.uuid,
    //     BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
    //     BT_GATT_PERM_READ,
    //     jis_bhi360_data3_read, nullptr,
    //     static_cast<void *>(&bhi360_data3_value_fixed)),
    // BT_GATT_CCC(jis_bhi360_data3_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Step count characteristics moved to Activity Metrics Service

    // Weight Measurement Characteristic
    BT_GATT_CHARACTERISTIC(&weight_measurement_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        jis_weight_measurement_read, nullptr,
        static_cast<void *>(&weight_kg_x10)),
    BT_GATT_CCC(jis_weight_measurement_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),


    // Packed Device Status Characteristic (new - combines multiple status fields)
    BT_GATT_CHARACTERISTIC(&device_status_packed_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        jis_device_status_packed_read, nullptr,
        static_cast<void *>(&device_status_packed)),
    BT_GATT_CCC(jis_device_status_packed_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // FOTA Progress Characteristic
    BT_GATT_CHARACTERISTIC(&fota_progress_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        jis_fota_progress_read, nullptr,
        static_cast<void *>(&fota_progress_value)),
    BT_GATT_CCC(jis_fota_progress_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Activity Log Available Characteristic
    BT_GATT_CHARACTERISTIC(&activity_log_available_primary_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        jis_activity_log_available_primary_read, nullptr,
        static_cast<void *>(&activity_log_available_primary)),
    BT_GATT_CCC(jis_activity_log_available_primary_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Activity Log Path Characteristic
    BT_GATT_CHARACTERISTIC(&activity_log_path_primary_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        jis_activity_log_path_primary_read, nullptr,
        static_cast<void *>(activity_log_path_primary)) ,
    BT_GATT_CCC(jis_activity_log_path_primary_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Secondary Device Information Characteristics
    BT_GATT_CHARACTERISTIC(&secondary_manufacturer_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        jis_secondary_info_read, nullptr,
        secondary_manufacturer),

    BT_GATT_CHARACTERISTIC(&secondary_model_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        jis_secondary_info_read, nullptr,
        secondary_model),

    BT_GATT_CHARACTERISTIC(&secondary_serial_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        jis_secondary_info_read, nullptr,
        secondary_serial),

    BT_GATT_CHARACTERISTIC(&secondary_hw_rev_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        jis_secondary_info_read, nullptr,
        secondary_hw_rev),

    BT_GATT_CHARACTERISTIC(&secondary_fw_rev_uuid.uuid,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        jis_secondary_info_read, nullptr,
        secondary_fw_rev),

    // Secondary FOTA Progress Characteristic
    BT_GATT_CHARACTERISTIC(&secondary_fota_progress_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        jis_secondary_fota_progress_read, nullptr,
        static_cast<void *>(&secondary_fota_progress_value)),
    BT_GATT_CCC(jis_secondary_fota_progress_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),


    // Secondary Activity Log Available
    BT_GATT_CHARACTERISTIC(&secondary_activity_log_available_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        jis_secondary_activity_log_available_read, nullptr,
        static_cast<void *>(&secondary_activity_log_available)),
    BT_GATT_CCC(jis_secondary_activity_log_available_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Secondary Activity Log Path
    BT_GATT_CHARACTERISTIC(&secondary_activity_log_path_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        jis_secondary_activity_log_path_read, nullptr,
        secondary_activity_log_path),
    BT_GATT_CCC(jis_secondary_activity_log_path_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // Activity Metrics Streaming Characteristic
BT_GATT_CHARACTERISTIC(&activity_metrics_header_uuid.uuid,
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
    BT_GATT_PERM_READ,
    jis_activity_metrics_header_read, nullptr,
    static_cast<void *>(&activity_header)),
BT_GATT_CCC(jis_activity_metrics_header_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

BT_GATT_CHARACTERISTIC(&activity_metrics_data_uuid.uuid,
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
    BT_GATT_PERM_READ,
    jis_activity_metrics_data_read, nullptr,
    static_cast<void *>(&activity_metrics_binary)),
BT_GATT_CCC(jis_activity_metrics_data_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

BT_GATT_CHARACTERISTIC(&activity_metrics_footer_uuid.uuid,
    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
    BT_GATT_PERM_READ,
    jis_activity_metrics_footer_read, nullptr,
    static_cast<void *>(&activity_footer)),
BT_GATT_CCC(jis_activity_metrics_footer_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif
    );

// clang-format on

// Helper function to safely send notifications
static int safe_gatt_notify(const struct bt_uuid *uuid, const void *data, uint16_t len)
{
    auto *gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, uuid);
    if (!gatt)
    {
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
    if (device_status_bitfield != new_status || !init_status_bt_update)
    {
        device_status_bitfield = new_status;
        if (status_subscribed)
        {
            auto *status_gatt = bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &STATUS_UUID.uuid);
            if (status_gatt)
            {
                bt_gatt_notify(nullptr, status_gatt, &device_status_bitfield, sizeof(device_status_bitfield));
                LOG_DBG("Device status updated and notified: 0x%08X", device_status_bitfield);
            }
            else
            {
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

    LOG_INF("Setting error status: error_code=%d, bit=0x%08X, new_status=0x%08X", (int)error_code, error_bit,
            new_status);

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
    if ((new_status & STATUS_ALL_ERRORS_MASK) == 0)
    {
        new_status &= ~STATUS_ERROR;
    }

    LOG_INF("Clearing error status: error_code=%d, bit=0x%08X, new_status=0x%08X", (int)error_code, error_bit,
            new_status);

    set_device_status(new_status);
}

void jis_foot_sensor_notify(const foot_samples_t *samples_data)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // PRIMARY DEVICE ONLY - handles aggregation and BLE notifications to phone

    // Always prepare aggregated data (for both notify and read)
    foot_sensor_aggregated_value.timestamp = k_uptime_get_32();

    // Copy primary data
    memcpy(foot_sensor_aggregated_value.primary, samples_data->values, sizeof(samples_data->values));

    // Copy secondary data if available
    if (g_secondary_data_valid)
    {
        memcpy(foot_sensor_aggregated_value.secondary, g_secondary_foot_data.values,
               sizeof(g_secondary_foot_data.values));
        //  LOG_INF("Aggregated foot data prepared - Primary[0]=%u, Secondary[0]=%u",
        //    foot_sensor_aggregated_value.primary[0], foot_sensor_aggregated_value.secondary[0]);
    }
    else
    {
        // No secondary data, send zeros
        memset(foot_sensor_aggregated_value.secondary, 0, sizeof(foot_sensor_aggregated_value.secondary));
        //  LOG_INF("Primary-only foot data prepared (secondary not available)");
    }

    // Send notification only if subscribed AND phone is connected
    if (ble_phone_is_connected())
    {
        auto *status_gatt =
            bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &foot_sensor_samples.uuid);
        if (status_gatt)
        {
            // Send aggregated format
            bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&foot_sensor_aggregated_value),
                           sizeof(foot_sensor_aggregated_value));
        }
    }

    // Store primary data for legacy compatibility
    memcpy(&foot_sensor_char_value, samples_data, sizeof(foot_samples_t));
#else
    // SECONDARY DEVICE - should NOT call this function!
    // Secondary device sends data via D2D TX, not via Information Service
    LOG_ERR("jis_foot_sensor_notify called on secondary device - this should not happen!");
    LOG_ERR("Secondary device should use ble_d2d_tx_send_foot_sensor_data() instead");
#endif
}

// Helper function to send BLE format data directly (used by recovery)
extern "C" void jis_foot_sensor_notify_ble(const foot_samples_ble_t *data)
{
    if (foot_sensor_subscribed && data) // Use correct subscription flag
    {
        auto *status_gatt =
            bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &foot_sensor_samples.uuid);
        if (status_gatt)
        {
            bt_gatt_notify(nullptr, status_gatt, static_cast<const void *>(data), sizeof(*data));
        }
    }
}

// Function to update secondary foot data and check if we should notify
void jis_foot_sensor_update_secondary(const foot_samples_t *secondary_data)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!secondary_data)
        return;

    // Update global secondary data
    memcpy(&g_secondary_foot_data, secondary_data, sizeof(foot_samples_t));
    g_secondary_data_valid = true;
    g_secondary_last_timestamp = k_uptime_get_32();

    // Update aggregated structure with secondary data
    memcpy(foot_sensor_aggregated_value.secondary, secondary_data->values, sizeof(secondary_data->values));

    LOG_DBG("Secondary foot data updated in aggregated structure");

    // Check if we have recent primary data (within 500ms)
    uint32_t now = k_uptime_get_32();
    if ((now - foot_sensor_aggregated_value.timestamp) < 500)
    {
        // We have recent primary data, send aggregated notification
        if (foot_sensor_subscribed)
        {
            auto *status_gatt =
                bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &foot_sensor_samples.uuid);
            if (status_gatt)
            {
                bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&foot_sensor_aggregated_value),
                               sizeof(foot_sensor_aggregated_value));
                LOG_DBG("Foot sensor aggregated notification sent after secondary update: %u bytes",
                        sizeof(foot_sensor_aggregated_value));
            }
        }
    }
#endif
}

// Similar function for quaternion data
void jis_bhi360_data1_update_secondary(const bhi360_3d_mapping_t *secondary_data)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!secondary_data)
        return;

    // Update global secondary quaternion data
    g_secondary_quat_data.quat_x = float_to_fixed16(secondary_data->accel_x, FixedPoint::QUAT_SCALE);
    g_secondary_quat_data.quat_y = float_to_fixed16(secondary_data->accel_y, FixedPoint::QUAT_SCALE);
    g_secondary_quat_data.quat_z = float_to_fixed16(secondary_data->accel_z, FixedPoint::QUAT_SCALE);
    g_secondary_quat_data.quat_w = float_to_fixed16(secondary_data->quat_w, FixedPoint::QUAT_SCALE);
    g_secondary_data_valid = true;
    g_secondary_last_timestamp = k_uptime_get_32();

    // Update aggregated structure with secondary data
    quaternion_aggregated_value.secondary_x = g_secondary_quat_data.quat_x;
    quaternion_aggregated_value.secondary_y = g_secondary_quat_data.quat_y;
    quaternion_aggregated_value.secondary_z = g_secondary_quat_data.quat_z;
    quaternion_aggregated_value.secondary_w = g_secondary_quat_data.quat_w;

    LOG_DBG("Secondary quaternion data updated in aggregated structure");

    // Check if we have recent primary data (within 500ms)
    uint32_t now = k_uptime_get_32();
    if ((now - quaternion_aggregated_value.timestamp) < 500)
    {
        // We have recent primary data, send aggregated notification
        if (bhi360_data1_subscribed)
        {
            safe_gatt_notify(&bhi360_data1_uuid.uuid, static_cast<const void *>(&quaternion_aggregated_value),
                             sizeof(quaternion_aggregated_value));
            LOG_DBG("Quaternion aggregated notification sent after secondary update: %u bytes",
                    sizeof(quaternion_aggregated_value));
        }
    }
#endif
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
    if (!attr)
    {
        LOG_ERR("jis_status_ccc_cfg_changed: attr is NULL");
        return;
    }
    status_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Status CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static void jis_foot_sensor_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("jis_foot_sensor_ccc_cfg_changed: attr is NULL");
        return;
    }
    foot_sensor_subscribed = value == BT_GATT_CCC_NOTIFY; // Use separate flag
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
    if (!attr)
    {
        LOG_ERR("jis_current_time_ccc_cfg_changed: attr is NULL");
        return;
    }
    current_time_notifications_enabled = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Time CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_read_current_time(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                     uint16_t offset)
{
    // 1. Request cts.cpp to update its internal Current Time characteristic
    // buffer. This ensures the buffer contains the latest calendar time.
    update_cts_characteristic_buffer();

    // 2. Get a pointer to the updated buffer and its size from cts.cpp.
    const void *cts_value_ptr = get_current_time_char_value_ptr();
    size_t cts_value_size = get_current_time_char_value_size();

    // 3. Check if the time is valid before attempting to read.
    // (This check depends on how update_cts_characteristic_buffer handles
    // unsynced time. If it zeros the buffer, checking the first byte might be
    // sufficient, or you could have a separate flag/function in cts.cpp to check
    // sync status). For simplicity, let's assume if cts_value_size is 0 or the
    // pointer is null, it's invalid.
    if (cts_value_ptr == nullptr || cts_value_size == 0)
    {
        LOG_WRN("Current Time characteristic data is not ready or valid.");
        // Return 0 bytes read or an error code if appropriate.
        // A common practice for GATT reads is to return 0 bytes if data is
        // unavailable.
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
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Return aggregated data for primary device
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &foot_sensor_aggregated_value,
                             sizeof(foot_sensor_aggregated_value));
#else
    // Secondary device returns simple format
    const foot_samples_t *value_to_read = static_cast<const foot_samples_t *>(attr->user_data);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value_to_read, sizeof(foot_samples_t));
#endif
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
    if (info_service.attrs && info_service.attr_count > 1)
    {
        bt_gatt_notify(NULL, &info_service.attrs[1], &ct, sizeof(ct));
    }
    else
    {
        LOG_WRN("Information service not ready for CTS notification");
    }
}

/**
 * @brief Update battery levels for both primary and secondary devices
 *
 * @param primary_level Primary device battery level (0-100%)
 * @param secondary_level Secondary device battery level (0-100%, 0 =
 * unknown/disconnected)
 */
void jis_battery_levels_notify(uint8_t primary_level, uint8_t secondary_level)
{
    LOG_DBG("Battery levels - Primary: %d%%, Secondary: %d%%", primary_level, secondary_level);

#if IS_ENABLED(CONFIG_TEST_RANDOM_GENERATOR)               // This is for testing only!!!!!!
    primary_level = (uint8_t)((sys_rand32_get() % 101));   // 0 to 100
    secondary_level = (uint8_t)((sys_rand32_get() % 101)); // 0 to 100
#endif

    battery_levels[0] = primary_level;
    battery_levels[1] = secondary_level;

    if (info_service.attrs && info_service.attr_count > 0)
    {
        // Find the battery characteristic and notify
        auto *battery_gatt =
            bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &BATTERY_LEVELS_UUID.uuid);
        if (battery_gatt)
        {
            bt_gatt_notify(nullptr, battery_gatt, static_cast<void *>(&battery_levels), sizeof(battery_levels));
        }
    }
    else
    {
        LOG_WRN("Information service not ready for battery levels notification");
    }
}

/**
 * @brief Read callback for battery levels characteristic
 */
static ssize_t jis_battery_levels_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                       uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, static_cast<void *>(&battery_levels),
                             sizeof(battery_levels));
}

/**
 * @brief CCC change callback for battery levels characteristic
 */
static void jis_battery_levels_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("jis_battery_levels_ccc_cfg_changed: attr is NULL");
        return;
    }
    LOG_DBG("Battery Levels CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

/**
 * @brief Update primary battery level
 *
 * @param level Primary device battery level (0-100%)
 */
extern "C" void jis_update_primary_battery(uint8_t level)
{
    LOG_DBG("Updating primary battery level: %d%%", level);
    battery_levels[0] = level;
    jis_battery_levels_notify(battery_levels[0], battery_levels[1]);
}

/**
 * @brief Update secondary battery level
 *
 * @param level Secondary device battery level (0-100%, 0 =
 * unknown/disconnected)
 */
void jis_update_secondary_battery(uint8_t level)
{
    LOG_DBG("Updating secondary battery level: %d%%", level);
    battery_levels[1] = level;
    jis_battery_levels_notify(battery_levels[0], battery_levels[1]);
}

/**
 * @brief Legacy function for charge status (kept for compatibility)
 *
 * @param new_charge_status
 */
void jis_charge_status_notify(uint8_t new_charge_status)
{
    LOG_DBG("charge status: %d", new_charge_status);

    charge_status = new_charge_status;

    if (info_service.attrs && info_service.attr_count > 0)
    {
        bt_gatt_notify_uuid(nullptr, &CHARGE_STATUS_UUID.uuid, info_service.attrs, static_cast<void *>(&charge_status),
                            sizeof(charge_status));
    }
    else
    {
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
    if (!attr)
    {
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

// --- BHI360 Data Set 1 ---
static void jis_bhi360_data1_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("jis_bhi360_data1_ccc_cfg_changed: attr is NULL");
        return;
    }
    bhi360_data1_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("BHI360 Data1 CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}
static ssize_t jis_bhi360_data1_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                     uint16_t offset)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Return aggregated quaternion data for primary device
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &quaternion_aggregated_value,
                             sizeof(quaternion_aggregated_value));
#else
    // Secondary device returns fixed-point format
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(bhi360_3d_mapping_fixed_t));
#endif
}
extern "C" void jis_bhi360_data1_notify(const bhi360_3d_mapping_t *data)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // PRIMARY DEVICE ONLY - handles aggregation and BLE notifications to phone

    // Always prepare aggregated quaternion data (for both notify and read)
    quaternion_aggregated_value.timestamp = k_uptime_get_32();

    // Convert primary data to fixed-point
    quaternion_aggregated_value.primary_x = float_to_fixed16(data->accel_x, FixedPoint::QUAT_SCALE);
    quaternion_aggregated_value.primary_y = float_to_fixed16(data->accel_y, FixedPoint::QUAT_SCALE);
    quaternion_aggregated_value.primary_z = float_to_fixed16(data->accel_z, FixedPoint::QUAT_SCALE);
    quaternion_aggregated_value.primary_w = float_to_fixed16(data->quat_w, FixedPoint::QUAT_SCALE);

    // Copy secondary data if available
    if (g_secondary_data_valid)
    {
        quaternion_aggregated_value.secondary_x = g_secondary_quat_data.quat_x;
        quaternion_aggregated_value.secondary_y = g_secondary_quat_data.quat_y;
        quaternion_aggregated_value.secondary_z = g_secondary_quat_data.quat_z;
        quaternion_aggregated_value.secondary_w = g_secondary_quat_data.quat_w;
        LOG_DBG("Aggregated quaternion prepared - Primary(%.2f,%.2f,%.2f,%.2f), Secondary(fixed)", data->accel_x,
                data->accel_y, data->accel_z, data->quat_w);
    }
    else
    {
        // No secondary data, send zeros
        quaternion_aggregated_value.secondary_x = 0;
        quaternion_aggregated_value.secondary_y = 0;
        quaternion_aggregated_value.secondary_z = 0;
        quaternion_aggregated_value.secondary_w = 0;
        LOG_DBG("Primary-only quaternion prepared (secondary not available)");
    }

    // Send notification only if subscribed AND phone is connected
    if (ble_phone_is_connected())
    {
        safe_gatt_notify(&bhi360_data1_uuid.uuid, static_cast<const void *>(&quaternion_aggregated_value),
                         sizeof(quaternion_aggregated_value));
        LOG_DBG("Quaternion aggregated notification sent: %u bytes", sizeof(quaternion_aggregated_value));
    }

    // Store primary data for legacy compatibility
    convert_3d_mapping_to_fixed(*data, bhi360_data1_value_fixed);
#else
    // SECONDARY DEVICE - should NOT call this function!
    // Secondary device sends data via D2D TX, not via Information Service
    LOG_ERR("jis_bhi360_data1_notify called on secondary device - this should not happen!");
    LOG_ERR("Secondary device should use ble_d2d_tx_send_bhi360_data1() instead");
#endif
}

// Helper function to send BLE format data directly (used by recovery)
extern "C" void jis_bhi360_data1_notify_ble(const bhi360_3d_mapping_ble_t *data)
{
    if (bhi360_data1_subscribed && data)
    {
        safe_gatt_notify(&bhi360_data1_uuid.uuid, static_cast<const void *>(data), sizeof(*data));
    }
}

// Commented out - only quaternion data sent to BLE at 5Hz
// --- BHI360 Data Set 2 ---
// static void jis_bhi360_data2_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
// {
//     if (!attr)
//     {
//         LOG_ERR("jis_bhi360_data2_ccc_cfg_changed: attr is NULL");
//         return;
//     }
//     bhi360_data2_subscribed = value == BT_GATT_CCC_NOTIFY;
//     LOG_DBG("BHI360 Data2 CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
// }
// static ssize_t jis_bhi360_data2_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
//                                      uint16_t offset)
// {
//     return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(bhi360_step_count_fixed_t));
// }

// extern "C" void jis_bhi360_data2_notify(const bhi360_step_count_t *data)
// {
//     // Step count data is already integers, just copy
//     bhi360_data2_value_fixed.step_count = data->step_count;
//     bhi360_data2_value_fixed.activity_duration_s = 0; // Deprecated - always 0

//     if (bhi360_data2_subscribed)
//     {
//         safe_gatt_notify(&bhi360_data2_uuid.uuid, static_cast<const void *>(&bhi360_data2_value_fixed),
//                          sizeof(bhi360_data2_value_fixed));
//     }

// #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
//     // Also update secondary step count for aggregation
//     // This function is called from d2d_data_handler when secondary data arrives
//     // We need to tell the bluetooth module about the secondary update
//     static uint32_t last_secondary_steps = 0;
//     static uint32_t last_secondary_duration = 0;

//     // Check if this is likely secondary data (different from last known
//     // secondary) This is a bit of a hack - ideally we'd have a separate function
//     // for secondary
//     if (data->step_count != last_secondary_steps || data->activity_duration_s != last_secondary_duration)
//     {
//         // Send a message to bluetooth module with secondary step count
//         generic_message_t msg = {};
//         msg.sender = SENDER_D2D_SECONDARY; // Indicate it's from D2D
//         msg.type = MSG_TYPE_BHI360_STEP_COUNT;
//         msg.data.bhi360_step_count = *data;

//         if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) == 0)
//         {
//             last_secondary_steps = data->step_count;
//             last_secondary_duration = data->activity_duration_s;
//             LOG_DBG("Sent secondary step count to bluetooth module for aggregation");
//         }
//     }
// #endif
// }

// --- BHI360 Data Set 3 ---
// static void jis_bhi360_data3_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
// {
//     if (!attr)
//     {
//         LOG_ERR("jis_bhi360_data3_ccc_cfg_changed: attr is NULL");
//         return;
//     }
//     bhi360_data3_subscribed = value == BT_GATT_CCC_NOTIFY;
//     LOG_DBG("BHI360 Data3 CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
// }

// static ssize_t jis_bhi360_data3_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
//                                      uint16_t offset)
// {
//     return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(bhi360_linear_accel_fixed_t));
// }

// void jis_bhi360_data3_notify(const bhi360_linear_accel_t *data)
// {
//     // Convert float data to fixed-point for BLE transmission
//     convert_linear_accel_to_fixed(*data, bhi360_data3_value_fixed);

//     if (bhi360_data3_subscribed)
//     {
//         // Convert to BLE format with sequence number
//         bhi360_linear_accel_ble_t ble_data;
//         BleSequenceManager::getInstance().addBhi360Accel(data, &ble_data);

//         safe_gatt_notify(&bhi360_data3_uuid.uuid, static_cast<const void *>(&ble_data), sizeof(ble_data));
//     }
// }

// Helper function to send BLE format data directly (used by recovery)
// extern "C" void jis_bhi360_data3_notify_ble(const bhi360_linear_accel_ble_t *data)
// {
//     if (bhi360_data3_subscribed && data)
//     {
//         safe_gatt_notify(&bhi360_data3_uuid.uuid, static_cast<const void *>(data), sizeof(*data));
//     }
// }

// --- FOTA Progress ---
static void jis_fota_progress_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("jis_fota_progress_ccc_cfg_changed: attr is NULL");
        return;
    }
    fota_progress_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("FOTA Progress CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_fota_progress_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                      uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(fota_progress_value));
}

void jis_fota_progress_notify(const fota_progress_msg_t *progress)
{
    // If FOTA is starting (transitioning to active with status 1), ensure clean
    // values
    if (progress->is_active && progress->status == 1 && progress->bytes_received == 0 &&
        progress->percent_complete == 0)
    {
        // This is a fresh start, clear any stale data
        memset(&fota_progress_value, 0, sizeof(fota_progress_value));
    }

    memcpy(&fota_progress_value, progress, sizeof(fota_progress_value));

    LOG_DBG("FOTA Progress: active=%d, status=%d, percent=%d%%, bytes=%u/%u", fota_progress_value.is_active,
            fota_progress_value.status, fota_progress_value.percent_complete, fota_progress_value.bytes_received,
            fota_progress_value.total_size);

    if (fota_progress_subscribed)
    {
        safe_gatt_notify(&fota_progress_uuid.uuid, static_cast<const void *>(&fota_progress_value),
                         sizeof(fota_progress_value));
    }
}

static ssize_t jis_activity_metrics_header_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                                uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &activity_header, sizeof(activity_header));
}

static ssize_t jis_activity_metrics_data_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                              uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &activity_metrics_binary, sizeof(activity_metrics_binary));
}

static ssize_t jis_activity_metrics_footer_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                                uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &activity_footer, sizeof(activity_footer));
}

static void jis_activity_metrics_header_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    activity_header_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Activity Metrics Header CCC Notify: %d", activity_header_subscribed);
}

static void jis_activity_metrics_data_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    activity_metrics_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Activity Metrics Data CCC Notify: %d", activity_metrics_subscribed);
}

static void jis_activity_metrics_footer_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    activity_footer_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Activity Metrics Footer CCC Notify: %d", activity_footer_subscribed);
}

extern "C" void jis_activity_metrics_send_header(const ActivityFileHeaderV3 *header)
{
    if (!header)
        return;

    memcpy(&activity_header, header, sizeof(activity_header));

    if (activity_header_subscribed)
    {
        safe_gatt_notify(&activity_metrics_header_uuid.uuid, static_cast<const void *>(&activity_header),
                         sizeof(activity_header));
        LOG_INF("Activity Metrics Header sent");
    }
}

extern "C" void jis_activity_metrics_send_packet(const activity_metrics_binary_t *metrics)
{
    if (!metrics)
        return;
    
    memcpy(&activity_metrics_binary, metrics, sizeof(activity_metrics_binary));

    if (activity_metrics_subscribed)
    {
        safe_gatt_notify(&activity_metrics_data_uuid.uuid, static_cast<const void *>(&activity_metrics_binary),
                         sizeof(activity_metrics_binary));
        LOG_DBG("Activity Metrics Packet sent: #%u", activity_metrics_binary.packet_number);
    }
}

extern "C" void jis_activity_metrics_send_footer(const ActivityFileFooterV3 *footer)
{
    if (!footer)
        return;

    memcpy(&activity_footer, footer, sizeof(activity_footer));

    if (activity_footer_subscribed)
    {
        safe_gatt_notify(&activity_metrics_footer_uuid.uuid, static_cast<const void *>(&activity_footer),
                         sizeof(activity_footer));
        LOG_INF("Activity Metrics Footer sent");
    }
}

// --- Activity Log Handlers ---
static void jis_activity_log_available_primary_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("jis_activity_log_available_primary_ccc_cfg_changed: attr is NULL");
        return;
    }
    activity_log_available_primary_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Activity Log Available CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_activity_log_available_primary_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                                       uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &activity_log_available_primary,
                             sizeof(activity_log_available_primary));
}

extern "C" void jis_activity_log_available_primary_notify(uint8_t log_id)
{
    activity_log_available_primary = log_id;
    LOG_INF("Activity Log Available: ID=%u", log_id);

    auto *status_gatt =
        bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &activity_log_available_primary_uuid.uuid);
    bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&activity_log_available_primary),
                   sizeof(activity_log_available_primary));
}

static void jis_activity_log_path_primary_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("jis_activity_log_path_ccc_cfg_changed: attr is NULL");
        return;
    }
    activity_log_path_primary_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Activity Log Path CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_activity_log_path_primary_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                                  uint16_t len, uint16_t offset)
{

    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, strlen(activity_log_path_primary));
}

extern "C" void jis_activity_log_path_primary_notify(const char *path)
{
    memset(activity_log_path_primary, 0, sizeof(activity_log_path_primary));
    strncpy(activity_log_path_primary, path, sizeof(activity_log_path_primary) - 1);
    activity_log_path_primary[sizeof(activity_log_path_primary) - 1] = '\0';

    LOG_INF("Activity Log Path: %s", activity_log_path_primary);

    auto *status_gatt =
        bt_gatt_find_by_uuid(info_service.attrs, info_service.attr_count, &activity_log_path_primary_uuid.uuid);
    bt_gatt_notify(nullptr, status_gatt, static_cast<void *>(&activity_log_path_primary),
                   sizeof(activity_log_path_primary));
}

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
// Secondary device info read handler
static ssize_t jis_secondary_info_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                                       uint16_t offset)
{
    const char *value = static_cast<const char *>(attr->user_data);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

// Function to update secondary device information
void jis_update_secondary_device_info(const char *manufacturer, const char *model, const char *serial,
                                      const char *hw_rev, const char *fw_rev)
{
    if (manufacturer)
    {
        strncpy(secondary_manufacturer, manufacturer, sizeof(secondary_manufacturer) - 1);
        secondary_manufacturer[sizeof(secondary_manufacturer) - 1] = '\0';
    }
    if (model)
    {
        strncpy(secondary_model, model, sizeof(secondary_model) - 1);
        secondary_model[sizeof(secondary_model) - 1] = '\0';
    }
    if (serial)
    {
        strncpy(secondary_serial, serial, sizeof(secondary_serial) - 1);
        secondary_serial[sizeof(secondary_serial) - 1] = '\0';
    }
    if (hw_rev)
    {
        strncpy(secondary_hw_rev, hw_rev, sizeof(secondary_hw_rev) - 1);
        secondary_hw_rev[sizeof(secondary_hw_rev) - 1] = '\0';
    }
    if (fw_rev)
    {
        strncpy(secondary_fw_rev, fw_rev, sizeof(secondary_fw_rev) - 1);
        secondary_fw_rev[sizeof(secondary_fw_rev) - 1] = '\0';
    }

    LOG_INF("Updated secondary device info: Mfg=%s, Model=%s, Serial=%s, HW=%s, "
            "FW=%s",
            secondary_manufacturer, secondary_model, secondary_serial, secondary_hw_rev, secondary_fw_rev);
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
    if (!attr)
    {
        LOG_ERR("jis_secondary_fota_progress_ccc_cfg_changed: attr is NULL");
        return;
    }
    secondary_fota_progress_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Secondary FOTA Progress CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_secondary_fota_progress_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                                uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(secondary_fota_progress_value));
}

void jis_secondary_fota_progress_notify(const fota_progress_msg_t *progress)
{
    memcpy(&secondary_fota_progress_value, progress, sizeof(secondary_fota_progress_value));

    LOG_INF("Secondary FOTA Progress: active=%d, status=%d, percent=%d%%, "
            "bytes=%u/%u",
            secondary_fota_progress_value.is_active, secondary_fota_progress_value.status,
            secondary_fota_progress_value.percent_complete, secondary_fota_progress_value.bytes_received,
            secondary_fota_progress_value.total_size);

    if (secondary_fota_progress_subscribed)
    {
        safe_gatt_notify(&secondary_fota_progress_uuid.uuid, static_cast<const void *>(&secondary_fota_progress_value),
                         sizeof(secondary_fota_progress_value));
    }
}

// Secondary Activity Log Available handlers
static void jis_secondary_activity_log_available_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("jis_secondary_activity_log_available_ccc_cfg_changed: attr is NULL");
        return;
    }
    secondary_activity_log_available_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Secondary Activity Log Available CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_secondary_activity_log_available_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                         void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(secondary_activity_log_available));
}

void jis_secondary_activity_log_available_notify(uint8_t log_id)
{
    secondary_activity_log_available = log_id;
    LOG_INF("Secondary Activity Log Available: ID=%u", log_id);

    if (secondary_activity_log_available_subscribed)
    {
        safe_gatt_notify(&secondary_activity_log_available_uuid.uuid,
                         static_cast<const void *>(&secondary_activity_log_available),
                         sizeof(secondary_activity_log_available));
    }
}

// Secondary Activity Log Path handlers
static void jis_secondary_activity_log_path_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("jis_secondary_activity_log_path_ccc_cfg_changed: attr is NULL");
        return;
    }
    secondary_activity_log_path_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Secondary Activity Log Path CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_secondary_activity_log_path_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                                    uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, secondary_activity_log_path,
                             strlen(secondary_activity_log_path) + 1);
}

void jis_secondary_activity_log_path_notify(const char *path)
{
    memset(secondary_activity_log_path, 0, sizeof(secondary_activity_log_path));
    strncpy(secondary_activity_log_path, path, sizeof(secondary_activity_log_path) - 1);
    secondary_activity_log_path[sizeof(secondary_activity_log_path) - 1] = '\0';

    LOG_INF("Secondary Activity Log Path: %s", secondary_activity_log_path);

    if (secondary_activity_log_path_subscribed)
    {
        safe_gatt_notify(&secondary_activity_log_path_uuid.uuid,
                         static_cast<const void *>(&secondary_activity_log_path), strlen(secondary_activity_log_path));
    }
}
#endif

// Total step count handlers moved to Activity Metrics Service

// --- Weight Measurement Handlers ---
static void jis_weight_measurement_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("jis_weight_measurement_ccc_cfg_changed: attr is NULL");
        return;
    }
    weight_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Weight Measurement CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_weight_measurement_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                           uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(weight_kg_x10));
}

extern "C" void jis_weight_measurement_notify(float weight_kg)
{
    // Weight is already calibrated by activity metrics module
    // Just convert to uint16_t with 0.1kg precision and send via BLE
    weight_kg_x10 = (uint16_t)(weight_kg * 10);

    LOG_INF("Weight Measurement: %.1f kg (BLE value=%u)", weight_kg, weight_kg_x10);

    if (weight_subscribed)
    {
        safe_gatt_notify(&weight_measurement_uuid.uuid, static_cast<const void *>(&weight_kg_x10),
                         sizeof(weight_kg_x10));
    }
}

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
// Function to handle weight measurement from secondary device
extern "C" void jis_secondary_weight_measurement_notify(float weight_kg)
{
    LOG_INF("Received weight from secondary device: %.1f kg", weight_kg);

    // Only process if weight was requested by phone
    if (weight_requested_by_phone)
    {
        // Apply primary device calibration and notify
        jis_weight_measurement_notify(weight_kg);

        // Reset request flag
        weight_requested_by_phone = false;
    }
    else
    {
        LOG_DBG("Weight received but not requested by phone, ignoring");
    }
}
#endif

// --- Packed Device Status Handlers ---
static void jis_device_status_packed_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (!attr)
    {
        LOG_ERR("jis_device_status_packed_ccc_cfg_changed: attr is NULL");
        return;
    }
    device_status_packed_subscribed = value == BT_GATT_CCC_NOTIFY;
    LOG_DBG("Packed Device Status CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY));
}

static ssize_t jis_device_status_packed_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                                             uint16_t len, uint16_t offset)
{
    // Update packed structure before reading
    jis_device_status_packed_notify();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, sizeof(device_status_packed_t));
}

extern "C" void jis_device_status_packed_notify(void)
{
    // Update the packed structure with current values
    device_status_packed.status_bitfield = device_status_bitfield;
    device_status_packed.battery_percent = battery_levels[0]; // Primary battery level
    device_status_packed.charge_status = charge_status;
    device_status_packed.temperature_c = 25; // TODO: Get from temperature sensor
    device_status_packed.activity_state = (device_status_bitfield & STATUS_LOGGING_ACTIVE) ? 1 : 0;
    device_status_packed.uptime_seconds = k_uptime_get_32() / 1000;
    device_status_packed.free_storage_mb = 0;   // TODO: Get from filesystem
    device_status_packed.connected_devices = 0; // TODO: Count connected devices

    if (device_status_packed_subscribed)
    {
        safe_gatt_notify(&device_status_packed_uuid.uuid, static_cast<const void *>(&device_status_packed),
                         sizeof(device_status_packed));
    }
}

// --- Activity State Management Functions ---

/**
 * @brief Update the activity state in the status bitfield
 *
 * @param new_state The new activity state to set
 */
void jis_update_activity_state(activity_state_t new_state)
{
    // Save previous state if transitioning to pause
    if (new_state == ACTIVITY_STATE_PAUSED && current_activity_state != ACTIVITY_STATE_PAUSED)
    {
        pre_pause_activity_state = current_activity_state;
        LOG_DBG("Saving pre-pause state: %d", pre_pause_activity_state);
    }

    current_activity_state = new_state;

    // Clear all activity bits first
    uint32_t status = device_status_bitfield & ~STATUS_ACTIVITY_MASK;

    // Set new activity state bits
    switch (new_state)
    {
        case ACTIVITY_STATE_1_RUNNING:
            status |= STATUS_ACTIVITY_1_RUNNING;
            LOG_INF("Activity state: RUNNING (1)");
            break;
        case ACTIVITY_STATE_3_FOOT_STREAM:
            status |= STATUS_ACTIVITY_3_FOOT_STREAM;
            LOG_INF("Activity state: FOOT STREAMING (3)");
            break;
        case ACTIVITY_STATE_4_QUAT_STREAM:
            status |= STATUS_ACTIVITY_4_QUAT_STREAM;
            LOG_INF("Activity state: QUATERNION STREAMING (4)");
            break;
        case ACTIVITY_STATE_5_BOTH_STREAM:
            status |= STATUS_ACTIVITY_5_BOTH_STREAM;
            LOG_INF("Activity state: BOTH STREAMING (5)");
            break;
        case ACTIVITY_STATE_PAUSED:
            status |= STATUS_ACTIVITY_PAUSED;
            LOG_INF("Activity state: PAUSED");
            break;
        case ACTIVITY_STATE_IDLE:
        default:
            // No bits set for idle
            LOG_INF("Activity state: IDLE");
            break;
    }

    set_device_status(status);
}

/**
 * @brief Get the current activity state
 *
 * @return The current activity state
 */
extern "C" activity_state_t jis_get_activity_state(void)
{
    return current_activity_state;
}

/**
 * @brief Restore the activity state from before pause
 */
extern "C" void jis_restore_activity_state_from_pause(void)
{
    if (pre_pause_activity_state != ACTIVITY_STATE_IDLE && pre_pause_activity_state != ACTIVITY_STATE_PAUSED)
    {
        LOG_INF("Restoring activity state from pause: %d", pre_pause_activity_state);
        jis_update_activity_state(pre_pause_activity_state);
    }
    else
    {
        // Default to activity 1 if no valid previous state
        LOG_INF("No valid pre-pause state, defaulting to RUNNING");
        jis_update_activity_state(ACTIVITY_STATE_1_RUNNING);
    }
}

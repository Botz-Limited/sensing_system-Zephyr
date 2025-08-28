/**
 * @file ble_services.hpp
 * @brief BLE services header file
 */

#ifndef BLE_SERVICES_HPP
#define BLE_SERVICES_HPP

#include <zephyr/bluetooth/gatt.h>
#include <app.hpp>

#ifdef __cplusplus
extern "C" {
#endif

// Service initialization
int ble_services_init(void);

// Information service functions
void set_device_status(uint32_t new_status);
void jis_weight_measurement_notify(float weight_kg);
void jis_set_err_status_notify(err_t error_code);
void jis_device_status_packed_notify(void);
void jis_clear_err_status_notify(err_t error_code);
void jis_foot_sensor_notify(const foot_samples_t *samples_data);
void jis_charge_status_notify(uint8_t new_charge_status);
void jis_foot_sensor_log_available_notify(uint8_t log_id);
void jis_foot_sensor_req_id_path_notify(const char *file_path);
void jis_bhi360_log_available_notify(uint8_t log_id);
void jis_bhi360_req_id_path_notify(const char *file_path);
void jis_bhi360_data1_notify(const bhi360_3d_mapping_t* data);
void jis_bhi360_data2_notify(const bhi360_step_count_t* data);
void jis_bhi360_data3_notify(const bhi360_linear_accel_t* data);
void jis_fota_progress_notify(const fota_progress_msg_t* progress);
void jis_activity_log_available_primary_notify(uint8_t log_id);
void jis_activity_log_path_primary_notify(const char *file_path);
void jis_update_secondary_battery(uint8_t level);
void cts_notify(void);

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
void jis_update_secondary_device_info(const char *manufacturer, const char *model, 
                                     const char *serial, const char *hw_rev, const char *fw_rev);
void jis_clear_secondary_device_info(void);
void jis_secondary_fota_progress_notify(const fota_progress_msg_t* progress);
void jis_secondary_weight_measurement_notify(float weight_kg);

// Secondary file management notifications
void jis_secondary_foot_log_available_notify(uint8_t log_id);
void jis_secondary_foot_log_path_notify(const char* path);
void jis_secondary_bhi360_log_available_notify(uint8_t log_id);
void jis_secondary_bhi360_log_path_notify(const char* path);
void jis_secondary_activity_log_available_notify(uint8_t log_id);
void jis_secondary_activity_log_path_notify(const char* path);
void jis_update_primary_battery(uint8_t level);
#endif

// CTS functions
void update_cts_characteristic_buffer(void);
const void* get_current_time_char_value_ptr(void);
size_t get_current_time_char_value_size(void);
int init_rtc_time(void);
uint32_t get_current_epoch_time(void);

// Device Information Service functions
const char* dis_get_manufacturer(void);
const char* dis_get_model(void);
const char* dis_get_serial(void);
const char* dis_get_hw_revision(void);
const char* dis_get_fw_revision(void);

#ifdef __cplusplus
}

// C++ only functions (for aggregation)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
void jis_foot_sensor_update_secondary(const foot_samples_t *secondary_data);
void jis_bhi360_data1_update_secondary(const bhi360_3d_mapping_t *secondary_data);
#endif

#endif

#endif // BLE_SERVICES_HPP
/**
 * @file ble_services.hpp
 * @brief
 * @version 1.0
 * @date 15/5/2025
 *
 * @copyright Botz Innovation 2025
 *
 */
#ifndef APP_INCLUDE_BLE_SERVICES_HEADER_
#define APP_INCLUDE_BLE_SERVICES_HEADER_

#include "time.h"
#include <cstdint>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <app.hpp>
#include <errors.hpp>

void jis_set_err_status_notify(err_t new_device_status);
void jis_clear_err_status_notify(err_t new_device_status);
void jis_charge_status_notify(uint8_t new_charge_status);
void jis_foot_sensor_notify(const foot_samples_t *samples_data);
void jis_foot_sensor_log_available_notify(uint8_t log_id);
void jis_foot_sensor_req_id_path_notify(const char *file_path);
void jis_bhi360_log_available_notify(uint8_t log_id);
void jis_bhi360_req_id_path_notify(const char *file_path);

// BHI360 Bluetooth notify functions
void jis_bhi360_quaternion_notify(const float* quat);
void jis_bhi360_linear_accel_notify(const float* lacc);
void jis_bhi360_step_count_notify(uint32_t step_count);

void cs_log_data_notify(uint8_t stu);
void cts_notify(void);
void cs_log_available_hw_notify(uint32_t stu);
void cs_log_available_meta_notify(uint32_t stu);
void cs_req_id_hw_notify(const char *stu);
void cs_req_id_meta_notify(const char *stu);
void cs_set_time_control_command_notify(uint32_t stu);
void cs_factory_reset_command_notify(uint8_t *stu);
void cs_delete_log_hw_control_command_notify(uint32_t stu);
void cs_delete_log_meta_control_command_notify(uint32_t stu);
void set_current_time_from_epoch(uint32_t new_epoch_time_s);
uint32_t get_current_epoch_time(void);
int init_rtc_time();
void update_cts_characteristic_buffer(void);
const void* get_current_time_char_value_ptr(void);
size_t get_current_time_char_value_size(void);



#define VND_MAX_LEN 20
#define BLE_ADVERTISING_TIMEOUT_MS 30000

#define bluetooth_timer 1000


#endif // APP_INCLUDE_BLE_SERVICES_HEADER_

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

#include "ble_data_seq.hpp"
#include "time.h"
#include <app.hpp>
#include <cstdint>
#include <errors.hpp>
#include <stdint.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Activity state enum (matches information_service.cpp)
typedef enum
{
    ACTIVITY_STATE_IDLE = 0,
    ACTIVITY_STATE_1_RUNNING,
    ACTIVITY_STATE_3_FOOT_STREAM,
    ACTIVITY_STATE_4_QUAT_STREAM,
    ACTIVITY_STATE_5_BOTH_STREAM,
    ACTIVITY_STATE_PAUSED
} activity_state_t;

// Activity state management functions
void jis_update_activity_state(activity_state_t new_state);
activity_state_t jis_get_activity_state(void);
void jis_restore_activity_state_from_pause(void);

// Status notification functions
void jis_set_err_status_notify(err_t new_device_status);
void jis_clear_err_status_notify(err_t new_device_status);
void jis_charge_status_notify(uint8_t new_charge_status);
void jis_battery_levels_notify(uint8_t primary_level, uint8_t secondary_level);
void jis_foot_sensor_notify(const foot_samples_t *samples_data);
void jis_foot_sensor_log_available_notify(uint8_t log_id);
void jis_foot_sensor_req_id_path_notify(const char *file_path);
void jis_bhi360_log_available_notify(uint8_t log_id);
void jis_bhi360_req_id_path_notify(const char *file_path);
extern "C" void jis_activity_metrics_send_packet(const activity_metrics_binary_t *metrics);
extern "C" void jis_activity_metrics_send_header(const ActivityFileHeaderV3 *header);
extern "C" void jis_activity_metrics_send_footer(const ActivityFileFooterV3 *footer);

// Device status setter/getter (bitfield)
void set_device_status(uint32_t new_status);
uint32_t get_device_status(void);

// BHI360 Bluetooth notify functions
void jis_bhi360_data1_notify(const bhi360_3d_mapping_t *data);
void jis_bhi360_data2_notify(const bhi360_step_count_t *data);
void jis_bhi360_data3_notify(const bhi360_linear_accel_t *data);
void jis_bhi360_quaternion_notify(const float *quat);
void jis_bhi360_linear_accel_notify(const float *lacc);
void jis_bhi360_step_count_notify(uint32_t step_count);

// FOTA progress notify function
void jis_fota_progress_notify(const fota_progress_msg_t *progress);

// Step count notify functions
extern "C" void jis_total_step_count_notify(uint32_t total_steps, uint32_t activity_duration);
extern "C" void jis_activity_step_count_notify(uint32_t activity_steps);

// Weight measurement notify function
extern "C" void jis_weight_measurement_notify(float weight_kg);

// BLE format notification functions (with sequence numbers)
void jis_foot_sensor_notify_ble(const foot_samples_ble_t *data);
void jis_bhi360_data1_notify_ble(const bhi360_3d_mapping_ble_t *data);
void jis_bhi360_data3_notify_ble(const bhi360_linear_accel_ble_t *data);

// Secondary data update functions (primary only)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)

#endif

// Secondary device info functions (primary only)
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
void jis_update_secondary_device_info(const char *manufacturer, const char *model, const char *serial,
                                      const char *hw_rev, const char *fw_rev);
void jis_clear_secondary_device_info(void);
void jis_secondary_weight_measurement_notify(float weight_kg);
err_t ble_reset_bonds(void);
#endif

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
extern "C" void cs_external_flash_erase_complete_notify(void);
void set_current_time_from_epoch(uint32_t new_epoch_time_s);
uint32_t get_current_epoch_time(void);
int init_rtc_time();
void update_cts_characteristic_buffer(void);
const void *get_current_time_char_value_ptr(void);
size_t get_current_time_char_value_size(void);

#ifdef __cplusplus
}
#endif

static constexpr uint8_t VND_MAX_LEN = 20;
static constexpr uint16_t BLE_ADVERTISING_TIMEOUT_MS = 30000;

static constexpr uint16_t bluetooth_timer = 1000;

// Helper function to send error status to Bluetooth module
static inline void send_error_to_bluetooth(sender_type_t sender, err_t error_code, bool is_set)
{
    generic_message_t msg;
    msg.sender = sender;
    msg.type = MSG_TYPE_ERROR_STATUS;
    msg.data.error_status.error_code = error_code;
    msg.data.error_status.is_set = is_set;

    // Send to queue, ignore if full (non-blocking)
    k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT);
}

#endif // APP_INCLUDE_BLE_SERVICES_HEADER_

#pragma once
#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <app.hpp>

#ifdef __cplusplus
extern "C" {
#endif

void ble_d2d_tx_init(void);
void ble_d2d_tx_set_connection(struct bt_conn *conn);

// Send functions for sensor data
int ble_d2d_tx_send_foot_sensor_data(const foot_samples_t *samples);
int ble_d2d_tx_send_foot_sensor_log_available(uint8_t log_id);
int ble_d2d_tx_send_foot_sensor_req_id_path(const char *path);
int ble_d2d_tx_send_bhi360_log_available(uint8_t log_id);
int ble_d2d_tx_send_bhi360_req_id_path(const char *path);
int ble_d2d_tx_send_bhi360_data1(const bhi360_3d_mapping_t *data);
int ble_d2d_tx_send_bhi360_data2(const bhi360_step_count_t *data);
int ble_d2d_tx_send_activity_step_count(const bhi360_step_count_t *data);
int ble_d2d_tx_send_bhi360_data3(const bhi360_linear_accel_t *data);
int ble_d2d_tx_send_status(uint32_t status);
int ble_d2d_tx_send_charge_status(uint8_t status);

// Send calculated metrics (secondary -> primary)
int ble_d2d_tx_send_metrics(const d2d_metrics_packet_t *metrics);

// Send functions for control commands (primary -> secondary)
int ble_d2d_tx_send_set_time_command(uint32_t epoch_time);
int ble_d2d_tx_send_delete_foot_log_command(uint8_t log_id);
int ble_d2d_tx_send_delete_bhi360_log_command(uint8_t log_id);
int ble_d2d_tx_send_delete_activity_log_command(uint8_t log_id);
int ble_d2d_tx_send_start_activity_command(uint8_t value);
int ble_d2d_tx_send_stop_activity_command(uint8_t value);
int ble_d2d_tx_send_trigger_bhi360_calibration_command(uint8_t value);
int ble_d2d_tx_send_erase_flash_command(uint8_t value);

// Send function for FOTA completion status (secondary -> primary)
int ble_d2d_tx_send_fota_complete(void);

// Send function for FOTA progress updates (secondary -> primary)
int ble_d2d_tx_send_fota_progress(const fota_progress_msg_t *progress);

// Send function for device info (secondary -> primary)
int ble_d2d_tx_send_device_info(const device_info_msg_t *info);

// Request device info from secondary (primary only)
int ble_d2d_tx_request_device_info(void);

// Send weight measurement (secondary -> primary)
int ble_d2d_tx_send_weight_measurement(float weight_kg);

// Send weight calibration command (primary -> secondary)
int ble_d2d_tx_send_weight_calibration_command(const weight_calibration_data_t *cal_data);

// Send weight calibration trigger command (primary -> secondary)
int ble_d2d_tx_send_weight_calibration_trigger_command(uint8_t value);

// Send weight calibration with known weight (primary -> secondary)
int ble_d2d_tx_send_weight_calibration_with_weight(const weight_calibration_step_t *calib_data);

// Send GPS update (primary -> secondary)
int ble_d2d_tx_send_gps_update(const GPSUpdateCommand *gps_data);

// Send connection parameter control command (primary -> secondary)
int ble_d2d_tx_send_conn_param_control_command(uint8_t profile);

#ifdef __cplusplus
}
#endif


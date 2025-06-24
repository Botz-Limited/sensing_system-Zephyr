#pragma once

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <app.hpp>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the D2D TX GATT service
 */
void d2d_tx_service_init(void);

/**
 * @brief Set the connection handle for the primary device
 * @param conn Connection handle (NULL to clear)
 */
void d2d_tx_service_set_connection(struct bt_conn *conn);

/**
 * @brief Send foot sensor data notification to primary device
 * @param samples Foot sensor sample data
 * @return 0 on success, negative error code on failure
 */
int d2d_tx_notify_foot_sensor_data(const foot_samples_t *samples);

/**
 * @brief Send BHI360 3D mapping data notification to primary device
 * @param data BHI360 3D mapping data
 * @return 0 on success, negative error code on failure
 */
int d2d_tx_notify_bhi360_data1(const bhi360_3d_mapping_t *data);

/**
 * @brief Send BHI360 step count notification to primary device
 * @param data BHI360 step count data
 * @return 0 on success, negative error code on failure
 */
int d2d_tx_notify_bhi360_data2(const bhi360_step_count_t *data);

/**
 * @brief Send BHI360 linear acceleration notification to primary device
 * @param data BHI360 linear acceleration data
 * @return 0 on success, negative error code on failure
 */
int d2d_tx_notify_bhi360_data3(const bhi360_linear_accel_t *data);

/**
 * @brief Send device status notification to primary device
 * @param status Device status bitfield
 * @return 0 on success, negative error code on failure
 */
int d2d_tx_notify_status(uint32_t status);

/**
 * @brief Send charge status notification to primary device
 * @param status Charge status value
 * @return 0 on success, negative error code on failure
 */
int d2d_tx_notify_charge_status(uint8_t status);

/**
 * @brief Send foot sensor log available notification to primary device
 * @param log_id Log ID that is available
 * @return 0 on success, negative error code on failure
 */
int d2d_tx_notify_foot_log_available(uint8_t log_id);

/**
 * @brief Send BHI360 log available notification to primary device
 * @param log_id Log ID that is available
 * @return 0 on success, negative error code on failure
 */
int d2d_tx_notify_bhi360_log_available(uint8_t log_id);

/**
 * @brief Send device info notification to primary device
 * @param info Device information structure
 * @return 0 on success, negative error code on failure
 */
int d2d_tx_notify_device_info(const device_info_msg_t *info);

#ifdef __cplusplus
}
#endif
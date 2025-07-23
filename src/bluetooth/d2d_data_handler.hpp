/**
 * @file d2d_data_handler.hpp
 * @brief Handler for D2D data received by primary device
 * 
 * This module processes sensor data and file paths received from
 * the secondary device and forwards them appropriately.
 */

#ifndef D2D_DATA_HANDLER_HPP
#define D2D_DATA_HANDLER_HPP

#include <stdint.h>
#include <app_fixed_point.hpp>
#include "ble_d2d_tx_service.hpp" // for d2d_sample_batch_t

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Handle D2D batch data from secondary device
 * @param batch Pointer to batch data
 * @return 0 on success, negative on error
 */
int d2d_data_handler_process_batch(const d2d_sample_batch_t *batch);

/**
 * @brief Initialize the D2D data handler
 * @return 0 on success, negative error code on failure
 */
int d2d_data_handler_init(void);

/**
 * @brief Handle foot sensor data from secondary device
 * 
 * This function should:
 * 1. Store/aggregate the data if needed
 * 2. Forward to phone via Information Service
 * 3. Log to file system if logging is active
 * 
 * @param samples Foot sensor samples
 * @return 0 on success, negative error code on failure
 */
int d2d_data_handler_process_foot_samples(const foot_samples_t *samples);

/**
 * @brief Handle BHI360 3D mapping data from secondary device
 * 
 * @param data BHI360 3D mapping data
 * @return 0 on success, negative error code on failure
 */
int d2d_data_handler_process_bhi360_3d_mapping(const bhi360_3d_mapping_t *data);

/**
 * @brief Handle BHI360 step count from secondary device
 * 
 * @param data Step count data
 * @return 0 on success, negative error code on failure
 */
int d2d_data_handler_process_bhi360_step_count(const bhi360_step_count_t *data);

/**
 * @brief Handle BHI360 linear acceleration from secondary device
 * 
 * @param data Linear acceleration data
 * @return 0 on success, negative error code on failure
 */
int d2d_data_handler_process_bhi360_linear_accel(const bhi360_linear_accel_t *data);

/**
 * @brief Handle file path from secondary device
 * 
 * This should update the Information Service with the path
 * so the phone can retrieve it.
 * 
 * @param log_id Log file ID
 * @param file_type File type (foot sensor or BHI360)
 * @param path File path string
 * @return 0 on success, negative error code on failure
 */
int d2d_data_handler_process_file_path(uint8_t log_id, uint8_t file_type, const char *path);

/**
 * @brief Handle log available notification from secondary device
 * 
 * This should update the Information Service to notify the phone
 * that a new log is available on the secondary device.
 * 
 * @param log_id Log file ID
 * @param file_type File type (foot sensor or BHI360)
 * @return 0 on success, negative error code on failure
 */
int d2d_data_handler_process_log_available(uint8_t log_id, uint8_t file_type);

/**
 * @brief Handle status update from secondary device
 * 
 * This should merge the secondary status with primary status
 * and update the Information Service.
 * 
 * @param status Secondary device status bits
 * @return 0 on success, negative error code on failure
 */
int d2d_data_handler_process_status(uint32_t status);

/**
 * @brief Handle charge status from secondary device
 * 
 * @param charge_status Charge status value
 * @return 0 on success, negative error code on failure
 */
int d2d_data_handler_process_charge_status(uint8_t charge_status);

/**
 * @brief Handle activity step count from secondary device
 * 
 * @param data Activity step count data
 * @return 0 on success, negative error code on failure
 */
int d2d_data_handler_process_activity_step_count(const bhi360_step_count_t *data);

#ifdef __cplusplus
}
#endif

#endif /* D2D_DATA_HANDLER_HPP */
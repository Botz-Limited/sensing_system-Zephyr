/**
 * @file bhi360_calibration.h
 * @brief BHI360 sensor calibration API
 *
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BHI360_CALIBRATION_H_
#define ZEPHYR_DRIVERS_SENSOR_BHI360_CALIBRATION_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sensor types for calibration
 */
enum bhi360_sensor_type {
    BHI360_SENSOR_ACCEL = 0,
    BHI360_SENSOR_GYRO,
    BHI360_SENSOR_MAG,
};

/**
 * @brief Fast Offset Compensation (FOC) result structure
 */
struct bhi360_foc_result {
    bool success;           /**< FOC success status */
    int16_t x_offset;      /**< X-axis offset */
    int16_t y_offset;      /**< Y-axis offset */
    int16_t z_offset;      /**< Z-axis offset */
};

/**
 * @brief Calibration status structure
 * 
 * Calibration status values:
 * 0 = Not calibrated
 * 1 = Calibration in progress
 * 2 = Calibrated with low accuracy
 * 3 = Calibrated with medium accuracy
 * 4 = Calibrated with high accuracy
 */
struct bhi360_calibration_status {
    uint8_t accel_calib_status;  /**< Accelerometer calibration status */
    uint8_t gyro_calib_status;   /**< Gyroscope calibration status */
    uint8_t mag_calib_status;    /**< Magnetometer calibration status */
};

/**
 * @brief Perform gyroscope Fast Offset Compensation (FOC)
 *
 * This function performs automatic calibration of the gyroscope to determine
 * and compensate for sensor offsets. The device must be kept stationary
 * during calibration.
 *
 * @param dev Pointer to the BHI360 device
 * @param result Pointer to store FOC results
 * @return 0 on success, negative error code on failure
 */
int bhi360_perform_gyro_foc(const struct device *dev, 
                           struct bhi360_foc_result *result);

/**
 * @brief Perform accelerometer Fast Offset Compensation (FOC)
 *
 * This function performs automatic calibration of the accelerometer to determine
 * and compensate for sensor offsets. The device should be placed on a flat
 * surface with Z-axis pointing up (1g position).
 *
 * @param dev Pointer to the BHI360 device
 * @param result Pointer to store FOC results
 * @return 0 on success, negative error code on failure
 */
int bhi360_perform_accel_foc(const struct device *dev, 
                            struct bhi360_foc_result *result);

/**
 * @brief Get calibration profile for a sensor
 *
 * Retrieves the current calibration profile data for the specified sensor.
 * This data can be saved and later restored to avoid recalibration.
 *
 * @param dev Pointer to the BHI360 device
 * @param sensor Sensor type to get calibration for
 * @param profile_data Buffer to store calibration profile
 * @param profile_size Size of the profile buffer
 * @param actual_size Actual size of the calibration profile
 * @return 0 on success, negative error code on failure
 */
int bhi360_get_calibration_profile(const struct device *dev, 
                                  enum bhi360_sensor_type sensor,
                                  uint8_t *profile_data,
                                  size_t profile_size,
                                  size_t *actual_size);

/**
 * @brief Set calibration profile for a sensor
 *
 * Loads a previously saved calibration profile to restore sensor calibration
 * without performing the calibration procedure again.
 *
 * @param dev Pointer to the BHI360 device
 * @param sensor Sensor type to set calibration for
 * @param profile_data Calibration profile data to load
 * @param profile_size Size of the calibration profile
 * @return 0 on success, negative error code on failure
 */
int bhi360_set_calibration_profile(const struct device *dev,
                                  enum bhi360_sensor_type sensor,
                                  const uint8_t *profile_data,
                                  size_t profile_size);

/**
 * @brief Get current calibration status
 *
 * Retrieves the calibration status for all sensors. Status values indicate
 * the quality of calibration from 0 (not calibrated) to 4 (high accuracy).
 *
 * @param dev Pointer to the BHI360 device
 * @param status Pointer to store calibration status
 * @return 0 on success, negative error code on failure
 */
int bhi360_get_calibration_status(const struct device *dev,
                                 struct bhi360_calibration_status *status);

/**
 * @brief Perform full calibration sequence
 *
 * Performs calibration for all sensors in the recommended sequence:
 * 1. Gyroscope calibration (device stationary)
 * 2. Accelerometer calibration (device on flat surface)
 * 
 * Note: Magnetometer calibration is performed automatically during
 * normal operation when the device is moved in various orientations.
 *
 * @param dev Pointer to the BHI360 device
 * @return 0 on success, negative error code on failure
 */
int bhi360_perform_full_calibration(const struct device *dev);

/**
 * @brief Save calibration data to flash
 *
 * Saves the current calibration profile to non-volatile storage.
 * This allows calibration to persist across power cycles.
 *
 * @param dev Pointer to the BHI360 device
 * @param sensor Sensor type to save calibration for
 * @return 0 on success, negative error code on failure
 * 
 * @note This function requires flash storage implementation
 */
int bhi360_save_calibration_to_flash(const struct device *dev,
                                    enum bhi360_sensor_type sensor);

/**
 * @brief Load calibration data from flash
 *
 * Loads previously saved calibration profile from non-volatile storage
 * and applies it to the sensor.
 *
 * @param dev Pointer to the BHI360 device
 * @param sensor Sensor type to load calibration for
 * @return 0 on success, negative error code on failure
 * 
 * @note This function requires flash storage implementation
 */
int bhi360_load_calibration_from_flash(const struct device *dev,
                                      enum bhi360_sensor_type sensor);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_BHI360_CALIBRATION_H_ */
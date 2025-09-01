/**
 * @file bhi360.h
 * @brief BHI360 sensor driver API
 *
 * Copyright (c) 2024 Your Company
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BHI360_H_
#define ZEPHYR_DRIVERS_SENSOR_BHI360_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "BHY2-Sensor-API/bhy2.h"
#include "bhi360_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Custom sensor channel for step counter */
#define BHI360_SENSOR_CHAN_STEPS    SENSOR_CHAN_PRIV_START

/**
 * @brief BHI360 sensor data structure
 *
 * This structure holds the latest sensor readings from the BHI360.
 * It is used to update the driver's internal state and provide data
 * through the standard Zephyr sensor API.
 */
struct bhi360_sensor_data {
    /* Quaternion data */
    float quat_x;
    float quat_y;
    float quat_z;
    float quat_w;
    uint8_t quat_accuracy;
    
    /* Linear acceleration data (m/s²) */
    float lacc_x;
    float lacc_y;
    float lacc_z;
    
    /* Gyroscope data (rad/s) */
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    /* Step counter */
    uint32_t step_count;
    
    /* Timestamp */
    uint64_t timestamp;
};

/**
 * @brief Get the BHY2 device handle from the BHI360 driver
 *
 * This function allows direct access to the BHY2 device structure
 * for advanced operations not covered by the standard sensor API.
 *
 * @param dev Pointer to the BHI360 device
 * @return Pointer to the BHY2 device structure, or NULL if not initialized
 */
struct bhy2_dev *bhi360_get_bhy2_dev(const struct device *dev);

/**
 * @brief Wait for data ready interrupt
 *
 * This function blocks until the BHI360 signals data ready via interrupt.
 *
 * @param dev Pointer to the BHI360 device
 * @param timeout Maximum time to wait (K_FOREVER to wait indefinitely)
 * @return 0 on success, -EAGAIN on timeout, or negative error code
 */
int bhi360_wait_for_data(const struct device *dev, k_timeout_t timeout);

/**
 * @brief Process FIFO data
 *
 * This function reads and processes data from the BHI360 FIFO.
 * It should be called after receiving a data ready interrupt.
 *
 * @param dev Pointer to the BHI360 device
 * @param work_buffer Buffer for FIFO processing (recommended size: 2048 bytes)
 * @param buffer_size Size of the work buffer
 * @return 0 on success, or negative error code
 */
int bhi360_process_fifo(const struct device *dev, uint8_t *work_buffer, size_t buffer_size);

/**
 * @brief Update sensor data in the driver
 *
 * This function updates the driver's internal sensor data cache.
 * It should be called by the application after parsing FIFO data
 * to make the latest values available through the standard sensor API.
 *
 * @param dev Pointer to the BHI360 device
 * @param new_data Pointer to the new sensor data
 * @return 0 on success, or negative error code
 */
int bhi360_update_sensor_data(const struct device *dev, 
                             const struct bhi360_sensor_data *new_data);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_BHI360_H_ */
/**
 * @file bhi360_calibration.c
 * @brief BHI360 sensor calibration functions
 *
 * This file provides calibration functions for the BHI360 sensor including:
 * - Gyroscope Fast Offset Compensation (FOC)
 * - Accelerometer Fast Offset Compensation (FOC)
 * - Magnetometer calibration
 * - Calibration profile save/restore
 *
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <string.h>

#include "bhi360.h"
#include "bhi360_calibration.h"
#include "BHY2-Sensor-API/bhy2.h"
#include "BHY2-Sensor-API/bhy2_defs.h"

LOG_MODULE_DECLARE(bhi360, CONFIG_SENSOR_LOG_LEVEL);

/* Calibration profile sizes for different sensors */
#define BHI360_ACCEL_CALIB_PROFILE_SIZE    512
#define BHI360_GYRO_CALIB_PROFILE_SIZE     512
#define BHI360_MAG_CALIB_PROFILE_SIZE      1024

/* Calibration timeout */
#define BHI360_CALIB_TIMEOUT_MS            10000

/* Physical sensor IDs */
#define BHI360_PHYS_SENSOR_ID_ACC          0x01
#define BHI360_PHYS_SENSOR_ID_GYRO         0x02
#define BHI360_PHYS_SENSOR_ID_MAG          0x03

int bhi360_perform_gyro_foc(const struct device *dev, struct bhi360_foc_result *result)
{
    struct bhy2_dev *bhy2_dev;
    struct bhy2_foc_resp foc_resp;
    int8_t rslt;
    
    if (!dev || !result) {
        return -EINVAL;
    }
    
    bhy2_dev = bhi360_get_bhy2_dev(dev);
    if (!bhy2_dev) {
        LOG_ERR("Failed to get BHY2 device");
        return -ENODEV;
    }
    
    LOG_INF("Starting gyroscope FOC calibration...");
    LOG_INF("Please keep the device stationary during calibration");
    
    /* Perform FOC for gyroscope */
    rslt = bhy2_perform_foc(BHI360_PHYS_SENSOR_ID_GYRO, &foc_resp, bhy2_dev);
    if (rslt != BHY2_OK) {
        LOG_ERR("Gyro FOC failed with error: %d", rslt);
        return -EIO;
    }
    
    /* Check FOC status */
    if (foc_resp.foc_status == BHY2_FOC_PASS) {
        LOG_INF("Gyro FOC successful!");
        result->success = true;
        result->x_offset = foc_resp.x_offset;
        result->y_offset = foc_resp.y_offset;
        result->z_offset = foc_resp.z_offset;
        LOG_INF("Gyro offsets - X: %d, Y: %d, Z: %d", 
                foc_resp.x_offset, foc_resp.y_offset, foc_resp.z_offset);
    } else {
        LOG_ERR("Gyro FOC failed with status: 0x%02X", foc_resp.foc_status);
        result->success = false;
        return -ECANCELED;
    }
    
    return 0;
}

int bhi360_perform_accel_foc(const struct device *dev, struct bhi360_foc_result *result)
{
    struct bhy2_dev *bhy2_dev;
    struct bhy2_foc_resp foc_resp;
    int8_t rslt;
    
    if (!dev || !result) {
        return -EINVAL;
    }
    
    bhy2_dev = bhi360_get_bhy2_dev(dev);
    if (!bhy2_dev) {
        LOG_ERR("Failed to get BHY2 device");
        return -ENODEV;
    }
    
    LOG_INF("Starting accelerometer FOC calibration...");
    LOG_INF("Please place the device on a flat surface with Z-axis pointing up");
    
    /* Give user time to position device */
    k_msleep(3000);
    
    /* Perform FOC for accelerometer */
    rslt = bhy2_perform_foc(BHI360_PHYS_SENSOR_ID_ACC, &foc_resp, bhy2_dev);
    if (rslt != BHY2_OK) {
        LOG_ERR("Accel FOC failed with error: %d", rslt);
        return -EIO;
    }
    
    /* Check FOC status */
    if (foc_resp.foc_status == BHY2_FOC_PASS) {
        LOG_INF("Accel FOC successful!");
        result->success = true;
        result->x_offset = foc_resp.x_offset;
        result->y_offset = foc_resp.y_offset;
        result->z_offset = foc_resp.z_offset;
        LOG_INF("Accel offsets - X: %d, Y: %d, Z: %d", 
                foc_resp.x_offset, foc_resp.y_offset, foc_resp.z_offset);
    } else {
        LOG_ERR("Accel FOC failed with status: 0x%02X", foc_resp.foc_status);
        result->success = false;
        return -ECANCELED;
    }
    
    return 0;
}

int bhi360_get_calibration_profile(const struct device *dev, 
                                  enum bhi360_sensor_type sensor,
                                  uint8_t *profile_data,
                                  size_t profile_size,
                                  size_t *actual_size)
{
    struct bhy2_dev *bhy2_dev;
    uint8_t phys_sensor_id;
    uint32_t actual_len;
    int8_t rslt;
    
    if (!dev || !profile_data || !actual_size) {
        return -EINVAL;
    }
    
    bhy2_dev = bhi360_get_bhy2_dev(dev);
    if (!bhy2_dev) {
        LOG_ERR("Failed to get BHY2 device");
        return -ENODEV;
    }
    
    /* Map sensor type to physical sensor ID */
    switch (sensor) {
    case BHI360_SENSOR_ACCEL:
        phys_sensor_id = BHI360_PHYS_SENSOR_ID_ACC;
        break;
    case BHI360_SENSOR_GYRO:
        phys_sensor_id = BHI360_PHYS_SENSOR_ID_GYRO;
        break;
    case BHI360_SENSOR_MAG:
        phys_sensor_id = BHI360_PHYS_SENSOR_ID_MAG;
        break;
    default:
        LOG_ERR("Invalid sensor type: %d", sensor);
        return -EINVAL;
    }
    
    /* Get calibration profile */
    rslt = bhy2_get_calibration_profile(phys_sensor_id, profile_data, 
                                       profile_size, &actual_len, bhy2_dev);
    if (rslt != BHY2_OK) {
        LOG_ERR("Failed to get calibration profile: %d", rslt);
        return -EIO;
    }
    
    *actual_size = actual_len;
    LOG_INF("Retrieved calibration profile for sensor %d, size: %d bytes", 
            sensor, actual_len);
    
    return 0;
}

int bhi360_set_calibration_profile(const struct device *dev,
                                  enum bhi360_sensor_type sensor,
                                  const uint8_t *profile_data,
                                  size_t profile_size)
{
    struct bhy2_dev *bhy2_dev;
    uint8_t phys_sensor_id;
    int8_t rslt;
    
    if (!dev || !profile_data || profile_size == 0) {
        return -EINVAL;
    }
    
    bhy2_dev = bhi360_get_bhy2_dev(dev);
    if (!bhy2_dev) {
        LOG_ERR("Failed to get BHY2 device");
        return -ENODEV;
    }
    
    /* Map sensor type to physical sensor ID */
    switch (sensor) {
    case BHI360_SENSOR_ACCEL:
        phys_sensor_id = BHI360_PHYS_SENSOR_ID_ACC;
        break;
    case BHI360_SENSOR_GYRO:
        phys_sensor_id = BHI360_PHYS_SENSOR_ID_GYRO;
        break;
    case BHI360_SENSOR_MAG:
        phys_sensor_id = BHI360_PHYS_SENSOR_ID_MAG;
        break;
    default:
        LOG_ERR("Invalid sensor type: %d", sensor);
        return -EINVAL;
    }
    
    /* Set calibration profile */
    rslt = bhy2_set_calibration_profile(phys_sensor_id, profile_data, 
                                       profile_size, bhy2_dev);
    if (rslt != BHY2_OK) {
        LOG_ERR("Failed to set calibration profile: %d", rslt);
        return -EIO;
    }
    
    LOG_INF("Set calibration profile for sensor %d, size: %d bytes", 
            sensor, profile_size);
    
    return 0;
}

int bhi360_get_calibration_status(const struct device *dev,
                                 struct bhi360_calibration_status *status)
{
    struct bhy2_dev *bhy2_dev;
    uint8_t param_data[16];
    uint32_t actual_len;
    int8_t rslt;
    
    if (!dev || !status) {
        return -EINVAL;
    }
    
    bhy2_dev = bhi360_get_bhy2_dev(dev);
    if (!bhy2_dev) {
        LOG_ERR("Failed to get BHY2 device");
        return -ENODEV;
    }
    
    /* Get accelerometer calibration status */
    rslt = bhy2_get_parameter(BHY2_PARAM_BSX_CALIB_STATE_ACCEL, param_data, 
                             sizeof(param_data), &actual_len, bhy2_dev);
    if (rslt == BHY2_OK && actual_len >= 1) {
        status->accel_calib_status = param_data[0];
    } else {
        status->accel_calib_status = 0;
    }
    
    /* Get gyroscope calibration status */
    rslt = bhy2_get_parameter(BHY2_PARAM_BSX_CALIB_STATE_GYRO, param_data, 
                             sizeof(param_data), &actual_len, bhy2_dev);
    if (rslt == BHY2_OK && actual_len >= 1) {
        status->gyro_calib_status = param_data[0];
    } else {
        status->gyro_calib_status = 0;
    }
    
    /* Get magnetometer calibration status */
    rslt = bhy2_get_parameter(BHY2_PARAM_BSX_CALIB_STATE_MAG, param_data, 
                             sizeof(param_data), &actual_len, bhy2_dev);
    if (rslt == BHY2_OK && actual_len >= 1) {
        status->mag_calib_status = param_data[0];
    } else {
        status->mag_calib_status = 0;
    }
    
    LOG_DBG("Calibration status - Accel: %d, Gyro: %d, Mag: %d",
            status->accel_calib_status, status->gyro_calib_status, 
            status->mag_calib_status);
    
    return 0;
}

int bhi360_perform_full_calibration(const struct device *dev)
{
    struct bhi360_foc_result foc_result;
    int ret;
    
    if (!dev) {
        return -EINVAL;
    }
    
    LOG_INF("Starting full sensor calibration sequence...");
    
    /* Step 1: Gyroscope calibration */
    LOG_INF("Step 1/2: Gyroscope calibration");
    ret = bhi360_perform_gyro_foc(dev, &foc_result);
    if (ret != 0) {
        LOG_ERR("Gyroscope calibration failed: %d", ret);
        return ret;
    }
    
    /* Step 2: Accelerometer calibration */
    LOG_INF("Step 2/2: Accelerometer calibration");
    ret = bhi360_perform_accel_foc(dev, &foc_result);
    if (ret != 0) {
        LOG_ERR("Accelerometer calibration failed: %d", ret);
        return ret;
    }
    
    LOG_INF("Full calibration sequence completed successfully!");
    
    /* Note: Magnetometer calibration typically requires movement in 
     * figure-8 pattern and is handled automatically by the BHI360 
     * during normal operation */
    
    return 0;
}

int bhi360_save_calibration_to_flash(const struct device *dev,
                                    enum bhi360_sensor_type sensor)
{
    uint8_t profile_data[BHI360_MAG_CALIB_PROFILE_SIZE];
    size_t actual_size;
    int ret;
    
    /* Get current calibration profile */
    ret = bhi360_get_calibration_profile(dev, sensor, profile_data, 
                                        sizeof(profile_data), &actual_size);
    if (ret != 0) {
        LOG_ERR("Failed to get calibration profile: %d", ret);
        return ret;
    }
    
    /* TODO: Implement flash storage for calibration data */
    /* This would typically use Zephyr's flash API or settings subsystem */
    /* to persistently store the calibration profile */
    
    LOG_WRN("Flash storage not implemented - calibration data not persisted");
    
    return -ENOSYS;
}

int bhi360_load_calibration_from_flash(const struct device *dev,
                                      enum bhi360_sensor_type sensor)
{
    /* TODO: Implement flash storage retrieval for calibration data */
    /* This would typically use Zephyr's flash API or settings subsystem */
    /* to load previously stored calibration profiles */
    
    LOG_WRN("Flash storage not implemented - cannot load calibration data");
    
    return -ENOSYS;
}
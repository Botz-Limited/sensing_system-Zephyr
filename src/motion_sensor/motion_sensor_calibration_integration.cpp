/**
 * @file motion_sensor_calibration_integration.cpp
 * @brief Example integration of BHI360 calibration storage
 * 
 * This file shows how to integrate calibration save/load functionality
 * into the motion sensor module.
 */

#include <data.hpp>
#include <bhi360.h>
#include <bhi360_calibration.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(motion_sensor);

/**
 * @brief Load saved calibrations from storage
 * 
 * This function should be called during initialization, after firmware upload
 * but before or after the calibration check.
 */
static void load_saved_calibrations(const struct device *bhi360_dev)
{
    struct {
        enum bhi360_sensor_type sensor;
        uint8_t type_id;
        const char *name;
    } sensors[] = {
        {BHI360_SENSOR_ACCEL, 0, "accel"},
        {BHI360_SENSOR_GYRO, 1, "gyro"},
        {BHI360_SENSOR_MAG, 2, "mag"}
    };

    LOG_INF("Attempting to load saved calibrations...");

    for (int i = 0; i < ARRAY_SIZE(sensors); i++) {
        uint8_t profile_data[512];
        size_t actual_size;
        
        err_t err = get_bhi360_calibration_data(sensors[i].type_id, 
                                               profile_data, 
                                               sizeof(profile_data), 
                                               &actual_size);
        if (err == err_t::NO_ERROR) {
            // Successfully loaded calibration from storage
            int ret = bhi360_set_calibration_profile(bhi360_dev, 
                                                    sensors[i].sensor,
                                                    profile_data, 
                                                    actual_size);
            if (ret == 0) {
                LOG_INF("Loaded %s calibration from storage (%u bytes)", 
                        sensors[i].name, actual_size);
            } else {
                LOG_WRN("Failed to apply %s calibration: %d", 
                        sensors[i].name, ret);
            }
        } else if (err == err_t::FILE_SYSTEM_NO_FILES) {
            LOG_INF("No saved %s calibration found (first time)", 
                    sensors[i].name);
        } else {
            LOG_WRN("Error loading %s calibration: %d", 
                    sensors[i].name, (int)err);
        }
    }
}

/**
 * @brief Save calibration profile after successful calibration
 */
static void save_calibration_profile(const struct device *bhi360_dev, 
                                   enum bhi360_sensor_type sensor,
                                   const char *sensor_name)
{
    uint8_t profile_data[512];
    size_t actual_size;
    
    // Map sensor type to storage ID
    uint8_t type_id;
    switch (sensor) {
        case BHI360_SENSOR_ACCEL: type_id = 0; break;
        case BHI360_SENSOR_GYRO: type_id = 1; break;
        case BHI360_SENSOR_MAG: type_id = 2; break;
        default: return;
    }
    
    // Get current calibration profile from BHI360
    int ret = bhi360_get_calibration_profile(bhi360_dev, sensor, 
                                            profile_data, 
                                            sizeof(profile_data), 
                                            &actual_size);
    if (ret == 0) {
        // Save to storage
        err_t err = store_bhi360_calibration_data(type_id, 
                                                 profile_data, 
                                                 actual_size);
        if (err == err_t::NO_ERROR) {
            LOG_INF("Saved %s calibration (%u bytes)", 
                    sensor_name, actual_size);
        } else {
            LOG_ERR("Failed to save %s calibration: %d", 
                    sensor_name, (int)err);
        }
    } else {
        LOG_ERR("Failed to get %s calibration profile: %d", 
                sensor_name, ret);
    }
}

/**
 * @brief Enhanced calibration initialization with storage support
 * 
 * This function replaces the calibration section in motion_sensor_init()
 */
void motion_sensor_calibration_with_storage(const struct device *bhi360_dev)
{
#if IS_ENABLED(CONFIG_BHI360_AUTO_CALIBRATION)
    LOG_INF("BHI360: Calibration with storage support");
    
    // First, try to load saved calibrations
    load_saved_calibrations(bhi360_dev);
    
    // Then check current calibration status
    struct bhi360_calibration_status calib_status;
    int ret = bhi360_get_calibration_status(bhi360_dev, &calib_status);
    if (ret == 0) {
        LOG_INF("BHI360: Calibration status after loading - Accel: %d, Gyro: %d, Mag: %d",
                calib_status.accel_calib_status,
                calib_status.gyro_calib_status,
                calib_status.mag_calib_status);
        
        // Perform gyroscope calibration if still needed
        if (calib_status.gyro_calib_status < CONFIG_BHI360_CALIBRATION_THRESHOLD) {
            LOG_INF("BHI360: Gyroscope needs calibration, performing FOC...");
            LOG_INF("BHI360: Please keep the device stationary");
            
            k_sleep(K_SECONDS(2));
            
            struct bhi360_foc_result foc_result;
            ret = bhi360_perform_gyro_foc(bhi360_dev, &foc_result);
            if (ret == 0 && foc_result.success) {
                LOG_INF("BHI360: Gyro calibration successful! Offsets: X=%d, Y=%d, Z=%d",
                        foc_result.x_offset, foc_result.y_offset, foc_result.z_offset);
                
                // Save the new calibration
                save_calibration_profile(bhi360_dev, BHI360_SENSOR_GYRO, "gyro");
            } else {
                LOG_WRN("BHI360: Gyro calibration failed");
            }
        } else {
            LOG_INF("BHI360: Gyroscope already calibrated (from storage or previous)");
        }
        
        // Perform accelerometer calibration if still needed
        if (calib_status.accel_calib_status < CONFIG_BHI360_CALIBRATION_THRESHOLD) {
            LOG_INF("BHI360: Accelerometer needs calibration, performing FOC...");
            LOG_INF("BHI360: Please place device on flat surface with Z-axis up");
            
            k_sleep(K_SECONDS(3));
            
            struct bhi360_foc_result foc_result;
            ret = bhi360_perform_accel_foc(bhi360_dev, &foc_result);
            if (ret == 0 && foc_result.success) {
                LOG_INF("BHI360: Accel calibration successful! Offsets: X=%d, Y=%d, Z=%d",
                        foc_result.x_offset, foc_result.y_offset, foc_result.z_offset);
                
                // Save the new calibration
                save_calibration_profile(bhi360_dev, BHI360_SENSOR_ACCEL, "accel");
            } else {
                LOG_WRN("BHI360: Accel calibration failed");
            }
        } else {
            LOG_INF("BHI360: Accelerometer already calibrated (from storage or previous)");
        }
        
        // Magnetometer calibration note
        if (calib_status.mag_calib_status < CONFIG_BHI360_CALIBRATION_THRESHOLD) {
            LOG_INF("BHI360: Magnetometer will calibrate automatically during use");
            LOG_INF("BHI360: Move device in figure-8 pattern to speed up mag calibration");
            
            // Set up a periodic check to save mag calibration when it improves
            // This would be done in the main loop or with a timer
        } else {
            LOG_INF("BHI360: Magnetometer already calibrated (from storage or previous)");
        }
        
        // Final status check
        ret = bhi360_get_calibration_status(bhi360_dev, &calib_status);
        if (ret == 0) {
            LOG_INF("BHI360: Final calibration status - Accel: %d, Gyro: %d, Mag: %d",
                    calib_status.accel_calib_status,
                    calib_status.gyro_calib_status,
                    calib_status.mag_calib_status);
        }
    } else {
        LOG_WRN("BHI360: Could not check calibration status: %d", ret);
    }
#else
    LOG_INF("BHI360: Automatic calibration disabled");
    // Still try to load saved calibrations even if auto-calibration is disabled
    load_saved_calibrations(bhi360_dev);
#endif /* CONFIG_BHI360_AUTO_CALIBRATION */
}

/**
 * @brief Periodic calibration check and save
 * 
 * This function can be called periodically to check if calibration
 * has improved and save it. Particularly useful for magnetometer.
 */
void check_and_save_calibration_updates(const struct device *bhi360_dev)
{
    static struct bhi360_calibration_status last_status = {0, 0, 0};
    struct bhi360_calibration_status current_status;
    
    int ret = bhi360_get_calibration_status(bhi360_dev, &current_status);
    if (ret != 0) {
        return;
    }
    
    // Check if any calibration status has improved
    if (current_status.accel_calib_status > last_status.accel_calib_status) {
        LOG_INF("Accel calibration improved: %d -> %d", 
                last_status.accel_calib_status, 
                current_status.accel_calib_status);
        save_calibration_profile(bhi360_dev, BHI360_SENSOR_ACCEL, "accel");
    }
    
    if (current_status.gyro_calib_status > last_status.gyro_calib_status) {
        LOG_INF("Gyro calibration improved: %d -> %d", 
                last_status.gyro_calib_status, 
                current_status.gyro_calib_status);
        save_calibration_profile(bhi360_dev, BHI360_SENSOR_GYRO, "gyro");
    }
    
    if (current_status.mag_calib_status > last_status.mag_calib_status) {
        LOG_INF("Mag calibration improved: %d -> %d", 
                last_status.mag_calib_status, 
                current_status.mag_calib_status);
        save_calibration_profile(bhi360_dev, BHI360_SENSOR_MAG, "mag");
    }
    
    last_status = current_status;
}
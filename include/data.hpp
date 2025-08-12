/**
 * @file data.hpp
 * @brief
 * @version 1.0
 * @date 15/5/2025
 *
 * @copyright Botz Innovation 2025
 *
 */

#ifndef APP_INCLUDE_DATA_HEADER_
#define APP_INCLUDE_DATA_HEADER_

#include <cstdint>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <errors.hpp>
#include <util.hpp>

#include <events/data_event.h>
#include <events/app_state_event.h>
#include <events/record_id_event.h>

constexpr uint32_t data_wait_timer = 2000;

constexpr uint32_t one_epoch_day_secs = 86400;
constexpr uint32_t one_epoch_week_secs = 604800;
static constexpr int readdir_micro_sleep_ms = 1;
static constexpr int empty_path_id = 0;

err_t delete_directory(const char *path);
uint8_t find_oldest_log_file(const char *file_prefix, record_type_t record_type, char *file_path);


// File prefixes for each sensor type
constexpr char foot_sensor_file_prefix[] = "foot_";
constexpr char bhi360_file_prefix[] = "bhi360_";
constexpr char activity_file_prefix[] = "activity_";


constexpr char hardware_dir_path[] = CONFIG_FILE_SYSTEM_MOUNT "/hardware";

extern char foot_sensor_file_path[util::max_path_length];
extern char bhi360_file_path[util::max_path_length];
extern char activity_file_path[util::max_path_length];

// BHI360 calibration API functions
err_t get_bhi360_calibration_data(uint8_t sensor_type, uint8_t *profile_data, 
                                  size_t max_size, size_t *actual_size);
err_t store_bhi360_calibration_data(uint8_t sensor_type, const uint8_t *profile_data, 
                                    size_t profile_size);

// Weight calibration API functions
err_t save_weight_calibration_data(const weight_calibration_data_t *calib_data);

typedef struct __attribute__((packed)) {
    uint16_t voltage_mV;    // Battery voltage in millivolts
    int8_t percentage;      // Estimated charge percentage (-1 if unknown)
    uint8_t status;         // Charging status (0=discharging, 1=charging, 2=full)
} battery_info_t;

typedef struct {
    uint8_t user_height_cm;
    uint8_t user_weight_kg;
    uint8_t user_age_years;
    uint8_t user_sex;
} user_config_t;

extern user_config_t user_config;

err_t load_weight_calibration_data(void);

#endif // APP_INCLUDE_DATA_HEADER_

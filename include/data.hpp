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

#define data_wait_timer 2000

constexpr uint32_t one_epoch_day_secs = 86400;
constexpr uint32_t one_epoch_week_secs = 604800;
static constexpr int readdir_micro_sleep_ms = 1;
static constexpr int empty_path_id = 0;

err_t delete_directory(const char *path);
uint8_t find_oldest_log_file(const char *file_prefix, record_type_t record_type, char *file_path);


// File prefixes for each sensor type
constexpr char foot_sensor_file_prefix[] = "foot_";
constexpr char bhi360_file_prefix[] = "bhi360_";


constexpr char hardware_dir_path[] = CONFIG_FILE_SYSTEM_MOUNT "/hardware";

char foot_sensor_file_path[util::max_path_length];
char bhi360_file_path[util::max_path_length];


#endif // APP_INCLUDE_DATA_HEADER_

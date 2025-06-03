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
#include <events/record_id_event.h>

#define data_wait_timer 2000

constexpr uint32_t one_epoch_day_secs = 86400;
constexpr uint32_t one_epoch_week_secs = 604800;
constexpr uint8_t max_retries = CONFIG_SENSOR_FAILURE_MAX_RETRIES;
constexpr uint8_t retry_sleep_timer_ms = CONFIG_RETRY_SLEEP_TIMER;
static constexpr int readdir_micro_sleep_ms = 1;
static constexpr int empty_path_id = 0;

err_t delete_directory(const char *path);
uint32_t find_oldest_log_file(const char *file_prefix, record_type_t record_type, char *file_path);

constexpr char metadata_file_prefix[] = "metadata_";
constexpr char metadata_dir_path[] = CONFIG_FILE_SYSTEM_MOUNT "/metadata";

constexpr char hardware_file_prefix[] = "hardware_";
constexpr char hardware_dir_path[] = CONFIG_FILE_SYSTEM_MOUNT "/hardware";

extern char metadata_file_path[util::max_path_length];
extern char hardware_file_path[util::max_path_length];

static inline void set_data_event(data_state_t new_evt)
{
    static data_state_t prev_evt = DATA_NO_ERROR;

    if(prev_evt != new_evt)
    {
        auto *event = new_data_event();
        event->data_state = new_evt;
        prev_evt = new_evt;
        APP_EVENT_SUBMIT(event);
    }
}

#endif // APP_INCLUDE_DATA_HEADER_

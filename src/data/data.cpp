/**
 * @file data.cpp
 * @brief Data logging and storage module with work queue architecture
 * @version 1.0.0
 * @date June 2025
 *
 * @copyright Botz Innovation 2025
 */

#define MODULE data

#include <algorithm>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>
#include <variant>
#include <vector>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/atomic.h>

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/bluetooth_state_event.h>
#include <events/data_event.h>
#include <util.hpp>

#include <activity_messages.pb.h>
#include <app.hpp>
#include <app_fixed_point.hpp>
#include <bhi360_sensor_messages.pb.h>
#include <ble_services.hpp>
#include <data.hpp>
#include <foot_sensor_messages.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <status_codes.h>

#include "../../include/events/foot_sensor_event.h"
#include "../../include/events/motion_sensor_event.h"

// External function to get the BLE service UUID from bluetooth module
extern "C" const uint8_t *get_target_service_uuid(void);

LOG_MODULE_REGISTER(MODULE, CONFIG_DATA_MODULE_LOG_LEVEL);

// Timer for periodic filesystem sync
static K_TIMER_DEFINE(fs_sync_timer, NULL, NULL);

// Flag to indicate a pending sync operation
static atomic_t sync_pending = ATOMIC_INIT(0);

static void fs_sync_work_handler(struct k_work *work);

// Work item for syncing the file system
static struct k_work fs_sync_work;

static uint32_t current_packet_number = 0U;
static uint32_t activity_start_epoch_time = 0U; // Epoch time when logging started
static uint32_t last_epoch_timestamp = 0U;      // Last epoch timestamp for delta calculation
static uint32_t start_epoch_timestamp = 0U;

// Constants from original implementation
constexpr char calibration_dir_path[] = "/lfs1/calibration";
constexpr char bhi360_calib_file_prefix[] = "bhi360_calib_";
constexpr size_t ACTIVITY_BATCH_SIZE = 2;

static uint32_t activity_packet_counter;

// Filesystem usage thresholds
constexpr uint8_t FILESYSTEM_USAGE_THRESHOLD_PERCENT = 80; // Start cleanup at 80% usage
constexpr uint8_t FILESYSTEM_USAGE_TARGET_PERCENT = 70;    // Clean up until 70% usage

// Counter for periodic filesystem space checks during logging
// static uint32_t filesystem_space_check_counter = 0;  // Currently unused
constexpr uint32_t FILESYSTEM_SPACE_CHECK_INTERVAL = 1000; // Check every 1000 writes

// File System Configuration
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(littlefs_storage);
#define STORAGE_PARTITION_LABEL storage_ext
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION_LABEL)
static struct fs_mount_t lfs_ext_storage_mnt = {};

// Thread Configuration
static constexpr int data_stack_size = CONFIG_DATA_MODULE_STACK_SIZE;
static constexpr int data_priority = CONFIG_DATA_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(data_stack_area, data_stack_size);
static struct k_thread data_thread_data;
static k_tid_t data_tid;

// Work Queue Configuration
static constexpr int data_workq_stack_size = 4096;
K_THREAD_STACK_DEFINE(data_workq_stack, data_workq_stack_size);
static struct k_work_q data_work_q;

// Work Items
static struct k_work process_sensor_data_work;
static struct k_work process_command_work;
static struct k_work process_calibration_work;
static struct k_work process_erase_flash_work;

// Message Buffers (protected by mutexes for thread safety)
static generic_message_t pending_sensor_data_msg;

// Command Queue - robust handling for multiple commands
#define MAX_QUEUED_COMMANDS 8
static generic_message_t command_queue;
static uint8_t cmd_queue_head = 0;
static uint8_t cmd_queue_tail = 0;
static atomic_t cmd_queue_count = ATOMIC_INIT(0);

static bhi360_calibration_data_t pending_calibration_data;

// Mutexes for work item message buffers
K_MUTEX_DEFINE(sensor_data_msg_mutex);
K_MUTEX_DEFINE(cmd_queue_mutex); // Renamed for command queue
K_MUTEX_DEFINE(calibration_msg_mutex);

// File System State
static struct fs_file_t activity_log_file;
static bool filesystem_available = false;
static atomic_t logging_activity_active = ATOMIC_INIT(0);

// Buffering Configuration
constexpr size_t FLASH_PAGE_SIZE = 256;
constexpr size_t PROTOBUF_ENCODE_BUFFER_SIZE = 64;
static uint8_t activity_write_buffer[FLASH_PAGE_SIZE];
static size_t activity_write_buffer_pos = 0;
static size_t activity_batch_count = 0;

// Timing and Sequence
static uint32_t activity_last_timestamp_ms = 0;
static bool activity_first_packet = true;
static uint8_t next_activity_file_sequence = 0;

// File paths
char activity_file_path[util::max_path_length];

// Mutexes
K_MUTEX_DEFINE(activity_file_mutex);
K_MUTEX_DEFINE(sequence_number_mutex);
K_MUTEX_DEFINE(calibration_file_mutex);

// Forward Declarations
static void data_init(void);
static void data_thread_fn(void *arg1, void *arg2, void *arg3);
static void process_sensor_data_work_handler(struct k_work *work);
static void process_command_work_handler(struct k_work *work);
static void process_calibration_work_handler(struct k_work *work);
static void process_erase_flash_work_handler(struct k_work *work);
static err_t mount_file_system(struct fs_mount_t *mount);
static err_t littlefs_flash_erase(unsigned int id);
static uint32_t get_next_file_sequence(const char *dir_path, const char *file_prefix);
err_t start_activity_logging(uint32_t sampling_frequency, const char *fw_version);
err_t end_activity_logging();
err_t save_bhi360_calibration_data(const bhi360_calibration_data_t *calib_data);
err_t cap_and_reindex_log_files(const char *file_prefix, const char *dir_path, uint8_t max_files, uint8_t delete_count);
int close_all_files_in_directory(const char *dir_path);
err_t delete_oldest_log_file_checked(const char *file_prefix, const char *dir_path);
int delete_all_files_in_directory(const char *dir_path);
static int lsdir(const char *path);
int find_latest_log_file(const char *file_prefix, record_type_t record_type, char *file_path,
                         const char *open_file_path);

int delete_oldest_log_files(const char *file_prefix, const char *dir_path);
err_t delete_by_type_and_id(record_type_t type, uint8_t id);
err_t type_and_id_to_path(record_type_t type, uint8_t id, char path[]);
err_t close_directory(fs_dir_t *dirp);
static void strremove(char *str, const char *sub);
static uint8_t get_filesystem_usage_percent(void);
static bool check_filesystem_space_and_cleanup(void);

typedef struct
{
    const char *str;
} string_callback_arg_t;

struct LogFileEntry
{
    uint8_t id;
    char name[util::max_path_length];
};

// TODO: Grab the user setting from moble phone
//  Default values that can be overwritten by settings
user_config_t user_config = {
    .user_height_cm = 175, // Default 175cm
    .user_weight_kg = 70,  // Default 70kg
    .user_age_years = 30,  // Default 30 years
    .user_sex = 1          // Default 1 (Male)
};

battery_info_t battery_start = {};

// Initialize the data module
static void data_init(void)
{
    LOG_INF("Initializing data module");

    // Initialize the fs_mount_t structure
    lfs_ext_storage_mnt.type = FS_LITTLEFS;
    lfs_ext_storage_mnt.mnt_point = "/lfs1";
    lfs_ext_storage_mnt.fs_data = &littlefs_storage;
    lfs_ext_storage_mnt.storage_dev = (void *)FLASH_AREA_ID(littlefs_storage);

    // Mount the file system
    err_t err = mount_file_system(&lfs_ext_storage_mnt);
    if (err != err_t::NO_ERROR)
    {
        LOG_ERR("Error mounting littlefs: %d", (int)err);
        filesystem_available = false;
        LOG_WRN("Filesystem not available - system will operate without logging "
                "capability");
    }
    else
    {
        filesystem_available = true;
        LOG_INF("File System Mounted at %s", lfs_ext_storage_mnt.mnt_point);
    }

    // Initialize work queue
    k_work_queue_init(&data_work_q);
    k_work_queue_start(&data_work_q, data_workq_stack, K_THREAD_STACK_SIZEOF(data_workq_stack), data_priority - 1,
                       NULL);
    k_thread_name_set(&data_work_q.thread, "data_wq");

    // Initialize work items
    k_work_init(&process_sensor_data_work, process_sensor_data_work_handler);
    k_work_init(&process_command_work, process_command_work_handler);
    k_work_init(&process_calibration_work, process_calibration_work_handler);
    k_work_init(&process_erase_flash_work, process_erase_flash_work_handler);

    // Initialize the fs_sync work item
    k_work_init(&fs_sync_work, fs_sync_work_handler);

    // Create the message processing thread
    data_tid = k_thread_create(&data_thread_data, data_stack_area, K_THREAD_STACK_SIZEOF(data_stack_area),
                               data_thread_fn, NULL, NULL, NULL, data_priority, 0, K_NO_WAIT);

    k_thread_name_set(data_tid, "data");

    module_set_state(MODULE_STATE_READY);
    LOG_INF("Data module initialized");
}

// Helper to find the next available sequence number for a log file
static uint32_t get_next_file_sequence(const char *dir_path, const char *file_prefix)
{
    fs_dir_t dirp;
    static fs_dirent entry;
    uint32_t max_seq = 1; // Start from 1
    int res;

    fs_dir_t_init(&dirp);

    res = fs_opendir(&dirp, dir_path);
    if (res != 0)
    {
        if (res == -ENOENT)
        {
            LOG_WRN("Directory '%s' does not exist yet for prefix '%s'. Starting "
                    "sequence from 1.",
                    dir_path, file_prefix);
            return 1;
        }
        LOG_ERR("Error opening directory '%s' [%d] for prefix '%s'. Cannot "
                "reliably determine next sequence.",
                dir_path, res, file_prefix);
        return 1;
    }

    while (true)
    {
        res = fs_readdir(&dirp, &entry);
        if (res != 0 || entry.name[0] == '\0')
        {
            break;
        }

        if (std::strncmp(entry.name, file_prefix, strlen(file_prefix)) == 0)
        {
            char *num_str_start = entry.name + strlen(file_prefix);
            uint8_t current_seq;
            if (std::sscanf(num_str_start, "%hhu.pb", &current_seq) == 1)
            {
                if (current_seq >= max_seq)
                {
                    max_seq = current_seq + 1;
                }
            }
        }
    }
    fs_closedir(&dirp);
    return max_seq;
}

// Main processing thread - waits for messages and queues appropriate work
static void data_thread_fn(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    generic_message_t msg;
    generic_message_t log_info_msg;

    if (filesystem_available)
    {
        // Notify about latest Activity log file
        uint8_t current_activity_seq = (uint8_t)get_next_file_sequence(hardware_dir_path, activity_file_prefix);
        uint8_t latest_activity_log_id_to_report = (current_activity_seq > 0) ? (current_activity_seq - 1) : 0;

        log_info_msg.type = MSG_TYPE_NEW_ACTIVITY_LOG_FILE;
        log_info_msg.data.new_hardware_log_file.file_sequence_id = latest_activity_log_id_to_report;
        if (latest_activity_log_id_to_report > 0)
        {
            type_and_id_to_path(RECORD_HARDWARE_ACTIVITY, latest_activity_log_id_to_report,
                                log_info_msg.data.new_hardware_log_file.file_path);
        }
        else
        {
            strncpy(log_info_msg.data.new_hardware_log_file.file_path, "",
                    sizeof(log_info_msg.data.new_hardware_log_file.file_path) - 1);
            log_info_msg.data.new_hardware_log_file
                .file_path[sizeof(log_info_msg.data.new_hardware_log_file.file_path) - 1] = '\0';
        }
        if (k_msgq_put(&bluetooth_msgq, &log_info_msg, K_NO_WAIT) != 0)
        {
            LOG_ERR("Failed to put initial activity log notification message.");
        }
        else
        {
            LOG_WRN("Initial activity log notification sent (ID: %u).", latest_activity_log_id_to_report);
        }
    }

    while (true)
    {
        // Wait for messages
        int ret = k_msgq_get(&data_msgq, &msg, K_FOREVER);

        if (ret == 0)
        {
            // Queue different work based on message type
            switch (msg.type)
            {

                case MSG_TYPE_REALTIME_METRICS_DATA:

                    if (atomic_get(&logging_activity_active) != 1)
                    {
                        break;
                    }
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    break;
#endif

                    memcpy(&pending_sensor_data_msg, &msg, sizeof(generic_message_t));

                    k_work_submit_to_queue(&data_work_q, &process_sensor_data_work);

                    break;

                case MSG_TYPE_COMMAND:
                    // Queue command - simplified like bluetooth module
                    k_mutex_lock(&cmd_queue_mutex, K_MSEC(100));

                    // Copy command to buffer
                    memcpy(&command_queue, &msg, sizeof(generic_message_t));

                    k_work_submit_to_queue(&data_work_q, &process_command_work);

                    k_mutex_unlock(&cmd_queue_mutex);
                    break;

                case MSG_TYPE_SAVE_BHI360_CALIBRATION:
                    // Check if work is already pending
                    if (!k_work_is_pending(&process_calibration_work))
                    {
                        k_mutex_lock(&calibration_msg_mutex, K_MSEC(100));
                        memcpy(&pending_calibration_data, &msg.data.bhi360_calibration,
                               sizeof(bhi360_calibration_data_t));
                        k_mutex_unlock(&calibration_msg_mutex);
                        k_work_submit_to_queue(&data_work_q, &process_calibration_work);
                    }
                    else
                    {
                        LOG_WRN("Calibration work already pending, dropping message");
                    }
                    break;

                case MSG_TYPE_ERASE_EXTERNAL_FLASH:
                    // Queue work to erase external flash
                    if (!k_work_is_pending(&process_erase_flash_work))
                    {
                        k_work_submit_to_queue(&data_work_q, &process_erase_flash_work);
                    }
                    else
                    {
                        LOG_WRN("Erase flash work already pending");
                    }
                    break;

                case MSG_TYPE_DELETE_ACTIVITY_LOG: {
// Process delete activity log command
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                    break;
#endif
                    LOG_INF("Received delete activity log command for ID: %u", msg.data.delete_cmd.id);

                    // Stop any active logging first if we're deleting files
                    if (atomic_get(&logging_activity_active))
                    {
                        LOG_WRN("Activity logging is active, stopping it before deletion");
                        atomic_set(&logging_activity_active, 0);
                        end_activity_logging();
                    }

                    // Delete the specified activity log file
                    err_t delete_status = delete_by_type_and_id(msg.data.delete_cmd.type, msg.data.delete_cmd.id);
                    if (delete_status != err_t::NO_ERROR)
                    {
                        LOG_ERR("Failed to delete activity log file with ID %u: %d", msg.data.delete_cmd.id,
                                (int)delete_status);
                    }
                    else
                    {
                        LOG_INF("Successfully deleted activity log file with ID %u", msg.data.delete_cmd.id);
                    }
                    break;
                }

                case MSG_TYPE_FOOT_WEIGHT_MAP_DATA: {
                    // Process foot weight map calibration data
                    LOG_INF("Received foot weight map calibration data");

                    // Save the calibration data to external flash
                    if (filesystem_available)
                    {
                        // Create calibration directory if it doesn't exist
                        int ret = fs_mkdir(calibration_dir_path);
                        if (ret != 0 && ret != -EEXIST)
                        {
                            LOG_ERR("Failed to create calibration directory: %d", ret);
                        }
                        else
                        {
                            // Create filename for foot weight map calibration
                            char filename[util::max_path_length];
                            snprintk(filename, sizeof(filename), "%s/foot_weight_map.bin", calibration_dir_path);

                            // Open file for writing
                            struct fs_file_t file;
                            fs_file_t_init(&file);

                            ret = fs_open(&file, filename, FS_O_CREATE | FS_O_WRITE);
                            if (ret != 0)
                            {
                                LOG_ERR("Failed to open foot weight map file %s: %d", filename, ret);
                            }
                            else
                            {
                                // Write calibration data
                                ret = fs_write(&file, &msg.data.foot_weight_map, sizeof(foot_weight_map_data_t));
                                if (ret < 0)
                                {
                                    LOG_ERR("Failed to write foot weight map data: %d", ret);
                                }
                                else
                                {
                                    LOG_INF("Foot weight map calibration saved to %s", filename);
                                    LOG_INF("  Total weight: %u", msg.data.foot_weight_map.total_weight);
                                    LOG_INF("  Timestamp: %u ms", msg.data.foot_weight_map.timestamp_ms);
                                    LOG_INF("  Sample count: %u", msg.data.foot_weight_map.sample_count);

                                    // Log sensor distribution
                                    for (uint8_t i = 0; i < NUM_FOOT_SENSOR_CHANNELS; i++)
                                    {
                                        LOG_INF("  Sensor[%d]: %u (%.1f%%)", i, msg.data.foot_weight_map.sensor_map[i],
                                                msg.data.foot_weight_map.weight_distribution[i]);
                                    }
                                }

                                // Sync and close file
                                fs_sync(&file);
                                fs_close(&file);
                            }
                        }
                    }
                    else
                    {
                        LOG_WRN("Cannot save foot weight map - filesystem not available");
                    }
                    break;
                }

                default:
                    LOG_WRN("Received unsupported message type %d  from sender %d", msg.type, msg.sender);
                    break;
            }
        }
    }
}

// Work handler for processing sensor data
static void process_sensor_data_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    uint32_t current_epoch = get_current_epoch_time();
    uint16_t delta_epoch = 0U;

    // Make a local copy of the message under mutex protection
    k_mutex_lock(&sensor_data_msg_mutex, K_MSEC(100));
    generic_message_t local_msg = pending_sensor_data_msg;
    k_mutex_unlock(&sensor_data_msg_mutex);

    if (atomic_get(&logging_activity_active))
    {

        const realtime_metrics_t *metrics = &local_msg.data.realtime_metrics;

        LOG_INF("Writing Activity data metrics %d", metrics->timestamp_ms);
        // Prepare a binary structure to store in the file


        // Reset counter if approaching maximum to prevent overflow
        if (activity_packet_counter >= (UINT32_MAX - 100))
        {
            LOG_INF("Resetting activity packet counter to prevent overflow (was %u)", activity_packet_counter);
            activity_packet_counter = 0;
        }

        activity_metrics_binary_t binary_metrics = {.packet_number = activity_packet_counter++,
                                                    .timestamp = current_epoch,
                                                    .cadence_spm = metrics->cadence_spm,
                                                    .pace_sec_km = metrics->pace_sec_km,
                                                    .speed_kmh_x10 =
                                                        (metrics->pace_sec_km > 0) ? (36000 / metrics->pace_sec_km) : 0,
                                                    .balance_lr_pct = metrics->balance_lr_pct,
                                                    .ground_contact_ms = metrics->ground_contact_ms,
                                                    .flight_time_ms = metrics->flight_time_ms,
                                                    .contact_time_asymmetry = metrics->contact_time_asymmetry,
                                                    .force_asymmetry = metrics->force_asymmetry,
                                                    .pronation_asymmetry = metrics->pronation_asymmetry,
                                                    .left_strike_pattern = metrics->left_strike_pattern,
                                                    .right_strike_pattern = metrics->right_strike_pattern,
                                                    .avg_pronation_deg = metrics->avg_pronation_deg,
                                                    .vertical_ratio = metrics->vertical_ratio};

        /*    LOG_WRN("activity_packet_counter = %u\n"
                    "timestamp = %u\n"
                    "cadence = %u\n"
                    "pace = %u\n"
                    "speed = %u\n"
                    "balance = %d\n"
                    "ground_contact = %u\n"
                    "flight_time = %u\n"
                    "contact_time_asymmetry = %d\n"
                    "force_asymmetry = %d\n"
                    "pronation_asymmetry = %d\n"
                    "left_strike_pattern = %u\n"
                    "right_strike_pattern = %u\n"
                    "avg_pronation_deg = %d\n"
                    "vertical_ratio = %u\n",
                    binary_metrics.packet_number, binary_metrics.timestamp_ms, binary_metrics.cadence_spm,
                    binary_metrics.pace_sec_km, binary_metrics.speed_kmh_x10, binary_metrics.balance_lr_pct,
                    binary_metrics.ground_contact_ms, binary_metrics.flight_time_ms,
           binary_metrics.contact_time_asymmetry, binary_metrics.force_asymmetry, binary_metrics.pronation_asymmetry,
           binary_metrics.left_strike_pattern, binary_metrics.right_strike_pattern, binary_metrics.avg_pronation_deg,
           binary_metrics.vertical_ratio); */

        // TODO: workaround only, send data to bluetooth:
        generic_message_t msg = {};
        msg.sender = SENDER_DATA;
        msg.type = MSG_TYPE_ACTIVITY_METRICS;

        memcpy(&msg.data.activity_metrics, &binary_metrics, sizeof(binary_metrics));

        if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0)
        {
            LOG_WRN("Bluetooth queue full, dropping activity metrics");
        }

        /*   k_mutex_lock(&activity_file_mutex, K_FOREVER);
           int ret = fs_write(&activity_log_file, &binary_metrics, sizeof(binary_metrics));
           k_mutex_unlock(&activity_file_mutex);
           if (ret < 0)
           {
               LOG_ERR("Failed to write activity buffer: %d", ret);

               return;
           }


           atomic_set(&sync_pending, 1);
       } */
    }
}
// Work handler for processing commands - simplified like bluetooth module
static void process_command_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    // Get the command from the buffer
    k_mutex_lock(&cmd_queue_mutex, K_MSEC(100));
    generic_message_t local_msg = command_queue;
    k_mutex_unlock(&cmd_queue_mutex);

    // Process the command
    const char *command_str = local_msg.data.command_str;
    LOG_WRN("Processing command: %s activity logging =%ld", command_str, atomic_get(&logging_activity_active));

    // For STOP commands, cancel any pending sensor data work first
    if (strstr(command_str, "STOP") != NULL)
    {
        LOG_INF("STOP command detected - cancelling sensor data work");
        k_work_cancel(&process_sensor_data_work);
    }

    if (strcmp(command_str, "START_LOGGING_ACTIVITY") == 0)
    {
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        return;
#endif
        if (atomic_get(&logging_activity_active) == 0)
        {
            LOG_INF("Starting activity logging");
            // Use values from the command message's metadata fields
            err_t activity_status = start_activity_logging(local_msg.sampling_frequency, local_msg.fw_version);
            if (activity_status == err_t::NO_ERROR)
            {
                atomic_set(&logging_activity_active, 1);
            }
            else
            {
                LOG_ERR("Failed to start activity logging: %d", (int)activity_status);
            }
        }
        else
        {
            LOG_WRN("Activity logging already active, ignoring START command");
        }
    }
    else if (strcmp(command_str, "STOP_LOGGING_ACTIVITY") == 0)
    {
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        return;
#endif
        if (atomic_get(&logging_activity_active) == 1)
        {
            LOG_INF("Stopping activity logging");
            atomic_set(&logging_activity_active, 0);

            err_t activity_status = end_activity_logging();
            if (activity_status != err_t::NO_ERROR)
            {
                LOG_ERR("Failed to stop activity logging: %d", (int)activity_status);
            }
            else
            {
                LOG_INF("Activity logging stopped successfully");
            }
            // Add small delay as in original to allow Bluetooth queue processing
            k_msleep(10);
        }
        else
        {
            LOG_WRN("Activity logging not active, ignoring STOP command");
        }
    }
    else if (strcmp(command_str, "STOP_LOGGING") == 0)
    {
        LOG_INF("Stopping all logging");
        if (atomic_get(&logging_activity_active) == 1)
        {
            atomic_set(&logging_activity_active, 0);
            if (filesystem_available)
            {
                err_t activity_status = end_activity_logging();
                if (activity_status != err_t::NO_ERROR)
                {
                    LOG_ERR("Failed to stop activity logging: %d", (int)activity_status);
                }
            }
        }
    }
    else
    {
        LOG_WRN("Unknown command: %s", command_str);
    }
}

// Work handler for processing calibration data
static void process_calibration_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    // Make a local copy of the calibration data under mutex protection
    k_mutex_lock(&calibration_msg_mutex, K_MSEC(100));
    bhi360_calibration_data_t local_calib_data = pending_calibration_data;
    k_mutex_unlock(&calibration_msg_mutex);

    LOG_INF("Processing BHI360 calibration data for sensor type: %u", local_calib_data.sensor_type);

    if (!filesystem_available)
    {
        LOG_WRN("Cannot save calibration - filesystem not available");
        return;
    }

    err_t save_status = save_bhi360_calibration_data(&local_calib_data);
    if (save_status != err_t::NO_ERROR)
    {
        LOG_ERR("Failed to save BHI360 calibration data: %d", (int)save_status);
    }
    else
    {
        LOG_INF("Successfully saved BHI360 calibration data");
    }
}

// Work handler for processing external flash erase
static void process_erase_flash_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    LOG_INF("Starting external flash erase operation");

    // Stop foot and motion sensor

    struct foot_sensor_stop_activity_event *foot_evt = new_foot_sensor_stop_activity_event();
    APP_EVENT_SUBMIT(foot_evt);

    struct motion_sensor_stop_activity_event *motion_evt = new_motion_sensor_stop_activity_event();
    APP_EVENT_SUBMIT(motion_evt);

    // Stop any active logging first
    if (atomic_get(&logging_activity_active))
    {
        LOG_INF("Stopping active logging before flash erase");
        atomic_set(&logging_activity_active, 0);
        if (filesystem_available)
        {
            end_activity_logging();
        }
    }

    // Close any open files
    if (activity_log_file.filep != nullptr)
    {
        fs_close(&activity_log_file);
        activity_log_file.filep = nullptr;
    }

    // Erase the flash
    LOG_WRN("Delete external flash - this may take a while...");

    int erase_status = delete_all_files_in_directory(hardware_dir_path);

    if (erase_status < 0)
    {
        LOG_ERR("Failed to erase external flash: %d", (int)erase_status);
    }
    else
    {
        LOG_INF("External flash erased successfully");
        // Reset sequence numbers
        next_activity_file_sequence = 0;
    }

    // Send completion notification to bluetooth module
    generic_message_t complete_msg = {};
    complete_msg.sender = SENDER_DATA;
    complete_msg.type = MSG_TYPE_EXTERNAL_FLASH_ERASE_COMPLETE;

    if (k_msgq_put(&bluetooth_msgq, &complete_msg, K_NO_WAIT) != 0)
    {
        LOG_ERR("Failed to send flash erase complete notification to bluetooth");
    }
    else
    {
        LOG_INF("Sent external flash erase complete notification");
    }
}

// Mount the file system
static err_t mount_file_system(struct fs_mount_t *mount)
{
    const struct device *flash_device = NULL;

    // Try PCB flash first
#if DT_NODE_EXISTS(DT_NODELABEL(w25q128))
    flash_device = DEVICE_DT_GET(DT_NODELABEL(w25q128));
    if (device_is_ready(flash_device))
    {
        LOG_WRN("Using Winbond W25Q128 flash (PCB)");
    }
    else
#endif
    {
        // Fall back to DK flash
#if DT_NODE_EXISTS(DT_NODELABEL(mx25r64))
        flash_device = DEVICE_DT_GET(DT_NODELABEL(mx25r64));
        if (device_is_ready(flash_device))
        {
            LOG_INF("Using Macronix MX25R64 flash (DK)");
        }
#endif
    }

    if (!device_is_ready(flash_device))
    {
        LOG_ERR("External flash device isn't ready");
        return err_t::DATA_ERROR;
    }

    // Conditionally erase flash if config is enabled
    if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE))
    {
        err_t rc = littlefs_flash_erase((uintptr_t)mount->storage_dev);
        if (rc != err_t::NO_ERROR)
        {
            return err_t::DATA_ERROR;
        }
    }

    int ret = fs_mount(mount);
    if (ret != 0)
    {
        LOG_ERR("FS mount failed: %d", ret);
        return err_t::DATA_ERROR;
    }

    return err_t::NO_ERROR;
}

// Erase flash area
static err_t littlefs_flash_erase(unsigned int id)
{
    const struct flash_area *pfa;
    int rc;

    rc = flash_area_open(id, &pfa);
    if (rc < 0)
    {
        LOG_ERR("FAIL: unable to find flash area %u: %d", id, rc);
        return err_t::DATA_ERROR;
    }

    LOG_WRN("Flash Area %u at 0x%x on %s for %u bytes", id, (unsigned int)pfa->fa_off, pfa->fa_dev->name,
            (unsigned int)pfa->fa_size);

    if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE))
    {
        LOG_WRN("Erasing flash area...");
        rc = flash_area_flatten(pfa, 0, pfa->fa_size);
        if (rc != 0)
        {
            LOG_ERR("Failed to erase flash area: %d", rc);
            flash_area_close(pfa);
            return err_t::DATA_ERROR;
        }
        LOG_WRN("Flash area erased successfully.");
    }

    flash_area_close(pfa);
    return err_t::NO_ERROR;
}

// Start activity logging
err_t start_activity_logging(uint32_t sampling_frequency, const char *fw_version)
{
    // Acquire a mutex for all activity file operations to ensure atomicity.
    k_mutex_lock(&activity_file_mutex, K_FOREVER);

    err_t status = err_t::NO_ERROR;

    // Declare all variables at the beginning of the function
    int ret_mkdir;
    int ret_fs_open;
    int ret_write;
    const uint8_t *uuid;
    typedef struct __attribute__((packed))
    {
        char magic[4];
        uint8_t version;
        uint32_t start_time;
        uint32_t sample_rate;
        char fw_version[16];
        uint8_t user_height_cm;
        uint8_t user_weight_kg;
        uint8_t user_age_years;
        uint8_t user_sex;
        battery_info_t battery;
        uint8_t service_uuid[16];
    } ActivityFileHeaderV3;
    ActivityFileHeaderV3 header;

    // Reset tracking variables
    activity_last_timestamp_ms = get_current_epoch_time();
    start_epoch_timestamp = activity_last_timestamp_ms;
    last_epoch_timestamp = 0;
    activity_first_packet = true;
    activity_batch_count = 0;
    activity_write_buffer_pos = 0;
    activity_packet_counter = 0;

    // Create directory if needed
    /* ret_mkdir = fs_mkdir(hardware_dir_path);
      if (ret_mkdir != 0 && ret_mkdir != -EEXIST)
      {
          LOG_ERR("Failed to create directory %s: %d", hardware_dir_path, ret_mkdir);
          status = err_t::FILE_SYSTEM_ERROR;
          goto cleanup;
      } */

    // Get next sequence number and create file path
    /*  next_activity_file_sequence = (uint8_t)get_next_file_sequence(hardware_dir_path, activity_file_prefix);
      if (next_activity_file_sequence == 0)
      {
          next_activity_file_sequence = 1;
      }
      snprintk(activity_file_path, sizeof(activity_file_path), "%s/%s%03u.dat", hardware_dir_path, activity_file_prefix,
               next_activity_file_sequence);

      LOG_WRN("activity_file_path=%s, activity_file_path_size=%d, hardware_dir_path=%s,    activity_file_prefix=%s, "
              "next_activity_file_sequence=%d",
              activity_file_path, sizeof(activity_file_path), hardware_dir_path, activity_file_prefix,
              next_activity_file_sequence);
      LOG_WRN("Start Epoch time is %u", activity_last_timestamp_ms);
      fs_file_t_init(&activity_log_file); */

    // Open file
    /*    ret_fs_open = fs_open(&activity_log_file, activity_file_path, FS_O_CREATE | FS_O_RDWR | FS_O_APPEND);
        if (ret_fs_open != 0)
        {
            LOG_ERR("Failed to open activity log %s: %d", activity_file_path, ret_fs_open);
            status = err_t::FILE_SYSTEM_ERROR;
            goto cleanup;
        }
        LOG_INF("Opened new activity log file: %s", activity_file_path); */

    // Initialize the header struct
    header = {.magic = {'B', 'O', 'T', 'Z'},
              .version = 3,
              .start_time = activity_last_timestamp_ms,
              .sample_rate = sampling_frequency,
              .user_height_cm = user_config.user_height_cm,
              .user_weight_kg = user_config.user_weight_kg,
              .user_age_years = user_config.user_age_years,
              .user_sex = user_config.user_sex,
              .battery = battery_start};

    strncpy(header.fw_version, fw_version, sizeof(header.fw_version) - 1);
    header.fw_version[sizeof(header.fw_version) - 1] = '\0';

    uuid = get_target_service_uuid();
    if (uuid != nullptr)
    {
        memcpy(header.service_uuid, uuid, 16);
        LOG_INF("Activity header includes BLE service UUID");
    }
    else
    {
        memset(header.service_uuid, 0, 16);
        LOG_WRN("Could not get BLE service UUID, using zeros");
    }

    generic_message_t msg = {};
    msg.sender = SENDER_DATA;
    msg.type = MSG_TYPE_ACTIVITY_HEADER;

    memcpy(&msg.data.activity_header, &header, sizeof(header));

    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0)
    {
        LOG_WRN("Bluetooth queue full, dropping activity header");
    }

    return err_t::NO_ERROR;

    /*  ret_write = fs_write(&activity_log_file, &header, sizeof(header));
      if (ret_write < 0)
      {
          LOG_ERR("Failed to write activity header: %d", ret_write);
          status = err_t::DATA_ERROR;
          goto cleanup_close;
      }

      ret_write = fs_sync(&activity_log_file);
      if (ret_write != 0)
      {
          LOG_ERR("Failed to sync activity file: %d", ret_write);
          status = err_t::DATA_ERROR;
          goto cleanup_close;
      } */

    LOG_INF("Activity session started. Battery: %d%%, Voltage: %dmV", battery_start.percentage,
            battery_start.voltage_mV);

cleanup_close:
    // If an error occurred after opening, close the file before cleaning up.
    if (status != err_t::NO_ERROR)
    {
        fs_close(&activity_log_file);
        activity_log_file.filep = nullptr;
    }

cleanup:
    k_mutex_unlock(&activity_file_mutex);
    // Start the periodic sync timer after successfully opening the file
    if (status == err_t::NO_ERROR)
    {
        k_timer_start(&fs_sync_timer, K_SECONDS(5), K_SECONDS(5));
        LOG_INF("Periodic fs_sync timer started.");
    }

    return status;
}

// End activity logging
err_t end_activity_logging()
{

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    return err_t::NO_ERROR;
#endif

    err_t overall_status = err_t::NO_ERROR;

    LOG_WRN("Closing file");

    // Stop the periodic timer
    k_timer_stop(&fs_sync_timer);
    LOG_INF("Periodic fs_sync timer stopped.");

    // Lock the mutex for all file operations
    //   k_mutex_lock(&activity_file_mutex, K_FOREVER);

    if (activity_log_file.filep != nullptr)
    {

        // 2. Get final battery state
        // battery_info_t battery_end = get_battery_info();

        battery_info_t battery_end = {
            .voltage_mV = 3500, // Placeholder, replace with actual voltage reading
            .percentage = 90,   // Placeholder, replace with actual battery reading
        };



        ActivityFileFooterV3 footer = {.end_time = get_current_epoch_time(),
                                       .record_count = activity_batch_count,
                                       .packet_count = activity_packet_counter,
                                       .file_crc = 0, // Would be calculated in full implementation
                                       .battery = battery_end};

        generic_message_t msg = {};
        msg.sender = SENDER_DATA;
        msg.type = MSG_TYPE_ACTIVITY_FOOTER;

            memcpy(&msg.data.activity_footer, &footer, sizeof(footer));

        if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0)
        {
            LOG_WRN("Bluetooth queue full, dropping activity footer");
        }

        return err_t::NO_ERROR;

        /*    int ret = fs_write(&activity_log_file, &footer, sizeof(footer));
            if (ret < 0)
            {
                LOG_ERR("Failed to write activity buffer: %d", ret);
            }

            ret = fs_sync(&activity_log_file);
            if (ret != 0)
            {
                LOG_ERR("Failed to sync activity file: %d", ret);
            } */

        // 7. Close file
        /*   int ret_close = fs_close(&activity_log_file);
           if (ret_close != 0)
           {
               LOG_ERR("Failed to close file: %d", ret_close);
               overall_status = err_t::FILE_SYSTEM_ERROR;
           }
           else
           {
               LOG_INF("Session ended");
           } */

        // Always send notification after closing, regardless of close result
        /*      generic_message_t activity_msg;
              activity_msg.sender = SENDER_DATA;
              activity_msg.type = MSG_TYPE_NEW_ACTIVITY_LOG_FILE;
              activity_msg.data.new_hardware_log_file.file_sequence_id = next_activity_file_sequence;
              strncpy(activity_msg.data.new_hardware_log_file.file_path, activity_file_path,
                      sizeof(activity_msg.data.new_hardware_log_file.file_path) - 1);
              activity_msg.data.new_hardware_log_file
                  .file_path[sizeof(activity_msg.data.new_hardware_log_file.file_path) - 1] = '\0';

              if (k_msgq_put(&bluetooth_msgq, &activity_msg, K_NO_WAIT) != 0)
              {
                  LOG_WRN("Failed to send session notification");
              }

              // 9. Reset state
              activity_batch_count = 0;
              activity_write_buffer_pos = 0;
              activity_log_file.filep = nullptr; // Explicitly set to nullptr after closing

              // Unlock the mutex after all file operations are complete
              k_mutex_unlock(&activity_file_mutex);
          } */

        LOG_WRN("Closing file CLOSED!!!!!!!!");

#if IS_ENABLED(CONFIG_LAB_VERSION)
        // Send message to copy the file to SD card
        generic_message_t copy_msg = {};
        copy_msg.sender = SENDER_DATA;
        copy_msg.type = MSG_TYPE_COPY_FILE_TO_SD;

        // Copy the current activity file path
        strncpy(copy_msg.data.copy_to_sd.source_path, activity_file_path,
                sizeof(copy_msg.data.copy_to_sd.source_path) - 1);
        copy_msg.data.copy_to_sd.source_path[sizeof(copy_msg.data.copy_to_sd.source_path) - 1] = '\0';

        // Generate destination filename (keep same name but with different extension for clarity)
        snprintf(copy_msg.data.copy_to_sd.dest_filename, sizeof(copy_msg.data.copy_to_sd.dest_filename),
                 "activity_%03u_copy.dat", next_activity_file_sequence);

        if (k_msgq_put(&data_sd_msgq, &copy_msg, K_NO_WAIT) != 0)
        {
            LOG_ERR("Failed to send copy file message to SD module");
        }
        else
        {
            LOG_INF("Requested copy of activity file %s to SD card as %s", activity_file_path,
                    copy_msg.data.copy_to_sd.dest_filename);
        }
#endif

        // TODO: test only - list directory contents
        /*    LOG_WRN("About to list directory contents for: %s", hardware_dir_path);
            int lsdir_result = lsdir(hardware_dir_path);
            if (lsdir_result != 0)
            {
                LOG_ERR("lsdir failed with error: %d", lsdir_result);
            } */

        return overall_status;
    }
}

// Save BHI360 calibration data
err_t save_bhi360_calibration_data(const bhi360_calibration_data_t *calib_data)
{
    if (!calib_data)
    {
        return err_t::INVALID_PARAMETER;
    }

    // Lock mutex for thread safety
    k_mutex_lock(&calibration_file_mutex, K_MSEC(100));

    // Create calibration directory if it doesn't exist
    int ret = fs_mkdir(calibration_dir_path);
    if (ret != 0 && ret != -EEXIST)
    {
        LOG_ERR("Failed to create calibration directory: %d", ret);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Create filename based on sensor type
    char filename[util::max_path_length];
    const char *sensor_name;
    switch (calib_data->sensor_type)
    {
        case 0:
            sensor_name = "accel";
            break;
        case 1:
            sensor_name = "gyro";
            break;
        default:
            LOG_ERR("Invalid sensor type: %u (only 0=accel, 1=gyro supported)", calib_data->sensor_type);
            k_mutex_unlock(&calibration_file_mutex);
            return err_t::INVALID_PARAMETER;
    }

    snprintk(filename, sizeof(filename), "%s/%s%s.bin", calibration_dir_path, bhi360_calib_file_prefix, sensor_name);

    // Open file for writing
    struct fs_file_t file;
    fs_file_t_init(&file);

    ret = fs_open(&file, filename, FS_O_CREATE | FS_O_WRITE);
    if (ret != 0)
    {
        LOG_ERR("Failed to open calibration file %s: %d", filename, ret);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Write calibration data
    ret = fs_write(&file, calib_data, sizeof(bhi360_calibration_data_t));
    if (ret < 0)
    {
        LOG_ERR("Failed to write calibration data: %d", ret);
        fs_close(&file);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Sync and close file
    fs_sync(&file);
    fs_close(&file);

    LOG_INF("Saved BHI360 %s calibration to %s", sensor_name, filename);
    k_mutex_unlock(&calibration_file_mutex);
    return err_t::NO_ERROR;
}

// Cap and reindex log files for a given prefix and directory
err_t cap_and_reindex_log_files(const char *file_prefix, const char *dir_path, uint8_t max_files, uint8_t delete_count)
{
    std::vector<LogFileEntry> log_files;
    fs_dir_t dirp;
    static fs_dirent entry;

    fs_dir_t_init(&dirp);

    int res = fs_opendir(&dirp, dir_path);
    if (res != 0)
    {
        LOG_ERR("Opend dir Error");
        if (res == -ENOENT)
        {
            // Directory doesn't exist, nothing to do
            LOG_ERR("Directory do not exist");
            return err_t::NO_ERROR;
        }
        return err_t::FILE_SYSTEM_ERROR;
    }
    LOG_INF("Checking log files in dir: %s for prefix: %s", dir_path, file_prefix);
    // 1. Collect all matching files
    while (true)
    {
        res = fs_readdir(&dirp, &entry);
        if (res != 0 || entry.name[0] == '\0')
            break;

        size_t prefix_len = strlen(file_prefix);
        size_t ext_len = strlen(".pb");
        size_t name_len = strlen(entry.name);

        if (entry.type == FS_DIR_ENTRY_FILE && name_len > (prefix_len + ext_len) &&
            strncmp(entry.name, file_prefix, prefix_len) == 0 && strcmp(entry.name + name_len - ext_len, ".pb") == 0)
        {
            uint8_t id = 0;
            if (sscanf(entry.name + prefix_len, "%3hhu.pb", &id) == 1)
            {
                LogFileEntry lfe;
                lfe.id = id;
                strncpy(lfe.name, entry.name, sizeof(lfe.name) - 1);
                lfe.name[sizeof(lfe.name) - 1] = '\0';
                log_files.push_back(lfe);
            }
        }
    }
    fs_closedir(&dirp);

    // 2. Sort by ID
    std::sort(log_files.begin(), log_files.end(),
              [](const LogFileEntry &a, const LogFileEntry &b) { return a.id < b.id; });

    // 3. If too many files, delete the oldest
    size_t file_count = log_files.size();
    LOG_INF("Found %zu log files for prefix %s", file_count, file_prefix);
    if (file_count > max_files)
    {
        size_t to_delete = std::min<size_t>(delete_count, file_count - max_files + delete_count);
        LOG_INF("Capping: Deleting %zu oldest files for prefix %s", to_delete, file_prefix);
        for (size_t i = 0; i < to_delete; ++i)
        {
            char full_path[UTIL_MAX_INTERNAL_PATH_LENGTH];
            int written = snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, log_files[i].name);
            if (written < 0 || written >= (int)sizeof(full_path))
            {
                LOG_ERR("Internal path buffer too small for deleting: '%s/%s'", dir_path, log_files[i].name);
                continue;
            }
            int del_res = fs_unlink(full_path);
            if (del_res != 0)
            {
                LOG_ERR("Failed to unlink %s: %d", full_path, del_res);
            }
            else
            {
                LOG_INF("Deleted log file: %s", full_path);
            }
        }
        // Remove deleted files from vector
        log_files.erase(log_files.begin(), log_files.begin() + to_delete);
    }

    // 4. Reindex only if more than 255 files
    if (log_files.size() > 255)
    {
        for (size_t i = 0; i < log_files.size(); ++i)
        {
            uint8_t new_id = (uint8_t)(i + 1); // Start from 1, not 0
            if (log_files[i].id != new_id)
            {
                char old_path[UTIL_MAX_INTERNAL_PATH_LENGTH];
                char new_name[32];
                char new_path[UTIL_MAX_INTERNAL_PATH_LENGTH];
                snprintf(old_path, sizeof(old_path), "%s/%s", dir_path, log_files[i].name);
                snprintf(new_name, sizeof(new_name), "%s%03u.pb", file_prefix, new_id);
                snprintf(new_path, sizeof(new_path), "%s/%s", dir_path, new_name);
                int rename_res = fs_rename(old_path, new_path);
                if (rename_res != 0)
                {
                    LOG_ERR("Failed to rename %s to %s: %d", old_path, new_path, rename_res);
                }
                else
                {
                    LOG_INF("Renamed %s to %s", old_path, new_path);
                }
            }
        }
    }
    // All IDs are now contiguous and <= 255
    return err_t::NO_ERROR;
}

/**
 * @brief Delete the oldest log file matching the prefix in the directory, using
 * large internal buffer for file ops. BLE/external buffers remain small for
 * transmission.
 */
err_t delete_oldest_log_file_checked(const char *file_prefix, const char *dir_path)
{
    fs_dir_t dirp;
    static fs_dirent entry;
    fs_dir_t_init(&dirp);

    int res = fs_opendir(&dirp, dir_path);
    if (res != 0)
    {
        if (res == -ENOENT)
        {
            // Directory doesn't exist, nothing to delete
            return err_t::FILE_SYSTEM_NO_FILES;
        }
        return err_t::FILE_SYSTEM_ERROR;
    }

    uint8_t oldest_id = UINT8_MAX;
    char oldest_name[util::max_path_length] = {0};
    size_t prefix_len = strlen(file_prefix);
    size_t ext_len = strlen(".pb");

    // Find the oldest file (lowest id)
    while (true)
    {
        res = fs_readdir(&dirp, &entry);
        if (res != 0 || entry.name[0] == '\0')
            break;

        size_t name_len = strlen(entry.name);
        if (entry.type == FS_DIR_ENTRY_FILE && name_len > (prefix_len + ext_len) &&
            strncmp(entry.name, file_prefix, prefix_len) == 0 && strcmp(entry.name + name_len - ext_len, ".pb") == 0)
        {
            uint8_t id = 0;
            if (sscanf(entry.name + prefix_len, "%3hhu.pb", &id) == 1)
            {
                if (id < oldest_id)
                {
                    oldest_id = id;
                    strncpy(oldest_name, entry.name, sizeof(oldest_name) - 1);
                    oldest_name[sizeof(oldest_name) - 1] = '\0';
                }
            }
        }
    }
    fs_closedir(&dirp);

    if (oldest_id == UINT8_MAX)
    {
        // No file found
        return err_t::FILE_SYSTEM_NO_FILES;
    }

    // Use a large buffer for file system operations
    char full_path[UTIL_MAX_INTERNAL_PATH_LENGTH];
    int written = snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, oldest_name);
    if (written < 0 || written >= (int)sizeof(full_path))
    {
        LOG_ERR("Internal path buffer too small for deleting oldest: '%s/%s'", dir_path, oldest_name);
        return err_t::FILE_SYSTEM_ERROR;
    }
    int del_res = fs_unlink(full_path);
    if (del_res != 0)
    {
        LOG_ERR("Failed to unlink %s: %d", full_path, del_res);
        return err_t::FILE_SYSTEM_ERROR;
    }
    LOG_INF("Deleted oldest log file: %s (ID: %u)", full_path, oldest_id);
    return err_t::NO_ERROR;
}

// Utility: Close all files in a directory by opening and closing each file.
// This does NOT track already-open handles elsewhere in the code.
int close_all_files_in_directory(const char *dir_path)
{
    struct fs_dir_t dirp;
    static struct fs_dirent entry;
    int closed_count = 0;

    fs_dir_t_init(&dirp);

    int res = fs_opendir(&dirp, dir_path);
    if (res != 0)
    {
        LOG_ERR("Failed to open directory %s: %d", dir_path, res);
        return res;
    }

    while (true)
    {
        res = fs_readdir(&dirp, &entry);
        if (res != 0 || entry.name[0] == '\0')
        {
            break;
        }

        if (entry.type == FS_DIR_ENTRY_FILE)
        {
            char file_path[256];
            int written = snprintf(file_path, sizeof(file_path), "%s/%s", dir_path, entry.name);
            if (written < 0 || written >= (int)sizeof(file_path))
            {
                LOG_ERR("Path buffer too small for %s/%s", dir_path, entry.name);
                continue;
            }

            struct fs_file_t file;
            fs_file_t_init(&file);
            int open_res = fs_open(&file, file_path, FS_O_RDWR);
            if (open_res == 0)
            {
                fs_close(&file);
                closed_count++;
                LOG_INF("Closed file: %s", file_path);
            }
            else if (open_res == -EACCES)
            {
                // File may be open elsewhere or not accessible for write
                LOG_WRN("Could not open file for closing (may be open elsewhere): %s", file_path);
            }
            else
            {
                LOG_WRN("Could not open file %s: %d", file_path, open_res);
            }
        }
    }

    fs_closedir(&dirp);
    LOG_INF("Closed %d files in directory %s", closed_count, dir_path);
    return closed_count;
}

/**
 * @brief Deletes all files in the specified directory.
 *
 * @param dir_path Path to the directory.
 * @return Number of files deleted, or negative error code on failure.
 */
int delete_all_files_in_directory(const char *dir_path)
{
    struct fs_dir_t dirp;
    static struct fs_dirent entry;
    int deleted_count = 0;

    fs_dir_t_init(&dirp);

    int res = fs_opendir(&dirp, dir_path);
    if (res != 0)
    {
        LOG_ERR("Failed to open directory %s: %d", dir_path, res);
        return res;
    }
    else
    {
        LOG_INF("Opened directory %s successfully", dir_path);
    }

    while (true)
    {
        res = fs_readdir(&dirp, &entry);
        if (res != 0 || entry.name[0] == '\0')
        {
            break;
        }

        if (entry.type == FS_DIR_ENTRY_FILE)
        {
            char file_path[256];
            int written = snprintf(file_path, sizeof(file_path), "%s/%s", dir_path, entry.name);
            if (written < 0 || written >= (int)sizeof(file_path))
            {
                LOG_ERR("Path buffer too small for %s/%s", dir_path, entry.name);
                continue;
            }

            int del_res = fs_unlink(file_path);
            if (del_res == 0)
            {
                deleted_count++;
                LOG_INF("Deleted file: %s", file_path);
            }
            else
            {
                LOG_ERR("Failed to delete file %s: %d", file_path, del_res);
            }
        }
    }

    fs_closedir(&dirp);
    LOG_INF("Deleted %d files in directory %s", deleted_count, dir_path);
    return deleted_count;
}

err_t type_and_id_to_path(record_type_t type, uint8_t id, char path[])
{
    switch (type)
    {
        case RECORD_HARDWARE_ACTIVITY: // Case for Activity files
            snprintk(path, util::max_path_length, "%s/%s%03u.pb", hardware_dir_path, activity_file_prefix, id);
            break;
        default:
            LOG_ERR("File path generation not implemented for type: %d", (int)type);
            return err_t::DATA_ERROR;
    }

    return err_t::NO_ERROR;
}

err_t delete_by_type_and_id(record_type_t type, uint8_t id)
{
    char path_to_delete[util::max_path_length] = {};

    err_t err = type_and_id_to_path(type, id, path_to_delete);
    if (err != err_t::NO_ERROR)
    {
        LOG_ERR("Failed to get path for deletion for type %d, ID %u: %d", (int)type, id, (int)err);
        return err;
    }
    LOG_WRN("Attempting to delete file: %s", path_to_delete);
    int ret = fs_unlink(path_to_delete);
    if (ret != 0)
    {
        if (ret == -ENOENT)
        {
            LOG_WRN("File '%s' not found for deletion. Maybe already deleted?", path_to_delete);
            return err_t::NO_ERROR;
        }
        else
        {
            LOG_ERR("Failed to delete file '%s'. Error %d", path_to_delete, ret);
            return err_t::FILE_SYSTEM_NO_FILES;
        }
    }
    LOG_INF("Successfully deleted file: %s", path_to_delete);

    char latest_file_path[util::max_path_length];
    uint8_t temp_latest_file_seq;           // Temporary to hold uint32_t return from
                                            // find_latest_log_file
    uint8_t latest_file_seq_for_notify = 0; // The uint8_t ID to send in notification

    if (type == RECORD_HARDWARE_ACTIVITY)
    {
        char open_file_path[util::max_path_length] = "";
        if (activity_log_file.filep != nullptr)
        {
            strncpy(open_file_path, activity_file_path, sizeof(open_file_path) - 1);
            open_file_path[sizeof(open_file_path) - 1] = '\0';
        }
        temp_latest_file_seq = (uint8_t)find_latest_log_file(activity_file_prefix, RECORD_HARDWARE_ACTIVITY,
                                                             latest_file_path, open_file_path);
        if (temp_latest_file_seq == (uint32_t)err_t::DATA_ERROR)
        {
            LOG_INF("Activity directory empty or error finding latest file after "
                    "deletion. No file to report.");
            latest_file_seq_for_notify = 0;
            generic_message_t activity_log_info_msg;
            activity_log_info_msg.sender = SENDER_DATA;
            activity_log_info_msg.type = MSG_TYPE_NEW_ACTIVITY_LOG_FILE;
            activity_log_info_msg.data.new_hardware_log_file.file_sequence_id = latest_file_seq_for_notify;
            strncpy(activity_log_info_msg.data.new_hardware_log_file.file_path, "",
                    sizeof(activity_log_info_msg.data.new_hardware_log_file.file_path) - 1);
            activity_log_info_msg.data.new_hardware_log_file
                .file_path[sizeof(activity_log_info_msg.data.new_hardware_log_file.file_path) - 1] = '\0';

            if (k_msgq_put(&bluetooth_msgq, &activity_log_info_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to put empty activity log notification message.");
            }
            else
            {
                LOG_WRN("Sent empty activity log notification.");
            }
        }
        else
        {
            latest_file_seq_for_notify = (uint8_t)temp_latest_file_seq;
            LOG_INF("New latest activity log file: %s (Seq: %u)", latest_file_path, latest_file_seq_for_notify);
            generic_message_t activity_log_info_msg;
            activity_log_info_msg.sender = SENDER_DATA;
            activity_log_info_msg.type = MSG_TYPE_NEW_ACTIVITY_LOG_FILE;
            activity_log_info_msg.data.new_hardware_log_file.file_sequence_id = latest_file_seq_for_notify;
            strncpy(activity_log_info_msg.data.new_hardware_log_file.file_path, latest_file_path,
                    sizeof(activity_log_info_msg.data.new_hardware_log_file.file_path) - 1);
            activity_log_info_msg.data.new_hardware_log_file
                .file_path[sizeof(activity_log_info_msg.data.new_hardware_log_file.file_path) - 1] = '\0';

            if (k_msgq_put(&bluetooth_msgq, &activity_log_info_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to put new latest activity log notification message.");
            }
            else
            {
                LOG_WRN("Sent new latest activity log notification (ID: %u).", latest_file_seq_for_notify);
            }
        }
    }
    return err_t::NO_ERROR;
}

err_t close_directory(fs_dir_t *dirp)
{
    int res = fs_closedir(dirp);
    if (res != 0)
    {
        if (res == -EINVAL)
        {
            LOG_ERR("Failed to close directory (fatal EINVAL)");
            return err_t::FILE_SYSTEM_ERROR;
        }
        LOG_WRN("Failed to close directory (Error: %d)", res);
    }
    return err_t::NO_ERROR;
}

// Find the latest (not open) log file for a given type
int find_latest_log_file(const char *file_prefix, record_type_t record_type, char *file_path,
                         const char *open_file_path)
{
    fs_dir_t dirp{};
    static fs_dirent entry{};

    char dir_path_buf[util::max_path_length]{};
    if (record_type == RECORD_HARDWARE_FOOT_SENSOR || record_type == RECORD_HARDWARE_BHI360 ||
        record_type == RECORD_HARDWARE_ACTIVITY)
    {
        std::strcpy(dir_path_buf, hardware_dir_path);
    }
    else
    {
        LOG_ERR("find_latest_log_file: Unsupported or unimplemented record_type %d", (int)record_type);
        return (int)(err_t::DATA_ERROR);
    }

    fs_dir_t_init(&dirp);

    int res = fs_opendir(&dirp, dir_path_buf);
    if (res != 0)
    {
        LOG_ERR("Error opening dir %s [%d]", dir_path_buf, res);
        return (int)(err_t::DATA_ERROR);
    }

    uint8_t latest_id_found = 0;
    bool found_any_matching_file = false;
    char temp_latest_file_name[util::max_path_length] = {0};

    while (true)
    {
        res = fs_readdir(&dirp, &entry);
        k_msleep(readdir_micro_sleep_ms);

        if (res != 0 || entry.name[0] == '\0')
        {
            break;
        }

        size_t name_len = std::strlen(entry.name);
        size_t prefix_len = std::strlen(file_prefix);
        size_t ext_len = std::strlen(".pb");

        if (entry.type == FS_DIR_ENTRY_FILE && name_len > (prefix_len + ext_len) &&
            std::strncmp(entry.name, file_prefix, prefix_len) == 0 &&
            std::strcmp(entry.name + name_len - ext_len, ".pb") == 0)
        {
            char *num_str_start = entry.name + prefix_len;
            uint8_t current_id = 0;
            if (std::sscanf(num_str_start, "%hhu.pb", &current_id) == 1)
            {
                char candidate_path[util::max_path_length];
                snprintk(candidate_path, util::max_path_length, "%s/%s", dir_path_buf, entry.name);
                if (open_file_path && strcmp(candidate_path, open_file_path) == 0)
                {
                    continue; // skip open file
                }
                if (!found_any_matching_file || current_id > latest_id_found)
                {
                    latest_id_found = current_id;
                    std::strncpy(temp_latest_file_name, entry.name, sizeof(temp_latest_file_name) - 1);
                    temp_latest_file_name[sizeof(temp_latest_file_name) - 1] = '\0';
                    found_any_matching_file = true;
                }
            }
            else
            {
                LOG_WRN("find_latest_log_file: Skipping non-conforming file name '%s' "
                        "for prefix '%s'",
                        entry.name, file_prefix);
            }
        }
    }
    fs_closedir(&dirp);

    if (found_any_matching_file)
    {
        snprintk(file_path, util::max_path_length, "%s/%s", dir_path_buf, temp_latest_file_name);
        LOG_WRN("Latest file found for prefix '%s': %s (ID: %u)", file_prefix, file_path, latest_id_found);
        return latest_id_found;
    }
    else
    {
        LOG_INF("No log files found for prefix '%s' in directory '%s'.", file_prefix, dir_path_buf);
        return (int)err_t::DATA_ERROR;
    }
}

static void strremove(char *str, const char *sub)
{
    char *q, *r;
    if (*sub && (q = r = std::strstr(str, sub)) != NULL)
    {
        char *p;
        size_t len = std::strlen(sub);
        while ((r = std::strstr(p = r + len, sub)) != NULL)
        {
            std::memmove(q, p, r - p);
            q += r - p;
        }
        std::memmove(q, p, std::strlen(p) + 1);
    }
    return;
}

/**
 * @brief Get the filesystem usage percentage
 *
 * @return uint8_t Percentage of filesystem used (0-100), or 255 on error
 */
static uint8_t get_filesystem_usage_percent(void)
{
    struct fs_statvfs stat;
    int ret = fs_statvfs(CONFIG_FILE_SYSTEM_MOUNT, &stat);

    if (ret != 0)
    {
        LOG_ERR("Failed to get filesystem statistics: %d", ret);
        return 255; // Error value
    }

    // Calculate total and used blocks
    uint64_t total_blocks = stat.f_blocks;
    uint64_t free_blocks = stat.f_bfree;
    uint64_t used_blocks = total_blocks - free_blocks;

    if (total_blocks == 0)
    {
        return 255; // Error: no blocks
    }

    // Calculate percentage (avoid overflow)
    uint8_t usage_percent = (uint8_t)((used_blocks * 100) / total_blocks);

    LOG_WRN("Filesystem usage: %u%% (used: %llu blocks, total: %llu blocks)", usage_percent, used_blocks, total_blocks);

    return usage_percent;
}

static bool check_filesystem_space_and_cleanup(void)
{
    uint8_t usage_percent = get_filesystem_usage_percent();

    if (usage_percent == 255)
    {
        // Error getting filesystem stats, assume we have space
        LOG_WRN("Could not get filesystem stats, assuming space available");
        return true;
    }

    if (usage_percent < FILESYSTEM_USAGE_THRESHOLD_PERCENT)
    {
        // Enough space available
        return true;
    }

    LOG_WRN("Filesystem usage at %u%%, starting cleanup (threshold: %u%%)", usage_percent,
            FILESYSTEM_USAGE_THRESHOLD_PERCENT);

    // Keep deleting oldest files until we reach target usage
    int total_files_deleted = 0;
    while (usage_percent > FILESYSTEM_USAGE_TARGET_PERCENT)
    {
        int files_deleted_this_iteration = 0;

        // Try to delete oldest foot sensor file
        err_t err = delete_oldest_log_file_checked(foot_sensor_file_prefix, hardware_dir_path);
        if (err == err_t::NO_ERROR)
        {
            files_deleted_this_iteration++;
            total_files_deleted++;
        }

        // Try to delete oldest BHI360 file
        err = delete_oldest_log_file_checked(bhi360_file_prefix, hardware_dir_path);
        if (err == err_t::NO_ERROR)
        {
            files_deleted_this_iteration++;
            total_files_deleted++;
        }

        // Try to delete oldest activity file
        err = delete_oldest_log_file_checked(activity_file_prefix, hardware_dir_path);
        if (err == err_t::NO_ERROR)
        {
            files_deleted_this_iteration++;
            total_files_deleted++;
        }

        // If we couldn't delete any files in this iteration, break to avoid
        // infinite loop
        if (files_deleted_this_iteration == 0)
        {
            LOG_ERR("Could not delete any more files to free space");
            break;
        }

        // Check usage again
        usage_percent = get_filesystem_usage_percent();
        if (usage_percent == 255)
        {
            // Error getting stats
            break;
        }
    }

    LOG_INF("Filesystem cleanup complete. Deleted %d files, usage now at %u%%", total_files_deleted, usage_percent);

    return (usage_percent < 100); // Return true if we have any space left
}

static int lsdir(const char *path)
{
    int res;
    struct fs_dir_t dirp;
    static struct fs_dirent entry;
    int file_count = 0;
    int dir_count = 0;

    LOG_WRN("Starting directory listing for: %s", path);

    fs_dir_t_init(&dirp);

    /* Verify fs_opendir() */
    res = fs_opendir(&dirp, path);
    if (res)
    {
        LOG_ERR("Error opening dir %s [%d]", path, res);
        return res;
    }

    LOG_WRN("Successfully opened directory: %s", path);
    LOG_WRN("Listing contents:");

    for (;;)
    {
        /* Verify fs_readdir() */
        res = fs_readdir(&dirp, &entry);

        /* entry.name[0] == 0 means end-of-dir */
        if (res || entry.name[0] == 0)
        {
            if (res < 0)
            {
                LOG_ERR("Error reading dir [%d]", res);
            }
            break;
        }

        if (entry.type == FS_DIR_ENTRY_DIR)
        {
            LOG_WRN("  [DIR ] %s", entry.name);
            dir_count++;
        }
        else
        {
            LOG_WRN("  [FILE] %s (size = %zu bytes)", entry.name, entry.size);
            file_count++;
        }
    }

    LOG_WRN("Directory listing complete: %d files, %d directories", file_count, dir_count);

    /* Verify fs_closedir() */
    res = fs_closedir(&dirp);
    if (res != 0)
    {
        LOG_ERR("Error closing directory: %d", res);
    }

    return res;
}

static void fs_sync_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    // Only run if the file is open and a sync is actually pending
    if (activity_log_file.filep != nullptr && atomic_get(&sync_pending))
    {
        LOG_INF("Timer-triggered file sync operation initiated.");
        k_mutex_lock(&activity_file_mutex, K_FOREVER);
        int ret = fs_sync(&activity_log_file);
        if (ret != 0)
        {
            LOG_ERR("Failed to sync activity file: %d", ret);
        }
        else
        {
            LOG_INF("File sync successful.");
        }
        // Clear the pending flag
        atomic_set(&sync_pending, 0);
        k_mutex_unlock(&activity_file_mutex);
    }
}

// App event handler
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        const struct module_state_event *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(app), MODULE_STATE_READY))
        {
            data_init();
        }
        return false;
    }
    else if (is_app_state_event(aeh))
    {
        const struct app_state_event *event = cast_app_state_event(aeh);
        if (event->state == APP_STATE_SHUTDOWN_PREPARING)
        {
            LOG_INF("Received shutdown event. Closing log files.");
            if (atomic_get(&logging_activity_active))
            {
                end_activity_logging();
            }
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);
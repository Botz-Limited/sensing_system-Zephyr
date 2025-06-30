/**
 * @file data.cpp
 * @author
 * @brief
 * @version 1.0.0
 * @date 2025-05-12
 *
 * @copyright Botz Innovation 2025
 *
 */

#define MODULE data

#include <algorithm>
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

#include <app.hpp>
#include <app_fixed_point.hpp>
#include <bhi360_sensor_messages.pb.h>
#include <ble_services.hpp>
#include <data.hpp>
#include <foot_sensor_messages.pb.h>
#include <activity_messages.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <status_codes.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_DATA_MODULE_LOG_LEVEL); // NOLINT

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(littlefs_storage);

// Define File System parameters
#define STORAGE_PARTITION_LABEL storage_ext
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION_LABEL)

static struct fs_mount_t lfs_ext_storage_mnt = {};

static void init_fs_mount(void)
{
    lfs_ext_storage_mnt.type = FS_LITTLEFS;
    lfs_ext_storage_mnt.mnt_point = "/lfs1";
    lfs_ext_storage_mnt.fs_data = &littlefs_storage;
    lfs_ext_storage_mnt.storage_dev = (void *)FLASH_AREA_ID(littlefs_storage);
}

static err_t mount_file_system(struct fs_mount_t *mount);
static err_t littlefs_flash_erase(unsigned int id);
static err_t data_init();
static int lsdir(const char *path);
static void strremove(char *str, const char *sub);

// --- Protobuf-specific write function for foot sensor ---
static err_t write_foot_sensor_protobuf_data(const sensor_data_messages_FootSensorLogMessage *sensor_msg);
static err_t write_bhi360_protobuf_data(const sensor_data_messages_BHI360LogMessage *sensor_msg);
static err_t write_activity_protobuf_data(const sensor_data_messages_ActivityLogMessage *sensor_msg);

static bool encode_foot_sensor_readings_callback(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);
static bool encode_string_callback(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);

err_t cap_and_reindex_log_files(const char *file_prefix, const char *dir_path, uint8_t max_files, uint8_t delete_count);
int close_all_files_in_directory(const char *dir_path);
err_t delete_oldest_log_file_checked(const char *file_prefix, const char *dir_path);
int delete_all_files_in_directory(const char *dir_path);
int find_latest_log_file(const char *file_prefix, record_type_t record_type, char *file_path, const char *open_file_path);

int delete_oldest_log_files(const char *file_prefix, const char *dir_path);
err_t delete_by_type_and_id(record_type_t type, uint8_t id);
err_t type_and_id_to_path(record_type_t type, uint8_t id, char path[]);

static bool check_filesystem_space_and_cleanup(void);
static uint8_t get_filesystem_usage_percent(void);

err_t start_foot_sensor_logging(uint32_t sampling_frequency, const char *fw_version);
err_t end_foot_sensor_logging();

err_t start_bhi360_logging(uint32_t sampling_frequency, const char *fw_version);
err_t end_bhi360_logging();

err_t start_activity_logging(uint32_t sampling_frequency, const char *fw_version);
err_t end_activity_logging();

err_t save_bhi360_calibration_data(const bhi360_calibration_data_t *calib_data);
err_t load_bhi360_calibration_data(uint8_t sensor_type);

// --- Foot Sensor Log File Handle ---
static struct fs_file_t foot_sensor_log_file;
static struct fs_file_t bhi360_log_file;
static struct fs_file_t activity_log_file;

// --- Batch Write Buffers ---
constexpr size_t FLASH_PAGE_SIZE = 256;
constexpr size_t PROTOBUF_ENCODE_BUFFER_SIZE = 64;
static uint8_t foot_sensor_write_buffer[FLASH_PAGE_SIZE];
static size_t foot_sensor_write_buffer_pos = 0;
static uint8_t bhi360_write_buffer[FLASH_PAGE_SIZE];
static size_t bhi360_write_buffer_pos = 0;
static uint8_t activity_write_buffer[FLASH_PAGE_SIZE];
static size_t activity_write_buffer_pos = 0;

// --- Thread safety ---
K_MUTEX_DEFINE(foot_sensor_file_mutex);
K_MUTEX_DEFINE(bhi360_file_mutex);
K_MUTEX_DEFINE(activity_file_mutex);

// --- Timestamp tracking for delta calculation ---
static uint32_t foot_sensor_last_timestamp_ms = 0;
static uint32_t bhi360_last_timestamp_ms = 0;
static uint32_t activity_last_timestamp_ms = 0;
static bool foot_sensor_first_packet = true;
static bool bhi360_first_packet = true;
static bool activity_first_packet = true;

static void flush_foot_sensor_buffer() {
    if (foot_sensor_write_buffer_pos > 0) {
        fs_write(&foot_sensor_log_file, foot_sensor_write_buffer, foot_sensor_write_buffer_pos);
        fs_sync(&foot_sensor_log_file);
        foot_sensor_write_buffer_pos = 0;
    }
}
static void flush_bhi360_buffer() {
    if (bhi360_write_buffer_pos > 0) {
        fs_write(&bhi360_log_file, bhi360_write_buffer, bhi360_write_buffer_pos);
        fs_sync(&bhi360_log_file);
        bhi360_write_buffer_pos = 0;
    }
}
static void flush_activity_buffer() {
    if (activity_write_buffer_pos > 0) {
        fs_write(&activity_log_file, activity_write_buffer, activity_write_buffer_pos);
        fs_sync(&activity_log_file);
        activity_write_buffer_pos = 0;
    }
}

// --- Sequence number counter for foot sensor log ---
static uint8_t next_foot_sensor_file_sequence = 0;
static uint8_t next_bhi360_file_sequence = 0;
static uint8_t next_activity_file_sequence = 0;

// Use atomic operations for thread-safe access to logging flags
static atomic_t logging_foot_active = ATOMIC_INIT(0);
static atomic_t logging_bhi360_active = ATOMIC_INIT(0);
static atomic_t logging_activity_active = ATOMIC_INIT(0);

// Mutex for protecting sequence number generation
K_MUTEX_DEFINE(sequence_number_mutex);

// Mutex for protecting calibration file access
K_MUTEX_DEFINE(calibration_file_mutex);

// Thread
static struct k_thread data_thread_data;
constexpr int data_stack_size = CONFIG_DATA_MODULE_STACK_SIZE;
constexpr int data_priority = CONFIG_DATA_MODULE_PRIORITY;
void proccess_data(void * /*unused*/, void * /*unused*/, void * /*unused*/);

K_THREAD_STACK_DEFINE(data_stack_area, data_stack_size);
k_tid_t data_tid;

// This dir_path is likely for a general-purpose directory listing, not for specific log files.
// Specific log file paths are defined above and declared in data.hpp.
constexpr char dir_path[] = CONFIG_FILE_SYSTEM_MOUNT;

// Flag to track filesystem availability
static bool filesystem_available = false;

// Counter for periodic filesystem mount retry
static uint32_t filesystem_retry_counter = 0;
constexpr uint32_t FILESYSTEM_RETRY_INTERVAL = 60000; // Retry every 60 seconds

// Calibration file paths
constexpr char calibration_dir_path[] = "/lfs1/calibration";
constexpr char bhi360_calib_file_prefix[] = "bhi360_calib_";

// Filesystem usage thresholds
constexpr uint8_t FILESYSTEM_USAGE_THRESHOLD_PERCENT = 80;  // Start cleanup at 80% usage
constexpr uint8_t FILESYSTEM_USAGE_TARGET_PERCENT = 70;     // Clean up until 70% usage

// Counter for periodic filesystem space checks during logging
// static uint32_t filesystem_space_check_counter = 0;  // Currently unused
constexpr uint32_t FILESYSTEM_SPACE_CHECK_INTERVAL = 1000;  // Check every 1000 writes

// Time-based filesystem space checking
static uint32_t last_filesystem_check_time_ms = 0;
constexpr uint32_t FILESYSTEM_CHECK_TIME_INTERVAL_MS = 60000;  // Check every 60 seconds

// Configuration for runtime space checking
constexpr bool ENABLE_RUNTIME_SPACE_CHECKING = true;  // Set to false to disable runtime checks

// Define the file path variables declared as extern in data.hpp
char foot_sensor_file_path[util::max_path_length];
char bhi360_file_path[util::max_path_length];
char activity_file_path[util::max_path_length];

/**
 * @brief Helper to find the next available sequence number for a log file.
 * It scans the directory for files matching the prefix and
 * determines the highest existing sequence number to return the next one.
 *
 * @param dir_path The directory to scan.
 * @param file_prefix The prefix of the files to consider (e.g., "foot_").
 * @return The next sequence number to use (max_found_sequence + 1), or 0 if no files found.
 */
static uint32_t get_next_file_sequence(const char *dir_path_param, const char *file_prefix)
{
    fs_dir_t dirp;
    static fs_dirent entry;
    uint32_t max_seq = 1; // Start from 1
    int res;

    fs_dir_t_init(&dirp);

    res = fs_opendir(&dirp, dir_path_param);
    if (res != 0)
    {
        if (res == -ENOENT)
        {
            LOG_DBG("Directory '%s' does not exist yet for prefix '%s'. Starting sequence from 1.", dir_path_param,
                    file_prefix);
            return 1;
        }
        LOG_ERR("Error opening directory '%s' [%d] for prefix '%s'. Cannot reliably determine next sequence.",
                dir_path_param, res, file_prefix);
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
            else
            {
                LOG_WRN("Skipping non-conforming file name '%s' in %s", entry.name, dir_path_param);
            }
        }
    }
    fs_closedir(&dirp);
    LOG_DBG("Determined next sequence number for files with prefix '%s' in '%s': %u", file_prefix, dir_path_param,
            max_seq);
    return max_seq;
}

/**
 * @brief Initialise the file system
 *
 * @return err_t NO_ERROR on success, or an error code if mounting fails.
 *
 */
static err_t data_init()
{
    // Initialize the fs_mount_t structure
    init_fs_mount();
    
    struct fs_mount_t *mp = &lfs_ext_storage_mnt;
    // Mount the File System
    err_t err = mount_file_system(mp);
    if (err != err_t::NO_ERROR)
    {
        LOG_ERR("Error mounting littlefs: %d", (int)err);
        
        // Report error to Bluetooth module but don't block system
        send_error_to_bluetooth(SENDER_DATA, err_t::FILE_SYSTEM_ERROR, true);
        
        // Set flag to indicate filesystem is not available
        filesystem_available = false;
        
        LOG_WRN("Filesystem not available - system will operate without logging capability");
    }
    else
    {
        LOG_DBG("File System Mounted at [%s]", mp->mnt_point);
        
        // Set flag to indicate filesystem is available
        filesystem_available = true;

        // Cap and reindex log files for foot sensor, BHI360, and activity at startup
        cap_and_reindex_log_files(foot_sensor_file_prefix, hardware_dir_path, 200, 1);
        cap_and_reindex_log_files(bhi360_file_prefix, hardware_dir_path, 200, 1);
        cap_and_reindex_log_files(activity_file_prefix, hardware_dir_path, 200, 1);
        
        // Check filesystem space at startup
        uint8_t usage = get_filesystem_usage_percent();
        if (usage != 255) {
            LOG_INF("Filesystem usage at startup: %u%%", usage);
            if (usage >= FILESYSTEM_USAGE_THRESHOLD_PERCENT) {
                LOG_WRN("Filesystem usage high at startup, performing cleanup");
                check_filesystem_space_and_cleanup();
            }
        }
    }

    // Always create the data thread - it's needed for message handling even without filesystem
    data_tid = k_thread_create(&data_thread_data, data_stack_area, K_THREAD_STACK_SIZEOF(data_stack_area),
                               proccess_data, nullptr, nullptr, nullptr, data_priority, 0, K_NO_WAIT);

    LOG_DBG("Data module initialised, thread created.");
    
    // Always set module to ready state to allow system to continue
    module_set_state(MODULE_STATE_READY);

    return err_t::NO_ERROR;
}

// Currently unused - kept for debugging directory contents
__attribute__((unused))
static int lsdir(const char *path)
{
    int res;
    struct fs_dir_t dirp;
    static struct fs_dirent entry;

    LOG_WRN("Starting Directory List for: %s", path);

    fs_dir_t_init(&dirp);

    res = fs_opendir(&dirp, path);
    if (res != 0)
    {
        LOG_ERR("Error opening directory %s [%d]", path, res);
        return res;
    }

    LOG_INF("Listing directory %s ...", path);
    for (;;)
    {
        res = fs_readdir(&dirp, &entry);

        if (res != 0 || entry.name[0] == '\0')
        {
            if (res < 0)
            {
                LOG_ERR("Error reading directory [%d]", res);
            }
            break;
        }

        if (entry.type == FS_DIR_ENTRY_DIR)
        {
            LOG_INF("[DIR ] %s", entry.name);
        }
        else
        {
            LOG_INF("[FILE] %s (size = %zu)", entry.name, entry.size);
        }
    }

    fs_closedir(&dirp);

    return res;
}

void proccess_data(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{
    k_thread_name_set(data_tid, "data");

    // --- Initial notification of existing log files to BTH module ---
    generic_message_t log_info_msg;
    log_info_msg.sender = SENDER_DATA;

    // For testing only
    // delete_all_files_in_directory("/lfs1/hardware");

    if (filesystem_available) {
        // Notify about latest FOOT sensor log file
        uint8_t current_foot_seq = (uint8_t)get_next_file_sequence(hardware_dir_path, foot_sensor_file_prefix);
        uint8_t latest_foot_log_id_to_report = (current_foot_seq > 0) ? (current_foot_seq - 1) : 0;

        log_info_msg.type = MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE;
        log_info_msg.data.new_hardware_log_file.file_sequence_id = latest_foot_log_id_to_report;
        if (latest_foot_log_id_to_report > 0)
        {
            type_and_id_to_path(RECORD_HARDWARE_FOOT_SENSOR, latest_foot_log_id_to_report,
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
            LOG_ERR("Failed to put initial foot sensor log notification message.");
        }
        else
        {
            LOG_DBG("Initial foot sensor log notification sent (ID: %u).", latest_foot_log_id_to_report);
        }

        // Notify about latest BHI360 log file
        uint8_t current_bhi360_seq = (uint8_t)get_next_file_sequence(hardware_dir_path, bhi360_file_prefix);
        uint8_t latest_bhi360_log_id_to_report = (current_bhi360_seq > 0) ? (current_bhi360_seq - 1) : 0;

        log_info_msg.type = MSG_TYPE_NEW_BHI360_LOG_FILE;
        log_info_msg.data.new_hardware_log_file.file_sequence_id = latest_bhi360_log_id_to_report;
        if (latest_bhi360_log_id_to_report > 0)
        {
            type_and_id_to_path(RECORD_HARDWARE_BHI360, latest_bhi360_log_id_to_report,
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
            LOG_ERR("Failed to put initial BHI360 log notification message.");
        }
        else
        {
            LOG_DBG("Initial BHI360 log notification sent (ID: %u).", latest_bhi360_log_id_to_report);
        }

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
            LOG_DBG("Initial activity log notification sent (ID: %u).", latest_activity_log_id_to_report);
        }
    } else {
        // Filesystem not available - send empty notifications
        LOG_WRN("Filesystem not available - sending empty log notifications");
        
        // Send empty foot sensor log notification
        log_info_msg.type = MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE;
        log_info_msg.data.new_hardware_log_file.file_sequence_id = 0;
        strncpy(log_info_msg.data.new_hardware_log_file.file_path, "",
                sizeof(log_info_msg.data.new_hardware_log_file.file_path) - 1);
        log_info_msg.data.new_hardware_log_file
            .file_path[sizeof(log_info_msg.data.new_hardware_log_file.file_path) - 1] = '\0';
        k_msgq_put(&bluetooth_msgq, &log_info_msg, K_NO_WAIT);
        
        // Send empty BHI360 log notification
        log_info_msg.type = MSG_TYPE_NEW_BHI360_LOG_FILE;
        log_info_msg.data.new_hardware_log_file.file_sequence_id = 0;
        strncpy(log_info_msg.data.new_hardware_log_file.file_path, "",
                sizeof(log_info_msg.data.new_hardware_log_file.file_path) - 1);
        log_info_msg.data.new_hardware_log_file
            .file_path[sizeof(log_info_msg.data.new_hardware_log_file.file_path) - 1] = '\0';
        k_msgq_put(&bluetooth_msgq, &log_info_msg, K_NO_WAIT);
        
        // Send empty activity log notification
        log_info_msg.type = MSG_TYPE_NEW_ACTIVITY_LOG_FILE;
        log_info_msg.data.new_hardware_log_file.file_sequence_id = 0;
        strncpy(log_info_msg.data.new_hardware_log_file.file_path, "",
                sizeof(log_info_msg.data.new_hardware_log_file.file_path) - 1);
        log_info_msg.data.new_hardware_log_file
            .file_path[sizeof(log_info_msg.data.new_hardware_log_file.file_path) - 1] = '\0';
        k_msgq_put(&bluetooth_msgq, &log_info_msg, K_NO_WAIT);
    }

    generic_message_t msg;
    int ret;

    while (true)
    {
        // Use timeout instead of K_FOREVER to allow periodic filesystem retry
        ret = k_msgq_get(&data_msgq, &msg, K_SECONDS(1));

        // Periodic filesystem mount retry if not available
        if (!filesystem_available) {
            filesystem_retry_counter++;
            if (filesystem_retry_counter >= (FILESYSTEM_RETRY_INTERVAL / 1000)) {
                filesystem_retry_counter = 0;
                LOG_INF("Retrying filesystem mount...");
                
                struct fs_mount_t *mp = &lfs_ext_storage_mnt;
                err_t err = mount_file_system(mp);
                if (err == err_t::NO_ERROR) {
                    LOG_INF("Filesystem mount successful on retry!");
                    filesystem_available = true;
                    
                    // Clear the error status
                    send_error_to_bluetooth(SENDER_DATA, err_t::FILE_SYSTEM_ERROR, false);
                    
                    // Perform initial setup
                    cap_and_reindex_log_files(foot_sensor_file_prefix, hardware_dir_path, 200, 1);
                    cap_and_reindex_log_files(bhi360_file_prefix, hardware_dir_path, 200, 1);
                    cap_and_reindex_log_files(activity_file_prefix, hardware_dir_path, 200, 1);
                }
            }
        }

        if (ret == 0)
        {
            LOG_DBG("Processing message from %s, type: %d", get_sender_name(msg.sender), msg.type);

            switch (msg.type)
            {
                case MSG_TYPE_FOOT_SAMPLES: {
                const foot_samples_t *foot_data = &msg.data.foot_samples;
                
                // Data module only handles logging, not real-time transmission
                // The foot sensor module should send data to the bluetooth module directly
                    
                    // Only attempt logging if filesystem is available and logging is active
                    if (filesystem_available && atomic_get(&logging_foot_active) == 1)
                    {
                        // Create FootSensorLogMessage
                        sensor_data_messages_FootSensorLogMessage foot_log_msg =
                            sensor_data_messages_FootSensorLogMessage_init_default;
                        foot_log_msg.which_payload = sensor_data_messages_FootSensorLogMessage_foot_sensor_tag;

                        // Populate FootSensorData fields
                        foot_log_msg.payload.foot_sensor.readings.funcs.encode = &encode_foot_sensor_readings_callback;
                        foot_log_msg.payload.foot_sensor.readings.arg = (void *)foot_data;

                        // Calculate delta time
                        uint32_t current_time_ms = (uint32_t)k_uptime_get();
                        uint16_t delta_ms = 0;
                        
                        if (foot_sensor_first_packet) {
                            delta_ms = 0;  // First packet after header has delta 0
                            foot_sensor_first_packet = false;
                        } else {
                            uint32_t time_diff = current_time_ms - foot_sensor_last_timestamp_ms;
                            // Cap at 65535ms (about 65 seconds) to fit in 2 bytes
                            delta_ms = (time_diff > 65535) ? 65535 : (uint16_t)time_diff;
                        }
                        
                        foot_sensor_last_timestamp_ms = current_time_ms;
                        foot_log_msg.payload.foot_sensor.delta_ms = delta_ms;

                        err_t write_status = write_foot_sensor_protobuf_data(&foot_log_msg);
                        if (write_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to write foot sensor data to file: %d", (int)write_status);
                        }
                    }
                    else if (!filesystem_available && atomic_get(&logging_foot_active) == 1)
                    {
                        LOG_DBG("Foot sensor logging requested but filesystem not available");
                    }
                    break;
                }

                case MSG_TYPE_BHI360_LOG_RECORD: {
                    const bhi360_log_record_t *record = &msg.data.bhi360_log_record;
                    
                    // Only attempt logging if filesystem is available and logging is active
                    if (filesystem_available && atomic_get(&logging_bhi360_active) == 1)
                    {
                         LOG_INF("bhi360 sample data");
                        sensor_data_messages_BHI360LogMessage bhi360_log_msg =
                            sensor_data_messages_BHI360LogMessage_init_default;
                        bhi360_log_msg.which_payload = sensor_data_messages_BHI360LogMessage_bhi360_log_record_tag;

                        // Populate BHI360LogRecord fields with fixed-point conversion
                        // Quaternion components scaled by 10000
                        bhi360_log_msg.payload.bhi360_log_record.quat_x = float_to_fixed16(record->quat_x, FixedPoint::QUAT_SCALE);
                        bhi360_log_msg.payload.bhi360_log_record.quat_y = float_to_fixed16(record->quat_y, FixedPoint::QUAT_SCALE);
                        bhi360_log_msg.payload.bhi360_log_record.quat_z = float_to_fixed16(record->quat_z, FixedPoint::QUAT_SCALE);
                        bhi360_log_msg.payload.bhi360_log_record.quat_w = float_to_fixed16(record->quat_w, FixedPoint::QUAT_SCALE);
                        // Quaternion accuracy scaled by 100
                        bhi360_log_msg.payload.bhi360_log_record.quat_accuracy = static_cast<uint8_t>(record->quat_accuracy * FixedPoint::ACCURACY_SCALE);
                        // Linear acceleration scaled by 1000 (mm/s²)
                        bhi360_log_msg.payload.bhi360_log_record.lacc_x = float_to_fixed16(record->lacc_x, FixedPoint::ACCEL_SCALE);
                        bhi360_log_msg.payload.bhi360_log_record.lacc_y = float_to_fixed16(record->lacc_y, FixedPoint::ACCEL_SCALE);
                        bhi360_log_msg.payload.bhi360_log_record.lacc_z = float_to_fixed16(record->lacc_z, FixedPoint::ACCEL_SCALE);
                        // Gyroscope scaled by 10000 (0.0001 rad/s)
                        bhi360_log_msg.payload.bhi360_log_record.gyro_x = float_to_fixed16(record->gyro_x, FixedPoint::GYRO_SCALE);
                        bhi360_log_msg.payload.bhi360_log_record.gyro_y = float_to_fixed16(record->gyro_y, FixedPoint::GYRO_SCALE);
                        bhi360_log_msg.payload.bhi360_log_record.gyro_z = float_to_fixed16(record->gyro_z, FixedPoint::GYRO_SCALE);
                        // Step count remains as is
                        bhi360_log_msg.payload.bhi360_log_record.step_count = record->step_count;

                        // Calculate delta time
                        uint32_t current_time_ms = (uint32_t)k_uptime_get();
                        uint16_t delta_ms = 0;
                        
                        if (bhi360_first_packet) {
                            delta_ms = 0;  // First packet after header has delta 0
                            bhi360_first_packet = false;
                        } else {
                            uint32_t time_diff = current_time_ms - bhi360_last_timestamp_ms;
                            // Cap at 65535ms (about 65 seconds) to fit in 2 bytes
                            delta_ms = (time_diff > 65535) ? 65535 : (uint16_t)time_diff;
                        }
                        
                        bhi360_last_timestamp_ms = current_time_ms;
                        bhi360_log_msg.payload.bhi360_log_record.delta_ms = delta_ms;

                        err_t write_status = write_bhi360_protobuf_data(&bhi360_log_msg);
                        if (write_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to write BHI360 log record to file: %d", (int)write_status);
                        }
                        // No BLE notification for BHI360 data yet.
                    }
                    else if (!filesystem_available && atomic_get(&logging_bhi360_active) == 1)
                    {
                        LOG_DBG("BHI360 logging requested but filesystem not available");
                    }
                    break;
                }

                case MSG_TYPE_BHI360_STEP_COUNT: {
                    const bhi360_step_count_t *step_count_data = &msg.data.bhi360_step_count;
                    
                    // Only attempt logging if filesystem is available and logging is active
                    if (filesystem_available && atomic_get(&logging_bhi360_active) == 1)
                    {
                        sensor_data_messages_BHI360LogMessage bhi360_log_msg =
                            sensor_data_messages_BHI360LogMessage_init_default;
                        bhi360_log_msg.which_payload = sensor_data_messages_BHI360LogMessage_bhi360_step_counter_tag;

                        // Populate BHI360 Step Count data fields
                        bhi360_log_msg.payload.bhi360_step_counter.step_count = step_count_data->step_count;
                        bhi360_log_msg.payload.bhi360_step_counter.activity_duration_s =
                            step_count_data->activity_duration_s;

                        err_t write_status = write_bhi360_protobuf_data(&bhi360_log_msg);
                        if (write_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to write BHI360 Step Count data to file: %d", (int)write_status);
                        }
                        // No BLE notification for BHI360 data yet
                    }
                    else if (!filesystem_available && atomic_get(&logging_bhi360_active) == 1)
                    {
                        LOG_DBG("BHI360 step count logging requested but filesystem not available");
                    }
                    break;
                }

                case MSG_TYPE_ACTIVITY_STEP_COUNT: {
                    const activity_step_count_t *activity_data = &msg.data.activity_step_count;
                    
                    // Only attempt logging if filesystem is available and logging is active
                    if (filesystem_available && atomic_get(&logging_activity_active) == 1)
                    {
                        sensor_data_messages_ActivityLogMessage activity_log_msg =
                            sensor_data_messages_ActivityLogMessage_init_default;
                        activity_log_msg.which_payload = sensor_data_messages_ActivityLogMessage_activity_data_tag;

                        // Populate Activity data fields with separate left/right counts
                        activity_log_msg.payload.activity_data.left_step_count = activity_data->left_step_count;
                        activity_log_msg.payload.activity_data.right_step_count = activity_data->right_step_count;

                        // Calculate delta time
                        uint32_t current_time_ms = (uint32_t)k_uptime_get();
                        uint16_t delta_ms = 0;
                        
                        if (activity_first_packet) {
                            delta_ms = 0;  // First packet after header has delta 0
                            activity_first_packet = false;
                        } else {
                            uint32_t time_diff = current_time_ms - activity_last_timestamp_ms;
                            // Cap at 65535ms (about 65 seconds) to fit in 2 bytes
                            delta_ms = (time_diff > 65535) ? 65535 : (uint16_t)time_diff;
                        }
                        
                        activity_last_timestamp_ms = current_time_ms;
                        activity_log_msg.payload.activity_data.delta_ms = delta_ms;

                        err_t write_status = write_activity_protobuf_data(&activity_log_msg);
                        if (write_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to write activity data to file: %d", (int)write_status);
                        }
                    }
                    else if (!filesystem_available && atomic_get(&logging_activity_active) == 1)
                    {
                        LOG_DBG("Activity logging requested but filesystem not available");
                    }
                    break;
                }

                case MSG_TYPE_COMMAND: {
                    LOG_INF("Received generic command: %s", msg.data.command_str);
                    if ((strcmp(msg.data.command_str, "START_LOGGING_FOOT_SENSOR") == 0) && (atomic_get(&logging_foot_active) == 0))
                    {
                        if (!filesystem_available) {
                            LOG_WRN("Command: Cannot start foot sensor logging - filesystem not available");
                            // Still set the flag so we know logging was requested
                            atomic_set(&logging_foot_active, 1);
                        } else {
                            LOG_INF("Command: Starting foot sensor logging.");
                            err_t foot_status = start_foot_sensor_logging(msg.sampling_frequency, msg.fw_version);
                            if (foot_status == err_t::NO_ERROR)
                            {
                                atomic_set(&logging_foot_active, 1);
                            }
                            else
                            {
                                LOG_ERR("Failed to start foot sensor logging from command: %d", (int)foot_status);
                                atomic_set(&logging_foot_active, 0);
                            }
                        }
                    }
                    else if ((strcmp(msg.data.command_str, "STOP_LOGGING_FOOT_SENSOR") == 0) && (atomic_get(&logging_foot_active) == 1))
                    {
                        LOG_INF("Command: Stopping foot sensor logging.");
                        atomic_set(&logging_foot_active, 0);
                        if (filesystem_available) {
                            err_t foot_status = end_foot_sensor_logging();
                            if (foot_status != err_t::NO_ERROR)
                            {
                                LOG_ERR("Failed to stop foot sensor logging from command: %d", (int)foot_status);
                            }
                            // Add small delay to allow bluetooth queue to process
                            k_msleep(10);
                        }
                    }
                    else if ((strcmp(msg.data.command_str, "START_LOGGING_BHI360") == 0) && (atomic_get(&logging_bhi360_active) == 0))
                    {
                        if (!filesystem_available) {
                            LOG_WRN("Command: Cannot start BHI360 logging - filesystem not available");
                            // Still set the flag so we know logging was requested
                            atomic_set(&logging_bhi360_active, 1);
                        } else {
                            LOG_INF("Command: Starting BHI360 logging.");
                            err_t bhi360_status = start_bhi360_logging(msg.sampling_frequency, msg.fw_version);
                            if (bhi360_status == err_t::NO_ERROR)
                            {
                                atomic_set(&logging_bhi360_active, 1);
                            }
                            else
                            {
                                LOG_ERR("Failed to start BHI360 logging from command: %d", (int)bhi360_status);
                                atomic_set(&logging_bhi360_active, 0);
                            }
                        }
                    }
                    else if ((strcmp(msg.data.command_str, "STOP_LOGGING_BHI360") == 0) && (atomic_get(&logging_bhi360_active) == 1))
                    {
                        LOG_INF("Command: Stopping BHI360 logging.");
                        atomic_set(&logging_bhi360_active, 0);
                        if (filesystem_available) {
                            err_t bhi360_status = end_bhi360_logging();
                            if (bhi360_status != err_t::NO_ERROR)
                            {
                                LOG_ERR("Failed to stop BHI360 logging from command: %d", (int)bhi360_status);
                            }
                            // Add small delay to allow bluetooth queue to process
                            k_msleep(10);
                        }
                    }
                    else if ((strcmp(msg.data.command_str, "START_LOGGING_ACTIVITY") == 0) && (atomic_get(&logging_activity_active) == 0))
                    {
                        if (!filesystem_available) {
                            LOG_WRN("Command: Cannot start activity logging - filesystem not available");
                            // Still set the flag so we know logging was requested
                            atomic_set(&logging_activity_active, 1);
                        } else {
                            LOG_INF("Command: Starting activity logging.");
                            err_t activity_status = start_activity_logging(msg.sampling_frequency, msg.fw_version);
                            if (activity_status == err_t::NO_ERROR)
                            {
                                atomic_set(&logging_activity_active, 1);
                            }
                            else
                            {
                                LOG_ERR("Failed to start activity logging from command: %d", (int)activity_status);
                                atomic_set(&logging_activity_active, 0);
                            }
                        }
                    }
                    else if ((strcmp(msg.data.command_str, "STOP_LOGGING_ACTIVITY") == 0) && (atomic_get(&logging_activity_active) == 1))
                    {
                        LOG_INF("Command: Stopping activity logging.");
                        atomic_set(&logging_activity_active, 0);
                        if (filesystem_available) {
                            err_t activity_status = end_activity_logging();
                            if (activity_status != err_t::NO_ERROR)
                            {
                                LOG_ERR("Failed to stop activity logging from command: %d", (int)activity_status);
                            }
                            // Add small delay to allow bluetooth queue to process
                            k_msleep(10);
                        }
                    }
                    else if (strcmp(msg.data.command_str, "STOP_LOGGING") == 0)
                    {
                        // Handle STOP_LOGGING command - stop all active logging
                        LOG_INF("Command: Stopping all logging (STOP_LOGGING).");
                        err_t foot_status = err_t::NO_ERROR;
                        err_t bhi360_status = err_t::NO_ERROR;
                        err_t activity_status = err_t::NO_ERROR;
                        
                        if (atomic_get(&logging_foot_active) == 1) {
                            atomic_set(&logging_foot_active, 0);
                            if (filesystem_available) {
                                foot_status = end_foot_sensor_logging();
                            }
                        }
                        if (atomic_get(&logging_bhi360_active) == 1) {
                            atomic_set(&logging_bhi360_active, 0);
                            if (filesystem_available) {
                                bhi360_status = end_bhi360_logging();
                            }
                        }
                        if (atomic_get(&logging_activity_active) == 1) {
                            atomic_set(&logging_activity_active, 0);
                            if (filesystem_available) {
                                activity_status = end_activity_logging();
                            }
                        }
                        
                        if (filesystem_available) {
                            if (foot_status != err_t::NO_ERROR)
                            {
                                LOG_ERR("Failed to stop foot sensor logging from command: %d", (int)foot_status);
                            }
                            if (bhi360_status != err_t::NO_ERROR)
                            {
                                LOG_ERR("Failed to stop BHI360 logging from command: %d", (int)bhi360_status);
                            }
                            if (activity_status != err_t::NO_ERROR)
                            {
                                LOG_ERR("Failed to stop activity logging from command: %d", (int)activity_status);
                            }
                        }
                    }
                    else
                    {
                        // Handle any other generic string commands here, or log as unknown
                        LOG_WRN("Unknown generic string command received: %s", msg.data.command_str);
                    }
                    break;
                }

                case MSG_TYPE_DELETE_FOOT_LOG: {
                    LOG_INF("Received DELETE FOOT LOG command for ID: %u", msg.data.delete_cmd.id);
                    if (!filesystem_available) {
                        LOG_WRN("Cannot delete foot log - filesystem not available");
                    } else {
                        err_t delete_status =
                            delete_by_type_and_id(RECORD_HARDWARE_FOOT_SENSOR, (uint8_t)msg.data.delete_cmd.id);
                        if (delete_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to delete foot sensor log file with ID %u: %d", msg.data.delete_cmd.id,
                                    (int)delete_status);
                        }
                        else
                        {
                            LOG_INF("Successfully deleted foot sensor log file with ID %u", msg.data.delete_cmd.id);
                        }
                    }
                    break;
                }

                case MSG_TYPE_DELETE_BHI360_LOG: { // New: Handle BHI360 delete messages
                    LOG_INF("Received DELETE BHI360 LOG command for ID: %u", msg.data.delete_cmd.id);
                    if (!filesystem_available) {
                        LOG_WRN("Cannot delete BHI360 log - filesystem not available");
                    } else {
                        err_t delete_status =
                            delete_by_type_and_id(RECORD_HARDWARE_BHI360, (uint8_t)msg.data.delete_cmd.id);
                        if (delete_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to delete BHI360 log file with ID %u: %d", msg.data.delete_cmd.id,
                                    (int)delete_status);
                        }
                        else
                        {
                            LOG_INF("Successfully deleted BHI360 log file with ID %u", msg.data.delete_cmd.id);
                        }
                    }
                    break;
                }

                case MSG_TYPE_DELETE_ACTIVITY_LOG: {
                    LOG_INF("Received DELETE ACTIVITY LOG command for ID: %u", msg.data.delete_cmd.id);
                    if (!filesystem_available) {
                        LOG_WRN("Cannot delete activity log - filesystem not available");
                    } else {
                        err_t delete_status =
                            delete_by_type_and_id(RECORD_HARDWARE_ACTIVITY, (uint8_t)msg.data.delete_cmd.id);
                        if (delete_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to delete activity log file with ID %u: %d", msg.data.delete_cmd.id,
                                    (int)delete_status);
                        }
                        else
                        {
                            LOG_INF("Successfully deleted activity log file with ID %u", msg.data.delete_cmd.id);
                        }
                    }
                    break;
                }

                case MSG_TYPE_SAVE_BHI360_CALIBRATION: {
                    LOG_INF("Received SAVE BHI360 CALIBRATION command for sensor type: %u", 
                            msg.data.bhi360_calibration.sensor_type);
                    if (!filesystem_available) {
                        LOG_WRN("Cannot save calibration - filesystem not available");
                    } else {
                        err_t save_status = save_bhi360_calibration_data(&msg.data.bhi360_calibration);
                        if (save_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to save BHI360 calibration data: %d", (int)save_status);
                        }
                        else
                        {
                            LOG_INF("Successfully saved BHI360 calibration data");
                        }
                    }
                    break;
                }

                case MSG_TYPE_LOAD_BHI360_CALIBRATION: {
                    LOG_INF("Received LOAD BHI360 CALIBRATION command for sensor type: %u", 
                            msg.data.bhi360_calibration.sensor_type);
                    if (!filesystem_available) {
                        LOG_WRN("Cannot load calibration - filesystem not available");
                    } else {
                        err_t load_status = load_bhi360_calibration_data(msg.data.bhi360_calibration.sensor_type);
                        if (load_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to load BHI360 calibration data: %d", (int)load_status);
                        }
                        else
                        {
                            LOG_INF("Successfully loaded BHI360 calibration data");
                        }
                    }
                    break;
                }

                case MSG_TYPE_REQUEST_BHI360_CALIBRATION: {
                    LOG_INF("Received REQUEST BHI360 CALIBRATION from %s", get_sender_name(msg.sender));
                    if (!filesystem_available) {
                        LOG_WRN("Cannot load calibration - filesystem not available");
                    } else {
                        // Load and send calibration data for both accel and gyro
                        for (uint8_t sensor_type = 0; sensor_type <= 1; sensor_type++) {
                            err_t load_status = load_bhi360_calibration_data(sensor_type);
                            if (load_status != err_t::NO_ERROR && load_status != err_t::FILE_SYSTEM_NO_FILES) {
                                LOG_ERR("Failed to load BHI360 calibration data for sensor %u: %d", 
                                        sensor_type, (int)load_status);
                            }
                        }
                    }
                    break;
                }

                default:
                    LOG_WRN("Unknown message type received: %d", msg.type);
                    break;
            }
        }
        else if (ret == -EAGAIN)
        {
            // Timeout occurred, this is expected for periodic checks
            // Continue to next iteration
        }
        else
        {
            LOG_ERR("Failed to get message from queue: %d", ret);
        }
    }
}

err_t mount_file_system(struct fs_mount_t *mount)
{
    const struct device *flash_device = DEVICE_DT_GET(DT_NODELABEL(mx25r64));

    if (!device_is_ready(flash_device))
    {
        LOG_ERR("External flash device isn't ready");
        return err_t::DATA_ERROR;
    }

    // Conditionally erase flash if config is enabled
    if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
        err_t rc = littlefs_flash_erase((uintptr_t)mount->storage_dev);
        if (rc != err_t::NO_ERROR) {
            return err_t::DATA_ERROR;
        }
    }

    int ret = fs_mount(mount);
    if (ret != 0)
    {
        LOG_ERR("FS mount failed: %d", ret);
        return err_t::DATA_ERROR;
    }

    // Use hardware_dir_path from data.hpp
    //For testing, list files in the directory
   //   int err_ls = lsdir(hardware_dir_path);

    return err_t::NO_ERROR;
}

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

int delete_oldest_log_files(const char *file_prefix, const char *dir_path_param)
{
    uint8_t log_id = 0;
    int res = 0;
    fs_dir_t dirp;
    static fs_dirent entry;
    char temp_name[util::max_path_length];

    fs_dir_t_init(&dirp);

    res = fs_opendir(&dirp, dir_path_param);
    if (res != 0) // Check if open failed
    {
        if (res == -ENOENT)
        { // If directory doesn't exist, it's not an error for this function
            LOG_DBG("Directory '%s' does not exist for deletion check.", dir_path_param);
            return 0; // No files to delete
        }
        LOG_ERR("Error opening dir %s [%d] for oldest file deletion.", dir_path_param, res);
        return 0;
    }

    uint16_t last_file_id = 0;
    uint16_t trailing_file_id = 0;
    bool trailing_file_id_active = false;

    for (;;)
    {
        res = fs_readdir(&dirp, &entry);
        k_msleep(readdir_micro_sleep_ms); // Keep small sleep to avoid tight loop

        if (res != 0 || entry.name[0] == '\0')
        {
            if (res < 0)
            {
                LOG_WRN("Error or no more files in this directory [%d]", res);
            }
            break;
        }

        if (last_file_id >= 100 && !trailing_file_id_active)
        {
            trailing_file_id_active = true;
            trailing_file_id = 0;
        }

        last_file_id++;

        if (trailing_file_id_active)
        {
            trailing_file_id++;
        }

        LOG_DBG("trailing_id: %u, last_id: %u", trailing_file_id, last_file_id);

        // Only process files that match the prefix
        if (std::strncmp(entry.name, file_prefix, strlen(file_prefix)) == 0)
        {
            std::strncpy(temp_name, entry.name, sizeof(temp_name));
            temp_name[sizeof(temp_name) - 1] = '\0';
            strremove(temp_name, file_prefix);
            std::sscanf(temp_name, "%hhu", &log_id);
        }
    }
    fs_closedir(&dirp);

    if (trailing_file_id > 0)
    {
        LOG_INF("Deleting %u oldest files for prefix '%s'...", trailing_file_id, file_prefix);
        res = fs_opendir(&dirp, dir_path_param);
        if (res != 0)
        {
            LOG_ERR("Error re-opening dir %s for deletion [%d]", dir_path_param, res);
            return 0;
        }

        for (uint16_t i = 0; i < trailing_file_id; i++)
        {
            res = fs_readdir(&dirp, &entry);
            k_msleep(readdir_micro_sleep_ms);

            if (res != 0 || entry.name[0] == '\0')
            {
                LOG_WRN("No more files to delete or error during readdir [%d]", res);
                break;
            }

            char file_path[util::max_path_length];
            snprintk(file_path, sizeof(file_path), "%s/%s", dir_path_param, entry.name);
            int unlink_res = fs_unlink(file_path);
            if (unlink_res == 0)
            {
                LOG_DBG("Unlinking %s", file_path);
            }
            else
            {
                LOG_ERR("Failed to unlink %s: %d", file_path, unlink_res);
            }
        }
        fs_closedir(&dirp); // Close directory after deletion loop
    }

    return 0;
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
int find_latest_log_file(const char *file_prefix, record_type_t record_type, char *file_path, const char *open_file_path)
{
    fs_dir_t dirp{};
    static fs_dirent entry{};

    char dir_path_buf[util::max_path_length]{};
    if (record_type == RECORD_HARDWARE_FOOT_SENSOR || record_type == RECORD_HARDWARE_BHI360 || record_type == RECORD_HARDWARE_ACTIVITY)
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

        if (entry.type == FS_DIR_ENTRY_FILE &&
            name_len > (prefix_len + ext_len) &&
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
                LOG_WRN("find_latest_log_file: Skipping non-conforming file name '%s' for prefix '%s'", entry.name, file_prefix);
            }
        }
    }
    fs_closedir(&dirp);

    if (found_any_matching_file)
    {
        snprintk(file_path, util::max_path_length, "%s/%s", dir_path_buf, temp_latest_file_name);
        LOG_DBG("Latest file found for prefix '%s': %s (ID: %u)", file_prefix, file_path, latest_id_found);
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
    
    if (ret != 0) {
        LOG_ERR("Failed to get filesystem statistics: %d", ret);
        return 255; // Error value
    }
    
    // Calculate total and used blocks
    uint64_t total_blocks = stat.f_blocks;
    uint64_t free_blocks = stat.f_bfree;
    uint64_t used_blocks = total_blocks - free_blocks;
    
    if (total_blocks == 0) {
        return 255; // Error: no blocks
    }
    
    // Calculate percentage (avoid overflow)
    uint8_t usage_percent = (uint8_t)((used_blocks * 100) / total_blocks);
    
    LOG_DBG("Filesystem usage: %u%% (used: %llu blocks, total: %llu blocks)", 
            usage_percent, used_blocks, total_blocks);
    
    return usage_percent;
}

/**
 * @brief Check filesystem space and cleanup if needed
 * 
 * @return true if there is enough space (or cleanup was successful), false otherwise
 */
static bool check_filesystem_space_and_cleanup(void)
{
    uint8_t usage_percent = get_filesystem_usage_percent();
    
    if (usage_percent == 255) {
        // Error getting filesystem stats, assume we have space
        LOG_WRN("Could not get filesystem stats, assuming space available");
        return true;
    }
    
    if (usage_percent < FILESYSTEM_USAGE_THRESHOLD_PERCENT) {
        // Enough space available
        return true;
    }
    
    LOG_WRN("Filesystem usage at %u%%, starting cleanup (threshold: %u%%)", 
            usage_percent, FILESYSTEM_USAGE_THRESHOLD_PERCENT);
    
    // Keep deleting oldest files until we reach target usage
    int total_files_deleted = 0;
    while (usage_percent > FILESYSTEM_USAGE_TARGET_PERCENT) {
        int files_deleted_this_iteration = 0;
        
        // Try to delete oldest foot sensor file
        err_t err = delete_oldest_log_file_checked(foot_sensor_file_prefix, hardware_dir_path);
        if (err == err_t::NO_ERROR) {
            files_deleted_this_iteration++;
            total_files_deleted++;
        }
        
        // Try to delete oldest BHI360 file
        err = delete_oldest_log_file_checked(bhi360_file_prefix, hardware_dir_path);
        if (err == err_t::NO_ERROR) {
            files_deleted_this_iteration++;
            total_files_deleted++;
        }
        
        // Try to delete oldest activity file
        err = delete_oldest_log_file_checked(activity_file_prefix, hardware_dir_path);
        if (err == err_t::NO_ERROR) {
            files_deleted_this_iteration++;
            total_files_deleted++;
        }
        
        // If we couldn't delete any files in this iteration, break to avoid infinite loop
        if (files_deleted_this_iteration == 0) {
            LOG_ERR("Could not delete any more files to free space");
            break;
        }
        
        // Check usage again
        usage_percent = get_filesystem_usage_percent();
        if (usage_percent == 255) {
            // Error getting stats
            break;
        }
    }
    
    LOG_INF("Filesystem cleanup complete. Deleted %d files, usage now at %u%%", 
            total_files_deleted, usage_percent);
    
    return (usage_percent < 100);  // Return true if we have any space left
}

typedef struct
{
    const char *str;
} string_callback_arg_t;

static bool encode_string_callback(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
    const string_callback_arg_t *s_arg = (const string_callback_arg_t *)(*arg);
    if (s_arg->str == NULL)
    {
        return true;
    }
    if (!pb_encode_tag_for_field(stream, field))
    {
        return false;
    }
    return pb_encode_string(stream, (pb_byte_t *)s_arg->str, strlen(s_arg->str));
}

/**
 * @brief Starts the data logging process for the foot sensor.
 * Opens a new log file with a sequence number and writes the SensingData header.
 *
 * @return err_t NO_ERROR on success, or an error code on failure.
 */
err_t start_foot_sensor_logging(uint32_t sampling_frequency, const char *fw_version)
{
    err_t status = err_t::NO_ERROR;
    
    // Lock mutex for thread safety
    k_mutex_lock(&foot_sensor_file_mutex, K_FOREVER);

    // Reset timestamp tracking for new logging session
    foot_sensor_last_timestamp_ms = (uint32_t)k_uptime_get();
    foot_sensor_first_packet = true;

    // --- Create hardware directory if it doesn't exist ---
    int ret_mkdir = fs_mkdir(hardware_dir_path);
    if (ret_mkdir != 0 && ret_mkdir != -EEXIST)
    {
        LOG_INF("FAIL: fs_mkdir %s: %d", hardware_dir_path, ret_mkdir);
        k_mutex_unlock(&foot_sensor_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // --- Check filesystem space and cleanup if needed ---
    if (!check_filesystem_space_and_cleanup()) {
        LOG_ERR("Insufficient filesystem space for new foot sensor log");
        k_mutex_unlock(&foot_sensor_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // --- Initialize Foot Sensor Log File ---
    // Protect sequence number generation with mutex
    k_mutex_lock(&sequence_number_mutex, K_FOREVER);
    next_foot_sensor_file_sequence = (uint8_t)get_next_file_sequence(hardware_dir_path, foot_sensor_file_prefix);
    if (next_foot_sensor_file_sequence == 0) {
        next_foot_sensor_file_sequence = 1;
    }
    k_mutex_unlock(&sequence_number_mutex);
    snprintk(foot_sensor_file_path, sizeof(foot_sensor_file_path), "%s/%s%03u.pb", hardware_dir_path,
             foot_sensor_file_prefix, next_foot_sensor_file_sequence);
    fs_file_t_init(&foot_sensor_log_file);

    int ret_fs_open = fs_open(&foot_sensor_log_file, foot_sensor_file_path, FS_O_CREATE | FS_O_RDWR | FS_O_APPEND);
    if (ret_fs_open != 0)
    {
        LOG_ERR("FAIL: open foot sensor log %s: %d", foot_sensor_file_path, ret_fs_open);
        status = err_t::FILE_SYSTEM_ERROR;
        
        // Report error to Bluetooth module
        send_error_to_bluetooth(SENDER_DATA, err_t::FILE_SYSTEM_ERROR, true);
    }
    else
    {
        LOG_INF("Opened new foot sensor log file: %s", foot_sensor_file_path);

        uint8_t buffer[PROTOBUF_ENCODE_BUFFER_SIZE]; // Sufficient for SensingData message
        sensor_data_messages_FootSensorLogMessage header_msg = sensor_data_messages_FootSensorLogMessage_init_default;
        header_msg.which_payload = sensor_data_messages_FootSensorLogMessage_sensing_data_tag;

        string_callback_arg_t fw_version_arg = {.str = fw_version};
        header_msg.payload.sensing_data.firmware_version.funcs.encode = &encode_string_callback;
        header_msg.payload.sensing_data.firmware_version.arg = &fw_version_arg;
        header_msg.payload.sensing_data.sampling_frequency = sampling_frequency;
        header_msg.payload.sensing_data.message_type = sensor_data_messages_FootSensorMessageType_FOOT_SENSOR_DATA;

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        bool encode_status = pb_encode(&stream, sensor_data_messages_FootSensorLogMessage_fields, &header_msg);
        if (!encode_status)
        {
            LOG_ERR("Foot sensor header encode error: %s", stream.errmsg);
            fs_close(&foot_sensor_log_file);
            status = err_t::PROTO_ENCODE_ERROR;
        }
        else
        {
            int write_ret = fs_write(&foot_sensor_log_file, buffer, stream.bytes_written);
            if (write_ret < 0)
            {
                LOG_ERR("fs_write error during foot sensor header write: %d", write_ret);
                fs_close(&foot_sensor_log_file);
                status = err_t::FILE_SYSTEM_ERROR;
            }
            else
            {
                int sync_ret = fs_sync(&foot_sensor_log_file);
                if (sync_ret != 0)
                {
                    LOG_ERR("FAIL: sync foot sensor header %s: [rd:%d]", foot_sensor_file_path, sync_ret);
                    fs_close(&foot_sensor_log_file);
                    status = err_t::FILE_SYSTEM_ERROR;
                }
                LOG_DBG("Foot sensor SensingData header written to %s (%u bytes).", foot_sensor_file_path,
                        stream.bytes_written);
                
                // Clear any previous file system errors since we succeeded
                send_error_to_bluetooth(SENDER_DATA, err_t::FILE_SYSTEM_ERROR, false);
            }
        }
    }

    k_mutex_unlock(&foot_sensor_file_mutex);
    return status;
}

/**
 * @brief Ends the data logging process for the foot sensor.
 * Appends a SessionEnd message, closes the file, and notifies Bluetooth.
 *
 * @return err_t NO_ERROR on success, or an error code on failure.
 */
err_t end_foot_sensor_logging()
{
    err_t overall_status = err_t::NO_ERROR;
    
    // Lock mutex for thread safety
    k_mutex_lock(&foot_sensor_file_mutex, K_FOREVER);

    // --- End Foot Sensor Log ---
    if (foot_sensor_log_file.filep != nullptr)
    {
        sensor_data_messages_FootSensorLogMessage end_foot_session_msg =
            sensor_data_messages_FootSensorLogMessage_init_default;
        end_foot_session_msg.which_payload = sensor_data_messages_FootSensorLogMessage_session_end_tag;
        end_foot_session_msg.payload.session_end.uptime_ms = k_uptime_get();

        err_t err_foot_write = write_foot_sensor_protobuf_data(&end_foot_session_msg);
        if (err_foot_write != err_t::NO_ERROR)
        {
            LOG_ERR("Failed to append foot sensor SessionEnd to file: %d", (int)err_foot_write);
            overall_status = err_foot_write;
        }

        // Flush any remaining buffered data
        flush_foot_sensor_buffer();

        int ret_foot_close = fs_close(&foot_sensor_log_file);
        if (ret_foot_close != 0)
        {
            LOG_ERR("FAIL: close foot sensor log %s: %d", foot_sensor_file_path, ret_foot_close);
            if (overall_status == err_t::NO_ERROR)
                overall_status = err_t::FILE_SYSTEM_ERROR;
        }
        else
        {
            LOG_INF("Closing Foot Sensor Log file '%s'", foot_sensor_file_path);
            // Notify Bluetooth for Foot Sensor log
            generic_message_t foot_msg;
            foot_msg.sender = SENDER_DATA;
            foot_msg.type = MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE;
            foot_msg.data.new_hardware_log_file.file_sequence_id = next_foot_sensor_file_sequence;
            strncpy(foot_msg.data.new_hardware_log_file.file_path, foot_sensor_file_path,
                    sizeof(foot_msg.data.new_hardware_log_file.file_path) - 1);
            foot_msg.data.new_hardware_log_file.file_path[sizeof(foot_msg.data.new_hardware_log_file.file_path) - 1] =
                '\0';

            if (k_msgq_put(&bluetooth_msgq, &foot_msg, K_NO_WAIT) != 0)
            {
                // First attempt failed, try with a small timeout
                if (k_msgq_put(&bluetooth_msgq, &foot_msg, K_MSEC(100)) != 0)
                {
                    LOG_ERR("end_foot_sensor_logging: Failed to put foot sensor log notification message.");
                }
                else
                {
                    LOG_DBG("Foot sensor log notification message sent after retry.");
                }
            }
            else
            {
                LOG_DBG("Foot sensor log notification message sent.");
            }
        }
        // Removed cap_and_reindex_log_files call from here
    }
    else
    {
        LOG_WRN("BHI360 log file was not open, skipping close/end session.");
        // Still send notification with current sequence and path (may be empty)
        generic_message_t foot_msg;
        foot_msg.sender = SENDER_DATA;
        foot_msg.type = MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE;
        foot_msg.data.new_hardware_log_file.file_sequence_id = next_foot_sensor_file_sequence;
        strncpy(foot_msg.data.new_hardware_log_file.file_path, foot_sensor_file_path,
                sizeof(foot_msg.data.new_hardware_log_file.file_path) - 1);
        foot_msg.data.new_hardware_log_file.file_path[sizeof(foot_msg.data.new_hardware_log_file.file_path) - 1] = '\0';
        if (k_msgq_put(&bluetooth_msgq, &foot_msg, K_NO_WAIT) != 0)
        {
            // First attempt failed, try with a small timeout
            if (k_msgq_put(&bluetooth_msgq, &foot_msg, K_MSEC(100)) != 0)
            {
                LOG_ERR("end_foot_sensor_logging: Failed to put foot sensor log notification message (file not open).");
            }
            else
            {
                LOG_DBG("Foot Sensor log notification message sent after retry (file not open).");
            }
        }
        else
        {
            LOG_DBG("Foot Sensor log notification message sent (file not open).");
        }
    }

    k_mutex_unlock(&foot_sensor_file_mutex);
    return overall_status;
}

static bool encode_foot_sensor_readings_callback(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
    const foot_samples_t *foot_data = (const foot_samples_t *)(*arg);

    for (uint8_t i = 0; i < NUM_FOOT_SENSOR_CHANNELS; ++i)
    {
        if (!pb_encode_tag_for_field(stream, field))
        {
            return false;
        }
        if (!pb_encode_varint(stream, foot_data->values[i]))
        {
            return false;
        }
    }
    return true;
}
/**
 * @brief Starts the data logging process for the BHI360 sensor.
 * Opens a new log file with a sequence number and writes the SensingData header.
 *
 * @return err_t NO_ERROR on success, or an error code on failure.
 */
err_t start_bhi360_logging(uint32_t sampling_frequency, const char *fw_version)
{
    err_t status = err_t::NO_ERROR;
    
    // Lock mutex for thread safety
    k_mutex_lock(&bhi360_file_mutex, K_FOREVER);

    // Reset timestamp tracking for new logging session
    bhi360_last_timestamp_ms = (uint32_t)k_uptime_get();
    bhi360_first_packet = true;

    // --- Check filesystem space and cleanup if needed ---
    if (!check_filesystem_space_and_cleanup()) {
        LOG_ERR("Insufficient filesystem space for new BHI360 log");
        k_mutex_unlock(&bhi360_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // --- Initialize BHI360 Log File ---
    // Protect sequence number generation with mutex
    k_mutex_lock(&sequence_number_mutex, K_FOREVER);
    next_bhi360_file_sequence = (uint8_t)get_next_file_sequence(hardware_dir_path, bhi360_file_prefix);
    if (next_bhi360_file_sequence == 0) {
        next_bhi360_file_sequence = 1;
    }
    k_mutex_unlock(&sequence_number_mutex);
    snprintk(bhi360_file_path, sizeof(bhi360_file_path), "%s/%s%03u.pb", hardware_dir_path, bhi360_file_prefix,
             next_bhi360_file_sequence);
    fs_file_t_init(&bhi360_log_file);

    int ret_fs_open = fs_open(&bhi360_log_file, bhi360_file_path, FS_O_CREATE | FS_O_RDWR | FS_O_APPEND);
    if (ret_fs_open != 0)
    {
        LOG_ERR("FAIL: open BHI360 log %s: %d", bhi360_file_path, ret_fs_open);
        status = err_t::FILE_SYSTEM_ERROR;
    }
    else
    {
        LOG_INF("Opened new BHI360 log file: %s", bhi360_file_path);

        uint8_t buffer[PROTOBUF_ENCODE_BUFFER_SIZE]; // Sufficient for SensingData message for BHI360
        sensor_data_messages_BHI360LogMessage header_msg = sensor_data_messages_BHI360LogMessage_init_default;
        header_msg.which_payload = sensor_data_messages_BHI360LogMessage_sensing_data_tag;

        string_callback_arg_t fw_version_arg = {.str = fw_version};
        header_msg.payload.sensing_data.firmware_version.funcs.encode = &encode_string_callback;
        header_msg.payload.sensing_data.firmware_version.arg = &fw_version_arg;
        header_msg.payload.sensing_data.sampling_frequency = sampling_frequency;
        header_msg.payload.sensing_data.message_type =
            sensor_data_messages_BHI360MessageType_BHI360_3D_DATA; // Or other BHI360 specific type

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        bool encode_status = pb_encode(&stream, sensor_data_messages_BHI360LogMessage_fields, &header_msg);
        if (!encode_status)
        {
            LOG_ERR("BHI360 header encode error: %s", stream.errmsg);
            fs_close(&bhi360_log_file);
            status = err_t::PROTO_ENCODE_ERROR;
        }
        else
        {
            int write_ret = fs_write(&bhi360_log_file, buffer, stream.bytes_written);
            if (write_ret < 0)
            {
                LOG_ERR("fs_write error during BHI360 header write: %d", write_ret);
                fs_close(&bhi360_log_file);
                status = err_t::FILE_SYSTEM_ERROR;
            }
            else
            {
                int sync_ret = fs_sync(&bhi360_log_file);
                if (sync_ret != 0)
                {
                    LOG_ERR("FAIL: sync BHI360 header %s: [rd:%d]", bhi360_file_path, sync_ret);
                    fs_close(&bhi360_log_file);
                    status = err_t::FILE_SYSTEM_ERROR;
                }
                LOG_DBG("BHI360 SensingData header written to %s (%u bytes).", bhi360_file_path, stream.bytes_written);
            }
        }
    }

    k_mutex_unlock(&bhi360_file_mutex);
    return status;
}

/**
 * @brief Ends the data logging process for the BHI360 sensor.
 * Appends a SessionEnd message, closes the file, and notifies Bluetooth.
 *
 * @return err_t NO_ERROR on success, or an error code on failure.
 */
err_t end_bhi360_logging()
{
    err_t overall_status = err_t::NO_ERROR;
    
    // Lock mutex for thread safety
    k_mutex_lock(&bhi360_file_mutex, K_FOREVER);

    // --- End BHI360 Log ---
    if (bhi360_log_file.filep != nullptr)
    {
        sensor_data_messages_BHI360LogMessage end_bhi360_session_msg =
            sensor_data_messages_BHI360LogMessage_init_default;
        end_bhi360_session_msg.which_payload = sensor_data_messages_BHI360LogMessage_session_end_tag;
        end_bhi360_session_msg.payload.session_end.uptime_ms = k_uptime_get();

        err_t err_bhi360_write = write_bhi360_protobuf_data(&end_bhi360_session_msg);
        if (err_bhi360_write != err_t::NO_ERROR)
        {
            LOG_ERR("Failed to append BHI360 SessionEnd to file: %d", (int)err_bhi360_write);
            overall_status = err_bhi360_write;
        }

        // Flush any remaining buffered data
        flush_bhi360_buffer();

        int ret_bhi360_close = fs_close(&bhi360_log_file);
        if (ret_bhi360_close != 0)
        {
            LOG_ERR("FAIL: close BHI360 log %s: %d", bhi360_file_path, ret_bhi360_close);
            if (overall_status == err_t::NO_ERROR)
                overall_status = err_t::FILE_SYSTEM_ERROR;
        }
        else
        {
            LOG_INF("Closing BHI360 Log file '%s'", bhi360_file_path);
        }
        // Always send notification after closing, regardless of close result
        generic_message_t bhi360_msg;
        bhi360_msg.sender = SENDER_DATA;
        bhi360_msg.type = MSG_TYPE_NEW_BHI360_LOG_FILE;
        bhi360_msg.data.new_hardware_log_file.file_sequence_id = next_bhi360_file_sequence;
        strncpy(bhi360_msg.data.new_hardware_log_file.file_path, bhi360_file_path,
                sizeof(bhi360_msg.data.new_hardware_log_file.file_path) - 1);
        bhi360_msg.data.new_hardware_log_file.file_path[sizeof(bhi360_msg.data.new_hardware_log_file.file_path) - 1] = '\0';

        if (k_msgq_put(&bluetooth_msgq, &bhi360_msg, K_NO_WAIT) != 0)
        {
            // First attempt failed, try with a small timeout
            if (k_msgq_put(&bluetooth_msgq, &bhi360_msg, K_MSEC(100)) != 0)
            {
                LOG_ERR("end_bhi360_logging: Failed to put BHI360 log notification message.");
            }
            else
            {
                LOG_DBG("BHI360 log notification message sent after retry.");
            }
        }
        else
        {
            LOG_DBG("BHI360 log notification message sent.");
        }
        // Removed cap_and_reindex_log_files call from here
    }
    else
    {
        LOG_WRN("BHI360 log file was not open, skipping close/end session.");
        // Still send notification with current sequence and path (may be empty)
        generic_message_t bhi360_msg;
        bhi360_msg.sender = SENDER_DATA;
        bhi360_msg.type = MSG_TYPE_NEW_BHI360_LOG_FILE;
        bhi360_msg.data.new_hardware_log_file.file_sequence_id = next_bhi360_file_sequence;
        strncpy(bhi360_msg.data.new_hardware_log_file.file_path, bhi360_file_path,
                sizeof(bhi360_msg.data.new_hardware_log_file.file_path) - 1);
        bhi360_msg.data.new_hardware_log_file.file_path[sizeof(bhi360_msg.data.new_hardware_log_file.file_path) - 1] = '\0';
        if (k_msgq_put(&bluetooth_msgq, &bhi360_msg, K_NO_WAIT) != 0)
        {
            // First attempt failed, try with a small timeout
            if (k_msgq_put(&bluetooth_msgq, &bhi360_msg, K_MSEC(100)) != 0)
            {
                LOG_ERR("end_bhi360_logging: Failed to put BHI360 log notification message (file not open).");
            }
            else
            {
                LOG_DBG("BHI360 log notification message sent after retry (file not open).");
            }
        }
        else
        {
            LOG_DBG("BHI360 log notification message sent (file not open).");
        }
    }

    k_mutex_unlock(&bhi360_file_mutex);
    return overall_status;
}

/**
 * @brief Writes a FootSensorLogMessage to the foot sensor log file.
 *
 * @param sensor_msg Pointer to the FootSensorLogMessage to write.
 * @return err_t NO_ERROR on success, or an error code on failure.
 */
static err_t write_foot_sensor_protobuf_data(const sensor_data_messages_FootSensorLogMessage *sensor_msg)
{
    if (!filesystem_available) {
        return err_t::FILE_SYSTEM_ERROR;
    }
    
    // Periodically check filesystem space (if enabled)
    if (ENABLE_RUNTIME_SPACE_CHECKING) {
        uint32_t current_time_ms = k_uptime_get_32();
        if ((current_time_ms - last_filesystem_check_time_ms) >= FILESYSTEM_CHECK_TIME_INTERVAL_MS) {
            last_filesystem_check_time_ms = current_time_ms;
            if (!check_filesystem_space_and_cleanup()) {
                LOG_ERR("Filesystem full during foot sensor logging");
                return err_t::FILE_SYSTEM_ERROR;
            }
        }
    }
    
    uint8_t buffer[PROTOBUF_ENCODE_BUFFER_SIZE]; // Ensure this buffer is large enough for any FootSensorLogMessage
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    bool status = pb_encode(&stream, sensor_data_messages_FootSensorLogMessage_fields, sensor_msg);
    if (!status)
    {
        LOG_ERR("Foot sensor Protobuf encode error: %s", stream.errmsg);
        return err_t::PROTO_ENCODE_ERROR;
    }

    // Lock mutex for buffer access
    k_mutex_lock(&foot_sensor_file_mutex, K_FOREVER);
    
    // Batch buffering logic
    if (foot_sensor_write_buffer_pos + stream.bytes_written > FLASH_PAGE_SIZE) {
        flush_foot_sensor_buffer();
    }
    memcpy(&foot_sensor_write_buffer[foot_sensor_write_buffer_pos], buffer, stream.bytes_written);
    foot_sensor_write_buffer_pos += stream.bytes_written;

    // Optionally, flush immediately for header or session end messages
    if (sensor_msg->which_payload == sensor_data_messages_FootSensorLogMessage_sensing_data_tag ||
        sensor_msg->which_payload == sensor_data_messages_FootSensorLogMessage_session_end_tag) {
        flush_foot_sensor_buffer();
    }

    k_mutex_unlock(&foot_sensor_file_mutex);
    
    LOG_DBG("Foot sensor data buffered (%u bytes, buffer at %u/%u).", stream.bytes_written, (unsigned)foot_sensor_write_buffer_pos, FLASH_PAGE_SIZE);
    return err_t::NO_ERROR;
}

/**
 * @brief Writes a BHI360LogMessage to the BHI360 log file.
 *
 * @param sensor_msg Pointer to the BHI360LogMessage to write.
 * @return err_t NO_ERROR on success, or an error code on failure.
 */
static err_t write_bhi360_protobuf_data(const sensor_data_messages_BHI360LogMessage *sensor_msg)
{
    if (!filesystem_available) {
        return err_t::FILE_SYSTEM_ERROR;
    }
    
    // Periodically check filesystem space (if enabled)
    if (ENABLE_RUNTIME_SPACE_CHECKING) {
        uint32_t current_time_ms = k_uptime_get_32();
        if ((current_time_ms - last_filesystem_check_time_ms) >= FILESYSTEM_CHECK_TIME_INTERVAL_MS) {
            last_filesystem_check_time_ms = current_time_ms;
            if (!check_filesystem_space_and_cleanup()) {
                LOG_ERR("Filesystem full during BHI360 logging");
                return err_t::FILE_SYSTEM_ERROR;
            }
        }
    }
    
    uint8_t buffer[PROTOBUF_ENCODE_BUFFER_SIZE];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    bool status = pb_encode(&stream, sensor_data_messages_BHI360LogMessage_fields, sensor_msg);
    if (!status)
    {
        LOG_ERR("BHI360 Protobuf encode error: %s", stream.errmsg);
        return err_t::PROTO_ENCODE_ERROR;
    }

    // Lock mutex for buffer access
    k_mutex_lock(&bhi360_file_mutex, K_FOREVER);
    
    // Batch buffering logic
    if (bhi360_write_buffer_pos + stream.bytes_written > FLASH_PAGE_SIZE) {
        flush_bhi360_buffer();
    }
    memcpy(&bhi360_write_buffer[bhi360_write_buffer_pos], buffer, stream.bytes_written);
    bhi360_write_buffer_pos += stream.bytes_written;

    // Optionally, flush immediately for header or session end messages
    if (sensor_msg->which_payload == sensor_data_messages_BHI360LogMessage_sensing_data_tag ||
        sensor_msg->which_payload == sensor_data_messages_BHI360LogMessage_session_end_tag) {
        flush_bhi360_buffer();
    }

    k_mutex_unlock(&bhi360_file_mutex);
    
    LOG_DBG("BHI360 data buffered (%u bytes, buffer at %u/%u).", stream.bytes_written, (unsigned)bhi360_write_buffer_pos, FLASH_PAGE_SIZE);
    return err_t::NO_ERROR;
}

/**
 * @brief Starts the data logging process for the activity file.
 * Opens a new log file with a sequence number and writes the SensingData header.
 *
 * @return err_t NO_ERROR on success, or an error code on failure.
 */
err_t start_activity_logging(uint32_t sampling_frequency, const char *fw_version)
{
    err_t status = err_t::NO_ERROR;
    
    // Lock mutex for thread safety
    k_mutex_lock(&activity_file_mutex, K_FOREVER);

    // Reset timestamp tracking for new logging session
    activity_last_timestamp_ms = (uint32_t)k_uptime_get();
    activity_first_packet = true;

    // --- Create hardware directory if it doesn't exist ---
    int ret_mkdir = fs_mkdir(hardware_dir_path);
    if (ret_mkdir != 0 && ret_mkdir != -EEXIST)
    {
        LOG_INF("FAIL: fs_mkdir %s: %d", hardware_dir_path, ret_mkdir);
        k_mutex_unlock(&activity_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // --- Check filesystem space and cleanup if needed ---
    if (!check_filesystem_space_and_cleanup()) {
        LOG_ERR("Insufficient filesystem space for new activity log");
        k_mutex_unlock(&activity_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // --- Initialize Activity Log File ---
    // Protect sequence number generation with mutex
    k_mutex_lock(&sequence_number_mutex, K_FOREVER);
    next_activity_file_sequence = (uint8_t)get_next_file_sequence(hardware_dir_path, activity_file_prefix);
    if (next_activity_file_sequence == 0) {
        next_activity_file_sequence = 1;
    }
    k_mutex_unlock(&sequence_number_mutex);
    snprintk(activity_file_path, sizeof(activity_file_path), "%s/%s%03u.pb", hardware_dir_path,
             activity_file_prefix, next_activity_file_sequence);
    fs_file_t_init(&activity_log_file);

    int ret_fs_open = fs_open(&activity_log_file, activity_file_path, FS_O_CREATE | FS_O_RDWR | FS_O_APPEND);
    if (ret_fs_open != 0)
    {
        LOG_ERR("FAIL: open activity log %s: %d", activity_file_path, ret_fs_open);
        status = err_t::FILE_SYSTEM_ERROR;
        
        // Report error to Bluetooth module
        send_error_to_bluetooth(SENDER_DATA, err_t::FILE_SYSTEM_ERROR, true);
    }
    else
    {
        LOG_INF("Opened new activity log file: %s", activity_file_path);

        uint8_t buffer[PROTOBUF_ENCODE_BUFFER_SIZE]; // Sufficient for SensingData message
        sensor_data_messages_ActivityLogMessage header_msg = sensor_data_messages_ActivityLogMessage_init_default;
        header_msg.which_payload = sensor_data_messages_ActivityLogMessage_sensing_data_tag;

        string_callback_arg_t fw_version_arg = {.str = fw_version};
        header_msg.payload.sensing_data.firmware_version.funcs.encode = &encode_string_callback;
        header_msg.payload.sensing_data.firmware_version.arg = &fw_version_arg;
        header_msg.payload.sensing_data.sampling_frequency = sampling_frequency;
        header_msg.payload.sensing_data.message_type = sensor_data_messages_ActivityMessageType_ACTIVITY_STEP_DATA;

        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        bool encode_status = pb_encode(&stream, sensor_data_messages_ActivityLogMessage_fields, &header_msg);
        if (!encode_status)
        {
            LOG_ERR("Activity header encode error: %s", stream.errmsg);
            fs_close(&activity_log_file);
            status = err_t::PROTO_ENCODE_ERROR;
        }
        else
        {
            int write_ret = fs_write(&activity_log_file, buffer, stream.bytes_written);
            if (write_ret < 0)
            {
                LOG_ERR("fs_write error during activity header write: %d", write_ret);
                fs_close(&activity_log_file);
                status = err_t::FILE_SYSTEM_ERROR;
            }
            else
            {
                int sync_ret = fs_sync(&activity_log_file);
                if (sync_ret != 0)
                {
                    LOG_ERR("FAIL: sync activity header %s: [rd:%d]", activity_file_path, sync_ret);
                    fs_close(&activity_log_file);
                    status = err_t::FILE_SYSTEM_ERROR;
                }
                LOG_DBG("Activity SensingData header written to %s (%u bytes).", activity_file_path,
                        stream.bytes_written);
                
                // Clear any previous file system errors since we succeeded
                send_error_to_bluetooth(SENDER_DATA, err_t::FILE_SYSTEM_ERROR, false);
            }
        }
    }

    k_mutex_unlock(&activity_file_mutex);
    return status;
}

/**
 * @brief Ends the data logging process for the activity file.
 * Appends a SessionEnd message, closes the file, and notifies Bluetooth.
 *
 * @return err_t NO_ERROR on success, or an error code on failure.
 */
err_t end_activity_logging()
{
    err_t overall_status = err_t::NO_ERROR;
    
    // Lock mutex for thread safety
    k_mutex_lock(&activity_file_mutex, K_FOREVER);

    // --- End Activity Log ---
    if (activity_log_file.filep != nullptr)
    {
        sensor_data_messages_ActivityLogMessage end_activity_session_msg =
            sensor_data_messages_ActivityLogMessage_init_default;
        end_activity_session_msg.which_payload = sensor_data_messages_ActivityLogMessage_session_end_tag;
        end_activity_session_msg.payload.session_end.uptime_ms = k_uptime_get();

        err_t err_activity_write = write_activity_protobuf_data(&end_activity_session_msg);
        if (err_activity_write != err_t::NO_ERROR)
        {
            LOG_ERR("Failed to append activity SessionEnd to file: %d", (int)err_activity_write);
            overall_status = err_activity_write;
        }

        // Flush any remaining buffered data
        flush_activity_buffer();

        int ret_activity_close = fs_close(&activity_log_file);
        if (ret_activity_close != 0)
        {
            LOG_ERR("FAIL: close activity log %s: %d", activity_file_path, ret_activity_close);
            if (overall_status == err_t::NO_ERROR)
                overall_status = err_t::FILE_SYSTEM_ERROR;
        }
        else
        {
            LOG_INF("Closing Activity Log file '%s'", activity_file_path);
        }
        // Always send notification after closing, regardless of close result
        generic_message_t activity_msg;
        activity_msg.sender = SENDER_DATA;
        activity_msg.type = MSG_TYPE_NEW_ACTIVITY_LOG_FILE;
        activity_msg.data.new_hardware_log_file.file_sequence_id = next_activity_file_sequence;
        strncpy(activity_msg.data.new_hardware_log_file.file_path, activity_file_path,
                sizeof(activity_msg.data.new_hardware_log_file.file_path) - 1);
        activity_msg.data.new_hardware_log_file.file_path[sizeof(activity_msg.data.new_hardware_log_file.file_path) - 1] = '\0';

        if (k_msgq_put(&bluetooth_msgq, &activity_msg, K_NO_WAIT) != 0)
        {
            // First attempt failed, try with a small timeout
            if (k_msgq_put(&bluetooth_msgq, &activity_msg, K_MSEC(100)) != 0)
            {
                LOG_ERR("end_activity_logging: Failed to put activity log notification message.");
            }
            else
            {
                LOG_DBG("Activity log notification message sent after retry.");
            }
        }
        else
        {
            LOG_DBG("Activity log notification message sent.");
        }
    }
    else
    {
        LOG_WRN("Activity log file was not open, skipping close/end session.");
        // Still send notification with current sequence and path (may be empty)
        generic_message_t activity_msg;
        activity_msg.sender = SENDER_DATA;
        activity_msg.type = MSG_TYPE_NEW_ACTIVITY_LOG_FILE;
        activity_msg.data.new_hardware_log_file.file_sequence_id = next_activity_file_sequence;
        strncpy(activity_msg.data.new_hardware_log_file.file_path, activity_file_path,
                sizeof(activity_msg.data.new_hardware_log_file.file_path) - 1);
        activity_msg.data.new_hardware_log_file.file_path[sizeof(activity_msg.data.new_hardware_log_file.file_path) - 1] = '\0';
        if (k_msgq_put(&bluetooth_msgq, &activity_msg, K_NO_WAIT) != 0)
        {
            LOG_ERR("Failed to put activity log notification message (file not open).");
        }
        else
        {
            LOG_DBG("Activity log notification message sent (file not open).");
        }
    }

    k_mutex_unlock(&activity_file_mutex);
    return overall_status;
}

/**
 * @brief Writes an ActivityLogMessage to the activity log file.
 *
 * @param sensor_msg Pointer to the ActivityLogMessage to write.
 * @return err_t NO_ERROR on success, or an error code on failure.
 */
static err_t write_activity_protobuf_data(const sensor_data_messages_ActivityLogMessage *sensor_msg)
{
    if (!filesystem_available) {
        return err_t::FILE_SYSTEM_ERROR;
    }
    
    // Periodically check filesystem space (if enabled)
    if (ENABLE_RUNTIME_SPACE_CHECKING) {
        uint32_t current_time_ms = k_uptime_get_32();
        if ((current_time_ms - last_filesystem_check_time_ms) >= FILESYSTEM_CHECK_TIME_INTERVAL_MS) {
            last_filesystem_check_time_ms = current_time_ms;
            if (!check_filesystem_space_and_cleanup()) {
                LOG_ERR("Filesystem full during activity logging");
                return err_t::FILE_SYSTEM_ERROR;
            }
        }
    }
    
    uint8_t buffer[PROTOBUF_ENCODE_BUFFER_SIZE];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    bool status = pb_encode(&stream, sensor_data_messages_ActivityLogMessage_fields, sensor_msg);
    if (!status)
    {
        LOG_ERR("Activity Protobuf encode error: %s", stream.errmsg);
        return err_t::PROTO_ENCODE_ERROR;
    }

    // Lock mutex for buffer access
    k_mutex_lock(&activity_file_mutex, K_FOREVER);
    
    // Batch buffering logic
    if (activity_write_buffer_pos + stream.bytes_written > FLASH_PAGE_SIZE) {
        flush_activity_buffer();
    }
    memcpy(&activity_write_buffer[activity_write_buffer_pos], buffer, stream.bytes_written);
    activity_write_buffer_pos += stream.bytes_written;

    // Optionally, flush immediately for header or session end messages
    if (sensor_msg->which_payload == sensor_data_messages_ActivityLogMessage_sensing_data_tag ||
        sensor_msg->which_payload == sensor_data_messages_ActivityLogMessage_session_end_tag) {
        flush_activity_buffer();
    }

    k_mutex_unlock(&activity_file_mutex);
    
    LOG_DBG("Activity data buffered (%u bytes, buffer at %u/%u).", stream.bytes_written, (unsigned)activity_write_buffer_pos, FLASH_PAGE_SIZE);
    return err_t::NO_ERROR;
}

err_t type_and_id_to_path(record_type_t type, uint8_t id, char path[])
{
    switch (type)
    {
        case RECORD_HARDWARE_FOOT_SENSOR:
            snprintk(path, util::max_path_length, "%s/%s%03u.pb", hardware_dir_path, foot_sensor_file_prefix, id);
            break;
        case RECORD_HARDWARE_BHI360: // New: Case for BHI360 files
            snprintk(path, util::max_path_length, "%s/%s%03u.pb", hardware_dir_path, bhi360_file_prefix, id);
            break;
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
    uint8_t temp_latest_file_seq;           // Temporary to hold uint32_t return from find_latest_log_file
    uint8_t latest_file_seq_for_notify = 0; // The uint8_t ID to send in notification

    if (type == RECORD_HARDWARE_FOOT_SENSOR)
    {
        char open_file_path[util::max_path_length] = "";
        if (foot_sensor_log_file.filep != nullptr)
        {
            strncpy(open_file_path, foot_sensor_file_path, sizeof(open_file_path) - 1);
            open_file_path[sizeof(open_file_path) - 1] = '\0';
        }
        temp_latest_file_seq = (uint8_t)find_latest_log_file(foot_sensor_file_prefix, RECORD_HARDWARE_FOOT_SENSOR, latest_file_path, open_file_path);
        if (temp_latest_file_seq == (uint32_t)err_t::DATA_ERROR)
        {
            LOG_INF("Foot sensor directory empty or error finding latest file after deletion. No file to report.");
            latest_file_seq_for_notify = 0;
            generic_message_t foot_log_info_msg;
            foot_log_info_msg.sender = SENDER_DATA;
            foot_log_info_msg.type = MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE;
            foot_log_info_msg.data.new_hardware_log_file.file_sequence_id = latest_file_seq_for_notify;
            strncpy(foot_log_info_msg.data.new_hardware_log_file.file_path, "",
                    sizeof(foot_log_info_msg.data.new_hardware_log_file.file_path) - 1);
            foot_log_info_msg.data.new_hardware_log_file
                .file_path[sizeof(foot_log_info_msg.data.new_hardware_log_file.file_path) - 1] = '\0';

            if (k_msgq_put(&bluetooth_msgq, &foot_log_info_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to put empty foot sensor log notification message.");
            }
            else
            {
                LOG_DBG("Sent empty foot sensor log notification.");
            }
        }
        else
        {
            latest_file_seq_for_notify = (uint8_t)temp_latest_file_seq;
            LOG_INF("New latest foot sensor log file: %s (Seq: %u)", latest_file_path, latest_file_seq_for_notify);
            generic_message_t foot_log_info_msg;
            foot_log_info_msg.sender = SENDER_DATA;
            foot_log_info_msg.type = MSG_TYPE_NEW_FOOT_SENSOR_LOG_FILE;
            foot_log_info_msg.data.new_hardware_log_file.file_sequence_id = latest_file_seq_for_notify;
            strncpy(foot_log_info_msg.data.new_hardware_log_file.file_path, latest_file_path,
                    sizeof(foot_log_info_msg.data.new_hardware_log_file.file_path) - 1);
            foot_log_info_msg.data.new_hardware_log_file
                .file_path[sizeof(foot_log_info_msg.data.new_hardware_log_file.file_path) - 1] = '\0';

            if (k_msgq_put(&bluetooth_msgq, &foot_log_info_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to put new latest foot sensor log notification message.");
            }
            else
            {
                LOG_DBG("Sent new latest foot sensor log notification (ID: %u).", latest_file_seq_for_notify);
            }
        }
    }
    else if (type == RECORD_HARDWARE_BHI360)
    {
        char open_file_path[util::max_path_length] = "";
        if (bhi360_log_file.filep != nullptr)
        {
            strncpy(open_file_path, bhi360_file_path, sizeof(open_file_path) - 1);
            open_file_path[sizeof(open_file_path) - 1] = '\0';
        }
        temp_latest_file_seq = (uint8_t)find_latest_log_file(bhi360_file_prefix, RECORD_HARDWARE_BHI360, latest_file_path, open_file_path);
        if (temp_latest_file_seq == (uint32_t)err_t::DATA_ERROR)
        {
            LOG_INF("BHI360 directory empty or error finding latest file after deletion. No file to report.");
            latest_file_seq_for_notify = 0;
            generic_message_t bhi360_log_info_msg;
            bhi360_log_info_msg.sender = SENDER_DATA;
            bhi360_log_info_msg.type = MSG_TYPE_NEW_BHI360_LOG_FILE;
            bhi360_log_info_msg.data.new_hardware_log_file.file_sequence_id = latest_file_seq_for_notify;
            strncpy(bhi360_log_info_msg.data.new_hardware_log_file.file_path, "",
                    sizeof(bhi360_log_info_msg.data.new_hardware_log_file.file_path) - 1);
            bhi360_log_info_msg.data.new_hardware_log_file
                .file_path[sizeof(bhi360_log_info_msg.data.new_hardware_log_file.file_path) - 1] = '\0';

            if (k_msgq_put(&bluetooth_msgq, &bhi360_log_info_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to put empty BHI360 log notification message.");
            }
            else
            {
                LOG_DBG("Sent empty BHI360 log notification.");
            }
        }
        else
        {
            latest_file_seq_for_notify = (uint8_t)temp_latest_file_seq;
            LOG_INF("New latest BHI360 log file: %s (Seq: %u)", latest_file_path, latest_file_seq_for_notify);
            generic_message_t bhi360_log_info_msg;
            bhi360_log_info_msg.sender = SENDER_DATA;
            bhi360_log_info_msg.type = MSG_TYPE_NEW_BHI360_LOG_FILE;
            bhi360_log_info_msg.data.new_hardware_log_file.file_sequence_id = latest_file_seq_for_notify;
            strncpy(bhi360_log_info_msg.data.new_hardware_log_file.file_path, latest_file_path,
                    sizeof(bhi360_log_info_msg.data.new_hardware_log_file.file_path) - 1);
            bhi360_log_info_msg.data.new_hardware_log_file
                .file_path[sizeof(bhi360_log_info_msg.data.new_hardware_log_file.file_path) - 1] = '\0';

            if (k_msgq_put(&bluetooth_msgq, &bhi360_log_info_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to put new latest BHI360 log notification message.");
            }
            else
            {
                LOG_DBG("Sent new latest BHI360 log notification (ID: %u).", latest_file_seq_for_notify);
            }
        }
    }
    else if (type == RECORD_HARDWARE_ACTIVITY)
    {
        char open_file_path[util::max_path_length] = "";
        if (activity_log_file.filep != nullptr)
        {
            strncpy(open_file_path, activity_file_path, sizeof(open_file_path) - 1);
            open_file_path[sizeof(open_file_path) - 1] = '\0';
        }
        temp_latest_file_seq = (uint8_t)find_latest_log_file(activity_file_prefix, RECORD_HARDWARE_ACTIVITY, latest_file_path, open_file_path);
        if (temp_latest_file_seq == (uint32_t)err_t::DATA_ERROR)
        {
            LOG_INF("Activity directory empty or error finding latest file after deletion. No file to report.");
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
                LOG_DBG("Sent empty activity log notification.");
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
                LOG_DBG("Sent new latest activity log notification (ID: %u).", latest_file_seq_for_notify);
            }
        }
    }
    return err_t::NO_ERROR;
}

// Helper struct for sorting
struct LogFileEntry
{
    uint8_t id;
    char name[util::max_path_length];
};


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
    if (file_count > max_files) {
        size_t to_delete = std::min<size_t>(delete_count, file_count - max_files + delete_count);
        LOG_INF("Capping: Deleting %zu oldest files for prefix %s", to_delete, file_prefix);
        for (size_t i = 0; i < to_delete; ++i) {
            char full_path[UTIL_MAX_INTERNAL_PATH_LENGTH];
            int written = snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, log_files[i].name);
            if (written < 0 || written >= (int)sizeof(full_path)) {
                LOG_ERR("Internal path buffer too small for deleting: '%s/%s'", dir_path, log_files[i].name);
                continue;
            }
            int del_res = fs_unlink(full_path);
            if (del_res != 0) {
                LOG_ERR("Failed to unlink %s: %d", full_path, del_res);
            } else {
                LOG_INF("Deleted log file: %s", full_path);
            }
        }
        // Remove deleted files from vector
        log_files.erase(log_files.begin(), log_files.begin() + to_delete);
    }

    // 4. Reindex only if more than 255 files
    if (log_files.size() > 255) {
        for (size_t i = 0; i < log_files.size(); ++i) {
            uint8_t new_id = (uint8_t)(i + 1); // Start from 1, not 0
            if (log_files[i].id != new_id) {
                char old_path[UTIL_MAX_INTERNAL_PATH_LENGTH];
                char new_name[32];
                char new_path[UTIL_MAX_INTERNAL_PATH_LENGTH];
                snprintf(old_path, sizeof(old_path), "%s/%s", dir_path, log_files[i].name);
                snprintf(new_name, sizeof(new_name), "%s%03u.pb", file_prefix, new_id);
                snprintf(new_path, sizeof(new_path), "%s/%s", dir_path, new_name);
                int rename_res = fs_rename(old_path, new_path);
                if (rename_res != 0) {
                    LOG_ERR("Failed to rename %s to %s: %d", old_path, new_path, rename_res);
                } else {
                    LOG_INF("Renamed %s to %s", old_path, new_path);
                }
            }
        }
    }
    // All IDs are now contiguous and <= 255
    return err_t::NO_ERROR;
}

/**
 * @brief Delete the oldest log file matching the prefix in the directory, using large internal buffer for file ops.
 *        BLE/external buffers remain small for transmission.
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

static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        const struct module_state_event *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(app), MODULE_STATE_READY))
        {
            if (data_init() == err_t::NO_ERROR)
            {
                LOG_DBG("Data module initialization successful.");
            }
            else
            {
                LOG_ERR("Data module initialization failed.");
            }
        }
        return false;
    }
    else if (is_app_state_event(aeh)) // Check for app state events
    {
        const struct app_state_event *event = cast_app_state_event(aeh);
        // Assuming APP_STATE_SHUTDOWN_PREPARING is a state defined in events/app_state_event.h
        // This state should be sent by your main application logic before a system shutdown/reboot.
        if (event->state == APP_STATE_SHUTDOWN_PREPARING)
        {
            LOG_INF("Received APP_STATE_SHUTDOWN_PREPARING event. Attempting to close log files.");
            // Call the end logging functions to flush and close files
            err_t foot_close_status = end_foot_sensor_logging();
            if (foot_close_status != err_t::NO_ERROR)
            {
                LOG_ERR("Error closing foot sensor log during shutdown: %d", (int)foot_close_status);
            }

            err_t bhi360_close_status = end_bhi360_logging();
            if (bhi360_close_status != err_t::NO_ERROR)
            {
                LOG_ERR("Error closing BHI360 log during shutdown: %d", (int)bhi360_close_status);
            }
        }
        return false;
    }
    return false;
}

/**
 * @brief Save BHI360 calibration data to file
 *
 * @param calib_data Pointer to calibration data structure
 * @return err_t NO_ERROR on success, or an error code on failure
 */
err_t save_bhi360_calibration_data(const bhi360_calibration_data_t *calib_data)
{
    if (!calib_data) {
        return err_t::INVALID_PARAMETER;
    }

    // Lock mutex for thread safety
    k_mutex_lock(&calibration_file_mutex, K_FOREVER);

    // Create calibration directory if it doesn't exist
    int ret = fs_mkdir(calibration_dir_path);
    if (ret != 0 && ret != -EEXIST) {
        LOG_ERR("Failed to create calibration directory: %d", ret);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Create filename based on sensor type
    char filename[util::max_path_length];
    const char *sensor_name;
    switch (calib_data->sensor_type) {
        case 0: sensor_name = "accel"; break;
        case 1: sensor_name = "gyro"; break;
        default:
            LOG_ERR("Invalid sensor type: %u (only 0=accel, 1=gyro supported)", calib_data->sensor_type);
            k_mutex_unlock(&calibration_file_mutex);
            return err_t::INVALID_PARAMETER;
    }
    
    snprintk(filename, sizeof(filename), "%s/%s%s.bin", 
             calibration_dir_path, bhi360_calib_file_prefix, sensor_name);

    // Open file for writing
    struct fs_file_t file;
    fs_file_t_init(&file);
    
    ret = fs_open(&file, filename, FS_O_CREATE | FS_O_WRITE);
    if (ret != 0) {
        LOG_ERR("Failed to open calibration file %s: %d", filename, ret);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Write calibration data
    // First write the profile size
    ret = fs_write(&file, &calib_data->profile_size, sizeof(calib_data->profile_size));
    if (ret < 0) {
        LOG_ERR("Failed to write profile size: %d", ret);
        fs_close(&file);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Then write the actual profile data
    ret = fs_write(&file, calib_data->profile_data, calib_data->profile_size);
    if (ret < 0) {
        LOG_ERR("Failed to write calibration data: %d", ret);
        fs_close(&file);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Sync and close file
    fs_sync(&file);
    fs_close(&file);

    LOG_INF("Saved BHI360 %s calibration to %s (%u bytes)", 
            sensor_name, filename, calib_data->profile_size);

    k_mutex_unlock(&calibration_file_mutex);
    return err_t::NO_ERROR;
}

/**
 * @brief Load BHI360 calibration data from file and send to motion sensor
 *
 * @param sensor_type Sensor type (0=accel, 1=gyro, 2=mag)
 * @return err_t NO_ERROR on success, or an error code on failure
 */
err_t load_bhi360_calibration_data(uint8_t sensor_type)
{
    // Create filename based on sensor type
    char filename[util::max_path_length];
    const char *sensor_name;
    switch (sensor_type) {
        case 0: sensor_name = "accel"; break;
        case 1: sensor_name = "gyro"; break;
        default:
            LOG_ERR("Invalid sensor type: %u (only 0=accel, 1=gyro supported)", sensor_type);
            k_mutex_unlock(&calibration_file_mutex);
            return err_t::INVALID_PARAMETER;
    }
    
    snprintk(filename, sizeof(filename), "%s/%s%s.bin", 
             calibration_dir_path, bhi360_calib_file_prefix, sensor_name);

    // Check if file exists
    struct fs_dirent entry;
    int ret = fs_stat(filename, &entry);
    if (ret != 0) {
        LOG_WRN("Calibration file %s not found", filename);
        return err_t::FILE_SYSTEM_NO_FILES;
    }

    // Open file for reading
    struct fs_file_t file;
    fs_file_t_init(&file);
    
    ret = fs_open(&file, filename, FS_O_READ);
    if (ret != 0) {
        LOG_ERR("Failed to open calibration file %s: %d", filename, ret);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Read profile size
    uint16_t profile_size;
    ret = fs_read(&file, &profile_size, sizeof(profile_size));
    if (ret < 0) {
        LOG_ERR("Failed to read profile size: %d", ret);
        fs_close(&file);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Validate profile size
    if (profile_size > 512) {
        LOG_ERR("Invalid profile size: %u", profile_size);
        fs_close(&file);
        return err_t::DATA_ERROR;
    }

    // Read calibration data
    uint8_t profile_data[512];
    ret = fs_read(&file, profile_data, profile_size);
    if (ret < 0) {
        LOG_ERR("Failed to read calibration data: %d", ret);
        fs_close(&file);
        return err_t::FILE_SYSTEM_ERROR;
    }

    fs_close(&file);

    LOG_INF("Loaded BHI360 %s calibration from %s (%u bytes)", 
    sensor_name, filename, profile_size);
    
    // Send calibration data to motion sensor via message queue
    generic_message_t calib_msg = {};
    calib_msg.sender = SENDER_DATA;
    calib_msg.type = MSG_TYPE_BHI360_CALIBRATION_DATA;
    calib_msg.data.bhi360_calibration.sensor_type = sensor_type;
    calib_msg.data.bhi360_calibration.profile_size = profile_size;
    memcpy(calib_msg.data.bhi360_calibration.profile_data, profile_data, profile_size);
    
    if (k_msgq_put(&motion_sensor_msgq, &calib_msg, K_NO_WAIT) != 0) {
    LOG_ERR("Failed to send calibration data to motion sensor");
    return err_t::QUEUE_FULL;
    }
    
    LOG_INF("Sent calibration data to motion sensor for sensor type %u", sensor_type);
    
    return err_t::NO_ERROR;
}

/**
 * @brief Public API to get BHI360 calibration data from storage
 * This function can be called by the motion sensor module during initialization
 *
 * @param sensor_type Sensor type (0=accel, 1=gyro, 2=mag)
 * @param profile_data Buffer to store calibration data
 * @param max_size Maximum size of the buffer
 * @param actual_size Actual size of the calibration data read
 * @return err_t NO_ERROR on success, or an error code on failure
 */
err_t get_bhi360_calibration_data(uint8_t sensor_type, uint8_t *profile_data, 
                                  size_t max_size, size_t *actual_size)
{
    if (!profile_data || !actual_size) {
        return err_t::INVALID_PARAMETER;
    }

    // Lock mutex for thread safety
    k_mutex_lock(&calibration_file_mutex, K_FOREVER);

    // Create filename based on sensor type
    char filename[util::max_path_length];
    const char *sensor_name;
    switch (sensor_type) {
        case 0: sensor_name = "accel"; break;
        case 1: sensor_name = "gyro"; break;
        default:
            LOG_ERR("Invalid sensor type: %u (only 0=accel, 1=gyro supported)", sensor_type);
            return err_t::INVALID_PARAMETER;
    }
    
    snprintk(filename, sizeof(filename), "%s/%s%s.bin", 
             calibration_dir_path, bhi360_calib_file_prefix, sensor_name);

    // Check if file exists
    struct fs_dirent entry;
    int ret = fs_stat(filename, &entry);
    if (ret != 0) {
        LOG_DBG("Calibration file %s not found", filename);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_NO_FILES;
    }

    // Open file for reading
    struct fs_file_t file;
    fs_file_t_init(&file);
    
    ret = fs_open(&file, filename, FS_O_READ);
    if (ret != 0) {
        LOG_ERR("Failed to open calibration file %s: %d", filename, ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Read profile size
    uint16_t profile_size;
    ret = fs_read(&file, &profile_size, sizeof(profile_size));
    if (ret < 0) {
        LOG_ERR("Failed to read profile size: %d", ret);
        fs_close(&file);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // Validate profile size
    if (profile_size > max_size) {
        LOG_ERR("Profile size %u exceeds buffer size %u", profile_size, max_size);
        fs_close(&file);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::DATA_ERROR;
    }

    // Read calibration data
    ret = fs_read(&file, profile_data, profile_size);
    if (ret < 0) {
        LOG_ERR("Failed to read calibration data: %d", ret);
        fs_close(&file);
        k_mutex_unlock(&calibration_file_mutex);
        return err_t::FILE_SYSTEM_ERROR;
    }

    fs_close(&file);
    *actual_size = profile_size;

    LOG_INF("Retrieved BHI360 %s calibration (%u bytes)", sensor_name, profile_size);
    k_mutex_unlock(&calibration_file_mutex);
    return err_t::NO_ERROR;
}

/**
 * @brief Public API to store BHI360 calibration data
 * This function can be called by the motion sensor module after calibration
 *
 * @param sensor_type Sensor type (0=accel, 1=gyro, 2=mag)
 * @param profile_data Calibration data to store
 * @param profile_size Size of the calibration data
 * @return err_t NO_ERROR on success, or an error code on failure
 */
err_t store_bhi360_calibration_data(uint8_t sensor_type, const uint8_t *profile_data, 
                                    size_t profile_size)
{
    if (!profile_data || profile_size == 0 || profile_size > 512) {
        return err_t::INVALID_PARAMETER;
    }

    // Create calibration data structure
    bhi360_calibration_data_t calib_data;
    calib_data.sensor_type = sensor_type;
    calib_data.profile_size = (uint16_t)profile_size;
    memcpy(calib_data.profile_data, profile_data, profile_size);

    // Use the existing save function
    return save_bhi360_calibration_data(&calib_data);
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);

#define MODULE data

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>
#include <variant>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>

#include <app_event_manager.h>
#include <ble_services.hpp>
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/bluetooth_state_event.h>
#include <events/data_event.h>
#include <util.hpp>

#include <app.hpp>
#include <bhi360_sensor_messages.pb.h>
#include <ble_services.hpp>
#include <data.hpp>
#include <foot_sensor_messages.pb.h>
#include <pb_decode.h>
#include <pb_encode.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_DATA_MODULE_LOG_LEVEL); // NOLINT

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(littlefs_storage);

// Define File System parameters
#define STORAGE_PARTITION_LABEL storage_ext
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION_LABEL)

static struct fs_mount_t lfs_ext_storage_mnt = {
    .type = FS_LITTLEFS,
    .mnt_point = "/lfs1",
    .fs_data = &littlefs_storage,
    .storage_dev = (void *)FLASH_AREA_ID(littlefs_storage),
};

static err_t mount_file_system(struct fs_mount_t *mount);
static err_t littlefs_flash_erase(unsigned int id);
static err_t data_init();
static int lsdir(const char *path);
static void strremove(char *str, const char *sub);

// --- Protobuf-specific write function for foot sensor ---
static err_t write_foot_sensor_protobuf_data(const sensor_data_messages_FootSensorLogMessage *sensor_msg);
static err_t write_bhi360_protobuf_data(const sensor_data_messages_BHI360LogMessage *sensor_msg);

static bool encode_foot_sensor_readings_callback(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);
static bool encode_string_callback(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);

int delete_oldest_log_files(const char *file_prefix, const char *dir_path);
err_t delete_by_type_and_id(record_type_t type, uint32_t id);
err_t type_and_id_to_path(record_type_t type, uint32_t id, char path[]);

err_t start_foot_sensor_logging();
err_t end_foot_sensor_logging();

err_t start_bhi360_logging();
err_t end_bhi360_logging();

// --- Foot Sensor Log File Handle ---
static struct fs_file_t foot_sensor_log_file;
static struct fs_file_t bhi360_log_file;

// --- Sequence number counter for foot sensor log ---
static uint16_t next_foot_sensor_file_sequence = 0;
static uint32_t next_bhi360_file_sequence = 0;

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
    uint32_t max_seq = 0;
    int res;

    fs_dir_t_init(&dirp);

    res = fs_opendir(&dirp, dir_path_param);
    if (res != 0)
    {
        if (res == -ENOENT)
        {
            LOG_DBG("Directory '%s' does not exist yet for prefix '%s'. Starting sequence from 0.", dir_path_param,
                    file_prefix);
            return 0;
        }
        LOG_ERR("Error opening directory '%s' [%d] for prefix '%s'. Cannot reliably determine next sequence.",
                dir_path_param, res, file_prefix);
        return 0;
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
            uint32_t current_seq;
            // Assuming ".pb" extension
            if (std::sscanf(num_str_start, "%u.pb", &current_seq) == 1)
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
    struct fs_mount_t *mp = &lfs_ext_storage_mnt;
    // Mount the File System
    err_t err = mount_file_system(mp);
    if (err != err_t::NO_ERROR)
    {
        LOG_ERR("Error mounting littlefs: %d", (int)err);
        module_set_state(MODULE_STATE_ERROR);
        return err;
    }
    else
    {
        LOG_DBG("File System Mounted at [%s]", mp->mnt_point);

        // Create the data thread
        data_tid = k_thread_create(&data_thread_data, data_stack_area, K_THREAD_STACK_SIZEOF(data_stack_area),
                                   proccess_data, nullptr, nullptr, nullptr, data_priority, 0, K_NO_WAIT);

        LOG_DBG("Data module initialised, thread created.");
        module_set_state(MODULE_STATE_READY);

        return err_t::NO_ERROR;
    }
}

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

    // TODO: Start files from command?
    err_t start_status = start_foot_sensor_logging();
    if (start_status != err_t::NO_ERROR)
    {
        LOG_ERR("Failed to start foot sensor data logging: %d. Data processing halted.", (int)start_status);
        return;
    }

    err_t start_bhi360_status = start_bhi360_logging(); // New: Call for BHI360 logging
    if (start_bhi360_status != err_t::NO_ERROR)
    {
        LOG_ERR("Failed to start BHI360 data logging: %d. Data processing halted.", (int)start_bhi360_status);
        return;
    }

    generic_message_t msg;
    int ret;

    while (true)
    {
        ret = k_msgq_get(&data_msgq, &msg, K_FOREVER);

        if (ret == 0)
        {
            LOG_DBG("Processing message from %s, type: %d", get_sender_name(msg.sender), msg.type);

            switch (msg.type)
            {
                case MSG_TYPE_FOOT_SAMPLES: {
                    const foot_samples_t *foot_data = &msg.data.foot_samples;

                    // Create FootSensorLogMessage
                    sensor_data_messages_FootSensorLogMessage foot_log_msg =
                        sensor_data_messages_FootSensorLogMessage_init_default;
                    foot_log_msg.which_payload = sensor_data_messages_FootSensorLogMessage_foot_sensor_tag;

                    // Populate FootSensorData fields
                    foot_log_msg.payload.foot_sensor.readings.funcs.encode = &encode_foot_sensor_readings_callback;
                    foot_log_msg.payload.foot_sensor.readings.arg = (void *)foot_data;

                    err_t write_status = write_foot_sensor_protobuf_data(&foot_log_msg);
                    if (write_status != err_t::NO_ERROR)
                    {
                        LOG_ERR("Failed to write foot sensor data to file: %d", (int)write_status);
                    }
                    jis_foot_sensor_notify(foot_data);
                    break;
                }

                case MSG_TYPE_BHI360_3D_MAPPING: {
                    const bhi360_3d_mapping_t *bhi360_3d_data = &msg.data.bhi360_3d_mapping;

                    sensor_data_messages_BHI360LogMessage bhi360_log_msg =
                        sensor_data_messages_BHI360LogMessage_init_default;
                    bhi360_log_msg.which_payload = sensor_data_messages_BHI360LogMessage_bhi360_3d_tag;

                    // Populate BHI360 3D data fields
                    bhi360_log_msg.payload.bhi360_3d.has_acceleration = true;
                    bhi360_log_msg.payload.bhi360_3d.acceleration.x = bhi360_3d_data->accel_x;
                    bhi360_log_msg.payload.bhi360_3d.acceleration.y = bhi360_3d_data->accel_y;
                    bhi360_log_msg.payload.bhi360_3d.acceleration.z = bhi360_3d_data->accel_z;

                    bhi360_log_msg.payload.bhi360_3d.has_angular_velocity = true;
                    bhi360_log_msg.payload.bhi360_3d.angular_velocity.x = bhi360_3d_data->gyro_x;
                    bhi360_log_msg.payload.bhi360_3d.angular_velocity.y = bhi360_3d_data->gyro_y;
                    bhi360_log_msg.payload.bhi360_3d.angular_velocity.z = bhi360_3d_data->gyro_z;

                    err_t write_status = write_bhi360_protobuf_data(&bhi360_log_msg);
                    if (write_status != err_t::NO_ERROR)
                    {
                        LOG_ERR("Failed to write BHI360 3D data to file: %d", (int)write_status);
                    }
                    // No BLE notification for BHI360 data yet.
                    break;
                }

                case MSG_TYPE_BHI360_STEP_COUNT: {
                    const bhi360_step_count_t *step_count_data = &msg.data.bhi360_step_count;

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
                    break;
                }

                case MSG_TYPE_COMMAND: {
                    LOG_INF("Received generic command: %s", msg.data.command_str);
                    if (strcmp(msg.data.command_str, "START_LOGGING") == 0)
                    {
                        LOG_INF("Command: Starting all logging.");
                        err_t foot_status = start_foot_sensor_logging();
                        err_t bhi360_status = start_bhi360_logging();

                        if (foot_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to start foot sensor logging from command: %d", (int)foot_status);
                        }
                        if (bhi360_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to start BHI360 logging from command: %d", (int)bhi360_status);
                        }
                    }
                    else if (strcmp(msg.data.command_str, "STOP_LOGGING") == 0)
                    {
                        LOG_INF("Command: Stopping all logging.");
                        err_t foot_status = end_foot_sensor_logging();
                        err_t bhi360_status = end_bhi360_logging();

                        if (foot_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to stop foot sensor logging from command: %d", (int)foot_status);
                        }
                        if (bhi360_status != err_t::NO_ERROR)
                        {
                            LOG_ERR("Failed to stop BHI360 logging from command: %d", (int)bhi360_status);
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
                    err_t delete_status = delete_by_type_and_id(RECORD_HARDWARE_FOOT_SENSOR, msg.data.delete_cmd.id);
                    if (delete_status != err_t::NO_ERROR)
                    {
                        LOG_ERR("Failed to delete foot sensor log file with ID %u: %d", msg.data.delete_cmd.id,
                                (int)delete_status);
                    }
                    else
                    {
                        LOG_INF("Successfully deleted foot sensor log file with ID %u", msg.data.delete_cmd.id);
                    }
                    break;
                }

                case MSG_TYPE_DELETE_BHI360_LOG: { // New: Handle BHI360 delete messages
                    LOG_INF("Received DELETE BHI360 LOG command for ID: %u", msg.data.delete_cmd.id);
                    err_t delete_status = delete_by_type_and_id(RECORD_HARDWARE_BHI360, msg.data.delete_cmd.id);
                    if (delete_status != err_t::NO_ERROR)
                    {
                        LOG_ERR("Failed to delete BHI360 log file with ID %u: %d", msg.data.delete_cmd.id,
                                (int)delete_status);
                    }
                    else
                    {
                        LOG_INF("Successfully deleted BHI360 log file with ID %u", msg.data.delete_cmd.id);
                    }
                    break;
                }

                default:
                    LOG_WRN("Unknown message type received: %d", msg.type);
                    break;
            }
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

    err_t rc = littlefs_flash_erase((uintptr_t)mount->storage_dev);
    if (rc != err_t::NO_ERROR)
    {
        return err_t::DATA_ERROR;
    }

    int ret = fs_mount(mount);
    if (ret != 0)
    {
        LOG_ERR("FS mount failed: %d", ret);
        return err_t::DATA_ERROR;
    }

    // Use hardware_dir_path from data.hpp
    int err_ls = lsdir(hardware_dir_path);
    if (err_ls < 0)
    {
        LOG_ERR("FAIL: lsdir %s: %d", mount->mnt_point, err_ls);
        return err_t::DATA_ERROR;
    }

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
    uint32_t log_id = 0;
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
            std::sscanf(temp_name, "%u", &log_id);
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

uint32_t find_oldest_log_file(const char *file_prefix, record_type_t record_type, char *file_path)
{
    fs_dir_t dirp{};
    static fs_dirent entry{};

    char dir_path_buf[util::max_path_length]{};
    // Only foot sensor logs are handled now
    if (record_type == RECORD_HARDWARE_FOOT_SENSOR)
    {
        std::strcpy(dir_path_buf, hardware_dir_path);
    }
    else
    {
        LOG_ERR("find_oldest_log_file: Unsupported or unimplemented record_type %d", (int)record_type);
        return (uint32_t)(err_t::DATA_ERROR);
    }

    fs_dir_t_init(&dirp);

    int res = 0;

    res = fs_opendir(&dirp, dir_path_buf);
    if (res != 0)
    {
        if ((res == -ENOENT) || (res == -EINVAL) || (res == -EACCES))
        {
            LOG_ERR("Error opening dir %s [%d] (fatal)", dir_path_buf, res);
            return (uint32_t)(err_t::DATA_ERROR);
        }
        LOG_WRN("Open dir failed. No retry.");
    }

    if (res < 0)
    {
        LOG_ERR("Error opening dir %s: %d", dir_path_buf, res);
        return (uint32_t)(err_t::DATA_ERROR);
    }

    res = fs_readdir(&dirp, &entry);
    k_msleep(readdir_micro_sleep_ms);

    if (res != 0 || entry.name[0] == '\0')
    {
        if (res < 0)
        {
            LOG_ERR("Error reading first entry in dir %s [%d]", dir_path_buf, res);
        }
        else
        {
            LOG_INF("Directory %s is empty.", dir_path_buf);
        }
        close_directory(&dirp);
        return (uint32_t)(err_t::DATA_ERROR);
    }

    char file_name[util::max_path_length] = {};
    std::strcpy(file_name, entry.name);

    close_directory(&dirp);

    std::strcpy(file_path, dir_path_buf);
    std::strcat(file_path, "/");
    std::strcat(file_path, file_name);

    char file_id_str[util::max_path_length];
    std::strncpy(file_id_str, file_name, sizeof(file_id_str) - 1);
    file_id_str[sizeof(file_id_str) - 1] = '\0';
    strremove(file_id_str, file_prefix);

    uint32_t record_id = 0;
    if (std::sscanf(file_id_str, "%u.pb", &record_id) != 1)
    {
        LOG_ERR("Failed to parse sequence ID from filename: %s", file_name);
        return (uint32_t)(err_t::DATA_ERROR);
    }

    LOG_DBG("Oldest file found: %s (Sequence ID: %u)", file_path, record_id);
    return record_id;
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
err_t start_foot_sensor_logging()
{
    err_t status = err_t::NO_ERROR;

    // --- Create hardware directory if it doesn't exist ---
    int ret_mkdir = fs_mkdir(hardware_dir_path);
    if (ret_mkdir != 0 && ret_mkdir != -EEXIST)
    {
        LOG_INF("FAIL: fs_mkdir %s: %d", hardware_dir_path, ret_mkdir);
        return err_t::FILE_SYSTEM_ERROR;
    }

    // --- Initialize Foot Sensor Log File ---
    next_foot_sensor_file_sequence = get_next_file_sequence(hardware_dir_path, foot_sensor_file_prefix);
    snprintk(foot_sensor_file_path, sizeof(foot_sensor_file_path), "%s/%s%03u.pb", hardware_dir_path,
             foot_sensor_file_prefix, next_foot_sensor_file_sequence);
    fs_file_t_init(&foot_sensor_log_file);

    int ret_fs_open = fs_open(&foot_sensor_log_file, foot_sensor_file_path, FS_O_CREATE | FS_O_RDWR | FS_O_APPEND);
    if (ret_fs_open != 0)
    {
        LOG_ERR("FAIL: open foot sensor log %s: %d", foot_sensor_file_path, ret_fs_open);
        status = err_t::FILE_SYSTEM_ERROR;
    }
    else
    {
        LOG_INF("Opened new foot sensor log file: %s", foot_sensor_file_path);

        uint8_t buffer[64]; // Sufficient for SensingData message
        sensor_data_messages_FootSensorLogMessage header_msg = sensor_data_messages_FootSensorLogMessage_init_default;
        header_msg.which_payload = sensor_data_messages_FootSensorLogMessage_sensing_data_tag;

        const char firmware_version_str[] = CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION;
        string_callback_arg_t fw_version_arg = {.str = firmware_version_str};
        header_msg.payload.sensing_data.firmware_version.funcs.encode = &encode_string_callback;
        header_msg.payload.sensing_data.firmware_version.arg = &fw_version_arg;
        header_msg.payload.sensing_data.sampling_frequency = 100; // Example: Foot sensor sampling frequency
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
            }
        }
    }

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
                LOG_ERR("Failed to put foot sensor log notification message.");
            }
            else
            {
                LOG_DBG("Foot sensor log notification message sent.");
            }
        }
    }
    else
    {
        LOG_WRN("Foot sensor log file was not open, skipping close/end session.");
    }

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
err_t start_bhi360_logging()
{
    err_t status = err_t::NO_ERROR;

    // --- Create hardware directory if it doesn't exist (already handled by foot sensor) ---
    // This is redundant as foot sensor logging already creates it.
    // int ret_mkdir = fs_mkdir(hardware_dir_path);
    // if (ret_mkdir != 0 && ret_mkdir != -EEXIST)
    // {
    //     LOG_ERR("FAIL: fs_mkdir %s: %d", hardware_dir_path, ret_mkdir);
    //     return err_t::FILE_SYSTEM_ERROR;
    // }

    // --- Initialize BHI360 Log File ---
    next_bhi360_file_sequence = get_next_file_sequence(hardware_dir_path, bhi360_file_prefix);
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

        uint8_t buffer[64]; // Sufficient for SensingData message for BHI360
        sensor_data_messages_BHI360LogMessage header_msg = sensor_data_messages_BHI360LogMessage_init_default;
        header_msg.which_payload = sensor_data_messages_BHI360LogMessage_sensing_data_tag;

        const char firmware_version_str[] = CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION;
        string_callback_arg_t fw_version_arg = {.str = firmware_version_str};
        header_msg.payload.sensing_data.firmware_version.funcs.encode = &encode_string_callback;
        header_msg.payload.sensing_data.firmware_version.arg = &fw_version_arg;
        header_msg.payload.sensing_data.sampling_frequency = 100; // Example: BHI360 sampling frequency
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
            // Notify Bluetooth for BHI360 Log
            generic_message_t bhi360_msg;
            bhi360_msg.sender = SENDER_DATA;
            bhi360_msg.type = MSG_TYPE_NEW_BHI360_LOG_FILE; // New specific type for BHI360
            bhi360_msg.data.new_hardware_log_file.file_sequence_id = next_bhi360_file_sequence;
            strncpy(bhi360_msg.data.new_hardware_log_file.file_path, bhi360_file_path,
                    sizeof(bhi360_msg.data.new_hardware_log_file.file_path) - 1);
            bhi360_msg.data.new_hardware_log_file
                .file_path[sizeof(bhi360_msg.data.new_hardware_log_file.file_path) - 1] = '\0';

            if (k_msgq_put(&bluetooth_msgq, &bhi360_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to put BHI360 log notification message.");
            }
            else
            {
                LOG_DBG("BHI360 log notification message sent.");
            }
        }
    }
    else
    {
        LOG_WRN("BHI360 log file was not open, skipping close/end session.");
    }

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
    uint8_t buffer[64]; // Ensure this buffer is large enough for any FootSensorLogMessage
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    bool status = pb_encode(&stream, sensor_data_messages_FootSensorLogMessage_fields, sensor_msg);
    if (!status)
    {
        LOG_ERR("Foot sensor Protobuf encode error: %s", stream.errmsg);
        return err_t::PROTO_ENCODE_ERROR;
    }

    int ret = fs_write(&foot_sensor_log_file, buffer, stream.bytes_written);
    if (ret < 0)
    {
        LOG_ERR("Foot sensor fs_write error: %d", ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    ret = fs_sync(&foot_sensor_log_file);
    if (ret != 0)
    {
        LOG_ERR("FAIL: sync foot sensor data %s: %d", foot_sensor_file_path, ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    LOG_DBG("Foot sensor data written to file (%u bytes).", stream.bytes_written);
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
    // BHI360 messages can be larger due to multiple float fields, ensure buffer is sufficient
    // BHI360_3D_Data_size is typically around 45 bytes, SessionEnd is smaller. 64 bytes is generally safe.
    uint8_t buffer[64];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    bool status = pb_encode(&stream, sensor_data_messages_BHI360LogMessage_fields, sensor_msg);
    if (!status)
    {
        LOG_ERR("BHI360 Protobuf encode error: %s", stream.errmsg);
        return err_t::PROTO_ENCODE_ERROR;
    }

    int ret = fs_write(&bhi360_log_file, buffer, stream.bytes_written);
    if (ret < 0)
    {
        LOG_ERR("BHI360 fs_write error: %d", ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    ret = fs_sync(&bhi360_log_file);
    if (ret != 0)
    {
        LOG_ERR("FAIL: sync BHI360 data %s: %d", bhi360_file_path, ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    LOG_DBG("BHI360 data written to file (%u bytes).", stream.bytes_written);
    return err_t::NO_ERROR;
}

err_t type_and_id_to_path(record_type_t type, uint32_t id, char path[])
{
    switch (type)
    {
        case RECORD_HARDWARE_FOOT_SENSOR:
            snprintk(path, util::max_path_length, "%s/%s%03u.pb", hardware_dir_path, foot_sensor_file_prefix, id);
            break;
        case RECORD_HARDWARE_BHI360: // New: Case for BHI360 files
            snprintk(path, util::max_path_length, "%s/%s%03u.pb", hardware_dir_path, bhi360_file_prefix, id);
            break;
        default:
            LOG_ERR("File path generation not implemented for type: %d", (int)type);
            return err_t::DATA_ERROR;
    }

    return err_t::NO_ERROR;
}

err_t delete_by_type_and_id(record_type_t type, uint32_t id)
{
    char path_to_delete[util::max_path_length] = {};

    err_t err = type_and_id_to_path(type, id, path_to_delete);
    if (err != err_t::NO_ERROR)
    {
        return err;
    }
    LOG_WRN("Attempting to delete file: %s", path_to_delete);
    int ret = fs_unlink(path_to_delete);
    if (ret != 0)
    {
        LOG_ERR("Failed to delete file '%s'. Error %d", path_to_delete, ret);
        return err_t::FILE_SYSTEM_NO_FILES;
    }
    LOG_INF("Successfully deleted file: %s", path_to_delete);

    char oldest_file_path[util::max_path_length];
    uint32_t oldest_file_seq = 0;

    if (type == RECORD_HARDWARE_FOOT_SENSOR)
    {
        oldest_file_seq = find_oldest_log_file(foot_sensor_file_prefix, RECORD_HARDWARE_FOOT_SENSOR, oldest_file_path);
        if (oldest_file_seq == (uint32_t)err_t::DATA_ERROR)
        {
            LOG_INF("Foot sensor directory empty or error finding oldest file after deletion.");
        }
        else
        {
            type_and_id_to_path(RECORD_HARDWARE_FOOT_SENSOR, oldest_file_seq, oldest_file_path);
            LOG_INF("New oldest foot sensor log file: %s (Seq: %u)", oldest_file_path, oldest_file_seq);
        }
    }
    return err_t::NO_ERROR;
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

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);

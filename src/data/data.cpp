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
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/bluetooth_state_event.h>
#include <events/data_event.h>
#include <util.hpp>

#include <data.hpp>

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

int delete_oldest_log_files(const char *file_prefix, const char *dir_path);
err_t write_metadata(uint8_t *buffer, size_t length, int64_t epoch_filename);
err_t delete_by_type_and_id(record_type_t type, uint32_t id);
err_t type_and_id_to_path(record_type_t type, uint32_t id, char path[]);

// Thread
static struct k_thread data_thread_data;
constexpr int data_stack_size = CONFIG_DATA_MODULE_STACK_SIZE;
constexpr int data_priority = CONFIG_DATA_MODULE_PRIORITY;
void proccess_data(void * /*unused*/, void * /*unused*/, void * /*unused*/);

K_THREAD_STACK_DEFINE(data_stack_area, data_stack_size);
k_tid_t data_tid;

constexpr char dir_path[] = "/lfs1";

void proccess_data();

/**
 * @brief Initialise the file system
 *
 * @return int
 *
 */
static err_t data_init()
{
    struct fs_mount_t *mp = &lfs_ext_storage_mnt;
    // Mount the File System
    err_t err = mount_file_system(mp);
    if (err != err_t::NO_ERROR) // to implement errors
    {
        LOG_ERR("Error mounting littlefs");
        module_set_state(MODULE_STATE_ERROR);
        return err;
    }
    else
    {
        LOG_DBG("System Mounted [%s]", mp->mnt_point);

        // Create the data thread
        data_tid = k_thread_create(&data_thread_data, data_stack_area, K_THREAD_STACK_SIZEOF(data_stack_area),
                                   proccess_data, nullptr, nullptr, nullptr, data_priority, 0, K_NO_WAIT);

        LOG_DBG("Data module initialised");
        module_set_state(MODULE_STATE_READY);

        return err_t::NO_ERROR; // to implement errors
    }
}

static int lsdir(const char *path)
{
    int res;
    struct fs_dir_t dirp;
    static struct fs_dirent entry;

    LOG_WRN("Starting Directories List");

    fs_dir_t_init(&dirp);

    /* Verify fs_opendir() */
    res = fs_opendir(&dirp, path);
    if (res)
    {
        LOG_ERR("Error opening dir %s [%d]\n", path, res);
        return res;
    }

    LOG_INF("\nListing dir %s ...\n", path);
    for (;;)
    {
        /* Verify fs_readdir() */
        res = fs_readdir(&dirp, &entry);

        /* entry.name[0] == 0 means end-of-dir */
        if (res || entry.name[0] == 0)
        {
            if (res < 0)
            {
                LOG_ERR("Error reading dir [%d]\n", res);
            }
            break;
        }

        if (entry.type == FS_DIR_ENTRY_DIR)
        {
            LOG_INF("[DIR ] %s\n", entry.name);
        }
        else
        {
            LOG_INF("[FILE] %s (size = %zu)\n", entry.name, entry.size);
        }
    }

    /* Verify fs_closedir() */
    fs_closedir(&dirp);

    return res;
}

void proccess_data(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{
    k_thread_name_set(data_tid, "data");

    while (true)
    {
        // TODO develop threaad
        k_msleep(data_wait_timer);
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
        LOG_ERR("FS mount failed");
        return err_t::DATA_ERROR;
    }

    int err = lsdir(mount->mnt_point);
    if (err < 0)
    {
        LOG_ERR("FAIL: lsdir %s: %d\n", mount->mnt_point, err);
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
        LOG_ERR("FAIL: unable to find flash area %u: %d\n", id, rc);
        return err_t::DATA_ERROR;
    }

    LOG_WRN("Area %u at 0x%x on %s for %u bytes\n", id, (unsigned int)pfa->fa_off, pfa->fa_dev->name,
            (unsigned int)pfa->fa_size);

    /* Optional wipe flash contents */
    if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE))
    {
        rc = flash_area_flatten(pfa, 0, pfa->fa_size);
        LOG_ERR("Erasing flash area ... %d", rc);
        return err_t::DATA_ERROR;
    }

    flash_area_close(pfa);
    return err_t::NO_ERROR;
}

int delete_oldest_log_files(const char *file_prefix, const char *dir_path)
{
    uint32_t log_id = 0;
    int res = 0;
    fs_dir_t dirp;
    static fs_dirent entry;
    char last_name[util::max_path_length];

    fs_dir_t_init(&dirp);

    for (uint8_t i = 0; i < max_retries; i++)
    {
        res = fs_opendir(&dirp, dir_path);
        if (!res)
        {
            break;
        }
        else if (res == -EINVAL)
        {
            LOG_ERR("Error opening dir %s [%d]\n", dir_path, res);
            return 0;
        }
        LOG_WRN("Open dir failed, retrying....");
    }

    if (res < 0)
    {
        LOG_ERR("Error opening dir %s [%d]\n", dir_path, res);
        return 0;
    }

    // Initialize last_id and trailing_id
    uint16_t last_file_id = 0;
    uint16_t trailing_file_id = 0;
    bool trailing_file_id_active = false;

    for (;;)
    {
        res = fs_readdir(&dirp, &entry);
        k_msleep(2);

        if (res || entry.name[0] == 0)
        {
            if (res < 0)
            {
                LOG_WRN("No files in this dir [%d]\n", res);
            }
            break;
        }

        // Only activate the trailing_id after the last_id has advanced 100 steps
        if (last_file_id >= 100 && !trailing_file_id_active)
        {
            trailing_file_id_active = true;
            trailing_file_id = 0; // trailing_id now starts moving
        }

        // Move the last_id one step ahead
        last_file_id++;

        if (trailing_file_id_active)
        {
            // Move the trailing_id one step ahead
            trailing_file_id++;
        }

        LOG_DBG("trailing_id: %d, last_id: %d", trailing_file_id, last_file_id);

        // Convert file name into uint32_t to get the log ID
        std::strncpy(last_name, entry.name, sizeof(last_name));
        strremove(last_name, file_prefix);
        std::sscanf(last_name, "%u", &log_id);
    }

    // Now trailing_file_id points to the first file that's 100 entries from the end
    if (trailing_file_id > 0)
    {
        fs_closedir(&dirp);
        fs_opendir(&dirp, dir_path);

        // Advance to the trailing_id position to delete x oldest files
        for (uint16_t i = 0; i < trailing_file_id; i++)
        {
            res = fs_readdir(&dirp, &entry);
            k_msleep(2);

            if (res || entry.name[0] == 0)
            {
                break;
            }

            // Build path and delete the file
            char file_path[util::max_path_length];
            snprintk(file_path, sizeof(file_path), "%s/%s", dir_path, entry.name);
            fs_unlink(file_path);
            LOG_DBG("Unlinking %s", file_path);
        }
    }

    fs_closedir(&dirp);
    return 0;
}

err_t close_directory(fs_dir_t *dirp)
{
    for (uint8_t i = 0; i < max_retries; i++)
    {
        int res = fs_closedir(dirp);
        if (!res)
        {
            break;
        }
        else
        {
            // retrying doesnt work here, exit
            if (res == EINVAL)
            {
                LOG_ERR("Failed to close\n");
                return err_t::FILE_SYSTEM_ERROR;
            }
            LOG_WRN("Close dir failed, retrying....");
        }
    }
    return err_t::NO_ERROR;
}

uint32_t find_oldest_log_file(const char *file_prefix, record_type_t record_type, char *file_path)
{
    fs_dir_t dirp{};
    static fs_dirent entry{};

    char dir_path[util::max_path_length]{};
    if (record_type == RECORD_METADATA)
    {
        std::strcpy(dir_path, metadata_dir_path);
    }
    else
    {
        std::strcpy(dir_path, hardware_dir_path);
    }

    fs_dir_t_init(&dirp);

    /* Verify fs_opendir() */
    int res = 0;
    for (uint8_t retry_cnt = 0; retry_cnt < max_retries; retry_cnt++)
    {
        res = fs_opendir(&dirp, dir_path);
        if (!res)
        {
            break;
        }
        else
        {
            // retry wont work for these errors, so exit the loop
            if ((res == -ENOENT) || (res == -EINVAL) || (res == -EACCES))
            {
                LOG_ERR("Error opening dir %s [%d]\n", dir_path, res);
                return (uint32_t)(err_t::DATA_ERROR);
            }
            LOG_WRN("Open dir failed, retrying....");
        }
    }

    if (res < 0)
    {
        LOG_ERR("Error opening dir %s [%d]\n", dir_path, res);
        return (uint32_t)(err_t::DATA_ERROR);
    }

    while (true)
    {
        res = fs_readdir(&dirp, &entry);
        k_msleep(readdir_micro_sleep_ms);
        if ((res == -ENOENT) || (res == -EINVAL) || (res == -EACCES))
        {
            LOG_ERR("Error reading dir %s [%d]\n", dir_path, res);
            close_directory(&dirp); // No need to check return as we are returning an error anyway
            return (uint32_t)(err_t::DATA_ERROR);
        }
        else if (res != 0)
        {
            close_directory(&dirp); // No need to check return as we are returning an error anyway
            return (uint32_t)(err_t::DATA_ERROR);
        }

        if (entry.name[0] == 0)
        {
            err_t err = close_directory(&dirp);
            if (err != err_t::NO_ERROR)
            {
                return (uint32_t)(err_t::DATA_ERROR);
            }
            return (uint32_t)(err_t::DATA_ERROR);
        }

        else
        {
            break;
        }
    }

    char file_name[util::max_path_length] = {};

    // Get the first (oldest) item
    std::strcpy(file_name, entry.name);

    err_t err = close_directory(&dirp);
    if (err != err_t::NO_ERROR)
    {
        return (uint32_t)(err_t::DATA_ERROR);
    }

    if (res < 0)
    {
        LOG_ERR("Failed to close the directory after retries: err [%d]", res);
        return (uint32_t)(err_t::DATA_ERROR);
        ;
    }

    // Create full file path
    std::strcpy(file_path, dir_path);
    std::strcat(file_path, "/");

    if (std::strcmp(file_name, hardware_file_path) == 0)
    {
        LOG_INF("Only the active hardware record present, returning empty");
        return (uint32_t)(err_t::DATA_ERROR);
    }
    else
    {
        std::strcat(file_path, file_name);
    }

    // Extract ID String
    char file_id_str[util::max_path_length];
    std::strcpy(file_id_str, file_name);
    strremove(file_id_str, file_prefix);

    // Convert ID String to int
    uint32_t record_id = 0;
    std::sscanf(file_id_str, "%u", &record_id);

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

err_t write_metadata(uint8_t *buffer, size_t length, int64_t epoch_filename)
{
    snprintk(metadata_file_path, sizeof(metadata_file_path), "%s/%s%u", metadata_dir_path, metadata_file_prefix,
             static_cast<uint32_t>(epoch_filename));

    struct fs_file_t file
    {
    };

    fs_file_t_init(&file);

    LOG_DBG("Metadata writing: %s", metadata_file_path);

    int ret = 0;
    for (uint8_t i = 0; i < max_retries; i++)
    {
        ret = fs_open(&file, metadata_file_path, FS_O_CREATE | FS_O_RDWR);
        if (!ret)
        {
            break;
        }
        else
        {
            // retry wont work for these errors, so exit the loop
            if ((ret == -ENOENT) || (ret == -EINVAL))
            {
                LOG_ERR("FAIL: open %s: %d", metadata_file_path, ret);
                return err_t::FILE_SYSTEM_ERROR;
            }
            LOG_WRN("Failed to open, retryig...");
        }
        k_msleep(retry_sleep_timer_ms);
    }
    // if fails on retry, report error.
    if (ret < 0)
    {
        LOG_ERR("FAIL: open %s: %d", metadata_file_path, ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    for (uint8_t i = 0; i < max_retries; i++)
    {
        ret = fs_write(&file, buffer, length);
        if (ret >= 0)
        {
            break;
        }
        else
        {
            // retry wont work for these errors, so exit the loop
            if ((ret == -ENOENT) || (ret == -EINVAL) || (ret == -EACCES))
            {
                LOG_ERR("FAIL: write %s: [rd:%d]", metadata_file_path, ret);
                return err_t::FILE_SYSTEM_ERROR;
            }
            LOG_WRN("Failed to write, retrying...");
        }
        k_msleep(retry_sleep_timer_ms);
    }
    if (ret < 0)
    {
        LOG_ERR("FAIL: write %s: [rd:%d]", metadata_file_path, ret);
        return err_t::FILE_SYSTEM_ERROR;
    }

    for (uint8_t i = 0; i < max_retries; i++)
    {
        ret = fs_close(&file);
        if (!ret)
        {
            break;
        }
        else
        {
            // retry wont work for these errors, so exit the loop
            if ((ret == -ENOENT) || (ret == -EINVAL) || (ret == -EACCES))
            {
                LOG_ERR("FAIL: close %s: %d", metadata_file_path, ret);
                return err_t::FILE_SYSTEM_ERROR;
            }
            LOG_WRN("Failed to close, retrying...");
        }
        k_msleep(retry_sleep_timer_ms);
    }
    if (ret < 0)
    {
        LOG_ERR("FAIL: close %s: %d", metadata_file_path, ret);
        return err_t::FILE_SYSTEM_ERROR;
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
            if (data_init() == err_t::NO_ERROR) // Develop errors
            {
                LOG_DBG("Data device initialization successful");
            }
            else
            {
                LOG_ERR("Data device initialization failed");
            }
        }

        return false;
    }
    return false;
}

err_t type_and_id_to_path(record_type_t type, uint32_t id, char path[])
{
    char buf[util::max_path_length];

    std::snprintf(buf, sizeof(buf), "%u", id);

    switch (type)
    {
        case RECORD_METADATA:
            snprintk(path, util::max_path_length, "%s/%s%.*s", metadata_dir_path, metadata_file_prefix,
                     util::max_path_length, buf);
            break;
        case RECORD_HARDWARE:
            snprintk(path, util::max_path_length, "%s/%s%.*s", hardware_dir_path, hardware_file_prefix,
                     util::max_path_length, buf);
            break;
        default:
            LOG_ERR("Not implemented type");
            return err_t::DATA_ERROR;
            break;
    }

    return err_t::NO_ERROR;
}

err_t delete_by_type_and_id(record_type_t type, uint32_t id)
{
    char path[util::max_path_length] = {};
    char empty_path[util::max_path_length] = "";

    err_t err = type_and_id_to_path(type, id, path);
    if (err != err_t::NO_ERROR)
    {
        return err;
    }
    LOG_WRN("Deleting file =%s", path);
    int ret = fs_unlink(path);
    if (ret != 0)
    {
        LOG_ERR("Failed to delete file. Error %d", ret);
        return err_t::FILE_SYSTEM_NO_FILES;
    }

    if (type == RECORD_HARDWARE)
    {
        char hw_path[util::max_path_length];

        uint32_t log_ret = find_oldest_log_file(hardware_file_prefix, RECORD_HARDWARE, hw_path);
        if (log_ret < 0)
        {
            LOG_ERR("Hardware directory empty");
            // new_log_available_bluetooth_notify(empty_path_id, empty_path, type);
        }
        else
        {
            LOG_INF("New oldest hardware path: %s", hw_path);
            //   new_log_available_bluetooth_notify(log_ret.value(), hw_path, type);
        }
    }
    else if (type == RECORD_METADATA)
    {
        char meta_path[util::max_path_length];

        uint32_t log_ret = find_oldest_log_file(metadata_file_prefix, RECORD_METADATA, meta_path);
        if (!log_ret)
        {

            LOG_INF("Metadata directory empty");
            // new_log_available_bluetooth_notify(empty_path_id, empty_path, type);
        }
        else
        {
            LOG_INF("New oldest metadata path: %s", meta_path);
            //   new_log_available_bluetooth_notify(log_ret.value(), meta_path, type);
        }
    }
    return err_t::NO_ERROR;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);

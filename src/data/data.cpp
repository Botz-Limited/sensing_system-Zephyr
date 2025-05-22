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

#include <caf/events/module_state_event.h>

#include <data.hpp>

LOG_MODULE_REGISTER(MODULE, CONFIG_DATA_MODULE_LOG_LEVEL); // NOLINT

// Define File System parameters
#define STORAGE_PARTITION_LABEL storage_partition
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION_LABEL)
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);

static int mount_file_system(struct fs_mount_t *mount);

// Thread
static struct k_thread data_thread_data;
constexpr int data_stack_size = CONFIG_DATA_MODULE_STACK_SIZE;
constexpr int data_priority = CONFIG_DATA_MODULE_PRIORITY;
void proccess_data(void * /*unused*/, void * /*unused*/, void * /*unused*/);

K_THREAD_STACK_DEFINE(data_stack_area, data_stack_size);
k_tid_t data_tid;

constexpr char dir_path[] = "/lfs1";

void proccess_data();

struct fs_mount_t fs_mnt;
struct fs_mount_t *mp = &fs_mnt;

/**
 * @brief Initialise the file system
 *
 * @return int
 *
 */
int data_init()
{
    // Mount the File System
    int err = mount_file_system(mp);
    if (err != 0) // to implement errors
    {
        LOG_ERR("Error mounting littlefs");
        return err;
    }
    else
    {
        LOG_DBG("System Mounted [%s]", mp->mnt_point);

        // Create the data thread
        data_tid = k_thread_create(&data_thread_data, data_stack_area, K_THREAD_STACK_SIZEOF(data_stack_area),
                                   proccess_data, nullptr, nullptr, nullptr, data_priority, 0, K_NO_WAIT);
        k_thread_name_set(data_tid, "Data"); // sets the name of the thread

        return 0; // to implement errors
    }
}

void proccess_data(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{
    k_thread_name_set(data_tid, "data");
    LOG_DBG("Data module initialised");

    while (true)
    {
        LOG_INF("Data Task");
        k_msleep(data_wait_timer);
    }
}

int mount_file_system(struct fs_mount_t *mount)
{
    const struct device *flash_device = DEVICE_DT_GET(DT_NODELABEL(mx25r64));

    if (!device_is_ready(flash_device))
    {
        LOG_ERR("External flash device isn't ready");
        return -1;
    }

    mount->type = FS_LITTLEFS;
    mount->fs_data = &storage;
    mount->storage_dev = reinterpret_cast<void *>(FIXED_PARTITION_ID(littlefs_storage));
    mount->mnt_point = CONFIG_FILE_SYSTEM_MOUNT;
    /* Do not mount if auto-mount has been enabled */

    int ret = fs_mount(mount);
    if (ret != 0)
    {
        return -1;
    }

    return 0;
}

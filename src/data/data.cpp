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
#include <events/bluetooth_state_event.h>
#include <events/app_state_event.h>
#include <events/data_event.h>
#include <app_event_manager.h>

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


static int mount_file_system(struct fs_mount_t *mount);
static int littlefs_flash_erase(unsigned int id);

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
static int data_init()
{
    struct fs_mount_t *mp = &lfs_ext_storage_mnt;
    // Mount the File System
    int err = mount_file_system(mp);
    if (err != 0) // to implement errors
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

        set_data_event(DATA_NO_ERROR);

        LOG_DBG("Data module initialised");
        module_set_state(MODULE_STATE_READY);
        

        return 0; // to implement errors
    }
}

void proccess_data(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{
    k_thread_name_set(data_tid, "data");

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

    int rc = littlefs_flash_erase((uintptr_t)mount->storage_dev);
	if (rc < 0) {
        return -1;
	}

    int ret = fs_mount(mount);
    if (ret != 0)
    {
        return -1;
    }

    return 0;
}

static int littlefs_flash_erase(unsigned int id)
{
	const struct flash_area *pfa;
	int rc;

	rc = flash_area_open(id, &pfa);
	if (rc < 0) {
		LOG_ERR("FAIL: unable to find flash area %u: %d\n",
			id, rc);
		return rc;
	}

	LOG_WRN("Area %u at 0x%x on %s for %u bytes\n",
		   id, (unsigned int)pfa->fa_off, pfa->fa_dev->name,
		   (unsigned int)pfa->fa_size);

	/* Optional wipe flash contents */
	if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
		rc = flash_area_flatten(pfa, 0, pfa->fa_size);
		LOG_ERR("Erasing flash area ... %d", rc);
	}

	flash_area_close(pfa);
	return rc;
}


static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        const struct module_state_event *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(app), MODULE_STATE_READY))
        {
            if (data_init() == 0) //Develop errors
            {
                LOG_DBG("Data device initialization successful");
            }
            else
            {
                LOG_ERR("Data device initialization failed");
                set_data_event(DATA_NO_ERROR);
            }
        }

        return false;
    }

}



APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);

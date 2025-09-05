/**
 * @file data.cpp
 * @brief Data logging and storage module with work queue architecture
 * @version 1.0.0
 * @date June 2025
 *
 * @copyright Botz Innovation 2025
 */

#define MODULE data_sd

#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/storage/disk_access.h>
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
#include <data_sd.hpp>
#include <foot_sensor_messages.pb.h>
#include <status_codes.h>

#include "../../include/events/foot_sensor_event.h"
#include "../../include/events/motion_sensor_event.h"

 #if IS_ENABLED(CONFIG_LAB_VERSION)

LOG_MODULE_REGISTER(MODULE, CONFIG_DATA_SD_MODULE_LOG_LEVEL);

static void data_sd_init(void);
static void data_sd_thread_fn(void *arg1, void *arg2, void *arg3);
static err_t mount_sdcard_file_system(struct fs_mount_t *mount);

// Thread Configuration
static constexpr int data_sd_stack_size = CONFIG_DATA_SD_MODULE_STACK_SIZE;
static constexpr int data_sd_priority = CONFIG_DATA_SD_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(data_sd_stack_area, data_sd_stack_size);
static struct k_thread data_sd_thread_data;
static k_tid_t data_sd_tid;


static struct fs_mount_t lfs_sd_storage_mnt = {};

static void data_sd_init(void)
{
    LOG_INF("Initializing data module");

    // Initialize the fs_mount_t structure
    lfs_sd_storage_mnt.type = FS_LITTLEFS;
    lfs_sd_storage_mnt.fs_data = &lfs_sd_storage_mnt;
    lfs_sd_storage_mnt.mnt_point = "/sd";
    lfs_sd_storage_mnt.flags = FS_MOUNT_FLAG_USE_DISK_ACCESS;

    lfs_sd_storage_mnt.storage_dev = (void *)DEVICE_DT_GET(DT_NODELABEL(sdhc));

    // Mount the file system
    err_t err = mount_sdcard_file_system(&lfs_sd_storage_mnt);
    if (err != err_t::NO_ERROR)
    {
        LOG_ERR("Error mounting littlefs: %d", (int)err);

        LOG_WRN("Filesystem not available - system will operate without logging "
                "capability");
    }
    else
    {

        LOG_INF("File System Mounted at %s", lfs_sd_storage_mnt.mnt_point);
    }

  

    // Create the message processing thread
    data_sd_tid = k_thread_create(&data_sd_thread_data, data_sd_stack_area, K_THREAD_STACK_SIZEOF(data_sd_stack_area),
                               data_sd_thread_fn, NULL, NULL, NULL, data_sd_priority, 0, K_NO_WAIT);

    k_thread_name_set(data_sd_tid, "data");


    module_set_state(MODULE_STATE_READY);
    LOG_INF("Data SD module initialized");
}

static err_t mount_sdcard_file_system(struct fs_mount_t *mount)
{
    const struct device *sd_card_device = NULL;

#if DT_NODE_EXISTS(DT_NODELABEL(sdhc))
    sd_card_device = DEVICE_DT_GET(DT_NODELABEL(sdhc));
    if (device_is_ready(sd_card_device)) {
        LOG_INF("Using SD card (on SPI2)");
    } else {
        LOG_ERR("SD card device isn't ready");
        return err_t::DATA_ERROR;
    }
#else
    LOG_ERR("SD card device node not found");
    return err_t::DATA_ERROR;
#endif

    int ret = fs_mount(mount);
    if (ret != 0) {
        LOG_ERR("SD card FS mount failed: %d", ret);

        // Optional: Attempt to format the card if mount fails
        if (ret == -ENOENT) {
            LOG_INF("File system not found, formatting...");
            ret = fs_mkfs(mount->type, (uintptr_t)mount->storage_dev, NULL, 0);
            if (ret == 0) {
                LOG_INF("SD card formatted successfully. Reboot to use.");
                return err_t::DATA_ERROR; // Exit and reboot
            } else {
                LOG_ERR("Failed to format SD card: %d", ret);
                return err_t::DATA_ERROR;
            }
        }
        return err_t::DATA_ERROR;
    }

    LOG_INF("SD card mounted at %s", mount->mnt_point);
    return err_t::NO_ERROR;
}

static void data_sd_thread_fn(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    generic_message_t msg;


    while (true)
    {
        // Wait for messages
        int ret = k_msgq_get(&data_sd_msgq, &msg, K_FOREVER);

        if (ret == 0)
        {
            // Queue different work based on message type
            switch (msg.type)
            {

                case MSG_TYPE_ERASE_EXTERNAL_FLASH:
                    // Queue work to erase external flash
             /*       if (!k_work_is_pending(&process_erase_flash_work))
                    {
                        k_work_submit_to_queue(&data_work_q, &process_erase_flash_work);
                    }
                    else
                    {
                        LOG_WRN("Erase flash work already pending");
                    } */
                    break;
                case MSG_TYPE_FOOT_SAMPLES: {
                    // Queue work to log foot sensor data
                }    break;
                
                case MSG_TYPE_BHI360_LOG_RECORD: {
                    // Queue work to log BHI360 data
                }    break; 

                default:
                    LOG_WRN("Received unsupported message type %d  from sender %d", msg.type, msg.sender);
                    break;
            }
        }
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
            data_sd_init();
        }
        return false;
    }
    else if (is_app_state_event(aeh))
    {
        const struct app_state_event *event = cast_app_state_event(aeh);
        if (event->state == APP_STATE_SHUTDOWN_PREPARING)
        {
           
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);

#endif // CONFIG_LAB_VERSION
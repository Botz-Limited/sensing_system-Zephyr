/**
 * @file app.cpp
 * @author Giorgio Guglielmino
 * @version 1.0.1
 * @date 2025-05-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#define MODULE app

/*************************** INCLUDE HEADERS ********************************/
#include <cstring>
#include <time.h>
#include <variant>

#include <cstdio>
#include <cstring>
#include <time.h>

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <events/app_state_event.h>
#include <events/bluetooth_state_event.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt.h>
#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt_callbacks.h>
#include <zephyr/sys/timeutil.h>

#include <app.hpp>
#include <app_version.h>
#include <errors.hpp>
#include <zephyr/sys/reboot.h>

#if IS_ENABLED(CONFIG_SECONDARY_DEVICE)
#include "../bluetooth/ble_d2d_tx.hpp"
#endif

LOG_MODULE_REGISTER(MODULE, CONFIG_APP_MODULE_LOG_LEVEL); // NOLINT

// FOTA reset work
static struct k_work_delayable fota_reset_work;

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

// FOTA progress tracking structure
struct fota_progress_state {
    bool is_active;
    uint32_t total_size;
    uint32_t bytes_received;
    uint8_t percent_complete;
    uint32_t chunks_received;
    uint32_t chunks_written;
    uint8_t status; // 0=idle, 1=in_progress, 2=pending, 3=confirmed, 4=error
    int32_t error_code;
    uint32_t last_reported_bytes;
    uint32_t last_completed_size; // Size of last successful FOTA
    bool is_first_image; // True for first image in multi-image update
    uint32_t app_core_size; // Learned size of app core
    uint32_t net_core_size; // Learned size of net core
} fota_progress = {false, 0, 0, 0, 0, 0, 0, 0, 0, 0, true, 0, 0};

void initializing_entry();

// FOTA reset handler
static void fota_reset_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    LOG_INF("=== FOTA UPDATE COMPLETE ===");
    LOG_INF("Preparing to reset system to apply new firmware...");
    
    // Send final status update before reset
    generic_message_t msg;
    msg.sender = SENDER_NONE;
    msg.type = MSG_TYPE_FOTA_PROGRESS;
    msg.data.fota_progress.is_active = true;
    msg.data.fota_progress.status = 3; // confirmed (about to reset)
    msg.data.fota_progress.percent_complete = 100;
    msg.data.fota_progress.bytes_received = fota_progress.bytes_received;
    msg.data.fota_progress.total_size = fota_progress.bytes_received;
    msg.data.fota_progress.error_code = 0;
    
    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send final FOTA status");
    } else {
        LOG_INF("Final FOTA status sent to Bluetooth");
    }
    
    // Give more time for the message to be sent and logged
    k_sleep(K_MSEC(500));
    
    LOG_INF("=== RESETTING SYSTEM NOW ===");
    sys_reboot(SYS_REBOOT_WARM);
}

K_MSGQ_DEFINE(bluetooth_msgq, MSG_QUEUE_MESSAGE_SIZE, MSG_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(data_msgq, MSG_QUEUE_MESSAGE_SIZE, MSG_QUEUE_DEPTH, 4);
K_MSGQ_DEFINE(motion_sensor_msgq, MSG_QUEUE_MESSAGE_SIZE, MSG_QUEUE_DEPTH, 4);

// Add other slab definitions here as needed

/********************************** APP THREAD ********************************/
static constexpr int app_stack_size = CONFIG_APP_MODULE_STACK_SIZE;
static constexpr int app_priority = CONFIG_APP_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(app_stack_area, app_stack_size);
static struct k_thread app_thread_data;
static k_tid_t app_tid;
void app_entry(void * /*unused*/, void * /*unused*/, void * /*unused*/);


// FOTA callback handlers
mgmt_cb_return fota_started_callback(uint32_t event, enum mgmt_cb_return prev_status, 
                                    int32_t *rc, uint16_t *group,
                                    bool *abort_more, void *data, size_t data_size)
{
    ARG_UNUSED(event);
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);
    
    LOG_INF("FOTA Started!");
    
    // Reset progress tracking but keep learned sizes
    uint32_t saved_app_size = fota_progress.app_core_size;
    uint32_t saved_net_size = fota_progress.net_core_size;
    bool was_first = fota_progress.is_first_image;
    
    memset(&fota_progress, 0, sizeof(fota_progress));
    
    // Restore learned sizes
    fota_progress.app_core_size = saved_app_size;
    fota_progress.net_core_size = saved_net_size;
    fota_progress.is_active = true;
    fota_progress.status = 1; // in_progress
    
    // For multi-image updates: first update is app core, second is net core
    // If this is starting and we just completed a large update, this must be net core
    if (was_first && saved_app_size > 500000) {
        fota_progress.is_first_image = false; // This is network core
        LOG_INF("Starting network core update (second image)");
    } else {
        fota_progress.is_first_image = true; // This is app core
        LOG_INF("Starting application core update (first image)");
    }
    
    // Send FOTA progress message to Bluetooth thread with zeroed values
    generic_message_t msg;
    msg.sender = SENDER_NONE;
    msg.type = MSG_TYPE_FOTA_PROGRESS;
    msg.data.fota_progress.is_active = true;
    msg.data.fota_progress.status = 1;
    msg.data.fota_progress.percent_complete = 0;
    msg.data.fota_progress.bytes_received = 0;
    msg.data.fota_progress.total_size = 0;
    msg.data.fota_progress.error_code = 0;
    
    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send FOTA progress to Bluetooth thread");
    }
    
    return MGMT_CB_OK;
}

mgmt_cb_return fota_chunk_callback(uint32_t event, enum mgmt_cb_return prev_status,
                                  int32_t *rc, uint16_t *group,
                                  bool *abort_more, void *data, size_t data_size)
{
    ARG_UNUSED(event);
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    
    // Log every chunk for debugging
    LOG_DBG("FOTA chunk callback triggered, data_size=%zu", data_size);
    
    if (data && data_size >= sizeof(struct img_mgmt_upload_check)) {
        struct img_mgmt_upload_check *check = (struct img_mgmt_upload_check *)data;
        
        if (check->req) {
        fota_progress.chunks_received++;
        fota_progress.bytes_received = check->req->off + check->req->size;
        
        // Log first chunk
        if (fota_progress.chunks_received == 1) {
            LOG_INF("FOTA transfer started - First chunk received, size: %u bytes", check->req->size);
        }
        
        // Log progress every 5 chunks or every 50KB
        if ((fota_progress.chunks_received % 5 == 0) || 
        (fota_progress.bytes_received - fota_progress.last_reported_bytes >= 51200)) {
        
        // Calculate approximate progress based on configured or learned firmware size
        uint32_t estimated_size;
        bool is_netcore = false;
        
        // Determine which image this is
        if (fota_progress.is_first_image) {
            // First image is app core
            is_netcore = false;
            if (fota_progress.app_core_size > 0) {
                estimated_size = fota_progress.app_core_size;
            } else {
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
                estimated_size = CONFIG_FOTA_PRIMARY_SIZE_ESTIMATE;
#else
                estimated_size = CONFIG_FOTA_SECONDARY_SIZE_ESTIMATE;
#endif
            }
        } else {
            // Second image is network core
            is_netcore = true;
            if (fota_progress.net_core_size > 0) {
                estimated_size = fota_progress.net_core_size;
            } else {
                estimated_size = CONFIG_FOTA_NETCORE_SIZE_ESTIMATE;
            }
        }
        
        uint8_t estimated_percent = MIN((fota_progress.bytes_received * 100) / estimated_size, 99);
        
        LOG_INF("FOTA Progress (%s): %u bytes received (%u chunks) - Estimated %u%%",
        is_netcore ? "Network Core" : "App Core",
        fota_progress.bytes_received,
        fota_progress.chunks_received,
        estimated_percent);
        
        fota_progress.last_reported_bytes = fota_progress.bytes_received;
        
        // Send FOTA progress message to Bluetooth thread
        generic_message_t msg;
        msg.sender = SENDER_NONE;
        msg.type = MSG_TYPE_FOTA_PROGRESS;
        msg.data.fota_progress.is_active = fota_progress.is_active;
        msg.data.fota_progress.status = fota_progress.status;
        msg.data.fota_progress.percent_complete = estimated_percent;
        msg.data.fota_progress.bytes_received = fota_progress.bytes_received;
        msg.data.fota_progress.total_size = estimated_size; // Estimated
        msg.data.fota_progress.error_code = fota_progress.error_code;
        
        if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send FOTA progress to Bluetooth thread");
        }
        }
        }
    }
    
    return MGMT_CB_OK;
}

mgmt_cb_return fota_chunk_written_callback(uint32_t event, enum mgmt_cb_return prev_status,
                                          int32_t *rc, uint16_t *group,
                                          bool *abort_more, void *data, size_t data_size)
{
    ARG_UNUSED(event);
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);
    
    fota_progress.chunks_written++;
    LOG_DBG("Chunk written to flash (total: %u)", fota_progress.chunks_written);
    
    return MGMT_CB_OK;
}

mgmt_cb_return fota_pending_callback(uint32_t event, enum mgmt_cb_return prev_status,
                                    int32_t *rc, uint16_t *group,
                                    bool *abort_more, void *data, size_t data_size)
{
    ARG_UNUSED(event);
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);
    
    LOG_INF("FOTA Transfer complete, pending verification");
    LOG_INF("Total chunks: %u received, %u written", 
            fota_progress.chunks_received, fota_progress.chunks_written);
    LOG_INF("Total size: %u bytes", fota_progress.bytes_received);
    
    fota_progress.status = 2; // pending
    fota_progress.percent_complete = 100; // Transfer complete
    
    // Send FOTA progress message to Bluetooth thread
    generic_message_t msg;
    msg.sender = SENDER_NONE;
    msg.type = MSG_TYPE_FOTA_PROGRESS;
    msg.data.fota_progress.is_active = fota_progress.is_active;
    msg.data.fota_progress.status = fota_progress.status;
    msg.data.fota_progress.percent_complete = fota_progress.percent_complete;
    msg.data.fota_progress.bytes_received = fota_progress.bytes_received;
    msg.data.fota_progress.total_size = fota_progress.total_size;
    msg.data.fota_progress.error_code = fota_progress.error_code;
    
    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send FOTA progress to Bluetooth thread");
    }
    
    return MGMT_CB_OK;
}

mgmt_cb_return fota_confirmed_callback(uint32_t event, enum mgmt_cb_return prev_status,
                                      int32_t *rc, uint16_t *group,
                                      bool *abort_more, void *data, size_t data_size)
{
    ARG_UNUSED(event);
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);
    
    LOG_INF("FOTA Image confirmed! Total bytes: %u", fota_progress.bytes_received);
    
#if IS_ENABLED(CONFIG_FOTA_DYNAMIC_SIZE_DETECTION)
    // Save the size for future FOTA operations
    if (fota_progress.bytes_received > 0) {
        if (fota_progress.bytes_received < 200000) {
            // Network core update
            fota_progress.net_core_size = fota_progress.bytes_received;
            LOG_INF("Saved network core size: %u bytes", fota_progress.net_core_size);
        } else {
            // App core update
            fota_progress.app_core_size = fota_progress.bytes_received;
            LOG_INF("Saved application core size: %u bytes", fota_progress.app_core_size);
        }
    }
#endif
    
    fota_progress.is_active = false;
    fota_progress.status = 3; // confirmed
    
    // Send FOTA progress message to Bluetooth thread
    generic_message_t msg;
    msg.sender = SENDER_NONE;
    msg.type = MSG_TYPE_FOTA_PROGRESS;
    msg.data.fota_progress.is_active = fota_progress.is_active;
    msg.data.fota_progress.status = fota_progress.status;
    msg.data.fota_progress.percent_complete = fota_progress.percent_complete;
    msg.data.fota_progress.bytes_received = fota_progress.bytes_received;
    msg.data.fota_progress.total_size = fota_progress.total_size;
    msg.data.fota_progress.error_code = fota_progress.error_code;
    
    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send FOTA progress to Bluetooth thread");
    }
    
    #if IS_ENABLED(CONFIG_SECONDARY_DEVICE)
    // Secondary device needs to notify primary that FOTA is complete
    LOG_INF("Notifying primary device of FOTA completion");
    int err = ble_d2d_tx_send_fota_complete();
    if (err) {
        LOG_ERR("Failed to notify primary of FOTA completion: %d", err);
    }
    #endif
    
    // For multi-image updates, only reset after the last image
    // Network core updates are typically last and smaller
    if (fota_progress.bytes_received < 200000) {
        // This was likely the network core (final image)
        LOG_INF("Network core update confirmed - scheduling system reset in 2 seconds...");
        k_work_schedule(&fota_reset_work, K_SECONDS(2));
    } else {
        // This was the app core, network core update will follow
        LOG_INF("Application core update confirmed - waiting for network core update...");
        // Don't reset yet, wait for network core update
    }
    
    return MGMT_CB_OK;
}

mgmt_cb_return fota_stopped_callback(uint32_t event, enum mgmt_cb_return prev_status,
                                    int32_t *rc, uint16_t *group,
                                    bool *abort_more, void *data, size_t data_size)
{
    ARG_UNUSED(event);
    ARG_UNUSED(prev_status);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);
    
    LOG_WRN("FOTA Stopped/Aborted");
    fota_progress.is_active = false;
    fota_progress.status = 4; // error
    if (rc) {
        fota_progress.error_code = *rc;
    }
    
    // Send FOTA progress message to Bluetooth thread
    generic_message_t msg;
    msg.sender = SENDER_NONE;
    msg.type = MSG_TYPE_FOTA_PROGRESS;
    msg.data.fota_progress.is_active = fota_progress.is_active;
    msg.data.fota_progress.status = fota_progress.status;
    msg.data.fota_progress.percent_complete = fota_progress.percent_complete;
    msg.data.fota_progress.bytes_received = fota_progress.bytes_received;
    msg.data.fota_progress.total_size = fota_progress.total_size;
    msg.data.fota_progress.error_code = fota_progress.error_code;
    
    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send FOTA progress to Bluetooth thread");
    }
    
    return MGMT_CB_OK;
}

// Callback registration structures
static struct mgmt_callback fota_callbacks[] = {
    {
        .node = {},
        .callback = fota_started_callback,
        .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_STARTED
    },
    {
        .node = {},
        .callback = fota_chunk_callback,
        .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK
    },
    {
        .node = {},
        .callback = fota_chunk_written_callback,
        .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK_WRITE_COMPLETE
    },
    {
        .node = {},
        .callback = fota_pending_callback,
        .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_PENDING
    },
    {
        .node = {},
        .callback = fota_confirmed_callback,
        .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CONFIRMED
    },
    {
        .node = {},
        .callback = fota_stopped_callback,
        .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_STOPPED
    },
};


// Public function to get current FOTA progress
struct fota_progress_state* get_fota_progress()
{
    return &fota_progress;
}

void app_log(const char *fmt, ...)
{
    LOG_MODULE_DECLARE(MODULE, CONFIG_APP_MODULE_LOG_LEVEL);
    char buf[256];
    va_list vl;
    va_start(vl, fmt);
    vsnprintk(buf, sizeof(buf), fmt, vl);
    va_end(vl);
    LOG_INF("%s", buf);
}

void initializing_entry()
{
    LOG_INF("Sensing FW version: %s, Sensing HW version: %s", APP_VERSION_STRING,
            CONFIG_HARDWARE_STRING);

    // To initialise message que here

    LOG_DBG("App initializing_entry done ");
}

static void app_init()
{
    initializing_entry();

    // Initialize FOTA reset work
    k_work_init_delayable(&fota_reset_work, fota_reset_handler);

    // Register all FOTA callbacks
    for (size_t i = 0; i < ARRAY_SIZE(fota_callbacks); i++) {
        mgmt_callback_register(&fota_callbacks[i]);
    }
    LOG_INF("FOTA progress tracking initialized");

    app_tid = k_thread_create(&app_thread_data, app_stack_area, K_THREAD_STACK_SIZEOF(app_stack_area), app_entry,
                              nullptr, nullptr, nullptr, app_priority, 0, K_NO_WAIT);

    LOG_INF("APP Module Initialised");
}

void app_entry(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{
    const int app_wait_timer = 2000;

    k_thread_name_set(app_tid, "app"); // sets the name of the thread

    module_set_state(MODULE_STATE_READY);

    while (true)
    {
        // TODO develop thread
        k_msleep(app_wait_timer);
    }
}

// Helper function to get sender name for logging (re-defined here for clarity, or put in common utility)
const char *get_sender_name(sender_type_t sender)
{
    switch (sender)
    {
        case SENDER_FOOT_SENSOR_THREAD:
            return "Foot Sensor Thread";
        case SENDER_BHI360_THREAD:
            return "BHI360 Thread";
        case SENDER_BTH:
            return "UI Thread";
        case SENDER_DATA:
            return "Data Thread";
        case SENDER_D2D_SECONDARY:
            return "D2D Secondary";
        case SENDER_MOTION_SENSOR:
            return "Motion Sensor";
        case SENDER_WIFI:
            return "WiFi";
        case SENDER_NONE:
            return "None/Unknown";
        default:
            return "Invalid Sender";
    }
}

// Return type dictates if event is consumed. False = Not Consumed, True =
// Consumed.
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(main), MODULE_STATE_READY))
        {
            app_init();
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, battery_state_event);
APP_EVENT_SUBSCRIBE(MODULE, data_event);

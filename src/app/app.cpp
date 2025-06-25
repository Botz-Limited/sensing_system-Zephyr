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
#include <errors.hpp>

#if IS_ENABLED(CONFIG_SECONDARY_DEVICE)
#include "../bluetooth/ble_d2d_tx.hpp"
#endif

LOG_MODULE_REGISTER(MODULE, CONFIG_APP_MODULE_LOG_LEVEL); // NOLINT

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
} fota_progress = {false, 0, 0, 0, 0, 0, 0, 0};

void initializing_entry();

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

// Forward declaration for BLE notification
void notify_fota_progress_to_ble();

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
    
    // Reset progress tracking
    memset(&fota_progress, 0, sizeof(fota_progress));
    fota_progress.is_active = true;
    fota_progress.status = 1; // in_progress
    
    // Notify BLE
    notify_fota_progress_to_ble();
    
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
    
    if (data && data_size >= sizeof(struct img_mgmt_upload_check)) {
        struct img_mgmt_upload_check *check = (struct img_mgmt_upload_check *)data;
        
        if (check->req) {
            fota_progress.chunks_received++;
            fota_progress.bytes_received = check->req->off + check->req->size;
            
            // First chunk contains total size
            if (check->req->off == 0) {
                fota_progress.total_size = check->req->size;
                LOG_INF("FOTA Total size: %u bytes", fota_progress.total_size);
            }
        }
        
        // Calculate progress
        if (fota_progress.total_size > 0) {
            uint8_t new_percent = (fota_progress.bytes_received * 100) / fota_progress.total_size;
            
            // Only notify when percentage changes
            if (new_percent != fota_progress.percent_complete) {
                fota_progress.percent_complete = new_percent;
                LOG_INF("FOTA Progress: %u%% (%u/%u bytes)", 
                        fota_progress.percent_complete,
                        fota_progress.bytes_received,
                        fota_progress.total_size);
                
                // Notify BLE
                notify_fota_progress_to_ble();
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
    
    fota_progress.status = 2; // pending
    notify_fota_progress_to_ble();
    
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
    
    LOG_INF("FOTA Image confirmed!");
    fota_progress.is_active = false;
    fota_progress.status = 3; // confirmed
    notify_fota_progress_to_ble();
    
    #if IS_ENABLED(CONFIG_SECONDARY_DEVICE)
    // Secondary device needs to notify primary that FOTA is complete
    LOG_INF("Notifying primary device of FOTA completion");
    int err = ble_d2d_tx_send_fota_complete();
    if (err) {
        LOG_ERR("Failed to notify primary of FOTA completion: %d", err);
    }
    #endif
    
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
    notify_fota_progress_to_ble();
    
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

// Function to send FOTA progress via message queue to Bluetooth thread
void notify_fota_progress_to_ble()
{
    generic_message_t msg;
    msg.sender = SENDER_NONE;
    msg.type = MSG_TYPE_FOTA_PROGRESS;
    
    // Pack progress data
    msg.data.fota_progress.is_active = fota_progress.is_active;
    msg.data.fota_progress.status = fota_progress.status;
    msg.data.fota_progress.percent_complete = fota_progress.percent_complete;
    msg.data.fota_progress.bytes_received = fota_progress.bytes_received;
    msg.data.fota_progress.total_size = fota_progress.total_size;
    msg.data.fota_progress.error_code = fota_progress.error_code;
    
    // Send to Bluetooth thread
    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send FOTA progress to Bluetooth thread");
    }
    
    #if IS_ENABLED(CONFIG_SECONDARY_DEVICE)
    // For secondary device, also log that progress would be sent via D2D
    // Note: Actual D2D sending happens in bluetooth thread
    LOG_DBG("FOTA progress ready for D2D transmission to primary");
    #endif
}

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
    LOG_INF("Sensing FW version: %s, Sensing HW version: %s", CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION,
            CONFIG_HARDWARE_STRING);

    // To initialise message que here

    LOG_DBG("App initializing_entry done ");
}

static void app_init()
{
    initializing_entry();

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

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
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/sys/timeutil.h>

#include <zephyr/devicetree.h>

#include <app.hpp>
#include <ble_services.hpp>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_APP_MODULE_LOG_LEVEL); // NOLINT

/********************************** APP THREAD ********************************/
static constexpr int app_stack_size = CONFIG_APP_MODULE_STACK_SIZE;
static constexpr int app_priority = CONFIG_APP_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(app_stack_area, app_stack_size);
static struct k_thread app_thread_data;
static k_tid_t app_tid;
void app_entry(void * /*unused*/, void * /*unused*/, void * /*unused*/);

mgmt_callback dfu_callback{};

mgmt_cb_return mcumgr_dfu_callback(uint32_t event, enum mgmt_cb_return prev_status, int32_t *rc, uint16_t *group,
                                    bool *abort_more, void *data, size_t data_size)
{
    ARG_UNUSED(prev_status);
    ARG_UNUSED(rc);
    ARG_UNUSED(group);
    ARG_UNUSED(abort_more);
    ARG_UNUSED(data);
    ARG_UNUSED(data_size);

    /* To develop */
    /* Return OK status code to continue with acceptance to underlying handler */
    return MGMT_CB_OK;
}



int app_init()
{

    dfu_callback.callback = mcumgr_dfu_callback;
    dfu_callback.event_id = (MGMT_EVT_OP_IMG_MGMT_DFU_STARTED);
    mgmt_callback_register(&dfu_callback);


    app_tid = k_thread_create(&app_thread_data, app_stack_area, K_THREAD_STACK_SIZEOF(app_stack_area), app_entry,
                              nullptr, nullptr, nullptr, app_priority, 0, K_NO_WAIT);

    k_thread_name_set(app_tid, "app"); // sets the name of the thread

    bt_module_init(); //To define errors

    LOG_INF("APP Module Initialised");
    module_set_state(MODULE_STATE_READY);

    return 0; //To define errors
}

void app_entry(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{
    while (true)
    {
        LOG_DBG("I'm the APP task");
        k_msleep(app_wait_timer);
    }
    
}

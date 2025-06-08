/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define MODULE main

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/timeutil.h>

#include <zephyr/logging/log.h>

#define LOG_LEVEL LOG_LEVEL_DBG

LOG_MODULE_REGISTER(MODULE);

int main(void)
{
    if (app_event_manager_init() != 0)
    {
        LOG_ERR("Application Event Manager not initialized");
    }
    else
    {
        // Confirm the new uploaded image started successfully, or the bootloader will revert
        // to the old version.
     /*   int err = boot_write_img_confirmed();
        if (err)
        {
            LOG_ERR("Error in uploading image: err: %d", err);
        }
        else
        {
            LOG_INF("Successfully, uploaded image ");
        } */

        module_set_state(MODULE_STATE_READY);
    }
     while (1)
    {
        k_sleep(K_FOREVER);
    }
}



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
#include "fota_fix.hpp"

#define LOG_LEVEL LOG_LEVEL_DBG

LOG_MODULE_REGISTER(MODULE);

int main(void)
{
    // Initialize FOTA fixes and check image status
    int err = fota_fix_init();
    if (err) {
        LOG_WRN("FOTA fix initialization failed: %d", err);
        // Continue anyway, this is not critical for normal operation
    }
    if (app_event_manager_init() != 0)
    {
        LOG_ERR("Application Event Manager not initialized");
    }
    else
    {

        module_set_state(MODULE_STATE_READY);
    }
    while (1)
    {
        k_sleep(K_FOREVER);
    }
}

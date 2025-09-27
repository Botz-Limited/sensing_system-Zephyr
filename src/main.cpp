/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define MODULE main

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/timeutil.h>

#include <zephyr/logging/log.h>

#define LOG_LEVEL LOG_LEVEL_WRN

LOG_MODULE_REGISTER(MODULE);

int main(void)
{
    uint32_t reset_cause;
    char reset_cause_str[128];

    int ret = hwinfo_get_reset_cause(&reset_cause);

    if (ret != 0)
    {
        LOG_WRN("Failed to get reset cause (err %d)\n", ret);
    }
    else
    {
        // Print the raw value first - it's always available
        LOG_WRN("Reset cause register: 0x%08X\n", reset_cause);

        // Now, manually check the most important bits and print a message.
        // This is the reliable way to do it.
        LOG_WRN("Reset reasons: ");

        if (reset_cause & RESET_POR)
        {
            LOG_WRN("Power-On-Reset ");
        }
        if (reset_cause & RESET_PIN)
        {
            LOG_WRN("Reset-Pin ");
        }
        if (reset_cause & RESET_BROWNOUT)
        {
            LOG_WRN("Brownout ");
        }
        if (reset_cause & RESET_SOFTWARE)
        {
            LOG_WRN("Software ");
        }
        if (reset_cause & RESET_WATCHDOG)
        {
            LOG_WRN("Watchdog ");
        }
        if (reset_cause & RESET_DEBUG)
        {
            LOG_WRN("Debugger ");
        }
        if (reset_cause & RESET_CPU_LOCKUP)
        {
            LOG_WRN("CPU-Lockup ");
        }
        if (reset_cause & RESET_LOW_POWER_WAKE)
        {
            LOG_WRN("Low-Power ");
        }
        if (reset_cause & RESET_USER)
        {
            LOG_WRN("Low-Power ");
        }
        if (reset_cause & RESET_HARDWARE)
        {
            LOG_WRN("Low-Power ");
        }
        if (reset_cause & RESET_SECURITY)
        {
            LOG_WRN("Low-Power ");
        }
        if (reset_cause & RESET_PLL)
        {
            LOG_WRN("Low-Power ");
        }
        if (reset_cause & RESET_CLOCK)
        {
            LOG_WRN("Low-Power ");
        }
        if (reset_cause & RESET_TEMPERATURE)
        {
            LOG_WRN("Low-Power ");
        }
        // Check if no flags were set (e.g., a previous reset cleared the register)
        if (reset_cause == 0)
        {
            LOG_WRN("Unknown (register cleared)");
        }

        LOG_WRN("\n");
    }

    if (app_event_manager_init() != 0)
    {
        LOG_ERR("Application Event Manager not initialized");
    }
    else
    {
        // Confirm the new uploaded image started successfully, or the bootloader will revert
        // to the old version.
        int err = boot_write_img_confirmed();
        if (err)
        {
            LOG_ERR("Error in uploading image: err: %d", err);
        }
        else
        {
            LOG_INF("Successfully, uploaded image ");
            LOG_INF("Starting up");
        }
        module_set_state(MODULE_STATE_READY);
    }
    while (1)
    {
        k_sleep(K_FOREVER);
    }
}

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <caf/events/module_state_event.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/timeutil.h>

#include "app.hpp"

#include <zephyr/logging/log.h>

#define LOG_LEVEL LOG_LEVEL_DBG

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
    sensing_app_init(); // to define errors

    return 0;
}

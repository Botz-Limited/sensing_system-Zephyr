#define MODULE idle
/**
 * @file idle.cpp
 * @brief
 * @version 1.0
 * @date August 2025
 *
 * @copyright Botz Innovation 2025
 *
 */


#include <cstdint>
#include <cstring>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <app.hpp>
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <errors.hpp>
#include <events/app_state_event.h>
#include <events/foot_sensor_event.h>
#include <events/streaming_control_event.h>
#include <status_codes.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_IDLE_MODULE_LOG_LEVEL); // NOLINT

// Thread stack and priority
K_THREAD_STACK_DEFINE(idle_thread_stack, CONFIG_IDLE_MODULE_STACK_SIZE);
static struct k_thread idle_thread_data;
static k_tid_t idle_tid = NULL;

// Forward declarations
static void idle_thread(void *p1, void *p2, void *p3);
static err_t init_idle(void);

static err_t init_idle(void)
{
    LOG_INF("Initializing idle module");

    // Create and start the idle thread
    idle_tid = k_thread_create(&idle_thread_data, idle_thread_stack,
                               K_THREAD_STACK_SIZEOF(idle_thread_stack), idle_thread, NULL, NULL, NULL,
                               K_PRIO_PREEMPT(CONFIG_IDLE_MODULE_PRIORITY), 0, K_NO_WAIT);

    k_thread_name_set(idle_tid, "idle");

    LOG_INF("Idle module initialized successfully");
    module_set_state(MODULE_STATE_READY);
    return err_t::NO_ERROR;
}

static void idle_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Idle thread started");

    while (true)
    {
        k_sleep(K_MSEC(1000));
    }
}

/**
 * @brief Event handler for foot sensor module
 */
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        // Initialize after bluetooth is ready
        if (check_state(event, MODULE_ID(bluetooth), MODULE_STATE_READY))
        {
            init_idle();
        }
        return false;
    }

    if (is_foot_sensor_start_activity_event(aeh))
    {
        LOG_INF("Received start activity event - enabling foot sensor sampling");
    //    atomic_set(&logging_active, 1);
        return false;
    }

    if (is_foot_sensor_stop_activity_event(aeh))
    {
        LOG_INF("Received stop activity event - disabling foot sensor sampling");
       // atomic_set(&logging_active, 0);
        return false;
    }


    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_stop_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, streaming_control_event);


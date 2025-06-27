/**
 * @file wifi.cpp
 * @brief
 * @version 1.0
 * @date June 2025
 *
 * @copyright Botz Innovation 2025
 *
 */

/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */

#define MODULE wifi

#include <chp_lib.hpp>
#include <crc32.hpp>
#include <errors.hpp>
#include <wifi.hpp>
#include <safe_buffer.hpp>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <events/app_state_event.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_DATA_MODULE_LOG_LEVEL); // NOLINT

/**
 *****************************************************************************************************************************************************
 *  \section LOCAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */

/**
 *****************************************************************************************************************************************************
 *  \section STATIC FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */

static void wifi_init();
static void wifi_initializing_entry();

/**
 *****************************************************************************************************************************************************
 *  \section STATIC VARIABLES
 *****************************************************************************************************************************************************
 */                                    // Length of the input buffer

/**
 *****************************************************************************************************************************************************
 *  \section GLOBAL VARIABLES
 *****************************************************************************************************************************************************
 */

/**
 *****************************************************************************************************************************************************
 *  \section SOURCE CODE
 *****************************************************************************************************************************************************
 */

static void wifi_initializing_entry()
{
    // To initialise message que here
}

static void wifi_init()
{
    wifi_initializing_entry();
    // To develop init

    LOG_INF("wifi init done ");
    module_set_state(MODULE_STATE_READY);
}


// Return type dictates if event is consumed. False = Not Consumed, True =
// Consumed.
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(motion_sensor), MODULE_STATE_READY))
        {
            wifi_init();
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);
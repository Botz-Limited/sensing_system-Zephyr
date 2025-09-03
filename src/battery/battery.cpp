/**
 * @file battery.cpp
 * @brief Battery monitoring
 * @version 1.0.0
 * @date June 2025
 * @copyright Botz Innovation 2025
 */

#define MODULE battery

#include <cstring>

#include <battery.hpp>
#include <errors.hpp>

#include <app.hpp>
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <events/app_state_event.h>
#include <events/motion_sensor_event.h>
#include <util.hpp>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>


LOG_MODULE_REGISTER(MODULE, CONFIG_BATTERY_MODULE_LOG_LEVEL); // NOLINT

#define BATTERY_UPDATE_INTERVAL_MS 60000 // 60 seconds

// --- PERIODIC BATTERY UPDATE ---
static struct k_work_delayable battery_update_work;

// Currently unused - kept for future battery monitoring implementation
static void battery_update_work_handler(struct k_work *work)
{
    (void)work;         // Silence unused parameter warning
    uint8_t level = 80; // This module is in development, so at the moment just hardcode a value
    bt_bas_set_battery_level(level); // Update standard BLE Battery Service
    #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    generic_message_t msg;
    msg.sender = SENDER_BATTTERY;
    msg.type = MSG_TYPE_BATTERY_LEVEL_PRIMARY;
    msg.data.battery_level = (battery_level_msg_t){ .level = level }; // Use the new battery_level_msg_t structure
    k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT); // Send to Bluetooth module via message queue on secondary
    #else
    generic_message_t msg;
    msg.sender = SENDER_BATTTERY;
    msg.type = MSG_TYPE_BATTERY_LEVEL_SECONDARY;
    msg.data.battery_level = (battery_level_msg_t){ .level = level }; // Use the new battery_level_msg_t structure
    k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT); // Send to Bluetooth module via message queue on secondary
    #endif
    LOG_DBG("Battery level updated: %d%%", level);
    k_work_reschedule(&battery_update_work, K_MSEC(BATTERY_UPDATE_INTERVAL_MS));
}

void battery_monitor_init(void)
{
    LOG_INF("Battery level Init");
    k_work_init_delayable(&battery_update_work, battery_update_work_handler);
    k_work_schedule(&battery_update_work, K_NO_WAIT);
    module_set_state(MODULE_STATE_READY);
}

static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(bluetooth), MODULE_STATE_READY))
        {
            battery_monitor_init();
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, _state_event);

/**
 * @file battery.cpp
 * @brief Battery monitoring using BQ27441
 * @version 1.0.0
 * @date September 2025
 * @copyright Botz Innovation 2025
 */

#include <battery/battery.hpp>
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
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_BATTERY_MODULE_LOG_LEVEL);

#define BATTERY_UPDATE_INTERVAL_MS 60000 // 60 seconds

// Battery devices
static const struct device *bq27441_dev = DEVICE_DT_GET(DT_NODELABEL(bq27441));
static const struct gpio_dt_spec stat1 = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25185), stat1_gpios);
static const struct gpio_dt_spec stat2 = GPIO_DT_SPEC_GET(DT_NODELABEL(bq25185), stat2_gpios);
static struct k_work_delayable battery_update_work;
static struct battery_data current_battery_data;

// Charging states based on STAT1 and STAT2 pins
enum bq25185_state
{
    BQ25185_NO_INPUT = 0, // STAT1=0, STAT2=0
    BQ25185_CHARGING,     // STAT1=0, STAT2=1
    BQ25185_CHARGE_DONE,  // STAT1=1, STAT2=0
    BQ25185_FAULT         // STAT1=1, STAT2=1
};

// Function to read BQ25185 charger status
static enum bq25185_state read_charger_status(void)
{
    int stat1_val = gpio_pin_get_dt(&stat1);
    int stat2_val = gpio_pin_get_dt(&stat2);

    if (stat1_val < 0 || stat2_val < 0)
    {
        LOG_ERR("Failed to read charger status pins");
        return BQ25185_NO_INPUT;
    }

    return (enum bq25185_state)((stat1_val << 1) | stat2_val);
}

// Function to read battery data from BQ27441 and charger status
static int read_battery_sensor(void)
{
    struct sensor_value temp, voltage, current, charge_state;

    // Read charger status first
    enum bq25185_state charger_state = read_charger_status();
    current_battery_data.charging = (charger_state == BQ25185_CHARGING);

    // Log charger status
    switch (charger_state)
    {
        case BQ25185_NO_INPUT:
            LOG_DBG("Charger: No input or sleep mode");
            break;
        case BQ25185_CHARGING:
            LOG_DBG("Charger: Charging");
            break;
        case BQ25185_CHARGE_DONE:
            LOG_DBG("Charger: Charge complete");
            break;
        case BQ25185_FAULT:
            LOG_WRN("Charger: Fault detected");
            break;
    }

    // Read temperature
    /*  if (sensor_sample_fetch_chan(bq27441_dev, SENSOR_CHAN_AMBIENT_TEMP) == 0)
      {
          sensor_channel_get(bq27441_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
          current_battery_data.temperature = temp.val1 * 10 + temp.val2 / 100000;
      }

      // Read voltage
      if (sensor_sample_fetch_chan(bq27441_dev, SENSOR_CHAN_GAUGE_VOLTAGE) == 0)
      {
          sensor_channel_get(bq27441_dev, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage);
          current_battery_data.voltage_mv = voltage.val1;
      }

      // Read current
      if (sensor_sample_fetch_chan(bq27441_dev, SENSOR_CHAN_GAUGE_AVG_CURRENT) == 0)
      {
          sensor_channel_get(bq27441_dev, SENSOR_CHAN_GAUGE_AVG_CURRENT, &current);
          current_battery_data.current_ma = current.val1;
      } */

    // Read state of charge
    if (sensor_sample_fetch_chan(bq27441_dev, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE) == 0)
    {
        sensor_channel_get(bq27441_dev, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &charge_state);
        current_battery_data.state_of_charge = charge_state.val1;
    }

    // Update charging status based on current
    current_battery_data.charging = (current_battery_data.current_ma > 0);

    // Update BAS service
    bt_bas_set_battery_level(current_battery_data.state_of_charge);

    return current_battery_data.state_of_charge;
}

// Battery update work handler
static void battery_update_work_handler(struct k_work *work)
{
    (void)work;

    uint8_t level = read_battery_sensor();

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    generic_message_t msg;
    msg.sender = SENDER_BATTTERY;
    msg.type = MSG_TYPE_BATTERY_LEVEL_PRIMARY;
    msg.data.battery_level = (battery_level_msg_t){.level = level}; // Use the new battery_level_msg_t structure
    k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT); // Send to Bluetooth module via message queue on secondary
    // Also send to the data module
    k_msgq_put(&data_msgq, &msg, K_NO_WAIT); // Send to Data module via message queue
#else
    generic_message_t msg;
    msg.sender = SENDER_BATTTERY;
    msg.type = MSG_TYPE_BATTERY_LEVEL_SECONDARY;
    msg.data.battery_level = (battery_level_msg_t){.level = level}; // Use the new battery_level_msg_t structure
    k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT); // Send to Bluetooth module via message queue on secondary
#endif
    LOG_DBG("Battery level updated: %d%%", level);
    k_work_schedule(&battery_update_work, K_MSEC(BATTERY_UPDATE_INTERVAL_MS));
}

void get_battery_data(struct battery_data *data)
{
    if (data != NULL)
    {
        *data = current_battery_data;
    }
}

void battery_monitor_init(void)
{
    LOG_INF("Battery level Init");
    bool init_ok = true;

    k_msleep(1000);

    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c3));
    if (!device_is_ready(i2c_dev))
    {
        LOG_ERR("I2C3 device not ready");
        init_ok = false;
    }
    else
    {
        LOG_INF("I2C3 device is ready");
    }
    k_msleep(1000);
    // Initialize BQ27441 fuel gauge
    if (!device_is_ready(bq27441_dev))
    {
        LOG_ERR("BQ27441 device not ready");
        init_ok = false;
        // Continue initialization - we can still monitor charger status
    }

    k_msleep(1000);
    // Initialize charger status pins
    if (!device_is_ready(stat1.port))
    {
        LOG_ERR("STAT1 GPIO device not ready");
        init_ok = false;
    }

    if (!device_is_ready(stat2.port))
    {
        LOG_ERR("STAT2 GPIO device not ready");
        init_ok = false;
    }

    if (init_ok)
    {
        // Configure GPIO pins
        int ret = gpio_pin_configure_dt(&stat1, GPIO_INPUT);
        if (ret < 0)
        {
            LOG_ERR("Failed to configure STAT1 pin: %d", ret);
            init_ok = false;
        }

        ret = gpio_pin_configure_dt(&stat2, GPIO_INPUT);
        if (ret < 0)
        {
            LOG_ERR("Failed to configure STAT2 pin: %d", ret);
            init_ok = false;
        }
    }

    // Initialize work queue for periodic updates
    k_work_init_delayable(&battery_update_work, battery_update_work_handler);
    k_work_schedule(&battery_update_work, K_NO_WAIT);

    if (init_ok)
    {
        LOG_INF("Battery monitoring initialized successfully");
    }
    else
    {
        LOG_WRN("Battery monitoring initialized with errors");
    }

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

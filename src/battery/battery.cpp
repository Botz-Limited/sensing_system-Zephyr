/**
 * @file battery.cpp
 * @brief Battery monitoring
 * @version 1.0.0
 * @date 2025-05-12
 * @copyright Botz Innovation 2025
 */

#define MODULE battery

#include <cstring>

#include <battery.hpp>
#include <errors.hpp>

#include <util.hpp>
#include <caf/events/module_state_event.h>
#include <app_event_manager.h>
#include <events/app_state_event.h>
#include <events/motion_sensor_event.h>

#include <zephyr/pm/device.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/kernel.h>

#include <hal/nrf_saadc.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_BATTERY_MODULE_LOG_LEVEL); // NOLINT

#define BATTERY_UPDATE_INTERVAL_MS 10000 // 10 seconds
#define SAADC_VDD_CHANNEL 7 // Virtual channel for VDD measurement

// --- BATTERY LEVEL READING (nRF HAL) ---
/**
 * @brief Read battery level as a percentage (0-100) using nRF SAADC HAL VDD channel.
 *
 * This does not consume a physical ADC pin. For future hardware, update this function as needed.
 */
#include <nrfx_saadc.h>

// Forward declaration for the restore function (implemented below)
extern "C" void foot_sensor_restore_channel0(void);

uint8_t read_battery_level(void) {
    int16_t buffer = 0;

    // --- BATTERY MEASUREMENT USING SAADC ---
   

    // Configure channel 0 for internal VDD measurement
    nrf_saadc_channel_config_t vdd_config = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain = NRF_SAADC_GAIN1_6,
        .reference = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time = NRF_SAADC_ACQTIME_10US,
        .mode = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst = NRF_SAADC_BURST_DISABLED
    };
    // Set channel 0 to measure VDD (internal supply voltage)
    nrf_saadc_channel_init(NRF_SAADC, 0, &vdd_config);
    nrf_saadc_channel_input_set(NRF_SAADC, 0, NRF_SAADC_INPUT_VDD, NRF_SAADC_INPUT_DISABLED);

    // Set up result buffer for one sample
    nrf_saadc_buffer_init(NRF_SAADC, &buffer, 1);

    // Start conversion: trigger START and wait for STARTED event (with timeout)
    #define SAADC_EVENT_TIMEOUT 10000 // Max iterations to wait for event
    nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_START);
    int timeout = SAADC_EVENT_TIMEOUT;
    while (!nrf_saadc_event_check(NRF_SAADC, NRF_SAADC_EVENT_STARTED) && --timeout);
    if (timeout == 0) {
        LOG_ERR("SAADC STARTED event timeout");
        nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_STARTED);
        return 100;
    }
    nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_STARTED);

    // Trigger SAMPLE and wait for END event (with timeout)
    timeout = SAADC_EVENT_TIMEOUT;
    nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_SAMPLE);
    while (!nrf_saadc_event_check(NRF_SAADC, NRF_SAADC_EVENT_END) && --timeout);
    if (timeout == 0) {
        LOG_ERR("SAADC END event timeout");
        nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_END);
        return 100;
    }
    nrf_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_END);

    // Disable channel 0 and SAADC to release for other use
    nrf_saadc_channel_input_set(NRF_SAADC, 0, NRF_SAADC_INPUT_DISABLED, NRF_SAADC_INPUT_DISABLED);
    nrfy_saadc_disable(NRF_SAADC);

    // --- RESTORE FOOT SENSOR CHANNEL 0 CONFIGURATION ---
    // This ensures the foot sensor can continue using channel 0 for its analog input
    foot_sensor_restore_channel0();

    // --- CONVERT RAW ADC VALUE TO BATTERY PERCENTAGE ---
    // Convert to voltage (12-bit result, VDD ref 3.6V)
    float vdd = ((float)buffer / 4095.0f) * 3.6f;
    int vdd_mv = (int)(vdd * 1000.0f);

    // Map voltage to percentage (simple linear mapping, adjust as needed)
    int percent = (vdd_mv - 1800) * 100 / (3600 - 1800);
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    return (uint8_t)percent;
}

// --- RESTORE FUNCTION FOR FOOT SENSOR CHANNEL 0 ---
// This function should be called after any battery measurement that reconfigures channel 0.
// It restores channel 0 to its original analog input for the foot sensor.
extern "C" void foot_sensor_restore_channel0(void) {
    // Configure channel 0 for the foot sensor's analog input (AIN0)
    nrf_saadc_channel_config_t config = {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain = NRF_SAADC_GAIN1_6,
        .reference = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time = NRF_SAADC_ACQTIME_10US,
        .mode = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst = NRF_SAADC_BURST_DISABLED
    };
    // Set channel 0 back to AIN0 (replace with your actual analog input if different)
    nrf_saadc_channel_init(NRF_SAADC, 0, &config);
    nrf_saadc_channel_input_set(NRF_SAADC, 0, NRF_SAADC_INPUT_AIN0, NRF_SAADC_INPUT_DISABLED);
}

// --- PERIODIC BATTERY UPDATE ---
static struct k_work_delayable battery_update_work;

static void battery_update_work_handler(struct k_work *work) {
    (void)work; // Silence unused parameter warning
    uint8_t level = read_battery_level();
 //   bt_bas_set_battery_level(level);
    LOG_INF("Battery level updated: %u%%", level);
    k_work_reschedule(&battery_update_work, K_MSEC(1000));

}

void battery_monitor_init(void) {
    LOG_INF("Battery level Init");
     // Enable SAADC peripheral
    nrfy_saadc_enable(NRF_SAADC);
    
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

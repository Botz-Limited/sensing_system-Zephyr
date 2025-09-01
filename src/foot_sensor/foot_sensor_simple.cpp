/**
 * @file foot_sensor_simple.cpp
 * @brief Simplified foot sensor using Zephyr ADC driver
 * @version 2.0.0
 * @date 2025-08-28
 *
 * @copyright Botz Innovation 2025
 */

#define MODULE foot_sensor

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

LOG_MODULE_REGISTER(MODULE, CONFIG_FOOT_SENSOR_MODULE_LOG_LEVEL);

// External message queues
extern struct k_msgq sensor_data_msgq;
extern struct k_msgq bluetooth_msgq;

// Constants
static constexpr uint8_t SAADC_CHANNEL_COUNT = 8;
static constexpr uint8_t CALIBRATION_SAMPLES = 100;
static constexpr uint16_t SAMPLE_RATE_HZ = 100;
static constexpr uint32_t SAMPLE_PERIOD_MS = 1000 / SAMPLE_RATE_HZ; // 10ms
static constexpr uint8_t BLUETOOTH_RATE_DIVIDER = 20;               // Send to BLE at 5Hz (100Hz / 20)

// Thread stack and priority
K_THREAD_STACK_DEFINE(foot_sensor_stack, 2048);
static struct k_thread foot_sensor_thread_data;
static k_tid_t foot_sensor_tid = NULL;

// Use atomic for thread-safe access to logging state
static atomic_t logging_active = ATOMIC_INIT(0);

// Local streaming state
static bool foot_sensor_streaming_enabled = false;

// ADC device and configuration
static const struct device *adc_dev;
static struct adc_channel_cfg channel_configs[SAADC_CHANNEL_COUNT];
static int16_t adc_buffer[SAADC_CHANNEL_COUNT];
static struct adc_sequence sequence = {
    .buffer = adc_buffer,
    .buffer_size = sizeof(adc_buffer),
    .resolution = 14, // 14-bit resolution like original
};

// Calibration data
static int32_t calibration_sum[SAADC_CHANNEL_COUNT] = {0};
static int16_t saadc_offset[SAADC_CHANNEL_COUNT] = {0};
static bool calibration_done = false;
static uint16_t calibration_counter = 0;

// Forward declarations
static void foot_sensor_thread(void *p1, void *p2, void *p3);
static err_t init_adc_channels(void);
static void calibrate_channels(void);
static void process_adc_samples(int16_t *raw_data);

/**
 * @brief Initialize ADC channels
 */
static err_t init_adc_channels(void)
{
    // Get ADC device
    adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    if (!device_is_ready(adc_dev))
    {
        LOG_ERR("ADC device not ready");
        return err_t::ADC_ERROR;
    }

    // Configure all 8 channels
    for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
    {
        channel_configs[i].gain = ADC_GAIN_1;
        channel_configs[i].reference = ADC_REF_INTERNAL;
        channel_configs[i].acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10);
        channel_configs[i].channel_id = i;                                         // Channel 0-7 maps to AIN0-AIN7
        channel_configs[i].input_positive = i + SAADC_CH_PSELP_PSELP_AnalogInput0; // AIN0 + i

        int err = adc_channel_setup(adc_dev, &channel_configs[i]);
        if (err < 0)
        {
            LOG_ERR("Failed to setup ADC channel %d: %d", i, err);
            return err_t::ADC_ERROR;
        }
        LOG_DBG("ADC channel %d configured (AIN%d)", i, i);
    }

    // Setup sequence for all channels
    sequence.channels = 0xFF;  // All 8 channels (bits 0-7)
    sequence.resolution = 14;  // 14-bit resolution
    sequence.oversampling = 0; // No oversampling

    LOG_INF("ADC initialized with %d channels at 14-bit resolution", SAADC_CHANNEL_COUNT);
    return err_t::NO_ERROR;
}

/**
 * @brief Calibrate channels by averaging first N samples
 */
static void calibrate_channels(void)
{
    LOG_INF("Starting calibration - collecting %d samples...", CALIBRATION_SAMPLES);

    // Reset calibration state
    calibration_done = false;
    calibration_counter = 0;
    memset(calibration_sum, 0, sizeof(calibration_sum));
    memset(saadc_offset, 0, sizeof(saadc_offset));
}

/**
 * @brief Process ADC samples and send to message queues
 */
static void process_adc_samples(int16_t *raw_data)
{
    static uint8_t ble_sample_counter = 0;

    if (!calibration_done)
    {
        // Calibration phase - accumulate samples
        if (calibration_counter < CALIBRATION_SAMPLES)
        {
            for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
            {
                calibration_sum[i] += raw_data[i];
            }
            calibration_counter++;

            if (calibration_counter == CALIBRATION_SAMPLES)
            {
                // Calculate offsets
                for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
                {
                    saadc_offset[i] = (int16_t)(calibration_sum[i] / CALIBRATION_SAMPLES);
                    LOG_DBG("Channel %d calibration offset: %d", i, saadc_offset[i]);
                }
                calibration_done = true;
                LOG_INF("Calibration complete - starting normal operation");
            }
        }
    }
    else
    {
        // Normal operation - send calibrated data
        if ((atomic_get(&logging_active) == 1) || (foot_sensor_streaming_enabled == true))
        {
            generic_message_t msg;
            msg.sender = SENDER_FOOT_SENSOR_THREAD;
            msg.type = MSG_TYPE_FOOT_SAMPLES;

            // Apply calibration offset and populate message
            for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
            {
                int16_t calibrated_value = raw_data[i] - saadc_offset[i];
                msg.data.foot_samples.values[i] = (calibrated_value < 0) ? 0 : calibrated_value;
            }

#if IS_ENABLED(CONFIG_FOOT_SIMULATION)
            // Get current uptime and calculate toggle state based on 800ms cycle (400ms high, 400ms low)
            int64_t current_time = k_uptime_get();
            bool is_high = (current_time / 400) % 2 == 0;

            for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
            {
                int16_t calibrated_value = is_high ? 11000 : 0;
                msg.data.foot_samples.values[i] = (calibrated_value < 0) ? 0 : calibrated_value;
            }
#endif

            if (atomic_get(&logging_active) == 1)
                // Always send to sensor_data module at 100Hz only if activity started
                if (k_msgq_put(&sensor_data_msgq, &msg, K_NO_WAIT) != 0)
                {
                    LOG_WRN("Failed to send foot sensor data to sensor_data module");
                }

            // Send to Bluetooth at 5Hz (every 20th sample)
            if (++ble_sample_counter >= BLUETOOTH_RATE_DIVIDER)
            {
                ble_sample_counter = 0;
                // Check streaming flag before sending to bluetooth
                if (foot_sensor_streaming_enabled == true)
                {
                    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0)
                    {
                        LOG_WRN("Failed to send foot sensor data to bluetooth module");
                    }
                }
                else
                {
                    LOG_DBG("Foot sensor BLE streaming disabled, skipping bluetooth_msgq");
                }
            }
        }
    }
}

/**
 * @brief Main foot sensor thread
 */
static void foot_sensor_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Foot sensor thread started");

    // Initialize ADC channels
    err_t err = init_adc_channels();
    if (err != err_t::NO_ERROR)
    {
        LOG_ERR("Failed to initialize ADC");
        module_set_state(MODULE_STATE_ERROR);
        return;
    }

    // Start calibration
    calibrate_channels();

    // Main sampling loop
    while (true)
    {
        // Read all 8 channels
        int ret = adc_read(adc_dev, &sequence);
        if (ret < 0)
        {
            LOG_ERR("ADC read failed: %d", ret);
            k_msleep(SAMPLE_PERIOD_MS);
            continue;
        }

        // Convert raw ADC values (driver provides them in millivolts by default)
        // We need raw ADC values for compatibility, so convert back
        for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
        {
            // Zephyr ADC driver may provide values in mV, convert to raw 14-bit
            // For now, just use the values as-is (may need adjustment based on actual behavior)
            // Raw 14-bit range: 0-16383
        }

        // Process samples (calibration or normal operation)
        process_adc_samples(adc_buffer);

        // Debug log periodically ONLY during active logging
        if (atomic_get(&logging_active) == 1)
        {
            static uint32_t debug_counter = 0;
            //    if (++debug_counter % 100 == 0)
            //   { // Every second at 100Hz
            //  LOG_INF("ADC values during activity: %d %d %d %d | %d %d %d %d",
            //        adc_buffer[0], adc_buffer[1], adc_buffer[2], adc_buffer[3],
            //      adc_buffer[4], adc_buffer[5], adc_buffer[6], adc_buffer[7]);
            // }
        }
        // Sleep for sampling period (10ms for 100Hz)
        k_msleep(SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief Initialize foot sensor module
 */
static void foot_sensor_init(void)
{
    LOG_INF("Initializing foot sensor module (simplified version)");

    // Create and start the sampling thread
    foot_sensor_tid = k_thread_create(&foot_sensor_thread_data, foot_sensor_stack,
                                      K_THREAD_STACK_SIZEOF(foot_sensor_stack), foot_sensor_thread, NULL, NULL, NULL,
                                      K_PRIO_PREEMPT(5), // Priority 5
                                      0, K_NO_WAIT);

    k_thread_name_set(foot_sensor_tid, "foot_sensor");

    LOG_INF("Foot sensor module initialized successfully");
    module_set_state(MODULE_STATE_READY);
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
            foot_sensor_init();
        }
        return false;
    }

    if (is_foot_sensor_start_activity_event(aeh))
    {
        LOG_INF("Received start activity event - enabling foot sensor sampling");
        atomic_set(&logging_active, 1);
        foot_sensor_streaming_enabled = false;
        return false;
    }

    if (is_foot_sensor_stop_activity_event(aeh))
    {
        LOG_INF("Received stop activity event - disabling foot sensor sampling");
        atomic_set(&logging_active, 0);
        return false;
    }

    if (is_streaming_control_event(aeh))
    {
        auto *event = cast_streaming_control_event(aeh);
        foot_sensor_streaming_enabled = event->foot_sensor_streaming_enabled;
        LOG_INF("foot_samples_work %s", foot_sensor_streaming_enabled ? "enabled" : "disabled");
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
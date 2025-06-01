/**
 * @file foot_sensor.cpp
 * @author Giorgio Guglielmino
 * @version 1.0.1
 * @date 2025-05-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <zephyr/arch/arm/arch.h>
#define MODULE foot_sensor

/*************************** INCLUDE HEADERS ********************************/
#include <cstring>
#include <time.h>
#include <variant>

#include <cstdio>
#include <cstring>

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <events/app_state_event.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <foot_sensor.hpp>

#include <errors.hpp>


LOG_MODULE_REGISTER(MODULE, CONFIG_FOOT_SENSOR_MODULE_LOG_LEVEL); // NOLINT

// ADC configuration
#define ADC_NODE		DT_NODELABEL(adc)
#define ADC_RESOLUTION	12
#define ADC_GAIN		ADC_GAIN_1_6
#define ADC_REFERENCE	ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_CHANNEL_COUNT 8

static int16_t adc_buffer[ADC_CHANNEL_COUNT];

static const struct adc_channel_cfg channel_cfgs[ADC_CHANNEL_COUNT] = {
    // Channel 0 (AIN0)
    {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = 0,
        .differential = 0,  // 0 = single-ended mode
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0,
    },
    // Channel 1 (AIN1)
    {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = 1,
        .differential = 0,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput1,
    },
    // Channel 2 (AIN2)
    {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = 2,
        .differential = 0,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput2,
    },
    // Channel 3 (AIN3)
    {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = 3,
        .differential = 0,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput3,
    },
    // Channel 4 (AIN4)
    {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = 4,
        .differential = 0,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput4,
    },
    // Channel 5 (AIN5)
    {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = 5,
        .differential = 0,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput5,
    },
    // Channel 6 (AIN6)
    {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = 6,
        .differential = 0,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput6,
    },
    // Channel 7 (AIN7)
    {
        .gain = ADC_GAIN,
        .reference = ADC_REFERENCE,
        .acquisition_time = ADC_ACQUISITION_TIME,
        .channel_id = 7,
        .differential = 0,
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput7,
    }
};

static const struct adc_sequence_options sequence_options = {
    .interval_us = 0,
    .callback = NULL,
    .extra_samplings = 1,
};

static const struct adc_sequence sequence = {
    .options = &sequence_options,
    .channels = BIT(0) | BIT(1) | BIT(2) | BIT(3) | 
                BIT(4) | BIT(5) | BIT(6) | BIT(7),
    .buffer = adc_buffer,
    .buffer_size = sizeof(adc_buffer),
    .resolution = ADC_RESOLUTION,
};

void foot_sensor_initializing_entry();

/********************************** Foot Sensor THREAD ********************************/
static constexpr int foot_sensor_stack_size = CONFIG_FOOT_SENSOR_MODULE_STACK_SIZE;
static constexpr int foot_sensor_priority = CONFIG_FOOT_SENSOR_MODULE_PRIORITY;
K_THREAD_STACK_DEFINE(foot_sensor_stack_area, foot_sensor_stack_size);
static struct k_thread foot_sensor_thread_data;
static k_tid_t foot_sensor_tid;
void foot_sensor_process(void * /*unused*/, void * /*unused*/, void * /*unused*/);


void foot_sensor_initializing_entry()
{

    // To initialise message que here

    LOG_INF("Foot Sensor initializing_entry done ");
}

static void foot_sensor_init()
{

    const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready\n");
        return;
    }

    // Configure all ADC channels
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        int err = adc_channel_setup(adc_dev, &channel_cfgs[i]);
        if (err < 0) {
            LOG_ERR("Failed to setup ADC channel %d (err %d)\n", i, err);
            return;
        }
    }

    foot_sensor_initializing_entry();

    foot_sensor_tid = k_thread_create(&foot_sensor_thread_data, foot_sensor_stack_area, K_THREAD_STACK_SIZEOF(foot_sensor_stack_area), foot_sensor_process,
                              nullptr, nullptr, nullptr, foot_sensor_priority, 0, K_NO_WAIT);

    LOG_INF("Foot Sensor Module Initialised");
}

void foot_sensor_process(void * /*unused*/, void * /*unused*/, void * /*unused*/)
{

    k_thread_name_set(foot_sensor_tid, "Foot_Sensor"); // sets the name of the thread

    module_set_state(MODULE_STATE_READY);

    while (true)
    {
        LOG_INF("I'm the Foot Sensor task");
        k_msleep(foot_sensor_timer);
    }
}


// Return type dictates if event is consumed. False = Not Consumed, True =
// Consumed.
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        if (check_state(event, MODULE_ID(bluetooth), MODULE_STATE_READY))
        {
            foot_sensor_init();
        }
        return false;
    }
    return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);


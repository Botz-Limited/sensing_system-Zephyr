/**
 * @file foot_sensor.cpp
 * @author Giorgio Guglielmino
 * @version 1.0.1
 * @date 2025-05-16
 *
 * @copyright Copyright (c) 2025
 *
 */
#define MODULE foot_sensor

#include <cstddef>
#include <cstdint>
#include <nrfx_config.h> // Ensure this is available and configured

#include <zephyr/arch/arm/arch.h>

/*************************** INCLUDE HEADERS ********************************/
#include <cstring>
#include <time.h>
#include <variant>

#include <cstdio>
// <cstring> is already included above

#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include <events/app_state_event.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Specific NRF HAL/nrfx/nrfy headers should be inside extern "C" if they are C libraries
extern "C"
{
// Core NRFX drivers (nrfx_dppi, nrfx_saadc, nrfx_timer for init and high-level)
#include <nrfx.h>
#include <nrfx_dppi.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>

// NRF HAL functions (for direct register access and basic peripheral structs/defines)
// Paths already adjusted to use <hal/xxx.h>
#include <hal/nrf_dppi.h>  // For NRF_DPPIC etc.
#include <hal/nrf_saadc.h> // For NRF_SAADC, nrf_saadc_value_t etc.
#include <hal/nrf_timer.h> // For NRF_TIMER1 etc.

// The nrfy headers are crucial here, as suggested by the compiler errors

#include "/home/ee/ncs/modules/hal/nordic/nrfx/haly//nrfy_dppi.h"
#include "/home/ee/ncs/modules/hal/nordic/nrfx/haly/nrfy_saadc.h" // For nrfy_saadc_config_t, nrfy_saadc_channel_init, nrfy_saadc_enable etc.
#include "/home/ee/ncs/modules/hal/nordic/nrfx/haly/nrfy_timer.h" // For nrfy_timer_event_enable, nrfy_timer_publish_set etc.
}

#include <errors.hpp>
#include <foot_sensor.hpp>

LOG_MODULE_REGISTER(MODULE, CONFIG_FOOT_SENSOR_MODULE_LOG_LEVEL); // NOLINT

#define SAADC_CHANNEL_COUNT 8
#define SAADC_SAMPLE_RATE_HZ 10 // Desired frequency for SAADC sampling
// ADJUSTMENT: Simplified TIME_TO_WAIT_US calculation to directly use SAADC_SAMPLE_RATE_HZ
#define TIME_TO_WAIT_US (uint32_t)(1000000UL / SAADC_SAMPLE_RATE_HZ)

// ADJUSTMENT: SAADC_BUFFER_SIZE is set to SAADC_CHANNEL_COUNT.
// This means the SAADC_EVENT_DONE will trigger after one complete set of 8 channel readings.
#define SAADC_BUFFER_SIZE SAADC_CHANNEL_COUNT // This will be 8
#define SAADC_IRQ_PRIORITY 1                  // Lower number means higher priority
#define BUFFER_COUNT 2UL                      // For double buffering
#define SAMPLING_ITERATIONS UINT16_MAX

#define TIMER_INSTANCE_ID 0

// Forward declarations
static void saadc_event_handler(nrfx_saadc_evt_t const *p_event);
static void timer_handler(nrf_timer_event_t event_type, void *p_context);
static void init_saadc(void);
static void init_timer(void);
static void calibrate_saadc_channels(void);
void init_dppi(void);
void TIMER0_IRQHandler(void);

// ADJUSTMENT: Added __aligned(4) for DMA-safe memory alignment
static int16_t saadc_buffer[BUFFER_COUNT][SAADC_BUFFER_SIZE] __aligned(4);

// Calibration specific variables
static int16_t saadc_offset[SAADC_CHANNEL_COUNT]; // Stores the zero-point offset for each channel
static bool calibration_done = false;             // Flag to indicate if calibration has been performed
static uint16_t calibration_sample_counter = 0;   // Counts samples collected during calibration
const uint16_t CALIBRATION_SAMPLES_TO_AVG = 100;   // Number of samples to average for calibration

// Use int32_t for the sum to prevent overflow during averaging, as raw values are positive.
static int32_t calibration_sum[SAADC_CHANNEL_COUNT] = {0};

static const nrfx_dppi_t m_dppi_instance = NRFX_DPPI_INSTANCE(0);

// Timer instance for triggering SAADC
nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(0); // Using TIMER0, confirm TIMER_INSTANCE_ID matches if different

// ADJUSTMENT: Two DPPI channel variables for distinct connections
uint8_t dppi_channel_timer_saadc;      // For Timer COMPARE0 -> SAADC SAMPLE
uint8_t dppi_channel_saadc_continuous; // For SAADC END -> SAADC START

// Define foot_sensor_timer. Assuming it's a duration in ms for k_msleep.
// You might want to get this from Kconfig or another source.
#define foot_sensor_timer 1000 // Default to 1000ms (1 second) if not defined

// This function is currently empty, kept as is.
void foot_sensor_initializing_entry();

static const nrf_saadc_input_t saadc_inputs[SAADC_CHANNEL_COUNT] = {
    NRF_SAADC_INPUT_AIN0, NRF_SAADC_INPUT_AIN1, NRF_SAADC_INPUT_AIN2, NRF_SAADC_INPUT_AIN3,
    NRF_SAADC_INPUT_AIN4, NRF_SAADC_INPUT_AIN5, NRF_SAADC_INPUT_AIN6, NRF_SAADC_INPUT_AIN7,
};

void foot_sensor_initializing_entry()
{
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, 0, 0);
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER0), IRQ_PRIO_LOWEST, TIMER0_IRQHandler, 0, 0);
    // Initialise queue here
    return;
}

static void foot_sensor_init()
{
    foot_sensor_initializing_entry(); // This function is currently empty

    // All hardware initialization (SAADC, Timer, DPPI) should be done here
    init_dppi();
    init_timer();
    init_saadc(); // This will internally call init_timer() and init_dppi()

    // Initiate the software calibration process.
    // The actual calibration samples will be collected by the SAADC_DONE handler
    // over the next CALIBRATION_SAMPLES_TO_AVG iterations.
    calibrate_saadc_channels();

    LOG_INF("Foot Sensor Module Initialized");

    module_set_state(MODULE_STATE_READY);
}

// --- SAADC event handler ---
static void saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
    nrfx_err_t status;
    (void)status; // Suppress unused variable warning

    static uint16_t buffer_index = 1;
    // MODIFICATION: Removed 'static uint16_t buf_req_evt_counter = 0;'
    // as it's no longer used for limiting continuous sampling.

    // NEW STATIC VARIABLES FOR CALIBRATION:
    // Counter for samples collected during the calibration phase
    static uint16_t calibration_sample_counter = 0;
    // Sum for averaging raw ADC values during calibration. Using int32_t to prevent overflow.
    static int32_t calibration_sum[SAADC_CHANNEL_COUNT] = {0};

    uint16_t samples_number = 0;
    // MODIFICATION: Changed 'uint16_t *data' to 'int16_t *data'
    // to match the buffer type and handle potential negative calibrated values.
    int16_t *data = NULL;

    switch(p_event->type)
    {
        case NRFX_SAADC_EVT_CALIBRATEDONE:
            LOG_INF("SAADC event: CALIBRATEDONE");
            status = nrfx_saadc_mode_trigger();
            NRFX_ASSERT(status == NRFX_SUCCESS);
            break;

        case NRFX_SAADC_EVT_READY:
            LOG_INF("SAADC event: READY");
            nrfx_dppi_channel_enable(&m_dppi_instance, dppi_channel_timer_saadc);
            LOG_DBG("SAADC sampling DPPI channel %u enabled.", dppi_channel_timer_saadc);
            break;

        case NRFX_SAADC_EVT_BUF_REQ:
            status = nrfx_saadc_buffer_set(saadc_buffer[buffer_index], SAADC_BUFFER_SIZE);
            NRFX_ASSERT(status == NRFX_SUCCESS);
            LOG_DBG("Provided buffer %u (%p) for next conversion.", buffer_index, saadc_buffer[buffer_index]);
            buffer_index = (buffer_index + 1) % BUFFER_COUNT;
            // MODIFICATION: Removed the 'if (++buf_req_evt_counter < SAMPLING_ITERATIONS)' block here
            // to allow truly continuous sampling without a software limit.
            break;

        case NRFX_SAADC_EVT_DONE:
            LOG_INF("SAADC event: DONE");

            samples_number = p_event->data.done.size;
            // MODIFICATION: Explicitly cast to int16_t* here as well.
            data = (int16_t *)p_event->data.done.p_buffer;

            // NEW BLOCK FOR CALIBRATION LOGIC:
            if (!calibration_done)
            {
                // Calibration phase: collect and average samples
                if (calibration_sample_counter < CALIBRATION_SAMPLES_TO_AVG)
                {
                    LOG_DBG("Calibrating... Sample set %u/%u", calibration_sample_counter + 1, CALIBRATION_SAMPLES_TO_AVG);
                    for (uint16_t i = 0; i < samples_number; i++)
                    {
                        calibration_sum[i] += data[i]; // Add raw value to sum
                    }
                    calibration_sample_counter++;

                    if (calibration_sample_counter == CALIBRATION_SAMPLES_TO_AVG)
                    {
                        // All calibration samples collected, calculate offsets
                        for (uint16_t i = 0; i < samples_number; i++)
                        {
                            saadc_offset[i] = (int16_t)(calibration_sum[i] / CALIBRATION_SAMPLES_TO_AVG);
                            LOG_INF("Channel %u calibrated offset: %d", i, saadc_offset[i]);
                        }
                        // Set flag to true: calibration is complete
                        calibration_done = true;
                        LOG_INF("SAADC calibration complete. Readings will now be offset.");
                    }
                }
            }
            else // MODIFICATION: New 'else' block for normal operation after calibration
            {
                // Calibration is done, process normal samples and apply offset
                for (uint16_t i = 0; i < samples_number; i++)
                {
                    // Apply calibration offset: subtract the stored offset from the raw value
                    int16_t calibrated_value = data[i] - saadc_offset[i];
                    LOG_INF("[Sample %u] value == %d (raw: %d)", i, calibrated_value, data[i]);
                }
            }
            break;

        case NRFX_SAADC_EVT_LIMIT:
            LOG_WRN("SAADC event: LIMIT on channel %d!", p_event->data.limit.channel);
            break;

        case NRFX_SAADC_EVT_FINISHED:
            LOG_INF("SAADC event: FINISHED - Sampling process has ended cleanly.");
            nrfx_dppi_channel_disable(&m_dppi_instance, dppi_channel_timer_saadc);
            LOG_DBG("SAADC sampling DPPI channel %u disabled on FINISHED event.", dppi_channel_timer_saadc);
            break;

        default:
            LOG_WRN("Unhandled SAADC Event Type: %d", p_event->type);
            break;
    }
}

/**
 * @brief Initiates a software calibration for all SAADC channels.
 * This function sets a flag that tells the SAADC event handler
 * to collect and average the next CALIBRATION_SAMPLES_TO_AVG samples.
 * Assumes no load on sensors during this process.
 */
static void calibrate_saadc_channels(void)
{
    LOG_INF("Starting SAADC calibration phase...");

    // Reset calibration state
    calibration_done = false;
    calibration_sample_counter = 0;
    // Clear the sum buffer for new calibration
    for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; ++i)
    {
        calibration_sum[i] = 0;
    }

    // The actual collection and calculation will happen in saadc_event_handler
    // during the NRFX_SAADC_EVT_DONE events.
}

// --- Timer handler (empty as per your use case, now marked with ARG_UNUSED) ---
static void timer_handler(nrf_timer_event_t event_type, void *p_context)
{
    ARG_UNUSED(event_type);
    ARG_UNUSED(p_context);
    // This handler remains empty as DPPI handles the direct trigger.
}

// --- Initialize timer ---
static void init_timer(void)
{
    nrfx_err_t err;

    // FIX: Use the global timer_inst directly instead of a local variable
    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
    // Configure the timer frequency to 4 MHz (source clock for TIMER0 if TIMER_INSTANCE_ID is 0)
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_config.p_context = NULL; // FIX: No context needed as handler is NULL and DPPI is used
    timer_config.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;

    // Initialize the global timer instance with the configuration. Handler is NULL as DPPI is used.
    err = nrfx_timer_init(&timer_inst, &timer_config, NULL);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Timer init failed: %d", err);
        return;
    }

    nrfx_timer_clear(&timer_inst); // FIX: Use global timer_inst

    // Calculate ticks for the desired sample rate (e.g., 1000 Hz)
    uint32_t ticks = nrfx_timer_us_to_ticks(&timer_inst, TIME_TO_WAIT_US); // FIX: Use global timer_inst

    // Set up compare channel 0: generate an event when timer reaches 'ticks', and clear timer on match.
    // Set 'false' for last argument if you only want DPPI trigger, not an interrupt.
    // nrfx_timer_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, ticks, false);

    nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

    // This line is redundant if compare is configured with 'clear_on_match = true'
    // However, in nrfx_timer_compare's last argument, it's boolean `interrupt_enable`, not `clear_on_match`.
    // So keeping one clear before enable is fine.
    // nrfx_timer_clear(&timer_inst); // FIX: Use global timer_inst - this one is now redundant for clarity.

    // Enable the timer.
    nrfx_timer_enable(&timer_inst); // FIX: Use global timer_inst
}

// --- Initialize DPPI and link timer event to SAADC sample task ---
void init_dppi(void)
{
    nrfx_err_t err;

    LOG_INF("Init DPPI");

    // REMOVED: nrfx_dppi_init() call
    // If your nrfx version is older than 3.8.0, this function doesn't exist.
    // The DPPI peripheral typically doesn't need an explicit 'nrfx_dppi_init()' call
    // in older versions; it's handled implicitly or by system init.

    // Allocate DPPI channel for Timer COMPARE0 -> SAADC SAMPLE
    err = nrfx_dppi_channel_alloc(&m_dppi_instance, &dppi_channel_timer_saadc);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to allocate DPPI channel (Timer->SAADC): %d", err);
        return;
    }

    // Link Timer COMPARE0 event to SAADC SAMPLE task
    nrfy_timer_publish_set(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0, dppi_channel_timer_saadc);
    nrfy_saadc_subscribe_set(NRF_SAADC, NRF_SAADC_TASK_SAMPLE, dppi_channel_timer_saadc);

    LOG_INF("Init DPPI completed. Channels will be enabled by SAADC_EVT_READY.");
}

// --- Initialize SAADC and channels ---
static void init_saadc(void)
{
    nrfx_err_t err;

    // 1. Initialize SAADC with interrupt priority using nrfx_saadc_init.
    err = nrfx_saadc_init(SAADC_IRQ_PRIORITY);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("SAADC driver init failed: %d", err);
        return;
    }

    // 2. Configure global SAADC settings using NRFY HAL functions.
    nrfy_saadc_resolution_set(NRF_SAADC, NRF_SAADC_RESOLUTION_14BIT);
    nrfy_saadc_oversample_set(NRF_SAADC, NRF_SAADC_OVERSAMPLE_DISABLED);

    // Declare an array to hold all 8 channel configurations
    nrfx_saadc_channel_t all_nrfx_channels[SAADC_CHANNEL_COUNT];

    // 3. Populate the configuration for each SAADC channel into the array.
    for (int i = 0; i < SAADC_CHANNEL_COUNT; i++)
    {
        all_nrfx_channels[i].channel_config = {.gain = NRF_SAADC_GAIN1_6,
                                               .reference = NRF_SAADC_REFERENCE_INTERNAL,
                                               .acq_time = NRF_SAADC_ACQTIME_10US,
                                               .mode = NRF_SAADC_MODE_SINGLE_ENDED,
                                               .burst = NRF_SAADC_BURST_DISABLED};

        all_nrfx_channels[i].pin_p = static_cast<nrf_saadc_input_t>(saadc_inputs[i]); // Positive input pin
        all_nrfx_channels[i].pin_n = NRF_SAADC_INPUT_DISABLED; // Single-ended (connect to ground internally)
        all_nrfx_channels[i].channel_index = (uint8_t)i;
    }

    // 4. Call nrfx_saadc_channels_config ONCE with the array of all configurations.
    err = nrfx_saadc_channels_config(all_nrfx_channels, SAADC_CHANNEL_COUNT);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("Failed to configure all SAADC channels: %d", err);
        return;
    }

    // 5. Set up advanced mode for external triggering and continuous sampling.
    nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    adv_config.internal_timer_cc = 0;
    adv_config.start_on_end = true;                               // External trigger from timer/DPPI
    uint32_t channel_mask = nrfx_saadc_channels_configured_get(); // Gets mask of all 8 configured channels

    // Pass your saadc_event_handler to process events.
    err = nrfx_saadc_advanced_mode_set(channel_mask,
                                       NRF_SAADC_RESOLUTION_14BIT,      
                                       &adv_config, saadc_event_handler); 
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("SAADC advanced mode set failed: %d", err);
        return;
    }

    // 6. Enable the SAADC peripheral after all configuration.
    nrfy_saadc_enable(NRF_SAADC);

    // FIX: Provide BOTH initial buffers to the SAADC driver for double buffering.
    // This is the correct way to do it for a ping-pong handler like yours.
    err = nrfx_saadc_buffer_set(saadc_buffer[0], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("SAADC buffer 0 set failed: %d", err);
        return;
    }
    err = nrfx_saadc_buffer_set(saadc_buffer[1], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("SAADC buffer 1 set failed: %d", err);
        return;
    }

    // This starts the first conversion cycle. After this, the DPPI chain
    // (Timer -> SAADC SAMPLE, and SAADC END -> SAADC START) will keep it going.
    // nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_START); // <-- UN-COMMENT THIS LINE!
    nrfx_saadc_offset_calibrate(saadc_event_handler); // This is for calibration, not continuous sampling initiation.
}

// --- IRQ handler for SAADC (call nrfx handler) ---
// This is the standard way Zephyr connects NRFX IRQ handlers.
void SAADC_IRQHandler(void)
{
    LOG_INF("Got IRQ SAADC");
    nrfx_saadc_irq_handler();
}

void TIMER0_IRQHandler(void)
{
    // Clear the event explicitly in the NRF_TIMER peripheral
    LOG_INF("Got Time IRQ");
    if (nrf_timer_event_check(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0))
    {
        nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0);
    }
}

// Return type dictates if event is consumed. False = Not Consumed, True = Consumed.
static bool app_event_handler(const struct app_event_header *aeh)
{
    if (is_module_state_event(aeh))
    {
        auto *event = cast_module_state_event(aeh);

        // Check if Bluetooth is ready before initializing foot sensor module
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
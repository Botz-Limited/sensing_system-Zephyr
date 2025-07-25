/**
 * @file foot_sensor.cpp
 * @author
 * @brief
 * @version 1.0.0
 * @date 2025-05-12
 *
 * @copyright Botz Innovation 2025
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
#include <events/foot_sensor_event.h>
#if defined(CONFIG_WIFI_MODULE)
#include <events/wifi_event.h>
#endif
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

// Specific NRF HAL/nrfx/nrfy headers should be inside extern "C" if they are C
// libraries
extern "C" {
// Core NRFX drivers (nrfx_dppi, nrfx_saadc, nrfx_timer for init and high-level)
#include <nrfx.h>
#include <nrfx_dppi.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>

// NRF HAL functions (for direct register access and basic peripheral
// structs/defines) Paths already adjusted to use <hal/xxx.h>
#include <hal/nrf_dppi.h>  // For NRF_DPPIC etc.
#include <hal/nrf_saadc.h> // For NRF_SAADC, nrf_saadc_value_t etc.
#include <hal/nrf_timer.h> // For NRF_TIMER0 etc.

#include "/home/ee/ncs/modules/hal/nordic/nrfx/haly//nrfy_dppi.h"
#include "/home/ee/ncs/modules/hal/nordic/nrfx/haly/nrfy_saadc.h" // For nrfy_saadc_config_t, nrfy_saadc_channel_init, nrfy_saadc_enable etc.
#include "/home/ee/ncs/modules/hal/nordic/nrfx/haly/nrfy_timer.h" // For nrfy_timer_event_enable, nrfy_timer_publish_set etc.
}

#include <app.hpp>
#include <app_version.h>
#include <ble_services.hpp>
#include <errors.hpp>
#include <foot_sensor.hpp>
#include <status_codes.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_FOOT_SENSOR_MODULE_LOG_LEVEL); // NOLINT

static constexpr uint32_t US_PER_SECOND = 1000000UL;
static constexpr uint8_t SAADC_CHANNEL_COUNT = 8;
static constexpr uint8_t SAADC_SAMPLE_RATE_HZ_NORMAL =
    80; // normal sampling rate (80 Hz)
static constexpr uint8_t SAADC_SAMPLE_RATE_HZ_CALIBRATION =
    200; // Higher rate for quick calibration

static constexpr uint32_t TIME_TO_WAIT_US_NORMAL =
    US_PER_SECOND / SAADC_SAMPLE_RATE_HZ_NORMAL;
// New definition for calibration
static constexpr uint32_t TIME_TO_WAIT_US_CALIBRATION =
    US_PER_SECOND / SAADC_SAMPLE_RATE_HZ_CALIBRATION;

// SAADC_BUFFER_SIZE is set to SAADC_CHANNEL_COUNT.
// This means the SAADC_EVENT_DONE will trigger after one complete set of 8
// channel readings.
static constexpr uint8_t SAADC_BUFFER_SIZE = SAADC_CHANNEL_COUNT;
static constexpr uint8_t SAADC_IRQ_PRIORITY =
    1;                                     // Lower number means higher priority
static constexpr uint8_t BUFFER_COUNT = 2; // For double buffering
static constexpr uint16_t SAMPLING_ITERATIONS =
    UINT16_MAX; // Continue sampling, never stop

static constexpr uint8_t TIMER_INSTANCE_ID = 0;

// Forward declarations
static void saadc_event_handler(nrfx_saadc_evt_t const *p_event);
static void timer_handler(nrf_timer_event_t event_type, void *p_context);
static err_t init_saadc(void);
static void init_timer(void);
static void calibrate_saadc_channels(void);
void init_dppi(void);
void TIMER0_IRQHandler(void);

// Use atomic for thread-safe access to logging state
static atomic_t logging_active = ATOMIC_INIT(0);
static atomic_t wifi_active = ATOMIC_INIT(0);

// ADJUSTMENT: Added __aligned(4) for DMA-safe memory alignment
static int16_t saadc_buffer[BUFFER_COUNT][SAADC_BUFFER_SIZE] __aligned(4);

// Calibration specific variables
static int16_t saadc_offset[SAADC_CHANNEL_COUNT]; // Stores the zero-point
                                                  // offset for each channel
static bool calibration_done =
    false; // Flag to indicate if calibration has been performed
static uint16_t calibration_sample_counter =
    0; // Counts samples collected during calibration
const uint16_t CALIBRATION_SAMPLES_TO_AVG =
    100; // Number of samples to average for calibration

// Use int32_t for the sum to prevent overflow during averaging, as raw values
// are positive.
static int32_t calibration_sum[SAADC_CHANNEL_COUNT] = {0};

static const nrfx_dppi_t m_dppi_instance = NRFX_DPPI_INSTANCE(0);

// Timer instance for triggering SAADC
nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(0); // Using TIMER0

// ADJUSTMENT: Two DPPI channel variables for distinct connections
uint8_t dppi_channel_timer_saadc;      // For Timer COMPARE0 -> SAADC SAMPLE
uint8_t dppi_channel_saadc_continuous; // For SAADC END -> SAADC START

// Define foot_sensor_timer. Assuming it's a duration in ms for k_msleep.
// You might want to get this from Kconfig or another source.
static constexpr uint16_t foot_sensor_timer =
    1000; // Default to 1000ms (1 second) if not defined

// This function is currently empty, kept as is.
void foot_sensor_initializing_entry();

static const nrf_saadc_input_t saadc_inputs[SAADC_CHANNEL_COUNT] = {
    NRF_SAADC_INPUT_AIN0, NRF_SAADC_INPUT_AIN1, NRF_SAADC_INPUT_AIN2,
    NRF_SAADC_INPUT_AIN3, NRF_SAADC_INPUT_AIN4, NRF_SAADC_INPUT_AIN5,
    NRF_SAADC_INPUT_AIN6, NRF_SAADC_INPUT_AIN7,
};

void foot_sensor_initializing_entry() {
  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST,
              nrfx_saadc_irq_handler, 0, 0);
  IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER0), IRQ_PRIO_LOWEST,
              TIMER0_IRQHandler, 0, 0);
  // Initialise queue here
  return;
}

static void foot_sensor_init() {
  bool init_failed = false;
  err_t result = err_t::NO_ERROR;

  foot_sensor_initializing_entry(); // This function is currently empty

  // All hardware initialization (SAADC, Timer, DPPI) should be done here
  init_dppi();
  init_timer();

  // Initialize SAADC and check result
  result = init_saadc();
  if (result != err_t::NO_ERROR) {
    LOG_ERR("SAADC initialization failed");
    init_failed = true;
  }

  if (init_failed) {
      // Report error to Bluetooth module via message queue
      generic_message_t err_msg;
      err_msg.sender = SENDER_FOOT_SENSOR_THREAD;
      err_msg.type = MSG_TYPE_ERROR_STATUS;
      err_msg.data.error_status.error_code = err_t::ADC_ERROR;
      err_msg.data.error_status.is_set = true;
      if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send error status to Bluetooth module");
      } else {
          LOG_INF("Sent error status to Bluetooth module");
      }

#if IS_ENABLED(CONFIG_FOOT_SENSOR_OPTIONAL)
    LOG_WRN("Foot sensor initialization failed but continuing (non-critical)");
    module_set_state(MODULE_STATE_READY);
#else
    LOG_ERR("Foot sensor initialization failed (critical)");
    module_set_state(MODULE_STATE_ERROR);
#endif /* CONFIG_WIFI_NRF70 */
    return;
  }

  // Initiate the software calibration process.
  // The actual calibration samples will be collected by the SAADC_DONE handler
  // over the next CALIBRATION_SAMPLES_TO_AVG iterations.
  calibrate_saadc_channels();

  LOG_INF("Foot Sensor Module Initialized");

  module_set_state(MODULE_STATE_READY);
}

// --- SAADC event handler ---
static void saadc_event_handler(nrfx_saadc_evt_t const *p_event) {
  nrfx_err_t status;
  (void)status; // Suppress unused variable warning

  static uint16_t buffer_index = 0;
  // MODIFICATION: Removed 'static uint16_t buf_req_evt_counter = 0;'
  // as it's no longer used for limiting continuous sampling.


  uint16_t samples_number = 0;

  int16_t *raw_adc_data = NULL;

  switch (p_event->type) {
  case NRFX_SAADC_EVT_CALIBRATEDONE:
    status = nrfx_saadc_mode_trigger();
    NRFX_ASSERT(status == NRFX_SUCCESS);
    break;

  case NRFX_SAADC_EVT_READY:
    nrfx_dppi_channel_enable(&m_dppi_instance, dppi_channel_timer_saadc);
    break;

  case NRFX_SAADC_EVT_BUF_REQ:
    LOG_DBG("SAADC BUF_REQ: Setting buffer[%d]", buffer_index);
    status =
        nrfx_saadc_buffer_set(saadc_buffer[buffer_index], SAADC_BUFFER_SIZE);
    if (status != NRFX_SUCCESS) {
      LOG_ERR("Failed to set SAADC buffer[%d]: %d", buffer_index, status);
      generic_message_t err_msg;
      err_msg.sender = SENDER_FOOT_SENSOR_THREAD;
      err_msg.type = MSG_TYPE_ERROR_STATUS;
      err_msg.data.error_status.error_code = err_t::ADC_ERROR;
      err_msg.data.error_status.is_set = true;
      if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send error status to Bluetooth module");
      } else {
          LOG_INF("Sent error status to Bluetooth module");
      }
      // Don't assert - just skip this buffer
    } else {
      buffer_index = (buffer_index + 1) % BUFFER_COUNT;
    }
    break;

  case NRFX_SAADC_EVT_DONE:
    samples_number = p_event->data.done.size;
    // Assign the buffer to our new uint16_t pointer
    raw_adc_data = (int16_t *)p_event->data.done.p_buffer;

    // Validate pointer before use
    if (!raw_adc_data) {
      LOG_ERR("SAADC buffer pointer is NULL");
      break;
    }

    if (!calibration_done) {
      if (calibration_sample_counter == 0) {
        LOG_INF("Calibrating...");
      }
      // Calibration phase: collect and average samples
      if (calibration_sample_counter < CALIBRATION_SAMPLES_TO_AVG) {
        LOG_DBG("Calibrating... Sample set %u/%u",
                calibration_sample_counter + 1, CALIBRATION_SAMPLES_TO_AVG);
        for (uint16_t i = 0; i < samples_number; i++) {
          // Use raw_adc_data[i] for summation
          calibration_sum[i] += raw_adc_data[i];
        }
        calibration_sample_counter++;

        // Check if we've collected all calibration samples
        if (calibration_sample_counter >= CALIBRATION_SAMPLES_TO_AVG) {
          // All calibration samples collected, calculate offsets
          for (uint16_t i = 0; i < samples_number; i++) {
            // Offset itself can be negative if calibration point is high, so
            // int16_t is fine.
            saadc_offset[i] =
                (int16_t)(calibration_sum[i] / CALIBRATION_SAMPLES_TO_AVG);
          }
          // Set flag to true: calibration is complete
          calibration_done = true;
          LOG_INF("SAADC calibration complete. Readings will now be offset.");
          // Adjust timer to normal sampling rate after calibration
          nrfx_timer_disable(&timer_inst);
          uint32_t ticks =
              nrfx_timer_us_to_ticks(&timer_inst, TIME_TO_WAIT_US_NORMAL);
          nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, ticks,
                                      NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                      false);
          nrfx_timer_clear(&timer_inst); // Keep this clear to restart timer
                                         // from 0 with new compare value
          nrfx_timer_enable(&timer_inst);
          LOG_INF("Switched SAADC sampling rate to %u Hz (normal operation).",
                  SAADC_SAMPLE_RATE_HZ_NORMAL);
        }
      }
    } else // Normal operation after calibration
    {
      if (atomic_get(&logging_active) == 0) {

        generic_message_t msg;

        msg.sender = SENDER_FOOT_SENSOR_THREAD;
        msg.type = MSG_TYPE_FOOT_SAMPLES;

        // Populate the foot sensor data directly into the message's union
        // member.
        for (uint8_t i = 0; i < samples_number; i++) {
          int16_t calibrated_value = raw_adc_data[i] - saadc_offset[i];
          msg.data.foot_samples.values[i] =
              (calibrated_value < 0) ? 0 : calibrated_value;
        }

        if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send foot sensor data to bluetooth module");
        }

        // Don't log foot sample to data module, commented out for now/
        //    if (k_msgq_put(&data_msgq, &msg, K_NO_WAIT) != 0)
        //   {
        //      LOG_WRN("Failed to send foot sensor data to data module");
        //   }

// Send to sensor data module (new multi-thread architecture)
#if IS_ENABLED(CONFIG_SENSOR_DATA_MODULE)
        if (k_msgq_put(&sensor_data_msgq, &msg, K_NO_WAIT) != 0) {
          LOG_WRN("Failed to send foot sensor data to sensor data module");
        }
#endif

        // Also send to WiFi module if WiFi is active
#if defined(CONFIG_WIFI_MODULE)
        if (atomic_get(&wifi_active) == 1) {
          if (k_msgq_put(&wifi_msgq, &msg, K_NO_WAIT) != 0) {
            LOG_WRN("Failed to send foot sensor data to wifi module");
          }
        }
#endif
      }
    }
    break;

  case NRFX_SAADC_EVT_LIMIT:
    LOG_WRN("SAADC event: LIMIT on channel %d!", p_event->data.limit.channel);
    break;

  case NRFX_SAADC_EVT_FINISHED:
    LOG_INF("SAADC event: FINISHED - Sampling process has ended cleanly.");
    nrfx_dppi_channel_disable(&m_dppi_instance, dppi_channel_timer_saadc);
    LOG_DBG("SAADC sampling DPPI channel %u disabled on FINISHED event.",
            dppi_channel_timer_saadc);
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
static void calibrate_saadc_channels(void) {
  LOG_INF("Starting SAADC calibration phase...");

  // Reset calibration state
  calibration_done = false;
  calibration_sample_counter = 0;
  // Clear the sum buffer for new calibration
  for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; ++i) {
    calibration_sum[i] = 0;
  }

  // The actual collection and calculation will happen in saadc_event_handler
  // during the NRFX_SAADC_EVT_DONE events.
}

// --- Timer handler (empty as per your use case, now marked with ARG_UNUSED)
// --- Currently unused - DPPI handles the direct trigger
__attribute__((unused)) static void timer_handler(nrf_timer_event_t event_type,
                                                  void *p_context) {
  ARG_UNUSED(event_type);
  ARG_UNUSED(p_context);
  // This handler remains empty as DPPI handles the direct trigger.
}

// --- Initialize timer ---
static void init_timer(void) {
  nrfx_err_t err;

  uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
  nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
  timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
  timer_config.p_context = NULL;
  timer_config.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;

  // Initialize the global timer instance with the configuration. Handler is
  // NULL as DPPI is used.
  err = nrfx_timer_init(&timer_inst, &timer_config, NULL);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("Timer init failed: %d", err);
    return;
  }

  nrfx_timer_clear(&timer_inst);

  // Calculate ticks for the desired sample rate (e.g., 1000 Hz)
  uint32_t ticks =
      nrfx_timer_us_to_ticks(&timer_inst, TIME_TO_WAIT_US_CALIBRATION);

  // Set up compare channel 0: generate an event when timer reaches 'ticks', and
  // clear timer on match. Set 'false' for last argument if you only want DPPI
  // trigger, not an interrupt. nrfx_timer_compare(&timer_inst,
  // NRF_TIMER_CC_CHANNEL0, ticks, false);

  nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

  // This line is redundant if compare is configured with 'clear_on_match =
  // true' However, in nrfx_timer_compare's last argument, it's boolean
  // `interrupt_enable`, not `clear_on_match`. So keeping one clear before
  // enable is fine.
  nrfx_timer_clear(&timer_inst);

  // Enable the timer.
  nrfx_timer_enable(&timer_inst);
}

// --- Initialize DPPI and link timer event to SAADC sample task ---
void init_dppi(void) {
  nrfx_err_t err;

  LOG_INF("Init DPPI");

  // Allocate DPPI channel for Timer COMPARE0 -> SAADC SAMPLE
  err = nrfx_dppi_channel_alloc(&m_dppi_instance, &dppi_channel_timer_saadc);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("Failed to allocate DPPI channel (Timer->SAADC): %d", err);
    return;
  }

  // Link Timer COMPARE0 event to SAADC SAMPLE task
  nrfy_timer_publish_set(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0,
                         dppi_channel_timer_saadc);
  nrfy_saadc_subscribe_set(NRF_SAADC, NRF_SAADC_TASK_SAMPLE,
                           dppi_channel_timer_saadc);

  LOG_INF("Init DPPI completed. Channels will be enabled by SAADC_EVT_READY.");
}

// --- Initialize SAADC and channels ---
static err_t init_saadc(void) {
  nrfx_err_t err;

  // 1. Initialize SAADC with interrupt priority using nrfx_saadc_init.
  err = nrfx_saadc_init(SAADC_IRQ_PRIORITY);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("SAADC driver init failed: %d", err);
    generic_message_t err_msg;
    err_msg.sender = SENDER_FOOT_SENSOR_THREAD;
    err_msg.type = MSG_TYPE_ERROR_STATUS;
    err_msg.data.error_status.error_code = err_t::ADC_ERROR;
    err_msg.data.error_status.is_set = true;
    if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send error status to Bluetooth module");
    } else {
        LOG_INF("Sent error status to Bluetooth module");
    }
    return err_t::ADC_ERROR;
  }

  // 2. Configure global SAADC settings using NRFY HAL functions.
  nrfy_saadc_resolution_set(NRF_SAADC, NRF_SAADC_RESOLUTION_14BIT);
  nrfy_saadc_oversample_set(NRF_SAADC, NRF_SAADC_OVERSAMPLE_DISABLED);

  // Declare an array to hold all 8 channel configurations
  nrfx_saadc_channel_t all_nrfx_channels[SAADC_CHANNEL_COUNT];

  // 3. Populate the configuration for each SAADC channel into the array.
  for (int i = 0; i < SAADC_CHANNEL_COUNT; i++) {
    all_nrfx_channels[i].channel_config = {.gain = NRF_SAADC_GAIN1_6,
                                           .reference =
                                               NRF_SAADC_REFERENCE_INTERNAL,
                                           .acq_time = NRF_SAADC_ACQTIME_10US,
                                           .mode = NRF_SAADC_MODE_SINGLE_ENDED,
                                           .burst = NRF_SAADC_BURST_DISABLED};

    all_nrfx_channels[i].pin_p =
        static_cast<nrf_saadc_input_t>(saadc_inputs[i]); // Positive input pin
    all_nrfx_channels[i].pin_n =
        NRF_SAADC_INPUT_DISABLED; // Single-ended (connect to ground internally)
    all_nrfx_channels[i].channel_index = (uint8_t)i;
  }

  // Call nrfx_saadc_channels_config ONCE with the array of all configurations.
  err = nrfx_saadc_channels_config(all_nrfx_channels, SAADC_CHANNEL_COUNT);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("Failed to configure all SAADC channels: %d", err);
    return err_t::ADC_ERROR;
  }

  // Set up advanced mode for external triggering and continuous sampling.
  nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
  adv_config.internal_timer_cc = 0;
  adv_config.start_on_end = true; // External trigger from timer/DPPI
  uint32_t channel_mask =
      nrfx_saadc_channels_configured_get(); // Gets mask of all 8 configured
                                            // channels

  // Pass your saadc_event_handler to process events.
  err = nrfx_saadc_advanced_mode_set(channel_mask, NRF_SAADC_RESOLUTION_14BIT,
                                     &adv_config, saadc_event_handler);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("SAADC advanced mode set failed: %d", err);
    generic_message_t err_msg;
    err_msg.sender = SENDER_FOOT_SENSOR_THREAD;
    err_msg.type = MSG_TYPE_ERROR_STATUS;
    err_msg.data.error_status.error_code = err_t::ADC_ERROR;
    err_msg.data.error_status.is_set = true;
    if (k_msgq_put(&bluetooth_msgq, &err_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send error status to Bluetooth module");
    } else {
        LOG_INF("Sent error status to Bluetooth module");
    }
    return err_t::ADC_ERROR;
  }

  // Enable the SAADC peripheral after all configuration.
  nrfy_saadc_enable(NRF_SAADC);

  err = nrfx_saadc_buffer_set(saadc_buffer[0], SAADC_BUFFER_SIZE);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("SAADC buffer 0 set failed: %d", err);
    return err_t::ADC_ERROR;
  }
  err = nrfx_saadc_buffer_set(saadc_buffer[1], SAADC_BUFFER_SIZE);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("SAADC buffer 1 set failed: %d", err);
    return err_t::ADC_ERROR;
  }

  // This starts the first conversion cycle. After this, the DPPI chain
  // (Timer -> SAADC SAMPLE, and SAADC END -> SAADC START) will keep it going.
  // nrf_saadc_task_trigger(NRF_SAADC, NRF_SAADC_TASK_START); // <-- UN-COMMENT
  // THIS LINE!
  nrfx_saadc_offset_calibrate(
      saadc_event_handler); // This is for calibration, not continuous sampling
                            // initiation.

  return err_t::NO_ERROR;
}

// --- IRQ handler for SAADC (call nrfx handler) ---
// This is the standard way Zephyr connects NRFX IRQ handlers.
void SAADC_IRQHandler(void) {
  LOG_INF("Got IRQ SAADC");
  nrfx_saadc_irq_handler();
}

void TIMER0_IRQHandler(void) {
  // Clear the event explicitly in the NRF_TIMER peripheral
  LOG_INF("Got Time IRQ");
  if (nrf_timer_event_check(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0)) {
    nrf_timer_event_clear(NRF_TIMER0, NRF_TIMER_EVENT_COMPARE0);
  }
}

// Return type dictates if event is consumed. False = Not Consumed, True =
// Consumed.

static bool app_event_handler(const struct app_event_header *aeh) {
  if (is_module_state_event(aeh)) {
    auto *event = cast_module_state_event(aeh);

    // Check if Bluetooth is ready before initializing foot sensor module
    if (check_state(event, MODULE_ID(bluetooth), MODULE_STATE_READY)) {
      foot_sensor_init();
    }
    return false;
  }
  if (is_foot_sensor_start_activity_event(aeh)) {
    // Trigger the same logic as auto-start logging
    if (atomic_get(&logging_active) == 0) {
      generic_message_t cmd_msg = {};
      cmd_msg.sender = SENDER_FOOT_SENSOR_THREAD;
      cmd_msg.type = MSG_TYPE_COMMAND;
      strncpy(cmd_msg.data.command_str, "START_LOGGING_FOOT_SENSOR",
              sizeof(cmd_msg.data.command_str) - 1);
      cmd_msg.data.command_str[sizeof(cmd_msg.data.command_str) - 1] = '\0';
      cmd_msg.sampling_frequency = SAADC_SAMPLE_RATE_HZ_NORMAL;
      strncpy(cmd_msg.fw_version, APP_VERSION_STRING,
              sizeof(cmd_msg.fw_version) - 1);
      cmd_msg.fw_version[sizeof(cmd_msg.fw_version) - 1] = '\0';

      // Don't want to log foot sample data at the moment.

      //   int cmd_ret = k_msgq_put(&data_msgq, &cmd_msg, K_NO_WAIT);
      //  if (cmd_ret == 0)
      //  {
      //     LOG_INF("Sent START_LOGGING_FOOT_SENSOR command (via event)");
      //    atomic_set(&logging_active, 1);
      //  }
      //  else
      //  {
      //     LOG_ERR("Failed to send START_LOGGING_FOOT_SENSOR command: %d",
      //     cmd_ret);
      // }
    }
    return false;
  }
  if (is_foot_sensor_stop_activity_event(aeh)) {
    LOG_INF("Received foot_sensor_stop_activity_event");
    if (atomic_get(&logging_active) == 1) {
      generic_message_t cmd_msg = {};
      cmd_msg.sender = SENDER_FOOT_SENSOR_THREAD;
      cmd_msg.type = MSG_TYPE_COMMAND;
      strncpy(cmd_msg.data.command_str, "STOP_LOGGING_FOOT_SENSOR",
              sizeof(cmd_msg.data.command_str) - 1);
      cmd_msg.data.command_str[sizeof(cmd_msg.data.command_str) - 1] = '\0';
      // Don't want to log foot sample data at the moment.
      //  int cmd_ret = k_msgq_put(&data_msgq, &cmd_msg, K_NO_WAIT);
      //  if (cmd_ret == 0)
      // {
      //    LOG_INF("Sent STOP_LOGGING_FOOT_SENSOR command (via event)");
      //   atomic_set(&logging_active, 0);
      // }
      // else
      // {
      //    LOG_ERR("Failed to send STOP_LOGGING_FOOT_SENSOR command: %d",
      //    cmd_ret);
      // }
    }
    return false;
  }
#if defined(CONFIG_WIFI_MODULE)
  if (is_wifi_connected_event(aeh)) {
    LOG_INF("Foot sensor: WiFi connected - enabling WiFi data transmission");
    atomic_set(&wifi_active, 1);
    return false;
  }
  if (is_wifi_disconnected_event(aeh)) {
    LOG_INF(
        "Foot sensor: WiFi disconnected - disabling WiFi data transmission");
    atomic_set(&wifi_active, 0);
    return false;
  }
#endif
  return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, app_state_event);
APP_EVENT_SUBSCRIBE_FIRST(MODULE, bluetooth_state_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_start_activity_event);
APP_EVENT_SUBSCRIBE(MODULE, foot_sensor_stop_activity_event);
#if defined(CONFIG_WIFI_MODULE)
APP_EVENT_SUBSCRIBE(MODULE, wifi_connected_event);
APP_EVENT_SUBSCRIBE(MODULE, wifi_disconnected_event);
#endif
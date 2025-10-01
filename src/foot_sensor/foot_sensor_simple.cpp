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
extern struct k_msgq data_msgq; // For sending calibration data to data module
#if IS_ENABLED(CONFIG_LAB_VERSION)
extern struct k_msgq data_sd_msgq; // For sending to data_sd module (lab version)
#endif

// Constants
static constexpr uint8_t SAADC_CHANNEL_COUNT = 8;
static constexpr uint8_t CALIBRATION_SAMPLES = 100;

static constexpr uint32_t SAMPLE_PERIOD_MS = 1000 / SAMPLE_RATE_HZ; // 5ms
static constexpr uint8_t BLUETOOTH_RATE_DIVIDER = 10;               // Send to BLE at 10Hz (100Hz / 10)


//for VDD only monitoring 
static constexpr uint32_t VDD_MONITOR_INTERVAL_MS = 1000; // Check VDD every 5 seconds
static constexpr uint16_t VDD_LOW_THRESHOLD_MV = 3300;    // 3.3V threshold
static int32_t vdd_voltage_mv = 0;
static uint32_t last_vdd_check_ms = 0;
static err_t configure_channel_for_vdd(uint8_t channel_num);
static err_t restore_channel_to_pressure(uint8_t channel_num);
static err_t read_vdd_voltage(uint8_t temp_channel);
//End VDD monitoring constants

// Thread stack and priority
K_THREAD_STACK_DEFINE(foot_sensor_stack, 4096);
static struct k_thread foot_sensor_thread_data;
static k_tid_t foot_sensor_tid = NULL;

// Use atomic for thread-safe access to logging state
static atomic_t logging_active = ATOMIC_INIT(0);

// Local streaming state
static bool foot_sensor_streaming_enabled = false;

// Weight map calibration state
static bool weight_map_calibration_active = false;
static uint32_t weight_map_sample_count = 0;
static constexpr uint32_t WEIGHT_MAP_SAMPLES = 500;        // 5 seconds at 100Hz
static uint32_t weight_map_sum[SAADC_CHANNEL_COUNT] = {0}; // Changed to uint32_t to avoid overflow
static foot_weight_map_data_t weight_map_data;

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
static void start_weight_map_calibration(void);
static void process_weight_map_sample(int16_t *raw_data);
static void complete_weight_map_calibration(void);

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
        channel_configs[i].gain = ADC_GAIN_1_6;
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
 * @brief Start weight map calibration
 */
static void start_weight_map_calibration(void)
{
    LOG_INF("Starting foot weight map calibration - stand still for 5 seconds");

    // Reset calibration state
    weight_map_calibration_active = true;
    weight_map_sample_count = 0;
    memset(weight_map_sum, 0, sizeof(weight_map_sum));
    memset(&weight_map_data, 0, sizeof(weight_map_data));
}

/**
 * @brief Process weight map calibration sample
 */
static void process_weight_map_sample(int16_t *raw_data)
{
    if (weight_map_sample_count < WEIGHT_MAP_SAMPLES)
    {
        // Accumulate sensor values
        for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
        {
            // Use raw values without offset calibration for weight map
            // ADC is 14-bit (0-16383), ensure we handle as unsigned
            uint16_t value = (raw_data[i] < 0) ? 0 : (uint16_t)raw_data[i];

            // Accumulate safely - max possible: 16383 * 500 = 8,191,500 (fits in uint32_t)
            weight_map_sum[i] += value;
        }

        weight_map_sample_count++;

        // Log progress every second (100 samples)
        if (weight_map_sample_count % 100 == 0)
        {
            LOG_INF("Weight map calibration progress: %d/%d samples", weight_map_sample_count, WEIGHT_MAP_SAMPLES);
        }

        // Check if calibration is complete
        if (weight_map_sample_count >= WEIGHT_MAP_SAMPLES)
        {
            complete_weight_map_calibration();
        }
    }
}

/**
 * @brief Complete weight map calibration and send data
 */
static void complete_weight_map_calibration(void)
{
    LOG_INF("Weight map calibration complete - processing data");

    // Calculate averages and total weight
    uint32_t total = 0;
    for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
    {
        weight_map_data.sensor_map[i] = (uint16_t)(weight_map_sum[i] / WEIGHT_MAP_SAMPLES);
        total += weight_map_data.sensor_map[i];
    }

    weight_map_data.total_weight = total;
    weight_map_data.sample_count = WEIGHT_MAP_SAMPLES;
    weight_map_data.timestamp_ms = k_uptime_get_32();
    weight_map_data.is_valid = true;

    // Calculate weight distribution percentages
    if (total > 0)
    {
        for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
        {
            weight_map_data.weight_distribution[i] = (float)weight_map_data.sensor_map[i] * 100.0f / (float)total;
        }
    }

    // Log the calibration results
    LOG_INF("Weight map calibration results:");
    LOG_INF("  Total weight: %u", weight_map_data.total_weight);
    for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
    {
        LOG_INF("  Sensor[%d]: %u (%.1f%%)", i, weight_map_data.sensor_map[i], weight_map_data.weight_distribution[i]);
    }

    // Send calibration data to data module for storage
    generic_message_t msg;
    msg.sender = SENDER_FOOT_SENSOR_THREAD;
    msg.type = MSG_TYPE_FOOT_WEIGHT_MAP_DATA;
    memcpy(&msg.data.foot_weight_map, &weight_map_data, sizeof(foot_weight_map_data_t));

    if (k_msgq_put(&data_msgq, &msg, K_NO_WAIT) != 0)
    {
        LOG_ERR("Failed to send weight map calibration data to data module");
    }
    else
    {
        LOG_INF("Weight map calibration data sent to data module");
    }

    // Reset calibration state
    weight_map_calibration_active = false;
}

/**
 * @brief Process ADC samples and send to message queues
 */
static void process_adc_samples(int16_t *raw_data)
{
    static uint8_t ble_sample_counter = 0;

    // Handle weight map calibration if active
    if (weight_map_calibration_active)
    {
        process_weight_map_sample(raw_data);
        return; // Don't process normal samples during calibration
    }

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
            // Simulate realistic gait pattern with proper 14-bit ADC values (0-16383)
            // Gait cycle: ~600ms total (300ms stance, 300ms swing) for ~100 spm cadence
            // Using high pressure values to ensure clear event detection
            int64_t current_time = k_uptime_get();

            // Add phase delay for secondary device to create realistic bilateral pattern
            // Secondary (left foot) should be ~180 degrees out of phase with primary (right foot)
            // This creates a natural alternating gait pattern
            int phase_offset = 0;

            // Create realistic differences between left and right feet
            float pressure_multiplier = 1.0f; // For simulating force differences
            float gct_multiplier = 1.0f;      // For simulating GCT differences
            bool is_left_foot = false;

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            // SECONDARY (LEFT FOOT) characteristics:
            is_left_foot = true;
            phase_offset = 375; // Half of 750ms cycle for opposite phase
            // Add small realistic variation (±10ms) for natural gait
            phase_offset += ((current_time / 1000) % 21) - 10; // -10 to +10 ms variation

            // Common asymmetries in runners:
            pressure_multiplier = 0.97f; // 3% less force on left foot (common)
            gct_multiplier = 1.02f;      // 2% longer GCT on left (slight asymmetry)

            // Log periodically for debugging
            static uint32_t phase_log_counter = 0;
            if (++phase_log_counter % 1000 == 0)
            { // Every 10 seconds at 100Hz
                LOG_INF("SECONDARY (Left foot): phase=%dms, force=%.0f%%, GCT=+%.0f%%", phase_offset,
                        pressure_multiplier * 100, (gct_multiplier - 1.0f) * 100);
            }
#else
            // PRIMARY (RIGHT FOOT) characteristics:
            is_left_foot = false;
            pressure_multiplier = 1.03f; // 3% more force on right foot
            gct_multiplier = 0.98f;      // 2% shorter GCT on right

            // Log periodically
            static uint32_t phase_log_counter = 0;
            if (++phase_log_counter % 1000 == 0)
            { // Every 10 seconds at 100Hz
                LOG_INF("PRIMARY (Right foot): phase=0ms, force=%.0f%%, GCT=-%.0f%%", pressure_multiplier * 100,
                        (1.0f - gct_multiplier) * 100);
            }
#endif

            // Adjust cycle time for GCT differences
            int cycle_time = (current_time + phase_offset) % 750; // 750ms gait cycle with phase offset

            // Adjust stance phase duration based on GCT multiplier
            int stance_duration = (int)(375 * gct_multiplier); // Normally 375ms

            // Create realistic pressure pattern with 14-bit ADC range
            // Total pressure during midstance: ~70,000-80,000 (clear detection)
            int16_t pressure_value = 0;

            if (cycle_time < stance_duration)
            {
                // Stance phase - foot is on ground (adjusted duration for GCT differences)
                if (cycle_time < 62)
                {
                    // Initial contact/loading (0-50ms) - heel strike
                    // Sharp ramp up pressure from 0 to 10000
                    pressure_value = (cycle_time * 200); // 0 to 10000 in 50ms
                    pressure_value = (int16_t)(pressure_value * pressure_multiplier);

                    // For LEFT foot, sensors are flipped (lateral/medial swapped)
                    // RIGHT foot: [1]=Lateral, [3]=Medial, [4]=Arch Lat, [5]=Arch Med, [6]=Heel Lat, [7]=Heel Med
                    // LEFT foot:  [1]=Medial, [3]=Lateral, [4]=Arch Med, [5]=Arch Lat, [6]=Heel Med, [7]=Heel Lat

                    if (is_left_foot)
                    {
                        // Left foot - flipped mapping with slight medial bias
                        msg.data.foot_samples.values[0] = 0; // Big Toe
                        msg.data.foot_samples.values[1] = 0; // Forefoot medial (flipped)
                        msg.data.foot_samples.values[2] = 0; // Forefoot center
                        msg.data.foot_samples.values[3] = 0; // Forefoot lateral (flipped)
                        msg.data.foot_samples.values[4] =
                            (pressure_value / 4) * 1.1; // Arch medial (flipped) - more pressure
                        msg.data.foot_samples.values[5] =
                            (pressure_value / 4) * 0.9; // Arch lateral (flipped) - less pressure
                        msg.data.foot_samples.values[6] =
                            pressure_value * 1.05; // Heel medial (flipped) - more pressure
                        msg.data.foot_samples.values[7] =
                            pressure_value * 0.95; // Heel lateral (flipped) - less pressure
                    }
                    else
                    {
                        // Right foot - normal mapping with slight lateral bias
                        msg.data.foot_samples.values[0] = 0;                          // Big Toe
                        msg.data.foot_samples.values[1] = 0;                          // Forefoot lateral
                        msg.data.foot_samples.values[2] = 0;                          // Forefoot center
                        msg.data.foot_samples.values[3] = 0;                          // Forefoot medial
                        msg.data.foot_samples.values[4] = (pressure_value / 4) * 1.1; // Arch lateral - more pressure
                        msg.data.foot_samples.values[5] = (pressure_value / 4) * 0.9; // Arch medial - less pressure
                        msg.data.foot_samples.values[6] = pressure_value * 1.05;      // Heel lateral - more pressure
                        msg.data.foot_samples.values[7] = pressure_value * 0.95;      // Heel medial - less pressure
                    }
                }
                else if (cycle_time < 187)
                {
                    // Midstance (50-150ms) - full foot contact, weight bearing
                    // All sensors have significant pressure - apply multiplier
                    if (is_left_foot)
                    {
                        // Left foot - flipped mapping with medial bias
                        msg.data.foot_samples.values[0] =
                            (int16_t)(9000 * pressure_multiplier * 1.05); // Big Toe - more pressure
                        msg.data.foot_samples.values[1] =
                            (int16_t)(10000 * pressure_multiplier * 1.02); // Forefoot medial (flipped)
                        msg.data.foot_samples.values[2] = (int16_t)(10000 * pressure_multiplier); // Forefoot center
                        msg.data.foot_samples.values[3] =
                            (int16_t)(10000 * pressure_multiplier * 0.98); // Forefoot lateral (flipped)
                        msg.data.foot_samples.values[4] =
                            (int16_t)(11000 * pressure_multiplier * 1.03); // Arch medial (flipped) - max
                        msg.data.foot_samples.values[5] =
                            (int16_t)(11000 * pressure_multiplier * 0.97); // Arch lateral (flipped)
                        msg.data.foot_samples.values[6] =
                            (int16_t)(8000 * pressure_multiplier * 1.02); // Heel medial (flipped)
                        msg.data.foot_samples.values[7] =
                            (int16_t)(8000 * pressure_multiplier * 0.98); // Heel lateral (flipped)
                    }
                    else
                    {
                        // Right foot - normal mapping with lateral bias
                        msg.data.foot_samples.values[0] =
                            (int16_t)(9000 * pressure_multiplier * 0.95); // Big Toe - less pressure
                        msg.data.foot_samples.values[1] =
                            (int16_t)(10000 * pressure_multiplier * 1.02);                        // Forefoot lateral
                        msg.data.foot_samples.values[2] = (int16_t)(10000 * pressure_multiplier); // Forefoot center
                        msg.data.foot_samples.values[3] =
                            (int16_t)(10000 * pressure_multiplier * 0.98); // Forefoot medial
                        msg.data.foot_samples.values[4] =
                            (int16_t)(11000 * pressure_multiplier * 1.03); // Arch lateral - max
                        msg.data.foot_samples.values[5] = (int16_t)(11000 * pressure_multiplier * 0.97); // Arch medial
                        msg.data.foot_samples.values[6] = (int16_t)(8000 * pressure_multiplier * 1.02);  // Heel lateral
                        msg.data.foot_samples.values[7] = (int16_t)(8000 * pressure_multiplier * 0.98);  // Heel medial
                    }
                }
                else if (cycle_time < (stance_duration - 62))
                {
                    // Push-off phase - weight shifts to forefoot/toes
                    // Adjust timing based on stance_duration
                    if (is_left_foot)
                    {
                        // Left foot - stronger medial push-off (common overpronation)
                        msg.data.foot_samples.values[0] =
                            (int16_t)(13000 * pressure_multiplier * 1.08); // Big Toe - max push
                        msg.data.foot_samples.values[1] =
                            (int16_t)(13000 * pressure_multiplier * 1.05); // Forefoot medial (flipped)
                        msg.data.foot_samples.values[2] = (int16_t)(13000 * pressure_multiplier); // Forefoot center
                        msg.data.foot_samples.values[3] =
                            (int16_t)(13000 * pressure_multiplier * 0.95); // Forefoot lateral (flipped)
                        msg.data.foot_samples.values[4] =
                            (int16_t)(4000 * pressure_multiplier * 1.1); // Arch medial (flipped)
                        msg.data.foot_samples.values[5] =
                            (int16_t)(4000 * pressure_multiplier * 0.9); // Arch lateral (flipped)
                        msg.data.foot_samples.values[6] = 0;             // Heel medial - off ground
                        msg.data.foot_samples.values[7] = 0;             // Heel lateral - off ground
                    }
                    else
                    {
                        // Right foot - stronger lateral push-off
                        msg.data.foot_samples.values[0] =
                            (int16_t)(13000 * pressure_multiplier * 0.92); // Big Toe - less push
                        msg.data.foot_samples.values[1] =
                            (int16_t)(13000 * pressure_multiplier * 1.05);                        // Forefoot lateral
                        msg.data.foot_samples.values[2] = (int16_t)(13000 * pressure_multiplier); // Forefoot center
                        msg.data.foot_samples.values[3] =
                            (int16_t)(13000 * pressure_multiplier * 0.95); // Forefoot medial
                        msg.data.foot_samples.values[4] = (int16_t)(4000 * pressure_multiplier * 1.1); // Arch lateral
                        msg.data.foot_samples.values[5] = (int16_t)(4000 * pressure_multiplier * 0.9); // Arch medial
                        msg.data.foot_samples.values[6] = 0; // Heel lateral - off ground
                        msg.data.foot_samples.values[7] = 0; // Heel medial - off ground
                    }
                }
                else
                {
                    // Toe-off transition (250-300ms) - leaving ground
                    // Pressure rapidly decreasing as foot leaves ground
                    // Fixed calculation: linear ramp from max to 0
                    int16_t time_in_phase = cycle_time - 312;            // 0 to 50
                    int16_t fade_factor = 13000 - (time_in_phase * 208); // 13000 to 0 over 62ms
                    if (fade_factor < 0)
                        fade_factor = 0;

                    // Using correct mapping: [0]=Big Toe, [1-3]=Forefoot, [4-5]=Arch, [6-7]=Heel
                    msg.data.foot_samples.values[0] = fade_factor;     // Big Toe - last to leave
                    msg.data.foot_samples.values[1] = fade_factor / 3; // Forefoot lateral
                    msg.data.foot_samples.values[2] = fade_factor / 3; // Forefoot center
                    msg.data.foot_samples.values[3] = fade_factor / 3; // Forefoot medial
                    msg.data.foot_samples.values[4] = 0;               // Arch lateral - off
                    msg.data.foot_samples.values[5] = 0;               // Arch medial - off
                    msg.data.foot_samples.values[6] = 0;               // Heel lateral - off
                    msg.data.foot_samples.values[7] = 0;               // Heel medial - off
                }
            }
            else
            {
                // Swing phase (300-600ms) - foot is in air, no ground contact
                // All pressure sensors read near 0 with minimal noise
                for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
                {
                    // Add small random noise for realism (0-100 range)
                    // Using simple pseudo-random based on time and channel
                    uint16_t noise = ((current_time + i * 17) % 101); // 0-100 noise
                    msg.data.foot_samples.values[i] = noise;
                }
            }

            // Ensure all values are within 14-bit ADC range (0-16383)
            for (uint8_t i = 0; i < SAADC_CHANNEL_COUNT; i++)
            {
                if (msg.data.foot_samples.values[i] < 0)
                {
                    msg.data.foot_samples.values[i] = 0;
                }
                else if (msg.data.foot_samples.values[i] > 16383)
                {
                    msg.data.foot_samples.values[i] = 16383;
                }
            }
#endif

            // Send to sensor_data module at 100Hz only if activity started
            if (atomic_get(&logging_active) == 1)
            {
                if (k_msgq_put(&sensor_data_msgq, &msg, K_NO_WAIT) != 0)
                {
                    LOG_WRN("Failed to send foot sensor data to sensor_data module");
                }
            }

            // Send to Bluetooth at 5Hz ONLY when streaming is enabled AND no activity is running
            if (foot_sensor_streaming_enabled == true && atomic_get(&logging_active) == 0)
            {
#if IS_ENABLED(CONFIG_LAB_VERSION)

                // For lab version, send all raw foot data at full rate (100Hz) to SD card module
                // during activity OR streaming (commands 3/5)
                if ((atomic_get(&logging_active) == 1) || (foot_sensor_streaming_enabled == true))
                {
                    if (k_msgq_put(&data_sd_msgq, &msg, K_NO_WAIT) != 0)
                    {
                        LOG_WRN("Failed to send foot sensor data to data_sd module");
                    }
                }
#endif
                if (++ble_sample_counter >= BLUETOOTH_RATE_DIVIDER)
                {
                    ble_sample_counter = 0;
                    if (k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT) != 0)
                    {
                        LOG_WRN("Failed to send foot sensor data to bluetooth module");
                    }
                }
            }
            else
            {
                // Reset counter when not streaming to ensure immediate response when enabled
                ble_sample_counter = 0;
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
        // Check for weight map calibration command via message queue
        generic_message_t cmd_msg;
        if (k_msgq_get(&bluetooth_msgq, &cmd_msg, K_NO_WAIT) == 0)
        {
            if (cmd_msg.type == MSG_TYPE_START_FOOT_WEIGHT_MAP)
            {
                LOG_INF("Received weight map calibration command");
                start_weight_map_calibration();
            }
        }

        // Check if it's time to monitor VDD (every 1 seconds)
    /**    uint32_t current_time = k_uptime_get();
        if (current_time - last_vdd_check_ms >= VDD_MONITOR_INTERVAL_MS) {
            last_vdd_check_ms = current_time;
            
            // Use channel 0 for VDD monitoring
            err_t vdd_err = read_vdd_voltage(0);
            if (vdd_err == err_t::NO_ERROR) {
                LOG_WRN("VDD monitoring successful: %d mV", vdd_voltage_mv);
            }
        }*/
        

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

        // Sleep for sampling period (10ms for 100Hz)
        k_msleep(SAMPLE_PERIOD_MS);
    }
}

/**
 * @brief Configure a channel for VDD monitoring
 */
static err_t configure_channel_for_vdd(uint8_t channel_num)
{
    struct adc_channel_cfg vdd_cfg = {
        .gain = ADC_GAIN_1_6,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10),
        .channel_id = channel_num,
        .input_positive = SAADC_CH_PSELP_PSELP_VDD,
        .input_negative = SAADC_CH_PSELN_PSELN_NC // Not connected
    };

    int err = adc_channel_setup(adc_dev, &vdd_cfg);
    if (err < 0) {
        LOG_ERR("Failed to setup ADC channel %d for VDD: %d", channel_num, err);
        return err_t::ADC_ERROR;
    }

    return err_t::NO_ERROR;
}

/**
 * @brief Restore a channel to its normal pressure sensor configuration
 */
static err_t restore_channel_to_pressure(uint8_t channel_num)
{
    int err = adc_channel_setup(adc_dev, &channel_configs[channel_num]);
    if (err < 0) {
        LOG_ERR("Failed to restore ADC channel %d to pressure sensor: %d", channel_num, err);
        return err_t::ADC_ERROR;
    }

    return err_t::NO_ERROR;
}

/**
 * @brief Read and calculate VDD voltage
 */
static err_t read_vdd_voltage(uint8_t temp_channel)
{
    int16_t vdd_sample;
    struct adc_sequence vdd_sequence = {
        .channels = BIT(temp_channel),
        .buffer = &vdd_sample,
        .buffer_size = sizeof(vdd_sample),
        .resolution = 12, 
        .oversampling = 4,
        .calibrate = true,
    };

    // 1. Configure the temporary channel for VDD
    err_t err = configure_channel_for_vdd(temp_channel);
    if (err != err_t::NO_ERROR) {
        return err;
    }

    // 2. Read the VDD sample
    int ret = adc_read(adc_dev, &vdd_sequence);
    if (ret < 0) {
        LOG_ERR("VDD ADC read failed: %d", ret);
        restore_channel_to_pressure(temp_channel);
        return err_t::ADC_ERROR;
    }

    // 3. Calculate VDD
    const int32_t ref_voltage_mv = 600; // Internal reference is 0.6V
    const int32_t max_adc_value = (1 << vdd_sequence.resolution) - 1; // 4095 for 12-bit
    
   // LOG_WRN("Raw VDD sample: %d, Max ADC value: %d", vdd_sample, max_adc_value);


     int32_t adc_voltage = (vdd_sample * ref_voltage_mv) / max_adc_value;
    vdd_voltage_mv = adc_voltage * 6*1.12;

  //  LOG_WRN("VDD voltage: %d mV", vdd_voltage_mv);

    // 4. Restore the channel to pressure sensing
    err = restore_channel_to_pressure(temp_channel);
    if (err != err_t::NO_ERROR) {
        return err;
    }

    // Check for low voltage
    if (vdd_voltage_mv < VDD_LOW_THRESHOLD_MV) {
        LOG_WRN("LOW VOLTAGE WARNING: %d mV (threshold: %d mV)", 
                vdd_voltage_mv, VDD_LOW_THRESHOLD_MV);
    }

    return err_t::NO_ERROR;
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

   // k_thread_suspend(foot_sensor_tid);
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
        k_thread_resume(foot_sensor_tid);
        atomic_set(&logging_active, 1);
        foot_sensor_streaming_enabled = false;
        return false;
    }

    if (is_foot_sensor_stop_activity_event(aeh))
    {
        LOG_INF("Received stop activity event - disabling foot sensor sampling");
        atomic_set(&logging_active, 0);
        k_thread_suspend(foot_sensor_tid);
        return false;
    }

    if (is_streaming_control_event(aeh))
    {
        auto *event = cast_streaming_control_event(aeh);
        foot_sensor_streaming_enabled = event->foot_sensor_streaming_enabled;
        if (foot_sensor_streaming_enabled)
        {
            k_thread_resume(foot_sensor_tid);
        }
        else
        {

            k_thread_suspend(foot_sensor_tid);
        }
        LOG_INF("foot_samples_work %s", foot_sensor_streaming_enabled ? "enabled" : "disabled");

// If in lab version, we have to stop the raw file logging in the sd card
#if IS_ENABLED(CONFIG_LAB_VERSION)
        if (foot_sensor_streaming_enabled == false)
        {
            generic_message_t msg_dt;
            msg_dt.sender = SENDER_FOOT_SENSOR_THREAD;
            msg_dt.type = MSG_TYPE_COMMAND;
            strcpy(msg_dt.data.command_str, "STOP_RAW_LOGGING");
            if (k_msgq_put(&data_sd_msgq, &msg_dt, K_NO_WAIT) != 0)
            {
                LOG_WRN("Failed to send foot sensor data to data_sd module");
            }
        }
#endif
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
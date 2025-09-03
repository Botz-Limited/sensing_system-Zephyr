/**
 * @file secondary_metrics.cpp
 * @brief Secondary device metrics calculation module
 * 
 * This module calculates metrics from the local ring buffer on the secondary device
 * and sends them to the primary device via D2D every second.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

// Define M_PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <app.hpp>
#include <d2d_metrics.h>
#include <choros_buffer.hpp>
#include "../bluetooth/ble_d2d_tx.hpp"
#include "secondary_metrics.h"

LOG_MODULE_REGISTER(secondary_metrics, LOG_LEVEL_WRN);

// External Choros buffer instance (defined in analytics.cpp)
extern choros_ring_buffer_t choros_buffer;

// Timer for periodic metric calculation (1 Hz)
static struct k_timer metrics_timer;

// Metrics state
static uint16_t sequence_num = 0;
static bool buffer_ready = false;
static uint32_t warmup_start_ms = 0;

// Metrics packet to send
static d2d_metrics_packet_t metrics_packet;

/**
 * @brief Calculate ground contact time from buffer
 * @return Ground contact time in milliseconds, or 0 if not calculable
 */
static float calculate_gct(void)
{
    if (!choros_buffer_is_healthy() || choros_buffer.count < 100) {
        return 0.0f;
    }

    // Simplified GCT calculation using pressure threshold
    float gct_sum = 0;
    int gct_count = 0;
    bool in_contact = false;
    uint32_t contact_start = 0;
    
    // Threshold for ground contact (sum of all pressure sensors)
    const float CONTACT_THRESHOLD = 500.0f;
    
    for (int i = 0; i < choros_buffer.count - 1; i++) {
        choros_imu_data_t* sample = &choros_buffer.samples[i];
        
        // Calculate total pressure
        float total_pressure = 0;
        for (int j = 0; j < 8; j++) {
            total_pressure += sample->insole_pressures[j];
        }
        
        if (!in_contact && total_pressure > CONTACT_THRESHOLD) {
            // Ground contact started
            in_contact = true;
            contact_start = sample->timestamp * 1000; // Convert to ms
        } else if (in_contact && total_pressure < CONTACT_THRESHOLD) {
            // Ground contact ended
            in_contact = false;
            uint32_t contact_end = sample->timestamp * 1000;
            float gct = contact_end - contact_start;
            if (gct > 50 && gct < 500) { // Sanity check: 50-500ms
                gct_sum += gct;
                gct_count++;
            }
        }
    }
    
    return (gct_count > 0) ? (gct_sum / gct_count) : 0.0f;
}

/**
 * @brief Calculate stride length from buffer
 * @return Stride length in centimeters, or 0 if not calculable
 */
static float calculate_stride_length(void)
{
    if (!choros_buffer_is_healthy() || choros_buffer.count < 200) {
        return 0.0f;
    }

    // Simplified stride length estimation
    // Based on velocity integration between steps
    float stride_sum = 0;
    int stride_count = 0;
    
    // Detect steps and integrate velocity
    for (int i = 1; i < choros_buffer.count - 1; i++) {
        choros_imu_data_t* prev = &choros_buffer.samples[i-1];
        choros_imu_data_t* curr = &choros_buffer.samples[i];
        choros_imu_data_t* next = &choros_buffer.samples[i+1];
        
        // Simple peak detection on vertical acceleration
        if (curr->acc_z > prev->acc_z && curr->acc_z > next->acc_z && curr->acc_z > 1.2f) {
            // Found a step
            stride_count++;
        }
    }
    
    // Estimate stride length based on step frequency and user height
    // Default assumption: 165cm height
    float step_frequency = (float)stride_count / (choros_buffer.count * 0.01f); // Hz
    if (step_frequency > 0) {
        // Empirical formula: stride_length = height * 0.43 * (step_freq / 2)
        stride_sum = 165.0f * 0.43f * (step_frequency / 2.0f);
    }
    
    return stride_sum;
}

/**
 * @brief Calculate cadence from buffer
 * @return Cadence in steps per minute, or 0 if not calculable
 */
static float calculate_cadence(void)
{
    if (!choros_buffer_is_healthy() || choros_buffer.count < 100) {
        return 0.0f;
    }

    int step_count = 0;
    
    // Count steps using simple peak detection
    for (int i = 1; i < choros_buffer.count - 1; i++) {
        choros_imu_data_t* prev = &choros_buffer.samples[i-1];
        choros_imu_data_t* curr = &choros_buffer.samples[i];
        choros_imu_data_t* next = &choros_buffer.samples[i+1];
        
        // Detect peaks in vertical acceleration
        if (curr->acc_z > prev->acc_z && curr->acc_z > next->acc_z && curr->acc_z > 1.2f) {
            step_count++;
        }
    }
    
    // Calculate time window
    float time_window_s = choros_buffer.count * 0.01f; // 100Hz sampling
    float cadence = (step_count / time_window_s) * 60.0f; // Convert to steps/min
    
    return cadence;
}

/**
 * @brief Calculate pronation angle from buffer
 * @return Pronation angle in degrees, or 0 if not calculable
 */
static float calculate_pronation(void)
{
    if (!choros_buffer_is_healthy() || choros_buffer.count < 50) {
        return 0.0f;
    }

    float pronation_sum = 0;
    int pronation_count = 0;
    
    // Calculate pronation from gyroscope data at initial contact
    for (int i = 0; i < choros_buffer.count; i++) {
        choros_imu_data_t* sample = &choros_buffer.samples[i];
        
        // Check if this is around initial contact (high vertical acceleration)
        if (sample->acc_z > 1.5f) {
            // Integrate gyroscope around Y-axis for pronation
            // Convert from radians to degrees: 180/PI = 57.2957795131
            float pronation_angle = sample->gyro_y * 0.01f * 57.2957795131f;
            pronation_sum += fabsf(pronation_angle);
            pronation_count++;
        }
    }
    
    return (pronation_count > 0) ? (pronation_sum / pronation_count) : 0.0f;
}

/**
 * @brief Calculate center of pressure from buffer
 * @param cop_x Output: COP X position in mm
 * @param cop_y Output: COP Y position in mm
 */
static void calculate_cop(float *cop_x, float *cop_y)
{
    *cop_x = 0;
    *cop_y = 0;
    
    if (!choros_buffer_is_healthy() || choros_buffer.count < 10) {
        return;
    }

    float sum_x = 0, sum_y = 0, sum_pressure = 0;
    int sample_count = 0;
    
    // Sensor positions (approximate, in mm from heel center)
    const float sensor_x[8] = {-20, 20, -20, 20, -15, 15, -10, 10};
    const float sensor_y[8] = {0, 0, 40, 40, 80, 80, 120, 120};
    
    // Average COP over recent samples
    int start_idx = (choros_buffer.count > 10) ? choros_buffer.count - 10 : 0;
    
    for (int i = start_idx; i < choros_buffer.count; i++) {
        choros_imu_data_t* sample = &choros_buffer.samples[i];
        
        float frame_sum_x = 0, frame_sum_y = 0, frame_sum_p = 0;
        
        for (int j = 0; j < 8; j++) {
            float pressure = sample->insole_pressures[j];
            frame_sum_x += sensor_x[j] * pressure;
            frame_sum_y += sensor_y[j] * pressure;
            frame_sum_p += pressure;
        }
        
        if (frame_sum_p > 100) { // Minimum pressure threshold
            sum_x += frame_sum_x / frame_sum_p;
            sum_y += frame_sum_y / frame_sum_p;
            sum_pressure += frame_sum_p;
            sample_count++;
        }
    }
    
    if (sample_count > 0) {
        *cop_x = sum_x / sample_count;
        *cop_y = sum_y / sample_count;
    }
}

/**
 * @brief Calculate peak pressure from buffer
 * @return Peak pressure in kPa, or 0 if not calculable
 */
static float calculate_peak_pressure(void)
{
    if (!choros_buffer_is_healthy() || choros_buffer.count < 10) {
        return 0.0f;
    }

    float max_pressure = 0;
    
    for (int i = 0; i < choros_buffer.count; i++) {
        choros_imu_data_t* sample = &choros_buffer.samples[i];
        
        for (int j = 0; j < 8; j++) {
            if (sample->insole_pressures[j] > max_pressure) {
                max_pressure = sample->insole_pressures[j];
            }
        }
    }
    
    // Convert ADC units to kPa (approximate)
    // Assuming 0-4095 ADC = 0-200 kPa
    return (max_pressure / 4095.0f) * 200.0f;
}

/**
 * @brief Timer callback - Calculate and send metrics
 */
static void metrics_timer_callback(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Only run on secondary device
    
    // Check if buffer is ready (3-second warmup)
    if (!buffer_ready) {
        if (warmup_start_ms == 0) {
            warmup_start_ms = k_uptime_get_32();
        }
        
        if (k_uptime_get_32() - warmup_start_ms < 3000) {
            LOG_INF("Metrics warmup: %u ms remaining", 
                    3000 - (k_uptime_get_32() - warmup_start_ms));
            return;
        }
        
        buffer_ready = true;
        LOG_INF("Metrics buffer ready, starting calculations");
    }
    
    // Clear packet
    d2d_metrics_clear(&metrics_packet);
    
    // Set metadata
    metrics_packet.timestamp = k_uptime_get_32();
    metrics_packet.sequence_num = sequence_num++;
    
    // Check buffer health
    if (!choros_buffer_is_healthy()) {
        metrics_packet.calculation_status = 2; // Error
        LOG_WRN("Buffer unhealthy, sending error status");
    } else if (choros_buffer.count < 100) {
        metrics_packet.calculation_status = 1; // Warm-up
        LOG_INF("Buffer warming up: %d samples", choros_buffer.count);
    } else {
        metrics_packet.calculation_status = 0; // OK
        
        // Calculate and store metrics
        float value;
        
        // Ground contact time
        value = calculate_gct();
        if (value > 0) {
            metrics_packet.metrics[IDX_GCT] = value;
            d2d_metric_set_valid(&metrics_packet, IDX_GCT);
        }
        
        // Stride length
        value = calculate_stride_length();
        if (value > 0) {
            metrics_packet.metrics[IDX_STRIDE_LENGTH] = value;
            d2d_metric_set_valid(&metrics_packet, IDX_STRIDE_LENGTH);
        }
        
        // Cadence
        value = calculate_cadence();
        if (value > 0) {
            metrics_packet.metrics[IDX_CADENCE] = value;
            d2d_metric_set_valid(&metrics_packet, IDX_CADENCE);
        }
        
        // Pronation
        value = calculate_pronation();
        if (value > 0) {
            metrics_packet.metrics[IDX_PRONATION] = value;
            d2d_metric_set_valid(&metrics_packet, IDX_PRONATION);
        }
        
        // Center of pressure
        float cop_x, cop_y;
        calculate_cop(&cop_x, &cop_y);
        if (cop_x != 0 || cop_y != 0) {
            metrics_packet.metrics[IDX_COP_X] = cop_x;
            metrics_packet.metrics[IDX_COP_Y] = cop_y;
            d2d_metric_set_valid(&metrics_packet, IDX_COP_X);
            d2d_metric_set_valid(&metrics_packet, IDX_COP_Y);
        }
        
        // Peak pressure
        value = calculate_peak_pressure();
        if (value > 0) {
            metrics_packet.metrics[IDX_PEAK_PRESSURE] = value;
            d2d_metric_set_valid(&metrics_packet, IDX_PEAK_PRESSURE);
        }
        
        // Comprehensive logging of ALL calculated metrics every second
        LOG_INF("=== SECONDARY DEVICE METRICS (Left Foot) ===");
        LOG_INF("Timestamp: %u ms, Sequence: %u", metrics_packet.timestamp, metrics_packet.sequence_num);
        
        if (d2d_metric_is_valid(&metrics_packet, IDX_GCT)) {
            LOG_INF("  Ground Contact Time (GCT): %.1f ms", (double)metrics_packet.metrics[IDX_GCT]);
        }
        
        if (d2d_metric_is_valid(&metrics_packet, IDX_STRIDE_LENGTH)) {
            LOG_INF("  Stride Length: %.2f cm (%.2f m)",
                    (double)metrics_packet.metrics[IDX_STRIDE_LENGTH],
                    (double)(metrics_packet.metrics[IDX_STRIDE_LENGTH] / 100.0f));
        }
        
        if (d2d_metric_is_valid(&metrics_packet, IDX_CADENCE)) {
            LOG_INF("  Cadence: %.1f steps/min", (double)metrics_packet.metrics[IDX_CADENCE]);
        }
        
        if (d2d_metric_is_valid(&metrics_packet, IDX_PRONATION)) {
            LOG_INF("  Pronation Angle: %.1f degrees", (double)metrics_packet.metrics[IDX_PRONATION]);
        }
        
        if (d2d_metric_is_valid(&metrics_packet, IDX_COP_X) && d2d_metric_is_valid(&metrics_packet, IDX_COP_Y)) {
            LOG_INF("  Center of Pressure (COP): X=%.1f mm, Y=%.1f mm",
                    (double)metrics_packet.metrics[IDX_COP_X],
                    (double)metrics_packet.metrics[IDX_COP_Y]);
        }
        
        if (d2d_metric_is_valid(&metrics_packet, IDX_PEAK_PRESSURE)) {
            LOG_INF("  Peak Pressure: %.1f kPa", (double)metrics_packet.metrics[IDX_PEAK_PRESSURE]);
        }
        
        // Additional metrics if available (only log defined indices from d2d_metrics.h)
        if (d2d_metric_is_valid(&metrics_packet, IDX_STANCE_TIME)) {
            LOG_INF("  Stance Time: %.1f ms", (double)metrics_packet.metrics[IDX_STANCE_TIME]);
        }
        
        if (d2d_metric_is_valid(&metrics_packet, IDX_SWING_TIME)) {
            LOG_INF("  Swing Time: %.1f ms", (double)metrics_packet.metrics[IDX_SWING_TIME]);
        }
        
        if (d2d_metric_is_valid(&metrics_packet, IDX_STEP_TIME)) {
            LOG_INF("  Step Time: %.1f ms", (double)metrics_packet.metrics[IDX_STEP_TIME]);
        }
        
        if (d2d_metric_is_valid(&metrics_packet, IDX_STEP_LENGTH)) {
            LOG_INF("  Step Length: %.1f cm", (double)metrics_packet.metrics[IDX_STEP_LENGTH]);
        }
        
        if (d2d_metric_is_valid(&metrics_packet, IDX_STEP_COUNT)) {
            LOG_INF("  Step Count: %u steps", (uint32_t)metrics_packet.metrics[IDX_STEP_COUNT]);
        }
        
        // Buffer statistics
        LOG_INF("  Buffer Status: %d samples, %.1f Hz",
                choros_buffer.count, (double)choros_buffer.last_sample_rate);
        LOG_INF("=============================================");
    }
    
    // Send metrics via D2D
    int err = ble_d2d_tx_send_metrics(&metrics_packet);
    if (err) {
        LOG_ERR("Failed to send metrics via D2D: %d", err);
    } else {
        LOG_DBG("Metrics sent successfully - seq %u", metrics_packet.sequence_num);
    }
#endif
}

/**
 * @brief Initialize secondary metrics calculation
 */
void secondary_metrics_init(void)
{
#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    LOG_INF("Initializing secondary metrics calculation (1Hz)");
    
    // Initialize timer for 1Hz metric calculation
    k_timer_init(&metrics_timer, metrics_timer_callback, NULL);
    k_timer_start(&metrics_timer, K_SECONDS(3), K_SECONDS(1)); // 3s initial delay, then 1Hz
    
    LOG_INF("Secondary metrics timer started");
#else
    LOG_INF("Secondary metrics disabled on primary device");
#endif
}
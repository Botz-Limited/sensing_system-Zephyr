/**
 * @file sensor_data_events.cpp
 * @brief Event-driven gait parameter detection stub implementations
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <app.hpp>
#include <gait_events.h>
#include <d2d_metrics.h>

LOG_MODULE_REGISTER(sensor_data_events, LOG_LEVEL_DBG);

// Static gait event detector for this module - make sure it's properly aligned
static gait_event_detector_t event_detector __aligned(8);
static atomic_t events_initialized = ATOMIC_INIT(0);  // Use atomic instead of volatile bool
static K_MUTEX_DEFINE(events_mutex);

extern "C" {

/**
 * @brief Initialize the event-driven gait parameter system
 */
void sensor_data_events_init(void)
{
    // Check if already initialized using atomic operation
    if (atomic_cas(&events_initialized, 0, 1)) {
        // We successfully changed from 0 to 1, so we're the first to initialize
        LOG_INF("Initializing sensor data events module");
        
        // Clear the detector structure first
        memset(&event_detector, 0, sizeof(event_detector));
        
        // Initialize the gait event detector
        bool is_primary = IS_ENABLED(CONFIG_PRIMARY_DEVICE);
        // Match actual sensor sampling rate: foot sensor at 100Hz
        float sampling_rate = 100.0f;  // 100Hz foot sensor sampling rate
        
        // Initialize with correct sampling rate
        gait_events_init(&event_detector, is_primary, sampling_rate);
        
        // Enable advanced features with optimized processing
        event_detector.use_advanced_features = true;
        LOG_INF("Advanced gait features ENABLED with optimized processing");
        
        // Set subject height if available (default to 1.75m)
        gait_events_set_subject_height(&event_detector, 1.75f);
        
        LOG_INF("Sensor data events module initialized successfully");
    } else {
        LOG_DBG("Sensor data events module already initialized");
    }
}

/**
 * @brief Add sensor data to the event buffer for processing
 * @param foot_data Foot pressure sensor data
 * @param imu_data IMU sensor data
 * @param timestamp Timestamp in seconds
 */
void sensor_data_add_to_event_buffer(const foot_samples_t *foot_data,
                                     const imu_data_t *imu_data,
                                     float timestamp)
{
    // Check initialization using atomic operation
    if (atomic_get(&events_initialized) == 0) {
        // Try to initialize if not already done
        sensor_data_events_init();
        // Check again after initialization attempt
        if (atomic_get(&events_initialized) == 0) {
            return;  // Still not initialized, skip
        }
    }
    
    if (!foot_data || !imu_data) {
        return;  // Skip without logging to avoid spam
    }
    
    // Add data to the gait event detector's ring buffer
    // Processing will be done in the 1Hz timer callback to avoid blocking
    gait_events_add_data(&event_detector, foot_data, imu_data, timestamp);
}

/**
 * @brief 1Hz timer callback for periodic metric transmission
 */
void sensor_data_1hz_timer_callback(void)
{
    static uint32_t callback_count = 0;
    static uint32_t buffer_stuck_count = 0;
    static int last_buffer_count = 0;
    
    // Check initialization using atomic operation
    if (atomic_get(&events_initialized) == 0) {
        return;
    }
    
    // Try to lock mutex with timeout
    if (k_mutex_lock(&events_mutex, K_MSEC(10)) != 0) {
        // Failed to lock, skip this cycle
        return;
    }
    
    callback_count++;
    
    // Monitor buffer health every 10 callbacks (20 seconds at 0.5Hz)
    if (callback_count % 10 == 0) {
        LOG_DBG("Gait buffer health check #%u: count=%d, full=%d, waiting=%d",
                callback_count,
                event_detector.buffer_count,
                event_detector.buffer_full ? 1 : 0,
                event_detector.waiting_for_data ? 1 : 0);
        
        // Check if buffer is stuck (same count for too long)
        if (event_detector.buffer_count == last_buffer_count && event_detector.buffer_count > 0) {
            buffer_stuck_count++;
            if (buffer_stuck_count >= 5) {  // Stuck for 10+ seconds
                LOG_WRN("Buffer appears stuck at %d samples for %u cycles, forcing clear",
                        event_detector.buffer_count, buffer_stuck_count);
                // Force process half the buffer
                int samples_to_clear = event_detector.buffer_count / 2;
                event_detector.read_index = (event_detector.read_index + samples_to_clear) % GAIT_BUFFER_SIZE_SAMPLES;
                event_detector.buffer_count -= samples_to_clear;
                event_detector.buffer_full = false;
                buffer_stuck_count = 0;
            }
        } else {
            buffer_stuck_count = 0;
        }
        last_buffer_count = event_detector.buffer_count;
    }
    
    // Emergency buffer management - prevent overflow
    if (event_detector.buffer_full || event_detector.buffer_count > (GAIT_BUFFER_SIZE_SAMPLES * 3 / 4)) {
        LOG_WRN("Buffer nearing capacity (%d/%d) after safety check, clearing oldest 25%%",
                event_detector.buffer_count, GAIT_BUFFER_SIZE_SAMPLES);
        int samples_to_clear = GAIT_BUFFER_SIZE_SAMPLES / 4;
        event_detector.read_index = (event_detector.read_index + samples_to_clear) % GAIT_BUFFER_SIZE_SAMPLES;
        event_detector.buffer_count = (event_detector.buffer_count > samples_to_clear) ?
                                      event_detector.buffer_count - samples_to_clear : 0;
        event_detector.buffer_full = false;
    }
    
    // Process any pending data
    int metrics_count = gait_events_process(&event_detector);
    
    if (metrics_count > 0) {
        LOG_DBG("1Hz timer: processed %d metrics", metrics_count);
        
        // Get metrics and prepare for transmission
        gait_metrics_t metrics[GAIT_MAX_EVENTS_PER_CHUNK];
        int retrieved = gait_events_get_metrics(&event_detector, metrics, GAIT_MAX_EVENTS_PER_CHUNK);
        
        if (retrieved > 0) {
            // Create D2D metrics packet - initialize all fields
            d2d_metrics_packet_t d2d_packet;
            memset(&d2d_packet, 0, sizeof(d2d_packet));
            d2d_packet.sequence_num = k_uptime_get_32() & 0xFFFF;
            d2d_packet.timestamp = k_uptime_get_32();
            d2d_packet.calculation_status = 0;  // OK
            
            // Fill in metrics from the most recent gait metric
            const gait_metrics_t *latest = &metrics[retrieved - 1];
            
            // Ground contact time
            d2d_packet.metrics[IDX_GCT] = latest->gct * 1000.0f;  // Convert to ms
            d2d_metric_set_valid(&d2d_packet, IDX_GCT);
            
            // Cadence
            d2d_packet.metrics[IDX_CADENCE] = latest->cadence;
            d2d_metric_set_valid(&d2d_packet, IDX_CADENCE);
            
            // Stride length
            d2d_packet.metrics[IDX_STRIDE_LENGTH] = latest->stride_length * 100.0f;  // Convert to cm
            d2d_metric_set_valid(&d2d_packet, IDX_STRIDE_LENGTH);
            
            // Vertical oscillation
            d2d_packet.metrics[IDX_VERTICAL_OSC] = latest->vertical_oscillation * 100.0f;  // Convert to cm
            d2d_metric_set_valid(&d2d_packet, IDX_VERTICAL_OSC);
            
            // Step count
            d2d_packet.metrics[IDX_STEP_COUNT] = (float)latest->step_count;
            d2d_metric_set_valid(&d2d_packet, IDX_STEP_COUNT);
            
            // Pronation (use max pronation during stance)
            d2d_packet.metrics[IDX_PRONATION] = (float)latest->max_pronation;
            d2d_metric_set_valid(&d2d_packet, IDX_PRONATION);
            
            // Peak pressure
            d2d_packet.metrics[IDX_PEAK_PRESSURE] = (float)latest->peak_pressure;
            d2d_metric_set_valid(&d2d_packet, IDX_PEAK_PRESSURE);
            
            // Center of pressure at toe-off (use most recent COP)
            d2d_packet.metrics[IDX_COP_X] = (float)latest->cop_x_at_to;
            d2d_metric_set_valid(&d2d_packet, IDX_COP_X);
            
            d2d_packet.metrics[IDX_COP_Y] = (float)latest->cop_y_at_to;
            d2d_metric_set_valid(&d2d_packet, IDX_COP_Y);
            
            // Foot strike angle (derived from strike pattern)
            float strike_angle = 0.0f;
            switch (latest->strike_pattern) {
                case 1: strike_angle = -15.0f; break;  // Heel strike
                case 2: strike_angle = 0.0f; break;    // Midfoot
                case 3: strike_angle = 15.0f; break;   // Forefoot
                default: strike_angle = 0.0f; break;
            }
            d2d_packet.metrics[IDX_FOOT_STRIKE_ANGLE] = strike_angle;
            d2d_metric_set_valid(&d2d_packet, IDX_FOOT_STRIKE_ANGLE);
            
            // Loading rate
            d2d_packet.metrics[IDX_LOADING_RATE] = (float)latest->loading_rate;
            d2d_metric_set_valid(&d2d_packet, IDX_LOADING_RATE);
            
            // Balance metrics (derived from COP path)
            d2d_packet.metrics[IDX_BALANCE_INDEX] = 100.0f - MIN(latest->cop_path_length, 100.0f);  // Simple balance index
            d2d_metric_set_valid(&d2d_packet, IDX_BALANCE_INDEX);
            
            // Stance/swing times (derived from GCT and duration)
            d2d_packet.metrics[IDX_STANCE_TIME] = latest->gct * 1000.0f;  // Same as GCT in ms
            d2d_metric_set_valid(&d2d_packet, IDX_STANCE_TIME);
            
            float swing_time = (latest->duration - latest->gct) * 1000.0f;  // ms
            d2d_packet.metrics[IDX_SWING_TIME] = swing_time;
            d2d_metric_set_valid(&d2d_packet, IDX_SWING_TIME);
            
            // Step length (half of stride length)
            d2d_packet.metrics[IDX_STEP_LENGTH] = latest->stride_length * 50.0f;  // Convert to cm and divide by 2
            d2d_metric_set_valid(&d2d_packet, IDX_STEP_LENGTH);
            
            // Fatigue indicators (simple heuristics for now)
            // Could be enhanced with more sophisticated algorithms
            d2d_packet.metrics[IDX_FATIGUE_INDEX] = (float)latest->arch_collapse_index;  // Use arch collapse as fatigue indicator
            d2d_metric_set_valid(&d2d_packet, IDX_FATIGUE_INDEX);
            
            // Power metrics
            d2d_packet.metrics[IDX_POWER] = (float)latest->push_off_power;
            d2d_metric_set_valid(&d2d_packet, IDX_POWER);
            
            // Count valid metrics
            int valid_count = 0;
            for (int i = 0; i < D2D_METRICS_COUNT; i++) {
                if (d2d_metric_is_valid(&d2d_packet, (d2d_metric_index)i)) {
                    valid_count++;
                }
            }
            
            // TODO: Send d2d_packet via D2D interface
            LOG_DBG("Prepared D2D packet with %d valid metrics", valid_count);
        }
        
        // Clear processed metrics
        gait_events_clear_metrics(&event_detector);
    }
    
    // Memory health check every 60 callbacks (2 minutes at 0.5Hz)
    if (callback_count % 60 == 0) {
        LOG_INF("System health after %u cycles: buffer=%d/%d, metrics=%d",
                callback_count, event_detector.buffer_count,
                GAIT_BUFFER_SIZE_SAMPLES, metrics_count);
        
        // Force garbage collection if needed
        if (event_detector.buffer_count > GAIT_BUFFER_SIZE_SAMPLES / 2) {
            LOG_WRN("Performing preventive buffer maintenance");
            // Clear oldest 10% to keep things flowing
            int samples_to_clear = GAIT_BUFFER_SIZE_SAMPLES / 10;
            event_detector.read_index = (event_detector.read_index + samples_to_clear) % GAIT_BUFFER_SIZE_SAMPLES;
            event_detector.buffer_count = (event_detector.buffer_count > samples_to_clear) ?
                                         event_detector.buffer_count - samples_to_clear : 0;
        }
    }
    
    k_mutex_unlock(&events_mutex);
}

/**
 * @brief Process received D2D metrics from secondary device
 * @param metrics Received D2D metrics packet
 */
void sensor_data_process_received_metrics(const d2d_metrics_packet_t *metrics)
{
    if (!metrics) {
        LOG_ERR("Invalid metrics pointer");
        return;
    }
    
    LOG_DBG("Processing received D2D metrics: seq=%u, timestamp=%u",
            metrics->sequence_num, metrics->timestamp);
    
    // Log received metrics
    if (d2d_metric_is_valid(metrics, IDX_GCT)) {
        LOG_DBG("  Received GCT: %.1f ms", (double)metrics->metrics[IDX_GCT]);
    }
    if (d2d_metric_is_valid(metrics, IDX_CADENCE)) {
        LOG_DBG("  Received Cadence: %.1f spm", (double)metrics->metrics[IDX_CADENCE]);
    }
    if (d2d_metric_is_valid(metrics, IDX_STRIDE_LENGTH)) {
        LOG_DBG("  Received Stride: %.2f m", (double)metrics->metrics[IDX_STRIDE_LENGTH]);
    }
    if (d2d_metric_is_valid(metrics, IDX_VERTICAL_OSC)) {
        LOG_DBG("  Received VO: %.1f cm", (double)metrics->metrics[IDX_VERTICAL_OSC]);
    }
    if (d2d_metric_is_valid(metrics, IDX_STEP_COUNT)) {
        LOG_DBG("  Received Steps: %u", (uint32_t)metrics->metrics[IDX_STEP_COUNT]);
    }
    
    // TODO: Store or process the received metrics for bilateral analysis
    // For now, just log that we received them
    
    #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // On primary device, we could combine these with local metrics
    // for bilateral gait analysis
    LOG_DBG("Primary device: received secondary metrics for bilateral analysis");
    #endif
}

} // extern "C"
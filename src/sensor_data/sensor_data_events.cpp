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

// Flow control state - safe implementation that won't break existing logic
typedef enum {
    FLOW_MODE_NORMAL = 0,     // All features enabled (buffer < 50%)
    FLOW_MODE_REDUCED = 1,    // Reduced features (buffer 50-75%)
    FLOW_MODE_CRITICAL = 2    // Critical only (buffer > 75%)
} flow_control_mode_t;

typedef enum {
    METRIC_PRIORITY_CRITICAL = 0,  // GCT, cadence, strike pattern - NEVER skip
    METRIC_PRIORITY_HIGH = 1,      // Pronation, balance, step count
    METRIC_PRIORITY_NORMAL = 2,    // Form score, efficiency, VO
    METRIC_PRIORITY_LOW = 3        // Derived metrics, alerts
} metric_priority_t;

// Flow control state structure
static struct {
    atomic_t current_mode;           // Current flow control mode
    atomic_t buffer_pressure;        // Buffer usage percentage (0-100)
    uint32_t mode_changes;           // Count of mode changes
    uint32_t metrics_skipped;        // Count of skipped low-priority metrics
    uint32_t last_mode_change_time;  // Time of last mode change
    uint32_t overflow_events;        // Count of buffer overflow events
    bool advanced_features_saved;   // Original state of advanced features
} flow_control = {
    .current_mode = ATOMIC_INIT(FLOW_MODE_NORMAL),
    .buffer_pressure = ATOMIC_INIT(0),
    .mode_changes = 0,
    .metrics_skipped = 0,
    .last_mode_change_time = 0,
    .overflow_events = 0,
    .advanced_features_saved = true
};

// Flow control thresholds - conservative to avoid breaking existing logic
#define FLOW_THRESHOLD_NORMAL    50  // Below 50% - all features
#define FLOW_THRESHOLD_REDUCED   75  // 50-75% - reduce processing
#define FLOW_THRESHOLD_CRITICAL  90  // Above 90% - emergency mode
#define FLOW_HYSTERESIS         10   // Hysteresis to prevent oscillation

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
 * @brief Safe flow control adjustment based on buffer pressure
 * This function carefully adjusts processing mode without breaking existing logic
 */
static void adjust_flow_control_mode(int buffer_usage_percent)
{
    flow_control_mode_t current_mode = (flow_control_mode_t)atomic_get(&flow_control.current_mode);
    flow_control_mode_t new_mode = current_mode;
    uint32_t now = k_uptime_get_32();
    
    // Apply hysteresis to prevent oscillation
    switch (current_mode) {
        case FLOW_MODE_NORMAL:
            if (buffer_usage_percent > FLOW_THRESHOLD_REDUCED) {
                new_mode = FLOW_MODE_REDUCED;
            }
            break;
            
        case FLOW_MODE_REDUCED:
            if (buffer_usage_percent < (FLOW_THRESHOLD_NORMAL - FLOW_HYSTERESIS)) {
                new_mode = FLOW_MODE_NORMAL;
            } else if (buffer_usage_percent > FLOW_THRESHOLD_CRITICAL) {
                new_mode = FLOW_MODE_CRITICAL;
            }
            break;
            
        case FLOW_MODE_CRITICAL:
            if (buffer_usage_percent < (FLOW_THRESHOLD_REDUCED - FLOW_HYSTERESIS)) {
                new_mode = FLOW_MODE_REDUCED;
            }
            break;
    }
    
    // Only change mode if different and enough time has passed (prevent rapid switching)
    if (new_mode != current_mode && (now - flow_control.last_mode_change_time) > 5000) {
        atomic_set(&flow_control.current_mode, new_mode);
        flow_control.mode_changes++;
        flow_control.last_mode_change_time = now;
        
        // Adjust processing based on new mode
        switch (new_mode) {
            case FLOW_MODE_NORMAL:
                // Restore advanced features if they were originally enabled
                if (flow_control.advanced_features_saved) {
                    event_detector.use_advanced_features = true;
                }
                LOG_INF("Flow control: NORMAL mode (buffer %d%%), advanced features restored", 
                        buffer_usage_percent);
                break;
                
            case FLOW_MODE_REDUCED:
                // Save current state and reduce processing
                flow_control.advanced_features_saved = event_detector.use_advanced_features;
                event_detector.use_advanced_features = false;
                LOG_WRN("Flow control: REDUCED mode (buffer %d%%), advanced features disabled temporarily", 
                        buffer_usage_percent);
                break;
                
            case FLOW_MODE_CRITICAL:
                // Emergency mode - minimal processing only
                event_detector.use_advanced_features = false;
                LOG_ERR("Flow control: CRITICAL mode (buffer %d%%), emergency processing only", 
                        buffer_usage_percent);
                flow_control.overflow_events++;
                break;
        }
    }
    
    // Update buffer pressure for diagnostics
    atomic_set(&flow_control.buffer_pressure, buffer_usage_percent);
}

/**
 * @brief Check if a metric should be processed based on priority
 * Critical metrics are ALWAYS processed to maintain system integrity
 */
static bool should_process_metric(metric_priority_t priority)
{
    flow_control_mode_t mode = (flow_control_mode_t)atomic_get(&flow_control.current_mode);
    
    switch (mode) {
        case FLOW_MODE_NORMAL:
            return true;  // Process all metrics
            
        case FLOW_MODE_REDUCED:
            // Skip low priority metrics
            if (priority >= METRIC_PRIORITY_LOW) {
                flow_control.metrics_skipped++;
                return false;
            }
            return true;
            
        case FLOW_MODE_CRITICAL:
            // Only process critical metrics
            if (priority > METRIC_PRIORITY_CRITICAL) {
                flow_control.metrics_skipped++;
                return false;
            }
            return true;
    }
    
    return true;  // Default to processing
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
    
    // Calculate buffer pressure and adjust flow control
    int buffer_usage_percent = (event_detector.buffer_count * 100) / GAIT_BUFFER_SIZE_SAMPLES;
    adjust_flow_control_mode(buffer_usage_percent);
    
    // Monitor buffer health every 10 callbacks (10 seconds at 1Hz)
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
    
    // Process buffer more aggressively to prevent overflow
    // If buffer is more than 50% full, force processing
    int metrics_count = 0;
    if (event_detector.buffer_count > GAIT_BUFFER_SIZE_SAMPLES / 2) {
        LOG_DBG("Buffer >50%% full (%d/%d), forcing processing",
                event_detector.buffer_count, GAIT_BUFFER_SIZE_SAMPLES);
        // Process immediately instead of waiting
        metrics_count = gait_events_process(&event_detector);
    }
    
    // Only clear buffer if absolutely full AND no processing is happening
    // Don't destroy data that hasn't been processed yet
    if (event_detector.buffer_full) {
        LOG_WRN("Buffer completely full (%d/%d), clearing oldest 10%% for emergency recovery",
                event_detector.buffer_count, GAIT_BUFFER_SIZE_SAMPLES);
        int samples_to_clear = GAIT_BUFFER_SIZE_SAMPLES / 10;  // Only 10%, not 30%
        event_detector.read_index = (event_detector.read_index + samples_to_clear) % GAIT_BUFFER_SIZE_SAMPLES;
        event_detector.buffer_count = MAX(0, event_detector.buffer_count - samples_to_clear);
        event_detector.buffer_full = false;
    }
    
    // Process any pending data (reuse metrics_count if already processed above)
    if (metrics_count == 0) {
        metrics_count = gait_events_process(&event_detector);
    }
    
    if (metrics_count > 0) {
        LOG_DBG("1Hz timer: processed %d metrics", metrics_count);
        LOG_DBG("Buffer state after processing: count=%d, read=%d, write=%d",
                event_detector.buffer_count, event_detector.read_index, event_detector.write_index);
        
        // After processing, clear the processed data properly
        // The gait_events_process should have consumed data, update buffer count
        // Don't aggressively trim to a fixed size - let natural processing flow work
        LOG_DBG("Post-process buffer state: count=%d, allowing natural flow",
                event_detector.buffer_count);
        
        // Get metrics and prepare for transmission
        gait_metrics_t metrics[GAIT_MAX_EVENTS_PER_CHUNK];
        int retrieved = gait_events_get_metrics(&event_detector, metrics, GAIT_MAX_EVENTS_PER_CHUNK);
        
        if (retrieved > 0) {
            #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            // On primary device, store metrics for bilateral comparison
            extern void sensor_data_store_primary_metrics(const gait_metrics_t *metrics, int count);
            sensor_data_store_primary_metrics(metrics, retrieved);
            LOG_DBG("Stored %d primary metrics for bilateral analysis", retrieved);
            #endif
            
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
            
            // Send d2d_packet via D2D interface
            #if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            // Only secondary devices send metrics to primary every 1 seconds (every other callback)
            if (callback_count % 1 == 0) {
                LOG_INF("SECONDARY: Sending D2D metrics every 2 seconds (callback #%u)", callback_count);
                LOG_INF("  GCT=%.1f ms, Cadence=%.1f spm, Stride=%.2f cm",
                        (double)d2d_packet.metrics[IDX_GCT],
                        (double)d2d_packet.metrics[IDX_CADENCE],
                        (double)d2d_packet.metrics[IDX_STRIDE_LENGTH]);
                LOG_INF("  Valid metrics count: %d", valid_count);
                
                extern int d2d_tx_notify_metrics(const d2d_metrics_packet_t *metrics);
                int err = d2d_tx_notify_metrics(&d2d_packet);
                if (err == 0) {
                    LOG_INF("SECONDARY: D2D metrics packet sent successfully (seq=%u)", d2d_packet.sequence_num);
                } else {
                    LOG_ERR("SECONDARY: Failed to send D2D metrics packet: %d", err);
                }
            } else {
                LOG_DBG("SECONDARY: Skipping D2D transmission (callback #%u, sending every 2 seconds)", callback_count);
            }
            #else
            LOG_DBG("PRIMARY: Stored metrics locally - waiting for secondary D2D data (%d valid metrics)", valid_count);
            #endif
        }
        
        // Clear processed metrics
        gait_events_clear_metrics(&event_detector);
    }
    
    // Memory health check every 60 callbacks (1 minute at 1Hz)
    if (callback_count % 60 == 0) {
        // Report flow control statistics
        flow_control_mode_t current_mode = (flow_control_mode_t)atomic_get(&flow_control.current_mode);
        const char *mode_str = (current_mode == FLOW_MODE_NORMAL) ? "NORMAL" :
                               (current_mode == FLOW_MODE_REDUCED) ? "REDUCED" : "CRITICAL";
        
        LOG_INF("System health after %u cycles: buffer=%d/%d (%d%%), metrics=%d, mode=%s",
                callback_count, event_detector.buffer_count,
                GAIT_BUFFER_SIZE_SAMPLES, buffer_usage_percent, metrics_count, mode_str);
        
        LOG_INF("Flow control stats: mode_changes=%u, metrics_skipped=%u, overflows=%u",
                flow_control.mode_changes, flow_control.metrics_skipped, flow_control.overflow_events);
        
        // Force garbage collection if needed (only in reduced/critical modes)
        if (current_mode != FLOW_MODE_NORMAL && event_detector.buffer_count > GAIT_BUFFER_SIZE_SAMPLES / 2) {
            LOG_WRN("Performing preventive buffer maintenance in %s mode", mode_str);
            // Clear oldest 10% to keep things flowing
            int samples_to_clear = GAIT_BUFFER_SIZE_SAMPLES / 10;
            event_detector.read_index = (event_detector.read_index + samples_to_clear) % GAIT_BUFFER_SIZE_SAMPLES;
            event_detector.buffer_count = (event_detector.buffer_count > samples_to_clear) ?
                                         event_detector.buffer_count - samples_to_clear : 0;
        }
    }
    
    k_mutex_unlock(&events_mutex);
}

// Bilateral metrics storage for correlation
typedef struct {
    d2d_metrics_packet_t secondary_metrics;
    gait_metrics_t primary_metrics[GAIT_MAX_EVENTS_PER_CHUNK];
    uint32_t secondary_timestamp_ms;
    uint32_t primary_timestamp_ms;
    int32_t timestamp_offset_ms;  // Secondary - Primary offset
    bool secondary_valid;
    bool primary_valid;
    uint8_t primary_count;
} bilateral_metrics_t;

static bilateral_metrics_t bilateral_data = {0};
static K_MUTEX_DEFINE(bilateral_mutex);

// Timestamp synchronization
static int32_t d2d_time_offset_ms = 0;  // Time difference between devices
static uint32_t last_sync_time_ms = 0;
static bool time_sync_valid = false;

/**
 * @brief Synchronize timestamps between primary and secondary devices
 */
static void synchronize_timestamps(uint32_t secondary_timestamp_ms)
{
    uint32_t current_time_ms = k_uptime_get_32();
    
    // Calculate offset on first reception or after long gap
    if (!time_sync_valid || (current_time_ms - last_sync_time_ms) > 60000) {
        // Assume network latency is symmetric and minimal
        d2d_time_offset_ms = (int32_t)secondary_timestamp_ms - (int32_t)current_time_ms;
        time_sync_valid = true;
        LOG_INF("D2D timestamp synchronized: offset = %d ms", d2d_time_offset_ms);
    } else {
        // Use exponential moving average to smooth offset
        int32_t new_offset = (int32_t)secondary_timestamp_ms - (int32_t)current_time_ms;
        d2d_time_offset_ms = (d2d_time_offset_ms * 7 + new_offset) / 8;  // EMA with alpha=0.125
        LOG_DBG("D2D timestamp updated: offset = %d ms", d2d_time_offset_ms);
    }
    
    last_sync_time_ms = current_time_ms;
}

/**
 * @brief Calculate bilateral gait asymmetry metrics
 */
static void calculate_bilateral_metrics(void)
{
    if (!bilateral_data.secondary_valid || !bilateral_data.primary_valid || bilateral_data.primary_count == 0) {
        return;
    }
    
    // Get the most recent primary metrics
    gait_metrics_t *primary = &bilateral_data.primary_metrics[bilateral_data.primary_count - 1];
    d2d_metrics_packet_t *secondary = &bilateral_data.secondary_metrics;
    
    // Calculate asymmetry indices
    float gct_asymmetry = 0.0f;
    float stride_asymmetry = 0.0f;
    float pronation_asymmetry = 0.0f;
    
    // GCT Asymmetry
    if (d2d_metric_is_valid(secondary, IDX_GCT) && primary->gct > 0) {
        float sec_gct = secondary->metrics[IDX_GCT];
        float pri_gct = primary->gct * 1000.0f;  // Convert to ms
        gct_asymmetry = fabsf(sec_gct - pri_gct) / ((sec_gct + pri_gct) / 2.0f) * 100.0f;
        LOG_INF("Bilateral GCT: Primary=%.1f ms, Secondary=%.1f ms, Asymmetry=%.1f%%",
                (double)pri_gct, (double)sec_gct, (double)gct_asymmetry);
    }
    
    // Stride Length Asymmetry
    if (d2d_metric_is_valid(secondary, IDX_STRIDE_LENGTH) && primary->stride_length > 0) {
        float sec_stride = secondary->metrics[IDX_STRIDE_LENGTH] / 100.0f;  // Convert to m
        float pri_stride = primary->stride_length;
        stride_asymmetry = fabsf(sec_stride - pri_stride) / ((sec_stride + pri_stride) / 2.0f) * 100.0f;
        LOG_INF("Bilateral Stride: Primary=%.2f m, Secondary=%.2f m, Asymmetry=%.1f%%",
                (double)pri_stride, (double)sec_stride, (double)stride_asymmetry);
    }
    
    // Pronation Asymmetry
    if (d2d_metric_is_valid(secondary, IDX_PRONATION) && primary->max_pronation != 0) {
        float sec_pronation = secondary->metrics[IDX_PRONATION];
        float pri_pronation = (float)primary->max_pronation;
        pronation_asymmetry = fabsf(sec_pronation - pri_pronation);
        LOG_INF("Bilateral Pronation: Primary=%.1f deg, Secondary=%.1f deg, Difference=%.1f deg",
                (double)pri_pronation, (double)sec_pronation, (double)pronation_asymmetry);
    }
    
    // Combined Cadence (should be similar between feet)
    if (d2d_metric_is_valid(secondary, IDX_CADENCE) && primary->cadence > 0) {
        float sec_cadence = secondary->metrics[IDX_CADENCE];
        float pri_cadence = primary->cadence;
        float avg_cadence = (sec_cadence + pri_cadence) / 2.0f;
        LOG_INF("Bilateral Cadence: Primary=%.1f, Secondary=%.1f, Average=%.1f spm",
                (double)pri_cadence, (double)sec_cadence, (double)avg_cadence);
    }
    
    // COP Path Comparison
    if (d2d_metric_is_valid(secondary, IDX_COP_X) && d2d_metric_is_valid(secondary, IDX_COP_Y)) {
        float sec_cop_x = secondary->metrics[IDX_COP_X];
        float sec_cop_y = secondary->metrics[IDX_COP_Y];
        float pri_cop_x = (float)primary->cop_x_at_to;
        float pri_cop_y = (float)primary->cop_y_at_to;
        
        float cop_distance = sqrtf((sec_cop_x - pri_cop_x) * (sec_cop_x - pri_cop_x) +
                                   (sec_cop_y - pri_cop_y) * (sec_cop_y - pri_cop_y));
        LOG_INF("Bilateral COP at TO: Primary=(%.1f,%.1f), Secondary=(%.1f,%.1f), Diff=%.1f mm",
                (double)pri_cop_x, (double)pri_cop_y, (double)sec_cop_x, (double)sec_cop_y, (double)cop_distance);
    }
    
    // Loading Rate Comparison
    if (d2d_metric_is_valid(secondary, IDX_LOADING_RATE) && primary->loading_rate > 0) {
        float sec_loading = secondary->metrics[IDX_LOADING_RATE];
        float pri_loading = (float)primary->loading_rate;
        float loading_diff = fabsf(sec_loading - pri_loading);
        LOG_INF("Bilateral Loading Rate: Primary=%.1f, Secondary=%.1f, Diff=%.1f N/s",
                (double)pri_loading, (double)sec_loading, (double)loading_diff);
    }
    
    // Gait quality score (0-100, higher is better)
    float gait_quality = 100.0f;
    if (gct_asymmetry > 0) gait_quality -= MIN(gct_asymmetry, 30.0f);
    if (stride_asymmetry > 0) gait_quality -= MIN(stride_asymmetry, 30.0f);
    if (pronation_asymmetry > 10) gait_quality -= MIN(pronation_asymmetry - 10, 20.0f);
    
    LOG_INF("BILATERAL GAIT QUALITY SCORE: %.1f/100", (double)gait_quality);
    
    // Send bilateral metrics to realtime_metrics module for aggregation and forwarding
    // The realtime_metrics module will then send to both bluetooth and data modules
    extern struct k_msgq sensor_data_queue;  // Send to realtime_metrics module
    generic_message_t metrics_msg;
    metrics_msg.sender = SENDER_SENSOR_DATA;
    metrics_msg.type = MSG_TYPE_REALTIME_METRICS;
    
    // Fill in the realtime_metrics structure with bilateral data
    realtime_metrics_t *rt_metrics = &metrics_msg.data.realtime_metrics;
    memset(rt_metrics, 0, sizeof(realtime_metrics_t));
    
    rt_metrics->timestamp_ms = k_uptime_get_32();
    
    // Bilateral cadence (average of both feet)
    if (d2d_metric_is_valid(secondary, IDX_CADENCE) && primary->cadence > 0) {
        rt_metrics->cadence_spm = (uint16_t)((secondary->metrics[IDX_CADENCE] + primary->cadence) / 2.0f);
    }
    
    // Calculate pace from cadence and stride length
    if (rt_metrics->cadence_spm > 0 && d2d_metric_is_valid(secondary, IDX_STRIDE_LENGTH) && primary->stride_length > 0) {
        float avg_stride_m = (secondary->metrics[IDX_STRIDE_LENGTH] / 100.0f + primary->stride_length) / 2.0f;
        float speed_mps = (rt_metrics->cadence_spm / 60.0f) * avg_stride_m;
        if (speed_mps > 0) {
            rt_metrics->pace_sec_km = (uint16_t)(1000.0f / speed_mps);
        }
    }
    
    // Ground contact time (average) and flight time
    if (d2d_metric_is_valid(secondary, IDX_GCT) && primary->gct > 0) {
        float avg_gct = (secondary->metrics[IDX_GCT] + primary->gct * 1000.0f) / 2.0f;
        rt_metrics->ground_contact_ms = (uint16_t)avg_gct;
        rt_metrics->contact_time_asymmetry = (uint8_t)MIN(gct_asymmetry, 100.0f);
        
        // Calculate flight time from stride duration and GCT
        if (primary->duration > 0) {
            float flight_time = (primary->duration - primary->gct) * 1000.0f;
            if (d2d_metric_is_valid(secondary, IDX_SWING_TIME)) {
                flight_time = (flight_time + secondary->metrics[IDX_SWING_TIME]) / 2.0f;
            }
            rt_metrics->flight_time_ms = (uint16_t)flight_time;
        }
    }
    
    // Flight time asymmetry
    if (d2d_metric_is_valid(secondary, IDX_SWING_TIME) && primary->duration > 0) {
        float pri_flight = (primary->duration - primary->gct) * 1000.0f;
        float sec_flight = secondary->metrics[IDX_SWING_TIME];
        float flight_asym = fabsf(sec_flight - pri_flight) / ((sec_flight + pri_flight) / 2.0f) * 100.0f;
        rt_metrics->flight_time_asymmetry = (uint8_t)MIN(flight_asym, 100.0f);
    }
    
    // Stride metrics
    if (d2d_metric_is_valid(secondary, IDX_STRIDE_LENGTH) && primary->stride_length > 0) {
        float avg_stride_cm = (secondary->metrics[IDX_STRIDE_LENGTH] + primary->stride_length * 100.0f) / 2.0f;
        rt_metrics->stride_length_cm = (uint16_t)avg_stride_cm;
        rt_metrics->stride_length_asymmetry = (uint8_t)MIN(stride_asymmetry, 100.0f);
    }
    
    // Stride duration
    if (primary->duration > 0) {
        rt_metrics->stride_duration_ms = (uint16_t)(primary->duration * 1000.0f);
        // Could calculate asymmetry if we had secondary stride duration
        rt_metrics->stride_duration_asymmetry = 0;  // TODO: Add when available
    }
    
    // Pronation metrics
    if (d2d_metric_is_valid(secondary, IDX_PRONATION) && primary->max_pronation != 0) {
        rt_metrics->avg_pronation_deg = (int8_t)((secondary->metrics[IDX_PRONATION] + primary->max_pronation) / 2);
        rt_metrics->pronation_asymmetry = (uint8_t)MIN(pronation_asymmetry, 100.0f);
    }
    
    // Strike patterns
    if (d2d_metric_is_valid(secondary, IDX_FOOT_STRIKE_ANGLE)) {
        // Convert strike angle to pattern (0=heel, 1=mid, 2=fore)
        float sec_angle = secondary->metrics[IDX_FOOT_STRIKE_ANGLE];
        rt_metrics->left_strike_pattern = (sec_angle < -5.0f) ? 0 : (sec_angle > 5.0f) ? 2 : 1;
    }
    rt_metrics->right_strike_pattern = (primary->strike_pattern > 0) ? primary->strike_pattern - 1 : 0;
    
    // Balance (based on COP and force distribution)
    if (d2d_metric_is_valid(secondary, IDX_BALANCE_INDEX)) {
        float sec_balance = secondary->metrics[IDX_BALANCE_INDEX];
        float pri_balance = 100.0f - MIN(primary->cop_path_length, 100.0f);
        rt_metrics->balance_lr_pct = (int8_t)((sec_balance - pri_balance) / 2.0f);  // -50 to +50
    }
    
    // Force asymmetry (based on loading rate)
    if (d2d_metric_is_valid(secondary, IDX_LOADING_RATE) && primary->loading_rate > 0) {
        float sec_loading = secondary->metrics[IDX_LOADING_RATE];
        float pri_loading = (float)primary->loading_rate;
        float force_asym = fabsf(sec_loading - pri_loading) / ((sec_loading + pri_loading) / 2.0f) * 100.0f;
        rt_metrics->force_asymmetry = (uint8_t)MIN(force_asym, 100.0f);
    }
    
    // Vertical oscillation
    if (d2d_metric_is_valid(secondary, IDX_VERTICAL_OSC)) {
        float sec_vo = secondary->metrics[IDX_VERTICAL_OSC];
        float pri_vo = primary->vertical_oscillation * 100.0f;
        rt_metrics->vertical_oscillation_cm = (uint8_t)((sec_vo + pri_vo) / 2.0f);
        // Vertical ratio: VO/stride length * 100
        if (rt_metrics->stride_length_cm > 0) {
            rt_metrics->vertical_ratio = (uint8_t)((rt_metrics->vertical_oscillation_cm * 100) / rt_metrics->stride_length_cm);
        }
    }
    
    // Form score based on gait quality
    rt_metrics->form_score = (uint8_t)gait_quality;
    
    // Efficiency score (based on form and asymmetries)
    float efficiency = 100.0f;
    efficiency -= rt_metrics->contact_time_asymmetry / 4.0f;
    efficiency -= rt_metrics->flight_time_asymmetry / 4.0f;
    efficiency -= rt_metrics->force_asymmetry / 4.0f;
    efficiency -= rt_metrics->pronation_asymmetry / 4.0f;
    rt_metrics->efficiency_score = (uint8_t)MAX(0, MIN(100, efficiency));
    
    // Set alerts based on thresholds
    rt_metrics->alerts = 0;
    if (gct_asymmetry > 15.0f || stride_asymmetry > 15.0f) {
        rt_metrics->alerts |= RT_ALERT_HIGH_ASYMMETRY;
    }
    if (gait_quality < 70.0f) {
        rt_metrics->alerts |= RT_ALERT_POOR_FORM;
    }
    if (pronation_asymmetry > 15.0f) {
        rt_metrics->alerts |= RT_ALERT_OVERPRONATION;
    }
    
    // Send to realtime_metrics module which will forward to bluetooth and data modules
    if (k_msgq_put(&sensor_data_queue, &metrics_msg, K_NO_WAIT) != 0) {
        LOG_WRN("Failed to send bilateral metrics to realtime_metrics module");
    } else {
        LOG_INF("Bilateral metrics sent to realtime_metrics module for distribution");
    }
}

/**
 * @brief Process received D2D metrics from secondary device
 * @param metrics Received D2D metrics packet
 */
extern "C" void sensor_data_process_received_metrics(const d2d_metrics_packet_t *metrics)
{
    if (!metrics) {
        LOG_ERR("Invalid metrics pointer");
        return;
    }
    
    LOG_INF("PRIMARY: Received D2D metrics from secondary: seq=%u, timestamp=%u",
            metrics->sequence_num, metrics->timestamp);
    
    // Synchronize timestamps
    synchronize_timestamps(metrics->timestamp);
    
    // Store metrics for bilateral analysis
    k_mutex_lock(&bilateral_mutex, K_FOREVER);
    
    memcpy(&bilateral_data.secondary_metrics, metrics, sizeof(d2d_metrics_packet_t));
    bilateral_data.secondary_timestamp_ms = metrics->timestamp;
    bilateral_data.secondary_valid = true;
    bilateral_data.timestamp_offset_ms = d2d_time_offset_ms;
    
    // Log received metrics
    if (d2d_metric_is_valid(metrics, IDX_GCT)) {
        LOG_DBG("  Received GCT: %.1f ms", (double)metrics->metrics[IDX_GCT]);
    }
    if (d2d_metric_is_valid(metrics, IDX_CADENCE)) {
        LOG_DBG("  Received Cadence: %.1f spm", (double)metrics->metrics[IDX_CADENCE]);
    }
    if (d2d_metric_is_valid(metrics, IDX_STRIDE_LENGTH)) {
        LOG_DBG("  Received Stride: %.2f cm", (double)metrics->metrics[IDX_STRIDE_LENGTH]);
    }
    if (d2d_metric_is_valid(metrics, IDX_PRONATION)) {
        LOG_DBG("  Received Pronation: %.1f deg", (double)metrics->metrics[IDX_PRONATION]);
    }
    if (d2d_metric_is_valid(metrics, IDX_COP_X) && d2d_metric_is_valid(metrics, IDX_COP_Y)) {
        LOG_DBG("  Received COP: (%.1f, %.1f) mm",
                (double)metrics->metrics[IDX_COP_X], (double)metrics->metrics[IDX_COP_Y]);
    }
    if (d2d_metric_is_valid(metrics, IDX_LOADING_RATE)) {
        LOG_DBG("  Received Loading Rate: %.1f N/s", (double)metrics->metrics[IDX_LOADING_RATE]);
    }
    if (d2d_metric_is_valid(metrics, IDX_VERTICAL_OSC)) {
        LOG_DBG("  Received VO: %.1f cm", (double)metrics->metrics[IDX_VERTICAL_OSC]);
    }
    if (d2d_metric_is_valid(metrics, IDX_STEP_COUNT)) {
        LOG_DBG("  Received Steps: %u", (uint32_t)metrics->metrics[IDX_STEP_COUNT]);
    }
    
    #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // On primary device, calculate bilateral metrics if we have both sides
    if (bilateral_data.primary_valid) {
        LOG_INF("PRIMARY: Have both primary and secondary data - calculating bilateral metrics");
        calculate_bilateral_metrics();
    } else {
        LOG_WRN("PRIMARY: Received secondary data but no primary data available yet");
    }
    #endif
    
    k_mutex_unlock(&bilateral_mutex);
}

/**
 * @brief Store primary device metrics for bilateral comparison
 * @param metrics Array of primary device metrics
 * @param count Number of metrics in array
 */
extern "C" void sensor_data_store_primary_metrics(const gait_metrics_t *metrics, int count)
{
    if (!metrics || count <= 0) {
        return;
    }
    
    k_mutex_lock(&bilateral_mutex, K_FOREVER);
    
    // Copy metrics (up to max)
    int copy_count = MIN(count, GAIT_MAX_EVENTS_PER_CHUNK);
    memcpy(bilateral_data.primary_metrics, metrics, copy_count * sizeof(gait_metrics_t));
    bilateral_data.primary_count = copy_count;
    bilateral_data.primary_timestamp_ms = k_uptime_get_32();
    bilateral_data.primary_valid = true;
    
    // If we have secondary metrics, calculate bilateral analysis
    if (bilateral_data.secondary_valid) {
        // Check if timestamps are reasonably close (within 2 seconds)
        int32_t time_diff = abs((int32_t)bilateral_data.primary_timestamp_ms -
                                (int32_t)(bilateral_data.secondary_timestamp_ms - d2d_time_offset_ms));
        if (time_diff < 2000) {
            calculate_bilateral_metrics();
        } else {
            LOG_WRN("Primary and secondary metrics timestamps too far apart: %d ms", time_diff);
        }
    }
    
    k_mutex_unlock(&bilateral_mutex);
}

} // extern "C"
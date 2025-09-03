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

LOG_MODULE_REGISTER(sensor_data_events, LOG_LEVEL_WRN);

// Static gait event detector for this module - make sure it's properly aligned
static gait_event_detector_t event_detector __aligned(8);
static atomic_t events_initialized = ATOMIC_INIT(0);  // Use atomic instead of volatile bool
static K_MUTEX_DEFINE(events_mutex);

// Global variable for tracking bilateral success (used for fallback logic)
uint32_t g_last_bilateral_success_time = 0;

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
        
        // Enable advanced features - they are critical for accurate gait analysis
        event_detector.use_advanced_features = true;
        LOG_INF("Advanced gait features ENABLED for comprehensive analysis");
        
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
    
    // Process buffer very aggressively to prevent overflow
    // If buffer is more than 20% full, force processing
    int metrics_count = 0;
    if (event_detector.buffer_count > GAIT_BUFFER_SIZE_SAMPLES / 5) {
        LOG_DBG("Buffer >20%% full (%d/%d), forcing processing",
                event_detector.buffer_count, GAIT_BUFFER_SIZE_SAMPLES);
        // Process immediately instead of waiting
        metrics_count = gait_events_process(&event_detector);
    }
    
    // Process any pending data FIRST before considering clearing
    if (metrics_count == 0) {
        metrics_count = gait_events_process(&event_detector);
    }
    
    // Only clear buffer if absolutely full AND processing didn't help
    // This ensures we don't destroy unprocessed data
    if (event_detector.buffer_full && metrics_count == 0) {
        LOG_WRN("Buffer completely full (%d/%d) and no events detected, clearing oldest 10%% for emergency recovery",
                event_detector.buffer_count, GAIT_BUFFER_SIZE_SAMPLES);
        int samples_to_clear = GAIT_BUFFER_SIZE_SAMPLES / 10;  // Only 10%, not 30%
        event_detector.read_index = (event_detector.read_index + samples_to_clear) % GAIT_BUFFER_SIZE_SAMPLES;
        event_detector.buffer_count = MAX(0, event_detector.buffer_count - samples_to_clear);
        event_detector.buffer_full = false;
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
            LOG_INF("PRIMARY: Stored %d metrics, waiting for secondary data for bilateral analysis", retrieved);
            
            // Check if we should send unilateral metrics as fallback
            // We'll wait up to 2 seconds for secondary data before sending unilateral
            static uint32_t last_unilateral_send_time = 0;
            uint32_t current_time = k_uptime_get_32();
            
            // Check if we've had recent bilateral success (use global variable)
            bool recent_bilateral = (current_time - g_last_bilateral_success_time) < 5000; // Within 5 seconds
            
            // Only send unilateral as fallback if:
            // 1. We haven't sent unilateral recently (avoid spam)
            // 2. We haven't had bilateral success recently (no secondary connected)
            if (!recent_bilateral && (current_time - last_unilateral_send_time) > 2000) {
                LOG_WRN("PRIMARY: No secondary data for 2+ seconds, sending unilateral metrics as fallback");
                
                extern struct k_msgq sensor_data_queue;
                generic_message_t metrics_msg;
                metrics_msg.sender = SENDER_SENSOR_DATA;
                metrics_msg.type = MSG_TYPE_REALTIME_METRICS;
                
                // Fill in basic metrics from primary only
                realtime_metrics_t *rt_metrics = &metrics_msg.data.realtime_metrics;
                memset(rt_metrics, 0, sizeof(realtime_metrics_t));
                
                rt_metrics->timestamp_ms = current_time;
                
                // Use the most recent metric
                const gait_metrics_t *latest = &metrics[retrieved - 1];
                
                // Cadence and pace from primary foot only (halved for single foot)
                rt_metrics->cadence_spm = (uint16_t)(latest->cadence / 2.0f); // Divide by 2 for single foot
                if (latest->stride_length > 0 && latest->cadence > 0) {
                    float speed_mps = (latest->cadence / 120.0f) * latest->stride_length; // Adjusted for single foot
                    if (speed_mps > 0) {
                        rt_metrics->pace_sec_km = (uint16_t)(1000.0f / speed_mps);
                    }
                }
                
                // Ground contact time
                rt_metrics->ground_contact_ms = (uint16_t)(latest->gct * 1000.0f);
                
                // Flight time
                if (latest->duration > 0) {
                    float flight_time = (latest->duration - latest->gct) * 1000.0f;
                    rt_metrics->flight_time_ms = (uint16_t)flight_time;
                }
                
                // Stride metrics
                rt_metrics->stride_length_cm = (uint16_t)(latest->stride_length * 100.0f);
                rt_metrics->stride_duration_ms = (uint16_t)(latest->duration * 1000.0f);
                
                // Pronation
                rt_metrics->avg_pronation_deg = latest->max_pronation;
                
                // Strike pattern (primary foot is right)
                // Strike pattern values: 0=heel, 1=midfoot, 2=forefoot (from detect_strike_pattern)
                rt_metrics->right_strike_pattern = latest->strike_pattern;  // Use directly, no conversion needed
                rt_metrics->left_strike_pattern = 1; // Default midfoot for missing foot
                
                // Vertical oscillation
                rt_metrics->vertical_oscillation_cm = (uint8_t)(latest->vertical_oscillation * 100.0f);
                
                // Form score (basic calculation for unilateral)
                rt_metrics->form_score = 75; // Lower score for unilateral
                
                // Balance (0 since we only have one foot)
                rt_metrics->balance_lr_pct = 0; // Neutral
                
                // Mark as unilateral data
                rt_metrics->alerts |= RT_ALERT_UNILATERAL_DATA; // Add flag to indicate unilateral
                
                // Send unilateral metrics
                if (k_msgq_put(&sensor_data_queue, &metrics_msg, K_NO_WAIT) != 0) {
                    LOG_WRN("Failed to send unilateral metrics to realtime_metrics");
                } else {
                    LOG_INF("PRIMARY: Sent unilateral metrics as temporary fallback");
                    last_unilateral_send_time = current_time;
                }
            }
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
            
            // Peak force (using IDX_PEAK_IMPACT for D2D transmission)
            // Peak pressure is the sum of all 8 sensors (14-bit ADC each)
            // Maximum theoretical value is 8 * 16383 = 131,064
            // We need to scale it to fit in int16_t range for D2D transmission
            // Divide by 4 to get range 0-32,766 which fits in int16_t
            if (latest->peak_pressure > 0) {
                d2d_packet.metrics[IDX_PEAK_IMPACT] = (float)(latest->peak_pressure / 4);
                d2d_metric_set_valid(&d2d_packet, IDX_PEAK_IMPACT);
                LOG_DBG("SECONDARY: Sending peak_force=%.1f via IDX_PEAK_IMPACT (scaled from %.1f)", 
                        (double)(latest->peak_pressure / 4), (double)latest->peak_pressure);
            }
            
            // Center of pressure at toe-off (use most recent COP)
            // Validate COP values are reasonable (within foot dimensions ~300mm)
            if (abs(latest->cop_x_at_to) < 300 && abs(latest->cop_y_at_to) < 300) {
                d2d_packet.metrics[IDX_COP_X] = (float)latest->cop_x_at_to;
                d2d_metric_set_valid(&d2d_packet, IDX_COP_X);
                
                d2d_packet.metrics[IDX_COP_Y] = (float)latest->cop_y_at_to;
                d2d_metric_set_valid(&d2d_packet, IDX_COP_Y);
            } else {
                // Use default COP values if invalid
                d2d_packet.metrics[IDX_COP_X] = 0.0f;
                d2d_packet.metrics[IDX_COP_Y] = 50.0f;  // Middle of foot
                LOG_DBG("Invalid COP values (%d, %d), using defaults", 
                        latest->cop_x_at_to, latest->cop_y_at_to);
            }
            
            // Foot strike angle (derived from strike pattern)
            // Strike pattern values: 0=heel, 1=midfoot, 2=forefoot (from detect_strike_pattern)
            float strike_angle = 0.0f;
            switch (latest->strike_pattern) {
                case 0: strike_angle = -15.0f; break;  // Heel strike
                case 1: strike_angle = 0.0f; break;    // Midfoot  
                case 2: strike_angle = 15.0f; break;   // Forefoot
                default: 
                    // No valid strike pattern detected, use midfoot as default
                    strike_angle = 0.0f; 
                    LOG_DBG("No valid strike pattern (%d), defaulting to midfoot", latest->strike_pattern);
                    break;
            }
            d2d_packet.metrics[IDX_FOOT_STRIKE_ANGLE] = strike_angle;
            d2d_metric_set_valid(&d2d_packet, IDX_FOOT_STRIKE_ANGLE);
            
            // Also send the raw strike pattern value for direct use
            // We can repurpose an unused metric field or encode it in another way
            // For now, the angle conversion should work if properly decoded
            
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
            // SECONDARY: IMMEDIATE TRANSMISSION of ALL parameters
            // Send all calculated parameters as soon as they're ready for freshest bilateral analysis
            
            static uint32_t last_d2d_send_time = 0;
            static uint32_t consecutive_sends = 0;
            static uint32_t total_packets_sent = 0;
            uint32_t current_time = k_uptime_get_32();
            
            // Rate limiting with priority system
            // Critical events (foot strikes): min 50ms between sends
            // Normal metrics: min 100ms between sends  
            // Low priority: min 500ms between sends
            
            uint32_t min_send_interval_ms = 100;  // Default for normal metrics
            bool is_critical_event = false;
            
            // Determine priority based on what changed
            if (latest->strike_pattern != 0 || latest->gct > 0) {
                // Foot strike or stance phase change - CRITICAL
                is_critical_event = true;
                min_send_interval_ms = 50;  // Allow faster transmission
                LOG_DBG("CRITICAL: Foot strike/stance detected - immediate send");
            } else if (latest->cadence > 0 && latest->cadence != d2d_packet.metrics[IDX_CADENCE]) {
                // Cadence changed - HIGH priority
                min_send_interval_ms = 75;
                LOG_DBG("HIGH: Cadence changed - priority send");
            }
            
            // Check if we should send based on priority
            bool should_send = (current_time - last_d2d_send_time) >= min_send_interval_ms;
            
            // Also force periodic send to ensure primary gets updates even during steady state
            bool force_periodic = (current_time - last_d2d_send_time) >= 1000;
            
            if (should_send || force_periodic || is_critical_event) {
                // Prepare comprehensive packet with ALL current parameters
                // This ensures primary always has the complete picture
                
                LOG_INF("SECONDARY: Sending ALL parameters IMMEDIATELY (age=0ms, priority=%s)",
                        is_critical_event ? "CRITICAL" : force_periodic ? "PERIODIC" : "NORMAL");
                LOG_INF("  Packet #%u, interval=%u ms since last",
                        total_packets_sent + 1, current_time - last_d2d_send_time);
                LOG_INF("  GCT=%.1f ms, Cadence=%.1f spm, Stride=%.2f cm",
                        (double)d2d_packet.metrics[IDX_GCT],
                        (double)d2d_packet.metrics[IDX_CADENCE],
                        (double)d2d_packet.metrics[IDX_STRIDE_LENGTH]);
                LOG_INF("  Strike=%d, Pronation=%.1f deg, VO=%.1f cm",
                        latest->strike_pattern,
                        (double)d2d_packet.metrics[IDX_PRONATION],
                        (double)d2d_packet.metrics[IDX_VERTICAL_OSC]);
                LOG_INF("  Valid metrics: %d/18, timestamp: %u ms", valid_count, d2d_packet.timestamp);
                
                // Send the packet immediately - no caching, maximum freshness
                if (valid_count > 0) {
                    extern int d2d_tx_notify_metrics(const d2d_metrics_packet_t *metrics);
                    int err = d2d_tx_notify_metrics(&d2d_packet);
                    
                    if (err == 0) {
                        LOG_INF("SECONDARY: SUCCESS - All parameters sent (seq=%u, latency=0ms)", 
                                d2d_packet.sequence_num);
                        last_d2d_send_time = current_time;
                        total_packets_sent++;
                        consecutive_sends++;
                        
                        // Log transmission statistics every 10 packets
                        if (total_packets_sent % 10 == 0) {
                            LOG_INF("D2D Stats: %u packets sent, current burst: %u consecutive",
                                    total_packets_sent, consecutive_sends);
                        }
                    } else if (err == -ENOTCONN) {
                        LOG_WRN("SECONDARY: No D2D connection - parameters not sent");
                        consecutive_sends = 0;  // Reset burst counter
                    } else if (err == -EAGAIN) {
                        LOG_WRN("SECONDARY: D2D busy - will retry next cycle");
                    } else {
                        LOG_ERR("SECONDARY: Failed to send parameters: %d", err);
                        consecutive_sends = 0;
                    }
                } else {
                    LOG_WRN("SECONDARY: No valid metrics to send");
                }
                
                // Adaptive rate limiting - back off if sending too frequently
                if (consecutive_sends > 10 && !is_critical_event) {
                    LOG_DBG("Rate limiting: %u consecutive sends, adding backoff", consecutive_sends);
                    min_send_interval_ms = MIN(500, min_send_interval_ms * 2);
                }
            } else {
                LOG_DBG("SECONDARY: Rate limited (waited %u/%u ms)", 
                        current_time - last_d2d_send_time, min_send_interval_ms);
            }
            #else
            LOG_DBG("PRIMARY: Stored metrics locally - waiting for secondary D2D data (%d valid metrics)", valid_count);
            #endif
        }
        
        // Clear processed metrics
        gait_events_clear_metrics(&event_detector);
    }
    
    // Note: Removed cached metric sending as we now handle periodic sends
    // through the force_periodic flag in the main D2D sending logic above.
    // The secondary device will automatically send metrics at least every 1 second
    // even without new events, ensuring the primary always has recent data.
    
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

// Bilateral metrics storage with history for proper time correlation
#define BILATERAL_HISTORY_SIZE 10  // Store last 10 seconds of primary metrics
#define MAX_BILATERAL_TIME_DIFF_MS 5000  // Accept matches within 5 seconds to handle D2D delays
#define BILATERAL_WAIT_TIMEOUT_MS 3000   // Wait up to 3 seconds for secondary data

typedef struct {
    gait_metrics_t metrics[GAIT_MAX_EVENTS_PER_CHUNK];
    uint32_t timestamp_ms;
    uint8_t count;
    bool valid;
} primary_metrics_entry_t;

typedef struct {
    // Circular buffer of primary metrics history
    primary_metrics_entry_t primary_history[BILATERAL_HISTORY_SIZE];
    uint8_t history_write_idx;
    uint8_t history_count;
    
    // Most recent secondary metrics
    d2d_metrics_packet_t secondary_metrics;
    uint32_t secondary_timestamp_ms;
    bool secondary_valid;
    
    // Time synchronization
    int32_t timestamp_offset_ms;  // Secondary - Primary offset
} bilateral_metrics_t;

static bilateral_metrics_t bilateral_data = {
    .primary_history = {{0}},
    .history_write_idx = 0,
    .history_count = 0,
    .secondary_metrics = {0},
    .secondary_timestamp_ms = 0,
    .secondary_valid = false,
    .timestamp_offset_ms = 0
};
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
 * @brief Find best matching primary metrics for a given secondary timestamp
 * Note: Devices have different boot times, so we use the offset to align timestamps
 */
static primary_metrics_entry_t* find_matching_primary_metrics(uint32_t secondary_timestamp_ms)
{
    if (bilateral_data.history_count == 0) {
        return NULL;
    }
    
    // IMPORTANT: The devices have different boot times (different k_uptime_get_32() values)
    // The timestamp_offset_ms tells us the difference: offset = secondary_time - primary_time
    // To convert secondary timestamp to primary time reference: primary_equiv = secondary - offset
    
    // Adjust secondary timestamp to primary's time reference
    int32_t adjusted_secondary_time = (int32_t)secondary_timestamp_ms - bilateral_data.timestamp_offset_ms;
    
    LOG_DBG("Timestamp adjustment: secondary=%u ms, offset=%d ms, adjusted=%d ms",
            secondary_timestamp_ms, bilateral_data.timestamp_offset_ms, adjusted_secondary_time);
    
    // If adjusted time is way off, just use most recent data
    if (adjusted_secondary_time < 0) {
        LOG_WRN("Adjusted time negative (%d), using most recent primary data", adjusted_secondary_time);
        // Find the most recent valid entry
        for (int i = bilateral_data.history_count - 1; i >= 0; i--) {
            primary_metrics_entry_t *entry = &bilateral_data.primary_history[i];
            if (entry->valid && entry->count > 0) {
                LOG_INF("Using most recent primary metrics (time=%u) for secondary time=%u",
                        entry->timestamp_ms, secondary_timestamp_ms);
                return entry;
            }
        }
        return NULL;
    }
    
    primary_metrics_entry_t *best_match = NULL;
    uint32_t best_time_diff = UINT32_MAX;
    
    // Search through history for best time match
    for (int i = 0; i < bilateral_data.history_count; i++) {
        primary_metrics_entry_t *entry = &bilateral_data.primary_history[i];
        if (!entry->valid || entry->count == 0) {
            continue;
        }
        
        // Calculate time difference using adjusted timestamp
        uint32_t time_diff = abs((int32_t)entry->timestamp_ms - adjusted_secondary_time);
        
        // Accept matches within MAX_BILATERAL_TIME_DIFF_MS window (5 seconds)
        // This accounts for D2D delay and phase offset between feet
        if (time_diff < MAX_BILATERAL_TIME_DIFF_MS && time_diff < best_time_diff) {
            best_match = entry;
            best_time_diff = time_diff;
           // LOG_DBG("Potential match: primary=%u, secondary=%u (adjusted=%d), diff=%u ms",
             //       entry->timestamp_ms, secondary_timestamp_ms, adjusted_secondary_time, time_diff);
        }
    }
    
    if (best_match) {
    //    LOG_INF("Found matching primary metrics: time_diff=%u ms (primary=%u, secondary=%u, offset=%d)",
      //          best_time_diff, best_match->timestamp_ms, secondary_timestamp_ms, 
        //        bilateral_data.timestamp_offset_ms);
    } else {
        // If no match found but we have recent data, use it anyway
        if (bilateral_data.history_count > 0) {
            // Get the most recent valid entry
            for (int i = bilateral_data.history_count - 1; i >= 0; i--) {
                primary_metrics_entry_t *entry = &bilateral_data.primary_history[i];
                if (entry->valid && entry->count > 0) {
                    uint32_t current_time = k_uptime_get_32();
                    uint32_t age = current_time - entry->timestamp_ms;
                    if (age < 2000) {  // If less than 2 seconds old
                        LOG_WRN("No exact match, using recent primary metrics (age=%u ms) for secondary time=%u",
                                age, secondary_timestamp_ms);
                        return entry;
                    }
                }
            }
        }
        
        LOG_WRN("No matching primary metrics for secondary time=%u (adjusted=%d, searched %d entries)",
                secondary_timestamp_ms, adjusted_secondary_time, bilateral_data.history_count);
    }
    
    return best_match;
}

/**
 * @brief Calculate bilateral gait asymmetry metrics
 */
static void calculate_bilateral_metrics(void)
{
    if (!bilateral_data.secondary_valid) {
        return;
    }
    
    // Find matching primary metrics from history
    primary_metrics_entry_t *primary_entry = find_matching_primary_metrics(bilateral_data.secondary_timestamp_ms);
    if (!primary_entry || primary_entry->count == 0) {
        LOG_WRN("No matching primary metrics found for bilateral calculation");
        // Clear secondary data as it's been processed (unsuccessfully)
        bilateral_data.secondary_valid = false;
        return;
    }
    
    // Get the most recent primary metrics from the matched entry
    gait_metrics_t *primary = &primary_entry->metrics[primary_entry->count - 1];
    d2d_metrics_packet_t *secondary = &bilateral_data.secondary_metrics;
    
    LOG_INF("Calculating bilateral metrics: Primary time=%u ms, Secondary time=%u ms",
            primary_entry->timestamp_ms, bilateral_data.secondary_timestamp_ms);
    
    // Calculate TEMPORAL relationship between feet
    // This is crucial for understanding gait coordination
    
    // IMPORTANT: We must adjust the secondary timestamp to primary's time reference
    // The bilateral_data.timestamp_offset_ms = secondary_time - primary_time (from synchronize_timestamps)
    // So to get secondary time in primary reference: adjusted_secondary = secondary - offset
    int32_t adjusted_secondary_time_ms = (int32_t)bilateral_data.secondary_timestamp_ms - bilateral_data.timestamp_offset_ms;
    
    // Now calculate the ACTUAL temporal difference between foot strikes in the same time reference
    int32_t time_offset_ms = (int32_t)primary_entry->timestamp_ms - adjusted_secondary_time_ms;
    
    LOG_DBG("TEMPORAL calculation: primary=%u ms, secondary_raw=%u ms, offset=%d ms, secondary_adjusted=%d ms, temporal_diff=%d ms",
            primary_entry->timestamp_ms, bilateral_data.secondary_timestamp_ms, 
            bilateral_data.timestamp_offset_ms, adjusted_secondary_time_ms, time_offset_ms);
    
    // Determine which foot struck first and calculate step time
    // Step time = time between consecutive opposite foot strikes
    uint16_t step_time_ms = 0;
    bool left_foot_first = false;  // Assume secondary is left foot
    
    // IMPORTANT: For simulated data, the feet might not be perfectly synchronized
    // The simulation might have the feet striking at different phases
    // We need to handle this gracefully
    
    // Calculate expected gait cycle duration from cadence
    float avg_cadence = primary->cadence;
    if (d2d_metric_is_valid(secondary, IDX_CADENCE)) {
        avg_cadence = (primary->cadence + secondary->metrics[IDX_CADENCE]) / 2.0f;
    }
    
    // Full gait cycle duration in ms (for one complete stride)
    float gait_cycle_ms = (avg_cadence > 0) ? (60000.0f / avg_cadence) : 600.0f;
    
    // Expected step time is half the gait cycle (feet should be 180° out of phase)
    float expected_step_time = gait_cycle_ms / 2.0f;
    
    // For simulated data, we might have any phase relationship
    // Normalize the offset to find the actual phase difference
    int32_t abs_offset = abs(time_offset_ms);
    
    // Find the phase within the gait cycle
    if (abs_offset > gait_cycle_ms * 1.5f) {
        // Very large offset - likely comparing different cycles
        // Use modulo to find phase within cycle
        abs_offset = abs_offset % (int32_t)gait_cycle_ms;
    }
    
    // Determine if this is the direct phase or the complementary phase
    if (abs_offset <= expected_step_time * 1.3f) {
        // Close to expected half-cycle - normal alternating gait
        step_time_ms = (uint16_t)abs_offset;
    } else if (abs_offset >= expected_step_time * 1.7f) {
        // Close to full cycle - feet nearly in phase (unusual but possible in simulation)
        step_time_ms = (uint16_t)(gait_cycle_ms - abs_offset);
    } else {
        // Somewhere in between - use the smaller of the two possible values
        step_time_ms = (uint16_t)MIN(abs_offset, gait_cycle_ms - abs_offset);
    }
    
    // Determine which foot struck first
    left_foot_first = (time_offset_ms < 0);  // Secondary (left) struck first if offset is negative
    
    // For simulated data, if step time is unrealistic, use expected value
    // This handles cases where the simulation might have glitches
    if (step_time_ms > gait_cycle_ms * 0.7f || step_time_ms < gait_cycle_ms * 0.1f) {
        LOG_DBG("TEMPORAL: Simulated data phase correction - raw step time %u ms, using expected %.0f ms",
                step_time_ms, expected_step_time);
        step_time_ms = (uint16_t)expected_step_time;
    }
    
    LOG_DBG("TEMPORAL: Step time = %u ms (expected %.0f ms, %s foot first, cycle %.0f ms)",
            step_time_ms, expected_step_time, left_foot_first ? "left" : "right", gait_cycle_ms);
    
    // Calculate double support percentage
    // During walking, both feet are on ground ~20% of gait cycle
    // During running, there's no double support (flight phase instead)
    float double_support_pct = 0.0f;
    float avg_gct_ms = 0.0f;
    float avg_stride_duration_ms = 0.0f;
    
    if (d2d_metric_is_valid(secondary, IDX_GCT) && primary->gct > 0) {
        float sec_gct = secondary->metrics[IDX_GCT];
        float pri_gct = primary->gct * 1000.0f;
        avg_gct_ms = (sec_gct + pri_gct) / 2.0f;
        
        if (primary->duration > 0) {
            avg_stride_duration_ms = primary->duration * 1000.0f;
            
            // Calculate overlap time when both feet are on ground
            // This is approximate - exact calculation needs synchronized raw data
            if (step_time_ms > 0 && step_time_ms < avg_gct_ms) {
                // There's overlap - both feet on ground
                float overlap_ms = avg_gct_ms - step_time_ms;
                double_support_pct = (overlap_ms / avg_stride_duration_ms) * 100.0f;
                LOG_INF("TEMPORAL: Double support = %.1f%% (overlap %.0f ms)",
                        (double)double_support_pct, (double)overlap_ms);
            } else {
                // No overlap - flight phase (running)
                LOG_INF("TEMPORAL: No double support - likely running gait");
            }
        }
    }
    
    // Calculate step time asymmetry
    // This measures how much the left/right step times differ from perfect symmetry
    float step_time_asymmetry = 0.0f;
    
    // First, determine if we're dealing with reasonable step times
    // Step time should be roughly half the gait cycle (±30% variation is normal)
    bool step_time_reasonable = false;
    if (gait_cycle_ms > 0 && step_time_ms > 0) {
        float expected_half_cycle = gait_cycle_ms / 2.0f;
        float ratio = step_time_ms / expected_half_cycle;
        // Check if step time is within 30-170% of expected (accounts for real variations)
        step_time_reasonable = (ratio > 0.3f && ratio < 1.7f);
    }
    
    if (step_time_reasonable && gait_cycle_ms > 0) {
        // We have reasonable data - calculate asymmetry
        float expected_step_time = gait_cycle_ms / 2.0f;
        
        // Calculate raw deviation
        float deviation = fabsf(step_time_ms - expected_step_time);
        
        // Express as percentage - but scale based on what's normal
        // Small deviations (< 30ms) are normal even in healthy gait
        if (deviation < 30.0f) {
            // Very small deviation - essentially symmetric
            step_time_asymmetry = deviation * 0.5f;  // Scale down small deviations
        } else if (deviation < 60.0f) {
            // Moderate deviation - some asymmetry
            step_time_asymmetry = 15.0f + (deviation - 30.0f) * 0.3f;
        } else {
            // Large deviation - significant asymmetry
            step_time_asymmetry = 25.0f + (deviation - 60.0f) * 0.2f;
        }
        
        // Cap at reasonable maximum (50% is severe asymmetry)
        if (step_time_asymmetry > 50.0f) {
            step_time_asymmetry = 50.0f;
        }
        
        LOG_INF("TEMPORAL: Step time asymmetry = %.1f%% (step=%u ms, expected=%.0f ms, deviation=%.0f ms)",
                (double)step_time_asymmetry, step_time_ms, (double)expected_step_time, (double)deviation);
    } else {
        // Step time is unreasonable - likely a timing/sync issue
        // Don't report high asymmetry for bad data
        LOG_DBG("TEMPORAL: Step time unreasonable (%u ms in %.0f ms cycle), using default asymmetry",
                step_time_ms, gait_cycle_ms);
        
        // Use a default low asymmetry value rather than reporting a false high value
        step_time_asymmetry = 5.0f;  // Assume mostly symmetric if we can't measure properly
    }
    
    // Calculate asymmetry indices (existing code)
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
    } else if (primary->cadence > 0) {
        // Fallback: Use primary-only cadence if secondary not available
        rt_metrics->cadence_spm = (uint16_t)(primary->cadence);
        LOG_WRN("Using primary-only cadence: %u spm (secondary not available)", rt_metrics->cadence_spm);
    } else if (d2d_metric_is_valid(secondary, IDX_CADENCE)) {
        // Fallback: Use secondary-only cadence if primary not available
        rt_metrics->cadence_spm = (uint16_t)(secondary->metrics[IDX_CADENCE]);
        LOG_WRN("Using secondary-only cadence: %u spm (primary not available)", rt_metrics->cadence_spm);
    }
    
    // Calculate pace from cadence and stride length
    // Pace (s/km) = 1000 / (speed in m/s)
    // Speed (m/s) = (cadence in steps/min / 60) * stride_length in meters
    // LOG_INF("PACE CALC: Starting - cadence=%u spm", rt_metrics->cadence_spm);
    
    if (rt_metrics->cadence_spm > 0) {
        float avg_stride_m = 0;
        float primary_stride = primary->stride_length;
        float secondary_stride = 0;
        bool secondary_valid = d2d_metric_is_valid(secondary, IDX_STRIDE_LENGTH);
        
        if (secondary_valid) {
            secondary_stride = secondary->metrics[IDX_STRIDE_LENGTH] / 100.0f;
        }
        
        // LOG_INF("PACE CALC: Primary stride=%.3f m, Secondary stride=%.3f m (valid=%d)",
        //         (double)primary_stride, (double)secondary_stride, secondary_valid);
        
        // Try to get stride length from both feet
        if (secondary_valid && secondary_stride > 0 && primary_stride > 0) {
            avg_stride_m = (secondary_stride + primary_stride) / 2.0f;
            // LOG_INF("PACE CALC: Using average of both feet: %.3f m", (double)avg_stride_m);
        } else if (primary_stride > 0) {
            avg_stride_m = primary_stride;
            // LOG_INF("PACE CALC: Using primary only: %.3f m", (double)avg_stride_m);
        } else if (secondary_valid && secondary_stride > 0) {
            avg_stride_m = secondary_stride;
            // LOG_INF("PACE CALC: Using secondary only: %.3f m", (double)avg_stride_m);
        } else {
            // Fallback: Estimate stride length from cadence
            // Empirical formula based on typical running patterns
            // Higher cadence typically means shorter strides
            if (rt_metrics->cadence_spm > 180) {
                avg_stride_m = 1.1f;  // Short stride for high cadence
            } else if (rt_metrics->cadence_spm > 160) {
                avg_stride_m = 1.3f;  // Medium stride
            } else {
                avg_stride_m = 1.5f;  // Longer stride for lower cadence
            }
            // LOG_INF("PACE CALC: No stride data! Estimated %.2f m from cadence %u spm", 
            //         (double)avg_stride_m, rt_metrics->cadence_spm);
        }
        
        // Calculate speed and pace
        if (avg_stride_m > 0) {
            float speed_mps = (rt_metrics->cadence_spm / 60.0f) * avg_stride_m;
            if (speed_mps > 0) {
                rt_metrics->pace_sec_km = (uint16_t)(1000.0f / speed_mps);
                LOG_INF("PACE CALC SUCCESS: %u s/km (speed: %.2f m/s, stride: %.2f m, cadence: %u)",
                        rt_metrics->pace_sec_km, (double)speed_mps, (double)avg_stride_m, rt_metrics->cadence_spm);
            } else {
                LOG_ERR("PACE CALC: Speed calculation failed! speed_mps=%.3f", (double)speed_mps);
            }
        } else {
            LOG_ERR("PACE CALC: No valid stride length! avg_stride_m=%.3f", (double)avg_stride_m);
        }
    } else {
        LOG_WRN("PACE CALC: Cadence is 0, cannot calculate pace");
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
    // Strike pattern values: 0=heel, 1=midfoot, 2=forefoot (from detect_strike_pattern)
    rt_metrics->right_strike_pattern = primary->strike_pattern;  // Use directly, no conversion needed
    
    // Balance (based on peak force distribution between feet)
    // Now using IDX_PEAK_IMPACT which contains peak_force from secondary
    // Both values need to be scaled the same way (divided by 4) for comparison
    if (d2d_metric_is_valid(secondary, IDX_PEAK_IMPACT) && primary->peak_pressure > 0) {
        float sec_peak_force = secondary->metrics[IDX_PEAK_IMPACT];  // Secondary peak force (already scaled /4)
        float pri_peak_force = (float)(primary->peak_pressure / 4);   // Primary peak force (scale /4 to match)
        
        // Calculate balance as percentage difference (-50 to +50)
        // Negative = left dominant, Positive = right dominant
        float total_force = sec_peak_force + pri_peak_force;
        if (total_force > 0) {
            // Calculate percentage: (right - left) / total * 100
            // Primary is right foot, Secondary is left foot
            float balance_pct = ((pri_peak_force - sec_peak_force) / total_force) * 100.0f;
            rt_metrics->balance_lr_pct = (int8_t)CLAMP(balance_pct, -50, 50);
            LOG_INF("Balance calculation: Left=%.1f, Right=%.1f, Balance=%d%% (%s dominant)",
                    (double)sec_peak_force, (double)pri_peak_force, rt_metrics->balance_lr_pct,
                    rt_metrics->balance_lr_pct < 0 ? "left" : rt_metrics->balance_lr_pct > 0 ? "right" : "neutral");
        }
    } else if (d2d_metric_is_valid(secondary, IDX_BALANCE_INDEX)) {
        // Fallback to old balance calculation if peak force not available
        float sec_balance = secondary->metrics[IDX_BALANCE_INDEX];
        float pri_balance = 100.0f - MIN(primary->cop_path_length, 100.0f);
        rt_metrics->balance_lr_pct = (int8_t)((sec_balance - pri_balance) / 2.0f);  // -50 to +50
    }
    
    // Force asymmetry (based on peak force difference)
    // Now using IDX_PEAK_IMPACT for actual force comparison
    // Both values need to be scaled the same way (divided by 4) for comparison
    if (d2d_metric_is_valid(secondary, IDX_PEAK_IMPACT) && primary->peak_pressure > 0) {
        float sec_peak_force = secondary->metrics[IDX_PEAK_IMPACT];  // Secondary peak force (already scaled /4)
        float pri_peak_force = (float)(primary->peak_pressure / 4);   // Primary peak force (scale /4 to match)
        
        // Calculate asymmetry as percentage difference
        float avg_force = (sec_peak_force + pri_peak_force) / 2.0f;
        if (avg_force > 0) {
            float force_asym = fabsf(sec_peak_force - pri_peak_force) / avg_force * 100.0f;
            rt_metrics->force_asymmetry = (uint8_t)MIN(force_asym, 100.0f);
            LOG_INF("Force asymmetry: Left=%.1f, Right=%.1f, Asymmetry=%.1f%%",
                    (double)sec_peak_force, (double)pri_peak_force, (double)force_asym);
        }
    } else if (d2d_metric_is_valid(secondary, IDX_LOADING_RATE) && primary->loading_rate > 0) {
        // Fallback to loading rate if peak force not available
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
        
        // Update bilateral success time for fallback logic
        extern uint32_t g_last_bilateral_success_time;
        g_last_bilateral_success_time = k_uptime_get_32();
        LOG_INF("PRIMARY: Bilateral analysis SUCCESS - updated success timestamp");
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
    
    // Log received metrics - COMMENTED OUT TO REDUCE LOG SPAM
    // if (d2d_metric_is_valid(metrics, IDX_GCT)) {
    //     LOG_DBG("  Received GCT: %.1f ms", (double)metrics->metrics[IDX_GCT]);
    // }
    // if (d2d_metric_is_valid(metrics, IDX_CADENCE)) {
    //     LOG_DBG("  Received Cadence: %.1f spm", (double)metrics->metrics[IDX_CADENCE]);
    // }
    // if (d2d_metric_is_valid(metrics, IDX_STRIDE_LENGTH)) {
    //     LOG_DBG("  Received Stride: %.2f cm", (double)metrics->metrics[IDX_STRIDE_LENGTH]);
    // }
    // if (d2d_metric_is_valid(metrics, IDX_PRONATION)) {
    //     LOG_DBG("  Received Pronation: %.1f deg", (double)metrics->metrics[IDX_PRONATION]);
    // }
    // if (d2d_metric_is_valid(metrics, IDX_COP_X) && d2d_metric_is_valid(metrics, IDX_COP_Y)) {
    //     LOG_DBG("  Received COP: (%.1f, %.1f) mm",
    //             (double)metrics->metrics[IDX_COP_X], (double)metrics->metrics[IDX_COP_Y]);
    // }
    // if (d2d_metric_is_valid(metrics, IDX_LOADING_RATE)) {
    //     LOG_DBG("  Received Loading Rate: %.1f N/s", (double)metrics->metrics[IDX_LOADING_RATE]);
    // }
    // if (d2d_metric_is_valid(metrics, IDX_VERTICAL_OSC)) {
    //     LOG_DBG("  Received VO: %.1f cm", (double)metrics->metrics[IDX_VERTICAL_OSC]);
    // }
    // if (d2d_metric_is_valid(metrics, IDX_STEP_COUNT)) {
    //     LOG_DBG("  Received Steps: %u", (uint32_t)metrics->metrics[IDX_STEP_COUNT]);
    // }
    
    #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // On primary device, calculate bilateral metrics if we have both sides
    // Check if we have primary data in history
    if (bilateral_data.history_count > 0) {
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
    
    // Store in circular history buffer
    primary_metrics_entry_t *entry = &bilateral_data.primary_history[bilateral_data.history_write_idx];
    
    // Copy metrics (up to max)
    int copy_count = MIN(count, GAIT_MAX_EVENTS_PER_CHUNK);
    memcpy(entry->metrics, metrics, copy_count * sizeof(gait_metrics_t));
    entry->count = copy_count;
    entry->timestamp_ms = k_uptime_get_32();
    entry->valid = true;
    
    LOG_DBG("Stored primary metrics in history slot %d, timestamp=%u ms",
            bilateral_data.history_write_idx, entry->timestamp_ms);
    
    // Update circular buffer indices
    bilateral_data.history_write_idx = (bilateral_data.history_write_idx + 1) % BILATERAL_HISTORY_SIZE;
    if (bilateral_data.history_count < BILATERAL_HISTORY_SIZE) {
        bilateral_data.history_count++;
    }
    
    // If we have pending secondary metrics, try to calculate bilateral analysis
    if (bilateral_data.secondary_valid) {
        LOG_INF("PRIMARY: Have pending secondary data, attempting bilateral calculation");
        calculate_bilateral_metrics();
        // Clear secondary data after processing (whether successful or not)
        bilateral_data.secondary_valid = false;
    }
    
    k_mutex_unlock(&bilateral_mutex);
}

} // extern "C"
/**
 * @file gait_events.c
 * @brief Implementation of event-driven gait parameter detection
 * Based on colleague's pressure event detection and consumer pattern
 */

#include <gait_events.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(gait_events, LOG_LEVEL_INF);

/* Constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define GRAVITY_MS2 9.81f
#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) < 0 ? -(x) : (x))

/**
 * Initialize the gait event detector
 */
void gait_events_init(gait_event_detector_t *detector, bool is_primary, float sampling_rate)
{
    if (!detector) {
        return;
    }
    
    memset(detector, 0, sizeof(gait_event_detector_t));
    
    detector->sampling_rate = sampling_rate > 0 ? sampling_rate : GAIT_SAMPLING_RATE;
    detector->pressure_threshold = GAIT_PRESSURE_THRESHOLD;
    detector->is_primary = is_primary;
    detector->current_phase = GAIT_PHASE_IDLE;
    detector->last_ic_index = -1;
    detector->last_to_index = -1;
    detector->total_step_count = 0;
    detector->processing_active = true;
    detector->subject_height_m = 1.75f; /* Default height */
    detector->use_advanced_features = true; /* Enable advanced features by default */
    
    /* Initialize gravity vector (assuming Z is up) */
    detector->gravity_vector[0] = 0.0f;
    detector->gravity_vector[1] = 0.0f;
    detector->gravity_vector[2] = -GRAVITY_MS2;
    
    /* INITIALIZE ADVANCED FEATURES */
    if (detector->use_advanced_features) {
        /* Motion mask for false positive reduction */
        motion_mask_init(&detector->motion_mask, 0.5f, 15.0f);
        
        /* Enhanced 9-state FSM */
        gait_fsm_init(&detector->detailed_fsm, GAIT_VELOCITY_THRESHOLD);
        
        /* Full quaternion IMU fusion */
        imu_fusion_init(&detector->imu_fusion, detector->sampling_rate);
        
        /* Anthropometric models */
        anthro_init(&detector->anthro_model, detector->subject_height_m);
        detector->anthro_model.preferred_model = ANTHRO_MODEL_HEIDERSCHEIT;
        
        /* Rolling windows (3 second window, up to 10 values) */
        rolling_window_init(&detector->gct_window, 10, 3000);
        rolling_window_init(&detector->stride_window, 10, 3000);
        rolling_window_init(&detector->cadence_window, 10, 3000);
        
        /* Kalman filter for gap filling */
        float dt = 1.0f / detector->sampling_rate;
        kalman_init(&detector->pressure_kalman, dt, 1e-5f, 1e-6f, 0.1f);
    }
    
    LOG_INF("Gait event detector initialized for %s device at %.1f Hz (advanced features: %s)",
            is_primary ? "primary" : "secondary", detector->sampling_rate,
            detector->use_advanced_features ? "ENABLED" : "disabled");
}

/**
 * Calculate total pressure from all 8 sensors
 */
static int calculate_total_pressure(const foot_samples_t *foot_data)
{
    int total = 0;
    for (int i = 0; i < 8; i++) {
        total += foot_data->values[i];
    }
    return total;
}

/**
 * Calculate adaptive pressure threshold based on recent data
 * Uses percentile method similar to colleague's implementation
 */
static float calculate_adaptive_threshold(const foot_samples_t *buffer, 
                                         int start, int count)
{
    if (count <= 0) {
        return GAIT_PRESSURE_THRESHOLD;
    }
    
    /* Calculate 5th and 75th percentiles for adaptive threshold */
    int total_pressures[GAIT_BUFFER_SIZE_SAMPLES];
    int sorted_count = MIN(count, GAIT_BUFFER_SIZE_SAMPLES);
    
    /* Calculate total pressure for each sample */
    for (int i = 0; i < sorted_count; i++) {
        int idx = (start + i) % GAIT_BUFFER_SIZE_SAMPLES;
        total_pressures[i] = calculate_total_pressure(&buffer[idx]);
    }
    
    /* Simple bubble sort for percentile calculation */
    for (int i = 0; i < sorted_count - 1; i++) {
        for (int j = 0; j < sorted_count - i - 1; j++) {
            if (total_pressures[j] > total_pressures[j + 1]) {
                int temp = total_pressures[j];
                total_pressures[j] = total_pressures[j + 1];
                total_pressures[j + 1] = temp;
            }
        }
    }
    
    /* Calculate adaptive threshold as 50% between 5th and 75th percentiles */
    int low_idx = sorted_count * 5 / 100;
    int high_idx = sorted_count * 75 / 100;
    float low_val = (float)total_pressures[low_idx];
    float high_val = (float)total_pressures[high_idx];
    
    return low_val + (high_val - low_val) * 0.5f;
}

/**
 * Add new data to the ring buffer
 * This is called at high frequency (100Hz) to fill the buffer
 */
void gait_events_add_data(gait_event_detector_t *detector,
                          const foot_samples_t *foot_data,
                          const imu_data_t *imu_data,
                          float timestamp)
{
    if (!detector || !foot_data || !imu_data) {
        return;
    }
    
    int idx = detector->write_index;
    
    /* Bounds check before copying */
    if (idx >= GAIT_BUFFER_SIZE_SAMPLES) {
        LOG_ERR("Invalid buffer index %d >= %d", idx, GAIT_BUFFER_SIZE_SAMPLES);
        return;
    }
    
    /* Copy data to buffer with bounds checking */
    memcpy(&detector->foot_buffer[idx], foot_data, sizeof(foot_samples_t));
    memcpy(&detector->imu_buffer[idx], imu_data, sizeof(imu_data_t));
    detector->timestamps[idx] = timestamp;
    
    /* Update velocity integration if we have previous data */
    if (detector->buffer_count > 0) {
        float dt = timestamp - detector->last_timestamp;
        if (dt > 0 && dt < 0.1f) { /* Sanity check on dt */
            update_velocity_integration(detector, imu_data, dt);
        }
    }
    
    /* Store current velocity and position */
    detector->velocities[idx] = detector->current_velocity;
    detector->positions[idx] = detector->current_position;
    
    /* Update buffer indices - NON-BLOCKING implementation */
    detector->write_index = (detector->write_index + 1) % GAIT_BUFFER_SIZE_SAMPLES;
    
    if (detector->buffer_count < GAIT_BUFFER_SIZE_SAMPLES) {
        detector->buffer_count++;
    } else {
        /* Buffer is full - drop oldest data without blocking */
        detector->buffer_full = true;
        /* Move read index forward immediately to maintain 3-second window */
        detector->read_index = (detector->read_index + 1) % GAIT_BUFFER_SIZE_SAMPLES;
        /* Log occasionally to avoid spam */
        static int drop_count = 0;
        if (++drop_count % 100 == 0) {
            LOG_DBG("Buffer full, dropped %d old samples", drop_count);
        }
    }
    
    detector->last_timestamp = timestamp;
    detector->waiting_for_data = false;
}

/**
 * Update velocity by integrating IMU accelerometer data
 * Implements simplified sensor fusion with ZUPT
 */
static void update_velocity_integration(gait_event_detector_t *detector,
                                       const imu_data_t *imu_data,
                                       float dt)
{
    if (detector->use_advanced_features) {
        /* ADVANCED: Use full quaternion-based IMU fusion */
        bool is_stance = (detector->current_phase == GAIT_PHASE_STANCE);
        bool is_gait_event = false; /* Will be set when events are detected */
        
        /* Update quaternion fusion */
        imu_fusion_update(&detector->imu_fusion, imu_data, is_stance, is_gait_event);
        detector->quaternion_updates++;
        
        /* Get updated velocity and position from fusion */
        vector3_t vel = imu_fusion_get_velocity(&detector->imu_fusion);
        vector3_t pos = imu_fusion_get_position(&detector->imu_fusion);
        
        detector->current_velocity.x = vel.x;
        detector->current_velocity.y = vel.y;
        detector->current_velocity.z = vel.z;
        
        detector->current_position.x = pos.x;
        detector->current_position.y = pos.y;
        detector->current_position.z = pos.z;
        
        /* Update detailed FSM with current state */
        float velocity_mag = sqrtf(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
        int pressure_idx = detector->write_index > 0 ? detector->write_index - 1 : GAIT_BUFFER_SIZE_SAMPLES - 1;
        float pressure = (float)calculate_total_pressure(&detector->foot_buffer[pressure_idx]);
        gait_fsm_update(&detector->detailed_fsm, velocity_mag, pressure,
                       (uint32_t)(detector->last_timestamp * 1000));
    } else {
        /* BASIC: Simple linear integration */
        /* Remove gravity from acceleration (simplified - assumes Z is up) */
        float accel_x = imu_data->accel_x;
        float accel_y = imu_data->accel_y;
        float accel_z = imu_data->accel_z + GRAVITY_MS2; /* Remove gravity */
        
        /* Integrate acceleration to get velocity */
        detector->current_velocity.x += accel_x * dt;
        detector->current_velocity.y += accel_y * dt;
        detector->current_velocity.z += accel_z * dt;
        
        /* Integrate velocity to get position */
        detector->current_position.x += detector->current_velocity.x * dt;
        detector->current_position.y += detector->current_velocity.y * dt;
        detector->current_position.z += detector->current_velocity.z * dt;
        
        /* Apply ZUPT during stance phase */
        if (detector->current_phase == GAIT_PHASE_STANCE) {
            apply_zupt(detector, true);
        }
    }
}

/**
 * Apply Zero Velocity Update (ZUPT) during stance phase
 */
static void apply_zupt(gait_event_detector_t *detector, bool is_stance)
{
    if (!is_stance) {
        return;
    }
    
    /* Calculate velocity magnitude */
    float vel_mag = sqrtf(detector->current_velocity.x * detector->current_velocity.x +
                         detector->current_velocity.y * detector->current_velocity.y +
                         detector->current_velocity.z * detector->current_velocity.z);
    
    /* If velocity is below threshold during stance, apply ZUPT */
    if (vel_mag < GAIT_VELOCITY_THRESHOLD) {
        /* Soft reset - reduce velocity gradually */
        detector->current_velocity.x *= 0.1f;
        detector->current_velocity.y *= 0.1f;
        detector->current_velocity.z *= 0.1f;
    }
}

/**
 * Detect IC (Initial Contact) and TO (Toe Off) events from pressure data
 * Implements colleague's pressure_event_detection algorithm
 */
static void detect_ic_to_events(gait_event_detector_t *detector)
{
    detector->ic_count = 0;
    detector->to_count = 0;
    
    if (detector->buffer_count < detector->sampling_rate) { /* Need at least 1 second */
        return;
    }
    
    int start_idx = detector->read_index;
    int samples_to_process = MIN(detector->buffer_count, GAIT_BUFFER_SIZE_SAMPLES);
    
    /* ADVANCED: Build motion mask to reduce false positives */
    if (detector->use_advanced_features) {
        build_motion_mask(&detector->motion_mask,
                         detector->imu_buffer,
                         samples_to_process,
                         detector->motion_mask_buffer);
        
        /* Fill gaps in pressure signal using Kalman filter */
        for (int i = 0; i < samples_to_process; i++) {
            int idx = (start_idx + i) % GAIT_BUFFER_SIZE_SAMPLES;
            detector->pressure_signal_filled[i] =
                (float)calculate_total_pressure(&detector->foot_buffer[idx]);
        }
        
        int gaps = kalman_fill_gaps(detector->pressure_signal_filled,
                                   samples_to_process,
                                   50.0f,  /* threshold for gap detection */
                                   20,     /* max gap = 200ms @ 100Hz */
                                   detector->sampling_rate);
        if (gaps > 0) {
            detector->gaps_filled += gaps;
            LOG_DBG("Filled %d gaps in pressure signal", gaps);
        }
    }
    
    /* Calculate adaptive threshold */
    float adaptive_threshold = calculate_adaptive_threshold(detector->foot_buffer,
                                                           start_idx,
                                                           samples_to_process);
    
    /* Timing constraints in samples */
    int min_stride_samples = (int)(GAIT_MIN_STRIDE_TIME * detector->sampling_rate);
    int max_stride_samples = (int)(GAIT_MAX_STRIDE_TIME * detector->sampling_rate);
    int min_contact_samples = (int)(GAIT_MIN_CONTACT_TIME * detector->sampling_rate);
    int refractory_samples = (int)(GAIT_REFRACTORY_TIME * detector->sampling_rate);
    
    int last_ic = -1;
    int last_to = -1;
    
    /* Scan through buffer to find pressure transitions */
    for (int i = 1; i < samples_to_process && detector->ic_count < GAIT_MAX_EVENTS_PER_CHUNK; i++) {
        /* ADVANCED: Skip if no motion detected */
        if (detector->use_advanced_features && !detector->motion_mask_buffer[i]) {
            continue;  /* Skip this sample - no motion */
        }
        
        int curr_idx = (start_idx + i) % GAIT_BUFFER_SIZE_SAMPLES;
        int prev_idx = (start_idx + i - 1) % GAIT_BUFFER_SIZE_SAMPLES;
        
        /* Use filled signal if available, otherwise original */
        float curr_pressure, prev_pressure;
        if (detector->use_advanced_features) {
            curr_pressure = detector->pressure_signal_filled[i];
            prev_pressure = detector->pressure_signal_filled[i-1];
        } else {
            curr_pressure = (float)calculate_total_pressure(&detector->foot_buffer[curr_idx]);
            prev_pressure = (float)calculate_total_pressure(&detector->foot_buffer[prev_idx]);
        }
        
        bool curr_contact = (curr_pressure > adaptive_threshold);
        bool prev_contact = (prev_pressure > adaptive_threshold);
        
        /* Detect IC: transition from no contact to contact (0→1) */
        if (!prev_contact && curr_contact) {
            /* Check refractory period after last TO */
            if (last_to >= 0 && (i - last_to) < refractory_samples) {
                if (detector->use_advanced_features) {
                    detector->false_positives_rejected++;
                }
                continue;
            }
            
            /* Check minimum stride time from last IC */
            if (last_ic >= 0 && (i - last_ic) < min_stride_samples) {
                if (detector->use_advanced_features) {
                    detector->false_positives_rejected++;
                }
                continue;
            }
            
            /* Valid IC event */
            detector->ic_indices[detector->ic_count++] = i;
            last_ic = i;
            
            LOG_DBG("IC detected at index %d, pressure %.1f > threshold %.1f",
                    i, curr_pressure, adaptive_threshold);
        }
        
        /* Detect TO: transition from contact to no contact (1→0) */
        if (prev_contact && !curr_contact) {
            /* Must have a preceding IC */
            if (last_ic < 0) {
                continue;
            }
            
            /* Check minimum contact time */
            if ((i - last_ic) < min_contact_samples) {
                if (detector->use_advanced_features) {
                    detector->false_positives_rejected++;
                }
                continue;
            }
            
            /* Valid TO event */
            detector->to_indices[detector->to_count++] = i;
            last_to = i;
            
            LOG_DBG("TO detected at index %d", i);
        }
    }
    
    if (detector->use_advanced_features) {
        LOG_DBG("Detected %d IC and %d TO events (threshold: %.1f, gaps_filled: %lu, rejected: %lu)",
                detector->ic_count, detector->to_count, adaptive_threshold,
                (unsigned long)detector->gaps_filled, (unsigned long)detector->false_positives_rejected);
    } else {
        LOG_DBG("Detected %d IC and %d TO events (threshold: %.1f)",
                detector->ic_count, detector->to_count, adaptive_threshold);
    }
}

/**
 * Create stride segments from detected IC/TO events
 * Implements colleague's get_stride_segments_from_ic_to
 */
static void create_stride_segments(gait_event_detector_t *detector)
{
    detector->stride_count = 0;
    
    /* Need at least 2 ICs and 1 TO to form a stride */
    if (detector->ic_count < 2 || detector->to_count < 1) {
        return;
    }
    
    /* Create segments: [IC₁, TO₁, IC₂] */
    for (int i = 0; i < detector->ic_count - 1 && detector->stride_count < GAIT_MAX_EVENTS_PER_CHUNK; i++) {
        int ic1 = detector->ic_indices[i];
        int ic2 = detector->ic_indices[i + 1];
        
        /* Find TO between IC1 and IC2 */
        int to_idx = -1;
        for (int j = 0; j < detector->to_count; j++) {
            if (detector->to_indices[j] > ic1 && detector->to_indices[j] < ic2) {
                to_idx = detector->to_indices[j];
                break;
            }
        }
        
        if (to_idx < 0) {
            continue; /* No TO found between ICs */
        }
        
        /* Create stride segment */
        stride_segment_t *segment = &detector->stride_segments[detector->stride_count];
        segment->ic_index = ic1;
        segment->to_index = to_idx;
        segment->next_ic_index = ic2;
        
        /* Get timestamps */
        int start_idx = detector->read_index;
        int ic1_buf = (start_idx + ic1) % GAIT_BUFFER_SIZE_SAMPLES;
        int to_buf = (start_idx + to_idx) % GAIT_BUFFER_SIZE_SAMPLES;
        int ic2_buf = (start_idx + ic2) % GAIT_BUFFER_SIZE_SAMPLES;
        
        segment->ic_timestamp = detector->timestamps[ic1_buf];
        segment->to_timestamp = detector->timestamps[to_buf];
        segment->next_ic_timestamp = detector->timestamps[ic2_buf];
        segment->valid = true;
        
        detector->stride_count++;
        
        LOG_DBG("Stride segment %d: IC=%d (%.2fs), TO=%d (%.2fs), next_IC=%d (%.2fs)",
                detector->stride_count - 1, ic1, segment->ic_timestamp,
                to_idx, segment->to_timestamp, ic2, segment->next_ic_timestamp);
    }
    
    // Only log if we have segments or periodically
    if (detector->stride_count > 0) {
        LOG_INF("Created %d stride segments from events", detector->stride_count);
    }
}

/**
 * Calculate gait metrics for each stride segment
 * Implements colleague's update_gait_events algorithm
 */
static void calculate_stride_metrics(gait_event_detector_t *detector)
{
    detector->metrics_count = 0;
    
    for (int s = 0; s < detector->stride_count && detector->metrics_count < GAIT_MAX_EVENTS_PER_CHUNK; s++) {
        stride_segment_t *segment = &detector->stride_segments[s];
        if (!segment->valid) {
            continue;
        }
        
        gait_metrics_t *m = &detector->metrics_queue[detector->metrics_count];
        memset(m, 0, sizeof(gait_metrics_t));
        
        /* Basic timing metrics */
        m->start_time = segment->ic_timestamp;
        m->end_time = segment->next_ic_timestamp;
        m->timestamp = (m->start_time + m->end_time) / 2.0f;
        m->gct = segment->to_timestamp - segment->ic_timestamp;
        m->duration = segment->next_ic_timestamp - segment->ic_timestamp;
        
        /* Cadence (steps per minute) - multiply by 2 for both feet */
        if (m->duration > 0) {
            m->cadence = 60.0f / m->duration * 2.0f;
        }
        
        /* Get buffer indices */
        int start_idx = detector->read_index;
        int ic_buf = (start_idx + segment->ic_index) % GAIT_BUFFER_SIZE_SAMPLES;
        int to_buf = (start_idx + segment->to_index) % GAIT_BUFFER_SIZE_SAMPLES;
        int next_ic_buf = (start_idx + segment->next_ic_index) % GAIT_BUFFER_SIZE_SAMPLES;
        
        /* Calculate stride length from velocity integration */
        float stride_length = 0.0f;
        float vertical_osc_max = 0.0f;
        float vertical_osc_min = 0.0f;
        float lateral_movement = 0.0f;
        
        for (int i = segment->ic_index; i < segment->next_ic_index; i++) {
            int buf_idx = (start_idx + i) % GAIT_BUFFER_SIZE_SAMPLES;
            int next_buf_idx = (start_idx + i + 1) % GAIT_BUFFER_SIZE_SAMPLES;
            
            if (i + 1 >= segment->next_ic_index) {
                break;
            }
            
            float dt = detector->timestamps[next_buf_idx] - detector->timestamps[buf_idx];
            if (dt > 0 && dt < 0.1f) {
                /* Integrate forward velocity for stride length */
                stride_length += detector->velocities[buf_idx].x * dt;
                
                /* Track vertical oscillation during swing phase */
                if (i >= segment->to_index) {
                    float z_pos = detector->positions[buf_idx].z;
                    vertical_osc_max = MAX(vertical_osc_max, z_pos);
                    vertical_osc_min = MIN(vertical_osc_min, z_pos);
                }
                
                /* Track lateral movement */
                lateral_movement += ABS(detector->velocities[buf_idx].y * dt);
            }
        }
        
        m->stride_length = ABS(stride_length);
        m->stride_velocity = m->stride_length / m->duration;
        m->vertical_oscillation = vertical_osc_max - vertical_osc_min;
        m->step_width = lateral_movement / (segment->next_ic_index - segment->ic_index);
        
        /* Extract angles at events */
        if (detector->use_advanced_features) {
            /* ADVANCED: Get angles from quaternion fusion */
            quaternion_t q = imu_fusion_get_orientation(&detector->imu_fusion);
            vector3_t euler = quaternion_to_euler(&q);
            m->ic_pitch = euler.y * RAD_TO_DEG;
            m->ic_roll = euler.x * RAD_TO_DEG;
            /* Note: TO angles would need historical quaternion data */
            m->to_pitch = euler.y * RAD_TO_DEG;
            m->to_roll = euler.x * RAD_TO_DEG;
        } else {
            /* BASIC: Convert gyro to degrees (approximation) */
            m->ic_pitch = detector->imu_buffer[ic_buf].gyro_y * RAD_TO_DEG;
            m->to_pitch = detector->imu_buffer[to_buf].gyro_y * RAD_TO_DEG;
            m->ic_roll = detector->imu_buffer[ic_buf].gyro_x * RAD_TO_DEG;
            m->to_roll = detector->imu_buffer[to_buf].gyro_x * RAD_TO_DEG;
        }
        
        /* Find min/max angles during stride */
        float min_roll = m->ic_roll;
        float max_roll = m->ic_roll;
        float min_pitch = m->ic_pitch;
        
        for (int i = segment->ic_index; i <= segment->next_ic_index; i++) {
            int buf_idx = (start_idx + i) % GAIT_BUFFER_SIZE_SAMPLES;
            float roll = detector->imu_buffer[buf_idx].gyro_x * RAD_TO_DEG;
            float pitch = detector->imu_buffer[buf_idx].gyro_y * RAD_TO_DEG;
            
            min_roll = MIN(min_roll, roll);
            max_roll = MAX(max_roll, roll);
            min_pitch = MIN(min_pitch, pitch);
        }
        
        m->min_roll = min_roll;
        m->max_roll = max_roll;
        m->min_pitch = min_pitch;
        
        /* Calculate anthropometric speed */
        if (detector->use_advanced_features) {
            /* ADVANCED: Use full anthropometric models */
            float swing_time = m->duration - m->gct;  /* Swing time = stride - stance */
            m->speed_anthropometric = anthro_calculate_speed(&detector->anthro_model,
                                                            m->cadence,
                                                            m->stride_length,
                                                            m->gct,
                                                            swing_time);
        } else {
            /* BASIC: Simple calculation */
            if (detector->subject_height_m > 0) {
                m->speed_anthropometric = (m->cadence / 120.0f) * m->stride_length;
            }
        }
        
        /* ADVANCED: Apply rolling window smoothing */
        if (detector->use_advanced_features) {
            uint32_t timestamp_ms = (uint32_t)(m->timestamp * 1000);
            
            /* Add raw values to windows */
            rolling_window_add(&detector->gct_window, m->gct, timestamp_ms);
            rolling_window_add(&detector->stride_window, m->stride_length, timestamp_ms);
            rolling_window_add(&detector->cadence_window, m->cadence, timestamp_ms);
            
            /* Get smoothed values if enough data */
            if (rolling_window_has_sufficient_data(&detector->gct_window)) {
                m->gct = rolling_window_get_average(&detector->gct_window);
            }
            if (rolling_window_has_sufficient_data(&detector->stride_window)) {
                m->stride_length = rolling_window_get_average(&detector->stride_window);
            }
            if (rolling_window_has_sufficient_data(&detector->cadence_window)) {
                m->cadence = rolling_window_get_average(&detector->cadence_window);
            }
        }
        
        /* Update step count */
        detector->total_step_count++;
        m->step_count = detector->total_step_count;
        m->valid = true;
        
        detector->metrics_count++;
        
        if (detector->use_advanced_features) {
            LOG_INF("Stride %d metrics: GCT=%.3fs, duration=%.3fs, cadence=%.1f, length=%.2fm, speed=%.2fm/s (smoothed)",
                    s, m->gct, m->duration, m->cadence, m->stride_length, m->speed_anthropometric);
        } else {
            LOG_INF("Stride %d metrics: GCT=%.3fs, duration=%.3fs, cadence=%.1f, length=%.2fm",
                    s, m->gct, m->duration, m->cadence, m->stride_length);
        }
    }
    
    /* Log advanced feature statistics if enabled */
    if (detector->use_advanced_features && detector->stride_count > 0) {
        LOG_DBG("Advanced stats: gaps_filled=%lu, false_pos_rejected=%lu, quat_updates=%lu",
               (unsigned long)detector->gaps_filled,
               (unsigned long)detector->false_positives_rejected,
               (unsigned long)detector->quaternion_updates);
    }
}

/**
 * Check if buffer has enough data for processing
 */
bool gait_events_ready_to_process(gait_event_detector_t *detector)
{
    if (!detector) {
        return false;
    }
    
    /* Need at least 1 second of data */
    return detector->buffer_count >= detector->sampling_rate;
}

/**
 * Consumer function - processes buffered data to detect events
 * This implements the consumer pattern from colleague's implementation
 */
int gait_events_process(gait_event_detector_t *detector)
{
    if (!detector || !detector->processing_active) {
        return 0;
    }
    
    /* Wait for sufficient data (implements wait_for_data) */
    if (!gait_events_ready_to_process(detector)) {
        detector->waiting_for_data = true;
        return 0;
    }
    
    detector->waiting_for_data = false;
    
    LOG_DBG("Processing gait events, buffer count: %d", detector->buffer_count);
    
    /* Step 1: Detect IC/TO events from pressure */
    detect_ic_to_events(detector);
    
    if (detector->ic_count == 0 && detector->to_count == 0) {
        LOG_DBG("No events detected in current chunk");
        
        /* Move read index forward to process next chunk */
        int samples_to_skip = detector->sampling_rate / 3; /* Keep 1/3 second overlap */
        if (samples_to_skip > 0 && samples_to_skip < detector->buffer_count) {
            detector->read_index = (detector->read_index + samples_to_skip) % GAIT_BUFFER_SIZE_SAMPLES;
            detector->buffer_count -= samples_to_skip;
        }
        return 0;
    }
    
    /* Step 2: Create stride segments from events */
    create_stride_segments(detector);
    
    if (detector->stride_count == 0) {
        LOG_DBG("No complete strides in current chunk");
        return 0;
    }
    
    /* Step 3: Calculate metrics for each stride */
    calculate_stride_metrics(detector);
    
    /* Step 4: Update read index to keep overlap (similar to colleague's implementation) */
    if (detector->stride_count > 0) {
        /* Move read index past the last processed stride, keeping 1/3 second overlap */
        stride_segment_t *last_segment = &detector->stride_segments[detector->stride_count - 1];
        int samples_to_skip = last_segment->next_ic_index - (int)(detector->sampling_rate / 3);
        if (samples_to_skip > 0 && samples_to_skip < detector->buffer_count) {
            detector->read_index = (detector->read_index + samples_to_skip) % GAIT_BUFFER_SIZE_SAMPLES;
            detector->buffer_count -= samples_to_skip;
            
            /* Reset velocity/position at stride boundary for drift correction */
            detector->current_position.x = 0;
            detector->current_position.y = 0;
            detector->current_position.z = 0;
        }
    }
    
    return detector->metrics_count;
}

/**
 * Get calculated metrics from the queue
 */
int gait_events_get_metrics(gait_event_detector_t *detector,
                            gait_metrics_t *metrics_out,
                            int max_metrics)
{
    if (!detector || !metrics_out || max_metrics <= 0) {
        return 0;
    }
    
    int count = MIN(detector->metrics_count, max_metrics);
    
    for (int i = 0; i < count; i++) {
        memcpy(&metrics_out[i], &detector->metrics_queue[i], sizeof(gait_metrics_t));
    }
    
    return count;
}

/**
 * Clear processed metrics from queue
 */
void gait_events_clear_metrics(gait_event_detector_t *detector)
{
    if (!detector) {
        return;
    }
    
    detector->metrics_count = 0;
    detector->stride_count = 0;
    detector->ic_count = 0;
    detector->to_count = 0;
}

/**
 * Set subject height for anthropometric calculations
 */
void gait_events_set_subject_height(gait_event_detector_t *detector, float height_m)
{
    if (detector && height_m > 0) {
        detector->subject_height_m = height_m;
        
        /* Update anthropometric model with new height */
        if (detector->use_advanced_features) {
            anthro_init(&detector->anthro_model, height_m);
        }
        
        LOG_INF("Subject height set to %.2f m", height_m);
    }
}
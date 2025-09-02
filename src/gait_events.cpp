/**
 * @file gait_events.c
 * @brief Implementation of event-driven gait parameter detection
 * Based on    pressure event detection and consumer pattern
 */

#include <gait_events.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

/* Include sensor data processing headers for algorithm functions */
#include "sensor_data/sensor_data_enhanced_algorithms.hpp"
#include "sensor_data/sensor_data_fast_processing.hpp"

LOG_MODULE_REGISTER(gait_events, LOG_LEVEL_INF);

/* Constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define GRAVITY_MS2 9.81f
#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)
/* MIN/MAX already defined in zephyr/sys/util.h */
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
 * Uses percentile method similar to    implementation
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
    
    /* ADAPTIVE THRESHOLD FOR CLEAR DETECTION */
    /* With 8 sensors, each reading 0-16383 (14-bit ADC): */
    /* - Theoretical max: 8 × 16383 = 131,064 */
    /* - Swing phase (foot in air): Total ~0-1000 (noise only) */
    /* - Light contact: Total ~5000-20000 */
    /* - Normal stance phase: Total ~20000-80000 */
    /* - Heavy pressure: Total ~80000-131064 */
    
    /* Find the minimum and maximum values in the buffer */
    int min_pressure = total_pressures[0];
    int max_pressure = total_pressures[sorted_count - 1];
    
    /* Calculate the range */
    int pressure_range = max_pressure - min_pressure;
    
    /* Set threshold adaptively based on the pressure range */
    /* This ensures we detect transitions between swing and stance phases */
    float threshold;
    if (pressure_range > 10000) {
        /* Clear difference between swing and stance - use 30% above minimum */
        /* This works well for normal walking/running patterns */
        threshold = (float)min_pressure + (float)pressure_range * 0.3f;
    } else if (pressure_range > 5000) {
        /* Moderate difference - use 25% above minimum */
        /* This handles lighter steps or transitions */
        threshold = (float)min_pressure + (float)pressure_range * 0.25f;
    } else {
        /* Small range - might be all swing or all stance */
        /* Use 20% above minimum but ensure minimum threshold */
        threshold = (float)min_pressure + (float)pressure_range * 0.2f;
        if (threshold < 1000.0f && max_pressure > 2000) {
            /* If max is significant, set threshold to detect transitions */
            threshold = 1000.0f;
        }
    }
    
    /* Sanity checks for the threshold */
    /* Must be high enough to avoid noise but low enough to detect contact */
    if (threshold < 500.0f) {
        /* Too low - would trigger on noise */
        threshold = 500.0f;
        LOG_DBG("Threshold too low, raised to %.1f", threshold);
    } else if (threshold > 50000.0f) {
        /* Extremely high - likely an error, cap at reasonable maximum */
        /* Most real-world scenarios won't exceed 50k total pressure */
        threshold = 20000.0f;
        LOG_WRN("Threshold %.1f extremely high, capped at 20000", threshold);
    }
    
    /* Log the pressure range for debugging */
    static int range_log_counter = 0;
    if (++range_log_counter % 20 == 0) {  // Log every 20th call
        LOG_INF("Pressure range: min=%d, max=%d, threshold=%.1f", 
                min_pressure, max_pressure, threshold);
    }
    
    /* Also ensure minimum threshold */
    if (threshold < GAIT_PRESSURE_THRESHOLD) {
        threshold = GAIT_PRESSURE_THRESHOLD;  /* Use default minimum */
    }
    
    return threshold;
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
    if (!detector || !foot_data) {
        return;  // Only foot_data is required, IMU is optional
    }
    
    /* Validate detector state before accessing */
    if (!detector->processing_active) {
        return;  // Don't add data if not processing
    }
    
    int idx = detector->write_index;
    
    /* Bounds check before copying */
    if (idx >= GAIT_BUFFER_SIZE_SAMPLES) {
        LOG_ERR("Invalid buffer index %d >= %d", idx, GAIT_BUFFER_SIZE_SAMPLES);
        return;
    }
    
    /* Don't validate foot_data values - they might all be 0 during initialization */
    /* The system should handle 0 values gracefully */
    
    /* Copy data to buffer with bounds checking */
    memcpy(&detector->foot_buffer[idx], foot_data, sizeof(foot_samples_t));
    
    /* Only copy IMU data if available and valid */
    if (imu_data) {
        /* Basic validation of IMU data */
        bool valid_imu = (fabsf(imu_data->accel_x) < 100.0f &&
                          fabsf(imu_data->accel_y) < 100.0f &&
                          fabsf(imu_data->accel_z) < 100.0f &&
                          fabsf(imu_data->gyro_x) < 10.0f &&
                          fabsf(imu_data->gyro_y) < 10.0f &&
                          fabsf(imu_data->gyro_z) < 10.0f);
        
        if (valid_imu) {
            memcpy(&detector->imu_buffer[idx], imu_data, sizeof(imu_data_t));
            
            /* Update velocity integration if we have previous data and IMU */
            if (detector->buffer_count > 0) {
                float dt = timestamp - detector->last_timestamp;
                if (dt > 0 && dt < 0.1f) { /* Sanity check on dt */
                    update_velocity_integration(detector, imu_data, dt);
                }
            }
        } else {
            /* Invalid IMU data - zero out */
            memset(&detector->imu_buffer[idx], 0, sizeof(imu_data_t));
        }
    } else {
        /* No IMU data - zero out the buffer entry */
        memset(&detector->imu_buffer[idx], 0, sizeof(imu_data_t));
    }
    
    detector->timestamps[idx] = timestamp;
    
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
 * @brief Detect swing phase from IMU data
 */
static bool detect_swing_phase_imu(const imu_data_t *imu, bool was_in_swing)
{
    // Swing phase detected by angular velocity and acceleration patterns
    float gyro_magnitude = sqrtf(imu->gyro_x * imu->gyro_x +
                                 imu->gyro_y * imu->gyro_y +
                                 imu->gyro_z * imu->gyro_z);
    
    float accel_magnitude = sqrtf(imu->accel_x * imu->accel_x +
                                  imu->accel_y * imu->accel_y +
                                  imu->accel_z * imu->accel_z);
    
    // Thresholds for swing detection
    const float SWING_GYRO_THRESHOLD = 2.0f;  // rad/s
    const float SWING_ACCEL_MIN = 8.0f;       // m/s^2
    const float SWING_ACCEL_MAX = 15.0f;      // m/s^2
    
    // Detect swing: moderate acceleration with controlled rotation
    bool in_swing = (gyro_magnitude < SWING_GYRO_THRESHOLD) &&
                    (accel_magnitude > SWING_ACCEL_MIN) &&
                    (accel_magnitude < SWING_ACCEL_MAX);
    
    // Apply hysteresis to prevent oscillation
    if (was_in_swing && !in_swing) {
        // Exiting swing - need stronger signal
        return (gyro_magnitude < SWING_GYRO_THRESHOLD * 0.8f);
    }
    
    return in_swing;
}

/**
 * @brief Detect turns from gyroscope data
 */
static void detect_turn_event(gait_event_detector_t *detector, const imu_data_t *imu, float dt)
{
    static float yaw_angle = 0.0f;
    static uint32_t turn_start_time = 0;
    static bool in_turn = false;
    
    // Integrate yaw rate to get heading change
    yaw_angle += imu->gyro_z * dt;
    
    // Detect significant yaw rate indicating turn
    const float TURN_RATE_THRESHOLD = 0.5f;  // rad/s (~30 deg/s)
    const float TURN_ANGLE_THRESHOLD = 0.785f;  // radians (~45 degrees)
    
    if (fabsf(imu->gyro_z) > TURN_RATE_THRESHOLD) {
        if (!in_turn) {
            in_turn = true;
            turn_start_time = k_uptime_get_32();  // Use system time
            yaw_angle = 0.0f;  // Reset integration
            LOG_DBG("Turn started at time %u ms", turn_start_time);
        }
    } else if (in_turn) {
        // Turn ended - check total angle
        if (fabsf(yaw_angle) > TURN_ANGLE_THRESHOLD) {
            float turn_duration = (float)(k_uptime_get_32() - turn_start_time);  // ms
            float turn_angle_deg = yaw_angle * (float)RAD_TO_DEG;
            LOG_INF("Turn completed: %.1f degrees in %.1f ms",
                    (double)turn_angle_deg, (double)turn_duration);
        }
        in_turn = false;
    }
}

/**
 * @brief Detect steps from accelerometer when pressure not available
 */
static void detect_step_from_imu(gait_event_detector_t *detector, const imu_data_t *imu)
{
    static float accel_history[10] = {0};
    static uint8_t accel_idx = 0;
    static uint32_t last_step_time = 0;
    
    // Store acceleration magnitude in circular buffer
    float accel_mag = sqrtf(imu->accel_x * imu->accel_x +
                            imu->accel_y * imu->accel_y +
                            imu->accel_z * imu->accel_z);
    accel_history[accel_idx] = accel_mag;
    accel_idx = (accel_idx + 1) % 10;
    
    uint32_t current_time = k_uptime_get_32();
    
    // Simple peak detection for steps (backup when no pressure)
    if (current_time - last_step_time > 300) {  // Min 300ms between steps
        float avg = 0.0f;
        for (int i = 0; i < 10; i++) {
            avg += accel_history[i];
        }
        avg /= 10.0f;
        
        // Current value significantly above average indicates step
        if (accel_mag > avg * 1.2f && accel_mag > 11.0f) {
            last_step_time = current_time;
            detector->total_step_count++;
            LOG_DBG("IMU step detected at time %u ms (total: %u)",
                    current_time, detector->total_step_count);
        }
    }
}

/**
 * Update velocity by integrating IMU accelerometer data
 * Implements simplified sensor fusion with ZUPT and IMU event detection
 */
static void update_velocity_integration(gait_event_detector_t *detector,
                                       const imu_data_t *imu_data,
                                       float dt)
{
    // IMU-only event detection
    static bool was_in_swing = false;
    static uint32_t last_log_time = 0;
    uint32_t current_time = k_uptime_get_32();
    
    // 1. Swing phase detection from IMU
    bool in_swing = detect_swing_phase_imu(imu_data, was_in_swing);
    if (in_swing != was_in_swing) {
        if (in_swing) {
            LOG_DBG("IMU swing phase started at time %u ms", current_time);
        } else {
            LOG_DBG("IMU swing phase ended at time %u ms", current_time);
        }
        was_in_swing = in_swing;
    }
    
    // 2. Turn detection
    detect_turn_event(detector, imu_data, dt);
    
    // 3. Step detection from IMU (backup for when pressure unavailable)
    detect_step_from_imu(detector, imu_data);
    
    // 4. Vertical oscillation tracking through velocity integration
    static float vertical_velocity = 0.0f;
    static float vertical_position = 0.0f;
    static float max_height = 0.0f;
    static float min_height = 0.0f;
    
    // Get vertical acceleration (simplified - assuming Z is vertical)
    float world_accel_z = imu_data->accel_z - GRAVITY_MS2;
    
    // Integrate to get velocity
    vertical_velocity += world_accel_z * dt;
    
    // Apply damping to reduce drift
    vertical_velocity *= 0.99f;
    
    // Integrate to get position
    vertical_position += vertical_velocity * dt;
    
    // Track max/min for vertical oscillation
    if (vertical_position > max_height) max_height = vertical_position;
    if (vertical_position < min_height) min_height = vertical_position;
    
    // Calculate vertical oscillation in cm
    float vertical_osc_cm = (max_height - min_height) * 100.0f;
    
    // Reset on new stride (check current phase)
    static gait_phase_t last_phase = GAIT_PHASE_IDLE;
    if (detector->current_phase == GAIT_PHASE_STANCE) {
        // Log vertical oscillation periodically
        if (current_time - last_log_time > 1000) {  // Every second
            LOG_DBG("Vertical oscillation: %.1f cm", (double)vertical_osc_cm);
            last_log_time = current_time;
        }
        
        // Reset tracking on phase change to stance
        if (detector->current_phase != last_phase) {
            max_height = vertical_position;
            min_height = vertical_position;
        }
    }
    last_phase = detector->current_phase;
    
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
        /* Validate pressure_idx before using it */
        if (pressure_idx >= 0 && pressure_idx < GAIT_BUFFER_SIZE_SAMPLES) {
            float pressure = (float)calculate_total_pressure(&detector->foot_buffer[pressure_idx]);
            gait_fsm_update(&detector->detailed_fsm, velocity_mag, pressure,
                           (uint32_t)(detector->last_timestamp * 1000));
        }
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
 * Implements    pressure_event_detection algorithm
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
    /* Check if we actually have IMU data before using advanced features */
    bool has_imu_data = false;
    if (detector->use_advanced_features) {
        /* Check if IMU buffer has any non-zero data */
        for (int i = 0; i < MIN(10, samples_to_process); i++) {
            int idx = (start_idx + i) % GAIT_BUFFER_SIZE_SAMPLES;
            imu_data_t *imu = &detector->imu_buffer[idx];
            if (imu->accel_x != 0 || imu->accel_y != 0 || imu->accel_z != 0 ||
                imu->gyro_x != 0 || imu->gyro_y != 0 || imu->gyro_z != 0) {
                has_imu_data = true;
                break;
            }
        }
        
        if (has_imu_data) {
            build_motion_mask(&detector->motion_mask,
                             detector->imu_buffer,
                             samples_to_process,
                             detector->motion_mask_buffer);
        } else {
            /* No IMU data - set all motion mask to true (allow all samples) */
            for (int i = 0; i < samples_to_process; i++) {
                detector->motion_mask_buffer[i] = true;
            }
            LOG_DBG("No IMU data detected - motion mask disabled");
        }
        
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
    
    /* Log threshold for debugging */
    static int threshold_log_counter = 0;
    if (++threshold_log_counter % 10 == 0) {  // Log every 10th call
        LOG_INF("Adaptive threshold: %.1f (processing %d samples)", 
                adaptive_threshold, samples_to_process);
    }
    
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
 * Implements    get_stride_segments_from_ic_to
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
                detector->stride_count - 1, ic1, (double)segment->ic_timestamp,
                to_idx, (double)segment->to_timestamp, ic2, (double)segment->next_ic_timestamp);
    }
    
    // Only log if we have segments or periodically
    if (detector->stride_count > 0) {
        LOG_INF("Created %d stride segments from events", detector->stride_count);
    }
}

/**
 * Calculate gait metrics for each stride segment
 * Implements    update_gait_events algorithm
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
        
        /* EVENT-BASED CALCULATIONS AT INITIAL CONTACT (IC) */
        foot_samples_t *ic_foot = &detector->foot_buffer[ic_buf];
        imu_data_t *ic_imu = &detector->imu_buffer[ic_buf];
        
        /* Calculate strike pattern at IC */
        uint8_t heel_pct, mid_pct, fore_pct;
        calculate_pressure_distribution(ic_foot->values, &heel_pct, &mid_pct, &fore_pct);
        m->strike_pattern = detect_strike_pattern(heel_pct, mid_pct, fore_pct);
        
        /* Calculate pronation at IC */
        float quat[4] = {ic_imu->quat_w, ic_imu->quat_x, ic_imu->quat_y, ic_imu->quat_z};
        m->pronation_at_ic = calculate_pronation_enhanced(quat, ic_foot->values, true, !detector->is_primary);
        
        /* Calculate COP at IC */
        calculate_center_of_pressure(ic_foot->values, &m->cop_x_at_ic, &m->cop_y_at_ic, !detector->is_primary);
        
        /* EVENT-BASED CALCULATIONS AT TOE-OFF (TO) */
        foot_samples_t *to_foot = &detector->foot_buffer[to_buf];
        
        /* Calculate COP at TO */
        calculate_center_of_pressure(to_foot->values, &m->cop_x_at_to, &m->cop_y_at_to, !detector->is_primary);
        
        /* STANCE PHASE CALCULATIONS (IC to TO) */
        m->max_pronation = m->pronation_at_ic;
        m->peak_pressure = 0;
        float cop_path_length = 0.0f;
        int16_t prev_cop_x = m->cop_x_at_ic;
        int16_t prev_cop_y = m->cop_y_at_ic;
        uint16_t prev_force = 0;
        uint16_t max_loading_rate = 0;
        contact_phase_t current_phase = PHASE_LOADING;
        
        /* Process each sample during stance phase */
        for (int i = segment->ic_index; i <= segment->to_index; i++) {
            int buf_idx = (start_idx + i) % GAIT_BUFFER_SIZE_SAMPLES;
            foot_samples_t *foot = &detector->foot_buffer[buf_idx];
            imu_data_t *imu = &detector->imu_buffer[buf_idx];
            
            /* Track peak pressure */
            uint16_t total_pressure = 0;
            for (int j = 0; j < 8; j++) {
                total_pressure += foot->values[j];
            }
            if (total_pressure > m->peak_pressure) {
                m->peak_pressure = total_pressure;
            }
            
            /* Track maximum pronation during stance */
            float quat_stance[4] = {imu->quat_w, imu->quat_x, imu->quat_y, imu->quat_z};
            int8_t pronation = calculate_pronation_enhanced(quat_stance, foot->values, true, !detector->is_primary);
            if (ABS(pronation) > ABS(m->max_pronation)) {
                m->max_pronation = pronation;
            }
            
            /* Calculate COP path length */
            int16_t cop_x, cop_y;
            calculate_center_of_pressure(foot->values, &cop_x, &cop_y, !detector->is_primary);
            float dx = (float)(cop_x - prev_cop_x);
            float dy = (float)(cop_y - prev_cop_y);
            cop_path_length += sqrtf(dx * dx + dy * dy);
            prev_cop_x = cop_x;
            prev_cop_y = cop_y;
            
            /* Update contact phase */
            bool was_in_contact = (i > segment->ic_index);
            current_phase = detect_contact_phase(foot->values, was_in_contact);
            
            /* Calculate loading rate during loading phase */
            if (current_phase == PHASE_LOADING && i > segment->ic_index) {
                uint16_t current_force = detect_peak_force(foot->values);
                if (prev_force > 0) {
                    float dt = detector->timestamps[buf_idx] - detector->timestamps[(start_idx + i - 1) % GAIT_BUFFER_SIZE_SAMPLES];
                    if (dt > 0 && dt < 0.1f) {
                        uint16_t loading_rate = calculate_loading_rate(current_force, prev_force, (uint16_t)(dt * 1000));
                        if (loading_rate > max_loading_rate) {
                            max_loading_rate = loading_rate;
                        }
                    }
                }
                prev_force = current_force;
            }
            
            /* Detect arch collapse during midstance */
            if (current_phase == PHASE_MIDSTANCE) {
                uint8_t arch_collapse = detect_arch_collapse(foot->values, current_phase);
                if (arch_collapse > m->arch_collapse_index) {
                    m->arch_collapse_index = arch_collapse;
                }
            }
            
            /* Calculate CPEI and push-off power during appropriate phases */
            if (current_phase == PHASE_MIDSTANCE || current_phase == PHASE_PUSH_OFF) {
                /* TODO: Implement calculate_cpei when function is available
                m->cpei = calculate_cpei(foot->values, cop_x, cop_y);
                */
                m->cpei = 0;  // Placeholder until function is implemented
                
                if (current_phase == PHASE_PUSH_OFF) {
                    uint16_t force = detect_peak_force(foot->values);
                    /* TODO: Implement calculate_push_off_power when function is available
                    m->push_off_power = calculate_push_off_power(force, imu->gyro_y, 12);
                    */
                    m->push_off_power = force / 10;  // Simple placeholder calculation
                }
            }
        }
        
        m->cop_path_length = cop_path_length;
        m->loading_rate = max_loading_rate;
        
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
 * This implements the consumer pattern from    implementation
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
        /* Clear more aggressively when no events detected to prevent overflow */
        int samples_to_skip = detector->sampling_rate / 2; /* Clear half second of data */
        if (samples_to_skip > 0 && samples_to_skip < detector->buffer_count) {
            detector->read_index = (detector->read_index + samples_to_skip) % GAIT_BUFFER_SIZE_SAMPLES;
            detector->buffer_count -= samples_to_skip;
            LOG_DBG("No events: cleared %d samples, %d remain", samples_to_skip, detector->buffer_count);
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
    
    /* Step 4: Update read index to keep minimal overlap for continuous processing */
    if (detector->stride_count > 0) {
        /* Move read index past the last processed stride, keeping only 0.2 second overlap */
        /* This prevents buffer overflow while maintaining continuity */
        stride_segment_t *last_segment = &detector->stride_segments[detector->stride_count - 1];
        int overlap_samples = (int)(detector->sampling_rate * 0.2f);  /* 20 samples at 100Hz */
        int samples_to_skip = last_segment->next_ic_index - overlap_samples;
        
        /* Ensure we clear at least 50% of processed data to prevent overflow */
        int min_clear = last_segment->next_ic_index / 2;
        if (samples_to_skip < min_clear) {
            samples_to_skip = min_clear;
        }
        
        if (samples_to_skip > 0 && samples_to_skip < detector->buffer_count) {
            detector->read_index = (detector->read_index + samples_to_skip) % GAIT_BUFFER_SIZE_SAMPLES;
            detector->buffer_count -= samples_to_skip;
            
            LOG_DBG("Cleared %d samples after processing, %d remain in buffer (overlap: %d)",
                    samples_to_skip, detector->buffer_count, overlap_samples);
            
            /* Reset velocity/position at stride boundary for drift correction */
            detector->current_position.x = 0;
            detector->current_position.y = 0;
            detector->current_position.z = 0;
        }
    } else {
        /* No complete strides found - clear some buffer to prevent overflow */
        /* But be more conservative to avoid losing potential events */
        int samples_to_clear = detector->sampling_rate / 4;  /* Clear 0.25 seconds worth */
        if (samples_to_clear > 0 && samples_to_clear < detector->buffer_count) {
            detector->read_index = (detector->read_index + samples_to_clear) % GAIT_BUFFER_SIZE_SAMPLES;
            detector->buffer_count -= samples_to_clear;
            LOG_DBG("No strides: cleared %d samples, %d remain", samples_to_clear, detector->buffer_count);
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
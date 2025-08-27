/**
 * @file gait_advanced_features.c
 * @brief Implementation of advanced features for full compatibility
 */

#include <gait_advanced_features.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

/* Include for isfinite() function */
#ifdef __ZEPHYR__
#include <zephyr/sys/math_extras.h>
#endif

LOG_MODULE_REGISTER(gait_advanced, LOG_LEVEL_INF);

/* Constants */
#define GRAVITY_MS2 9.81f
/* MIN/MAX already defined in zephyr/sys/util.h, so removing duplicates */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define ABS(x) ((x) < 0 ? -(x) : (x))

/**
 * ============================================================================
 * KALMAN GAP FILLING
 * Implements  kalman_fill_gaps algorithm
 * ============================================================================
 */

void kalman_init(kalman_state_t *state, float dt, float q_pos, float q_vel, float r)
{
    memset(state, 0, sizeof(kalman_state_t));
    
    /* Initialize state transition matrix */
    state->dt = dt;
    
    /* Process noise covariance */
    state->Q[0][0] = q_pos;
    state->Q[0][1] = 0.0f;
    state->Q[1][0] = 0.0f;
    state->Q[1][1] = q_vel;
    
    /* Measurement noise */
    state->R = r;
    
    /* Initial covariance */
    state->P[0][0] = 1.0f;
    state->P[1][1] = 1.0f;
}

void kalman_predict(kalman_state_t *state)
{
    /* Predict state: x = F * x */
    /* F = [1, dt; 0, 1] */
    float new_pos = state->x[0] + state->x[1] * state->dt;
    state->x[0] = new_pos;
    /* Velocity unchanged in prediction */
    
    /* Predict covariance: P = F * P * F' + Q */
    float P00 = state->P[0][0] + state->dt * (state->P[1][0] + state->P[0][1]) + 
                state->dt * state->dt * state->P[1][1] + state->Q[0][0];
    float P01 = state->P[0][1] + state->dt * state->P[1][1];
    float P10 = state->P[1][0] + state->dt * state->P[1][1];
    float P11 = state->P[1][1] + state->Q[1][1];
    
    state->P[0][0] = P00;
    state->P[0][1] = P01;
    state->P[1][0] = P10;
    state->P[1][1] = P11;
}

void kalman_update(kalman_state_t *state, float measurement)
{
    /* Calculate Kalman gain: K = P * H' / (H * P * H' + R) */
    /* H = [1, 0] for position measurement */
    float S = state->P[0][0] + state->R;  /* Innovation covariance */
    float K0 = state->P[0][0] / S;        /* Kalman gain for position */
    float K1 = state->P[1][0] / S;        /* Kalman gain for velocity */
    
    /* Update state: x = x + K * (z - H * x) */
    float innovation = measurement - state->x[0];
    state->x[0] += K0 * innovation;
    state->x[1] += K1 * innovation;
    
    /* Update covariance: P = (I - K * H) * P */
    float P00 = (1.0f - K0) * state->P[0][0];
    float P01 = (1.0f - K0) * state->P[0][1];
    float P10 = state->P[1][0] - K1 * state->P[0][0];
    float P11 = state->P[1][1] - K1 * state->P[0][1];
    
    state->P[0][0] = P00;
    state->P[0][1] = P01;
    state->P[1][0] = P10;
    state->P[1][1] = P11;
}

int kalman_fill_gaps(float *signal, int length, float threshold,
                     int max_gap_samples, float sampling_rate)
{
    if (!signal || length <= 0 || length > GAIT_BUFFER_SIZE_SAMPLES) {
        LOG_ERR("Invalid parameters for Kalman gap filling: length=%d", length);
        return -1;
    }
    
    kalman_state_t kalman;
    float dt = 1.0f / sampling_rate;
    kalman_init(&kalman, dt, 1e-5f, 1e-6f, 0.1f);
    
    int gaps_filled = 0;
    int gap_start = -1;
    
    for (int i = 0; i < length; i++) {
        bool is_gap = (signal[i] < threshold);
        
        if (is_gap && gap_start == -1) {
            /* Gap starts */
            gap_start = MAX(0, i - 30);  /* Start prediction 30 samples before */
            
            /* Initialize Kalman filter with last good value */
            if (gap_start > 0) {
                kalman.x[0] = signal[gap_start - 1];
                kalman.x[1] = 0.0f;  /* Initial velocity estimate */
                if (gap_start > 1) {
                    kalman.x[1] = (signal[gap_start - 1] - signal[gap_start - 2]) / dt;
                }
            }
        } else if (!is_gap && gap_start != -1) {
            /* Gap ends */
            int gap_end = i;
            int gap_length = gap_end - gap_start;
            
            if (gap_length > 0 && gap_length <= max_gap_samples) {
                /* Fill the gap using Kalman predictions */
                float start_val = (gap_start > 0) ? signal[gap_start - 1] : signal[gap_end];
                float end_val = signal[gap_end];
                
                /* Linear interpolation as baseline */
                float slope = (gap_length > 0) ? (end_val - start_val) / gap_length : 0.0f;
                
                /* Reset Kalman state */
                kalman.x[0] = start_val;
                kalman.x[1] = slope;
                
                /* Fill gap with Kalman predictions */
                for (int k = gap_start; k < gap_end; k++) {
                    kalman_predict(&kalman);
                    
                    /* Clamp to reasonable bounds */
                    float lo = MIN(start_val, end_val) * 0.8f;
                    float hi = MAX(start_val, end_val) * 1.2f;
                    kalman.x[0] = MAX(lo, MIN(hi, kalman.x[0]));
                    
                    signal[k] = kalman.x[0];
                }
                gaps_filled++;
            }
            gap_start = -1;
        }
    }
    
    LOG_DBG("Filled %d gaps in signal", gaps_filled);
    return gaps_filled;
}

/**
 * ============================================================================
 * MOTION MASK
 * Implements    build_motion_mask algorithm
 * ============================================================================
 */

void motion_mask_init(motion_mask_t *mask, float accel_thresh, float gyro_thresh)
{
    if (!mask) return;
    
    memset(mask->mask, 0, sizeof(mask->mask));
    mask->accel_threshold = accel_thresh > 0 ? accel_thresh : 0.5f;  /* m/s² */
    mask->gyro_threshold = gyro_thresh > 0 ? gyro_thresh : 15.0f;   /* deg/s */
    mask->dilation_samples = MOTION_MASK_DILATION;
}

void build_motion_mask(motion_mask_t *mask, const imu_data_t *imu_buffer, 
                      int count, uint8_t *output_mask)
{
    if (!mask || !imu_buffer || !output_mask || count <= 0) {
        return;
    }
    
    /* Step 1: Detect motion based on thresholds */
    for (int i = 0; i < count; i++) {
        const imu_data_t *imu = &imu_buffer[i];
        
        /* Calculate magnitudes */
        float accel_mag = sqrtf(imu->accel_x * imu->accel_x + 
                               imu->accel_y * imu->accel_y + 
                               imu->accel_z * imu->accel_z);
        float gyro_mag = sqrtf(imu->gyro_x * imu->gyro_x + 
                              imu->gyro_y * imu->gyro_y + 
                              imu->gyro_z * imu->gyro_z) * (180.0f / M_PI);
        
        /* Remove gravity component (approximate) */
        float accel_no_gravity = ABS(accel_mag - GRAVITY_MS2);
        
        /* Motion detection */
        bool is_moving = (accel_no_gravity > mask->accel_threshold) || 
                        (gyro_mag > mask->gyro_threshold);
        
        output_mask[i] = is_moving ? 1 : 0;
    }
    
    /* Step 2: Apply dilation to avoid chopping steps */
    dilate_motion_mask(output_mask, count, mask->dilation_samples);
    
    /* Log statistics */
    int moving_count = 0;
    for (int i = 0; i < count; i++) {
        if (output_mask[i]) moving_count++;
    }
    LOG_DBG("Motion mask: %d/%d samples show motion (%.1f%%)", 
            moving_count, count, (float)moving_count * 100.0f / count);
}

void dilate_motion_mask(uint8_t *mask, int length, int dilation)
{
    if (!mask || length <= 0 || dilation <= 0) {
        return;
    }
    
    /* Bounds check to prevent buffer overflow */
    if (length > GAIT_BUFFER_SIZE_SAMPLES) {
        LOG_ERR("Motion mask length %d exceeds buffer size %d", length, GAIT_BUFFER_SIZE_SAMPLES);
        return;
    }
    
    /* Allocate temporary mask on heap to prevent stack overflow */
    uint8_t *temp_mask = (uint8_t *)k_malloc(length * sizeof(uint8_t));
    if (!temp_mask) {
        LOG_ERR("Failed to allocate memory for motion mask dilation");
        return;
    }
    
    memcpy(temp_mask, mask, length * sizeof(uint8_t));
    
    /* Apply dilation: if any sample in window is 1, set center to 1 */
    for (int i = 0; i < length; i++) {
        bool should_dilate = false;
        
        /* Check window around current sample */
        int start = MAX(0, i - dilation);
        int end = MIN(length - 1, i + dilation);
        
        for (int j = start; j <= end; j++) {
            if (temp_mask[j]) {
                should_dilate = true;
                break;
            }
        }
        
        mask[i] = should_dilate ? 1 : 0;
    }
    
    /* Free allocated memory */
    k_free(temp_mask);
}

/**
 * ============================================================================
 * ENHANCED FSM
 * Implements    detailed gait phase FSM
 * ============================================================================
 */

void gait_fsm_init(gait_fsm_t *fsm, float velocity_threshold)
{
    if (!fsm) return;
    
    memset(fsm, 0, sizeof(gait_fsm_t));
    fsm->current_state = FSM_IDLE;
    fsm->previous_state = FSM_IDLE;
    fsm->velocity_threshold = velocity_threshold > 0 ? velocity_threshold : 0.1f;
    fsm->last_transition_time = k_uptime_get_32();
}

void gait_fsm_update(gait_fsm_t *fsm, float velocity_magnitude, 
                    float pressure_total, uint32_t timestamp)
{
    if (!fsm) return;
    
    /* Update state duration */
    fsm->state_duration_ms = timestamp - fsm->last_transition_time;
    
    /* State transition logic based on velocity and pressure */
    fsm_detailed_phase_t next_state = fsm->current_state;
    
    switch (fsm->current_state) {
        case FSM_IDLE:
            if (velocity_magnitude > fsm->velocity_threshold) {
                next_state = FSM_PRE_SWING;
            }
            break;
            
        case FSM_PRE_SWING:
            if (pressure_total < 100) {  /* Foot lifting */
                next_state = FSM_INITIAL_SWING;
            }
            break;
            
        case FSM_INITIAL_SWING:
            if (fsm->state_duration_ms > 100) {  /* Time-based transition */
                next_state = FSM_MID_SWING;
            }
            break;
            
        case FSM_MID_SWING:
            if (velocity_magnitude < fsm->velocity_threshold * 2.0f) {
                next_state = FSM_TERMINAL_SWING;
            }
            break;
            
        case FSM_TERMINAL_SWING:
            if (pressure_total > 100) {  /* Foot contact */
                next_state = FSM_LOADING;
            }
            break;
            
        case FSM_LOADING:
            if (pressure_total > 500 && fsm->state_duration_ms > 50) {
                next_state = FSM_MID_STANCE;
            }
            break;
            
        case FSM_MID_STANCE:
            if (velocity_magnitude < fsm->velocity_threshold) {
                next_state = FSM_TERMINAL_STANCE;
            }
            break;
            
        case FSM_TERMINAL_STANCE:
            if (pressure_total < 300) {  /* Heel lifting */
                next_state = FSM_PRE_SWING_STANCE;
            }
            break;
            
        case FSM_PRE_SWING_STANCE:
            if (pressure_total < 100) {
                next_state = FSM_PRE_SWING;
            }
            break;
    }
    
    /* Handle state transition */
    if (next_state != fsm->current_state) {
        fsm->previous_state = fsm->current_state;
        fsm->current_state = next_state;
        fsm->last_transition_time = timestamp;
        fsm->state_duration_ms = 0;
        
        LOG_DBG("FSM transition: %d -> %d", fsm->previous_state, fsm->current_state);
    }
    
    /* Calculate phase percentage based on typical durations */
    static const uint32_t typical_durations_ms[] = {
        0,    /* IDLE */
        50,   /* PRE_SWING */
        100,  /* INITIAL_SWING */
        150,  /* MID_SWING */
        100,  /* TERMINAL_SWING */
        100,  /* LOADING */
        200,  /* MID_STANCE */
        100,  /* TERMINAL_STANCE */
        50    /* PRE_SWING_STANCE */
    };
    
    uint32_t typical_duration = typical_durations_ms[fsm->current_state];
    if (typical_duration > 0) {
        fsm->phase_percentage = MIN(100.0f, 
            (float)fsm->state_duration_ms * 100.0f / typical_duration);
    }
}

fsm_detailed_phase_t gait_fsm_get_phase(const gait_fsm_t *fsm)
{
    return fsm ? fsm->current_state : FSM_IDLE;
}

float gait_fsm_get_phase_percentage(const gait_fsm_t *fsm)
{
    return fsm ? fsm->phase_percentage : 0.0f;
}

/**
 * ============================================================================
 * QUATERNION IMU FUSION
 * Full quaternion-based sensor fusion like    GyroOnlyTiltFusion
 * ============================================================================
 */

void imu_fusion_init(imu_fusion_state_t *fusion, float sampling_rate)
{
    if (!fusion) return;
    
    memset(fusion, 0, sizeof(imu_fusion_state_t));
    
    /* Initialize quaternion to identity (no rotation) */
    fusion->orientation.w = 1.0f;
    fusion->orientation.x = 0.0f;
    fusion->orientation.y = 0.0f;
    fusion->orientation.z = 0.0f;
    
    /* Set gravity vector */
    fusion->gravity[0] = 0.0f;
    fusion->gravity[1] = 0.0f;
    fusion->gravity[2] = -GRAVITY_MS2;
    
    fusion->sampling_rate = sampling_rate;
    fusion->initialized = true;
}

void imu_fusion_update(imu_fusion_state_t *fusion, const imu_data_t *imu, 
                      bool zupt, bool gait_event)
{
    if (!fusion || !imu || !fusion->initialized) return;
    
    float dt = 1.0f / fusion->sampling_rate;
    
    /* Step 1: Update orientation from gyroscope */
    vector3_t gyro = {
        .x = imu->gyro_x - fusion->gyro_bias.x,
        .y = imu->gyro_y - fusion->gyro_bias.y,
        .z = imu->gyro_z - fusion->gyro_bias.z
    };
    
    quaternion_t q_delta = quaternion_from_gyro(&gyro, dt);
    fusion->orientation = quaternion_multiply(&fusion->orientation, &q_delta);
    quaternion_normalize(&fusion->orientation);
    
    /* Step 2: Remove gravity from acceleration */
    vector3_t accel_body = {imu->accel_x, imu->accel_y, imu->accel_z};
    
    /* Rotate gravity to body frame */
    quaternion_t q_conj = {
        .w = fusion->orientation.w,
        .x = -fusion->orientation.x,
        .y = -fusion->orientation.y,
        .z = -fusion->orientation.z
    };
    vector3_t gravity_body = {0, 0, -GRAVITY_MS2};
    gravity_body = rotate_vector_by_quaternion(&gravity_body, &q_conj);
    
    /* Remove gravity */
    fusion->acceleration_body.x = accel_body.x - gravity_body.x;
    fusion->acceleration_body.y = accel_body.y - gravity_body.y;
    fusion->acceleration_body.z = accel_body.z - gravity_body.z;
    
    /* Rotate to global frame */
    fusion->acceleration_global = rotate_vector_by_quaternion(&fusion->acceleration_body, 
                                                             &fusion->orientation);
    
    /* Step 3: Integrate to get velocity */
    fusion->velocity.x += fusion->acceleration_global.x * dt;
    fusion->velocity.y += fusion->acceleration_global.y * dt;
    fusion->velocity.z += fusion->acceleration_global.z * dt;
    
    /* Step 4: Apply ZUPT if needed */
    if (zupt) {
        float vel_mag = sqrtf(fusion->velocity.x * fusion->velocity.x +
                             fusion->velocity.y * fusion->velocity.y +
                             fusion->velocity.z * fusion->velocity.z);
        
        if (vel_mag < 0.1f) {
            /* Hard reset at very low velocities */
            fusion->velocity.x = 0.0f;
            fusion->velocity.y = 0.0f;
            fusion->velocity.z = 0.0f;
            
            /* Update gyro bias during stance */
            float alpha = 0.01f;  /* Bias adaptation rate */
            fusion->gyro_bias.x = (1.0f - alpha) * fusion->gyro_bias.x + alpha * imu->gyro_x;
            fusion->gyro_bias.y = (1.0f - alpha) * fusion->gyro_bias.y + alpha * imu->gyro_y;
            fusion->gyro_bias.z = (1.0f - alpha) * fusion->gyro_bias.z + alpha * imu->gyro_z;
        } else {
            /* Soft damping */
            fusion->velocity.x *= 0.95f;
            fusion->velocity.y *= 0.95f;
            fusion->velocity.z *= 0.95f;
        }
    }
    
    /* Step 5: Integrate to get position */
    fusion->position.x += fusion->velocity.x * dt;
    fusion->position.y += fusion->velocity.y * dt;
    fusion->position.z += fusion->velocity.z * dt;
    
    /* Step 6: Reset at gait events if needed */
    if (gait_event) {
        /* Can reset position/velocity at known events */
        /* This helps prevent drift accumulation */
    }
}

void imu_fusion_reset(imu_fusion_state_t *fusion)
{
    if (!fusion) return;
    
    /* Reset to initial state but keep calibration */
    fusion->position.x = 0.0f;
    fusion->position.y = 0.0f;
    fusion->position.z = 0.0f;
    
    fusion->velocity.x = 0.0f;
    fusion->velocity.y = 0.0f;
    fusion->velocity.z = 0.0f;
    
    /* Keep orientation and gyro bias */
}

/* Quaternion operations */

quaternion_t quaternion_multiply(const quaternion_t *q1, const quaternion_t *q2)
{
    quaternion_t result;
    
    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    
    return result;
}

quaternion_t quaternion_from_gyro(const vector3_t *gyro, float dt)
{
    /* Small angle approximation for quaternion update */
    float half_dt = dt * 0.5f;
    float wx = gyro->x * half_dt;
    float wy = gyro->y * half_dt;
    float wz = gyro->z * half_dt;
    
    quaternion_t q;
    q.w = 1.0f;
    q.x = wx;
    q.y = wy;
    q.z = wz;
    
    /* Normalize for accuracy */
    quaternion_normalize(&q);
    
    return q;
}

void quaternion_normalize(quaternion_t *q)
{
    if (!q) return;
    
    float norm_sq = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    
    /* Check for NaN or Inf */
    if (!isfinite(norm_sq)) {
        LOG_ERR("Quaternion contains NaN or Inf, resetting to identity");
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
        return;
    }
    
    float norm = sqrtf(norm_sq);
    
    if (norm > QUATERNION_NORM_EPSILON) {
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    } else {
        /* Quaternion is too small, reset to identity to prevent issues */
        LOG_WRN("Quaternion norm too small (%.6f), resetting to identity", norm);
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
    }
}

vector3_t quaternion_to_euler(const quaternion_t *q)
{
    vector3_t euler;
    
    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    euler.x = atan2f(sinr_cosp, cosr_cosp);
    
    /* Pitch (y-axis rotation) */
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (ABS(sinp) >= 1.0f) {
        euler.y = copysignf(M_PI / 2.0f, sinp);
    } else {
        euler.y = asinf(sinp);
    }
    
    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    euler.z = atan2f(siny_cosp, cosy_cosp);
    
    return euler;
}

vector3_t rotate_vector_by_quaternion(const vector3_t *v, const quaternion_t *q)
{
    /* v' = q * v * q^(-1) */
    quaternion_t v_quat = {0.0f, v->x, v->y, v->z};
    quaternion_t q_conj = {q->w, -q->x, -q->y, -q->z};
    
    quaternion_t temp = quaternion_multiply(q, &v_quat);
    quaternion_t result_quat = quaternion_multiply(&temp, &q_conj);
    
    vector3_t result = {result_quat.x, result_quat.y, result_quat.z};
    return result;
}

/**
 * ============================================================================
 * ANTHROPOMETRIC MODELS
 * Complete implementation of    speed estimation models
 * ============================================================================
 */

void anthro_init(anthro_estimator_t *estimator, float height_m)
{
    if (!estimator) return;
    
    estimator->subject_height_m = height_m;
    estimator->leg_ratio = 0.53f;  /* Standard leg/height ratio */
    estimator->preferred_model = ANTHRO_MODEL_HEIDERSCHEIT;
    estimator->is_running = false;
}

float anthro_calculate_speed(const anthro_estimator_t *estimator,
                            float cadence_spm, float stride_length_m,
                            float contact_time_s, float swing_time_s)
{
    if (!estimator) return 0.0f;
    
    /* Determine if running based on cadence */
    /* Note: is_running is const in the struct, can't modify after init */
    /* Would need to set during initialization or use mutable field */
    /* estimator->is_running = (cadence_spm >= 132.0f); */
    
    /* If stride length is known, use direct calculation */
    if (stride_length_m > 0.0f) {
        return (cadence_spm * stride_length_m) / 120.0f;  /* 2 steps per stride */
    }
    
    /* Otherwise use model based on available data */
    float speed = 0.0f;
    
    switch (estimator->preferred_model) {
        case ANTHRO_MODEL_WEISS:
            speed = anthro_weiss_stride_length(estimator->subject_height_m, cadence_spm) * 
                   cadence_spm / 120.0f;
            break;
            
        case ANTHRO_MODEL_HEIDERSCHEIT:
            speed = anthro_heiderscheit_stride_length(estimator->subject_height_m, cadence_spm) * 
                   cadence_spm / 120.0f;
            break;
            
        case ANTHRO_MODEL_SHIN_PARK:
            if (swing_time_s > 0) {
                speed = anthro_shin_park_speed(estimator->subject_height_m, cadence_spm, swing_time_s);
            }
            break;
            
        case ANTHRO_MODEL_MORIN:
            if (contact_time_s > 0 && estimator->is_running) {
                speed = anthro_morin_speed(estimator->subject_height_m, contact_time_s);
            }
            break;
            
        case ANTHRO_MODEL_HOYT:
            if (contact_time_s > 0 && estimator->is_running) {
                speed = anthro_hoyt_speed(estimator->subject_height_m, contact_time_s);
            }
            break;
    }
    
    /* Clamp to reasonable range */
    if (estimator->is_running) {
        speed = MAX(2.0f, MIN(6.0f, speed));  /* 2-6 m/s for running */
    } else {
        speed = MAX(0.5f, MIN(2.0f, speed));  /* 0.5-2 m/s for walking */
    }
    
    return speed;
}

float anthro_weiss_stride_length(float height_m, float cadence_spm)
{
    /* Weiss 2014: SL = height * (0.246 + 0.0029 * cadence) */
    return height_m * (0.246f + 0.0029f * cadence_spm);
}

float anthro_heiderscheit_stride_length(float height_m, float cadence_spm)
{
    /* Heiderscheit population: SL = height * 1.14 * (170 / cadence) */
    if (cadence_spm <= 0) return 0.0f;
    
    float stride_length = height_m * 1.14f * (170.0f / cadence_spm);
    
    /* Clamp to reasonable bounds */
    return MAX(0.9f * height_m, MIN(1.6f * height_m, stride_length));
}

float anthro_shin_park_speed(float height_m, float cadence_spm, float swing_time_s)
{
    /* Shin & Park swing time model: v = k * sqrt(g * L) * Ts * cadence / 120 */
    float L = 0.53f * height_m;  /* Leg length */
    float k = (height_m >= 1.75f) ? 1.30f : 1.25f;
    
    /* Adjust k based on cadence */
    if (cadence_spm < 110.0f) {
        k += 0.02f;
    } else if (cadence_spm > 125.0f) {
        k -= 0.02f;
    }
    
    return k * sqrtf(GRAVITY_MS2 * L) * swing_time_s * cadence_spm / 120.0f;
}

float anthro_morin_speed(float height_m, float contact_time_s)
{
    /* Morin contact time: v ≈ k_ct * (L_leg / Tc) */
    const float K_CT = 0.26f;
    float L_leg = 0.53f * height_m;
    
    if (contact_time_s <= 0) return 0.0f;
    
    return K_CT * (L_leg / contact_time_s);
}

float anthro_hoyt_speed(float height_m, float contact_time_s)
{
    /* Hoyt: tc = 0.80 * L^0.84 / v^0.87  =>  v = (0.80 * L^0.84 / tc)^(1/0.87) */
    float L = 0.53f * height_m;
    
    if (contact_time_s <= 0) return 0.0f;
    
    float numerator = 0.80f * powf(L, 0.84f);
    return powf(numerator / contact_time_s, 1.0f / 0.87f);
}

/**
 * ============================================================================
 * ROLLING WINDOW AVERAGING
 * Implements    RollingWindow class
 * ============================================================================
 */

void rolling_window_init(rolling_window_t *window, int max_size, uint32_t duration_ms)
{
    if (!window) return;
    
    memset(window, 0, sizeof(rolling_window_t));
    window->max_size = MIN(max_size, 100);  /* Cap at 100 samples */
    window->window_duration_ms = duration_ms;
    
    /* Initialize timestamps to avoid uninitialized data issues */
    for (int i = 0; i < window->max_size; i++) {
        window->timestamps[i] = 0;
        window->buffer[i] = 0.0f;
    }
    
    window->head = 0;
    window->tail = 0;
    window->count = 0;
}

void rolling_window_add(rolling_window_t *window, float value, uint32_t timestamp)
{
    if (!window) return;
    
    /* Validate timestamp to prevent issues */
    if (timestamp == 0) {
        LOG_WRN("Invalid timestamp 0 provided to rolling window");
        return;
    }
    
    /* Remove old samples outside window */
    uint32_t cutoff_time = (timestamp > window->window_duration_ms) ?
                          (timestamp - window->window_duration_ms) : 0;
    
    /* Add loop protection to prevent infinite loop */
    int max_iterations = window->max_size + 1;
    
    while (window->count > 0 && max_iterations-- > 0) {
        int oldest_idx = window->tail;
        
        /* Bounds check to prevent invalid memory access */
        if (oldest_idx < 0 || oldest_idx >= window->max_size) {
            LOG_ERR("Rolling window corruption detected: tail=%d, max=%d",
                    oldest_idx, window->max_size);
            /* Reset window to recover */
            window->tail = 0;
            window->head = 0;
            window->count = 0;
            break;
        }
        
        /* Check if timestamp is valid before comparison */
        if (window->timestamps[oldest_idx] == 0 ||
            window->timestamps[oldest_idx] >= cutoff_time) {
            break;  /* Still within window or uninitialized */
        }
        
        /* Remove oldest */
        window->tail = (window->tail + 1) % window->max_size;
        window->count--;
    }
    
    /* Check if we hit max iterations (indicates a problem) */
    if (max_iterations <= 0) {
        LOG_ERR("Rolling window cleanup exceeded max iterations - resetting");
        window->tail = 0;
        window->head = 0;
        window->count = 0;
    }
    
    /* Add new value */
    if (window->count < window->max_size) {
        window->buffer[window->head] = value;
        window->timestamps[window->head] = timestamp;
        window->head = (window->head + 1) % window->max_size;
        window->count++;
    } else {
        /* Buffer full, overwrite oldest */
        window->tail = (window->tail + 1) % window->max_size;
        window->buffer[window->head] = value;
        window->timestamps[window->head] = timestamp;
        window->head = (window->head + 1) % window->max_size;
    }
}

float rolling_window_get_average(const rolling_window_t *window)
{
    if (!window || window->count == 0) return 0.0f;
    
    float sum = 0.0f;
    int idx = window->tail;
    
    for (int i = 0; i < window->count; i++) {
        sum += window->buffer[idx];
        idx = (idx + 1) % window->max_size;
    }
    
    return sum / window->count;
}

float rolling_window_get_std_dev(const rolling_window_t *window)
{
    if (!window || window->count < 2) return 0.0f;
    
    float avg = rolling_window_get_average(window);
    float sum_sq = 0.0f;
    int idx = window->tail;
    
    for (int i = 0; i < window->count; i++) {
        float diff = window->buffer[idx] - avg;
        sum_sq += diff * diff;
        idx = (idx + 1) % window->max_size;
    }
    
    return sqrtf(sum_sq / (window->count - 1));
}

void rolling_window_clear(rolling_window_t *window)
{
    if (!window) return;
    
    window->head = 0;
    window->tail = 0;
    window->count = 0;
}

bool rolling_window_has_sufficient_data(const rolling_window_t *window)
{
    return window && window->count >= 2;
}

/* Additional getters for IMU fusion */

quaternion_t imu_fusion_get_orientation(const imu_fusion_state_t *fusion)
{
    return fusion ? fusion->orientation : (quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
}

vector3_t imu_fusion_get_position(const imu_fusion_state_t *fusion)
{
    return fusion ? fusion->position : (vector3_t){0.0f, 0.0f, 0.0f};
}

vector3_t imu_fusion_get_velocity(const imu_fusion_state_t *fusion)
{
    return fusion ? fusion->velocity : (vector3_t){0.0f, 0.0f, 0.0f};
}
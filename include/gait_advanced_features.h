/**
 * @file gait_advanced_features.h
 * @brief Advanced features for full compatibility with colleague's implementation
 */

#ifndef GAIT_ADVANCED_FEATURES_H
#define GAIT_ADVANCED_FEATURES_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Define buffer size here to avoid dependency issues */
#ifndef GAIT_BUFFER_SIZE_SAMPLES
#define GAIT_BUFFER_SIZE_SAMPLES   600    /* 3 seconds @ 200Hz */
#endif

/* Constants */
#define KALMAN_MAX_GAP_MS        200    /* Max gap to fill (200ms) */
#define MOTION_MASK_DILATION     10     /* Samples to dilate motion mask */
#define QUATERNION_NORM_EPSILON  0.001f /* For quaternion normalization */

/* IMU data structure - define here so both headers can use it */
#ifndef IMU_DATA_T_DEFINED
#define IMU_DATA_T_DEFINED
typedef struct {
    float quat_w, quat_x, quat_y, quat_z;  /* Quaternion */
    float gyro_x, gyro_y, gyro_z;          /* Gyroscope (rad/s) */
    float accel_x, accel_y, accel_z;       /* Accelerometer (m/s²) */
} imu_data_t;
#endif

/* Quaternion structure */
typedef struct {
    float w, x, y, z;
} quaternion_t;

/* 3D vector */
typedef struct {
    float x, y, z;
} vector3_t;

/* Kalman filter state for gap filling */
typedef struct {
    float x[2];      /* State: [position, velocity] */
    float P[2][2];   /* Covariance matrix */
    float Q[2][2];   /* Process noise */
    float R;         /* Measurement noise */
    float dt;        /* Time step */
} kalman_state_t;

/* Motion mask builder */
typedef struct {
    uint8_t mask[GAIT_BUFFER_SIZE_SAMPLES];
    float accel_threshold;
    float gyro_threshold;
    int dilation_samples;
} motion_mask_t;

/* Enhanced FSM for gait phases */
typedef enum {
    FSM_IDLE,
    FSM_PRE_SWING,
    FSM_INITIAL_SWING,
    FSM_MID_SWING,
    FSM_TERMINAL_SWING,
    FSM_LOADING,
    FSM_MID_STANCE,
    FSM_TERMINAL_STANCE,
    FSM_PRE_SWING_STANCE
} fsm_detailed_phase_t;

typedef struct {
    fsm_detailed_phase_t current_state;
    fsm_detailed_phase_t previous_state;
    float velocity_threshold;
    uint32_t state_duration_ms;
    uint32_t last_transition_time;
    float phase_percentage;  /* 0-100% within current phase */
} gait_fsm_t;

/* Full quaternion-based IMU fusion */
typedef struct {
    quaternion_t orientation;
    vector3_t position;
    vector3_t velocity;
    vector3_t acceleration_global;
    vector3_t acceleration_body;
    vector3_t gyro_bias;
    float gravity[3];
    float sampling_rate;
    bool initialized;
} imu_fusion_state_t;

/* Anthropometric model types */
typedef enum {
    ANTHRO_MODEL_WEISS,           /* Weiss 2014 */
    ANTHRO_MODEL_HEIDERSCHEIT,    /* Heiderscheit population */
    ANTHRO_MODEL_SHIN_PARK,       /* Shin & Park swing time */
    ANTHRO_MODEL_MORIN,           /* Morin contact time */
    ANTHRO_MODEL_HOYT             /* Hoyt scaling */
} anthro_model_type_t;

/* Anthropometric speed estimator */
typedef struct {
    float subject_height_m;
    float leg_ratio;  /* Typically 0.53 */
    anthro_model_type_t preferred_model;
    bool is_running;  /* Walk vs run detection */
} anthro_estimator_t;

/* Rolling window for averaging */
typedef struct {
    float buffer[100];  /* Up to 100 samples */
    uint32_t timestamps[100];
    int head;
    int tail;
    int count;
    int max_size;
    uint32_t window_duration_ms;
} rolling_window_t;

/* Function prototypes */

/* Kalman gap filling */
void kalman_init(kalman_state_t *state, float dt, float q_pos, float q_vel, float r);
void kalman_predict(kalman_state_t *state);
void kalman_update(kalman_state_t *state, float measurement);
int kalman_fill_gaps(float *signal, int length, float threshold, 
                     int max_gap_samples, float sampling_rate);

/* Motion mask */
void motion_mask_init(motion_mask_t *mask, float accel_thresh, float gyro_thresh);
void build_motion_mask(motion_mask_t *mask, const imu_data_t *imu_buffer, 
                      int count, uint8_t *output_mask);
void dilate_motion_mask(uint8_t *mask, int length, int dilation);

/* Enhanced FSM */
void gait_fsm_init(gait_fsm_t *fsm, float velocity_threshold);
void gait_fsm_update(gait_fsm_t *fsm, float velocity_magnitude, 
                    float pressure_total, uint32_t timestamp);
fsm_detailed_phase_t gait_fsm_get_phase(const gait_fsm_t *fsm);
float gait_fsm_get_phase_percentage(const gait_fsm_t *fsm);

/* Quaternion IMU fusion */
void imu_fusion_init(imu_fusion_state_t *fusion, float sampling_rate);
void imu_fusion_update(imu_fusion_state_t *fusion, const imu_data_t *imu, 
                      bool zupt, bool gait_event);
void imu_fusion_reset(imu_fusion_state_t *fusion);
quaternion_t imu_fusion_get_orientation(const imu_fusion_state_t *fusion);
vector3_t imu_fusion_get_position(const imu_fusion_state_t *fusion);
vector3_t imu_fusion_get_velocity(const imu_fusion_state_t *fusion);

/* Quaternion operations */
quaternion_t quaternion_multiply(const quaternion_t *q1, const quaternion_t *q2);
quaternion_t quaternion_from_gyro(const vector3_t *gyro, float dt);
void quaternion_normalize(quaternion_t *q);
vector3_t quaternion_to_euler(const quaternion_t *q);
vector3_t rotate_vector_by_quaternion(const vector3_t *v, const quaternion_t *q);

/* Anthropometric models */
void anthro_init(anthro_estimator_t *estimator, float height_m);
float anthro_calculate_speed(const anthro_estimator_t *estimator,
                            float cadence_spm, float stride_length_m,
                            float contact_time_s, float swing_time_s);
float anthro_weiss_stride_length(float height_m, float cadence_spm);
float anthro_heiderscheit_stride_length(float height_m, float cadence_spm);
float anthro_shin_park_speed(float height_m, float cadence_spm, float swing_time_s);
float anthro_morin_speed(float height_m, float contact_time_s);
float anthro_hoyt_speed(float height_m, float contact_time_s);

/* Rolling window averaging */
void rolling_window_init(rolling_window_t *window, int max_size, uint32_t duration_ms);
void rolling_window_add(rolling_window_t *window, float value, uint32_t timestamp);
float rolling_window_get_average(const rolling_window_t *window);
float rolling_window_get_std_dev(const rolling_window_t *window);
void rolling_window_clear(rolling_window_t *window);
bool rolling_window_has_sufficient_data(const rolling_window_t *window);

#ifdef __cplusplus
}
#endif

#endif /* GAIT_ADVANCED_FEATURES_H */
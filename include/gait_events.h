/**
 * @file gait_events.h
 * @brief Event-driven gait parameter detection and calculation
 * Based on pressure threshold crossing events (IC/TO)
 */

#ifndef GAIT_EVENTS_H
#define GAIT_EVENTS_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Include app.hpp to get foot_samples_t definition */
#include <app.hpp>

/* Include advanced features header */
#include <gait_advanced_features.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration constants for 100Hz foot sensor sampling rate */
#define GAIT_BUFFER_SIZE_SAMPLES   300    /* 3 seconds @ 100Hz */
#define GAIT_SAMPLING_RATE         100.0f /* Hz (foot sensor rate) */
#define GAIT_MIN_STRIDE_TIME       0.33f  /* 330ms minimum between strides */
#define GAIT_MAX_STRIDE_TIME       6.0f   /* 6 seconds max (after pause) */
#define GAIT_MIN_CONTACT_TIME      0.120f /* 120ms minimum contact time */
#define GAIT_REFRACTORY_TIME       0.120f /* 120ms refractory after TO */
#define GAIT_PRESSURE_THRESHOLD    150    /* Adaptive pressure threshold */
#define GAIT_MAX_EVENTS_PER_CHUNK  10     /* Max strides per 3-second chunk */
#define GAIT_VELOCITY_THRESHOLD    0.1f   /* m/s for ZUPT */

/* Gait phase states (FSM) */
typedef enum {
    GAIT_PHASE_IDLE = 0,
    GAIT_PHASE_SWING,
    GAIT_PHASE_STANCE,
} gait_phase_t;

/* Event types */
typedef enum {
    GAIT_EVENT_NONE = 0,
    GAIT_EVENT_IC,    /* Initial Contact (foot strike) */
    GAIT_EVENT_TO,    /* Toe Off (foot lift) */
} gait_event_type_t;

/* Stride segment structure [IC -> TO -> next_IC] */
typedef struct {
    int ic_index;           /* Initial contact index in buffer */
    int to_index;           /* Toe-off index in buffer */
    int next_ic_index;      /* Next initial contact index */
    float ic_timestamp;     /* IC timestamp in seconds */
    float to_timestamp;     /* TO timestamp in seconds */
    float next_ic_timestamp;/* Next IC timestamp in seconds */
    bool valid;            /* Validity flag */
} stride_segment_t;

/* Gait metrics for one stride (matches colleague's GaitMetrics struct) */
typedef struct {
    float timestamp;              /* Midpoint timestamp */
    float start_time;            /* IC timestamp */
    float end_time;              /* Next IC timestamp */
    float gct;                   /* Ground Contact Time (seconds) */
    float duration;              /* Total stride duration (seconds) */
    float cadence;               /* Steps per minute */
    float stride_length;         /* Forward distance (meters) */
    float stride_velocity;       /* Average velocity (m/s) */
    float vertical_oscillation;  /* Vertical movement (meters) */
    float step_width;            /* Lateral movement estimate (meters) */
    float ic_pitch;              /* Pitch angle at IC (degrees) */
    float to_pitch;              /* Pitch angle at TO (degrees) */
    float ic_roll;               /* Roll angle at IC (degrees) */
    float to_roll;               /* Roll angle at TO (degrees) */
    float min_roll;              /* Minimum roll during stride */
    float max_roll;              /* Maximum roll during stride */
    float min_pitch;             /* Minimum pitch during stride */
    float speed_anthropometric;  /* Speed from anthropometric model */
    uint32_t step_count;         /* Cumulative step count */
    
    /* Additional metrics for event-based processing */
    int8_t pronation_at_ic;       /* Pronation angle at initial contact (degrees) */
    int8_t max_pronation;         /* Maximum pronation during stance (degrees) */
    int16_t cop_x_at_ic;          /* COP X at initial contact (mm) */
    int16_t cop_y_at_ic;          /* COP Y at initial contact (mm) */
    int16_t cop_x_at_to;          /* COP X at toe-off (mm) */
    int16_t cop_y_at_to;          /* COP Y at toe-off (mm) */
    float cop_path_length;        /* Total COP path length during stance (mm) */
    uint8_t strike_pattern;       /* 0=unknown, 1=heel, 2=midfoot, 3=forefoot */
    uint16_t loading_rate;        /* Peak loading rate (N/s) */
    uint8_t arch_collapse_index;  /* Arch collapse during midstance (0-100) */
    uint16_t cpei;                /* Center of Pressure Excursion Index */
    uint16_t push_off_power;      /* Push-off power index */
    uint16_t peak_pressure;       /* Peak total pressure during stance */
    
    bool valid;                   /* Validity flag */
} gait_metrics_t;

/* 3D velocity state for integration */
typedef struct {
    float x;  /* Forward velocity (m/s) */
    float y;  /* Lateral velocity (m/s) */
    float z;  /* Vertical velocity (m/s) */
} velocity_3d_t;

/* Position state */
typedef struct {
    float x;  /* Forward position (m) */
    float y;  /* Lateral position (m) */
    float z;  /* Vertical position (m) */
} position_3d_t;

/* foot_samples_t is already defined in app.hpp with values[8] array */

/* imu_data_t is defined in gait_advanced_features.h */

/* Main event detector state (Consumer) */
typedef struct {
    /* Ring buffer for continuous data (3 seconds) */
    foot_samples_t foot_buffer[GAIT_BUFFER_SIZE_SAMPLES];
    imu_data_t imu_buffer[GAIT_BUFFER_SIZE_SAMPLES];
    float timestamps[GAIT_BUFFER_SIZE_SAMPLES];
    
    /* Sensor fusion state */
    velocity_3d_t velocities[GAIT_BUFFER_SIZE_SAMPLES];
    position_3d_t positions[GAIT_BUFFER_SIZE_SAMPLES];
    
    /* Buffer management */
    int write_index;       /* Where to write next sample */
    int read_index;        /* Where to start processing */
    int buffer_count;      /* Number of samples in buffer */
    bool buffer_full;      /* Buffer has filled at least once */
    
    /* Event detection results */
    int ic_indices[GAIT_MAX_EVENTS_PER_CHUNK];
    int to_indices[GAIT_MAX_EVENTS_PER_CHUNK];
    int ic_count;
    int to_count;
    
    /* Stride segments */
    stride_segment_t stride_segments[GAIT_MAX_EVENTS_PER_CHUNK];
    int stride_count;
    
    /* Calculated metrics queue */
    gait_metrics_t metrics_queue[GAIT_MAX_EVENTS_PER_CHUNK];
    int metrics_count;
    
    /* FSM state tracking */
    gait_phase_t current_phase;
    int last_ic_index;
    int last_to_index;
    float last_timestamp;
    uint32_t total_step_count;
    
    /* IMU fusion state - BASIC VERSION */
    velocity_3d_t current_velocity;
    position_3d_t current_position;
    float gravity_vector[3];
    
    /* ADVANCED FEATURES INTEGRATION */
    /* Motion mask for false positive reduction */
    motion_mask_t motion_mask;
    uint8_t motion_mask_buffer[GAIT_BUFFER_SIZE_SAMPLES];
    
    /* Enhanced 9-state FSM */
    gait_fsm_t detailed_fsm;
    
    /* Full quaternion IMU fusion (replaces basic fusion when enabled) */
    imu_fusion_state_t imu_fusion;
    
    /* Anthropometric models for speed estimation */
    anthro_estimator_t anthro_model;
    
    /* Rolling windows for metric smoothing */
    rolling_window_t gct_window;
    rolling_window_t stride_window;
    rolling_window_t cadence_window;
    
    /* Kalman filter for gap filling */
    kalman_state_t pressure_kalman;
    float pressure_signal_filled[GAIT_BUFFER_SIZE_SAMPLES];
    
    /* Configuration */
    float sampling_rate;
    int pressure_threshold;
    bool is_primary;
    float subject_height_m;  /* For anthropometric calculations */
    bool use_advanced_features;  /* Enable/disable advanced features */
    
    /* Processing state */
    bool processing_active;
    bool waiting_for_data;
    
    /* Statistics for advanced features */
    uint32_t gaps_filled;
    uint32_t false_positives_rejected;
    uint32_t quaternion_updates;
} gait_event_detector_t;

/* Function prototypes */

/**
 * Initialize the gait event detector
 */
void gait_events_init(gait_event_detector_t *detector, bool is_primary, float sampling_rate);

/**
 * Add new data to the ring buffer (called at high frequency)
 */
void gait_events_add_data(gait_event_detector_t *detector,
                          const foot_samples_t *foot_data,
                          const imu_data_t *imu_data,
                          float timestamp);

/**
 * Consumer function - processes buffered data to detect events
 * Should be called continuously by consumer thread
 * Returns number of new metrics calculated
 */
int gait_events_process(gait_event_detector_t *detector);

/**
 * Get calculated metrics from the queue
 * Returns number of metrics retrieved
 */
int gait_events_get_metrics(gait_event_detector_t *detector,
                            gait_metrics_t *metrics_out,
                            int max_metrics);

/**
 * Clear processed metrics from queue
 */
void gait_events_clear_metrics(gait_event_detector_t *detector);

/**
 * Check if buffer has enough data for processing
 */
bool gait_events_ready_to_process(gait_event_detector_t *detector);

/**
 * Set subject height for anthropometric speed calculations
 */
void gait_events_set_subject_height(gait_event_detector_t *detector, float height_m);

/* Internal helper functions */
static int calculate_total_pressure(const foot_samples_t *foot_data);
static void detect_ic_to_events(gait_event_detector_t *detector);
static void create_stride_segments(gait_event_detector_t *detector);
static void calculate_stride_metrics(gait_event_detector_t *detector);
static void update_velocity_integration(gait_event_detector_t *detector, 
                                       const imu_data_t *imu_data, float dt);
static void apply_zupt(gait_event_detector_t *detector, bool is_stance);
static float calculate_adaptive_threshold(const foot_samples_t *buffer, 
                                         int start, int count);

#ifdef __cplusplus
}
#endif

#endif /* GAIT_EVENTS_H */
/**
 * @file choros_buffer.hpp
 * @brief Choros-compatible ring buffer for advanced gait algorithms
 * @version 1.0
 * @date 2025
 * 
 * This buffer is added IN PARALLEL to existing code - no breaking changes
 */

#ifndef CHOROS_BUFFER_HPP
#define CHOROS_BUFFER_HPP

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Buffer configuration - 3 seconds at 200Hz */
#define CHOROS_SAMPLE_RATE_HZ 200
#define CHOROS_BUFFER_SECONDS 3
#define CHOROS_BUFFER_SIZE 600  /* 3 sec × 200Hz */

/* Enable/disable Choros features (safety switch) */
#define CHOROS_ENABLED 1
#define CHOROS_USE_FOR_GCT 1  /* Set to 0 to use old calculation */

/* Choros-compatible IMU data structure */
typedef struct {
    /* Motion sensor data */
    float acc_x, acc_y, acc_z;       /* Accelerometer (m/s²) */
    float gyro_x, gyro_y, gyro_z;    /* Gyroscope (rad/s) */
    float mag_x, mag_y, mag_z;       /* Magnetometer (μT) - we set to 0 */
    
    /* Timestamp - CRITICAL for algorithms */
    float timestamp;                  /* Seconds as float */
    
    /* Foot pressure data */
    float insole_pressures[8];        /* 8 pressure sensors */
    
    /* Metadata */
    float battery;                    /* Battery percentage */
    bool reverse_pressure_order;     /* Pressure array order */
    uint8_t orientation;              /* 0=Heel, 1=Side mount */
    uint8_t foot;                     /* 0=Left, 1=Right */
    char shoe[10];                    /* "left" or "right" */
} choros_imu_data_t;

/* Ring buffer structure */
typedef struct {
    choros_imu_data_t samples[CHOROS_BUFFER_SIZE];
    uint16_t write_idx;
    uint16_t read_idx;
    uint16_t count;
    
    /* Statistics for monitoring */
    uint32_t total_samples_written;
    uint32_t buffer_overruns;
    float last_sample_rate;
    uint32_t last_timestamp_ms;
    
    /* Safety flags */
    bool initialized;
    bool has_valid_data;
} choros_ring_buffer_t;

/* Initialize the Choros buffer */
extern "C" void choros_buffer_init(void);

/* Add consolidated sensor data to buffer */
void choros_buffer_add_consolidated(void* consolidated_data);

/* Get latest calculated metrics (with fallback to old values) */
float choros_get_gct_safe(float fallback_value);
float choros_get_stride_length_safe(float fallback_value);
float choros_get_pronation_safe(float fallback_value);
float choros_get_cadence_safe(float fallback_value);

/* Debug functions */
void choros_buffer_print_stats(void);
bool choros_buffer_is_healthy(void);

#ifdef __cplusplus
}
#endif

#endif /* CHOROS_BUFFER_HPP */
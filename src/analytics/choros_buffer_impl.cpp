/**
 * @file choros_buffer_impl.cpp
 * @brief Implementation of Choros-compatible ring buffer
 * @version 1.0
 * @date 2025
 *
 * SAFETY: All functions have fallback to preserve existing functionality
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

#include <choros_buffer.hpp>
#include <app.hpp>

LOG_MODULE_REGISTER(choros_buffer, CONFIG_ANALYTICS_MODULE_LOG_LEVEL);

/* Global Choros buffer instance - defined in analytics.cpp */
extern choros_ring_buffer_t choros_buffer;

/* Initialize the Choros buffer */
void choros_buffer_init(void)
{
    memset(&choros_buffer, 0, sizeof(choros_buffer));
    choros_buffer.initialized = true;
    LOG_INF("Choros buffer initialized: %d samples (%.1f seconds at %dHz)",
            CHOROS_BUFFER_SIZE,
            (double)CHOROS_BUFFER_SECONDS,
            CHOROS_SAMPLE_RATE_HZ);
}

/* Add consolidated sensor data to buffer - NOW WITH REAL RAW PRESSURE DATA */
void choros_buffer_add_consolidated(void* consolidated_data)
{
    if (!choros_buffer.initialized || !consolidated_data) {
        return;  /* Safety check */
    }
    
    sensor_data_consolidated_t* data = (sensor_data_consolidated_t*)consolidated_data;
    
    #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    /* PRIMARY DEVICE: Add BOTH feet to buffer (like Choros expects) */
    
    /* Check if we have valid LEFT foot data based on raw pressure sum */
    uint32_t left_total = 0;
    for (int i = 0; i < 8; i++) {
        left_total += data->pressure_left[i];
    }
    bool has_left_data = (left_total > 0) || data->left_in_contact;
    
    /* Check if we have valid RIGHT foot data */
    uint32_t right_total = 0;
    for (int i = 0; i < 8; i++) {
        right_total += data->pressure_right[i];
    }
    bool has_right_data = (right_total > 0) || data->right_in_contact;
    
    /* Add LEFT foot sample if available */
    if (has_left_data) {
        choros_imu_data_t* left_sample = &choros_buffer.samples[choros_buffer.write_idx];
        
        /* Fill motion sensor data from arrays */
        left_sample->acc_x = data->linear_acc[0];
        left_sample->acc_y = data->linear_acc[1];
        left_sample->acc_z = data->linear_acc[2];
        left_sample->gyro_x = data->gyro[0];
        left_sample->gyro_y = data->gyro[1];
        left_sample->gyro_z = data->gyro[2];
        left_sample->mag_x = 0.0f;
        left_sample->mag_y = 0.0f;
        left_sample->mag_z = 0.0f;
        
        /* Use real timestamp */
        left_sample->timestamp = data->timestamp_ms / 1000.0f;
        
        /* Use REAL raw pressure data */
        for (int i = 0; i < 8; i++) {
            left_sample->insole_pressures[i] = (float)data->pressure_left[i];
        }
        
        /* Mark as LEFT foot */
        left_sample->foot = 0;  /* LEFT = 0 */
        strcpy(left_sample->shoe, "left");
        
        /* Metadata */
        left_sample->battery = 100.0f;
        left_sample->reverse_pressure_order = false;
        left_sample->orientation = 1;  /* Side mount */
        
        /* Update buffer index */
        choros_buffer.write_idx = (choros_buffer.write_idx + 1) % CHOROS_BUFFER_SIZE;
        if (choros_buffer.count < CHOROS_BUFFER_SIZE) {
            choros_buffer.count++;
        } else {
            choros_buffer.buffer_overruns++;
        }
        choros_buffer.total_samples_written++;
    }
    
    /* Add RIGHT foot sample if available */
    if (has_right_data) {
        choros_imu_data_t* right_sample = &choros_buffer.samples[choros_buffer.write_idx];
        
        /* Fill motion sensor data */
        right_sample->acc_x = data->linear_acc[0];
        right_sample->acc_y = data->linear_acc[1];
        right_sample->acc_z = data->linear_acc[2];
        right_sample->gyro_x = data->gyro[0];
        right_sample->gyro_y = data->gyro[1];
        right_sample->gyro_z = data->gyro[2];
        right_sample->mag_x = 0.0f;
        right_sample->mag_y = 0.0f;
        right_sample->mag_z = 0.0f;
        
        /* Use real timestamp */
        right_sample->timestamp = data->timestamp_ms / 1000.0f;
        
        /* Use REAL raw pressure data */
        for (int i = 0; i < 8; i++) {
            right_sample->insole_pressures[i] = (float)data->pressure_right[i];
        }
        
        /* Mark as RIGHT foot */
        right_sample->foot = 1;  /* RIGHT = 1 */
        strcpy(right_sample->shoe, "right");
        
        /* Metadata */
        right_sample->battery = 100.0f;
        right_sample->reverse_pressure_order = false;
        right_sample->orientation = 1;  /* Side mount */
        
        /* Update buffer index */
        choros_buffer.write_idx = (choros_buffer.write_idx + 1) % CHOROS_BUFFER_SIZE;
        if (choros_buffer.count < CHOROS_BUFFER_SIZE) {
            choros_buffer.count++;
        } else {
            choros_buffer.buffer_overruns++;
        }
        choros_buffer.total_samples_written++;
    }
    
    #else
    /* SECONDARY DEVICE: Only add LEFT foot (local) */
    choros_imu_data_t* sample = &choros_buffer.samples[choros_buffer.write_idx];
    
    /* Fill motion sensor data */
    sample->acc_x = data->linear_acc[0];
    sample->acc_y = data->linear_acc[1];
    sample->acc_z = data->linear_acc[2];
    sample->gyro_x = data->gyro[0];
    sample->gyro_y = data->gyro[1];
    sample->gyro_z = data->gyro[2];
    sample->mag_x = 0.0f;
    sample->mag_y = 0.0f;
    sample->mag_z = 0.0f;
    
    /* Use real timestamp */
    sample->timestamp = data->timestamp_ms / 1000.0f;
    
    /* Use REAL raw pressure data for LEFT foot */
    for (int i = 0; i < 8; i++) {
        sample->insole_pressures[i] = (float)data->pressure_left[i];
    }
    
    /* Mark as LEFT foot */
    sample->foot = 0;  /* LEFT = 0 */
    strcpy(sample->shoe, "left");
    
    /* Metadata */
    sample->battery = 100.0f;
    sample->reverse_pressure_order = false;
    sample->orientation = 1;  /* Side mount */
    
    /* Update buffer index */
    choros_buffer.write_idx = (choros_buffer.write_idx + 1) % CHOROS_BUFFER_SIZE;
    if (choros_buffer.count < CHOROS_BUFFER_SIZE) {
        choros_buffer.count++;
    } else {
        choros_buffer.buffer_overruns++;
    }
    choros_buffer.total_samples_written++;
    #endif
    
    /* Update buffer management */
    uint32_t now = k_uptime_get_32();
    if (choros_buffer.last_timestamp_ms > 0) {
        float dt = (now - choros_buffer.last_timestamp_ms) / 1000.0f;
        if (dt > 0) {
            choros_buffer.last_sample_rate = 1.0f / dt;
        }
    }
    choros_buffer.last_timestamp_ms = now;
    
    /* Mark as having valid data after minimum samples */
    if (choros_buffer.count >= 20) {
        if (!choros_buffer.has_valid_data) {
            LOG_INF("Choros buffer ready! Have %d samples", choros_buffer.count);
        }
        choros_buffer.has_valid_data = true;
    }
    
    /* Log periodically */
    static uint32_t last_log = 0;
    if (now - last_log > 5000) {  // Log every 5 seconds
        LOG_INF("Choros buffer: %d samples, %.1f Hz",
                choros_buffer.count,
                (double)choros_buffer.last_sample_rate);
        last_log = now;
    }
}

/* Calculate GCT from Choros buffer with fallback - now handles BOTH feet */
float choros_get_gct_safe(float fallback_value)
{
    /* Safety check - return fallback if not ready */
    if (!choros_buffer.initialized || !choros_buffer.has_valid_data ||
        choros_buffer.count < 100) {
        return fallback_value;
    }
    
    /* Track IC->TO for BOTH feet separately */
    bool left_in_contact = false, right_in_contact = false;
    float left_ic_time = 0, right_ic_time = 0;
    float left_gct = fallback_value, right_gct = fallback_value;
    
    /* Process last 100 samples (~0.5 second at 200Hz) */
    int samples_to_check = MIN(choros_buffer.count, 100);
    int start_idx = (choros_buffer.write_idx - samples_to_check + CHOROS_BUFFER_SIZE) % CHOROS_BUFFER_SIZE;
    
    for (int i = 0; i < samples_to_check; i++) {
        int idx = (start_idx + i) % CHOROS_BUFFER_SIZE;
        choros_imu_data_t* sample = &choros_buffer.samples[idx];
        
        /* Calculate total pressure */
        float total_pressure = 0;
        for (int j = 0; j < 8; j++) {
            total_pressure += sample->insole_pressures[j];
        }
        
        /* Process based on which foot this sample is from */
        if (sample->foot == 0) {  /* LEFT foot */
            /* Detect IC (Initial Contact) */
            if (!left_in_contact && total_pressure > 1000.0f) {
                left_ic_time = sample->timestamp;
                left_in_contact = true;
            }
            /* Detect TO (Toe Off) */
            else if (left_in_contact && total_pressure < 1000.0f) {
                float to_time = sample->timestamp;
                left_gct = (to_time - left_ic_time) * 1000.0f;  /* Convert to ms */
                left_in_contact = false;
                
                /* Sanity check - GCT should be 100-500ms */
                if (left_gct >= 100.0f && left_gct <= 500.0f) {
                    LOG_WRN("Choros LEFT GCT: %.1f ms", (double)left_gct);
                }
            }
        } else if (sample->foot == 1) {  /* RIGHT foot */
            /* Detect IC (Initial Contact) */
            if (!right_in_contact && total_pressure > 1000.0f) {
                right_ic_time = sample->timestamp;
                right_in_contact = true;
            }
            /* Detect TO (Toe Off) */
            else if (right_in_contact && total_pressure < 1000.0f) {
                float to_time = sample->timestamp;
                right_gct = (to_time - right_ic_time) * 1000.0f;  /* Convert to ms */
                right_in_contact = false;
                
                /* Sanity check - GCT should be 100-500ms */
                if (right_gct >= 100.0f && right_gct <= 500.0f) {
                    LOG_WRN("Choros RIGHT GCT: %.1f ms", (double)right_gct);
                }
            }
        }
    }
    
    /* Return average of both feet if both valid, or single foot, or fallback */
    bool left_valid = (left_gct >= 100.0f && left_gct <= 500.0f);
    bool right_valid = (right_gct >= 100.0f && right_gct <= 500.0f);
    
    if (left_valid && right_valid) {
        return (left_gct + right_gct) / 2.0f;  /* Average of both */
    } else if (left_valid) {
        return left_gct;
    } else if (right_valid) {
        return right_gct;
    }
    
    /* No complete contact found, return fallback */
    return fallback_value;
}

/* Calculate stride length from Choros buffer with fallback */
float choros_get_stride_length_safe(float fallback_value)
{
    /* Safety check */
    if (!choros_buffer.initialized || !choros_buffer.has_valid_data || 
        choros_buffer.count < 200) {
        return fallback_value;
    }
    
    /* Find stride boundaries (IC to next IC) */
    int first_ic = -1, second_ic = -1;
    int samples_to_check = MIN(choros_buffer.count, 400);
    int start_idx = (choros_buffer.write_idx - samples_to_check + CHOROS_BUFFER_SIZE) % CHOROS_BUFFER_SIZE;
    
    for (int i = 0; i < samples_to_check - 1; i++) {
        int idx = (start_idx + i) % CHOROS_BUFFER_SIZE;
        choros_imu_data_t* sample = &choros_buffer.samples[idx];
        
        float pressure = 0;
        for (int j = 0; j < 8; j++) {
            pressure += sample->insole_pressures[j];
        }
        
        /* Detect IC events */
        if (pressure > 1000.0f) {
            int next_idx = (idx + 1) % CHOROS_BUFFER_SIZE;
            choros_imu_data_t* next_sample = &choros_buffer.samples[next_idx];
            
            float next_pressure = 0;
            for (int j = 0; j < 8; j++) {
                next_pressure += next_sample->insole_pressures[j];
            }
            
            /* Rising edge detection */
            if (next_pressure <= 1000.0f) {
                if (first_ic == -1) {
                    first_ic = i;
                } else if (second_ic == -1) {
                    second_ic = i;
                    break;
                }
            }
        }
    }
    
    /* Calculate stride length if we found boundaries */
    if (first_ic >= 0 && second_ic > first_ic) {
        float distance = 0;
        
        /* Simple velocity integration */
        for (int i = first_ic; i < second_ic && i < samples_to_check - 1; i++) {
            int idx = (start_idx + i) % CHOROS_BUFFER_SIZE;
            int next_idx = (start_idx + i + 1) % CHOROS_BUFFER_SIZE;
            
            choros_imu_data_t* sample = &choros_buffer.samples[idx];
            choros_imu_data_t* next_sample = &choros_buffer.samples[next_idx];
            
            float dt = next_sample->timestamp - sample->timestamp;
            if (dt > 0 && dt < 0.1f) {  /* Sanity check dt */
                /* Simple integration: assume velocity proportional to acceleration */
                float vel_x = sample->acc_x * dt;
                distance += fabsf(vel_x * dt);
            }
        }
        
        /* Sanity check - stride should be 0.5-2.0m */
        if (distance > 0.5f && distance < 2.0f) {
            LOG_WRN("Choros stride length: %.2f m", distance);
            return distance;
        }
    }
    
    return fallback_value;
}

/* Calculate pronation angle from Choros buffer with fallback */
float choros_get_pronation_safe(float fallback_value)
{
    /* Safety check */
    if (!choros_buffer.initialized || !choros_buffer.has_valid_data || 
        choros_buffer.count < 50) {
        return fallback_value;
    }
    
    /* Find most recent IC event and get foot angle */
    int samples_to_check = MIN(choros_buffer.count, 100);
    int start_idx = (choros_buffer.write_idx - samples_to_check + CHOROS_BUFFER_SIZE) % CHOROS_BUFFER_SIZE;
    
    for (int i = samples_to_check - 2; i >= 0; i--) {
        int idx = (start_idx + i) % CHOROS_BUFFER_SIZE;
        int next_idx = (start_idx + i + 1) % CHOROS_BUFFER_SIZE;
        
        choros_imu_data_t* sample = &choros_buffer.samples[idx];
        choros_imu_data_t* next_sample = &choros_buffer.samples[next_idx];
        
        float pressure = 0, next_pressure = 0;
        for (int j = 0; j < 8; j++) {
            pressure += sample->insole_pressures[j];
            next_pressure += next_sample->insole_pressures[j];
        }
        
        /* Detect IC (rising edge) */
        if (pressure <= 1000.0f && next_pressure > 1000.0f) {
            /* Calculate roll angle from gyro integration (simplified) */
            float roll_angle = 0;
            float pitch_angle = 0;
            
            /* Integrate gyro over last 50ms before IC */
            for (int j = MAX(0, i - 10); j < i; j++) {
                int gyro_idx = (start_idx + j) % CHOROS_BUFFER_SIZE;
                choros_imu_data_t* gyro_sample = &choros_buffer.samples[gyro_idx];
                
                roll_angle += gyro_sample->gyro_x * 0.005f;  /* Assume 5ms per sample */
                pitch_angle += gyro_sample->gyro_y * 0.005f;
            }
            
            /* Convert to degrees */
            float pronation_deg = roll_angle * 180.0f / 3.14159f;
            
            /* Sanity check - pronation should be -30 to +30 degrees */
            if (pronation_deg > -30.0f && pronation_deg < 30.0f) {
                LOG_WRN("Choros pronation: %.1f deg", pronation_deg);
                return pronation_deg;
            }
        }
    }
    
    return fallback_value;
}

/* Calculate cadence from Choros buffer with fallback */
float choros_get_cadence_safe(float fallback_value)
{
    /* Safety check */
    if (!choros_buffer.initialized || !choros_buffer.has_valid_data || 
        choros_buffer.count < 200) {
        return fallback_value;
    }
    
    /* Count strides in buffer */
    int stride_count = 0;
    float first_ic_time = 0, last_ic_time = 0;
    
    int samples_to_check = MIN(choros_buffer.count, CHOROS_BUFFER_SIZE);
    int start_idx = (choros_buffer.write_idx - samples_to_check + CHOROS_BUFFER_SIZE) % CHOROS_BUFFER_SIZE;
    
    for (int i = 0; i < samples_to_check - 1; i++) {
        int idx = (start_idx + i) % CHOROS_BUFFER_SIZE;
        int next_idx = (start_idx + i + 1) % CHOROS_BUFFER_SIZE;
        
        choros_imu_data_t* sample = &choros_buffer.samples[idx];
        choros_imu_data_t* next_sample = &choros_buffer.samples[next_idx];
        
        float pressure = 0, next_pressure = 0;
        for (int j = 0; j < 8; j++) {
            pressure += sample->insole_pressures[j];
            next_pressure += next_sample->insole_pressures[j];
        }
        
        /* Detect IC events */
        if (pressure <= 1000.0f && next_pressure > 1000.0f) {
            if (first_ic_time == 0) {
                first_ic_time = next_sample->timestamp;
            }
            last_ic_time = next_sample->timestamp;
            stride_count++;
        }
    }
    
    /* Calculate cadence if we have enough strides */
    if (stride_count >= 2 && last_ic_time > first_ic_time) {
        float duration_min = (last_ic_time - first_ic_time) / 60.0f;
        float cadence = (float)(stride_count * 2) / duration_min;  /* x2 for steps */
        
        /* Sanity check - cadence should be 120-220 spm */
        if (cadence >= 120.0f && cadence <= 220.0f) {
            LOG_WRN("Choros cadence: %.1f spm", cadence);
            return cadence;
        }
    }
    
    return fallback_value;
}

/* Print buffer statistics for debugging */
void choros_buffer_print_stats(void)
{
    LOG_INF("Choros Buffer Stats:");
    LOG_INF("  Samples: %d/%d", choros_buffer.count, CHOROS_BUFFER_SIZE);
    LOG_INF("  Total written: %u", choros_buffer.total_samples_written);
    LOG_INF("  Overruns: %u", choros_buffer.buffer_overruns);
    LOG_INF("  Sample rate: %.1f Hz", choros_buffer.last_sample_rate);
    LOG_INF("  Initialized: %s", choros_buffer.initialized ? "Yes" : "No");
    LOG_INF("  Has valid data: %s", choros_buffer.has_valid_data ? "Yes" : "No");
}

/* Check if buffer is healthy */
bool choros_buffer_is_healthy(void)
{
    return choros_buffer.initialized && 
           choros_buffer.has_valid_data &&
           choros_buffer.count >= 100 &&
           choros_buffer.last_sample_rate > 50.0f &&
           choros_buffer.last_sample_rate < 300.0f;
}
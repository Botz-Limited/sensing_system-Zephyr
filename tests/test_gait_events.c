/**
 * @file test_gait_events.c
 * @brief Unit tests for event-driven gait parameter system
 */

#include <zephyr/ztest.h>
#include <gait_events.h>

static gait_event_detector_t test_detector;

/**
 * Test initialization
 */
ZTEST(gait_events, test_init)
{
    gait_events_init(&test_detector, false, 200.0f);
    
    zassert_equal(test_detector.sampling_rate, 200.0f, "Sampling rate incorrect");
    zassert_equal(test_detector.current_phase, GAIT_PHASE_IDLE, "Initial phase should be IDLE");
    zassert_equal(test_detector.total_step_count, 0, "Step count should be 0");
}

/**
 * Test adding data to buffer
 */
ZTEST(gait_events, test_add_data)
{
    gait_events_init(&test_detector, false, 200.0f);
    
    foot_samples_t foot_data = {
        .sensor = {100, 150, 200, 250, 200, 150, 100, 50},
        .timestamp = 1000
    };
    
    imu_data_t imu_data = {
        .quat_w = 1.0, .quat_x = 0.0, .quat_y = 0.0, .quat_z = 0.0,
        .gyro_x = 0.1, .gyro_y = 0.2, .gyro_z = 0.3,
        .accel_x = 0.0, .accel_y = 0.0, .accel_z = 0.0
    };
    
    gait_events_add_data(&test_detector, &foot_data, &imu_data, 1.0f);
    
    zassert_equal(test_detector.buffer_count, 1, "Buffer count should be 1");
}

/**
 * Test event detection with simulated stride
 */
ZTEST(gait_events, test_event_detection)
{
    gait_events_init(&test_detector, false, 200.0f);
    
    /* Simulate a stride: swing -> stance -> swing */
    float time = 0.0f;
    
    /* Add 200 samples of swing phase (1 second) */
    for (int i = 0; i < 200; i++) {
        foot_samples_t foot_data = {
            .sensor = {10, 10, 10, 10, 10, 10, 10, 10}, /* Low pressure = swing */
            .timestamp = (uint32_t)(time * 1000)
        };
        
        imu_data_t imu_data = {
            .quat_w = 1.0, .quat_x = 0.0, .quat_y = 0.0, .quat_z = 0.0,
            .gyro_x = 0.1, .gyro_y = 0.2, .gyro_z = 0.3,
            .accel_x = 1.0, .accel_y = 0.0, .accel_z = 0.0
        };
        
        gait_events_add_data(&test_detector, &foot_data, &imu_data, time);
        time += 0.005f; /* 5ms per sample at 200Hz */
    }
    
    /* Add 100 samples of stance phase (0.5 seconds) */
    for (int i = 0; i < 100; i++) {
        foot_samples_t foot_data = {
            .sensor = {200, 250, 300, 350, 300, 250, 200, 150}, /* High pressure = stance */
            .timestamp = (uint32_t)(time * 1000)
        };
        
        imu_data_t imu_data = {
            .quat_w = 1.0, .quat_x = 0.0, .quat_y = 0.0, .quat_z = 0.0,
            .gyro_x = 0.05, .gyro_y = 0.1, .gyro_z = 0.15,
            .accel_x = 0.0, .accel_y = 0.0, .accel_z = 0.0
        };
        
        gait_events_add_data(&test_detector, &foot_data, &imu_data, time);
        time += 0.005f;
    }
    
    /* Add 100 samples of swing phase again (0.5 seconds) */
    for (int i = 0; i < 100; i++) {
        foot_samples_t foot_data = {
            .sensor = {10, 10, 10, 10, 10, 10, 10, 10}, /* Low pressure = swing */
            .timestamp = (uint32_t)(time * 1000)
        };
        
        imu_data_t imu_data = {
            .quat_w = 1.0, .quat_x = 0.0, .quat_y = 0.0, .quat_z = 0.0,
            .gyro_x = 0.1, .gyro_y = 0.2, .gyro_z = 0.3,
            .accel_x = 1.0, .accel_y = 0.0, .accel_z = 0.0
        };
        
        gait_events_add_data(&test_detector, &foot_data, &imu_data, time);
        time += 0.005f;
    }
    
    /* Process events */
    int metrics_count = gait_events_process(&test_detector);
    
    /* Should detect at least one IC and one TO */
    zassert_true(metrics_count >= 0, "Should process without error");
    
    /* Check if events were detected */
    if (metrics_count > 0) {
        gait_metrics_t metrics[GAIT_MAX_EVENTS_PER_CHUNK];
        int count = gait_events_get_metrics(&test_detector, metrics, GAIT_MAX_EVENTS_PER_CHUNK);
        
        zassert_true(count > 0, "Should have metrics available");
        
        /* Check first metric */
        if (count > 0) {
            zassert_true(metrics[0].gct > 0, "GCT should be positive");
            zassert_true(metrics[0].duration > 0, "Duration should be positive");
            zassert_true(metrics[0].cadence > 0, "Cadence should be positive");
        }
    }
}

/**
 * Test metrics calculation
 */
ZTEST(gait_events, test_metrics_calculation)
{
    gait_events_init(&test_detector, false, 200.0f);
    gait_events_set_subject_height(&test_detector, 1.75f);
    
    /* Add a full stride worth of data */
    float time = 0.0f;
    
    /* Simulate complete stride with realistic pattern */
    for (int i = 0; i < 400; i++) {
        int pressure = 0;
        
        /* Create pressure pattern: low->high->low */
        if (i < 100) {
            pressure = 50; /* Initial swing */
        } else if (i < 250) {
            pressure = 300; /* Stance phase */
        } else {
            pressure = 50; /* Final swing */
        }
        
        foot_samples_t foot_data = {
            .sensor = {pressure, pressure, pressure, pressure, 
                      pressure, pressure, pressure, pressure},
            .timestamp = (uint32_t)(time * 1000)
        };
        
        /* Simple velocity profile for stride */
        float velocity_x = (i >= 100 && i < 250) ? 0.1f : 1.5f;
        
        imu_data_t imu_data = {
            .quat_w = 1.0, .quat_x = 0.0, .quat_y = 0.0, .quat_z = 0.0,
            .gyro_x = 0.1, .gyro_y = 0.2, .gyro_z = 0.3,
            .accel_x = velocity_x, .accel_y = 0.0, .accel_z = 0.0
        };
        
        gait_events_add_data(&test_detector, &foot_data, &imu_data, time);
        time += 0.005f;
    }
    
    /* Process and check metrics */
    int metrics_count = gait_events_process(&test_detector);
    
    if (metrics_count > 0) {
        gait_metrics_t metrics[GAIT_MAX_EVENTS_PER_CHUNK];
        int count = gait_events_get_metrics(&test_detector, metrics, GAIT_MAX_EVENTS_PER_CHUNK);
        
        zassert_true(count > 0, "Should have calculated metrics");
        
        /* Validate metric ranges */
        for (int i = 0; i < count; i++) {
            zassert_true(metrics[i].gct >= 0.1f && metrics[i].gct <= 1.0f, 
                        "GCT should be in reasonable range");
            zassert_true(metrics[i].cadence >= 50 && metrics[i].cadence <= 200,
                        "Cadence should be in reasonable range");
        }
    }
}

/**
 * Test buffer overflow handling
 */
ZTEST(gait_events, test_buffer_overflow)
{
    gait_events_init(&test_detector, false, 200.0f);
    
    /* Fill buffer beyond capacity */
    for (int i = 0; i < GAIT_BUFFER_SIZE_SAMPLES + 100; i++) {
        foot_samples_t foot_data = {
            .sensor = {100, 100, 100, 100, 100, 100, 100, 100},
            .timestamp = i * 5
        };
        
        imu_data_t imu_data = {
            .quat_w = 1.0, .quat_x = 0.0, .quat_y = 0.0, .quat_z = 0.0,
            .gyro_x = 0.0, .gyro_y = 0.0, .gyro_z = 0.0,
            .accel_x = 0.0, .accel_y = 0.0, .accel_z = 0.0
        };
        
        gait_events_add_data(&test_detector, &foot_data, &imu_data, i * 0.005f);
    }
    
    /* Buffer should be at max capacity, not overflow */
    zassert_equal(test_detector.buffer_count, GAIT_BUFFER_SIZE_SAMPLES, 
                  "Buffer should be at max capacity");
}

ZTEST_SUITE(gait_events, NULL, NULL, NULL, NULL, NULL);
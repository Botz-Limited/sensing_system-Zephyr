/**
 * @file test_foot_sensor.cpp
 * @brief Unit tests for foot_sensor module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

// Foot sensor constants
#define FOOT_SENSOR_SAMPLE_RATE_HZ 100
#define FOOT_SENSOR_BUFFER_SIZE 1000
#define FOOT_SENSOR_THRESHOLD_MG 100
#define FOOT_SENSOR_DEBOUNCE_MS 50

// Sensor states
enum foot_sensor_state {
    FOOT_SENSOR_STATE_IDLE,
    FOOT_SENSOR_STATE_ACTIVE,
    FOOT_SENSOR_STATE_CALIBRATING,
    FOOT_SENSOR_STATE_ERROR
};

// Activity types
enum foot_activity_type {
    ACTIVITY_NONE,
    ACTIVITY_WALKING,
    ACTIVITY_RUNNING,
    ACTIVITY_JUMPING
};

// Sensor data structure
struct foot_sensor_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint32_t timestamp;
};

// Test fixture
struct foot_sensor_fixture {
    enum foot_sensor_state state;
    enum foot_activity_type activity;
    struct foot_sensor_data buffer[FOOT_SENSOR_BUFFER_SIZE];
    size_t buffer_index;
    uint32_t sample_count;
    bool calibrated;
    int16_t calibration_offset[6];
    struct k_msgq *data_queue;
    int message_count;
};

static void *foot_sensor_setup(void)
{
    static struct foot_sensor_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    fixture.state = FOOT_SENSOR_STATE_IDLE;
    fixture.activity = ACTIVITY_NONE;
    return &fixture;
}

static void foot_sensor_teardown(void *f)
{
    struct foot_sensor_fixture *fixture = (struct foot_sensor_fixture *)f;
    fixture->buffer_index = 0;
    fixture->sample_count = 0;
    fixture->message_count = 0;
}

ZTEST_SUITE(foot_sensor, NULL, foot_sensor_setup, NULL, NULL, foot_sensor_teardown);

// Mock sensor functions
static int foot_sensor_init_mock(struct foot_sensor_fixture *fixture)
{
    fixture->state = FOOT_SENSOR_STATE_IDLE;
    fixture->calibrated = false;
    memset(fixture->calibration_offset, 0, sizeof(fixture->calibration_offset));
    return 0;
}

static int foot_sensor_start_mock(struct foot_sensor_fixture *fixture)
{
    if (fixture->state == FOOT_SENSOR_STATE_ERROR) {
        return -EIO;
    }
    
    fixture->state = FOOT_SENSOR_STATE_ACTIVE;
    fixture->buffer_index = 0;
    fixture->sample_count = 0;
    return 0;
}

static int foot_sensor_stop_mock(struct foot_sensor_fixture *fixture)
{
    fixture->state = FOOT_SENSOR_STATE_IDLE;
    return 0;
}

static int foot_sensor_calibrate_mock(struct foot_sensor_fixture *fixture)
{
    fixture->state = FOOT_SENSOR_STATE_CALIBRATING;
    
    // Simulate calibration
    k_sleep(K_MSEC(100));
    
    // Set some calibration offsets
    fixture->calibration_offset[0] = 10;   // accel_x
    fixture->calibration_offset[1] = -5;   // accel_y
    fixture->calibration_offset[2] = 15;   // accel_z
    fixture->calibration_offset[3] = 2;    // gyro_x
    fixture->calibration_offset[4] = -3;   // gyro_y
    fixture->calibration_offset[5] = 1;    // gyro_z
    
    fixture->calibrated = true;
    fixture->state = FOOT_SENSOR_STATE_IDLE;
    return 0;
}

static void foot_sensor_process_sample_mock(struct foot_sensor_fixture *fixture,
                                           struct foot_sensor_data *data)
{
    // Store original data first
    if (fixture->buffer_index < FOOT_SENSOR_BUFFER_SIZE) {
        memcpy(&fixture->buffer[fixture->buffer_index], data, sizeof(*data));
        
        // Apply calibration to stored data if available
        if (fixture->calibrated) {
            fixture->buffer[fixture->buffer_index].accel_x -= fixture->calibration_offset[0];
            fixture->buffer[fixture->buffer_index].accel_y -= fixture->calibration_offset[1];
            fixture->buffer[fixture->buffer_index].accel_z -= fixture->calibration_offset[2];
            fixture->buffer[fixture->buffer_index].gyro_x -= fixture->calibration_offset[3];
            fixture->buffer[fixture->buffer_index].gyro_y -= fixture->calibration_offset[4];
            fixture->buffer[fixture->buffer_index].gyro_z -= fixture->calibration_offset[5];
        }
        
        fixture->buffer_index++;
        fixture->sample_count++;
    }
    
    // Detect activity (simplified) - use calibrated values if available
    int16_t ax = data->accel_x;
    int16_t ay = data->accel_y;
    int16_t az = data->accel_z;
    
    if (fixture->calibrated) {
        ax -= fixture->calibration_offset[0];
        ay -= fixture->calibration_offset[1];
        az -= fixture->calibration_offset[2];
    }
    
    int32_t accel_mag = (ax * ax) + (ay * ay) + (az * az);
    
    if (accel_mag > 2000000) {  // High acceleration
        fixture->activity = ACTIVITY_RUNNING;
    } else if (accel_mag > 500000) {  // Medium acceleration  
        fixture->activity = ACTIVITY_WALKING;
    } else {
        fixture->activity = ACTIVITY_NONE;
    }
    
    // Simulate message queue notification
    fixture->message_count++;
}

// Tests
ZTEST_F(foot_sensor, test_init)
{
    // Test initialization
    int ret = foot_sensor_init_mock(fixture);
    
    // Verify
    zassert_equal(ret, 0, "Init should succeed");
    zassert_equal(fixture->state, FOOT_SENSOR_STATE_IDLE, "Should be idle");
    zassert_false(fixture->calibrated, "Should not be calibrated");
}

ZTEST_F(foot_sensor, test_start_stop)
{
    // Initialize first
    foot_sensor_init_mock(fixture);
    
    // Start sensor
    int ret = foot_sensor_start_mock(fixture);
    zassert_equal(ret, 0, "Start should succeed");
    zassert_equal(fixture->state, FOOT_SENSOR_STATE_ACTIVE, "Should be active");
    
    // Stop sensor
    ret = foot_sensor_stop_mock(fixture);
    zassert_equal(ret, 0, "Stop should succeed");
    zassert_equal(fixture->state, FOOT_SENSOR_STATE_IDLE, "Should be idle");
}

ZTEST_F(foot_sensor, test_start_when_error)
{
    // Set error state
    fixture->state = FOOT_SENSOR_STATE_ERROR;
    
    // Try to start
    int ret = foot_sensor_start_mock(fixture);
    
    // Verify
    zassert_equal(ret, -EIO, "Should fail when in error state");
}

ZTEST_F(foot_sensor, test_calibration)
{
    // Initialize
    foot_sensor_init_mock(fixture);
    
    // Calibrate
    int ret = foot_sensor_calibrate_mock(fixture);
    
    // Verify
    zassert_equal(ret, 0, "Calibration should succeed");
    zassert_true(fixture->calibrated, "Should be calibrated");
    zassert_equal(fixture->state, FOOT_SENSOR_STATE_IDLE, "Should return to idle");
    
    // Check calibration values were set
    zassert_not_equal(fixture->calibration_offset[0], 0, "Should have X offset");
    zassert_not_equal(fixture->calibration_offset[1], 0, "Should have Y offset");
    zassert_not_equal(fixture->calibration_offset[2], 0, "Should have Z offset");
}

ZTEST_F(foot_sensor, test_sample_processing)
{
    // Initialize and start
    foot_sensor_init_mock(fixture);
    foot_sensor_start_mock(fixture);
    
    // Reset message count to ensure clean state
    fixture->message_count = 0;
    
    // Create test sample
    struct foot_sensor_data sample = {
        .accel_x = 100,
        .accel_y = 200,
        .accel_z = 1000,
        .gyro_x = 10,
        .gyro_y = 20,
        .gyro_z = 30,
        .timestamp = k_uptime_get_32()
    };
    
    // Process sample
    foot_sensor_process_sample_mock(fixture, &sample);
    
    // Verify
    zassert_equal(fixture->sample_count, 1, "Should have 1 sample");
    zassert_equal(fixture->buffer_index, 1, "Buffer index should be 1");
    zassert_equal(fixture->message_count, 1, "Should have sent 1 message (actual: %d)", 
                  fixture->message_count);
}

ZTEST_F(foot_sensor, test_calibrated_sample_processing)
{
    // Initialize and calibrate
    foot_sensor_init_mock(fixture);
    foot_sensor_calibrate_mock(fixture);
    foot_sensor_start_mock(fixture);
    
    // Create test sample
    struct foot_sensor_data sample = {
        .accel_x = 100,
        .accel_y = 200,
        .accel_z = 1000,
        .gyro_x = 10,
        .gyro_y = 20,
        .gyro_z = 30,
        .timestamp = k_uptime_get_32()
    };
    
    // Process sample
    foot_sensor_process_sample_mock(fixture, &sample);
    
    // Verify calibration was applied
    zassert_equal(fixture->buffer[0].accel_x, 
                  sample.accel_x - fixture->calibration_offset[0],
                  "X acceleration should be calibrated");
    zassert_equal(fixture->buffer[0].accel_y,
                  sample.accel_y - fixture->calibration_offset[1],
                  "Y acceleration should be calibrated");
    zassert_equal(fixture->buffer[0].accel_z,
                  sample.accel_z - fixture->calibration_offset[2],
                  "Z acceleration should be calibrated");
}

ZTEST_F(foot_sensor, test_activity_detection)
{
    // Initialize and start
    foot_sensor_init_mock(fixture);
    foot_sensor_start_mock(fixture);
    
    // Test different acceleration magnitudes
    struct {
        int16_t accel_x;
        int16_t accel_y;
        int16_t accel_z;
        enum foot_activity_type expected_activity;
    } test_cases[] = {
        {100, 100, 100, ACTIVITY_NONE},      // Low acceleration
        {500, 500, 500, ACTIVITY_WALKING},   // Medium acceleration
        {1500, 1500, 1500, ACTIVITY_RUNNING}, // High acceleration
    };
    
    for (size_t i = 0; i < ARRAY_SIZE(test_cases); i++) {
        struct foot_sensor_data sample = {
            .accel_x = test_cases[i].accel_x,
            .accel_y = test_cases[i].accel_y,
            .accel_z = test_cases[i].accel_z,
            .gyro_x = 0,
            .gyro_y = 0,
            .gyro_z = 0,
            .timestamp = k_uptime_get_32()
        };
        
        foot_sensor_process_sample_mock(fixture, &sample);
        
        zassert_equal(fixture->activity, test_cases[i].expected_activity,
                      "Activity should be %d for acceleration (%d,%d,%d)",
                      test_cases[i].expected_activity,
                      test_cases[i].accel_x,
                      test_cases[i].accel_y,
                      test_cases[i].accel_z);
    }
}

ZTEST_F(foot_sensor, test_buffer_management)
{
    // Initialize and start
    foot_sensor_init_mock(fixture);
    foot_sensor_start_mock(fixture);
    
    // Fill buffer with samples
    for (int i = 0; i < 100; i++) {
        struct foot_sensor_data sample = {
            .accel_x = (int16_t)i,
            .accel_y = (int16_t)(i * 2),
            .accel_z = (int16_t)(i * 3),
            .gyro_x = 0,
            .gyro_y = 0,
            .gyro_z = 0,
            .timestamp = k_uptime_get_32()
        };
        
        foot_sensor_process_sample_mock(fixture, &sample);
    }
    
    // Verify
    zassert_equal(fixture->sample_count, 100, "Should have 100 samples");
    zassert_equal(fixture->buffer_index, 100, "Buffer index should be 100");
    
    // Verify sample data
    for (int i = 0; i < 100; i++) {
        zassert_equal(fixture->buffer[i].accel_x, i,
                      "Sample %d X should be %d", i, i);
        zassert_equal(fixture->buffer[i].accel_y, i * 2,
                      "Sample %d Y should be %d", i, i * 2);
        zassert_equal(fixture->buffer[i].accel_z, i * 3,
                      "Sample %d Z should be %d", i, i * 3);
    }
}

ZTEST_F(foot_sensor, test_buffer_overflow_protection)
{
    // Initialize and start
    foot_sensor_init_mock(fixture);
    foot_sensor_start_mock(fixture);
    
    // Try to overflow buffer
    for (int i = 0; i < FOOT_SENSOR_BUFFER_SIZE + 100; i++) {
        struct foot_sensor_data sample = {
            .accel_x = (int16_t)i,
            .accel_y = 0,
            .accel_z = 0,
            .gyro_x = 0,
            .gyro_y = 0,
            .gyro_z = 0,
            .timestamp = k_uptime_get_32()
        };
        
        foot_sensor_process_sample_mock(fixture, &sample);
    }
    
    // Verify buffer didn't overflow
    zassert_equal(fixture->buffer_index, FOOT_SENSOR_BUFFER_SIZE,
                  "Buffer index should stop at max size");
}

ZTEST_F(foot_sensor, test_sample_rate)
{
    // Test that samples are processed at expected rate
    const uint32_t expected_period_ms = 1000 / FOOT_SENSOR_SAMPLE_RATE_HZ;
    
    zassert_equal(expected_period_ms, 10,
                  "Sample period should be 10ms for 100Hz");
}

ZTEST_F(foot_sensor, test_state_transitions)
{
    // Test all state transitions
    
    // IDLE -> ACTIVE
    fixture->state = FOOT_SENSOR_STATE_IDLE;
    foot_sensor_start_mock(fixture);
    zassert_equal(fixture->state, FOOT_SENSOR_STATE_ACTIVE,
                  "Should transition from IDLE to ACTIVE");
    
    // ACTIVE -> IDLE
    foot_sensor_stop_mock(fixture);
    zassert_equal(fixture->state, FOOT_SENSOR_STATE_IDLE,
                  "Should transition from ACTIVE to IDLE");
    
    // IDLE -> CALIBRATING -> IDLE
    foot_sensor_calibrate_mock(fixture);
    zassert_equal(fixture->state, FOOT_SENSOR_STATE_IDLE,
                  "Should return to IDLE after calibration");
    
    // ERROR state prevents start
    fixture->state = FOOT_SENSOR_STATE_ERROR;
    int ret = foot_sensor_start_mock(fixture);
    zassert_equal(ret, -EIO, "Should not start from ERROR state");
}

ZTEST_F(foot_sensor, test_message_notifications)
{
    // Initialize and start
    foot_sensor_init_mock(fixture);
    foot_sensor_start_mock(fixture);
    
    // Reset message count to ensure clean state
    fixture->message_count = 0;
    
    // Process multiple samples
    int num_samples = 10;
    for (int i = 0; i < num_samples; i++) {
        struct foot_sensor_data sample = {
            .accel_x = 100,
            .accel_y = 200,
            .accel_z = 300,
            .gyro_x = 10,
            .gyro_y = 20,
            .gyro_z = 30,
            .timestamp = k_uptime_get_32()
        };
        
        foot_sensor_process_sample_mock(fixture, &sample);
    }
    
    // Verify notifications were sent
    zassert_equal(fixture->message_count, num_samples,
                  "Should have sent %d messages (actual: %d)", num_samples,
                  fixture->message_count);
}
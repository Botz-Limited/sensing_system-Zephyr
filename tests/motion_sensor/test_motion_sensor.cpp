/**
 * @file test_motion_sensor.cpp
 * @brief Unit tests for motion_sensor module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

// Motion sensor constants
#define MOTION_SENSOR_SAMPLE_RATE_HZ 50
#define MOTION_SENSOR_BUFFER_SIZE 500
#define MOTION_SENSOR_THRESHOLD_MG 50
#define MOTION_SENSOR_TIMEOUT_MS 5000

// Sensor states
enum motion_sensor_state {
    MOTION_SENSOR_STATE_IDLE,
    MOTION_SENSOR_STATE_ACTIVE,
    MOTION_SENSOR_STATE_CALIBRATING,
    MOTION_SENSOR_STATE_ERROR,
    MOTION_SENSOR_STATE_LOW_POWER
};

// Motion types
enum motion_type {
    MOTION_NONE,
    MOTION_STATIONARY,
    MOTION_MOVING,
    MOTION_ROTATING,
    MOTION_FREEFALL
};

// Sensor data structure
struct motion_sensor_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    uint32_t timestamp;
};

// Test fixture
struct motion_sensor_fixture {
    enum motion_sensor_state state;
    enum motion_type motion;
    struct motion_sensor_data buffer[MOTION_SENSOR_BUFFER_SIZE];
    size_t buffer_index;
    uint32_t sample_count;
    bool calibrated;
    int16_t calibration_offset[9];
    bool low_power_enabled;
    uint32_t last_activity_time;
    int message_count;
};

static void *motion_sensor_setup(void)
{
    static struct motion_sensor_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    fixture.state = MOTION_SENSOR_STATE_IDLE;
    fixture.motion = MOTION_NONE;
    return &fixture;
}

static void motion_sensor_teardown(void *f)
{
    struct motion_sensor_fixture *fixture = (struct motion_sensor_fixture *)f;
    fixture->buffer_index = 0;
    fixture->sample_count = 0;
    fixture->message_count = 0;
}

ZTEST_SUITE(motion_sensor, NULL, motion_sensor_setup, NULL, NULL, motion_sensor_teardown);

// Mock sensor functions
static int motion_sensor_init_mock(struct motion_sensor_fixture *fixture)
{
    fixture->state = MOTION_SENSOR_STATE_IDLE;
    fixture->calibrated = false;
    fixture->low_power_enabled = false;
    memset(fixture->calibration_offset, 0, sizeof(fixture->calibration_offset));
    return 0;
}

static int motion_sensor_start_mock(struct motion_sensor_fixture *fixture)
{
    if (fixture->state == MOTION_SENSOR_STATE_ERROR) {
        return -EIO;
    }
    
    fixture->state = MOTION_SENSOR_STATE_ACTIVE;
    fixture->buffer_index = 0;
    fixture->sample_count = 0;
    fixture->last_activity_time = k_uptime_get_32();
    return 0;
}

static int motion_sensor_stop_mock(struct motion_sensor_fixture *fixture)
{
    fixture->state = MOTION_SENSOR_STATE_IDLE;
    return 0;
}

static int motion_sensor_calibrate_mock(struct motion_sensor_fixture *fixture)
{
    fixture->state = MOTION_SENSOR_STATE_CALIBRATING;
    
    // Simulate calibration
    k_sleep(K_MSEC(200));
    
    // Set some calibration offsets
    // Accelerometer offsets
    fixture->calibration_offset[0] = 5;    // accel_x
    fixture->calibration_offset[1] = -10;  // accel_y
    fixture->calibration_offset[2] = 20;   // accel_z
    // Gyroscope offsets
    fixture->calibration_offset[3] = 3;    // gyro_x
    fixture->calibration_offset[4] = -2;   // gyro_y
    fixture->calibration_offset[5] = 1;    // gyro_z
    // Magnetometer offsets
    fixture->calibration_offset[6] = 15;   // mag_x
    fixture->calibration_offset[7] = -20;  // mag_y
    fixture->calibration_offset[8] = 10;   // mag_z
    
    fixture->calibrated = true;
    fixture->state = MOTION_SENSOR_STATE_IDLE;
    return 0;
}

static void motion_sensor_process_sample_mock(struct motion_sensor_fixture *fixture,
                                             struct motion_sensor_data *data)
{
    // Store original data first
    if (fixture->buffer_index < MOTION_SENSOR_BUFFER_SIZE) {
        memcpy(&fixture->buffer[fixture->buffer_index], data, sizeof(*data));
        
        // Apply calibration to stored data if available
        if (fixture->calibrated) {
            fixture->buffer[fixture->buffer_index].accel_x -= fixture->calibration_offset[0];
            fixture->buffer[fixture->buffer_index].accel_y -= fixture->calibration_offset[1];
            fixture->buffer[fixture->buffer_index].accel_z -= fixture->calibration_offset[2];
            fixture->buffer[fixture->buffer_index].gyro_x -= fixture->calibration_offset[3];
            fixture->buffer[fixture->buffer_index].gyro_y -= fixture->calibration_offset[4];
            fixture->buffer[fixture->buffer_index].gyro_z -= fixture->calibration_offset[5];
            fixture->buffer[fixture->buffer_index].mag_x -= fixture->calibration_offset[6];
            fixture->buffer[fixture->buffer_index].mag_y -= fixture->calibration_offset[7];
            fixture->buffer[fixture->buffer_index].mag_z -= fixture->calibration_offset[8];
        }
        
        fixture->buffer_index++;
        fixture->sample_count++;
    }
    
    // Detect motion type (simplified) - use calibrated values if available
    int16_t ax = data->accel_x;
    int16_t ay = data->accel_y;
    int16_t az = data->accel_z;
    int16_t gx = data->gyro_x;
    int16_t gy = data->gyro_y;
    int16_t gz = data->gyro_z;
    
    if (fixture->calibrated) {
        ax -= fixture->calibration_offset[0];
        ay -= fixture->calibration_offset[1];
        az -= fixture->calibration_offset[2];
        gx -= fixture->calibration_offset[3];
        gy -= fixture->calibration_offset[4];
        gz -= fixture->calibration_offset[5];
    }
    
    int32_t accel_mag = (ax * ax) + (ay * ay) + (az * az);
    int32_t gyro_mag = (gx * gx) + (gy * gy) + (gz * gz);
    
    if (accel_mag < 100000 && gyro_mag < 1000) {
        fixture->motion = MOTION_STATIONARY;
    } else if (gyro_mag > 10000) {
        fixture->motion = MOTION_ROTATING;
    } else if (accel_mag > 500000) {
        fixture->motion = MOTION_MOVING;
    } else {
        fixture->motion = MOTION_NONE;
    }
    
    // Update activity time
    fixture->last_activity_time = k_uptime_get_32();
    
    // Simulate message queue notification
    fixture->message_count++;
}

static int motion_sensor_set_low_power_mock(struct motion_sensor_fixture *fixture, bool enable)
{
    if (enable && fixture->state == MOTION_SENSOR_STATE_ACTIVE) {
        fixture->state = MOTION_SENSOR_STATE_LOW_POWER;
        fixture->low_power_enabled = true;
    } else if (!enable && fixture->state == MOTION_SENSOR_STATE_LOW_POWER) {
        fixture->state = MOTION_SENSOR_STATE_ACTIVE;
        fixture->low_power_enabled = false;
    }
    return 0;
}

// Tests
ZTEST_F(motion_sensor, test_init)
{
    // Test initialization
    int ret = motion_sensor_init_mock(fixture);
    
    // Verify
    zassert_equal(ret, 0, "Init should succeed");
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_IDLE, "Should be idle");
    zassert_false(fixture->calibrated, "Should not be calibrated");
    zassert_false(fixture->low_power_enabled, "Low power should be disabled");
}

ZTEST_F(motion_sensor, test_start_stop)
{
    // Initialize first
    motion_sensor_init_mock(fixture);
    
    // Start sensor
    int ret = motion_sensor_start_mock(fixture);
    zassert_equal(ret, 0, "Start should succeed");
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_ACTIVE, "Should be active");
    
    // Stop sensor
    ret = motion_sensor_stop_mock(fixture);
    zassert_equal(ret, 0, "Stop should succeed");
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_IDLE, "Should be idle");
}

ZTEST_F(motion_sensor, test_start_when_error)
{
    // Set error state
    fixture->state = MOTION_SENSOR_STATE_ERROR;
    
    // Try to start
    int ret = motion_sensor_start_mock(fixture);
    
    // Verify
    zassert_equal(ret, -EIO, "Should fail when in error state");
}

ZTEST_F(motion_sensor, test_calibration)
{
    // Initialize
    motion_sensor_init_mock(fixture);
    
    // Calibrate
    int ret = motion_sensor_calibrate_mock(fixture);
    
    // Verify
    zassert_equal(ret, 0, "Calibration should succeed");
    zassert_true(fixture->calibrated, "Should be calibrated");
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_IDLE, "Should return to idle");
    
    // Check calibration values were set
    bool has_offsets = false;
    for (int i = 0; i < 9; i++) {
        if (fixture->calibration_offset[i] != 0) {
            has_offsets = true;
            break;
        }
    }
    zassert_true(has_offsets, "Should have calibration offsets");
}

ZTEST_F(motion_sensor, test_sample_processing)
{
    // Initialize and start
    motion_sensor_init_mock(fixture);
    motion_sensor_start_mock(fixture);
    
    // Reset message count to ensure clean state
    fixture->message_count = 0;
    
    // Create test sample
    struct motion_sensor_data sample = {
        .accel_x = 100,
        .accel_y = 200,
        .accel_z = 1000,
        .gyro_x = 10,
        .gyro_y = 20,
        .gyro_z = 30,
        .mag_x = 50,
        .mag_y = 60,
        .mag_z = 70,
        .timestamp = k_uptime_get_32()
    };
    
    // Process sample
    motion_sensor_process_sample_mock(fixture, &sample);
    
    // Verify
    zassert_equal(fixture->sample_count, 1, "Should have 1 sample");
    zassert_equal(fixture->buffer_index, 1, "Buffer index should be 1");
    zassert_equal(fixture->message_count, 1, "Should have sent 1 message (actual: %d)",
                  fixture->message_count);
}

ZTEST_F(motion_sensor, test_calibrated_sample_processing)
{
    // Initialize and calibrate
    motion_sensor_init_mock(fixture);
    motion_sensor_calibrate_mock(fixture);
    motion_sensor_start_mock(fixture);
    
    // Create test sample
    struct motion_sensor_data sample = {
        .accel_x = 100,
        .accel_y = 200,
        .accel_z = 1000,
        .gyro_x = 10,
        .gyro_y = 20,
        .gyro_z = 30,
        .mag_x = 50,
        .mag_y = 60,
        .mag_z = 70,
        .timestamp = k_uptime_get_32()
    };
    
    // Process sample
    motion_sensor_process_sample_mock(fixture, &sample);
    
    // Verify calibration was applied
    zassert_equal(fixture->buffer[0].accel_x, 
                  sample.accel_x - fixture->calibration_offset[0],
                  "X acceleration should be calibrated");
    zassert_equal(fixture->buffer[0].gyro_y,
                  sample.gyro_y - fixture->calibration_offset[4],
                  "Y gyroscope should be calibrated");
    zassert_equal(fixture->buffer[0].mag_z,
                  sample.mag_z - fixture->calibration_offset[8],
                  "Z magnetometer should be calibrated");
}

ZTEST_F(motion_sensor, test_motion_detection)
{
    // Initialize and start
    motion_sensor_init_mock(fixture);
    motion_sensor_start_mock(fixture);
    
    // Test different motion scenarios
    struct {
        int16_t accel_x, accel_y, accel_z;
        int16_t gyro_x, gyro_y, gyro_z;
        enum motion_type expected_motion;
    } test_cases[] = {
        {50, 50, 50, 10, 10, 10, MOTION_STATIONARY},      // Low values
        {500, 500, 500, 50, 50, 50, MOTION_MOVING},       // Medium acceleration
        {100, 100, 100, 200, 200, 200, MOTION_ROTATING},  // High rotation
    };
    
    for (size_t i = 0; i < ARRAY_SIZE(test_cases); i++) {
        struct motion_sensor_data sample = {
            .accel_x = test_cases[i].accel_x,
            .accel_y = test_cases[i].accel_y,
            .accel_z = test_cases[i].accel_z,
            .gyro_x = test_cases[i].gyro_x,
            .gyro_y = test_cases[i].gyro_y,
            .gyro_z = test_cases[i].gyro_z,
            .mag_x = 0,
            .mag_y = 0,
            .mag_z = 0,
            .timestamp = k_uptime_get_32()
        };
        
        motion_sensor_process_sample_mock(fixture, &sample);
        
        zassert_equal(fixture->motion, test_cases[i].expected_motion,
                      "Motion should be %d for test case %d",
                      test_cases[i].expected_motion, i);
    }
}

ZTEST_F(motion_sensor, test_low_power_mode)
{
    // Initialize and start
    motion_sensor_init_mock(fixture);
    motion_sensor_start_mock(fixture);
    
    // Enable low power mode
    int ret = motion_sensor_set_low_power_mock(fixture, true);
    zassert_equal(ret, 0, "Low power enable should succeed");
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_LOW_POWER,
                  "Should be in low power state");
    zassert_true(fixture->low_power_enabled, "Low power should be enabled");
    
    // Disable low power mode
    ret = motion_sensor_set_low_power_mock(fixture, false);
    zassert_equal(ret, 0, "Low power disable should succeed");
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_ACTIVE,
                  "Should be in active state");
    zassert_false(fixture->low_power_enabled, "Low power should be disabled");
}

ZTEST_F(motion_sensor, test_buffer_management)
{
    // Initialize and start
    motion_sensor_init_mock(fixture);
    motion_sensor_start_mock(fixture);
    
    // Fill buffer with samples
    for (int i = 0; i < 50; i++) {
        struct motion_sensor_data sample = {
            .accel_x = (int16_t)i,
            .accel_y = (int16_t)(i * 2),
            .accel_z = (int16_t)(i * 3),
            .gyro_x = (int16_t)(i / 2),
            .gyro_y = (int16_t)(i / 3),
            .gyro_z = (int16_t)(i / 4),
            .mag_x = (int16_t)(i + 10),
            .mag_y = (int16_t)(i + 20),
            .mag_z = (int16_t)(i + 30),
            .timestamp = k_uptime_get_32()
        };
        
        motion_sensor_process_sample_mock(fixture, &sample);
    }
    
    // Verify
    zassert_equal(fixture->sample_count, 50, "Should have 50 samples");
    zassert_equal(fixture->buffer_index, 50, "Buffer index should be 50");
    
    // Verify sample data
    for (int i = 0; i < 50; i++) {
        zassert_equal(fixture->buffer[i].accel_x, i,
                      "Sample %d accel_x should be %d", i, i);
        zassert_equal(fixture->buffer[i].mag_z, i + 30,
                      "Sample %d mag_z should be %d", i, i + 30);
    }
}

ZTEST_F(motion_sensor, test_buffer_overflow_protection)
{
    // Initialize and start
    motion_sensor_init_mock(fixture);
    motion_sensor_start_mock(fixture);
    
    // Try to overflow buffer
    for (int i = 0; i < MOTION_SENSOR_BUFFER_SIZE + 100; i++) {
        struct motion_sensor_data sample = {
            .accel_x = (int16_t)i,
            .accel_y = 0,
            .accel_z = 0,
            .gyro_x = 0,
            .gyro_y = 0,
            .gyro_z = 0,
            .mag_x = 0,
            .mag_y = 0,
            .mag_z = 0,
            .timestamp = k_uptime_get_32()
        };
        
        motion_sensor_process_sample_mock(fixture, &sample);
    }
    
    // Verify buffer didn't overflow
    zassert_equal(fixture->buffer_index, MOTION_SENSOR_BUFFER_SIZE,
                  "Buffer index should stop at max size");
}

ZTEST_F(motion_sensor, test_sample_rate)
{
    // Test that samples are processed at expected rate
    const uint32_t expected_period_ms = 1000 / MOTION_SENSOR_SAMPLE_RATE_HZ;
    
    zassert_equal(expected_period_ms, 20,
                  "Sample period should be 20ms for 50Hz");
}

ZTEST_F(motion_sensor, test_state_transitions)
{
    // Test all state transitions
    
    // IDLE -> ACTIVE
    fixture->state = MOTION_SENSOR_STATE_IDLE;
    motion_sensor_start_mock(fixture);
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_ACTIVE,
                  "Should transition from IDLE to ACTIVE");
    
    // ACTIVE -> LOW_POWER
    motion_sensor_set_low_power_mock(fixture, true);
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_LOW_POWER,
                  "Should transition from ACTIVE to LOW_POWER");
    
    // LOW_POWER -> ACTIVE
    motion_sensor_set_low_power_mock(fixture, false);
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_ACTIVE,
                  "Should transition from LOW_POWER to ACTIVE");
    
    // ACTIVE -> IDLE
    motion_sensor_stop_mock(fixture);
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_IDLE,
                  "Should transition from ACTIVE to IDLE");
    
    // IDLE -> CALIBRATING -> IDLE
    motion_sensor_calibrate_mock(fixture);
    zassert_equal(fixture->state, MOTION_SENSOR_STATE_IDLE,
                  "Should return to IDLE after calibration");
}

ZTEST_F(motion_sensor, test_activity_timeout)
{
    // Initialize and start
    motion_sensor_init_mock(fixture);
    motion_sensor_start_mock(fixture);
    
    // Record initial activity time
    uint32_t initial_time = fixture->last_activity_time;
    
    // Wait a bit to ensure time passes
    k_sleep(K_MSEC(10));
    
    // Process a sample
    struct motion_sensor_data sample = {
        .accel_x = 100,
        .accel_y = 200,
        .accel_z = 300,
        .gyro_x = 10,
        .gyro_y = 20,
        .gyro_z = 30,
        .mag_x = 0,
        .mag_y = 0,
        .mag_z = 0,
        .timestamp = k_uptime_get_32()
    };
    
    motion_sensor_process_sample_mock(fixture, &sample);
    
    // Verify activity time was updated
    zassert_true(fixture->last_activity_time > initial_time,
                 "Activity time should be updated (initial: %u, current: %u)",
                 initial_time, fixture->last_activity_time);
    
    // Check timeout threshold
    uint32_t time_since_activity = k_uptime_get_32() - fixture->last_activity_time;
    zassert_true(time_since_activity < MOTION_SENSOR_TIMEOUT_MS,
                 "Should not timeout immediately");
}
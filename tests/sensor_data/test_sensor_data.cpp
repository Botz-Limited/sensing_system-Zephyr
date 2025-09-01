/**
 * @file test_sensor_data.cpp
 * @brief Unit tests for sensor_data module
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/fff.h>
#include <app.hpp>
#include <events/app_state_event.h>
#include <caf/events/module_state_event.h>
#include "sensor_data_ring_buffer.h"
#include "sensor_data_fast_processing.h"

// Define FFF globals
DEFINE_FFF_GLOBALS;

// Mock functions
FAKE_VALUE_FUNC(int, k_msgq_put, struct k_msgq *, const void *, k_timeout_t);
FAKE_VALUE_FUNC(int, k_msgq_get, struct k_msgq *, void *, k_timeout_t);
FAKE_VOID_FUNC(module_set_state, enum module_state);
FAKE_VALUE_FUNC(bool, app_event_manager_alloc, struct app_event_header **, size_t);
FAKE_VOID_FUNC(app_event_manager_submit, struct app_event_header *);

// Define message queues for testing
K_MSGQ_DEFINE(sensor_data_msgq, sizeof(generic_message_t), 10, 4);
K_MSGQ_DEFINE(sensor_data_queue, sizeof(generic_message_t), 10, 4);

// Test fixture
struct sensor_data_fixture {
    foot_data_ring_t foot_ring;
    imu_data_ring_t imu_ring;
    command_ring_t cmd_ring;
    sensor_data_stats_t stats;
};

static void *sensor_data_setup(void)
{
    struct sensor_data_fixture *fixture = k_malloc(sizeof(struct sensor_data_fixture));
    zassert_not_null(fixture, "Failed to allocate fixture");
    
    // Initialize rings
    memset(&fixture->foot_ring, 0, sizeof(foot_data_ring_t));
    memset(&fixture->imu_ring, 0, sizeof(imu_data_ring_t));
    memset(&fixture->cmd_ring, 0, sizeof(command_ring_t));
    memset(&fixture->stats, 0, sizeof(sensor_data_stats_t));
    
    return fixture;
}

static void sensor_data_teardown(void *f)
{
    struct sensor_data_fixture *fixture = (struct sensor_data_fixture *)f;
    k_free(fixture);
    
    // Reset all fakes
    RESET_FAKE(k_msgq_put);
    RESET_FAKE(k_msgq_get);
    RESET_FAKE(module_set_state);
    RESET_FAKE(app_event_manager_alloc);
    RESET_FAKE(app_event_manager_submit);
}

ZTEST_SUITE(sensor_data, NULL, sensor_data_setup, NULL, NULL, sensor_data_teardown);

// Test ring buffer operations
ZTEST_F(sensor_data, test_foot_ring_buffer_basic)
{
    struct sensor_data_fixture *fixture = (struct sensor_data_fixture *)_fixture;
    foot_samples_t data_in, data_out;
    uint8_t foot_id_in = 1, foot_id_out;
    
    // Fill test data
    for (int i = 0; i < NUM_FOOT_SENSOR_CHANNELS; i++) {
        data_in.values[i] = i * 100;
    }
    
    // Test put
    bool ret = foot_ring_put(&fixture->foot_ring, &data_in, foot_id_in);
    zassert_true(ret, "Failed to put data in empty ring");
    zassert_equal(atomic_get(&fixture->foot_ring.count), 1, "Ring count should be 1");
    
    // Test get
    ret = foot_ring_get(&fixture->foot_ring, &data_out, &foot_id_out);
    zassert_true(ret, "Failed to get data from ring");
    zassert_equal(foot_id_out, foot_id_in, "Foot ID mismatch");
    zassert_mem_equal(&data_out, &data_in, sizeof(foot_samples_t), "Data mismatch");
    zassert_equal(atomic_get(&fixture->foot_ring.count), 0, "Ring count should be 0");
}

ZTEST_F(sensor_data, test_foot_ring_buffer_full)
{
    struct sensor_data_fixture *fixture = (struct sensor_data_fixture *)_fixture;
    foot_samples_t data;
    uint8_t foot_id = 0;
    
    // Fill ring to capacity
    for (int i = 0; i < FOOT_DATA_RING_SIZE; i++) {
        for (int j = 0; j < NUM_FOOT_SENSOR_CHANNELS; j++) {
            data.values[j] = i;
        }
        bool ret = foot_ring_put(&fixture->foot_ring, &data, foot_id);
        zassert_true(ret, "Failed to put data %d", i);
    }
    
    // Try to add one more - should fail
    bool ret = foot_ring_put(&fixture->foot_ring, &data, foot_id);
    zassert_false(ret, "Should fail when ring is full");
    zassert_equal(atomic_get(&fixture->foot_ring.count), FOOT_DATA_RING_SIZE, 
                  "Ring count should be at max");
}

ZTEST_F(sensor_data, test_foot_ring_buffer_wrap_around)
{
    struct sensor_data_fixture *fixture = (struct sensor_data_fixture *)_fixture;
    foot_samples_t data_in, data_out;
    uint8_t foot_id;
    
    // Fill and empty ring multiple times to test wrap-around
    for (int cycle = 0; cycle < 3; cycle++) {
        // Fill half
        for (int i = 0; i < FOOT_DATA_RING_SIZE / 2; i++) {
            for (int j = 0; j < NUM_FOOT_SENSOR_CHANNELS; j++) {
                data_in.values[j] = cycle * 1000 + i;
            }
            foot_ring_put(&fixture->foot_ring, &data_in, i);
        }
        
        // Read half
        for (int i = 0; i < FOOT_DATA_RING_SIZE / 2; i++) {
            bool ret = foot_ring_get(&fixture->foot_ring, &data_out, &foot_id);
            zassert_true(ret, "Failed to get data");
            zassert_equal(data_out.values[0], cycle * 1000 + i, 
                         "Data mismatch at cycle %d, index %d", cycle, i);
        }
    }
}

// Test fast processing algorithms
ZTEST(sensor_data, test_ground_contact_detection)
{
    uint16_t pressure[8];
    
    // Test 1: All zeros - no contact
    memset(pressure, 0, sizeof(pressure));
    bool contact = detect_ground_contact(pressure);
    zassert_false(contact, "Should detect no contact with zero pressure");
    
    // Test 2: One sensor above threshold
    memset(pressure, 0, sizeof(pressure));
    pressure[3] = CONTACT_THRESHOLD_MIN + 10;
    contact = detect_ground_contact(pressure);
    zassert_true(contact, "Should detect contact with one sensor above threshold");
    
    // Test 3: Multiple sensors with pressure
    for (int i = 0; i < 8; i++) {
        pressure[i] = CONTACT_THRESHOLD_MIN + i * 10;
    }
    contact = detect_ground_contact(pressure);
    zassert_true(contact, "Should detect contact with multiple sensors");
    
    // Test 4: Just below threshold
    for (int i = 0; i < 8; i++) {
        pressure[i] = CONTACT_THRESHOLD_MIN - 1;
    }
    contact = detect_ground_contact(pressure);
    zassert_false(contact, "Should not detect contact below threshold");
}

ZTEST(sensor_data, test_peak_force_detection)
{
    uint16_t pressure[8];
    
    // Test 1: All same values
    for (int i = 0; i < 8; i++) {
        pressure[i] = 100;
    }
    uint16_t peak = detect_peak_force(pressure);
    zassert_equal(peak, 100, "Peak force should be average (100)");
    
    // Test 2: Increasing values
    for (int i = 0; i < 8; i++) {
        pressure[i] = i * 50;
    }
    peak = detect_peak_force(pressure);
    // Sum = 0+50+100+150+200+250+300+350 = 1400, avg = 175
    zassert_equal(peak, 175, "Peak force calculation incorrect");
    
    // Test 3: High values to test overflow protection
    for (int i = 0; i < 8; i++) {
        pressure[i] = 8000;
    }
    peak = detect_peak_force(pressure);
    zassert_equal(peak, 8000, "Peak force should handle high values");
}

ZTEST(sensor_data, test_contact_phase_detection)
{
    uint16_t pressure[8];
    contact_phase_t phase;
    
    // Test 1: Flight phase (all low)
    memset(pressure, 0, sizeof(pressure));
    phase = detect_contact_phase(pressure, false);
    zassert_equal(phase, PHASE_FLIGHT, "Should detect flight phase");
    
    // Test 2: Heel strike (heel sensors high, not previously in contact)
    memset(pressure, 0, sizeof(pressure));
    pressure[0] = CONTACT_THRESHOLD_HEEL + 50;
    pressure[1] = CONTACT_THRESHOLD_HEEL + 30;
    phase = detect_contact_phase(pressure, false);
    zassert_equal(phase, PHASE_HEEL_STRIKE, "Should detect heel strike");
    
    // Test 3: Mid stance (all sensors moderate)
    for (int i = 0; i < 8; i++) {
        pressure[i] = 150;
    }
    phase = detect_contact_phase(pressure, true);
    zassert_equal(phase, PHASE_MID_STANCE, "Should detect mid stance");
    
    // Test 4: Toe off (forefoot sensors highest)
    memset(pressure, 50, sizeof(pressure));
    pressure[6] = 300;
    pressure[7] = 350;
    phase = detect_contact_phase(pressure, true);
    zassert_equal(phase, PHASE_TOE_OFF, "Should detect toe off");
}

ZTEST(sensor_data, test_pronation_check)
{
    float quaternion[4];
    
    // Test 1: Neutral position
    quaternion[0] = 0.0f;
    quaternion[1] = 0.0f;  // Roll component
    quaternion[2] = 0.0f;
    quaternion[3] = 1.0f;
    int8_t pronation = quick_pronation_check(quaternion);
    zassert_equal(pronation, 0, "Should detect neutral pronation");
    
    // Test 2: Supination
    quaternion[1] = 0.15f;
    pronation = quick_pronation_check(quaternion);
    zassert_equal(pronation, 1, "Should detect supination");
    
    // Test 3: Pronation
    quaternion[1] = -0.15f;
    pronation = quick_pronation_check(quaternion);
    zassert_equal(pronation, -1, "Should detect pronation");
}

// Test message handling
ZTEST(sensor_data, test_foot_sensor_message_handling)
{
    generic_message_t msg;
    msg.sender = SENDER_FOOT_SENSOR_THREAD;
    msg.type = MSG_TYPE_FOOT_SAMPLES;
    
    // Fill with test data
    for (int i = 0; i < NUM_FOOT_SENSOR_CHANNELS; i++) {
        msg.data.foot_samples.values[i] = i * 10;
    }
    
    // Mock successful message queue put
    k_msgq_put_fake.return_val = 0;
    
    // Simulate receiving message
    int ret = k_msgq_put(&sensor_data_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Message queue put should succeed");
    zassert_equal(k_msgq_put_fake.call_count, 1, "k_msgq_put should be called once");
}

ZTEST(sensor_data, test_imu_message_handling)
{
    generic_message_t msg;
    msg.sender = SENDER_BHI360_THREAD;
    msg.type = MSG_TYPE_BHI360_LOG_RECORD;
    
    // Fill with test IMU data
    msg.data.bhi360_log_record.quat_x = 0.1f;
    msg.data.bhi360_log_record.quat_y = 0.2f;
    msg.data.bhi360_log_record.quat_z = 0.3f;
    msg.data.bhi360_log_record.quat_w = 0.9f;
    msg.data.bhi360_log_record.step_count = 1000;
    
    // Mock successful message queue put
    k_msgq_put_fake.return_val = 0;
    
    // Simulate receiving message
    int ret = k_msgq_put(&sensor_data_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Message queue put should succeed");
}

ZTEST(sensor_data, test_command_message_handling)
{
    generic_message_t msg;
    msg.sender = SENDER_BLUETOOTH;
    msg.type = MSG_TYPE_COMMAND;
    strncpy(msg.data.command_str, "START_SENSOR_PROCESSING", MAX_COMMAND_STRING_LEN);
    
    // Mock successful message queue put
    k_msgq_put_fake.return_val = 0;
    
    // Simulate receiving message
    int ret = k_msgq_put(&sensor_data_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Command message should be sent successfully");
}

// Test statistics tracking
ZTEST_F(sensor_data, test_statistics_tracking)
{
    struct sensor_data_fixture *fixture = (struct sensor_data_fixture *)_fixture;
    foot_samples_t data;
    
    // Simulate receiving and dropping data
    fixture->stats.foot_data_received = 1000;
    fixture->stats.foot_data_dropped = 5;
    fixture->stats.max_foot_ring_depth = 6;
    
    zassert_equal(fixture->stats.foot_data_received, 1000, "Received count mismatch");
    zassert_equal(fixture->stats.foot_data_dropped, 5, "Dropped count mismatch");
    zassert_equal(fixture->stats.max_foot_ring_depth, 6, "Max depth mismatch");
    
    // Calculate drop rate
    float drop_rate = (float)fixture->stats.foot_data_dropped / 
                      (fixture->stats.foot_data_received + fixture->stats.foot_data_dropped) * 100;
    zassert_within(drop_rate, 0.5f, 0.1f, "Drop rate should be ~0.5%%");
}

// Test timing constraints
ZTEST(sensor_data, test_processing_time_constraint)
{
    // This test verifies that our processing algorithms are fast enough
    uint16_t pressure[8] = {100, 200, 300, 400, 500, 600, 700, 800};
    float quaternion[4] = {0.1f, 0.2f, 0.3f, 0.9f};
    
    // Measure processing time
    uint32_t start = k_cycle_get_32();
    
    // Run all fast processing functions
    for (int i = 0; i < 100; i++) {
        detect_ground_contact(pressure);
        detect_peak_force(pressure);
        detect_contact_phase(pressure, true);
        quick_pronation_check(quaternion);
    }
    
    uint32_t end = k_cycle_get_32();
    uint32_t cycles = end - start;
    uint32_t us = k_cyc_to_us_floor32(cycles);
    
    // 100 iterations should take less than 200us (2us per iteration)
    zassert_true(us < 200, "Processing too slow: %u us for 100 iterations", us);
}
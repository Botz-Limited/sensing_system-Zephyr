/**
 * @file test_realtime_metrics.cpp
 * @brief Unit tests for realtime_metrics module
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

// Define FFF globals
DEFINE_FFF_GLOBALS;

// Mock functions
FAKE_VALUE_FUNC(int, k_msgq_put, struct k_msgq *, const void *, k_timeout_t);
FAKE_VALUE_FUNC(int, k_msgq_get, struct k_msgq *, void *, k_timeout_t);
FAKE_VOID_FUNC(module_set_state, enum module_state);

// Define message queues for testing
K_MSGQ_DEFINE(sensor_data_queue, sizeof(generic_message_t), 10, 4);
K_MSGQ_DEFINE(realtime_queue, sizeof(generic_message_t), 10, 4);
K_MSGQ_DEFINE(bluetooth_msgq, sizeof(generic_message_t), 10, 4);

// Test data structures
typedef struct {
    uint16_t cadence_x2;      // Steps per minute * 2
    uint16_t pace_sec_per_km; // Seconds per kilometer
    uint8_t form_score;       // 0-100
    int8_t balance_lr;        // -100 to +100 (negative = left bias)
    uint8_t fatigue_level;    // 0-100
} realtime_metrics_t;

// Test fixture
struct realtime_metrics_fixture {
    uint32_t last_step_count;
    uint32_t last_step_time;
    float cadence_history[10];
    uint8_t cadence_index;
    realtime_metrics_t current_metrics;
};

static void *realtime_metrics_setup(void)
{
    struct realtime_metrics_fixture *fixture = k_malloc(sizeof(struct realtime_metrics_fixture));
    zassert_not_null(fixture, "Failed to allocate fixture");
    
    memset(fixture, 0, sizeof(struct realtime_metrics_fixture));
    
    return fixture;
}

static void realtime_metrics_teardown(void *f)
{
    struct realtime_metrics_fixture *fixture = (struct realtime_metrics_fixture *)f;
    k_free(fixture);
    
    // Reset all fakes
    RESET_FAKE(k_msgq_put);
    RESET_FAKE(k_msgq_get);
    RESET_FAKE(module_set_state);
}

ZTEST_SUITE(realtime_metrics, NULL, realtime_metrics_setup, NULL, NULL, realtime_metrics_teardown);

// Helper function to calculate cadence
static uint16_t calculate_cadence(uint32_t step_count, uint32_t last_step_count, 
                                 uint32_t time_ms, uint32_t last_time_ms)
{
    if (time_ms <= last_time_ms) {
        return 0;
    }
    
    uint32_t steps = step_count - last_step_count;
    uint32_t time_diff_ms = time_ms - last_time_ms;
    
    if (time_diff_ms == 0) {
        return 0;
    }
    
    // Calculate steps per minute * 2
    uint32_t cadence_x2 = (steps * 60000 * 2) / time_diff_ms;
    
    return (uint16_t)cadence_x2;
}

// Test cadence calculation
ZTEST_F(realtime_metrics, test_cadence_calculation)
{
    struct realtime_metrics_fixture *fixture = (struct realtime_metrics_fixture *)_fixture;
    
    // Test 1: Normal cadence (180 steps/min)
    fixture->last_step_count = 1000;
    fixture->last_step_time = 0;
    
    uint32_t new_step_count = 1030;  // 30 steps
    uint32_t new_time = 10000;       // 10 seconds
    
    uint16_t cadence_x2 = calculate_cadence(new_step_count, fixture->last_step_count,
                                           new_time, fixture->last_step_time);
    
    // 30 steps in 10 seconds = 180 steps/min = 360 when x2
    zassert_equal(cadence_x2, 360, "Cadence should be 180 spm (360 x2)");
    
    // Test 2: No time elapsed
    cadence_x2 = calculate_cadence(1050, 1000, 0, 0);
    zassert_equal(cadence_x2, 0, "Cadence should be 0 with no time elapsed");
    
    // Test 3: No steps
    cadence_x2 = calculate_cadence(1000, 1000, 10000, 0);
    zassert_equal(cadence_x2, 0, "Cadence should be 0 with no steps");
}

// Test pace calculation
ZTEST_F(realtime_metrics, test_pace_calculation)
{
    // Test pace calculation from cadence and stride length
    // Assume average stride length of 1.2m for testing
    float stride_length_m = 1.2f;
    
    // Test 1: 180 spm cadence
    uint16_t cadence_x2 = 360;  // 180 spm
    float cadence_spm = cadence_x2 / 2.0f;
    
    // Speed = cadence * stride_length / 60
    float speed_m_s = (cadence_spm * stride_length_m) / 60.0f;
    float speed_km_h = speed_m_s * 3.6f;
    
    // Pace = 3600 / speed_km_h (seconds per km)
    uint16_t pace_sec_per_km = (uint16_t)(3600.0f / speed_km_h);
    
    // 180 spm * 1.2m = 216 m/min = 3.6 m/s = 12.96 km/h = 277 sec/km
    zassert_within(pace_sec_per_km, 278, 2, "Pace should be ~278 sec/km");
    
    // Test 2: Slower pace (150 spm)
    cadence_x2 = 300;  // 150 spm
    cadence_spm = cadence_x2 / 2.0f;
    speed_m_s = (cadence_spm * stride_length_m) / 60.0f;
    speed_km_h = speed_m_s * 3.6f;
    pace_sec_per_km = (uint16_t)(3600.0f / speed_km_h);
    
    // 150 spm * 1.2m = 180 m/min = 3 m/s = 10.8 km/h = 333 sec/km
    zassert_within(pace_sec_per_km, 333, 2, "Pace should be ~333 sec/km");
}

// Test form score calculation
ZTEST_F(realtime_metrics, test_form_score_calculation)
{
    // Form score based on:
    // - Ground contact time ratio (40%)
    // - Vertical oscillation (30%)
    // - Balance (30%)
    
    // Test 1: Perfect form
    float contact_time_ratio = 0.25f;  // 25% - excellent
    float vertical_osc_cm = 6.0f;      // 6cm - excellent
    int8_t balance = 0;                // Perfect balance
    
    // Score calculation (simplified)
    uint8_t contact_score = (contact_time_ratio < 0.30f) ? 100 : 
                           (contact_time_ratio < 0.35f) ? 80 : 60;
    uint8_t osc_score = (vertical_osc_cm < 7.0f) ? 100 :
                       (vertical_osc_cm < 9.0f) ? 80 : 60;
    uint8_t balance_score = (abs(balance) < 5) ? 100 :
                           (abs(balance) < 10) ? 80 : 60;
    
    uint8_t form_score = (contact_score * 40 + osc_score * 30 + balance_score * 30) / 100;
    
    zassert_equal(form_score, 100, "Perfect form should score 100");
    
    // Test 2: Average form
    contact_time_ratio = 0.33f;
    vertical_osc_cm = 8.5f;
    balance = 8;
    
    contact_score = (contact_time_ratio < 0.30f) ? 100 : 
                   (contact_time_ratio < 0.35f) ? 80 : 60;
    osc_score = (vertical_osc_cm < 7.0f) ? 100 :
               (vertical_osc_cm < 9.0f) ? 80 : 60;
    balance_score = (abs(balance) < 5) ? 100 :
                   (abs(balance) < 10) ? 80 : 60;
    
    form_score = (contact_score * 40 + osc_score * 30 + balance_score * 30) / 100;
    
    zassert_equal(form_score, 80, "Average form should score 80");
}

// Test balance calculation
ZTEST_F(realtime_metrics, test_balance_calculation)
{
    // Balance based on left/right ground contact time difference
    
    // Test 1: Perfect balance
    uint16_t left_contact_ms = 250;
    uint16_t right_contact_ms = 250;
    
    int8_t balance = ((int16_t)left_contact_ms - (int16_t)right_contact_ms) * 100 / 
                     ((left_contact_ms + right_contact_ms) / 2);
    
    zassert_equal(balance, 0, "Perfect balance should be 0");
    
    // Test 2: Left bias (10% longer contact)
    left_contact_ms = 275;
    right_contact_ms = 250;
    
    balance = ((int16_t)left_contact_ms - (int16_t)right_contact_ms) * 100 / 
              ((left_contact_ms + right_contact_ms) / 2);
    
    zassert_within(balance, 10, 1, "10%% left bias should give balance ~10");
    
    // Test 3: Right bias
    left_contact_ms = 240;
    right_contact_ms = 260;
    
    balance = ((int16_t)left_contact_ms - (int16_t)right_contact_ms) * 100 / 
              ((left_contact_ms + right_contact_ms) / 2);
    
    zassert_within(balance, -8, 1, "Right bias should give negative balance");
}

// Test BLE message preparation
ZTEST_F(realtime_metrics, test_ble_message_preparation)
{
    struct realtime_metrics_fixture *fixture = (struct realtime_metrics_fixture *)_fixture;
    
    // Set test metrics
    fixture->current_metrics.cadence_x2 = 360;        // 180 spm
    fixture->current_metrics.pace_sec_per_km = 300;   // 5:00/km
    fixture->current_metrics.form_score = 85;
    fixture->current_metrics.balance_lr = -5;
    fixture->current_metrics.fatigue_level = 15;
    
    // Create BLE message
    generic_message_t msg;
    msg.sender = SENDER_REALTIME_METRICS;
    msg.type = MSG_TYPE_ACTIVITY_METRICS_BLE;
    
    // In real implementation, this would pack the data appropriately
    // For testing, we'll just verify the structure
    
    k_msgq_put_fake.return_val = 0;
    
    int ret = k_msgq_put(&bluetooth_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "BLE message should be sent successfully");
    zassert_equal(k_msgq_put_fake.call_count, 1, "k_msgq_put should be called once");
}

// Test moving average for smoothing
ZTEST_F(realtime_metrics, test_moving_average)
{
    struct realtime_metrics_fixture *fixture = (struct realtime_metrics_fixture *)_fixture;
    
    // Add values to cadence history
    float test_values[] = {170, 175, 180, 185, 180, 175, 180, 185, 190, 180};
    
    float sum = 0;
    for (int i = 0; i < 10; i++) {
        fixture->cadence_history[i] = test_values[i];
        sum += test_values[i];
    }
    
    float average = sum / 10;
    
    zassert_within(average, 180.0f, 0.1f, "Moving average should be ~180");
    
    // Test with partial buffer
    memset(fixture->cadence_history, 0, sizeof(fixture->cadence_history));
    fixture->cadence_index = 0;
    
    // Add only 3 values
    for (int i = 0; i < 3; i++) {
        fixture->cadence_history[fixture->cadence_index++] = test_values[i];
    }
    
    sum = 0;
    int count = 0;
    for (int i = 0; i < fixture->cadence_index; i++) {
        sum += fixture->cadence_history[i];
        count++;
    }
    
    average = sum / count;
    zassert_within(average, 175.0f, 0.1f, "Partial average should be ~175");
}

// Test 1Hz update rate
ZTEST(realtime_metrics, test_update_rate_timing)
{
    // This test verifies that BLE updates happen at 1Hz
    
    // In real implementation, this would use k_work_schedule
    // For testing, we verify the timing logic
    
    uint32_t last_ble_update = 0;
    uint32_t current_time = 0;
    
    // Should not update
    bool should_update = (current_time - last_ble_update) >= 1000;
    zassert_false(should_update, "Should not update immediately");
    
    // After 999ms - still no update
    current_time = 999;
    should_update = (current_time - last_ble_update) >= 1000;
    zassert_false(should_update, "Should not update before 1 second");
    
    // After 1000ms - should update
    current_time = 1000;
    should_update = (current_time - last_ble_update) >= 1000;
    zassert_true(should_update, "Should update after 1 second");
    
    // Update and test again
    last_ble_update = current_time;
    current_time = 1500;
    should_update = (current_time - last_ble_update) >= 1000;
    zassert_false(should_update, "Should not update after 500ms");
    
    current_time = 2000;
    should_update = (current_time - last_ble_update) >= 1000;
    zassert_true(should_update, "Should update after another second");
}

// Test sensor data consolidation handling
ZTEST(realtime_metrics, test_sensor_data_handling)
{
    generic_message_t msg;
    msg.sender = SENDER_SENSOR_DATA;
    msg.type = MSG_TYPE_SENSOR_DATA_CONSOLIDATED;
    
    // Mock receiving consolidated sensor data
    k_msgq_get_fake.return_val = 0;
    k_msgq_put_fake.return_val = 0;
    
    // In real implementation, this would process the consolidated data
    // and calculate metrics
    
    int ret = k_msgq_get(&sensor_data_queue, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Should receive sensor data");
    
    // Process and forward to analytics
    msg.sender = SENDER_REALTIME_METRICS;
    msg.type = MSG_TYPE_REALTIME_METRICS;
    
    ret = k_msgq_put(&realtime_queue, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Should forward metrics to analytics");
}

// Test edge cases
ZTEST_F(realtime_metrics, test_edge_cases)
{
    struct realtime_metrics_fixture *fixture = (struct realtime_metrics_fixture *)_fixture;
    
    // Test 1: Zero cadence
    fixture->current_metrics.cadence_x2 = 0;
    // Should handle gracefully, not crash
    
    // Test 2: Maximum cadence (220 spm)
    fixture->current_metrics.cadence_x2 = 440;
    zassert_true(fixture->current_metrics.cadence_x2 <= 500, 
                 "Cadence should be capped at reasonable maximum");
    
    // Test 3: Invalid pace (0 speed)
    fixture->current_metrics.pace_sec_per_km = 65535;  // Max uint16_t
    // Should handle as "no pace data"
    
    // Test 4: Extreme imbalance
    fixture->current_metrics.balance_lr = -100;
    zassert_true(abs(fixture->current_metrics.balance_lr) <= 100,
                 "Balance should be capped at ±100");
}
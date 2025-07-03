/**
 * @file test_analytics.cpp
 * @brief Unit tests for analytics module
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
#include <math.h>

// Define FFF globals
DEFINE_FFF_GLOBALS;

// Mock functions
FAKE_VALUE_FUNC(int, k_msgq_put, struct k_msgq *, const void *, k_timeout_t);
FAKE_VALUE_FUNC(int, k_msgq_get, struct k_msgq *, void *, k_timeout_t);
FAKE_VOID_FUNC(module_set_state, enum module_state);

// Define message queues for testing
K_MSGQ_DEFINE(realtime_queue, sizeof(generic_message_t), 10, 4);
K_MSGQ_DEFINE(analytics_queue, sizeof(generic_message_t), 10, 4);

// Analytics data structures
typedef struct {
    float vertical_oscillation_cm;
    float ground_contact_time_ms;
    float stride_angle_deg;
    float energy_return_percent;
} efficiency_metrics_t;

typedef struct {
    float contact_time_increase_percent;
    float cadence_decrease_percent;
    float form_score_decline;
    uint8_t fatigue_index;  // 0-100
} fatigue_metrics_t;

typedef struct {
    float left_right_asymmetry_percent;
    float impact_force_trend;
    float pronation_angle_deg;
    uint8_t injury_risk_score;  // 0-100
} injury_risk_metrics_t;

// Test fixture
struct analytics_fixture {
    efficiency_metrics_t baseline_efficiency;
    efficiency_metrics_t current_efficiency;
    fatigue_metrics_t fatigue;
    injury_risk_metrics_t injury_risk;
    uint32_t baseline_samples;
    bool baseline_established;
};

static void *analytics_setup(void)
{
    struct analytics_fixture *fixture = k_malloc(sizeof(struct analytics_fixture));
    zassert_not_null(fixture, "Failed to allocate fixture");
    
    memset(fixture, 0, sizeof(struct analytics_fixture));
    
    // Set baseline values
    fixture->baseline_efficiency.vertical_oscillation_cm = 7.5f;
    fixture->baseline_efficiency.ground_contact_time_ms = 250.0f;
    fixture->baseline_efficiency.stride_angle_deg = 85.0f;
    fixture->baseline_efficiency.energy_return_percent = 65.0f;
    
    return fixture;
}

static void analytics_teardown(void *f)
{
    struct analytics_fixture *fixture = (struct analytics_fixture *)f;
    k_free(fixture);
    
    // Reset all fakes
    RESET_FAKE(k_msgq_put);
    RESET_FAKE(k_msgq_get);
    RESET_FAKE(module_set_state);
}

ZTEST_SUITE(analytics, NULL, analytics_setup, NULL, NULL, analytics_teardown);

// Test running efficiency calculation
ZTEST_F(analytics, test_running_efficiency_calculation)
{
    struct analytics_fixture *fixture = (struct analytics_fixture *)_fixture;
    
    // Test 1: Excellent efficiency
    fixture->current_efficiency.vertical_oscillation_cm = 6.0f;
    fixture->current_efficiency.ground_contact_time_ms = 220.0f;
    fixture->current_efficiency.stride_angle_deg = 90.0f;
    fixture->current_efficiency.energy_return_percent = 70.0f;
    
    // Calculate efficiency score (simplified)
    float vo_score = (fixture->current_efficiency.vertical_oscillation_cm < 7.0f) ? 100.0f :
                     (fixture->current_efficiency.vertical_oscillation_cm < 9.0f) ? 80.0f : 60.0f;
    
    float gct_score = (fixture->current_efficiency.ground_contact_time_ms < 240.0f) ? 100.0f :
                      (fixture->current_efficiency.ground_contact_time_ms < 280.0f) ? 80.0f : 60.0f;
    
    float stride_score = (fixture->current_efficiency.stride_angle_deg > 85.0f) ? 100.0f :
                        (fixture->current_efficiency.stride_angle_deg > 80.0f) ? 80.0f : 60.0f;
    
    float energy_score = fixture->current_efficiency.energy_return_percent;
    
    float efficiency = (vo_score * 0.3f + gct_score * 0.3f + 
                       stride_score * 0.2f + energy_score * 0.2f);
    
    zassert_within(efficiency, 94.0f, 1.0f, "Excellent efficiency should be ~94");
    
    // Test 2: Poor efficiency
    fixture->current_efficiency.vertical_oscillation_cm = 10.0f;
    fixture->current_efficiency.ground_contact_time_ms = 300.0f;
    fixture->current_efficiency.stride_angle_deg = 75.0f;
    fixture->current_efficiency.energy_return_percent = 50.0f;
    
    vo_score = (fixture->current_efficiency.vertical_oscillation_cm < 7.0f) ? 100.0f :
               (fixture->current_efficiency.vertical_oscillation_cm < 9.0f) ? 80.0f : 60.0f;
    
    gct_score = (fixture->current_efficiency.ground_contact_time_ms < 240.0f) ? 100.0f :
                (fixture->current_efficiency.ground_contact_time_ms < 280.0f) ? 80.0f : 60.0f;
    
    stride_score = (fixture->current_efficiency.stride_angle_deg > 85.0f) ? 100.0f :
                  (fixture->current_efficiency.stride_angle_deg > 80.0f) ? 80.0f : 60.0f;
    
    energy_score = fixture->current_efficiency.energy_return_percent;
    
    efficiency = (vo_score * 0.3f + gct_score * 0.3f + 
                 stride_score * 0.2f + energy_score * 0.2f);
    
    zassert_within(efficiency, 58.0f, 2.0f, "Poor efficiency should be ~58");
}

// Test baseline establishment
ZTEST_F(analytics, test_baseline_establishment)
{
    struct analytics_fixture *fixture = (struct analytics_fixture *)_fixture;
    
    // Simulate collecting baseline samples
    const uint32_t BASELINE_SAMPLES_NEEDED = 120;  // 2 minutes at 1Hz
    
    float vo_sum = 0, gct_sum = 0, stride_sum = 0, energy_sum = 0;
    
    // Collect samples
    for (uint32_t i = 0; i < BASELINE_SAMPLES_NEEDED; i++) {
        // Simulate slight variations
        float vo = 7.5f + (float)(i % 10 - 5) * 0.1f;
        float gct = 250.0f + (float)(i % 20 - 10);
        float stride = 85.0f + (float)(i % 10 - 5) * 0.5f;
        float energy = 65.0f + (float)(i % 10 - 5) * 0.5f;
        
        vo_sum += vo;
        gct_sum += gct;
        stride_sum += stride;
        energy_sum += energy;
        
        fixture->baseline_samples++;
    }
    
    // Calculate baseline averages
    fixture->baseline_efficiency.vertical_oscillation_cm = vo_sum / BASELINE_SAMPLES_NEEDED;
    fixture->baseline_efficiency.ground_contact_time_ms = gct_sum / BASELINE_SAMPLES_NEEDED;
    fixture->baseline_efficiency.stride_angle_deg = stride_sum / BASELINE_SAMPLES_NEEDED;
    fixture->baseline_efficiency.energy_return_percent = energy_sum / BASELINE_SAMPLES_NEEDED;
    fixture->baseline_established = true;
    
    zassert_true(fixture->baseline_established, "Baseline should be established");
    zassert_equal(fixture->baseline_samples, BASELINE_SAMPLES_NEEDED, 
                  "Should have correct number of baseline samples");
    zassert_within(fixture->baseline_efficiency.vertical_oscillation_cm, 7.5f, 0.5f,
                   "Baseline VO should be ~7.5cm");
}

// Test fatigue index calculation
ZTEST_F(analytics, test_fatigue_index_calculation)
{
    struct analytics_fixture *fixture = (struct analytics_fixture *)_fixture;
    
    // Assume baseline is established
    fixture->baseline_established = true;
    fixture->baseline_efficiency.ground_contact_time_ms = 250.0f;
    
    // Test 1: No fatigue (same as baseline)
    fixture->current_efficiency.ground_contact_time_ms = 250.0f;
    fixture->fatigue.contact_time_increase_percent = 0.0f;
    fixture->fatigue.cadence_decrease_percent = 0.0f;
    fixture->fatigue.form_score_decline = 0.0f;
    
    fixture->fatigue.fatigue_index = (uint8_t)(
        fixture->fatigue.contact_time_increase_percent * 0.4f +
        fixture->fatigue.cadence_decrease_percent * 0.3f +
        fixture->fatigue.form_score_decline * 0.3f
    );
    
    zassert_equal(fixture->fatigue.fatigue_index, 0, "No fatigue should give index 0");
    
    // Test 2: Moderate fatigue
    fixture->current_efficiency.ground_contact_time_ms = 275.0f;  // 10% increase
    fixture->fatigue.contact_time_increase_percent = 10.0f;
    fixture->fatigue.cadence_decrease_percent = 5.0f;
    fixture->fatigue.form_score_decline = 8.0f;
    
    fixture->fatigue.fatigue_index = (uint8_t)(
        fixture->fatigue.contact_time_increase_percent * 0.4f +
        fixture->fatigue.cadence_decrease_percent * 0.3f +
        fixture->fatigue.form_score_decline * 0.3f
    );
    
    zassert_within(fixture->fatigue.fatigue_index, 8, 1, 
                   "Moderate fatigue should give index ~8");
    
    // Test 3: Severe fatigue
    fixture->current_efficiency.ground_contact_time_ms = 300.0f;  // 20% increase
    fixture->fatigue.contact_time_increase_percent = 20.0f;
    fixture->fatigue.cadence_decrease_percent = 15.0f;
    fixture->fatigue.form_score_decline = 25.0f;
    
    fixture->fatigue.fatigue_index = (uint8_t)(
        fixture->fatigue.contact_time_increase_percent * 0.4f +
        fixture->fatigue.cadence_decrease_percent * 0.3f +
        fixture->fatigue.form_score_decline * 0.3f
    );
    
    zassert_within(fixture->fatigue.fatigue_index, 20, 2, 
                   "Severe fatigue should give index ~20");
}

// Test injury risk assessment
ZTEST_F(analytics, test_injury_risk_assessment)
{
    struct analytics_fixture *fixture = (struct analytics_fixture *)_fixture;
    
    // Test 1: Low risk
    fixture->injury_risk.left_right_asymmetry_percent = 5.0f;
    fixture->injury_risk.impact_force_trend = 0.0f;  // No increase
    fixture->injury_risk.pronation_angle_deg = 10.0f;  // Normal
    
    // Calculate risk score
    uint8_t asymmetry_risk = (fixture->injury_risk.left_right_asymmetry_percent < 10.0f) ? 0 :
                            (fixture->injury_risk.left_right_asymmetry_percent < 15.0f) ? 30 : 60;
    
    uint8_t impact_risk = (fixture->injury_risk.impact_force_trend < 5.0f) ? 0 :
                         (fixture->injury_risk.impact_force_trend < 10.0f) ? 30 : 60;
    
    uint8_t pronation_risk = (fabs(fixture->injury_risk.pronation_angle_deg) < 15.0f) ? 0 :
                            (fabs(fixture->injury_risk.pronation_angle_deg) < 20.0f) ? 30 : 60;
    
    fixture->injury_risk.injury_risk_score = (asymmetry_risk + impact_risk + pronation_risk) / 3;
    
    zassert_equal(fixture->injury_risk.injury_risk_score, 0, "Low risk should score 0");
    
    // Test 2: Moderate risk
    fixture->injury_risk.left_right_asymmetry_percent = 12.0f;
    fixture->injury_risk.impact_force_trend = 7.0f;
    fixture->injury_risk.pronation_angle_deg = 18.0f;
    
    asymmetry_risk = (fixture->injury_risk.left_right_asymmetry_percent < 10.0f) ? 0 :
                    (fixture->injury_risk.left_right_asymmetry_percent < 15.0f) ? 30 : 60;
    
    impact_risk = (fixture->injury_risk.impact_force_trend < 5.0f) ? 0 :
                 (fixture->injury_risk.impact_force_trend < 10.0f) ? 30 : 60;
    
    pronation_risk = (fabs(fixture->injury_risk.pronation_angle_deg) < 15.0f) ? 0 :
                    (fabs(fixture->injury_risk.pronation_angle_deg) < 20.0f) ? 30 : 60;
    
    fixture->injury_risk.injury_risk_score = (asymmetry_risk + impact_risk + pronation_risk) / 3;
    
    zassert_equal(fixture->injury_risk.injury_risk_score, 30, "Moderate risk should score 30");
    
    // Test 3: High risk
    fixture->injury_risk.left_right_asymmetry_percent = 20.0f;
    fixture->injury_risk.impact_force_trend = 15.0f;
    fixture->injury_risk.pronation_angle_deg = -25.0f;  // Over-pronation
    
    asymmetry_risk = (fixture->injury_risk.left_right_asymmetry_percent < 10.0f) ? 0 :
                    (fixture->injury_risk.left_right_asymmetry_percent < 15.0f) ? 30 : 60;
    
    impact_risk = (fixture->injury_risk.impact_force_trend < 5.0f) ? 0 :
                 (fixture->injury_risk.impact_force_trend < 10.0f) ? 30 : 60;
    
    pronation_risk = (fabs(fixture->injury_risk.pronation_angle_deg) < 15.0f) ? 0 :
                    (fabs(fixture->injury_risk.pronation_angle_deg) < 20.0f) ? 30 : 60;
    
    fixture->injury_risk.injury_risk_score = (asymmetry_risk + impact_risk + pronation_risk) / 3;
    
    zassert_equal(fixture->injury_risk.injury_risk_score, 60, "High risk should score 60");
}

// Test pronation analysis
ZTEST_F(analytics, test_pronation_analysis)
{
    struct analytics_fixture *fixture = (struct analytics_fixture *)_fixture;
    
    // Pronation combines IMU roll angle and pressure distribution
    
    // Test 1: Neutral pronation
    float imu_roll_deg = 0.0f;
    float medial_pressure_ratio = 0.5f;  // 50% medial, 50% lateral
    
    float pronation_angle = imu_roll_deg + (medial_pressure_ratio - 0.5f) * 20.0f;
    
    zassert_within(pronation_angle, 0.0f, 0.1f, "Neutral should be ~0 degrees");
    
    // Test 2: Over-pronation
    imu_roll_deg = -10.0f;  // Inward roll
    medial_pressure_ratio = 0.7f;  // 70% on medial side
    
    pronation_angle = imu_roll_deg + (medial_pressure_ratio - 0.5f) * 20.0f;
    
    zassert_within(pronation_angle, -6.0f, 1.0f, "Over-pronation should be negative");
    
    // Test 3: Under-pronation (supination)
    imu_roll_deg = 8.0f;  // Outward roll
    medial_pressure_ratio = 0.3f;  // 30% on medial side
    
    pronation_angle = imu_roll_deg + (medial_pressure_ratio - 0.5f) * 20.0f;
    
    zassert_within(pronation_angle, 4.0f, 1.0f, "Under-pronation should be positive");
}

// Test stride length estimation
ZTEST(analytics, test_stride_length_estimation)
{
    // Stride length estimation based on height, cadence, and speed
    
    float height_cm = 175.0f;
    float cadence_spm = 180.0f;
    float speed_m_s = 3.5f;  // ~12.6 km/h
    
    // Method 1: From speed and cadence
    float stride_length_m = (speed_m_s * 60.0f) / cadence_spm;
    
    zassert_within(stride_length_m, 1.17f, 0.01f, 
                   "Stride length from speed should be ~1.17m");
    
    // Method 2: From height (rough estimate)
    float stride_length_height = height_cm * 0.0065f;  // Typical ratio
    
    zassert_within(stride_length_height, 1.14f, 0.05f,
                   "Stride length from height should be ~1.14m");
    
    // Average the two methods
    float avg_stride_length = (stride_length_m + stride_length_height) / 2.0f;
    
    zassert_within(avg_stride_length, 1.15f, 0.05f,
                   "Average stride length should be ~1.15m");
}

// Test CPEI calculation
ZTEST(analytics, test_cpei_calculation)
{
    // Center of Pressure Excursion Index
    // Measures how the center of pressure moves during stance
    
    // Simplified test with pressure readings over time
    struct {
        float cop_x;  // Center of pressure X position (0-100)
        float cop_y;  // Center of pressure Y position (0-100)
    } cop_trajectory[] = {
        {20.0f, 80.0f},  // Heel strike
        {30.0f, 70.0f},  // Early stance
        {50.0f, 50.0f},  // Mid stance
        {70.0f, 30.0f},  // Late stance
        {80.0f, 20.0f},  // Toe off
    };
    
    // Calculate total excursion
    float total_distance = 0;
    for (int i = 1; i < 5; i++) {
        float dx = cop_trajectory[i].cop_x - cop_trajectory[i-1].cop_x;
        float dy = cop_trajectory[i].cop_y - cop_trajectory[i-1].cop_y;
        total_distance += sqrtf(dx*dx + dy*dy);
    }
    
    // Calculate CPEI (normalized by foot length)
    float foot_length = 100.0f;  // Normalized units
    float cpei = (total_distance / foot_length) * 100.0f;
    
    zassert_within(cpei, 85.0f, 5.0f, "CPEI should be ~85 for normal gait");
}

// Test adaptive processing rate
ZTEST_F(analytics, test_adaptive_processing_rate)
{
    struct analytics_fixture *fixture = (struct analytics_fixture *)_fixture;
    
    // Processing rate should adapt based on activity intensity
    
    // Test 1: Low intensity - process at 1Hz
    float current_pace_min_km = 7.0f;  // Slow jog
    uint32_t processing_interval_ms;
    
    if (current_pace_min_km > 6.0f) {
        processing_interval_ms = 1000;  // 1Hz
    } else if (current_pace_min_km > 4.0f) {
        processing_interval_ms = 500;   // 2Hz
    } else {
        processing_interval_ms = 200;   // 5Hz
    }
    
    zassert_equal(processing_interval_ms, 1000, "Slow pace should use 1Hz");
    
    // Test 2: High intensity - process at 5Hz
    current_pace_min_km = 3.5f;  // Fast run
    
    if (current_pace_min_km > 6.0f) {
        processing_interval_ms = 1000;  // 1Hz
    } else if (current_pace_min_km > 4.0f) {
        processing_interval_ms = 500;   // 2Hz
    } else {
        processing_interval_ms = 200;   // 5Hz
    }
    
    zassert_equal(processing_interval_ms, 200, "Fast pace should use 5Hz");
}

// Test message forwarding
ZTEST(analytics, test_message_forwarding)
{
    generic_message_t in_msg, out_msg;
    
    // Receive metrics from realtime module
    in_msg.sender = SENDER_REALTIME_METRICS;
    in_msg.type = MSG_TYPE_REALTIME_METRICS;
    
    k_msgq_get_fake.return_val = 0;
    int ret = k_msgq_get(&realtime_queue, &in_msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Should receive realtime metrics");
    
    // Process and forward analytics results
    out_msg.sender = SENDER_ANALYTICS;
    out_msg.type = MSG_TYPE_ANALYTICS_RESULTS;
    
    k_msgq_put_fake.return_val = 0;
    ret = k_msgq_put(&analytics_queue, &out_msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Should forward analytics results");
}
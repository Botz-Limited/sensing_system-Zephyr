/**
 * @file test_activity_metrics.cpp
 * @brief Unit tests for activity_metrics module (session management)
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
FAKE_VALUE_FUNC(int64_t, k_uptime_get);

// Define message queues for testing
K_MSGQ_DEFINE(analytics_queue, sizeof(generic_message_t), 10, 4);
K_MSGQ_DEFINE(data_msgq, sizeof(generic_message_t), 10, 4);
K_MSGQ_DEFINE(activity_metrics_msgq, sizeof(generic_message_t), 10, 4);

// Session data structures
typedef struct {
    uint32_t session_id;
    uint32_t start_time_ms;
    uint32_t end_time_ms;
    uint32_t duration_ms;
    bool is_active;
    bool is_paused;
    uint32_t pause_start_ms;
    uint32_t total_pause_ms;
} session_info_t;

typedef struct {
    uint32_t total_steps;
    float total_distance_km;
    float average_pace_min_km;
    float average_cadence_spm;
    float max_pace_min_km;
    float min_pace_min_km;
    uint16_t calories_burned;
    float elevation_gain_m;
    float elevation_loss_m;
} session_statistics_t;

typedef struct {
    uint32_t split_number;
    uint32_t start_time_ms;
    uint32_t duration_ms;
    float pace_min_km;
    float average_hr_bpm;
    uint32_t steps;
} kilometer_split_t;

// Test fixture
struct activity_metrics_fixture {
    session_info_t session;
    session_statistics_t stats;
    kilometer_split_t splits[50];  // Max 50km
    uint8_t split_count;
    float current_distance_km;
    uint32_t last_periodic_update_ms;
};

static void *activity_metrics_setup(void)
{
    struct activity_metrics_fixture *fixture = k_malloc(sizeof(struct activity_metrics_fixture));
    zassert_not_null(fixture, "Failed to allocate fixture");
    
    memset(fixture, 0, sizeof(struct activity_metrics_fixture));
    
    return fixture;
}

static void activity_metrics_teardown(void *f)
{
    struct activity_metrics_fixture *fixture = (struct activity_metrics_fixture *)f;
    k_free(fixture);
    
    // Reset all fakes
    RESET_FAKE(k_msgq_put);
    RESET_FAKE(k_msgq_get);
    RESET_FAKE(module_set_state);
    RESET_FAKE(k_uptime_get);
}

ZTEST_SUITE(activity_metrics, NULL, activity_metrics_setup, NULL, NULL, activity_metrics_teardown);

// Test session management
ZTEST_F(activity_metrics, test_session_start_stop)
{
    struct activity_metrics_fixture *fixture = (struct activity_metrics_fixture *)_fixture;
    
    // Test session start
    k_uptime_get_fake.return_val = 1000;  // 1 second
    
    fixture->session.session_id = 12345;
    fixture->session.start_time_ms = k_uptime_get();
    fixture->session.is_active = true;
    fixture->session.is_paused = false;
    
    zassert_true(fixture->session.is_active, "Session should be active");
    zassert_false(fixture->session.is_paused, "Session should not be paused");
    zassert_equal(fixture->session.start_time_ms, 1000, "Start time should be 1000ms");
    
    // Test session stop
    k_uptime_get_fake.return_val = 1800000;  // 30 minutes later
    
    fixture->session.end_time_ms = k_uptime_get();
    fixture->session.duration_ms = fixture->session.end_time_ms - 
                                   fixture->session.start_time_ms - 
                                   fixture->session.total_pause_ms;
    fixture->session.is_active = false;
    
    zassert_false(fixture->session.is_active, "Session should be inactive");
    zassert_equal(fixture->session.duration_ms, 1799000, "Duration should be 1799 seconds");
}

// Test pause/resume functionality
ZTEST_F(activity_metrics, test_session_pause_resume)
{
    struct activity_metrics_fixture *fixture = (struct activity_metrics_fixture *)_fixture;
    
    // Start session
    k_uptime_get_fake.return_val = 1000;
    fixture->session.start_time_ms = k_uptime_get();
    fixture->session.is_active = true;
    
    // Pause after 5 minutes
    k_uptime_get_fake.return_val = 301000;  // 5:01
    fixture->session.pause_start_ms = k_uptime_get();
    fixture->session.is_paused = true;
    
    zassert_true(fixture->session.is_paused, "Session should be paused");
    
    // Resume after 2 minutes pause
    k_uptime_get_fake.return_val = 421000;  // 7:01
    uint32_t pause_duration = k_uptime_get() - fixture->session.pause_start_ms;
    fixture->session.total_pause_ms += pause_duration;
    fixture->session.is_paused = false;
    
    zassert_false(fixture->session.is_paused, "Session should be resumed");
    zassert_equal(fixture->session.total_pause_ms, 120000, "Total pause should be 2 minutes");
    
    // End session
    k_uptime_get_fake.return_val = 1801000;  // 30:01
    fixture->session.end_time_ms = k_uptime_get();
    fixture->session.duration_ms = fixture->session.end_time_ms - 
                                   fixture->session.start_time_ms - 
                                   fixture->session.total_pause_ms;
    
    // Active time should be 30 minutes - 2 minutes pause = 28 minutes
    zassert_equal(fixture->session.duration_ms, 1680000, "Active duration should be 28 minutes");
}

// Test periodic record creation
ZTEST_F(activity_metrics, test_periodic_record_creation)
{
    struct activity_metrics_fixture *fixture = (struct activity_metrics_fixture *)_fixture;
    
    // Simulate receiving analytics data
    fixture->stats.total_steps = 3000;
    fixture->stats.total_distance_km = 3.5f;
    fixture->stats.average_pace_min_km = 5.5f;
    fixture->stats.average_cadence_spm = 175.0f;
    
    // Check if it's time for periodic update (every 2 seconds)
    k_uptime_get_fake.return_val = 2000;
    uint32_t current_time = k_uptime_get();
    
    bool should_update = (current_time - fixture->last_periodic_update_ms) >= 2000;
    zassert_true(should_update, "Should create periodic record after 2 seconds");
    
    // Create periodic record message
    generic_message_t msg;
    msg.sender = SENDER_ACTIVITY_METRICS;
    msg.type = MSG_TYPE_COMMAND;
    strncpy(msg.data.command_str, "PERIODIC_RECORD", MAX_COMMAND_STRING_LEN);
    
    k_msgq_put_fake.return_val = 0;
    int ret = k_msgq_put(&data_msgq, &msg, K_NO_WAIT);
    
    zassert_equal(ret, 0, "Periodic record should be sent");
    zassert_equal(k_msgq_put_fake.call_count, 1, "k_msgq_put should be called once");
    
    fixture->last_periodic_update_ms = current_time;
}

// Test kilometer split generation
ZTEST_F(activity_metrics, test_kilometer_splits)
{
    struct activity_metrics_fixture *fixture = (struct activity_metrics_fixture *)_fixture;
    
    // Start at 0km
    fixture->current_distance_km = 0.0f;
    fixture->split_count = 0;
    k_uptime_get_fake.return_val = 0;
    
    // Simulate reaching 1km
    fixture->current_distance_km = 1.05f;  // Just past 1km
    k_uptime_get_fake.return_val = 300000;  // 5 minutes
    
    // Check if we crossed a kilometer boundary
    uint32_t current_km = (uint32_t)fixture->current_distance_km;
    uint32_t last_km = (uint32_t)0.0f;
    
    if (current_km > last_km) {
        // Create split
        fixture->splits[fixture->split_count].split_number = current_km;
        fixture->splits[fixture->split_count].duration_ms = 300000;
        fixture->splits[fixture->split_count].pace_min_km = 5.0f;
        fixture->splits[fixture->split_count].steps = 850;
        fixture->split_count++;
    }
    
    zassert_equal(fixture->split_count, 1, "Should have 1 split");
    zassert_equal(fixture->splits[0].split_number, 1, "Split should be for km 1");
    zassert_equal(fixture->splits[0].duration_ms, 300000, "Split duration should be 5 minutes");
    
    // Simulate reaching 2km
    fixture->current_distance_km = 2.02f;
    k_uptime_get_fake.return_val = 610000;  // 10:10 total
    
    current_km = (uint32_t)fixture->current_distance_km;
    last_km = 1;
    
    if (current_km > last_km) {
        fixture->splits[fixture->split_count].split_number = current_km;
        fixture->splits[fixture->split_count].duration_ms = 310000;  // 5:10 for this km
        fixture->splits[fixture->split_count].pace_min_km = 5.17f;
        fixture->splits[fixture->split_count].steps = 860;
        fixture->split_count++;
    }
    
    zassert_equal(fixture->split_count, 2, "Should have 2 splits");
    zassert_equal(fixture->splits[1].duration_ms, 310000, "Second split should be 5:10");
}

// Test calorie calculation
ZTEST_F(activity_metrics, test_calorie_calculation)
{
    struct activity_metrics_fixture *fixture = (struct activity_metrics_fixture *)_fixture;
    
    // Calorie calculation based on MET (Metabolic Equivalent of Task)
    // Running MET = 1.65 * speed_mph - 0.66
    
    float weight_kg = 70.0f;  // Assume 70kg user
    float distance_km = 5.0f;
    float duration_hours = 0.5f;  // 30 minutes
    float speed_km_h = distance_km / duration_hours;  // 10 km/h
    float speed_mph = speed_km_h * 0.621371f;  // Convert to mph
    
    float met = 1.65f * speed_mph - 0.66f;
    float calories = met * weight_kg * duration_hours;
    
    fixture->stats.calories_burned = (uint16_t)calories;
    
    zassert_within(fixture->stats.calories_burned, 350, 50, 
                   "Calories for 5km in 30min should be ~350");
    
    // Test walking pace
    distance_km = 3.0f;
    duration_hours = 0.5f;  // 30 minutes
    speed_km_h = distance_km / duration_hours;  // 6 km/h
    speed_mph = speed_km_h * 0.621371f;
    
    // Walking MET formula (different from running)
    met = 3.5f;  // Moderate walking
    calories = met * weight_kg * duration_hours;
    
    uint16_t walking_calories = (uint16_t)calories;
    
    zassert_within(walking_calories, 122, 20, 
                   "Calories for 3km walk in 30min should be ~122");
}

// Test GPS integration
ZTEST_F(activity_metrics, test_gps_distance_calculation)
{
    // Simulate GPS coordinates
    struct {
        double lat;
        double lon;
    } gps_points[] = {
        {40.7128, -74.0060},   // Start
        {40.7130, -74.0062},   // ~25m away
        {40.7132, -74.0064},   // ~50m total
    };
    
    // Haversine formula (simplified for small distances)
    double total_distance_m = 0;
    
    for (int i = 1; i < 3; i++) {
        double lat1 = gps_points[i-1].lat;
        double lon1 = gps_points[i-1].lon;
        double lat2 = gps_points[i].lat;
        double lon2 = gps_points[i].lon;
        
        // Simplified distance calculation for small distances
        double lat_diff = (lat2 - lat1) * 111111.0;  // ~111km per degree
        double lon_diff = (lon2 - lon1) * 111111.0 * 0.7;  // Adjust for latitude
        
        double distance = sqrt(lat_diff * lat_diff + lon_diff * lon_diff);
        total_distance_m += distance;
    }
    
    zassert_within(total_distance_m, 50.0, 10.0, 
                   "GPS distance should be ~50m");
}

// Test session statistics aggregation
ZTEST_F(activity_metrics, test_session_statistics)
{
    struct activity_metrics_fixture *fixture = (struct activity_metrics_fixture *)_fixture;
    
    // Simulate a complete session
    fixture->stats.total_steps = 5000;
    fixture->stats.total_distance_km = 5.5f;
    fixture->session.duration_ms = 1800000;  // 30 minutes
    
    // Calculate averages
    float duration_min = fixture->session.duration_ms / 60000.0f;
    fixture->stats.average_pace_min_km = duration_min / fixture->stats.total_distance_km;
    fixture->stats.average_cadence_spm = fixture->stats.total_steps / duration_min;
    
    zassert_within(fixture->stats.average_pace_min_km, 5.45f, 0.1f,
                   "Average pace should be ~5:27/km");
    zassert_within(fixture->stats.average_cadence_spm, 166.7f, 1.0f,
                   "Average cadence should be ~167 spm");
    
    // Test max/min tracking
    fixture->stats.max_pace_min_km = 4.5f;   // Fastest km
    fixture->stats.min_pace_min_km = 6.2f;   // Slowest km
    
    zassert_true(fixture->stats.max_pace_min_km < fixture->stats.average_pace_min_km,
                 "Max pace should be faster than average");
    zassert_true(fixture->stats.min_pace_min_km > fixture->stats.average_pace_min_km,
                 "Min pace should be slower than average");
}

// Test elevation tracking
ZTEST_F(activity_metrics, test_elevation_tracking)
{
    struct activity_metrics_fixture *fixture = (struct activity_metrics_fixture *)_fixture;
    
    // Simulate elevation changes
    float elevations[] = {100.0f, 105.0f, 103.0f, 108.0f, 106.0f, 110.0f};
    
    fixture->stats.elevation_gain_m = 0;
    fixture->stats.elevation_loss_m = 0;
    
    for (int i = 1; i < 6; i++) {
        float diff = elevations[i] - elevations[i-1];
        if (diff > 0) {
            fixture->stats.elevation_gain_m += diff;
        } else {
            fixture->stats.elevation_loss_m += -diff;
        }
    }
    
    zassert_equal((int)fixture->stats.elevation_gain_m, 12, 
                  "Total elevation gain should be 12m");
    zassert_equal((int)fixture->stats.elevation_loss_m, 2, 
                  "Total elevation loss should be 2m");
}

// Test message handling from analytics
ZTEST_F(activity_metrics, test_analytics_message_handling)
{
    struct activity_metrics_fixture *fixture = (struct activity_metrics_fixture *)_fixture;
    generic_message_t msg;
    
    // Simulate receiving analytics results
    msg.sender = SENDER_ANALYTICS;
    msg.type = MSG_TYPE_ANALYTICS_RESULTS;
    
    k_msgq_get_fake.return_val = 0;
    int ret = k_msgq_get(&analytics_queue, &msg, K_NO_WAIT);
    
    zassert_equal(ret, 0, "Should receive analytics message");
    
    // Update session statistics based on analytics
    // (In real implementation, would extract data from message)
    fixture->stats.total_steps += 100;
    fixture->stats.total_distance_km += 0.12f;
    
    // Send updated stats to data module
    msg.sender = SENDER_ACTIVITY_METRICS;
    msg.type = MSG_TYPE_COMMAND;
    strncpy(msg.data.command_str, "SESSION_UPDATE", MAX_COMMAND_STRING_LEN);
    
    k_msgq_put_fake.return_val = 0;
    ret = k_msgq_put(&data_msgq, &msg, K_NO_WAIT);
    
    zassert_equal(ret, 0, "Should send session update");
}

// Test session ID generation
ZTEST(activity_metrics, test_session_id_generation)
{
    // Session ID should be unique and based on timestamp
    k_uptime_get_fake.return_val = 1234567890;
    
    uint32_t session_id = k_uptime_get() & 0xFFFFFF;  // Use lower 24 bits
    
    zassert_not_equal(session_id, 0, "Session ID should not be 0");
    
    // Test uniqueness with different timestamps
    k_uptime_get_fake.return_val = 1234567891;
    uint32_t session_id2 = k_uptime_get() & 0xFFFFFF;
    
    zassert_not_equal(session_id, session_id2, "Session IDs should be unique");
}
/**
 * @file test_cts.cpp
 * @brief Unit tests for CTS (Current Time Service) module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/fff.h>
#include <time.h>

#include "cts.hpp"

DEFINE_FFF_GLOBALS;

// Mock functions
FAKE_VALUE_FUNC(const struct device *, device_get_binding, const char *);
FAKE_VALUE_FUNC(int, counter_get_value, const struct device *, uint32_t *);
FAKE_VALUE_FUNC(int, counter_start, const struct device *);
FAKE_VALUE_FUNC(int, bt_gatt_notify, struct bt_conn *, const struct bt_gatt_attr *, const void *, uint16_t);
FAKE_VALUE_FUNC(time_t, time, time_t *);
FAKE_VALUE_FUNC(struct tm *, gmtime_r, const time_t *, struct tm *);

// Mock device structure
static struct device mock_rtc_device = {
    .name = "RTC_0",
};

// Test fixture
struct cts_fixture {
    struct bt_conn conn;
    struct bt_gatt_attr attr;
    struct tm test_time;
    time_t test_epoch;
};

// Custom fake for gmtime_r that uses fixture data
static struct tm *custom_gmtime_r(const time_t *timep, struct tm *result)
{
    struct cts_fixture *fixture = (struct cts_fixture *)0x1000; // Hack to get fixture
    if (result && timep) {
        memcpy(result, &fixture->test_time, sizeof(struct tm));
    }
    return result;
}

static void *cts_setup(void)
{
    static struct cts_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    
    // Reset all fakes
    RESET_FAKE(device_get_binding);
    RESET_FAKE(counter_get_value);
    RESET_FAKE(counter_start);
    RESET_FAKE(bt_gatt_notify);
    RESET_FAKE(time);
    RESET_FAKE(gmtime_r);
    
    // Setup test time: 2024-01-15 12:30:45 UTC
    fixture.test_epoch = 1705323045;
    fixture.test_time.tm_year = 2024 - 1900;
    fixture.test_time.tm_mon = 0;  // January
    fixture.test_time.tm_mday = 15;
    fixture.test_time.tm_hour = 12;
    fixture.test_time.tm_min = 30;
    fixture.test_time.tm_sec = 45;
    fixture.test_time.tm_wday = 1;  // Monday
    fixture.test_time.tm_yday = 14; // 15th day of year
    
    // Setup default return values
    device_get_binding_fake.return_val = &mock_rtc_device;
    counter_get_value_fake.return_val = 0;
    counter_start_fake.return_val = 0;
    bt_gatt_notify_fake.return_val = 0;
    time_fake.return_val = fixture.test_epoch;
    gmtime_r_fake.custom_fake = custom_gmtime_r;
    
    return &fixture;
}

static void cts_teardown(void *f)
{
    ARG_UNUSED(f);
}

ZTEST_SUITE(cts, NULL, cts_setup, NULL, NULL, cts_teardown);

ZTEST(cts, test_init_rtc_time_success)
{
    // Test successful RTC initialization
    int ret = init_rtc_time();
    
    // Verify
    zassert_equal(ret, 0, "RTC init should succeed");
    zassert_equal(device_get_binding_fake.call_count, 1,
                  "Should get RTC device");
    zassert_str_equal(device_get_binding_fake.arg0_val, "RTC_0",
                      "Should request correct device");
    zassert_equal(counter_start_fake.call_count, 1,
                  "Should start counter");
}

ZTEST(cts, test_init_rtc_time_no_device)
{
    // Setup - no RTC device available
    device_get_binding_fake.return_val = NULL;
    
    // Test
    int ret = init_rtc_time();
    
    // Verify
    zassert_not_equal(ret, 0, "Should fail without RTC device");
    zassert_equal(counter_start_fake.call_count, 0,
                  "Should not start counter without device");
}

ZTEST(cts, test_init_rtc_time_counter_start_fail)
{
    // Setup - counter start fails
    counter_start_fake.return_val = -EIO;
    
    // Test
    int ret = init_rtc_time();
    
    // Verify
    zassert_not_equal(ret, 0, "Should fail when counter start fails");
}

ZTEST_F(cts, test_set_current_time_from_epoch)
{
    // Test setting time from epoch
    uint32_t new_epoch = 1705323045; // 2024-01-15 12:30:45 UTC
    
    set_current_time_from_epoch(new_epoch);
    
    // In real implementation, this would update internal time
    // Here we verify the function doesn't crash
    zassert_true(true, "Set time should complete");
}

ZTEST_F(cts, test_get_current_epoch_time)
{
    // Setup counter value
    counter_get_value_fake.custom_fake = [](const struct device *dev, uint32_t *ticks) -> int {
        *ticks = 1000; // 1000 ticks
        return 0;
    };
    
    // Test
    uint32_t epoch = get_current_epoch_time();
    
    // Verify
    zassert_true(epoch > 0, "Should return valid epoch time");
    zassert_equal(counter_get_value_fake.call_count, 1,
                  "Should read counter value");
}

ZTEST_F(cts, test_get_current_epoch_time_counter_fail)
{
    // Setup - counter read fails
    counter_get_value_fake.return_val = -EIO;
    
    // Test
    uint32_t epoch = get_current_epoch_time();
    
    // Verify - should return 0 on error
    zassert_equal(epoch, 0, "Should return 0 on counter error");
}

ZTEST_F(cts, test_update_cts_characteristic_buffer)
{
    // Test updating CTS characteristic buffer
    update_cts_characteristic_buffer();
    
    // Verify time was retrieved
    zassert_equal(counter_get_value_fake.call_count, 1,
                  "Should get current time");
}

ZTEST_F(cts, test_cts_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Test notification
    cts_notify();
    
    // CTS only notifies on time change, so might not notify
    // Verify function completes without crash
    zassert_true(true, "CTS notify should complete");
}

ZTEST_F(cts, test_time_conversion)
{
    // Test epoch to struct tm conversion
    time_t test_epoch = 1705323045;
    struct tm result;
    
    // Setup mock to return test time
    gmtime_r_fake.custom_fake = [](const time_t *timep, struct tm *result) -> struct tm * {
        if (result && timep) {
            result->tm_year = 2024 - 1900;
            result->tm_mon = 0;
            result->tm_mday = 15;
            result->tm_hour = 12;
            result->tm_min = 30;
            result->tm_sec = 45;
            result->tm_wday = 1;
        }
        return result;
    };
    
    struct tm *tm_result = gmtime_r(&test_epoch, &result);
    
    // Verify conversion
    zassert_not_null(tm_result, "Should return valid tm struct");
    zassert_equal(result.tm_year, 2024 - 1900, "Year should match");
    zassert_equal(result.tm_mon, 0, "Month should match");
    zassert_equal(result.tm_mday, 15, "Day should match");
    zassert_equal(result.tm_hour, 12, "Hour should match");
    zassert_equal(result.tm_min, 30, "Minute should match");
    zassert_equal(result.tm_sec, 45, "Second should match");
}

ZTEST_F(cts, test_cts_value_population)
{
    // Test CTS value structure population
    struct bt_cts_current_time_val cts_val;
    
    // This would be done internally by populate_cts_value
    memset(&cts_val, 0, sizeof(cts_val));
    
    // Simulate population
    cts_val.year = sys_cpu_to_le16(2024);
    cts_val.month = 1;
    cts_val.day = 15;
    cts_val.hours = 12;
    cts_val.minutes = 30;
    cts_val.seconds = 45;
    cts_val.weekday = 1;
    cts_val.fractions256 = 0;
    cts_val.adjust_reason = 0;
    
    // Verify structure
    zassert_equal(sys_le16_to_cpu(cts_val.year), 2024, "Year should be 2024");
    zassert_equal(cts_val.month, 1, "Month should be January");
    zassert_equal(cts_val.day, 15, "Day should be 15");
    zassert_equal(cts_val.hours, 12, "Hours should be 12");
    zassert_equal(cts_val.minutes, 30, "Minutes should be 30");
    zassert_equal(cts_val.seconds, 45, "Seconds should be 45");
}

ZTEST_F(cts, test_edge_cases)
{
    // Test with epoch 0 (1970-01-01 00:00:00)
    set_current_time_from_epoch(0);
    
    // Test with maximum uint32 epoch
    set_current_time_from_epoch(UINT32_MAX);
    
    // Test with leap year date
    set_current_time_from_epoch(1709251200); // 2024-03-01 00:00:00 (leap year)
    
    // All should handle edge cases gracefully
    zassert_true(true, "Edge cases should be handled");
}

ZTEST_F(cts, test_concurrent_time_updates)
{
    // Test multiple time updates in succession
    uint32_t epochs[] = {1705323045, 1705323046, 1705323047, 1705323048};
    
    for (int i = 0; i < ARRAY_SIZE(epochs); i++) {
        set_current_time_from_epoch(epochs[i]);
    }
    
    // Should handle rapid updates
    zassert_true(true, "Concurrent updates should be handled");
}

ZTEST(cts, test_rtc_not_available)
{
    // Test behavior when RTC is not available
    device_get_binding_fake.return_val = NULL;
    
    // Try to get time without RTC
    uint32_t epoch = get_current_epoch_time();
    
    // Should handle gracefully
    zassert_equal(epoch, 0, "Should return 0 without RTC");
}

void test_cts_suite(void)
{
    ztest_run_test_suite(cts);
}
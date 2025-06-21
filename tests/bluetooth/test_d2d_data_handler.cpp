/**
 * @file test_d2d_data_handler.cpp
 * @brief Unit tests for d2d_data_handler module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/fff.h>

#include "d2d_data_handler.hpp"
#include "app.hpp"
#include "ble_services.hpp"

DEFINE_FFF_GLOBALS;

// Mock functions for BLE services
FAKE_VOID_FUNC(jis_foot_sensor_notify, const foot_samples_t *);
FAKE_VOID_FUNC(jis_bhi360_data1_notify, const bhi360_3d_mapping_t *);
FAKE_VOID_FUNC(jis_bhi360_data2_notify, const bhi360_step_count_t *);
FAKE_VOID_FUNC(jis_bhi360_data3_notify, const bhi360_linear_accel_t *);
FAKE_VOID_FUNC(set_device_status, uint32_t);
FAKE_VOID_FUNC(jis_charge_status_notify, uint8_t);
FAKE_VOID_FUNC(jis_foot_sensor_log_available_notify, uint8_t);
FAKE_VOID_FUNC(jis_bhi360_log_available_notify, uint8_t);

// Test fixture
struct d2d_data_handler_fixture {
    foot_samples_t test_foot_samples;
    bhi360_3d_mapping_t test_3d_mapping;
    bhi360_step_count_t test_step_count;
    bhi360_linear_accel_t test_linear_accel;
};

static void *d2d_data_handler_setup(void)
{
    static struct d2d_data_handler_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    
    // Reset all fakes
    RESET_FAKE(jis_foot_sensor_notify);
    RESET_FAKE(jis_bhi360_data1_notify);
    RESET_FAKE(jis_bhi360_data2_notify);
    RESET_FAKE(jis_bhi360_data3_notify);
    RESET_FAKE(set_device_status);
    RESET_FAKE(jis_charge_status_notify);
    RESET_FAKE(jis_foot_sensor_log_available_notify);
    RESET_FAKE(jis_bhi360_log_available_notify);
    
    // Initialize test data
    for (int i = 0; i < NUM_FOOT_SENSOR_CHANNELS; i++) {
        fixture.test_foot_samples.values[i] = i * 100;
    }
    
    fixture.test_3d_mapping.accel_x = 1.0f;
    fixture.test_3d_mapping.accel_y = 2.0f;
    fixture.test_3d_mapping.accel_z = 3.0f;
    fixture.test_3d_mapping.gyro_x = 4.0f;
    fixture.test_3d_mapping.gyro_y = 5.0f;
    fixture.test_3d_mapping.gyro_z = 6.0f;
    fixture.test_3d_mapping.quat_w = 0.7071f;
    
    fixture.test_step_count.step_count = 1000;
    fixture.test_step_count.activity_duration_s = 3600;
    
    fixture.test_linear_accel.x = 0.1f;
    fixture.test_linear_accel.y = 0.2f;
    fixture.test_linear_accel.z = 0.3f;
    
    return &fixture;
}

static void d2d_data_handler_teardown(void *f)
{
    ARG_UNUSED(f);
}

ZTEST_SUITE(d2d_data_handler, NULL, d2d_data_handler_setup, NULL, NULL, d2d_data_handler_teardown);

ZTEST(d2d_data_handler, test_init)
{
    int ret = d2d_data_handler_init();
    zassert_equal(ret, 0, "Init should succeed");
}

ZTEST_F(d2d_data_handler, test_process_foot_samples)
{
    // Test valid foot samples
    int ret = d2d_data_handler_process_foot_samples(&fixture->test_foot_samples);
    zassert_equal(ret, 0, "Should process foot samples successfully");
    
    // Test null pointer
    ret = d2d_data_handler_process_foot_samples(NULL);
    zassert_equal(ret, -EINVAL, "Should return error for null pointer");
}

ZTEST_F(d2d_data_handler, test_process_bhi360_3d_mapping)
{
    // Test valid 3D mapping data
    int ret = d2d_data_handler_process_bhi360_3d_mapping(&fixture->test_3d_mapping);
    zassert_equal(ret, 0, "Should process 3D mapping successfully");
    
    // Test null pointer
    ret = d2d_data_handler_process_bhi360_3d_mapping(NULL);
    zassert_equal(ret, -EINVAL, "Should return error for null pointer");
}

ZTEST_F(d2d_data_handler, test_process_bhi360_step_count)
{
    // Test valid step count data
    int ret = d2d_data_handler_process_bhi360_step_count(&fixture->test_step_count);
    zassert_equal(ret, 0, "Should process step count successfully");
    
    // Test null pointer
    ret = d2d_data_handler_process_bhi360_step_count(NULL);
    zassert_equal(ret, -EINVAL, "Should return error for null pointer");
}

ZTEST_F(d2d_data_handler, test_process_bhi360_linear_accel)
{
    // Test valid linear acceleration data
    int ret = d2d_data_handler_process_bhi360_linear_accel(&fixture->test_linear_accel);
    zassert_equal(ret, 0, "Should process linear accel successfully");
    
    // Test null pointer
    ret = d2d_data_handler_process_bhi360_linear_accel(NULL);
    zassert_equal(ret, -EINVAL, "Should return error for null pointer");
}

ZTEST_F(d2d_data_handler, test_process_file_path)
{
    // Test valid file path
    const char *test_path = "/lfs1/hardware/foot_001.pb";
    int ret = d2d_data_handler_process_file_path(1, 0, test_path);
    zassert_equal(ret, 0, "Should process file path successfully");
    
    // Test null pointer
    ret = d2d_data_handler_process_file_path(1, 0, NULL);
    zassert_equal(ret, -EINVAL, "Should return error for null pointer");
}

ZTEST_F(d2d_data_handler, test_process_log_available)
{
    // Test foot sensor log available
    int ret = d2d_data_handler_process_log_available(5, 0);
    zassert_equal(ret, 0, "Should process foot log available successfully");
    
    // Test BHI360 log available
    ret = d2d_data_handler_process_log_available(10, 1);
    zassert_equal(ret, 0, "Should process BHI360 log available successfully");
}

ZTEST_F(d2d_data_handler, test_process_status)
{
    // Test various status values
    uint32_t test_status = 0x00000001; // STATUS_ERROR
    int ret = d2d_data_handler_process_status(test_status);
    zassert_equal(ret, 0, "Should process status successfully");
    
    test_status = 0x00000100; // STATUS_LOGGING_ACTIVE
    ret = d2d_data_handler_process_status(test_status);
    zassert_equal(ret, 0, "Should process status successfully");
}

ZTEST_F(d2d_data_handler, test_process_charge_status)
{
    // Test various charge status values
    uint8_t charge_status = 0; // Empty
    int ret = d2d_data_handler_process_charge_status(charge_status);
    zassert_equal(ret, 0, "Should process charge status successfully");
    
    charge_status = 50; // Half
    ret = d2d_data_handler_process_charge_status(charge_status);
    zassert_equal(ret, 0, "Should process charge status successfully");
    
    charge_status = 100; // Full
    ret = d2d_data_handler_process_charge_status(charge_status);
    zassert_equal(ret, 0, "Should process charge status successfully");
}

ZTEST_F(d2d_data_handler, test_future_aggregation)
{
    // Test placeholder for future aggregation functionality
    // When implemented, this should test:
    // 1. Combining primary and secondary foot sensor data
    // 2. Merging status bits from both devices
    // 3. Aggregating step counts
    // 4. Handling simultaneous data from both feet
    
    // For now, just verify the handlers don't crash
    d2d_data_handler_process_foot_samples(&fixture->test_foot_samples);
    d2d_data_handler_process_bhi360_step_count(&fixture->test_step_count);
    
    zassert_true(true, "Future aggregation tests placeholder");
}

void test_d2d_data_handler_suite(void)
{
    ztest_run_test_suite(d2d_data_handler);
}
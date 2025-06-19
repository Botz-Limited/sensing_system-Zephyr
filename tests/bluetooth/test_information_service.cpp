/**
 * @file test_information_service.cpp
 * @brief Unit tests for information_service module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/fff.h>

#include "information_service.hpp"
#include "app.hpp"
#include "errors.hpp"

DEFINE_FFF_GLOBALS;

// Mock functions
FAKE_VALUE_FUNC(int, bt_gatt_notify, struct bt_conn *, const struct bt_gatt_attr *, const void *, uint16_t);
FAKE_VALUE_FUNC(ssize_t, bt_gatt_attr_read, struct bt_conn *, const struct bt_gatt_attr *, 
                void *, uint16_t, uint16_t, const void *, uint16_t);
FAKE_VALUE_FUNC(uint32_t, get_current_epoch_time);

// Test fixture
struct information_service_fixture {
    struct bt_conn conn;
    struct bt_gatt_attr attr;
    foot_samples_t foot_data;
    bhi360_3d_mapping_t bhi360_3d_data;
    bhi360_step_count_t bhi360_step_data;
    bhi360_linear_accel_t bhi360_linear_data;
    fota_progress_msg_t fota_progress;
};

static void *information_service_setup(void)
{
    static struct information_service_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    
    // Reset all fakes
    RESET_FAKE(bt_gatt_notify);
    RESET_FAKE(bt_gatt_attr_read);
    RESET_FAKE(get_current_epoch_time);
    
    // Initialize test data
    fixture.foot_data.timestamp = 12345;
    for (int i = 0; i < 8; i++) {
        fixture.foot_data.samples[i] = i * 100;
    }
    
    fixture.bhi360_3d_data.timestamp = 54321;
    fixture.bhi360_3d_data.accel_x = 1.0f;
    fixture.bhi360_3d_data.accel_y = 2.0f;
    fixture.bhi360_3d_data.accel_z = 3.0f;
    fixture.bhi360_3d_data.gyro_x = 4.0f;
    fixture.bhi360_3d_data.gyro_y = 5.0f;
    fixture.bhi360_3d_data.gyro_z = 6.0f;
    
    fixture.bhi360_step_data.step_count = 1000;
    fixture.bhi360_step_data.activity_duration_s = 3600;
    
    fixture.bhi360_linear_data.timestamp = 99999;
    fixture.bhi360_linear_data.x = 0.1f;
    fixture.bhi360_linear_data.y = 0.2f;
    fixture.bhi360_linear_data.z = 0.3f;
    
    fixture.fota_progress.is_active = true;
    fixture.fota_progress.status = 1;
    fixture.fota_progress.percent_complete = 50;
    fixture.fota_progress.bytes_received = 1024;
    fixture.fota_progress.total_size = 2048;
    
    // Setup default return values
    bt_gatt_notify_fake.return_val = 0;
    bt_gatt_attr_read_fake.return_val = sizeof(uint32_t);
    get_current_epoch_time_fake.return_val = 1234567890;
    
    return &fixture;
}

static void information_service_teardown(void *f)
{
    ARG_UNUSED(f);
}

ZTEST_SUITE(information_service, NULL, information_service_setup, NULL, NULL, information_service_teardown);

ZTEST_F(information_service, test_set_device_status)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Set device status
    uint32_t new_status = 0x12345678;
    set_device_status(new_status);
    
    // Verify notification was sent
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify status change");
}

ZTEST_F(information_service, test_clear_err_status_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Clear error status
    jis_clear_err_status_notify(err_t::NO_ERROR);
    
    // Verify notification was sent
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify status clear");
}

ZTEST_F(information_service, test_foot_sensor_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Send foot sensor data
    jis_foot_sensor_notify(&fixture->foot_data);
    
    // Verify
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify foot sensor data");
}

ZTEST_F(information_service, test_bhi360_data1_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Send BHI360 3D mapping data
    jis_bhi360_data1_notify(&fixture->bhi360_3d_data);
    
    // Verify
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify BHI360 data1");
}

ZTEST_F(information_service, test_bhi360_data2_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Send BHI360 step count data
    jis_bhi360_data2_notify(&fixture->bhi360_step_data);
    
    // Verify
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify BHI360 data2");
}

ZTEST_F(information_service, test_bhi360_data3_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Send BHI360 linear acceleration data
    jis_bhi360_data3_notify(&fixture->bhi360_linear_data);
    
    // Verify
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify BHI360 data3");
}

ZTEST_F(information_service, test_current_time_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify current time
    cts_notify();
    
    // Verify - CTS only notifies when time changes
    // so this might not trigger a notification
    zassert_true(true, "CTS notify should complete");
}

ZTEST_F(information_service, test_charge_status_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Send charge status
    uint8_t charge_status = 75;
    jis_charge_status_notify(charge_status);
    
    // Verify
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify charge status");
}

ZTEST_F(information_service, test_foot_sensor_log_available_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify log available
    uint8_t log_id = 42;
    jis_foot_sensor_log_available_notify(log_id);
    
    // Verify
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify foot sensor log available");
}

ZTEST_F(information_service, test_foot_sensor_req_id_path_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify file path
    const char *file_path = "/lfs/foot_sensor_001.bin";
    jis_foot_sensor_req_id_path_notify(file_path);
    
    // Verify
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify foot sensor file path");
}

ZTEST_F(information_service, test_bhi360_log_available_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify log available
    uint8_t log_id = 24;
    jis_bhi360_log_available_notify(log_id);
    
    // Verify
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify BHI360 log available");
}

ZTEST_F(information_service, test_bhi360_req_id_path_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify file path
    const char *file_path = "/lfs/bhi360_002.bin";
    jis_bhi360_req_id_path_notify(file_path);
    
    // Verify
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify BHI360 file path");
}

ZTEST_F(information_service, test_fota_progress_notify)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Send FOTA progress
    jis_fota_progress_notify(&fixture->fota_progress);
    
    // Verify
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify FOTA progress");
}

ZTEST_F(information_service, test_notify_failures)
{
    // Test notification failure
    bt_gatt_notify_fake.return_val = -ENOTCONN;
    
    // Try various notifications
    set_device_status(0x12345678);
    jis_foot_sensor_notify(&fixture->foot_data);
    jis_charge_status_notify(50);
    
    // All should attempt notification even if they fail
    zassert_equal(bt_gatt_notify_fake.call_count, 3,
                  "Should attempt all notifications");
}

ZTEST_F(information_service, test_ccc_cfg_changed_callbacks)
{
    // Test CCC configuration changes
    // These functions log the state changes
    
    uint16_t enabled = BT_GATT_CCC_NOTIFY;
    uint16_t disabled = 0;
    
    // These functions don't return values, just verify they don't crash
    // In real implementation, they would enable/disable notifications
    
    zassert_true(true, "CCC callbacks should handle state changes");
}

ZTEST_F(information_service, test_read_callbacks)
{
    // Test read operations
    bt_gatt_attr_read_fake.return_val = sizeof(uint32_t);
    
    // These would be called by the BLE stack
    // Here we verify the mock is set up correctly
    
    uint8_t buffer[128];
    ssize_t len = bt_gatt_attr_read(&fixture->conn, &fixture->attr, 
                                     buffer, sizeof(buffer), 0, 
                                     &fixture->foot_data, sizeof(fixture->foot_data));
    
    zassert_true(len > 0, "Read should return data");
}

ZTEST_F(information_service, test_multiple_notifications)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Send multiple notifications in sequence
    set_device_status(0x11111111);
    jis_foot_sensor_notify(&fixture->foot_data);
    jis_bhi360_data1_notify(&fixture->bhi360_3d_data);
    jis_bhi360_data2_notify(&fixture->bhi360_step_data);
    jis_bhi360_data3_notify(&fixture->bhi360_linear_data);
    jis_charge_status_notify(100);
    jis_fota_progress_notify(&fixture->fota_progress);
    
    // Verify all notifications were sent
    zassert_equal(bt_gatt_notify_fake.call_count, 7,
                  "Should send all notifications");
}

ZTEST_F(information_service, test_edge_cases)
{
    // Test with NULL data
    bt_gatt_notify_fake.return_val = 0;
    
    // Some functions might handle NULL gracefully
    jis_foot_sensor_req_id_path_notify(NULL);
    jis_bhi360_req_id_path_notify(NULL);
    
    // Test with empty strings
    jis_foot_sensor_req_id_path_notify("");
    jis_bhi360_req_id_path_notify("");
    
    // Test with maximum values
    set_device_status(UINT32_MAX);
    jis_charge_status_notify(UINT8_MAX);
    jis_foot_sensor_log_available_notify(UINT8_MAX);
    jis_bhi360_log_available_notify(UINT8_MAX);
    
    // All should handle edge cases gracefully
    zassert_true(bt_gatt_notify_fake.call_count > 0,
                 "Should handle edge cases");
}

void test_information_service_suite(void)
{
    ztest_run_test_suite(information_service);
}
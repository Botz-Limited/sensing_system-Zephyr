/**
 * @file test_ble_d2d_tx.cpp
 * @brief Unit tests for ble_d2d_tx module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/fff.h>

#include "ble_d2d_tx.hpp"
#include "app.hpp"

DEFINE_FFF_GLOBALS;

// Mock functions
FAKE_VALUE_FUNC(int, bt_gatt_write_without_response, struct bt_conn *, uint16_t, const void *, uint16_t, bool);
FAKE_VALUE_FUNC(int, bt_gatt_discover, struct bt_conn *, struct bt_gatt_discover_params *);
FAKE_VALUE_FUNC(int, bt_gatt_subscribe, struct bt_conn *, struct bt_gatt_subscribe_params *);

// Test fixture
struct ble_d2d_tx_fixture {
    struct bt_conn conn;
    foot_samples_t foot_data;
    bhi360_3d_mapping_t bhi360_3d_data;
    bhi360_step_count_t bhi360_step_data;
    bhi360_linear_accel_t bhi360_linear_data;
    fota_progress_msg_t fota_progress;
};

static void *ble_d2d_tx_setup(void)
{
    static struct ble_d2d_tx_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    
    // Reset all fakes
    RESET_FAKE(bt_gatt_write_without_response);
    RESET_FAKE(bt_gatt_discover);
    RESET_FAKE(bt_gatt_subscribe);
    
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
    
    return &fixture;
}

static void ble_d2d_tx_teardown(void *f)
{
    ARG_UNUSED(f);
    // Reset connection
    ble_d2d_tx_set_connection(NULL);
}

ZTEST_SUITE(ble_d2d_tx, NULL, ble_d2d_tx_setup, NULL, NULL, ble_d2d_tx_teardown);

ZTEST(ble_d2d_tx, test_init)
{
    // Test initialization
    ble_d2d_tx_init();
    // Function just logs, verify it doesn't crash
}

ZTEST_F(ble_d2d_tx, test_set_connection)
{
    // Test setting connection
    ble_d2d_tx_set_connection(&fixture->conn);
    
    // Test clearing connection
    ble_d2d_tx_set_connection(NULL);
}

ZTEST_F(ble_d2d_tx, test_send_foot_sensor_data_no_conn)
{
    // Ensure no connection
    ble_d2d_tx_set_connection(NULL);
    
    // Try to send data
    int ret = ble_d2d_tx_send_foot_sensor_data(&fixture->foot_data);
    
    // Verify
    zassert_equal(ret, -ENOTCONN, "Should return -ENOTCONN when no connection");
    zassert_equal(bt_gatt_write_without_response_fake.call_count, 0,
                  "Should not attempt to write without connection");
}

ZTEST_F(ble_d2d_tx, test_send_foot_sensor_data_success)
{
    // Setup connection
    ble_d2d_tx_set_connection(&fixture->conn);
    bt_gatt_write_without_response_fake.return_val = 0;
    
    // Send data
    int ret = ble_d2d_tx_send_foot_sensor_data(&fixture->foot_data);
    
    // Verify
    zassert_equal(ret, 0, "Should return success");
    zassert_equal(bt_gatt_write_without_response_fake.call_count, 1,
                  "Should call write once");
    zassert_equal(bt_gatt_write_without_response_fake.arg0_val, &fixture->conn,
                  "Should use correct connection");
}

ZTEST_F(ble_d2d_tx, test_send_foot_sensor_data_write_fail)
{
    // Setup connection
    ble_d2d_tx_set_connection(&fixture->conn);
    bt_gatt_write_without_response_fake.return_val = -EIO;
    
    // Send data
    int ret = ble_d2d_tx_send_foot_sensor_data(&fixture->foot_data);
    
    // Verify
    zassert_equal(ret, -EIO, "Should return write error");
}

ZTEST_F(ble_d2d_tx, test_send_fota_complete_no_conn)
{
    // Ensure no connection
    ble_d2d_tx_set_connection(NULL);
    
    // Try to send
    int ret = ble_d2d_tx_send_fota_complete();
    
    // Verify
    zassert_equal(ret, -ENOTCONN, "Should return -ENOTCONN");
}

ZTEST_F(ble_d2d_tx, test_send_fota_complete_success)
{
    // Setup connection
    ble_d2d_tx_set_connection(&fixture->conn);
    bt_gatt_write_without_response_fake.return_val = 0;
    
    // Send
    int ret = ble_d2d_tx_send_fota_complete();
    
    // Verify
    zassert_equal(ret, 0, "Should return success");
    zassert_equal(bt_gatt_write_without_response_fake.call_count, 1,
                  "Should write once");
}

ZTEST_F(ble_d2d_tx, test_send_log_available_functions)
{
    // Setup connection
    ble_d2d_tx_set_connection(&fixture->conn);
    bt_gatt_write_without_response_fake.return_val = 0;
    
    // Test foot sensor log available
    int ret = ble_d2d_tx_send_foot_sensor_log_available(42);
    zassert_equal(ret, 0, "Foot sensor log available should succeed");
    
    // Test BHI360 log available
    ret = ble_d2d_tx_send_bhi360_log_available(24);
    zassert_equal(ret, 0, "BHI360 log available should succeed");
    
    zassert_equal(bt_gatt_write_without_response_fake.call_count, 2,
                  "Should have written twice");
}

ZTEST_F(ble_d2d_tx, test_send_req_id_path_functions)
{
    // Setup connection
    ble_d2d_tx_set_connection(&fixture->conn);
    bt_gatt_write_without_response_fake.return_val = 0;
    
    const char *test_path = "/lfs/test_file.bin";
    
    // Test foot sensor req id path
    int ret = ble_d2d_tx_send_foot_sensor_req_id_path(test_path);
    zassert_equal(ret, 0, "Foot sensor req id path should succeed");
    
    // Test BHI360 req id path
    ret = ble_d2d_tx_send_bhi360_req_id_path(test_path);
    zassert_equal(ret, 0, "BHI360 req id path should succeed");
}

ZTEST_F(ble_d2d_tx, test_send_bhi360_data_functions)
{
    // Setup connection
    ble_d2d_tx_set_connection(&fixture->conn);
    bt_gatt_write_without_response_fake.return_val = 0;
    
    // Test data1 (3D mapping)
    int ret = ble_d2d_tx_send_bhi360_data1(&fixture->bhi360_3d_data);
    zassert_equal(ret, 0, "BHI360 data1 should succeed");
    
    // Test data2 (step count)
    ret = ble_d2d_tx_send_bhi360_data2(&fixture->bhi360_step_data);
    zassert_equal(ret, 0, "BHI360 data2 should succeed");
    
    // Test data3 (linear accel)
    ret = ble_d2d_tx_send_bhi360_data3(&fixture->bhi360_linear_data);
    zassert_equal(ret, 0, "BHI360 data3 should succeed");
    
    zassert_equal(bt_gatt_write_without_response_fake.call_count, 3,
                  "Should have written three times");
}

ZTEST_F(ble_d2d_tx, test_send_status_functions)
{
    // Setup connection
    ble_d2d_tx_set_connection(&fixture->conn);
    bt_gatt_write_without_response_fake.return_val = 0;
    
    // Test device status
    int ret = ble_d2d_tx_send_status(0x12345678);
    zassert_equal(ret, 0, "Send status should succeed");
    
    // Test charge status
    ret = ble_d2d_tx_send_charge_status(75);
    zassert_equal(ret, 0, "Send charge status should succeed");
}

ZTEST_F(ble_d2d_tx, test_send_control_commands)
{
    // Setup connection
    ble_d2d_tx_set_connection(&fixture->conn);
    bt_gatt_write_without_response_fake.return_val = 0;
    
    // Test set time command
    int ret = ble_d2d_tx_send_set_time_command(1234567890);
    zassert_equal(ret, 0, "Set time command should succeed");
    
    // Test delete foot log command
    ret = ble_d2d_tx_send_delete_foot_log_command(5);
    zassert_equal(ret, 0, "Delete foot log command should succeed");
    
    // Test delete BHI360 log command
    ret = ble_d2d_tx_send_delete_bhi360_log_command(10);
    zassert_equal(ret, 0, "Delete BHI360 log command should succeed");
    
    // Test start activity command
    ret = ble_d2d_tx_send_start_activity_command(1);
    zassert_equal(ret, 0, "Start activity command should succeed");
    
    // Test stop activity command
    ret = ble_d2d_tx_send_stop_activity_command(0);
    zassert_equal(ret, 0, "Stop activity command should succeed");
    
    zassert_equal(bt_gatt_write_without_response_fake.call_count, 5,
                  "Should have written five times");
}

ZTEST_F(ble_d2d_tx, test_send_commands_no_connection)
{
    // Ensure no connection
    ble_d2d_tx_set_connection(NULL);
    
    // Test all command functions return -ENOTCONN
    zassert_equal(ble_d2d_tx_send_set_time_command(0), -ENOTCONN,
                  "Set time should return -ENOTCONN");
    zassert_equal(ble_d2d_tx_send_delete_foot_log_command(0), -ENOTCONN,
                  "Delete foot log should return -ENOTCONN");
    zassert_equal(ble_d2d_tx_send_delete_bhi360_log_command(0), -ENOTCONN,
                  "Delete BHI360 log should return -ENOTCONN");
    zassert_equal(ble_d2d_tx_send_start_activity_command(0), -ENOTCONN,
                  "Start activity should return -ENOTCONN");
    zassert_equal(ble_d2d_tx_send_stop_activity_command(0), -ENOTCONN,
                  "Stop activity should return -ENOTCONN");
}

void test_ble_d2d_tx_suite(void)
{
    ztest_run_test_suite(ble_d2d_tx);
}
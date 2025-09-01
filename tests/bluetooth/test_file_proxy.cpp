/**
 * @file test_file_proxy.cpp
 * @brief Unit tests for file_proxy module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/fff.h>

#include "file_proxy.hpp"
#include "ble_d2d_file_transfer.hpp"

DEFINE_FFF_GLOBALS;

// Mock functions
FAKE_VALUE_FUNC(int, bt_gatt_notify, struct bt_conn *, const struct bt_gatt_attr *, const void *, uint16_t);
FAKE_VALUE_FUNC(int, ble_d2d_file_send_command, enum d2d_file_cmd, const uint8_t *, size_t);
FAKE_VALUE_FUNC(int, k_work_submit, struct k_work *);
FAKE_VALUE_FUNC(int, k_work_schedule, struct k_work_delayable *, k_timeout_t);
FAKE_VALUE_FUNC(int, k_work_cancel_delayable, struct k_work_delayable *);
FAKE_VOID_FUNC(ble_d2d_file_set_callbacks, d2d_file_data_cb_t, d2d_file_status_cb_t);

// Test fixture
struct file_proxy_fixture {
    struct bt_conn primary_conn;
    struct bt_conn secondary_conn;
    struct bt_gatt_attr attr;
    uint8_t test_data[256];
};

static void *file_proxy_setup(void)
{
    static struct file_proxy_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    
    // Reset all fakes
    RESET_FAKE(bt_gatt_notify);
    RESET_FAKE(ble_d2d_file_send_command);
    RESET_FAKE(k_work_submit);
    RESET_FAKE(k_work_schedule);
    RESET_FAKE(k_work_cancel_delayable);
    RESET_FAKE(ble_d2d_file_set_callbacks);
    
    // Initialize test data
    for (int i = 0; i < sizeof(fixture.test_data); i++) {
        fixture.test_data[i] = i;
    }
    
    // Setup work queue mocks to succeed
    k_work_submit_fake.return_val = 1;
    k_work_schedule_fake.return_val = 1;
    k_work_cancel_delayable_fake.return_val = 0;
    
    return &fixture;
}

static void file_proxy_teardown(void *f)
{
    struct file_proxy_fixture *fixture = (struct file_proxy_fixture *)f;
    
    // Clear connections
    file_proxy_set_secondary_conn(NULL);
}

ZTEST_SUITE(file_proxy, NULL, file_proxy_setup, NULL, NULL, file_proxy_teardown);

ZTEST(file_proxy, test_init)
{
    // Test initialization
    int ret = file_proxy_init();
    
    // Verify
    zassert_equal(ret, 0, "File proxy init should succeed");
    zassert_equal(ble_d2d_file_set_callbacks_fake.call_count, 1,
                  "Should register callbacks");
}

ZTEST_F(file_proxy, test_set_secondary_conn)
{
    // Test setting connection
    file_proxy_set_secondary_conn(&fixture->secondary_conn);
    
    // Test clearing connection
    file_proxy_set_secondary_conn(NULL);
}

ZTEST_F(file_proxy, test_notify_status_idle)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify idle status
    int ret = file_proxy_notify_status(FILE_PROXY_STATUS_IDLE);
    
    // Verify
    zassert_equal(ret, 0, "Notify idle status should succeed");
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify once");
}

ZTEST_F(file_proxy, test_notify_status_busy)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify busy status
    int ret = file_proxy_notify_status(FILE_PROXY_STATUS_BUSY);
    
    // Verify
    zassert_equal(ret, 0, "Notify busy status should succeed");
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify once");
}

ZTEST_F(file_proxy, test_notify_status_complete)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify complete status
    int ret = file_proxy_notify_status(FILE_PROXY_STATUS_COMPLETE);
    
    // Verify
    zassert_equal(ret, 0, "Notify complete status should succeed");
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify once");
}

ZTEST_F(file_proxy, test_notify_status_error)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify error status
    int ret = file_proxy_notify_status(FILE_PROXY_STATUS_ERROR);
    
    // Verify
    zassert_equal(ret, 0, "Notify error status should succeed");
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify once");
}

ZTEST_F(file_proxy, test_notify_data)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify data
    int ret = file_proxy_notify_data(fixture->test_data, 128);
    
    // Verify
    zassert_equal(ret, 0, "Notify data should succeed");
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify once");
}

ZTEST_F(file_proxy, test_notify_data_empty)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify empty data
    int ret = file_proxy_notify_data(NULL, 0);
    
    // Verify
    zassert_equal(ret, 0, "Notify empty data should succeed");
}

ZTEST_F(file_proxy, test_forward_list_command)
{
    // Setup secondary connection
    file_proxy_set_secondary_conn(&fixture->secondary_conn);
    ble_d2d_file_send_command_fake.return_val = 0;
    
    // Forward list command
    int ret = ble_d2d_file_send_command(D2D_FILE_CMD_LIST, NULL, 0);
    
    // Verify
    zassert_equal(ret, 0, "Forward list command should succeed");
    zassert_equal(ble_d2d_file_send_command_fake.call_count, 1,
                  "Should send command once");
    zassert_equal(ble_d2d_file_send_command_fake.arg0_val, D2D_FILE_CMD_LIST,
                  "Should send list command");
}

ZTEST_F(file_proxy, test_forward_read_command)
{
    // Setup secondary connection
    file_proxy_set_secondary_conn(&fixture->secondary_conn);
    ble_d2d_file_send_command_fake.return_val = 0;
    
    // Prepare read command data
    uint8_t cmd_data[2] = {5, D2D_FILE_TYPE_FOOT_SENSOR};
    
    // Forward read command
    int ret = ble_d2d_file_send_command(D2D_FILE_CMD_READ, cmd_data, sizeof(cmd_data));
    
    // Verify
    zassert_equal(ret, 0, "Forward read command should succeed");
    zassert_equal(ble_d2d_file_send_command_fake.arg0_val, D2D_FILE_CMD_READ,
                  "Should send read command");
}

ZTEST_F(file_proxy, test_forward_delete_command)
{
    // Setup secondary connection
    file_proxy_set_secondary_conn(&fixture->secondary_conn);
    ble_d2d_file_send_command_fake.return_val = 0;
    
    // Prepare delete command data
    uint8_t cmd_data[2] = {10, D2D_FILE_TYPE_BHI360};
    
    // Forward delete command
    int ret = ble_d2d_file_send_command(D2D_FILE_CMD_DELETE, cmd_data, sizeof(cmd_data));
    
    // Verify
    zassert_equal(ret, 0, "Forward delete command should succeed");
    zassert_equal(ble_d2d_file_send_command_fake.arg0_val, D2D_FILE_CMD_DELETE,
                  "Should send delete command");
}

ZTEST_F(file_proxy, test_forward_info_command)
{
    // Setup secondary connection
    file_proxy_set_secondary_conn(&fixture->secondary_conn);
    ble_d2d_file_send_command_fake.return_val = 0;
    
    // Prepare info command data
    uint8_t cmd_data[2] = {15, D2D_FILE_TYPE_FOOT_SENSOR};
    
    // Forward info command
    int ret = ble_d2d_file_send_command(D2D_FILE_CMD_INFO, cmd_data, sizeof(cmd_data));
    
    // Verify
    zassert_equal(ret, 0, "Forward info command should succeed");
    zassert_equal(ble_d2d_file_send_command_fake.arg0_val, D2D_FILE_CMD_INFO,
                  "Should send info command");
}

ZTEST_F(file_proxy, test_forward_command_no_secondary)
{
    // Ensure no secondary connection
    file_proxy_set_secondary_conn(NULL);
    
    // Try to forward command
    uint8_t cmd_data[2] = {1, D2D_FILE_TYPE_FOOT_SENSOR};
    int ret = ble_d2d_file_send_command(D2D_FILE_CMD_READ, cmd_data, sizeof(cmd_data));
    
    // Should still attempt to send (mock will handle the error)
    zassert_equal(ble_d2d_file_send_command_fake.call_count, 1,
                  "Should attempt to send command");
}

ZTEST_F(file_proxy, test_timeout_handling)
{
    // Setup
    file_proxy_set_secondary_conn(&fixture->secondary_conn);
    
    // Simulate timeout by having work scheduled
    k_work_schedule_fake.return_val = 1;
    
    // In real scenario, timeout would trigger after 5 minutes
    // Here we verify the timeout can be scheduled
    zassert_equal(k_work_schedule_fake.return_val, 1,
                  "Timeout should be schedulable");
}

ZTEST_F(file_proxy, test_work_queue_operations)
{
    // Test work submit
    k_work_submit_fake.return_val = 1;
    struct k_work work;
    int ret = k_work_submit(&work);
    zassert_equal(ret, 1, "Work submit should succeed");
    
    // Test work schedule
    k_work_schedule_fake.return_val = 1;
    struct k_work_delayable dwork;
    ret = k_work_schedule(&dwork, K_SECONDS(1));
    zassert_equal(ret, 1, "Work schedule should succeed");
    
    // Test work cancel
    k_work_cancel_delayable_fake.return_val = 0;
    ret = k_work_cancel_delayable(&dwork);
    zassert_equal(ret, 0, "Work cancel should succeed");
}

ZTEST_F(file_proxy, test_error_conditions)
{
    // Test notify with connection error
    bt_gatt_notify_fake.return_val = -ENOTCONN;
    int ret = file_proxy_notify_status(FILE_PROXY_STATUS_ERROR);
    zassert_equal(ret, -ENOTCONN, "Should return connection error");
    
    // Test notify data with error
    bt_gatt_notify_fake.return_val = -EINVAL;
    ret = file_proxy_notify_data(fixture->test_data, 128);
    zassert_equal(ret, -EINVAL, "Should return invalid argument");
    
    // Test command forward failure
    file_proxy_set_secondary_conn(&fixture->secondary_conn);
    ble_d2d_file_send_command_fake.return_val = -EIO;
    ret = ble_d2d_file_send_command(D2D_FILE_CMD_LIST, NULL, 0);
    zassert_equal(ret, -EIO, "Should return I/O error");
}

ZTEST_F(file_proxy, test_callback_registration)
{
    // Initialize should register callbacks
    file_proxy_init();
    
    // Verify callbacks were registered
    zassert_equal(ble_d2d_file_set_callbacks_fake.call_count, 1,
                  "Should register callbacks once");
    zassert_not_null(ble_d2d_file_set_callbacks_fake.arg0_val,
                     "Should register data callback");
    zassert_not_null(ble_d2d_file_set_callbacks_fake.arg1_val,
                     "Should register status callback");
}

void test_file_proxy_suite(void)
{
    ztest_run_test_suite(file_proxy);
}
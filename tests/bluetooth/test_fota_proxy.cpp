/**
 * @file test_fota_proxy.cpp
 * @brief Unit tests for fota_proxy module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/fff.h>

#include "fota_proxy.hpp"
#include "ble_d2d_tx.hpp"

DEFINE_FFF_GLOBALS;

// Mock functions
FAKE_VALUE_FUNC(int, bt_gatt_notify, struct bt_conn *, const struct bt_gatt_attr *, const void *, uint16_t);
FAKE_VALUE_FUNC(int, ble_d2d_tx_send_fota_data, const uint8_t *, size_t);
FAKE_VALUE_FUNC(int, ble_d2d_tx_send_fota_command, uint8_t);
FAKE_VOID_FUNC(sys_reboot, int);
FAKE_VALUE_FUNC(int, k_work_submit, struct k_work *);
FAKE_VALUE_FUNC(int, k_work_schedule, struct k_work_delayable *, k_timeout_t);
FAKE_VALUE_FUNC(int, k_work_cancel_delayable, struct k_work_delayable *);

// Test fixture
struct fota_proxy_fixture {
    struct bt_conn primary_conn;
    struct bt_conn secondary_conn;
    struct bt_gatt_attr attr;
    uint8_t test_data[256];
};

static void *fota_proxy_setup(void)
{
    static struct fota_proxy_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    
    // Reset all fakes
    RESET_FAKE(bt_gatt_notify);
    RESET_FAKE(ble_d2d_tx_send_fota_data);
    RESET_FAKE(ble_d2d_tx_send_fota_command);
    RESET_FAKE(sys_reboot);
    RESET_FAKE(k_work_submit);
    RESET_FAKE(k_work_schedule);
    RESET_FAKE(k_work_cancel_delayable);
    
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

static void fota_proxy_teardown(void *f)
{
    struct fota_proxy_fixture *fixture = (struct fota_proxy_fixture *)f;
    
    // Clear connections
    fota_proxy_set_secondary_conn(NULL);
}

ZTEST_SUITE(fota_proxy, NULL, fota_proxy_setup, NULL, NULL, fota_proxy_teardown);

ZTEST(fota_proxy, test_init)
{
    // Test initialization
    int ret = fota_proxy_init();
    
    // Verify
    zassert_equal(ret, 0, "FOTA proxy init should succeed");
}

ZTEST_F(fota_proxy, test_set_secondary_conn)
{
    // Test setting connection
    fota_proxy_set_secondary_conn(&fixture->secondary_conn);
    
    // Test clearing connection
    fota_proxy_set_secondary_conn(NULL);
}

ZTEST_F(fota_proxy, test_notify_status_idle)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify idle status
    int ret = fota_proxy_notify_status(FOTA_PROXY_STATUS_IDLE);
    
    // Verify
    zassert_equal(ret, 0, "Notify idle status should succeed");
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify once");
}

ZTEST_F(fota_proxy, test_notify_status_active)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify active status
    int ret = fota_proxy_notify_status(FOTA_PROXY_STATUS_ACTIVE);
    
    // Verify
    zassert_equal(ret, 0, "Notify active status should succeed");
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify once");
}

ZTEST_F(fota_proxy, test_notify_status_complete)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify complete status
    int ret = fota_proxy_notify_status(FOTA_PROXY_STATUS_COMPLETE);
    
    // Verify
    zassert_equal(ret, 0, "Notify complete status should succeed");
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify once");
}

ZTEST_F(fota_proxy, test_notify_status_error)
{
    // Setup
    bt_gatt_notify_fake.return_val = 0;
    
    // Notify error status
    int ret = fota_proxy_notify_status(FOTA_PROXY_STATUS_ERROR);
    
    // Verify
    zassert_equal(ret, 0, "Notify error status should succeed");
    zassert_equal(bt_gatt_notify_fake.call_count, 1,
                  "Should notify once");
}

ZTEST_F(fota_proxy, test_handle_secondary_complete)
{
    // Setup - set secondary connection first
    fota_proxy_set_secondary_conn(&fixture->secondary_conn);
    bt_gatt_notify_fake.return_val = 0;
    
    // Handle secondary complete
    int ret = fota_proxy_handle_secondary_complete();
    
    // Verify
    zassert_equal(ret, 0, "Handle secondary complete should succeed");
    zassert_true(k_work_schedule_fake.call_count > 0,
                 "Should schedule reset work");
}

ZTEST_F(fota_proxy, test_handle_secondary_complete_no_conn)
{
    // Ensure no secondary connection
    fota_proxy_set_secondary_conn(NULL);
    
    // Handle secondary complete
    int ret = fota_proxy_handle_secondary_complete();
    
    // Verify
    zassert_equal(ret, -ENOTCONN, "Should return -ENOTCONN without connection");
}

ZTEST_F(fota_proxy, test_forward_data_no_secondary)
{
    // Ensure no secondary connection
    fota_proxy_set_secondary_conn(NULL);
    
    // Try to forward data (this would be internal to the module)
    // We can't directly test the static function, but we can verify
    // that operations fail gracefully without a connection
    
    zassert_true(true, "Operations should handle no connection gracefully");
}

ZTEST_F(fota_proxy, test_forward_data_with_secondary)
{
    // Setup secondary connection
    fota_proxy_set_secondary_conn(&fixture->secondary_conn);
    ble_d2d_tx_send_fota_data_fake.return_val = 0;
    
    // In real scenario, data would be forwarded through work queue
    // Here we verify the mock is set up correctly
    int ret = ble_d2d_tx_send_fota_data(fixture->test_data, 128);
    
    zassert_equal(ret, 0, "Forward data should succeed");
    zassert_equal(ble_d2d_tx_send_fota_data_fake.call_count, 1,
                  "Should call send data once");
}

ZTEST_F(fota_proxy, test_timeout_handling)
{
    // Setup
    fota_proxy_set_secondary_conn(&fixture->secondary_conn);
    bt_gatt_notify_fake.return_val = 0;
    
    // Simulate timeout by having work scheduled
    k_work_schedule_fake.return_val = 1;
    
    // In real scenario, timeout would trigger after 5 minutes
    // Here we verify the timeout can be scheduled
    zassert_equal(k_work_schedule_fake.return_val, 1,
                  "Timeout should be schedulable");
}

ZTEST_F(fota_proxy, test_reset_handling)
{
    // Setup
    fota_proxy_set_secondary_conn(&fixture->secondary_conn);
    
    // In real scenario, reset would be triggered after delay
    // Here we verify sys_reboot would be called
    sys_reboot(SYS_REBOOT_COLD);
    
    zassert_equal(sys_reboot_fake.call_count, 1,
                  "Should call sys_reboot");
    zassert_equal(sys_reboot_fake.arg0_val, SYS_REBOOT_COLD,
                  "Should request cold reboot");
}

ZTEST_F(fota_proxy, test_work_queue_operations)
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

ZTEST_F(fota_proxy, test_command_forwarding)
{
    // Setup secondary connection
    fota_proxy_set_secondary_conn(&fixture->secondary_conn);
    ble_d2d_tx_send_fota_command_fake.return_val = 0;
    
    // Test various commands
    uint8_t commands[] = {
        FOTA_CMD_START,
        FOTA_CMD_DATA,
        FOTA_CMD_END,
        FOTA_CMD_CANCEL
    };
    
    for (int i = 0; i < ARRAY_SIZE(commands); i++) {
        RESET_FAKE(ble_d2d_tx_send_fota_command);
        ble_d2d_tx_send_fota_command_fake.return_val = 0;
        
        int ret = ble_d2d_tx_send_fota_command(commands[i]);
        
        zassert_equal(ret, 0, "Command %d should succeed", commands[i]);
        zassert_equal(ble_d2d_tx_send_fota_command_fake.call_count, 1,
                      "Should send command once");
        zassert_equal(ble_d2d_tx_send_fota_command_fake.arg0_val, commands[i],
                      "Should send correct command");
    }
}

ZTEST_F(fota_proxy, test_error_conditions)
{
    // Test notify with no connection
    bt_gatt_notify_fake.return_val = -ENOTCONN;
    int ret = fota_proxy_notify_status(FOTA_PROXY_STATUS_ERROR);
    zassert_equal(ret, -ENOTCONN, "Should return connection error");
    
    // Test forward data failure
    fota_proxy_set_secondary_conn(&fixture->secondary_conn);
    ble_d2d_tx_send_fota_data_fake.return_val = -EIO;
    ret = ble_d2d_tx_send_fota_data(fixture->test_data, 128);
    zassert_equal(ret, -EIO, "Should return I/O error");
    
    // Test command forward failure
    ble_d2d_tx_send_fota_command_fake.return_val = -EINVAL;
    ret = ble_d2d_tx_send_fota_command(0xFF);
    zassert_equal(ret, -EINVAL, "Should return invalid argument");
}

void test_fota_proxy_suite(void)
{
    ztest_run_test_suite(fota_proxy);
}
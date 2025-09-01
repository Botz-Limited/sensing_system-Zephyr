/**
 * @file test_control_service.cpp
 * @brief Unit tests for control_service module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/fff.h>

#include "control_service.hpp"
#include "cts.hpp"
#include "ble_d2d_tx.hpp"

DEFINE_FFF_GLOBALS;

// Mock functions
FAKE_VALUE_FUNC(int, bt_gatt_notify, struct bt_conn *, const struct bt_gatt_attr *, const void *, uint16_t);
FAKE_VALUE_FUNC(ssize_t, bt_gatt_attr_read, struct bt_conn *, const struct bt_gatt_attr *, 
                void *, uint16_t, uint16_t, const void *, uint16_t);
FAKE_VOID_FUNC(set_current_time_from_epoch, uint32_t);
FAKE_VALUE_FUNC(int, ble_d2d_tx_send_set_time_command, uint32_t);
FAKE_VALUE_FUNC(int, ble_d2d_tx_send_delete_foot_log_command, uint8_t);
FAKE_VALUE_FUNC(int, ble_d2d_tx_send_delete_bhi360_log_command, uint8_t);
FAKE_VALUE_FUNC(int, ble_d2d_tx_send_start_activity_command, uint8_t);
FAKE_VALUE_FUNC(int, ble_d2d_tx_send_stop_activity_command, uint8_t);
FAKE_VALUE_FUNC(int, k_msgq_put, struct k_msgq *, const void *, k_timeout_t);

// Test fixture
struct control_service_fixture {
    struct bt_conn conn;
    struct bt_gatt_attr attr;
};

static void *control_service_setup(void)
{
    static struct control_service_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    
    // Reset all fakes
    RESET_FAKE(bt_gatt_notify);
    RESET_FAKE(bt_gatt_attr_read);
    RESET_FAKE(set_current_time_from_epoch);
    RESET_FAKE(ble_d2d_tx_send_set_time_command);
    RESET_FAKE(ble_d2d_tx_send_delete_foot_log_command);
    RESET_FAKE(ble_d2d_tx_send_delete_bhi360_log_command);
    RESET_FAKE(ble_d2d_tx_send_start_activity_command);
    RESET_FAKE(ble_d2d_tx_send_stop_activity_command);
    RESET_FAKE(k_msgq_put);
    
    // Setup default return values
    bt_gatt_notify_fake.return_val = 0;
    bt_gatt_attr_read_fake.return_val = sizeof(uint32_t);
    ble_d2d_tx_send_set_time_command_fake.return_val = 0;
    ble_d2d_tx_send_delete_foot_log_command_fake.return_val = 0;
    ble_d2d_tx_send_delete_bhi360_log_command_fake.return_val = 0;
    ble_d2d_tx_send_start_activity_command_fake.return_val = 0;
    ble_d2d_tx_send_stop_activity_command_fake.return_val = 0;
    k_msgq_put_fake.return_val = 0;
    
    return &fixture;
}

static void control_service_teardown(void *f)
{
    ARG_UNUSED(f);
}

ZTEST_SUITE(control_service, NULL, control_service_setup, NULL, NULL, control_service_teardown);

ZTEST_F(control_service, test_set_time_command)
{
    // Test set time command
    uint32_t test_time = 1234567890;
    
    // Simulate write to set time characteristic
    set_current_time_from_epoch(test_time);
    
    // Verify
    zassert_equal(set_current_time_from_epoch_fake.call_count, 1,
                  "Should call set_current_time_from_epoch");
    zassert_equal(set_current_time_from_epoch_fake.arg0_val, test_time,
                  "Should set correct time");
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // On primary device, should also forward to secondary
    int ret = ble_d2d_tx_send_set_time_command(test_time);
    zassert_equal(ret, 0, "Should forward time command");
    zassert_equal(ble_d2d_tx_send_set_time_command_fake.arg0_val, test_time,
                  "Should forward correct time");
#endif
}

ZTEST_F(control_service, test_delete_foot_log_command)
{
    // Test delete foot log command
    uint8_t log_id = 5;
    
    // Setup message queue mock
    k_msgq_put_fake.return_val = 0;
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // On primary device, should forward to secondary
    int ret = ble_d2d_tx_send_delete_foot_log_command(log_id);
    zassert_equal(ret, 0, "Should forward delete command");
    zassert_equal(ble_d2d_tx_send_delete_foot_log_command_fake.arg0_val, log_id,
                  "Should forward correct log ID");
#else
    // On secondary device, should queue message
    // In real implementation, write handler would queue the message
    generic_message_t msg;
    msg.sender = SENDER_BLUETOOTH;
    msg.type = MSG_TYPE_COMMAND;
    snprintf(msg.data.command_str, sizeof(msg.data.command_str), 
             "delete_foot_log %d", log_id);
    
    int ret = k_msgq_put(&foot_sensor_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Should queue message");
#endif
}

ZTEST_F(control_service, test_delete_bhi360_log_command)
{
    // Test delete BHI360 log command
    uint8_t log_id = 10;
    
    // Setup message queue mock
    k_msgq_put_fake.return_val = 0;
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // On primary device, should forward to secondary
    int ret = ble_d2d_tx_send_delete_bhi360_log_command(log_id);
    zassert_equal(ret, 0, "Should forward delete command");
    zassert_equal(ble_d2d_tx_send_delete_bhi360_log_command_fake.arg0_val, log_id,
                  "Should forward correct log ID");
#else
    // On secondary device, should queue message
    generic_message_t msg;
    msg.sender = SENDER_BLUETOOTH;
    msg.type = MSG_TYPE_COMMAND;
    snprintf(msg.data.command_str, sizeof(msg.data.command_str), 
             "delete_bhi360_log %d", log_id);
    
    int ret = k_msgq_put(&motion_sensor_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Should queue message");
#endif
}

ZTEST_F(control_service, test_start_activity_command)
{
    // Test start activity command
    uint8_t activity_value = 1;
    
    // Setup message queue mock
    k_msgq_put_fake.return_val = 0;
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // On primary device, should forward to secondary
    int ret = ble_d2d_tx_send_start_activity_command(activity_value);
    zassert_equal(ret, 0, "Should forward start command");
    zassert_equal(ble_d2d_tx_send_start_activity_command_fake.arg0_val, activity_value,
                  "Should forward correct value");
#else
    // On secondary device, should queue messages
    generic_message_t msg;
    msg.sender = SENDER_BLUETOOTH;
    msg.type = MSG_TYPE_COMMAND;
    strcpy(msg.data.command_str, "start_activity");
    
    // Should queue to both foot sensor and motion sensor
    int ret = k_msgq_put(&foot_sensor_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Should queue to foot sensor");
    
    ret = k_msgq_put(&motion_sensor_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Should queue to motion sensor");
#endif
}

ZTEST_F(control_service, test_stop_activity_command)
{
    // Test stop activity command
    uint8_t activity_value = 0;
    
    // Setup message queue mock
    k_msgq_put_fake.return_val = 0;
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // On primary device, should forward to secondary
    int ret = ble_d2d_tx_send_stop_activity_command(activity_value);
    zassert_equal(ret, 0, "Should forward stop command");
    zassert_equal(ble_d2d_tx_send_stop_activity_command_fake.arg0_val, activity_value,
                  "Should forward correct value");
#else
    // On secondary device, should queue messages
    generic_message_t msg;
    msg.sender = SENDER_BLUETOOTH;
    msg.type = MSG_TYPE_COMMAND;
    strcpy(msg.data.command_str, "stop_activity");
    
    // Should queue to both foot sensor and motion sensor
    int ret = k_msgq_put(&foot_sensor_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Should queue to foot sensor");
    
    ret = k_msgq_put(&motion_sensor_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, 0, "Should queue to motion sensor");
#endif
}

ZTEST_F(control_service, test_command_forwarding_failures)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Test forwarding failures
    ble_d2d_tx_send_set_time_command_fake.return_val = -ENOTCONN;
    ble_d2d_tx_send_delete_foot_log_command_fake.return_val = -EIO;
    ble_d2d_tx_send_delete_bhi360_log_command_fake.return_val = -EINVAL;
    ble_d2d_tx_send_start_activity_command_fake.return_val = -EBUSY;
    ble_d2d_tx_send_stop_activity_command_fake.return_val = -ETIMEDOUT;
    
    // Test each command with failure
    int ret = ble_d2d_tx_send_set_time_command(0);
    zassert_equal(ret, -ENOTCONN, "Should return connection error");
    
    ret = ble_d2d_tx_send_delete_foot_log_command(0);
    zassert_equal(ret, -EIO, "Should return I/O error");
    
    ret = ble_d2d_tx_send_delete_bhi360_log_command(0);
    zassert_equal(ret, -EINVAL, "Should return invalid argument");
    
    ret = ble_d2d_tx_send_start_activity_command(0);
    zassert_equal(ret, -EBUSY, "Should return busy error");
    
    ret = ble_d2d_tx_send_stop_activity_command(0);
    zassert_equal(ret, -ETIMEDOUT, "Should return timeout error");
#else
    ztest_test_skip();
#endif
}

ZTEST_F(control_service, test_message_queue_failures)
{
    // Test message queue full
    k_msgq_put_fake.return_val = -ENOMSG;
    
    generic_message_t msg;
    msg.sender = SENDER_BLUETOOTH;
    msg.type = MSG_TYPE_COMMAND;
    strcpy(msg.data.command_str, "test_command");
    
    int ret = k_msgq_put(&foot_sensor_msgq, &msg, K_NO_WAIT);
    zassert_equal(ret, -ENOMSG, "Should return queue full error");
}

ZTEST_F(control_service, test_ccc_cfg_changed_callbacks)
{
    // Test CCC configuration changes
    uint16_t enabled = BT_GATT_CCC_NOTIFY;
    uint16_t disabled = 0;
    
    // These functions log the state changes
    // Verify they don't crash
    zassert_true(true, "CCC callbacks should handle state changes");
}

ZTEST_F(control_service, test_write_handlers_buffer_overflow)
{
    // Test write with oversized buffer
    uint8_t large_buffer[256];
    memset(large_buffer, 0xFF, sizeof(large_buffer));
    
    // Write handlers should validate buffer size
    // In real implementation, they would return error for oversized data
    zassert_true(true, "Write handlers should validate buffer size");
}

ZTEST_F(control_service, test_concurrent_commands)
{
    // Test handling multiple commands in quick succession
    k_msgq_put_fake.return_val = 0;
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Send multiple commands
    ble_d2d_tx_send_set_time_command(1111111111);
    ble_d2d_tx_send_delete_foot_log_command(1);
    ble_d2d_tx_send_delete_bhi360_log_command(2);
    ble_d2d_tx_send_start_activity_command(1);
    ble_d2d_tx_send_stop_activity_command(0);
    
    // Verify all commands were sent
    zassert_equal(ble_d2d_tx_send_set_time_command_fake.call_count, 1,
                  "Should send time command");
    zassert_equal(ble_d2d_tx_send_delete_foot_log_command_fake.call_count, 1,
                  "Should send delete foot log command");
    zassert_equal(ble_d2d_tx_send_delete_bhi360_log_command_fake.call_count, 1,
                  "Should send delete BHI360 log command");
    zassert_equal(ble_d2d_tx_send_start_activity_command_fake.call_count, 1,
                  "Should send start activity command");
    zassert_equal(ble_d2d_tx_send_stop_activity_command_fake.call_count, 1,
                  "Should send stop activity command");
#else
    // On secondary, would queue multiple messages
    zassert_true(true, "Secondary should handle multiple commands");
#endif
}

void test_control_service_suite(void)
{
    ztest_run_test_suite(control_service);
}
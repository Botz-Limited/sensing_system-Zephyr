/**
 * @file test_bluetooth_debug.cpp
 * @brief Unit tests for bluetooth_debug module
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/fff.h>

#include "bluetooth_debug.hpp"

DEFINE_FFF_GLOBALS;

// Mock functions
FAKE_VALUE_FUNC(bt_security_t, bt_conn_get_security, struct bt_conn *);
FAKE_VOID_FUNC(bt_addr_le_to_str, const bt_addr_le_t *, char *, size_t);
FAKE_VALUE_FUNC(const bt_addr_le_t *, bt_conn_get_dst, struct bt_conn *);
FAKE_VOID_FUNC(bt_foreach_bond, uint8_t, bt_foreach_bond_cb, void *);
FAKE_VALUE_FUNC(int, bt_unpair, uint8_t, const bt_addr_le_t *);

// Test fixture
struct bluetooth_debug_fixture {
    struct bt_conn conn;
    bt_addr_le_t addr;
};

static void *bluetooth_debug_setup(void)
{
    static struct bluetooth_debug_fixture fixture;
    memset(&fixture, 0, sizeof(fixture));
    
    // Reset all fakes
    RESET_FAKE(bt_conn_get_security);
    RESET_FAKE(bt_addr_le_to_str);
    RESET_FAKE(bt_conn_get_dst);
    RESET_FAKE(bt_foreach_bond);
    RESET_FAKE(bt_unpair);
    
    return &fixture;
}

static void bluetooth_debug_teardown(void *f)
{
    ARG_UNUSED(f);
}

ZTEST_SUITE(bluetooth_debug, NULL, bluetooth_debug_setup, NULL, NULL, bluetooth_debug_teardown);

ZTEST_F(bluetooth_debug, test_bt_debug_check_security_null_conn)
{
    // Test with NULL connection
    bt_debug_check_security(NULL);
    
    // Should not call get_security with NULL
    zassert_equal(bt_conn_get_security_fake.call_count, 0, 
                  "Should not call bt_conn_get_security with NULL connection");
}

ZTEST_F(bluetooth_debug, test_bt_debug_check_security_valid_conn)
{
    // Setup
    bt_conn_get_security_fake.return_val = BT_SECURITY_L2;
    
    // Execute
    bt_debug_check_security(&fixture->conn);
    
    // Verify
    zassert_equal(bt_conn_get_security_fake.call_count, 1, 
                  "Should call bt_conn_get_security once");
    zassert_equal(bt_conn_get_security_fake.arg0_val, &fixture->conn,
                  "Should pass correct connection");
}

ZTEST_F(bluetooth_debug, test_bt_debug_check_security_all_levels)
{
    bt_security_t levels[] = {
        BT_SECURITY_L0, BT_SECURITY_L1, BT_SECURITY_L2, 
        BT_SECURITY_L3, BT_SECURITY_L4, (bt_security_t)99
    };
    
    for (int i = 0; i < ARRAY_SIZE(levels); i++) {
        RESET_FAKE(bt_conn_get_security);
        bt_conn_get_security_fake.return_val = levels[i];
        
        bt_debug_check_security(&fixture->conn);
        
        zassert_equal(bt_conn_get_security_fake.call_count, 1,
                      "Should call bt_conn_get_security for level %d", levels[i]);
    }
}

static void mock_bond_cb(const struct bt_bond_info *info, void *data)
{
    int *count = (int *)data;
    (*count)++;
}

ZTEST(bluetooth_debug, test_bt_debug_list_bonds)
{
    // Setup mock to call the callback
    bt_foreach_bond_fake.custom_fake = [](uint8_t id, bt_foreach_bond_cb cb, void *data) {
        struct bt_bond_info info = {0};
        // Simulate 3 bonds
        for (int i = 0; i < 3; i++) {
            cb(&info, data);
        }
    };
    
    // Execute
    bt_debug_list_bonds();
    
    // Verify
    zassert_equal(bt_foreach_bond_fake.call_count, 1, 
                  "Should call bt_foreach_bond once");
    zassert_equal(bt_foreach_bond_fake.arg0_val, BT_ID_DEFAULT,
                  "Should use default ID");
}

ZTEST(bluetooth_debug, test_bt_debug_check_services)
{
    // This function just logs information, so we just verify it doesn't crash
    bt_debug_check_services();
}

ZTEST_F(bluetooth_debug, test_bt_debug_force_repairing_null_conn)
{
    // Test with NULL connection
    bt_debug_force_repairing(NULL);
    
    // Should not call unpair
    zassert_equal(bt_unpair_fake.call_count, 0,
                  "Should not call bt_unpair with NULL connection");
}

ZTEST_F(bluetooth_debug, test_bt_debug_force_repairing_valid_conn)
{
    // Setup
    bt_conn_get_dst_fake.return_val = &fixture->addr;
    bt_unpair_fake.return_val = 0;
    
    // Execute
    bt_debug_force_repairing(&fixture->conn);
    
    // Verify
    zassert_equal(bt_conn_get_dst_fake.call_count, 1,
                  "Should call bt_conn_get_dst");
    zassert_equal(bt_unpair_fake.call_count, 1,
                  "Should call bt_unpair");
    zassert_equal(bt_unpair_fake.arg0_val, BT_ID_DEFAULT,
                  "Should use default ID");
    zassert_equal(bt_unpair_fake.arg1_val, &fixture->addr,
                  "Should pass correct address");
}

ZTEST_F(bluetooth_debug, test_bt_debug_force_repairing_no_addr)
{
    // Setup - no address available
    bt_conn_get_dst_fake.return_val = NULL;
    
    // Execute
    bt_debug_force_repairing(&fixture->conn);
    
    // Verify
    zassert_equal(bt_unpair_fake.call_count, 0,
                  "Should not call bt_unpair when no address");
}

ZTEST_F(bluetooth_debug, test_bt_debug_force_repairing_unpair_fail)
{
    // Setup
    bt_conn_get_dst_fake.return_val = &fixture->addr;
    bt_unpair_fake.return_val = -EINVAL;
    
    // Execute
    bt_debug_force_repairing(&fixture->conn);
    
    // Verify
    zassert_equal(bt_unpair_fake.call_count, 1,
                  "Should attempt to unpair even if it fails");
}

ZTEST(bluetooth_debug, test_bt_debug_test_characteristic_access)
{
    // This function just logs information
    bt_debug_test_characteristic_access();
}

ZTEST_F(bluetooth_debug, test_bt_debug_info_service_status_null)
{
    // Execute with NULL
    bt_debug_info_service_status(NULL);
    
    // Should still call other debug functions
    zassert_equal(bt_foreach_bond_fake.call_count, 1,
                  "Should still list bonds with NULL connection");
}

ZTEST_F(bluetooth_debug, test_bt_debug_info_service_status_valid)
{
    // Setup
    bt_conn_get_security_fake.return_val = BT_SECURITY_L2;
    
    // Execute
    bt_debug_info_service_status(&fixture->conn);
    
    // Verify all debug functions are called
    zassert_equal(bt_conn_get_security_fake.call_count, 1,
                  "Should check security");
    zassert_equal(bt_foreach_bond_fake.call_count, 1,
                  "Should list bonds");
}

void test_bluetooth_debug_suite(void)
{
    ztest_run_test_suite(bluetooth_debug);
}
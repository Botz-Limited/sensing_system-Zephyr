/**
 * @file bluetooth_debug.cpp
 * @brief Bluetooth debugging utilities for information service issues
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "bluetooth_debug.hpp"

LOG_MODULE_REGISTER(bt_debug, LOG_LEVEL_WRN);

// Debug function to check connection security status
void bt_debug_check_security(struct bt_conn *conn)
{
    if (!conn) {
        LOG_ERR("No active connection to check security");
        return;
    }

    bt_security_t security = bt_conn_get_security(conn);
    LOG_INF("Current security level: %d", security);
    
    switch (security) {
        case BT_SECURITY_L0:
            LOG_INF("Security Level 0: No security (no authentication and no encryption)");
            break;
        case BT_SECURITY_L1:
            LOG_INF("Security Level 1: Unauthenticated pairing with encryption");
            break;
        case BT_SECURITY_L2:
            LOG_INF("Security Level 2: Authenticated pairing with encryption");
            break;
        case BT_SECURITY_L3:
            LOG_INF("Security Level 3: Authenticated pairing with encryption using SC");
            break;
        case BT_SECURITY_L4:
            LOG_INF("Security Level 4: Authenticated LE Secure Connections pairing");
            break;
        default:
            LOG_ERR("Unknown security level: %d", security);
    }
}

// Debug function to list all bonded devices
void bt_debug_list_bonds(void)
{
    struct bond_info {
        int count;
    } info = {0};
    
    auto bond_cb = [](const struct bt_bond_info *bond_info, void *user_data) {
        struct bond_info *info = (struct bond_info *)user_data;
        char addr_str[BT_ADDR_LE_STR_LEN];
        
        bt_addr_le_to_str(&bond_info->addr, addr_str, sizeof(addr_str));
        LOG_INF("Bond %d: %s", info->count++, addr_str);
    };
    
    LOG_INF("=== Listing all bonded devices ===");
    bt_foreach_bond(BT_ID_DEFAULT, bond_cb, &info);
    LOG_INF("Total bonds: %d", info.count);
}

// Debug function to check GATT service registration
void bt_debug_check_services(void)
{
    LOG_INF("=== Checking GATT Services ===");
    
    // This is a simplified check - in real implementation you'd iterate through services
    LOG_INF("Information Service should be registered with UUID: 0c372eaa-27eb-437e-bef4-775aefaf3c97");
    LOG_INF("Check if all characteristics are accessible after pairing");
}

// Function to force re-pairing
void bt_debug_force_repairing(struct bt_conn *conn)
{
    if (!conn) {
        LOG_ERR("No active connection to force re-pairing");
        return;
    }
    
    const bt_addr_le_t *addr = bt_conn_get_dst(conn);
    if (!addr) {
        LOG_ERR("Failed to get connection address");
        return;
    }
    
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    
    LOG_WRN("Attempting to unpair device: %s", addr_str);
    int err = bt_unpair(BT_ID_DEFAULT, addr);
    if (err) {
        LOG_ERR("Failed to unpair: %d", err);
    } else {
        LOG_INF("Device unpaired successfully. Disconnect and reconnect to re-pair.");
    }
}

// Function to test characteristic access
void bt_debug_test_characteristic_access(void)
{
    LOG_INF("=== Testing Characteristic Access ===");
    LOG_INF("If characteristics are not accessible:");
    LOG_INF("1. Ensure device is paired (check phone BT settings)");
    LOG_INF("2. Security level must be >= L2 for encrypted characteristics");
    LOG_INF("3. Try unpairing and re-pairing the device");
    LOG_INF("4. Check if CONFIG_BT_SMP is enabled (it is in your config)");
}

// Main debug function to call from shell or elsewhere
void bt_debug_info_service_status(struct bt_conn *conn)
{
    LOG_INF("========================================");
    LOG_INF("Bluetooth Information Service Debug Info");
    LOG_INF("========================================");
    
    bt_debug_check_security(conn);
    bt_debug_list_bonds();
    bt_debug_check_services();
    bt_debug_test_characteristic_access();
    
    LOG_INF("========================================");
}
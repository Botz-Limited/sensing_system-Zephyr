/**
 * @file bluetooth_debug.hpp
 * @brief Bluetooth debugging utilities header
 */

#ifndef BLUETOOTH_DEBUG_HPP
#define BLUETOOTH_DEBUG_HPP

#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Check the current security level of a connection
 * @param conn The Bluetooth connection to check
 */
void bt_debug_check_security(struct bt_conn *conn);

/**
 * @brief List all bonded devices
 */
void bt_debug_list_bonds(void);

/**
 * @brief Check if GATT services are properly registered
 */
void bt_debug_check_services(void);

/**
 * @brief Force re-pairing by removing existing bond
 * @param conn The connection to unpair
 */
void bt_debug_force_repairing(struct bt_conn *conn);

/**
 * @brief Test characteristic access and provide guidance
 */
void bt_debug_test_characteristic_access(void);

/**
 * @brief Main debug function to diagnose information service issues
 * @param conn The active Bluetooth connection (can be NULL)
 */
void bt_debug_info_service_status(struct bt_conn *conn);

#ifdef __cplusplus
}
#endif

#endif // BLUETOOTH_DEBUG_HPP
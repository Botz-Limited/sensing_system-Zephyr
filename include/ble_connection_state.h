/**
 * @file ble_connection_state.h
 * @brief Global BLE connection state tracking
 * 
 * This header provides a global way for all modules to check
 * if a phone is connected via BLE, allowing them to skip
 * BLE operations when disconnected to prevent buffer overflow.
 */

#ifndef BLE_CONNECTION_STATE_H
#define BLE_CONNECTION_STATE_H

#include <zephyr/kernel.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Check if a phone is currently connected via BLE
 * @return true if connected, false otherwise
 */
bool ble_phone_is_connected(void);

/**
 * @brief Set the phone BLE connection state (called by bluetooth module)
 * @param connected true if phone connected, false if disconnected
 */
void ble_set_phone_connection_state(bool connected);

/**
 * @brief Clear all BLE message queues (called on reconnection)
 */
void ble_clear_message_queues(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CONNECTION_STATE_H */
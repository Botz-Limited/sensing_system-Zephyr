/**
 * @file ble_d2d_smp_client.hpp
 * @brief D2D SMP Client for forwarding MCUmgr commands to secondary device
 */

#ifndef BLE_D2D_SMP_CLIENT_HPP
#define BLE_D2D_SMP_CLIENT_HPP

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

// Callback for SMP responses from secondary device
typedef void (*d2d_smp_response_cb_t)(const uint8_t *data, size_t len);

/**
 * @brief Initialize the D2D SMP client
 * @param response_cb Callback for SMP responses from secondary
 * @return 0 on success, negative error code on failure
 */
int ble_d2d_smp_client_init(d2d_smp_response_cb_t response_cb);

/**
 * @brief Set the D2D connection
 * @param conn Connection handle to secondary device (NULL to clear)
 */
void ble_d2d_smp_client_set_connection(struct bt_conn *conn);

/**
 * @brief Send SMP frame to secondary device
 * @param data SMP frame data
 * @param len Length of SMP frame
 * @return 0 on success, negative error code on failure
 */
int ble_d2d_smp_client_send(const uint8_t *data, size_t len);

/**
 * @brief Check if D2D SMP client is ready
 * @return true if connected and service discovered
 */
bool ble_d2d_smp_client_is_ready(void);

/**
 * @brief Start service discovery for SMP service on secondary
 * @param conn Connection to secondary device
 * @return 0 on success, negative error code on failure
 */
int ble_d2d_smp_client_discover(struct bt_conn *conn);

#ifdef __cplusplus
}
#endif

#endif /* BLE_D2D_SMP_CLIENT_HPP */
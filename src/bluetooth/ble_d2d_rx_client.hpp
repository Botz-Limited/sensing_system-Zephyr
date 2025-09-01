#pragma once

#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start discovery of D2D TX service on secondary device
 * @param conn Connection handle to secondary device
 */
void d2d_rx_client_start_discovery(struct bt_conn *conn);

/**
 * @brief Handle disconnection from secondary device
 */
void d2d_rx_client_disconnected(void);

#ifdef __cplusplus
}
#endif
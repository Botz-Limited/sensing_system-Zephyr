/**
 * @file smp_proxy.hpp
 * @brief SMP Proxy Service for transparent MCUmgr forwarding to secondary devices
 *
 * This service allows mobile apps to use standard MCUmgr/SMP protocol for both
 * primary and secondary devices by transparently forwarding SMP frames.
 */

#ifndef SMP_PROXY_HPP
#define SMP_PROXY_HPP

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#ifdef __cplusplus
extern "C" {
#endif

// SMP Proxy Service UUID: 8D53DC1E-1DB7-4CD3-868B-8A527460AA84
// (One digit different from standard SMP service to avoid conflicts)
#define SMP_PROXY_SERVICE_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x8D53DC1E, 0x1DB7, 0x4CD3, 0x868B, 0x8A527460AA84))

// Characteristic UUIDs
#define SMP_PROXY_TARGET_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xDA2E7829, 0xFBCE, 0x4E01, 0xAE9E, 0x261174997C48))
#define SMP_PROXY_DATA_UUID BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0xDA2E7828, 0xFBCE, 0x4E01, 0xAE9E, 0x261174997C48))

// Target device selection
enum smp_proxy_target {
    SMP_TARGET_PRIMARY = 0x00,
    SMP_TARGET_SECONDARY = 0x01,
};

/**
 * @brief Initialize the SMP proxy service
 * @return 0 on success, negative error code on failure
 */
int smp_proxy_init(void);

/**
 * @brief Set the secondary device connection
 * @param conn Secondary device connection handle (NULL to clear)
 */
void smp_proxy_set_secondary_conn(struct bt_conn *conn);

/**
 * @brief Get current target device
 * @return Current target (PRIMARY or SECONDARY)
 */
enum smp_proxy_target smp_proxy_get_target(void);

/**
 * @brief Check if SMP proxy is ready for secondary device operations
 * @return true if secondary device is connected and ready
 */
bool smp_proxy_is_secondary_ready(void);

/**
 * @brief Clear the phone connection when it disconnects
 * @param conn Phone connection handle that disconnected
 */
void smp_proxy_clear_phone_conn(struct bt_conn *conn);

#ifdef __cplusplus
}
#endif

#endif /* SMP_PROXY_HPP */
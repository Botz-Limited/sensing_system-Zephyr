/**
 * @file ble_connection_manager.hpp
 * @brief Centralized BLE connection management with thread safety
 */

#ifndef BLE_CONNECTION_MANAGER_HPP
#define BLE_CONNECTION_MANAGER_HPP

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>

class BleConnectionManager {
public:
    static BleConnectionManager& getInstance() {
        static BleConnectionManager instance;
        return instance;
    }

    // Phone connection management
    void setPhoneConnection(struct bt_conn *conn);
    struct bt_conn* getPhoneConnection();
    void clearPhoneConnection();
    bool hasPhoneConnection();

    // D2D connection management
    void setD2DConnection(struct bt_conn *conn);
    struct bt_conn* getD2DConnection();
    void clearD2DConnection();
    bool hasD2DConnection();

    // Secondary connection (for primary device acting as central)
    void setSecondaryConnection(struct bt_conn *conn);
    struct bt_conn* getSecondaryConnection();
    void clearSecondaryConnection();
    bool hasSecondaryConnection();

    // Connection state queries
    bool isConnectionValid(struct bt_conn *conn);
    bool isPhoneConnection(struct bt_conn *conn);
    bool isD2DConnection(struct bt_conn *conn);

private:
    BleConnectionManager();
    ~BleConnectionManager() = default;
    BleConnectionManager(const BleConnectionManager&) = delete;
    BleConnectionManager& operator=(const BleConnectionManager&) = delete;

    struct k_mutex conn_mutex;
    struct bt_conn *phone_conn;
    struct bt_conn *d2d_conn;
    struct bt_conn *secondary_conn;  // For primary device only
};

#endif // BLE_CONNECTION_MANAGER_HPP
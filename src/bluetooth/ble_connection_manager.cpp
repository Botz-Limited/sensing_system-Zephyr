/**
 * @file ble_connection_manager.cpp
 * @brief Implementation of centralized BLE connection management
 */

#include "ble_connection_manager.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_conn_mgr, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

BleConnectionManager::BleConnectionManager() 
    : phone_conn(nullptr), d2d_conn(nullptr), secondary_conn(nullptr) {
    k_mutex_init(&conn_mutex);
}

void BleConnectionManager::setPhoneConnection(struct bt_conn *conn) {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    if (phone_conn && phone_conn != conn) {
        bt_conn_unref(phone_conn);
    }
    phone_conn = conn;
    if (phone_conn) {
        bt_conn_ref(phone_conn);
    }
    k_mutex_unlock(&conn_mutex);
    LOG_INF("Phone connection %s", conn ? "set" : "cleared");
}

struct bt_conn* BleConnectionManager::getPhoneConnection() {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    struct bt_conn *conn = phone_conn;
    k_mutex_unlock(&conn_mutex);
    return conn;
}

void BleConnectionManager::clearPhoneConnection() {
    setPhoneConnection(nullptr);
}

bool BleConnectionManager::hasPhoneConnection() {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    bool has_conn = (phone_conn != nullptr) && isConnectionValid(phone_conn);
    k_mutex_unlock(&conn_mutex);
    return has_conn;
}

void BleConnectionManager::setD2DConnection(struct bt_conn *conn) {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    if (d2d_conn && d2d_conn != conn) {
        bt_conn_unref(d2d_conn);
    }
    d2d_conn = conn;
    if (d2d_conn) {
        bt_conn_ref(d2d_conn);
    }
    k_mutex_unlock(&conn_mutex);
    LOG_INF("D2D connection %s", conn ? "set" : "cleared");
}

struct bt_conn* BleConnectionManager::getD2DConnection() {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    struct bt_conn *conn = d2d_conn;
    k_mutex_unlock(&conn_mutex);
    return conn;
}

void BleConnectionManager::clearD2DConnection() {
    setD2DConnection(nullptr);
}

bool BleConnectionManager::hasD2DConnection() {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    bool has_conn = (d2d_conn != nullptr) && isConnectionValid(d2d_conn);
    k_mutex_unlock(&conn_mutex);
    return has_conn;
}

void BleConnectionManager::setSecondaryConnection(struct bt_conn *conn) {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    if (secondary_conn && secondary_conn != conn) {
        bt_conn_unref(secondary_conn);
    }
    secondary_conn = conn;
    if (secondary_conn) {
        bt_conn_ref(secondary_conn);
    }
    k_mutex_unlock(&conn_mutex);
    LOG_INF("Secondary connection %s", conn ? "set" : "cleared");
}

struct bt_conn* BleConnectionManager::getSecondaryConnection() {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    struct bt_conn *conn = secondary_conn;
    k_mutex_unlock(&conn_mutex);
    return conn;
}

void BleConnectionManager::clearSecondaryConnection() {
    setSecondaryConnection(nullptr);
}

bool BleConnectionManager::hasSecondaryConnection() {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    bool has_conn = (secondary_conn != nullptr) && isConnectionValid(secondary_conn);
    k_mutex_unlock(&conn_mutex);
    return has_conn;
}

bool BleConnectionManager::isConnectionValid(struct bt_conn *conn) {
    if (!conn) return false;
    
    struct bt_conn_info info;
    int err = bt_conn_get_info(conn, &info);
    return (err == 0 && info.state == BT_CONN_STATE_CONNECTED);
}

bool BleConnectionManager::isPhoneConnection(struct bt_conn *conn) {
    if (!conn) return false;
    
    k_mutex_lock(&conn_mutex, K_FOREVER);
    bool is_phone = (conn == phone_conn);
    k_mutex_unlock(&conn_mutex);
    return is_phone;
}

bool BleConnectionManager::isD2DConnection(struct bt_conn *conn) {
    if (!conn) return false;
    
    k_mutex_lock(&conn_mutex, K_FOREVER);
    bool is_d2d = (conn == d2d_conn || conn == secondary_conn);
    k_mutex_unlock(&conn_mutex);
    return is_d2d;
}
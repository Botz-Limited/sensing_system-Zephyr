#include "ble_d2d_rx.hpp"
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_d2d_rx, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// D2D RX Service UUID: e060ca1f-3115-4ad6-9709-8c5ff3bf558b
static struct bt_uuid_128 d2d_rx_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe060ca1f, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// D2D RX Characteristics (increment first byte from control_service.cpp)
static struct bt_uuid_128 d2d_set_time_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca1f, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_delete_foot_log_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca82, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_delete_bhi360_log_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca83, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_start_activity_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca84, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_stop_activity_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca85, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// Dummy handlers (replace with your logic as needed)
static ssize_t d2d_set_time_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) { LOG_INF("D2D RX: Set Time Command"); return len; }
static ssize_t d2d_delete_foot_log_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) { LOG_INF("D2D RX: Delete Foot Log Command"); return len; }
static ssize_t d2d_delete_bhi360_log_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) { LOG_INF("D2D RX: Delete BHI360 Log Command"); return len; }
static ssize_t d2d_start_activity_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) { LOG_INF("D2D RX: Start Activity Command"); return len; }
static ssize_t d2d_stop_activity_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) { LOG_INF("D2D RX: Stop Activity Command"); return len; }

BT_GATT_SERVICE_DEFINE(d2d_rx_service,
    BT_GATT_PRIMARY_SERVICE(&d2d_rx_service_uuid),
    BT_GATT_CHARACTERISTIC(&d2d_set_time_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_set_time_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_delete_foot_log_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_delete_foot_log_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_delete_bhi360_log_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_delete_bhi360_log_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_start_activity_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_start_activity_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_stop_activity_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_stop_activity_write, NULL)
);

void ble_d2d_rx_init(void) {
    // Nothing needed if using BT_GATT_SERVICE_DEFINE
}

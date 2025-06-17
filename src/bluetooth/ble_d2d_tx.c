#include "ble_d2d_tx.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bluetooth, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// D2D TX Service UUID: 75ad68d6-200c-437d-98b5-061862076c5f
static struct bt_uuid_128 d2d_tx_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x75ad68d6, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// D2D TX Characteristics (increment first byte from information_service.cpp)
static struct bt_uuid_128 d2d_status_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d6, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_foot_sensor_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d7, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_charge_status_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d8, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_foot_sensor_req_id_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68d9, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_bhi360_log_available_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68da, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_bhi360_req_id_path_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68db, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_foot_sensor_samples_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68dc, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_bhi360_data1_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68dd, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_bhi360_data2_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68de, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_bhi360_data3_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68df, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));
static struct bt_uuid_128 d2d_current_time_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x76ad68e0, 0x200c, 0x437d, 0x98b5, 0x061862076c5f));

// For each characteristic, you would discover the handle and provide a send function.
// Here is a template for one characteristic:
static struct bt_conn *d2d_conn;
static uint16_t d2d_status_handle;

void ble_d2d_tx_init(void) {
    // TODO: Implement scanning, connecting, and discovering the D2D RX service and characteristics
    // Save the handle for each characteristic
}

int ble_d2d_tx_send_status(uint32_t status) {
    if (!d2d_conn || !d2d_status_handle) return -ENOTCONN;
    struct bt_gatt_write_params params = {
        .handle = d2d_status_handle,
        .data = &status,
        .length = sizeof(status),
        .func = NULL,
    };
    return bt_gatt_write(d2d_conn, &params);
}
// Add more send functions for other characteristics as needed

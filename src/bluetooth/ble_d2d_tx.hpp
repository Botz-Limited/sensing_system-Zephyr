#pragma once
#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>


void ble_d2d_tx_init(void);
int ble_d2d_tx_send_set_time(uint32_t epoch_time);
// Add more send functions for other characteristics as needed


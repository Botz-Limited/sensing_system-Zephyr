#pragma once

#include <zephyr/kernel.h>
#include <app.hpp>

#ifdef __cplusplus
extern "C" {
#endif

// Command types for the queue
enum d2d_tx_cmd_type {
    D2D_TX_CMD_SET_TIME,
    D2D_TX_CMD_DELETE_FOOT_LOG,
    D2D_TX_CMD_DELETE_BHI360_LOG,
    D2D_TX_CMD_DELETE_ACTIVITY_LOG,
    D2D_TX_CMD_START_ACTIVITY,
    D2D_TX_CMD_STOP_ACTIVITY,
    D2D_TX_CMD_TRIGGER_CALIBRATION,
    D2D_TX_CMD_REQUEST_DEVICE_INFO,
    D2D_TX_CMD_WEIGHT_CALIBRATION_TRIGGER,
    D2D_TX_CMD_CONN_PARAM_CONTROL
};

// Command structure for the queue
struct d2d_tx_queued_cmd {
    enum d2d_tx_cmd_type type;
    union {
        uint32_t time_value;
        uint8_t byte_value;
    } data;
};

// Initialize the D2D TX command queue
void ble_d2d_tx_queue_init(void);

// Queue a command (returns 0 on success, -ENOMEM if queue full)
int ble_d2d_tx_queue_command(enum d2d_tx_cmd_type type, const void *data);

// Process all queued commands (called when D2D TX discovery completes)
void ble_d2d_tx_process_queued_commands(void);

// Check if D2D TX is ready
bool ble_d2d_tx_is_ready(void);

// Queue weight calibration trigger command
int ble_d2d_tx_queue_weight_calibration_trigger_command(uint8_t value);

#ifdef __cplusplus
}
#endif
#include "ble_d2d_tx_queue.hpp"
#include "ble_d2d_tx.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(d2d_tx_queue, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// Queue for pending D2D TX commands
#define D2D_TX_QUEUE_SIZE 10
K_MSGQ_DEFINE(d2d_tx_cmd_queue, sizeof(struct d2d_tx_queued_cmd), D2D_TX_QUEUE_SIZE, 4);

// Flag to track if D2D TX is ready
static bool d2d_tx_ready = false;

void ble_d2d_tx_queue_init(void)
{
    k_msgq_purge(&d2d_tx_cmd_queue);
    d2d_tx_ready = false;
    LOG_INF("D2D TX command queue initialized");
}

int ble_d2d_tx_queue_command(enum d2d_tx_cmd_type type, const void *data)
{
    struct d2d_tx_queued_cmd cmd;
    cmd.type = type;
    
    // Copy data based on command type
    switch (type) {
    case D2D_TX_CMD_SET_TIME:
        cmd.data.time_value = *(const uint32_t *)data;
        break;
    case D2D_TX_CMD_DELETE_FOOT_LOG:
    case D2D_TX_CMD_DELETE_BHI360_LOG:
    case D2D_TX_CMD_DELETE_ACTIVITY_LOG:
    case D2D_TX_CMD_START_ACTIVITY:
    case D2D_TX_CMD_STOP_ACTIVITY:
    case D2D_TX_CMD_TRIGGER_CALIBRATION:
    case D2D_TX_CMD_REQUEST_DEVICE_INFO:
    case D2D_TX_CMD_WEIGHT_CALIBRATION_TRIGGER:
        cmd.data.byte_value = *(const uint8_t *)data;
        break;
    default:
        LOG_ERR("Unknown command type: %d", type);
        return -EINVAL;
    }
    
    int err = k_msgq_put(&d2d_tx_cmd_queue, &cmd, K_NO_WAIT);
    if (err) {
        LOG_ERR("Failed to queue D2D TX command (queue full?)");
        return err;
    }
    
    LOG_INF("Queued D2D TX command type %d", type);
    return 0;
}

void ble_d2d_tx_process_queued_commands(void)
{
    struct d2d_tx_queued_cmd cmd;
    int processed = 0;
    
    LOG_INF("Processing queued D2D TX commands");
    d2d_tx_ready = true;
    
    while (k_msgq_get(&d2d_tx_cmd_queue, &cmd, K_NO_WAIT) == 0) {
        int err = 0;
        
        switch (cmd.type) {
        case D2D_TX_CMD_SET_TIME:
            LOG_INF("Executing queued set time command: %u", cmd.data.time_value);
            err = ble_d2d_tx_send_set_time_command(cmd.data.time_value);
            break;
            
        case D2D_TX_CMD_DELETE_FOOT_LOG:
            LOG_INF("Executing queued delete foot log command: %u", cmd.data.byte_value);
            err = ble_d2d_tx_send_delete_foot_log_command(cmd.data.byte_value);
            break;
            
        case D2D_TX_CMD_DELETE_BHI360_LOG:
            LOG_INF("Executing queued delete BHI360 log command: %u", cmd.data.byte_value);
            err = ble_d2d_tx_send_delete_bhi360_log_command(cmd.data.byte_value);
            break;
            
        case D2D_TX_CMD_DELETE_ACTIVITY_LOG:
            LOG_INF("Executing queued delete activity log command: %u", cmd.data.byte_value);
            err = ble_d2d_tx_send_delete_activity_log_command(cmd.data.byte_value);
            break;
            
        case D2D_TX_CMD_START_ACTIVITY:
            LOG_INF("Executing queued start activity command: %u", cmd.data.byte_value);
            err = ble_d2d_tx_send_start_activity_command(cmd.data.byte_value);
            break;
            
        case D2D_TX_CMD_STOP_ACTIVITY:
            LOG_INF("Executing queued stop activity command: %u", cmd.data.byte_value);
            err = ble_d2d_tx_send_stop_activity_command(cmd.data.byte_value);
            break;
            
        case D2D_TX_CMD_TRIGGER_CALIBRATION:
            LOG_INF("Executing queued trigger calibration command: %u", cmd.data.byte_value);
            err = ble_d2d_tx_send_trigger_bhi360_calibration_command(cmd.data.byte_value);
            break;
            
        case D2D_TX_CMD_REQUEST_DEVICE_INFO:
            LOG_INF("Executing queued request device info command");
            err = ble_d2d_tx_request_device_info();
            break;
            
        case D2D_TX_CMD_WEIGHT_CALIBRATION_TRIGGER:
            LOG_INF("Executing queued weight calibration trigger command: %u", cmd.data.byte_value);
            err = ble_d2d_tx_send_weight_calibration_trigger_command(cmd.data.byte_value);
            break;
            
        default:
            LOG_ERR("Unknown queued command type: %d", cmd.type);
            continue;
        }
        
        if (err) {
            LOG_ERR("Failed to execute queued command type %d: %d", cmd.type, err);
        } else {
            processed++;
        }
    }
    
    if (processed > 0) {
        LOG_INF("Processed %d queued D2D TX commands", processed);
    }
}

bool ble_d2d_tx_is_ready(void)
{
    return d2d_tx_ready;
}

int ble_d2d_tx_queue_weight_calibration_trigger_command(uint8_t value)
{
    return ble_d2d_tx_queue_command(D2D_TX_CMD_WEIGHT_CALIBRATION_TRIGGER, &value);
}
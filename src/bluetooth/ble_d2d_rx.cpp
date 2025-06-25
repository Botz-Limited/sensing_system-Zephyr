#include "ble_d2d_rx.hpp"
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <cstring>
#include <app.hpp>
#include <ble_services.hpp>
#include <app_event_manager.h>
#include <events/foot_sensor_event.h>
#include <events/motion_sensor_event.h>

LOG_MODULE_REGISTER(ble_d2d_rx, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

/*
 * D2D RX Module - Used by both Primary and Secondary devices
 * - Primary: Receives sensor data from secondary
 * - Secondary: Receives control commands from primary
 */

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
static struct bt_uuid_128 d2d_trigger_bhi360_calibration_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca87, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_fota_status_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca86, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_delete_activity_log_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca88, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// Helper function to swap endianness if needed
static uint32_t swap_to_little_endian(uint32_t value)
{
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return ((value << 24) & 0xFF000000) | ((value << 8) & 0x00FF0000) | 
           ((value >> 8) & 0x0000FF00) | ((value >> 24) & 0x000000FF);
    #else
    return value;
    #endif
}

// Set Time Command Handler - mirrors control_service.cpp implementation
static ssize_t d2d_set_time_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, 
                                  const void *buf, uint16_t len, uint16_t offset, uint8_t flags) 
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    
    if (len != sizeof(uint32_t) || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint32_t epoch_time;
    memcpy(&epoch_time, buf, sizeof(uint32_t));
    
    // Convert endianness if needed
    uint32_t converted_time = swap_to_little_endian(epoch_time);
    
    // Save epoch time to RTC
    set_current_time_from_epoch(converted_time);
    
    LOG_INF("D2D RX: Set Time Command - epoch: %u", converted_time);
    return len;
}

// Delete Foot Log Command Handler - mirrors control_service.cpp implementation
static ssize_t d2d_delete_foot_log_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, 
                                        const void *buf, uint16_t len, uint16_t offset, uint8_t flags) 
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    
    if (len != sizeof(uint8_t) || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint8_t id_to_delete;
    memcpy(&id_to_delete, buf, sizeof(uint8_t));
    
    generic_message_t delete_msg;
    delete_msg.sender = SENDER_BTH;  // Use BTH sender as we're acting on behalf of phone
    delete_msg.type = MSG_TYPE_DELETE_FOOT_LOG;
    delete_msg.data.delete_cmd.type = RECORD_HARDWARE_FOOT_SENSOR;
    delete_msg.data.delete_cmd.id = id_to_delete;
    
    if (k_msgq_put(&data_msgq, &delete_msg, K_NO_WAIT) != 0) {
        LOG_ERR("Failed to send delete foot log message for ID %u.", id_to_delete);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    
    LOG_INF("D2D RX: Delete Foot Log Command - ID: %u", id_to_delete);
    return len;
}

// Delete BHI360 Log Command Handler - mirrors control_service.cpp implementation
static ssize_t d2d_delete_bhi360_log_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, 
                                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags) 
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    
    if (len != sizeof(uint8_t) || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint8_t id_to_delete;
    memcpy(&id_to_delete, buf, sizeof(uint8_t));
    
    generic_message_t delete_msg;
    delete_msg.sender = SENDER_BTH;  // Use BTH sender as we're acting on behalf of phone
    delete_msg.type = MSG_TYPE_DELETE_BHI360_LOG;
    delete_msg.data.delete_cmd.type = RECORD_HARDWARE_BHI360;
    delete_msg.data.delete_cmd.id = id_to_delete;
    
    if (k_msgq_put(&data_msgq, &delete_msg, K_NO_WAIT) != 0) {
        LOG_ERR("Failed to send delete BHI360 log message for ID %u.", id_to_delete);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    
    LOG_INF("D2D RX: Delete BHI360 Log Command - ID: %u", id_to_delete);
    return len;
}

// Start Activity Command Handler - mirrors control_service.cpp implementation
static ssize_t d2d_start_activity_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, 
                                       const void *buf, uint16_t len, uint16_t offset, uint8_t flags) 
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    
    LOG_INF("D2D RX: Received start activity command write, len=%u", len);
    
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint8_t value = *((const uint8_t *)buf);
    LOG_INF("D2D RX: Start activity command value=%u", value);
    
    if (value == 1) {
        struct foot_sensor_start_activity_event *foot_evt = new_foot_sensor_start_activity_event();
        APP_EVENT_SUBMIT(foot_evt);
        
        struct motion_sensor_start_activity_event *motion_evt = new_motion_sensor_start_activity_event();
        APP_EVENT_SUBMIT(motion_evt);
        
        LOG_INF("D2D RX: Start Activity Command - submitted foot and motion sensor events");
    } else {
        LOG_WRN("D2D RX: Start activity command ignored (value=%u)", value);
    }
    
    return len;
}

// Stop Activity Command Handler - mirrors control_service.cpp implementation
static ssize_t d2d_stop_activity_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, 
                                      const void *buf, uint16_t len, uint16_t offset, uint8_t flags) 
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint8_t value = *((const uint8_t *)buf);
    
    if (value == 1) {
        struct foot_sensor_stop_activity_event *foot_evt = new_foot_sensor_stop_activity_event();
        APP_EVENT_SUBMIT(foot_evt);
        
        struct motion_sensor_stop_activity_event *motion_evt = new_motion_sensor_stop_activity_event();
        APP_EVENT_SUBMIT(motion_evt);
        
        LOG_INF("D2D RX: Stop Activity Command - submitted events");
    } else {
        LOG_WRN("D2D RX: Stop activity command ignored (value=%u)", value);
    }
    
    return len;
}

// Trigger BHI360 Calibration Handler - mirrors control_service.cpp implementation
static ssize_t d2d_trigger_bhi360_calibration_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint8_t value = *((const uint8_t *)buf);
    
    if (value == 1) {
        // Send message to motion sensor to trigger calibration
        generic_message_t calib_msg = {};
        calib_msg.sender = SENDER_BTH;
        calib_msg.type = MSG_TYPE_TRIGGER_BHI360_CALIBRATION;
        
        if (k_msgq_put(&motion_sensor_msgq, &calib_msg, K_NO_WAIT) != 0) {
            LOG_ERR("Failed to send calibration trigger to motion sensor");
            return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
        }
        
        LOG_INF("D2D RX: Trigger BHI360 Calibration Command - sent to motion sensor");
    } else {
        LOG_WRN("D2D RX: Trigger BHI360 calibration command ignored (value=%u)", value);
    }
    
    return len;
}

// FOTA Status Handler - receives completion status from secondary
static ssize_t d2d_fota_status_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    
    if (len != sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint8_t status = *((const uint8_t *)buf);
    
    if (status == 1) {  // 1 = complete
        LOG_INF("D2D RX: Secondary device reported FOTA complete");
        
        #if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
        // For primary device, we need to notify the FOTA proxy
        // But since FOTA proxy is conditionally compiled, we can't call it directly
        // Instead, we'll set a flag that can be checked by the FOTA proxy
        extern bool d2d_secondary_fota_complete;
        d2d_secondary_fota_complete = true;
        #endif
    }
    
    return len;
}

// Delete Activity Log Command Handler - mirrors control_service.cpp implementation
static ssize_t d2d_delete_activity_log_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, 
                                            const void *buf, uint16_t len, uint16_t offset, uint8_t flags) 
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);
    
    if (len != sizeof(uint8_t) || offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint8_t id_to_delete;
    memcpy(&id_to_delete, buf, sizeof(uint8_t));
    
    generic_message_t delete_msg;
    delete_msg.sender = SENDER_BTH;  // Use BTH sender as we're acting on behalf of phone
    delete_msg.type = MSG_TYPE_DELETE_ACTIVITY_LOG;
    delete_msg.data.delete_cmd.type = RECORD_HARDWARE_ACTIVITY;
    delete_msg.data.delete_cmd.id = id_to_delete;
    
    if (k_msgq_put(&data_msgq, &delete_msg, K_NO_WAIT) != 0) {
        LOG_ERR("Failed to send delete activity log message for ID %u.", id_to_delete);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    
    LOG_INF("D2D RX: Delete Activity Log Command - ID: %u", id_to_delete);
    return len;
}

BT_GATT_SERVICE_DEFINE(d2d_rx_service,
    BT_GATT_PRIMARY_SERVICE(&d2d_rx_service_uuid),
    BT_GATT_CHARACTERISTIC(&d2d_set_time_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_set_time_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_delete_foot_log_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_delete_foot_log_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_delete_bhi360_log_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_delete_bhi360_log_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_start_activity_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_start_activity_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_stop_activity_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_stop_activity_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_trigger_bhi360_calibration_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_trigger_bhi360_calibration_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_fota_status_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_fota_status_write, NULL),
    BT_GATT_CHARACTERISTIC(&d2d_delete_activity_log_command_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, d2d_delete_activity_log_write, NULL)
);

void ble_d2d_rx_init(void) {
    // Nothing needed if using BT_GATT_SERVICE_DEFINE
}

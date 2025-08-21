#include "ble_d2d_rx.hpp"
#include <app.hpp>
#include <app_event_manager.h>
#include <ble_services.hpp>
#include <cstring>
#include <events/foot_sensor_event.h>
#include <events/motion_sensor_event.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ble_d2d_rx, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

/*
 * D2D RX Module - Device-to-Device Receive Service
 *
 * DEVICE ROLES:
 * - PRIMARY DEVICE (Right shoe):
 *   - Receives sensor data from secondary via D2D TX service
 *   - This service receives status updates and sensor data
 *
 * - SECONDARY DEVICE (Left shoe):
 *   - Receives control commands from primary via this D2D RX service
 *   - Handles commands like: start/stop activity, set time, trigger calibration
 *   - Ensures secondary device mirrors primary device behavior
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
static struct bt_uuid_128 d2d_erase_flash_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca8e, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_trigger_bhi360_calibration_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca87, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_fota_status_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca86, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));
static struct bt_uuid_128 d2d_delete_activity_log_command_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca88, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// Request device info command UUID
static struct bt_uuid_128 d2d_request_device_info_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca89, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// GPS update command UUID
static struct bt_uuid_128 d2d_gps_update_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca8a, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// Weight calibration command UUID
static struct bt_uuid_128 d2d_weight_calibration_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xe160ca8b, 0x3115, 0x4ad6, 0x9709, 0x8c5ff3bf558b));

// Helper function to swap endianness if needed
static uint32_t swap_to_little_endian(uint32_t value)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return ((value << 24) & 0xFF000000) | ((value << 8) & 0x00FF0000) | ((value >> 8) & 0x0000FF00) |
           ((value >> 24) & 0x000000FF);
#else
    return value;
#endif
}

// Set Time Command Handler - mirrors control_service.cpp implementation
static ssize_t d2d_set_time_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
                                  uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (len != sizeof(uint32_t) || offset != 0)
    {
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
static ssize_t d2d_delete_foot_log_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                         uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (len != sizeof(uint8_t) || offset != 0)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t id_to_delete;
    memcpy(&id_to_delete, buf, sizeof(uint8_t));

    generic_message_t delete_msg;
    delete_msg.sender = SENDER_BTH; // Use BTH sender as we're acting on behalf of phone
    delete_msg.type = MSG_TYPE_DELETE_FOOT_LOG;
    delete_msg.data.delete_cmd.type = RECORD_HARDWARE_FOOT_SENSOR;
    delete_msg.data.delete_cmd.id = id_to_delete;

    if (k_msgq_put(&data_msgq, &delete_msg, K_NO_WAIT) != 0)
    {
        LOG_ERR("Failed to send delete foot log message for ID %u.", id_to_delete);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    LOG_INF("D2D RX: Delete Foot Log Command - ID: %u", id_to_delete);
    return len;
}

// Delete BHI360 Log Command Handler - mirrors control_service.cpp implementation
static ssize_t d2d_delete_bhi360_log_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                           uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    if (len != sizeof(uint8_t) || offset != 0)
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t id_to_delete;
    memcpy(&id_to_delete, buf, sizeof(uint8_t));

    generic_message_t delete_msg;
    delete_msg.sender = SENDER_BTH; // Use BTH sender as we're acting on behalf of phone
    delete_msg.type = MSG_TYPE_DELETE_BHI360_LOG;
    delete_msg.data.delete_cmd.type = RECORD_HARDWARE_BHI360;
    delete_msg.data.delete_cmd.id = id_to_delete;

    if (k_msgq_put(&data_msgq, &delete_msg, K_NO_WAIT) != 0)
    {
        LOG_ERR("Failed to send delete BHI360 log message for ID %u.", id_to_delete);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    LOG_INF("D2D RX: Delete BHI360 Log Command - ID: %u", id_to_delete);
    return len;
}

// Start Activity Command Handler - mirrors control_service.cpp implementation
// Role: SECONDARY DEVICE - Receives activity start commands from primary device
static ssize_t d2d_start_activity_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                        uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    LOG_INF("D2D RX: Received start activity command write, len=%u", len);

    if (len != sizeof(uint8_t))
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t value = *((const uint8_t *)buf);
    LOG_INF("D2D RX: Start activity command value=%u", value);

    if (value == 1)
    {
        // Start sensor events (same as primary device)
        struct foot_sensor_start_activity_event *foot_evt = new_foot_sensor_start_activity_event();
        APP_EVENT_SUBMIT(foot_evt);

        struct motion_sensor_start_activity_event *motion_evt = new_motion_sensor_start_activity_event();
        APP_EVENT_SUBMIT(motion_evt);

        // Start Logging Activity file (mirrors primary device behavior from control_service.cpp)
        generic_message_t start_logging_msg = {};
        start_logging_msg.sender = SENDER_BTH;
        start_logging_msg.type = MSG_TYPE_COMMAND;
        strncpy(start_logging_msg.data.command_str, "START_LOGGING_ACTIVITY",
                sizeof(start_logging_msg.data.command_str) - 1);
        start_logging_msg.data.command_str[sizeof(start_logging_msg.data.command_str) - 1] = '\0';

        if (k_msgq_put(&data_msgq, &start_logging_msg, K_NO_WAIT) != 0) {
            LOG_WRN("Failed to send START_LOGGING_ACTIVITY command to DATA module");
        } else {
            LOG_INF("Sent START_LOGGING_ACTIVITY command to DATA module");
        }

        LOG_INF("D2D RX: Start Activity Command - submitted foot and motion sensor events, started activity logging");
    }
    else
    {
        LOG_WRN("D2D RX: Start activity command ignored (value=%u)", value);
    }

    return len;
}

// Stop Activity Command Handler - mirrors control_service.cpp implementation
// Role: SECONDARY DEVICE - Receives activity stop commands from primary device
static ssize_t d2d_stop_activity_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                       uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    if (len != sizeof(uint8_t))
    {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t value = *((const uint8_t *)buf);

    if (value == 1)
    {
        // Stop sensor events (same as primary device)
        struct foot_sensor_stop_activity_event *foot_evt = new_foot_sensor_stop_activity_event();
        APP_EVENT_SUBMIT(foot_evt);

        struct motion_sensor_stop_activity_event *motion_evt = new_motion_sensor_stop_activity_event();
        APP_EVENT_SUBMIT(motion_evt);

        // Stop Logging Activity file (mirrors primary device behavior from control_service.cpp)
        generic_message_t stop_logging_msg = {};
        stop_logging_msg.sender = SENDER_BTH;
        stop_logging_msg.type = MSG_TYPE_COMMAND;
        strncpy(stop_logging_msg.data.command_str, "STOP_LOGGING_ACTIVITY",
                sizeof(stop_logging_msg.data.command_str) - 1);
        stop_logging_msg.data.command_str[sizeof(stop_logging_msg.data.command_str) - 1] = '\0';

        if (k_msgq_put(&data_msgq, &stop_logging_msg, K_NO_WAIT) != 0) {
            LOG_WRN("Failed to send STOP_LOGGING_ACTIVITY command to DATA module");
        } else {
            LOG_INF("Sent STOP_LOGGING_ACTIVITY command to DATA module");
        }

        LOG_INF("D2D RX: Stop Activity Command - submitted events and stopped activity logging");
    }
    else
    {
        LOG_WRN("D2D RX: Stop activity command ignored (value=%u)", value);
    }

    return len;
}

// Role: SECONDARY DEVICE - Receives erase flash commands from primary device
static ssize_t d2d_erase_flash_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                                     uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    if (len != sizeof(uint8_t))
    {
        LOG_ERR("D2D RX: Invalid erase flash command length");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    uint8_t value = *((const uint8_t *)buf);
    LOG_INF("D2D RX: Received erase flash command from primary: %u", value);

    if (value == 1)
    {
        // Send message to data module to trigger flash erase
        generic_message_t msg = {};
        msg.sender = SENDER_BTH;
        msg.type = MSG_TYPE_ERASE_EXTERNAL_FLASH;
        
        if (k_msgq_put(&data_msgq, &msg, K_NO_WAIT) != 0)
        {
            LOG_ERR("D2D RX: Failed to send erase flash command to DATA module");
            return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
        }
        
        LOG_INF("D2D RX: Forwarded erase flash command to DATA module");
    }

    return len;
}

// Trigger Calibration Handler - handles both BHI360 and weight calibration triggers
// This characteristic is shared for both calibration types as they use the same trigger mechanism
static ssize_t d2d_trigger_bhi360_calibration_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    // Check if this is a simple trigger (1 byte) or weight calibration data
    if (len == sizeof(uint8_t))
    {
        // Legacy mode - simple trigger
        uint8_t value = *((const uint8_t *)buf);

        if (value == 1)
        {
            // Value 1: Trigger calibration (both BHI360 and weight)
            // Send message to motion sensor to trigger BHI360 calibration
            generic_message_t calib_msg = {};
            calib_msg.sender = SENDER_BTH;
            calib_msg.type = MSG_TYPE_TRIGGER_BHI360_CALIBRATION;

            if (k_msgq_put(&motion_sensor_msgq, &calib_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to send BHI360 calibration trigger to motion sensor");
                return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
            }

            LOG_INF("D2D RX: Trigger BHI360 Calibration Command - sent to motion sensor");

            // Also send weight calibration trigger to activity metrics (without known weight)
            generic_message_t weight_calib_msg = {};
            weight_calib_msg.sender = SENDER_BTH;
            weight_calib_msg.type = MSG_TYPE_START_WEIGHT_CALIBRATION;

            if (k_msgq_put(&activity_metrics_msgq, &weight_calib_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to send weight calibration trigger to activity metrics");
                // Don't return error as BHI360 calibration was already sent
            }
            else
            {
                LOG_INF("D2D RX: Weight Calibration Command - sent to activity metrics");
            }
        }
        else
        {
            LOG_ERR("D2D RX: Invalid calibration trigger length: %u", len);
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }

        return len;
    }
}

    // FOTA Status Handler - receives completion status from secondary
    static ssize_t d2d_fota_status_write(struct bt_conn * conn, const struct bt_gatt_attr *attr, const void *buf,
                                         uint16_t len, uint16_t offset, uint8_t flags)
    {
        ARG_UNUSED(conn);
        ARG_UNUSED(attr);
        ARG_UNUSED(offset);
        ARG_UNUSED(flags);

        if (len != sizeof(uint8_t))
        {
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }

        uint8_t status = *((const uint8_t *)buf);

        if (status == 1)
        { // 1 = complete
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
    static ssize_t d2d_delete_activity_log_write(struct bt_conn * conn, const struct bt_gatt_attr *attr,
                                                 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
    {
        ARG_UNUSED(conn);
        ARG_UNUSED(attr);
        ARG_UNUSED(flags);

        if (len != sizeof(uint8_t) || offset != 0)
        {
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }

        uint8_t id_to_delete;
        memcpy(&id_to_delete, buf, sizeof(uint8_t));

        generic_message_t delete_msg;
        delete_msg.sender = SENDER_BTH; // Use BTH sender as we're acting on behalf of phone
        delete_msg.type = MSG_TYPE_DELETE_ACTIVITY_LOG;
        delete_msg.data.delete_cmd.type = RECORD_HARDWARE_ACTIVITY;
        delete_msg.data.delete_cmd.id = id_to_delete;

        if (k_msgq_put(&data_msgq, &delete_msg, K_NO_WAIT) != 0)
        {
            LOG_ERR("Failed to send delete activity log message for ID %u.", id_to_delete);
            return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
        }

        LOG_INF("D2D RX: Delete Activity Log Command - ID: %u", id_to_delete);
        return len;
    }

    // Write handler for request device info command
    static ssize_t d2d_request_device_info_write(struct bt_conn * conn, const struct bt_gatt_attr *attr,
                                                 const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
    {
        ARG_UNUSED(conn);
        ARG_UNUSED(attr);
        ARG_UNUSED(offset);
        ARG_UNUSED(flags);

        if (len != sizeof(uint8_t))
        {
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }

        uint8_t value = *((const uint8_t *)buf);

        if (value == 1)
        {
            LOG_INF("Received request for device info from primary");

#if !IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            // Send device info immediately (only on secondary device)
            extern void send_device_info_to_primary(void);
            send_device_info_to_primary();
#else
            // Primary device shouldn't receive this request
            LOG_WRN("Primary device received device info request - ignoring");
#endif
        }

        return len;
    }

    // Write handler for weight measurement trigger command
    static ssize_t d2d_weight_measurement_trigger_write(struct bt_conn * conn, const struct bt_gatt_attr *attr,
                                                        const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
    {
        ARG_UNUSED(conn);
        ARG_UNUSED(attr);
        ARG_UNUSED(offset);
        ARG_UNUSED(flags);

        if (len != sizeof(uint8_t))
        {
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }

        uint8_t value = *((const uint8_t *)buf);
        LOG_INF("D2D RX: Weight measurement trigger command value=%u", value);

        if (value == 2)
        {
            // Value 2: Trigger weight measurement only
            generic_message_t weight_msg = {};
            weight_msg.sender = SENDER_BTH;
            weight_msg.type = MSG_TYPE_WEIGHT_MEASUREMENT;

            if (k_msgq_put(&activity_metrics_msgq, &weight_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to send weight measurement trigger to activity metrics");
                return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
            }
            else
            {
                LOG_INF("D2D RX: Weight Measurement Command - sent to activity metrics");
            }
        }
        else
        {
            LOG_WRN("D2D RX: Weight measurement trigger command ignored (value=%u)", value);
        }

        return len;
    }

    // Write handler for GPS update command
    static ssize_t d2d_gps_update_write(struct bt_conn * conn, const struct bt_gatt_attr *attr, const void *buf,
                                        uint16_t len, uint16_t offset, uint8_t flags)
    {
        ARG_UNUSED(conn);
        ARG_UNUSED(attr);
        ARG_UNUSED(offset);
        ARG_UNUSED(flags);

        if (len != sizeof(GPSUpdateCommand))
        {
            LOG_ERR("D2D RX: Invalid GPS update length: %u (expected %zu)", len, sizeof(GPSUpdateCommand));
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }

        GPSUpdateCommand gps_data;
        memcpy(&gps_data, buf, sizeof(GPSUpdateCommand));

        LOG_INF("D2D RX: GPS update received - lat=%d, lon=%d, speed=%u cm/s", gps_data.latitude_e7,
                gps_data.longitude_e7, gps_data.speed_cms);

        // Send GPS update to activity metrics module
        generic_message_t gps_msg = {};
        gps_msg.sender = SENDER_BTH;
        gps_msg.type = MSG_TYPE_GPS_UPDATE;
        memcpy(&gps_msg.data.gps_update, &gps_data, sizeof(GPSUpdateCommand));

        if (k_msgq_put(&activity_metrics_msgq, &gps_msg, K_NO_WAIT) != 0)
        {
            LOG_ERR("Failed to send GPS update to activity metrics");
            return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
        }

        LOG_INF("D2D RX: GPS update sent to activity metrics");
        return len;
    }

    // Write handler for weight calibration command
    // This is a dedicated handler for weight calibration to match the D2D TX client expectations
    static ssize_t d2d_weight_calibration_write(struct bt_conn * conn, const struct bt_gatt_attr *attr, const void *buf,
                                                uint16_t len, uint16_t offset, uint8_t flags)
    {
        ARG_UNUSED(conn);
        ARG_UNUSED(attr);
        ARG_UNUSED(offset);
        ARG_UNUSED(flags);

        // This handler supports both simple trigger and calibration with known weight
        if (len == sizeof(uint8_t))
        {
            // Simple trigger mode
            uint8_t value = *((const uint8_t *)buf);

            if (value == 1)
            {
                // Trigger weight calibration without known weight
                generic_message_t weight_calib_msg = {};
                weight_calib_msg.sender = SENDER_BTH;
                weight_calib_msg.type = MSG_TYPE_START_WEIGHT_CALIBRATION;

                if (k_msgq_put(&activity_metrics_msgq, &weight_calib_msg, K_NO_WAIT) != 0)
                {
                    LOG_ERR("Failed to send weight calibration trigger to activity metrics");
                    return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
                }

                LOG_INF("D2D RX: Weight Calibration Command - sent to activity metrics");
            }
            else
            {
                LOG_WRN("D2D RX: Weight calibration command ignored (value=%u)", value);
            }
        }
        else if (len == sizeof(weight_calibration_step_t))
        {
            // Calibration with known weight
            weight_calibration_step_t *calib_data = (weight_calibration_step_t *)buf;
            LOG_INF("D2D RX: Weight calibration with known weight: %.1f kg", (double)calib_data->known_weight_kg);

            // Send weight calibration with known weight to activity metrics
            generic_message_t weight_calib_msg = {};
            weight_calib_msg.sender = SENDER_BTH;
            weight_calib_msg.type = MSG_TYPE_START_WEIGHT_CALIBRATION;
            memcpy(&weight_calib_msg.data.weight_calibration_step, calib_data, sizeof(weight_calibration_step_t));

            if (k_msgq_put(&activity_metrics_msgq, &weight_calib_msg, K_NO_WAIT) != 0)
            {
                LOG_ERR("Failed to send weight calibration data to activity metrics");
                return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
            }

            LOG_INF("D2D RX: Weight Calibration with known weight sent to activity metrics");
        }
        else
        {
            LOG_ERR("D2D RX: Invalid weight calibration length: %u", len);
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }

        return len;
    }

    BT_GATT_SERVICE_DEFINE(d2d_rx_service, BT_GATT_PRIMARY_SERVICE(&d2d_rx_service_uuid),
                           BT_GATT_CHARACTERISTIC(&d2d_set_time_command_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_set_time_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_delete_foot_log_command_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_delete_foot_log_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_delete_bhi360_log_command_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_delete_bhi360_log_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_delete_activity_log_command_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_delete_activity_log_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_start_activity_command_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_start_activity_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_stop_activity_command_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_stop_activity_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_erase_flash_command_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_erase_flash_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_fota_status_uuid.uuid, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE,
                                                  NULL, d2d_fota_status_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_trigger_bhi360_calibration_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_trigger_bhi360_calibration_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_request_device_info_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_request_device_info_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_gps_update_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_gps_update_write, NULL),
                           BT_GATT_CHARACTERISTIC(&d2d_weight_calibration_uuid.uuid, BT_GATT_CHRC_WRITE,
                                                  BT_GATT_PERM_WRITE, NULL, d2d_weight_calibration_write, NULL));

    void ble_d2d_rx_init(void)
    {
        // Nothing needed if using BT_GATT_SERVICE_DEFINE
    }

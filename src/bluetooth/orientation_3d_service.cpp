/**
 * @file orientation_3d_service.cpp
 * @brief High-rate 3D orientation service for real-time visualization
 * @version 1.0
 * @date 2024-12-20
 *
 * @copyright Botz Innovation 2024
 */

#define MODULE bluetooth

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <app.hpp>
#include <ble_services.hpp>

LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

// 3D Orientation packet structure (20 bytes)
typedef struct __attribute__((packed)) {
    uint16_t delta_time_ms;         // 0-1: Time since last packet
    
    // Left shoe quaternion (scaled to int16)
    int16_t left_quat_w;            // 2-3: w × 16384
    int16_t left_quat_x;            // 4-5: x × 16384
    int16_t left_quat_y;            // 6-7: y × 16384
    int16_t left_quat_z;            // 8-9: z × 16384
    
    // Right shoe quaternion (scaled to int16)
    int16_t right_quat_w;           // 10-11: w × 16384
    int16_t right_quat_x;           // 12-13: x × 16384
    int16_t right_quat_y;           // 14-15: y × 16384
    int16_t right_quat_z;           // 16-17: z × 16384
    
    // Status flags
    uint8_t left_contact;           // 18: 0=air, 1=ground
    uint8_t right_contact;          // 19: 0=air, 1=ground
} orientation_3d_packet_t;

// Static variables
static orientation_3d_packet_t orientation_packet = {
    .delta_time_ms = 0,
    .left_quat_w = 0,
    .left_quat_x = 0,
    .left_quat_y = 0,
    .left_quat_z = 0,
    .right_quat_w = 0,
    .right_quat_x = 0,
    .right_quat_y = 0,
    .right_quat_z = 0,
    .left_contact = 0,
    .right_contact = 0
};
static bool orientation_3d_subscribed = false;
static uint32_t last_packet_time_ms = 0;

// Latest quaternion data from both shoes
static struct {
    float quat_w, quat_x, quat_y, quat_z;
    bool valid;
    uint32_t timestamp;
} left_shoe_data = {1.0f, 0.0f, 0.0f, 0.0f, false, 0};

static struct {
    float quat_w, quat_x, quat_y, quat_z;
    bool valid;
    uint32_t timestamp;
} right_shoe_data = {1.0f, 0.0f, 0.0f, 0.0f, false, 0};

// Forward declarations
static ssize_t orientation_3d_read(struct bt_conn* conn, const struct bt_gatt_attr* attr, 
                                  void* buf, uint16_t len, uint16_t offset);
static void orientation_3d_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value);
extern "C" void orientation_3d_try_send(void);

// UUID for 3D Orientation Service and Characteristic
static struct bt_uuid_128 ORIENTATION_3D_SERVICE_UUID =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec0, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

static struct bt_uuid_128 ORIENTATION_3D_CHAR_UUID =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0c372ec1, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97));

// Service definition
BT_GATT_SERVICE_DEFINE(
    orientation_3d_service,
    BT_GATT_PRIMARY_SERVICE(&ORIENTATION_3D_SERVICE_UUID),
    
    // 3D Orientation Characteristic (Read + Notify)
    BT_GATT_CHARACTERISTIC(&ORIENTATION_3D_CHAR_UUID.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ_ENCRYPT,
                          orientation_3d_read, nullptr,
                          &orientation_packet),
    BT_GATT_CCC(orientation_3d_ccc_cfg_changed, 
                BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT)
);

/**
 * @brief Read callback for 3D orientation characteristic
 */
static ssize_t orientation_3d_read(struct bt_conn* conn, const struct bt_gatt_attr* attr,
                                  void* buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, 
                            &orientation_packet, sizeof(orientation_packet));
}

/**
 * @brief CCC configuration change callback
 */
static void orientation_3d_ccc_cfg_changed(const struct bt_gatt_attr* attr, uint16_t value)
{
    ARG_UNUSED(attr);
    orientation_3d_subscribed = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("3D Orientation notifications %s", 
            orientation_3d_subscribed ? "enabled" : "disabled");
}

/**
 * @brief Update quaternion data for primary (right) shoe
 * 
 * This is called directly from motion sensor at high rate (50Hz) when logging is NOT active.
 * When logging is active, quaternion data is sent through the regular logging channels
 * to avoid redundancy. We downsample to ~20Hz for BLE transmission.
 */
extern "C" void orientation_3d_update_primary(float quat_w, float quat_x, float quat_y, float quat_z)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Update right shoe data (primary is right shoe)
    right_shoe_data.quat_w = quat_w;
    right_shoe_data.quat_x = quat_x;
    right_shoe_data.quat_y = quat_y;
    right_shoe_data.quat_z = quat_z;
    right_shoe_data.valid = true;
    right_shoe_data.timestamp = k_uptime_get_32();
    
    // Try to send combined packet if we have data from both shoes
    orientation_3d_try_send();
#else
    // Secondary device (left shoe) - store locally
    left_shoe_data.quat_w = quat_w;
    left_shoe_data.quat_x = quat_x;
    left_shoe_data.quat_y = quat_y;
    left_shoe_data.quat_z = quat_z;
    left_shoe_data.valid = true;
    left_shoe_data.timestamp = k_uptime_get_32();
#endif
}

/**
 * @brief Update quaternion data from secondary (left) shoe via D2D
 * 
 * This is called when primary receives data from secondary
 */
extern "C" void orientation_3d_update_secondary(float quat_w, float quat_x, float quat_y, float quat_z)
{
    // Update left shoe data (secondary is left shoe)
    left_shoe_data.quat_w = quat_w;
    left_shoe_data.quat_x = quat_x;
    left_shoe_data.quat_y = quat_y;
    left_shoe_data.quat_z = quat_z;
    left_shoe_data.valid = true;
    left_shoe_data.timestamp = k_uptime_get_32();
    
    // Try to send combined packet
    orientation_3d_try_send();
}

/**
 * @brief Try to send orientation packet if conditions are met
 * 
 * Conditions:
 * 1. Notifications are enabled
 * 2. Enough time has passed since last packet (50ms = 20Hz)
 * 3. We have valid data from at least one shoe
 */
extern "C" void orientation_3d_try_send(void)
{
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!orientation_3d_subscribed) {
        return;
    }
    
    uint32_t current_time = k_uptime_get_32();
    uint32_t time_since_last = current_time - last_packet_time_ms;
    
    // Rate limit for BLE transmission
    // With 50Hz input from BHI360, we'll send at 20Hz to BLE
    const uint32_t min_interval_ms = 50;  // 20Hz BLE updates
    
    if (time_since_last < min_interval_ms) {
        return;
    }
    
    // Need at least one valid shoe
    if (!left_shoe_data.valid && !right_shoe_data.valid) {
        return;
    }
    
    // Calculate delta time
    orientation_packet.delta_time_ms = (uint16_t)time_since_last;
    
    // Convert quaternions to fixed point
    if (right_shoe_data.valid) {
        orientation_packet.right_quat_w = (int16_t)(right_shoe_data.quat_w * 16384.0f);
        orientation_packet.right_quat_x = (int16_t)(right_shoe_data.quat_x * 16384.0f);
        orientation_packet.right_quat_y = (int16_t)(right_shoe_data.quat_y * 16384.0f);
        orientation_packet.right_quat_z = (int16_t)(right_shoe_data.quat_z * 16384.0f);
    } else {
        // No data - send identity quaternion
        orientation_packet.right_quat_w = 16384;  // 1.0
        orientation_packet.right_quat_x = 0;
        orientation_packet.right_quat_y = 0;
        orientation_packet.right_quat_z = 0;
    }
    
    if (left_shoe_data.valid) {
        orientation_packet.left_quat_w = (int16_t)(left_shoe_data.quat_w * 16384.0f);
        orientation_packet.left_quat_x = (int16_t)(left_shoe_data.quat_x * 16384.0f);
        orientation_packet.left_quat_y = (int16_t)(left_shoe_data.quat_y * 16384.0f);
        orientation_packet.left_quat_z = (int16_t)(left_shoe_data.quat_z * 16384.0f);
    } else {
        // No data - send identity quaternion
        orientation_packet.left_quat_w = 16384;  // 1.0
        orientation_packet.left_quat_x = 0;
        orientation_packet.left_quat_y = 0;
        orientation_packet.left_quat_z = 0;
    }
    
    // TODO: Add ground contact detection from pressure sensors
    orientation_packet.left_contact = 0;
    orientation_packet.right_contact = 0;
    
    // Send notification
    auto *attr = bt_gatt_find_by_uuid(orientation_3d_service.attrs, 
                                     orientation_3d_service.attr_count, 
                                     &ORIENTATION_3D_CHAR_UUID.uuid);
    if (attr) {
        int err = bt_gatt_notify(nullptr, attr, &orientation_packet, sizeof(orientation_packet));
        if (err) {
            LOG_WRN("Failed to send 3D orientation notification: %d", err);
        } else {
            last_packet_time_ms = current_time;
        }
    } else {
        LOG_WRN("3D Orientation GATT attribute not found, skipping notification");
    }
#endif
}

/**
 * @brief Get the 3D orientation service UUID string for advertising
 */
extern "C" const char* orientation_3d_get_service_uuid_str(void)
{
    static char uuid_str[37];  // UUID string format: "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
    snprintf(uuid_str, sizeof(uuid_str), 
             "%08x-%04x-%04x-%04x-%012llx",
             0x0c372ec0, 0x27eb, 0x437e, 0xbef4, 0x775aefaf3c97ULL);
    return uuid_str;
}
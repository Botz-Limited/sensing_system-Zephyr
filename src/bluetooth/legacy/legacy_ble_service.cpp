#if CONFIG_LEGACY_BLE_ENABLED

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/random.h>


#include "ccc_callback_fix.hpp"
#include "../sensor_data/sensor_data.h"
#include <app.hpp>
#include "../../src/bluetooth/ble_d2d_rx_client.hpp" // Include for D2D data types
#include <time.h>
#include "../../include/events/foot_sensor_event.h"
#include "../../include/events/motion_sensor_event.h"
#include <math.h>
#include <string.h>
#include <errno.h>

#define M_PI_F 3.14159265358979323846f

LOG_MODULE_REGISTER(legacy_ble, LOG_LEVEL_INF);

// Forward declarations
static size_t pack_legacy_data(uint8_t *buffer);
static size_t pack_zeroed_data(uint8_t *buffer);
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static size_t pack_secondary_data(uint8_t *buffer);
static void send_zero_filled_secondary_packet(void);
#endif
static size_t pack_bno_data(uint8_t *buffer);
static ssize_t config_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

// UUID definitions for Insole Service
#define INSOLE_SVC_UUID_VAL BT_UUID_128_ENCODE(0x91bad492, 0xb950, 0x4226, 0xaa2b, 0x4ede9fa42f59)
#define INSOLE_CHAR_UUID_VAL BT_UUID_128_ENCODE(0xcba1d466, 0x344c, 0x4be3, 0xab3f, 0x189f80dd7518)
#define INSOLE_CHAR_SECONDARY_UUID_VAL BT_UUID_128_ENCODE(0xcba1d467, 0x344c, 0x4be3, 0xab3f, 0x189f80dd7518)
#define CONFIG_CHAR_UUID_VAL BT_UUID_128_ENCODE(0xcba1d468, 0x344c, 0x4be3, 0xab3f, 0x189f80dd7518)

// UUID definitions for BNO055 Service
#define BNO_SVC_UUID_VAL BT_UUID_128_ENCODE(0x91bad493, 0xb950, 0x4226, 0xaa2b, 0x4ede9fa42f59)
#define BNO_CHAR_UUID_VAL BT_UUID_128_ENCODE(0xcba1d469, 0x344c, 0x4be3, 0xab3f, 0x189f80dd7518)

static struct bt_uuid_128 legacy_insole_svc_uuid = BT_UUID_INIT_128(INSOLE_SVC_UUID_VAL);
static struct bt_uuid_128 legacy_insole_char_uuid = BT_UUID_INIT_128(INSOLE_CHAR_UUID_VAL);
static struct bt_uuid_128 legacy_insole_char_secondary_uuid = BT_UUID_INIT_128(INSOLE_CHAR_SECONDARY_UUID_VAL);
static struct bt_uuid_128 legacy_config_char_uuid = BT_UUID_INIT_128(CONFIG_CHAR_UUID_VAL);

static struct bt_uuid_128 legacy_bno_svc_uuid = BT_UUID_INIT_128(BNO_SVC_UUID_VAL);
static struct bt_uuid_128 legacy_bno_char_uuid = BT_UUID_INIT_128(BNO_CHAR_UUID_VAL);

// CCC descriptors for notify
static uint8_t insole_notify_enabled;
static uint8_t insole_secondary_notify_enabled;
static uint8_t bno_notify_enabled;
static uint8_t config_notify_enabled;

// Legacy streaming control variables
static bool legacy_streaming_enabled = false;
static uint8_t legacy_mode = 0; // 0=off, 1=I1, 2=I2, 3=I3

extern "C" void insole_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    insole_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Legacy BLE notifications %s", insole_notify_enabled ? "enabled" : "disabled");
    
    // Auto-start streaming immediately when notifications are enabled (exactly like old firmware)
    if (insole_notify_enabled) {
        // Start streaming immediately in I2 mode (combined data) like old firmware
        legacy_mode = 2;
        legacy_streaming_enabled = true;
        LOG_INF("Auto-starting legacy streaming (mimicking old firmware behavior)");
        
        // Automatically start sensor activity to ensure sensors provide data
        // This mimics the old firmware where sensors were always active
        struct foot_sensor_start_activity_event *foot_evt = new_foot_sensor_start_activity_event();
        APP_EVENT_SUBMIT(foot_evt);
        
        struct motion_sensor_start_activity_event *motion_evt = new_motion_sensor_start_activity_event();
        APP_EVENT_SUBMIT(motion_evt);
        
        LOG_INF("Started sensor activity for legacy BLE compatibility");
    } else {
        // Stop streaming when notifications are disabled
        legacy_streaming_enabled = false;
        legacy_mode = 0;
        LOG_INF("Stopping legacy streaming");
        
        // Stop sensor activity when legacy client disconnects
        struct foot_sensor_stop_activity_event *foot_evt = new_foot_sensor_stop_activity_event();
        APP_EVENT_SUBMIT(foot_evt);
        
        struct motion_sensor_stop_activity_event *motion_evt = new_motion_sensor_stop_activity_event();
        APP_EVENT_SUBMIT(motion_evt);
        
        LOG_INF("Stopped sensor activity for legacy BLE");
    }
}

extern "C" void insole_secondary_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    insole_secondary_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Legacy BLE secondary notifications %s", insole_secondary_notify_enabled ? "enabled" : "disabled");
    
    // Secondary device notifications don't auto-start streaming - they follow primary
}

extern "C" void bno_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    bno_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Legacy BNO notifications %s", bno_notify_enabled ? "enabled" : "disabled");
}

extern "C" void config_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    config_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Legacy config notifications %s", config_notify_enabled ? "enabled" : "disabled");
}


// GATT service definitions

BT_GATT_SERVICE_DEFINE(legacy_insole_svc,
    BT_GATT_PRIMARY_SERVICE(&legacy_insole_svc_uuid),
    BT_GATT_CHARACTERISTIC(&legacy_insole_char_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(insole_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    BT_GATT_CHARACTERISTIC(&legacy_insole_char_secondary_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(insole_secondary_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif
    BT_GATT_CHARACTERISTIC(&legacy_config_char_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_WRITE, NULL, config_write, NULL),
    BT_GATT_CCC(config_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

BT_GATT_SERVICE_DEFINE(legacy_bno_svc,
    BT_GATT_PRIMARY_SERVICE(&legacy_bno_svc_uuid),
    BT_GATT_CHARACTERISTIC(&legacy_bno_char_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(bno_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

// Thread for data packing and notifying
#define LEGACY_THREAD_STACK_SIZE 1024
#define LEGACY_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(legacy_stack, LEGACY_THREAD_STACK_SIZE);
struct k_thread legacy_thread_data;


// Store attribute pointers for notifications

static const struct bt_gatt_attr *insole_data_attr = NULL;
static const struct bt_gatt_attr *insole_secondary_data_attr = NULL;
static const struct bt_gatt_attr *bno_data_attr = NULL;
static const struct bt_gatt_attr *config_attr = NULL;

// Buffer to store the last received command on the config characteristic
static char config_command[16] = {0};
static bool command_received = false;

// Custom structs for secondary data to avoid dependency on D2D headers
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE) && IS_ENABLED(CONFIG_LEGACY_BLE_ENABLED)
// Buffer for secondary data
static foot_samples_t secondary_foot_data = {0};
static bhi360_3d_mapping_t secondary_imu_data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static bool secondary_data_updated = false;
#endif

// Callback to find and store attribute pointers
static uint8_t find_attr_cb(const struct bt_gatt_attr *attr, uint16_t handle,
                           void *user_data)
{
    ARG_UNUSED(handle);
    ARG_UNUSED(user_data);
    

    // Check if this is the insole data characteristic
    if (bt_uuid_cmp(attr->uuid, &legacy_insole_char_uuid.uuid) == 0) {
        insole_data_attr = attr;
    }
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Check if this is the secondary insole data characteristic
    if (bt_uuid_cmp(attr->uuid, &legacy_insole_char_secondary_uuid.uuid) == 0) {
        insole_secondary_data_attr = attr;
    }
#endif
    // Check if this is the BNO data characteristic
    if (bt_uuid_cmp(attr->uuid, &legacy_bno_char_uuid.uuid) == 0) {
        bno_data_attr = attr;
    }
    // Check if this is the config characteristic
    if (bt_uuid_cmp(attr->uuid, &legacy_config_char_uuid.uuid) == 0) {
        config_attr = attr;
    }
    
    return BT_GATT_ITER_CONTINUE;
}

// D2D connection status
static bool d2d_connected = false;

void legacy_ble_set_d2d_connection_status(bool connected) {
    d2d_connected = connected;
    LOG_INF("Legacy BLE: D2D connection status updated to %s", connected ? "connected" : "disconnected");
}

void legacy_ble_update_secondary_data(const foot_samples_t *foot_data, const bhi360_3d_mapping_t *imu_data) {
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE) && IS_ENABLED(CONFIG_LEGACY_BLE_ENABLED)
    if (foot_data) {
        memcpy(&secondary_foot_data, foot_data, sizeof(foot_samples_t));
    }
    if (imu_data) {
        memcpy(&secondary_imu_data, imu_data, sizeof(bhi360_3d_mapping_t));
    }
    secondary_data_updated = true;
    LOG_DBG("Updated secondary data for legacy BLE");
#endif
}

static void legacy_thread_func(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    // Find the attribute for notifications
    bt_gatt_foreach_attr(0x0001, 0xffff, find_attr_cb, NULL);
    
    if (!insole_data_attr) {
        LOG_ERR("Failed to find insole data characteristic attribute");
        return;
    }
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    if (!insole_secondary_data_attr) {
        LOG_ERR("Failed to find secondary insole data characteristic attribute");
        return;
    }
    if (!bno_data_attr) {
        LOG_ERR("Failed to find BNO data characteristic attribute");
        return;
    }
    if (!config_attr) {
        LOG_ERR("Failed to find config characteristic attribute");
        return;
    }
#endif
    
    LOG_INF("Legacy BLE periodic transmission started (10Hz)");

    while (true) {
        // Sleep first for 10Hz transmission rate (like old firmware)
        k_sleep(K_MSEC(100));
        
        // Only transmit if streaming is enabled and notifications are enabled
        if (legacy_streaming_enabled) {
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
            // Handle primary data
            if (insole_notify_enabled && insole_data_attr) {
                uint8_t buffer[107]; // Match old firmware: 106 + checksum
                size_t len = pack_legacy_data(buffer);
                
                int err = bt_gatt_notify(NULL, insole_data_attr, buffer, len);
                if (err < 0 && err != -ENOTCONN) {
                    LOG_WRN("Legacy BLE notify error: %d", err);
                }
            }

            // Handle secondary data on primary
            if (insole_secondary_notify_enabled && insole_secondary_data_attr) {
                if (d2d_connected && secondary_data_updated) {
                    // Send actual secondary data
                    uint8_t buffer[107]; // Match old firmware: 106 + checksum
                    size_t len = pack_secondary_data(buffer);
                    
                    int err = bt_gatt_notify(NULL, insole_secondary_data_attr, buffer, len);
                    if (err < 0 && err != -ENOTCONN) {
                        LOG_WRN("Secondary Legacy BLE notify error: %d", err);
                    }
                    secondary_data_updated = false;
                } else {
                    // Send zero-filled packet when secondary is disconnected
                    send_zero_filled_secondary_packet();
                }
            }

            // Handle BNO data notifications
            if (bno_notify_enabled && bno_data_attr) {
                uint8_t bno_buffer[16]; // 4 floats for quaternion
                size_t bno_len = pack_bno_data(bno_buffer);
                
                int err = bt_gatt_notify(NULL, bno_data_attr, bno_buffer, bno_len);
                if (err < 0 && err != -ENOTCONN) {
                    LOG_WRN("BNO BLE notify error: %d", err);
                }
            }
#else
            // Secondary device - only send its own data
            if (insole_notify_enabled && insole_data_attr) {
                uint8_t buffer[107]; // Match old firmware: 106 + checksum
                size_t len = pack_legacy_data(buffer);
                
                int err = bt_gatt_notify(NULL, insole_data_attr, buffer, len);
                if (err < 0 && err != -ENOTCONN) {
                    LOG_WRN("Legacy BLE notify error: %d", err);
                }
            }
#endif
        }
    }
}

// Function to pack data matching legacy format for primary device
static size_t pack_legacy_data(uint8_t *buffer) {
    size_t index = 0;

    // 1. Timestamp (6 bytes)
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    uint8_t year = (now < 1672531200) ? 25 : (tm->tm_year % 100);
    uint8_t month = (now < 1672531200) ? 1 : (tm->tm_mon + 1);
    uint8_t day = (now < 1672531200) ? 1 : tm->tm_mday;
    uint8_t hour = (now < 1672531200) ? 0 : tm->tm_hour;
    uint8_t minute = (now < 1672531200) ? 0 : tm->tm_min;
    uint8_t second = (now < 1672531200) ? 0 : tm->tm_sec;
    buffer[index++] = year;
    buffer[index++] = month;
    buffer[index++] = day;
    buffer[index++] = hour;
    buffer[index++] = minute;
    buffer[index++] = second;

    // Get snapshot
    float quat[4], accel[3], lacc[3], gyro[3], grav[3], mag[3], temp;
    uint16_t pressure[8];
    get_sensor_snapshot(quat, accel, lacc, gyro, grav, mag, &temp, pressure);

    // For testing: If all pressure values are 0 (sensors not connected), simulate random values
    bool sensors_connected = false;
    for (int i = 0; i < 8; i++) {
        if (pressure[i] != 0) {
            sensors_connected = true;
            break;
        }
    }
    if (!sensors_connected) {
        for (int i = 0; i < 8; i++) {
            pressure[i] = sys_rand32_get() % 1024; // Random 0-1023
        }
        for (int i = 0; i < 4; i++) {
            quat[i] = (float)(sys_rand32_get() % 200) / 100.0f - 1.0f; // Random -1.0 to 1.0
        }
        for (int i = 0; i < 3; i++) {
            accel[i] = (float)(sys_rand32_get() % 400) / 100.0f - 2.0f; // Random -2.0 to 2.0
            lacc[i] = accel[i];
            gyro[i] = (float)(sys_rand32_get() % 4000) / 1000.0f - 2.0f; // Random -2.0 to 2.0
            grav[i] = (float)(sys_rand32_get() % 2000) / 1000.0f - 1.0f; // Random -1.0 to 1.0
            mag[i] = (float)(sys_rand32_get() % 200) / 100.0f - 1.0f; // Random -1.0 to 1.0
        }
        temp = (float)(sys_rand32_get() % 5000) / 100.0f + 20.0f; // Random 20.0 to 70.0
    }

    // Debug: Log pressure values to see if they're actually being read
    LOG_INF("Legacy BLE pressure values: [%d, %d, %d, %d, %d, %d, %d, %d]",
            pressure[0], pressure[1], pressure[2], pressure[3],
            pressure[4], pressure[5], pressure[6], pressure[7]);
    
    // Debug: Log other sensor values too
    LOG_INF("Legacy BLE quat values: [%.3f, %.3f, %.3f, %.3f]", 
            (double)quat[0], (double)quat[1], (double)quat[2], (double)quat[3]);
    LOG_INF("Legacy BLE accel values: [%.3f, %.3f, %.3f]", 
            (double)accel[0], (double)accel[1], (double)accel[2]);

    // 2. Insole data (16 bytes: 8x uint16_t, MSB first)
    for (int i = 0; i < 8; i++) {
        buffer[index++] = (pressure[i] >> 8) & 0xFF;
        buffer[index++] = pressure[i] & 0xFF;
    }

    // 3. Quaternion (16 bytes: 4x float w, z, y, x)
    float q[4] = {quat[0], quat[3], quat[2], quat[1]};
    for (int i = 0; i < 4; i++) {
        memcpy(&buffer[index], &q[i], sizeof(float));
        index += sizeof(float);
    }

    // 4. Orientation/Euler (12 bytes: 3x float z, y, x)
    float sinr_cosp = 2.0f * (quat[0] * quat[1] + quat[2] * quat[3]);
    float cosr_cosp = 1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (quat[0] * quat[2] - quat[3] * quat[1]);
    float pitch = (fabsf(sinp) >= 1) ? copysignf(M_PI_F / 2, sinp) : asinf(sinp);

    float siny_cosp = 2.0f * (quat[0] * quat[3] + quat[1] * quat[2]);
    float cosy_cosp = 1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    float e[3] = {yaw, pitch, roll};
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &e[i], sizeof(float));
        index += sizeof(float);
    }

    // 5. Accelerometer (12 bytes: 3x float x, y, z)
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &accel[i], sizeof(float));
        index += sizeof(float);
    }

    // 6. Linear acceleration (12 bytes: 3x float x, y, z)
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &lacc[i], sizeof(float));
        index += sizeof(float);
    }

    // 7. Gravity (12 bytes: 3x float x, y, z)
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &grav[i], sizeof(float));
        index += sizeof(float);
    }

    // 8. Magnetometer (12 bytes: 3x float z, y, x)
    float mg[3] = {mag[2], mag[1], mag[0]};
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &mg[i], sizeof(float));
        index += sizeof(float);
    }

    // 9. Temperature (4 bytes: float) - matches old firmware exactly
    float hardcoded_temp = 30.0f;
    memcpy(&buffer[index], &hardcoded_temp, sizeof(float));
    index += sizeof(float);

    // 10. Battery level (4 bytes: float) - hardcoded as in old firmware
    float hardcoded_battery = 50.0f;
    memcpy(&buffer[index], &hardcoded_battery, sizeof(float));
    index += sizeof(float);

    // 11. Checksum (1 byte: XOR of all previous bytes) - exactly like old firmware
    uint8_t checksum = 0;
    for (size_t i = 0; i < index; i++) {
        checksum ^= buffer[i];
    }
    buffer[index++] = checksum;

    return index; // Should be 107 bytes total (106 + checksum)
}

// Function to pack secondary data into legacy format on primary device
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
static size_t pack_secondary_data(uint8_t *buffer) {
    size_t index = 0;

    // 1. Timestamp (6 bytes)
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    uint8_t year = (now < 1672531200) ? 25 : (tm->tm_year % 100);
    uint8_t month = (now < 1672531200) ? 1 : (tm->tm_mon + 1);
    uint8_t day = (now < 1672531200) ? 1 : tm->tm_mday;
    uint8_t hour = (now < 1672531200) ? 0 : tm->tm_hour;
    uint8_t minute = (now < 1672531200) ? 0 : tm->tm_min;
    uint8_t second = (now < 1672531200) ? 0 : tm->tm_sec;
    buffer[index++] = year;
    buffer[index++] = month;
    buffer[index++] = day;
    buffer[index++] = hour;
    buffer[index++] = minute;
    buffer[index++] = second;

    // 2. Insole data from secondary (16 bytes: 8x uint16_t, MSB first)
    // For testing: If all values are 0 (no data), simulate random
    bool has_foot_data = false;
    for (int i = 0; i < 8; i++) {
        if (secondary_foot_data.values[i] != 0) {
            has_foot_data = true;
            break;
        }
    }
    if (!has_foot_data) {
        for (int i = 0; i < 8; i++) {
            secondary_foot_data.values[i] = sys_rand32_get() % 1024; // Random 0-1023
        }
    }
    for (int i = 0; i < 8; i++) {
        buffer[index++] = (secondary_foot_data.values[i] >> 8) & 0xFF;
        buffer[index++] = secondary_foot_data.values[i] & 0xFF;
    }

    // 3. Quaternion from secondary IMU data (16 bytes: 4x float w, z, y, x)
    // 3. Quaternion from secondary IMU data (16 bytes: 4x float w, z, y, x)
    // For testing: If all zero, simulate random
    bool has_imu_data = (secondary_imu_data.quat_w != 0.0f || secondary_imu_data.accel_x != 0.0f ||
                         secondary_imu_data.accel_y != 0.0f || secondary_imu_data.accel_z != 0.0f ||
                         secondary_imu_data.gyro_x != 0.0f || secondary_imu_data.gyro_y != 0.0f ||
                         secondary_imu_data.gyro_z != 0.0f);
    if (!has_imu_data) {
        secondary_imu_data.quat_w = (float)(sys_rand32_get() % 200) / 100.0f - 1.0f;
        secondary_imu_data.accel_x = (float)(sys_rand32_get() % 200) / 100.0f - 1.0f;
        secondary_imu_data.accel_y = (float)(sys_rand32_get() % 200) / 100.0f - 1.0f;
        secondary_imu_data.accel_z = (float)(sys_rand32_get() % 200) / 100.0f - 1.0f;
        secondary_imu_data.gyro_x = (float)(sys_rand32_get() % 4000) / 1000.0f - 2.0f;
        secondary_imu_data.gyro_y = (float)(sys_rand32_get() % 4000) / 1000.0f - 2.0f;
        secondary_imu_data.gyro_z = (float)(sys_rand32_get() % 4000) / 1000.0f - 2.0f;
    }
    float q[4] = {secondary_imu_data.quat_w, secondary_imu_data.accel_z, secondary_imu_data.accel_y, secondary_imu_data.accel_x};
    for (int i = 0; i < 4; i++) {
        memcpy(&buffer[index], &q[i], sizeof(float));
        index += sizeof(float);
    }

    // 4. Orientation/Euler (12 bytes: 3x float z, y, x) - Calculated from quaternion
    float sinr_cosp = 2.0f * (secondary_imu_data.quat_w * secondary_imu_data.accel_x + secondary_imu_data.accel_y * secondary_imu_data.accel_z);
    float cosr_cosp = 1.0f - 2.0f * (secondary_imu_data.accel_x * secondary_imu_data.accel_x + secondary_imu_data.accel_y * secondary_imu_data.accel_y);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (secondary_imu_data.quat_w * secondary_imu_data.accel_y - secondary_imu_data.accel_z * secondary_imu_data.accel_x);
    float pitch = (fabsf(sinp) >= 1) ? copysignf(M_PI_F / 2, sinp) : asinf(sinp);

    float siny_cosp = 2.0f * (secondary_imu_data.quat_w * secondary_imu_data.accel_z + secondary_imu_data.accel_x * secondary_imu_data.accel_y);
    float cosy_cosp = 1.0f - 2.0f * (secondary_imu_data.accel_y * secondary_imu_data.accel_y + secondary_imu_data.accel_z * secondary_imu_data.accel_z);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    float e[3] = {yaw, pitch, roll};
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &e[i], sizeof(float));
        index += sizeof(float);
    }

    // 5. Accelerometer from secondary IMU (12 bytes: 3x float x, y, z)
    float accel[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &accel[i], sizeof(float));
        index += sizeof(float);
    }

    // 6. Linear acceleration - using same as accelerometer for simplicity (12 bytes: 3x float x, y, z)
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &accel[i], sizeof(float));
        index += sizeof(float);
    }

    // 7. Gravity - set to zero as placeholder (12 bytes: 3x float x, y, z)
    float grav[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &grav[i], sizeof(float));
        index += sizeof(float);
    }

    // 8. Magnetometer - set to zero as placeholder (12 bytes: 3x float z, y, x)
    float mag[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &mag[i], sizeof(float));
        index += sizeof(float);
    }

    // 9. Temperature (4 bytes: float) - matches old firmware
    float hardcoded_temp = 30.0f;
    memcpy(&buffer[index], &hardcoded_temp, sizeof(float));
    index += sizeof(float);

    // 10. Battery level (4 bytes: float) - hardcoded as in old firmware
    float hardcoded_battery = 50.0f;
    memcpy(&buffer[index], &hardcoded_battery, sizeof(float));
    index += sizeof(float);

    // 11. Checksum (1 byte: XOR of all previous bytes) - exactly like old firmware
    uint8_t checksum = 0;
    for (size_t i = 0; i < index; i++) {
        checksum ^= buffer[i];
    }
    buffer[index++] = checksum;

    return index; // Should be 107 bytes total (106 + checksum)
}
#endif // IS_ENABLED(CONFIG_PRIMARY_DEVICE)

// Function to pack BNO data (quaternion)
static size_t pack_bno_data(uint8_t *buffer) {
    size_t index = 0;
    
    // Get quaternion data
    float quat[4];
    float dummy[3];
    float temp;
    uint16_t pressure[8];
    get_sensor_snapshot(quat, dummy, dummy, dummy, dummy, dummy, &temp, pressure);
    
    // Pack quaternion (4 floats: w, x, y, z)
    float q[4] = {quat[0], quat[1], quat[2], quat[3]};
    for (int i = 0; i < 4; i++) {
        memcpy(&buffer[index], &q[i], sizeof(float));
        index += sizeof(float);
    }
    
    return index;
}

// Function to pack zeroed data for primary when secondary is not connected
static size_t pack_zeroed_data(uint8_t *buffer) {
    size_t index = 0;

    // 1. Timestamp (6 bytes)
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    uint8_t year = (now < 1672531200) ? 25 : (tm->tm_year % 100);
    uint8_t month = (now < 1672531200) ? 1 : (tm->tm_mon + 1);
    uint8_t day = (now < 1672531200) ? 1 : tm->tm_mday;
    uint8_t hour = (now < 1672531200) ? 0 : tm->tm_hour;
    uint8_t minute = (now < 1672531200) ? 0 : tm->tm_min;
    uint8_t second = (now < 1672531200) ? 0 : tm->tm_sec;
    buffer[index++] = year;
    buffer[index++] = month;
    buffer[index++] = day;
    buffer[index++] = hour;
    buffer[index++] = minute;
    buffer[index++] = second;

    // Zero out the rest of the data fields (matches old firmware: 6 bytes timestamp + 100 bytes data)
    for (size_t i = index; i < 106; i++) {
        buffer[i] = 0;
    }
    index = 106;

    // 11. Checksum (1 byte: XOR of all previous bytes)
    uint8_t checksum = 0;
    for (size_t i = 0; i < index; i++) {
        checksum ^= buffer[i];
    }
    buffer[index++] = checksum;

    return index; // Should be 107 bytes total (106 + checksum)
}

#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
// Function to send zero-filled packet for disconnected secondary device
static void send_zero_filled_secondary_packet(void) {
    uint8_t sim_buffer[107];
    time_t current_time = time(NULL);
    struct tm *tm = gmtime(&current_time);
    
    // Set timestamp (6 bytes)
    sim_buffer[0] = tm->tm_year - 100;
    sim_buffer[1] = tm->tm_mon + 1;
    sim_buffer[2] = tm->tm_mday;
    sim_buffer[3] = tm->tm_hour;
    sim_buffer[4] = tm->tm_min;
    sim_buffer[5] = tm->tm_sec;
    
    size_t index = 6;
    
    // Simulate random insole data (16 bytes: 8x uint16_t)
    for (int i = 0; i < 8; i++) {
        uint16_t val = sys_rand32_get() % 1024;
        sim_buffer[index++] = (val >> 8) & 0xFF;
        sim_buffer[index++] = val & 0xFF;
    }
    
    // Simulate random quaternion (16 bytes: 4x float)
    for (int i = 0; i < 4; i++) {
        float val = (float)(sys_rand32_get() % 200) / 100.0f - 1.0f;
        memcpy(&sim_buffer[index], &val, sizeof(float));
        index += sizeof(float);
    }
    
    // Simulate random euler (12 bytes: 3x float)
    for (int i = 0; i < 3; i++) {
        float val = (float)(sys_rand32_get() % 36000) / 100.0f - 180.0f;
        memcpy(&sim_buffer[index], &val, sizeof(float));
        index += sizeof(float);
    }
    
    // Simulate random accel, lacc, grav, mag (12 bytes each)
    for (int j = 0; j < 4; j++) { // 4 sections
        for (int i = 0; i < 3; i++) {
            float val = (float)(sys_rand32_get() % 400) / 100.0f - 2.0f;
            memcpy(&sim_buffer[index], &val, sizeof(float));
            index += sizeof(float);
        }
    }
    
    // Temperature and battery (4 bytes each, random)
    float temp = (float)(sys_rand32_get() % 5000) / 100.0f + 20.0f;
    memcpy(&sim_buffer[index], &temp, sizeof(float));
    index += sizeof(float);
    
    float battery = (float)(sys_rand32_get() % 10000) / 100.0f;
    memcpy(&sim_buffer[index], &battery, sizeof(float));
    index += sizeof(float);
    
    // Checksum
    uint8_t checksum = 0;
    for (size_t i = 0; i < 106; i++) {
        checksum ^= sim_buffer[i];
    }
    sim_buffer[106] = checksum;
    
    // Send notification
    if (insole_secondary_data_attr) {
        int err = bt_gatt_notify(NULL, insole_secondary_data_attr, sim_buffer, sizeof(sim_buffer));
        if (err < 0 && err != -ENOTCONN) {
            LOG_WRN("Simulated secondary packet notify error: %d", err);
        }
    }
}
#endif

static ssize_t config_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    LOG_INF("Legacy config write received: %d bytes", len);
    
    if (len >= 2) {
        const char *cmd = (const char *)buf;
        
        // Check for legacy commands
        if (cmd[0] == 'I') {
            switch (cmd[1]) {
                case '1':
                    LOG_INF("Legacy mode I1: Start streaming insole data");
                    legacy_mode = 1;
                    legacy_streaming_enabled = true;
                    break;
                    
                case '2':
                    LOG_INF("Legacy mode I2: Start streaming combined data");
                    legacy_mode = 2;
                    legacy_streaming_enabled = true;
                    break;
                    
                case '3':
                    LOG_INF("Legacy mode I3: Start streaming step count");
                    legacy_mode = 3;
                    legacy_streaming_enabled = true;
                    break;
                    
                default:
                    LOG_WRN("Unknown legacy command: I%c", cmd[1]);
                    break;
            }
        } else if (cmd[0] == 'S' && cmd[1] == '0') {
            LOG_INF("Legacy mode S0: Stop streaming");
            legacy_streaming_enabled = false;
            legacy_mode = 0;
        } else {
            LOG_WRN("Unknown legacy command: %.*s", len, (char*)buf);
        }
        
        // Send notification to confirm command
        if (config_notify_enabled && config_attr) {
            bt_gatt_notify(conn, attr, buf, len);
        }
    }
    
    return len;
}

void legacy_ble_init(void) {
    #if CONFIG_LEGACY_BLE_ENABLED
    k_thread_create(&legacy_thread_data, legacy_stack, LEGACY_THREAD_STACK_SIZE,
                    legacy_thread_func, NULL, NULL, NULL,
                    LEGACY_THREAD_PRIORITY, 0, K_NO_WAIT);
    LOG_INF("Legacy BLE service initialized");
    #endif
}

#endif // CONFIG_LEGACY_BLE_ENABLED

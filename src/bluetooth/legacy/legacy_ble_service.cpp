#if CONFIG_LEGACY_BLE_ENABLED

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/sys/util.h>

#include "ccc_callback_fix.hpp"
#include "../sensor_data/sensor_data.h"
#include <time.h>
#include <math.h>
#include <string.h>
#include <errno.h>

#define M_PI_F 3.14159265358979323846f

LOG_MODULE_REGISTER(legacy_ble, LOG_LEVEL_INF);

// Forward declarations
static size_t pack_legacy_data(uint8_t *buffer);

// UUID definitions
#define INSOLE_SVC_UUID_VAL BT_UUID_128_ENCODE(0x91bad492, 0xb950, 0x4226, 0xaa2b, 0x4ede9fa42f59)
#define INSOLE_CHAR_UUID_VAL BT_UUID_128_ENCODE(0xcba1d466, 0x344c, 0x4be3, 0xab3f, 0x189f80dd7518)
#define CONFIG_CHAR_UUID_VAL BT_UUID_128_ENCODE(0x45678901, 0x1234, 0x5678, 0x1234, 0x56789abcdef3)
#define BNO_SVC_UUID_VAL BT_UUID_128_ENCODE(0x45678901, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)
#define BNO_CHAR_UUID_VAL BT_UUID_128_ENCODE(0x45678901, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

static struct bt_uuid_128 legacy_insole_svc_uuid = BT_UUID_INIT_128(INSOLE_SVC_UUID_VAL);
static struct bt_uuid_128 legacy_insole_char_uuid = BT_UUID_INIT_128(INSOLE_CHAR_UUID_VAL);
static struct bt_uuid_128 legacy_config_char_uuid = BT_UUID_INIT_128(CONFIG_CHAR_UUID_VAL);
static struct bt_uuid_128 legacy_bno_svc_uuid = BT_UUID_INIT_128(BNO_SVC_UUID_VAL);
static struct bt_uuid_128 legacy_bno_char_uuid = BT_UUID_INIT_128(BNO_CHAR_UUID_VAL);

// CCC descriptors for notify
static uint8_t insole_notify_enabled;
static uint8_t config_notify_enabled;
static uint8_t bno_notify_enabled;

extern "C" void insole_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    insole_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

extern "C" void config_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    config_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

extern "C" void bno_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    ARG_UNUSED(attr);
    bno_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

// Store the config characteristic attribute pointer for notifications
static const struct bt_gatt_attr *config_char_attr = NULL;

// Write callback for config char
extern "C" ssize_t on_config_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);

    if (len == 0) return 0;

    char command[32] = {0};
    size_t copy_len = (len < sizeof(command) - 1) ? len : sizeof(command) - 1;
    memcpy(command, buf, copy_len);

    LOG_INF("Config command: %s", command);

    // Support legacy commands I1, I2 in addition to START/STOP
    if (strcmp(command, "START") == 0 || strcmp(command, "I1") == 0 || strcmp(command, "I2") == 0) {
        insole_notify_enabled = 1;
        LOG_INF("Streaming started via command: %s", command);
        
        // Send notification on config characteristic to confirm command receipt
        if (conn && config_notify_enabled && attr) {
            const char *response = "OK";
            bt_gatt_notify(conn, attr, response, strlen(response));
        }
    } else if (strcmp(command, "STOP") == 0) {
        insole_notify_enabled = 0;
        LOG_INF("Streaming stopped");
        
        // Send notification on config characteristic to confirm command receipt
        if (conn && config_notify_enabled && attr) {
            const char *response = "STOPPED";
            bt_gatt_notify(conn, attr, response, strlen(response));
        }
    } else {
        LOG_WRN("Unknown command: %s", command);
        
        // Send error notification
        if (conn && config_notify_enabled && attr) {
            const char *response = "ERROR";
            bt_gatt_notify(conn, attr, response, strlen(response));
        }
    }
    return len;
}

// GATT service definition
BT_GATT_SERVICE_DEFINE(legacy_insole_svc,
    BT_GATT_PRIMARY_SERVICE(&legacy_insole_svc_uuid),
    BT_GATT_CHARACTERISTIC(&legacy_insole_char_uuid.uuid, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(insole_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&legacy_config_char_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_WRITE, NULL, on_config_write, NULL),
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
    
    return BT_GATT_ITER_CONTINUE;
}

static void legacy_thread_func(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    // Auto-start streaming like legacy firmware
    // Wait a bit for BLE stack to be ready
    k_sleep(K_SECONDS(2));
    
    // Find the attribute for notifications
    bt_gatt_foreach_attr(0x0001, 0xffff, find_attr_cb, NULL);
    
    if (!insole_data_attr) {
        LOG_ERR("Failed to find insole data characteristic attribute");
        return;
    }
    
    insole_notify_enabled = 1;
    LOG_INF("Legacy BLE auto-start streaming enabled");

    while (true) {
        if (insole_notify_enabled && insole_data_attr) {
            uint8_t buffer[107];
            size_t len = pack_legacy_data(buffer);
            
            // Only send notifications if there are subscribers
            // This prevents errors when no client is connected
            int err = bt_gatt_notify(NULL, insole_data_attr, buffer, len);
            if (err == -ENOTCONN) {
                // No connections, this is normal
                LOG_DBG("No BLE connections for legacy notifications");
            } else if (err < 0 && err != -ENOTCONN) {
                LOG_WRN("Legacy BLE notify error: %d", err);
            }
        }
        k_sleep(K_MSEC(50));
    }
}

// Function to pack data matching legacy format
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

    // 4. Orientation/Euler (12 bytes: 3x float yaw, pitch, roll remapped as z, y, x)
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
    // Calculate gravity from quaternion (assuming standard gravity of 9.81 m/s²)
    // Gravity vector in body frame from quaternion
    float gx = 2.0f * (quat[1] * quat[3] - quat[0] * quat[2]) * 9.81f;
    float gy = 2.0f * (quat[0] * quat[1] + quat[2] * quat[3]) * 9.81f;
    float gz = (quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3]) * 9.81f;
    
    float gravity[3] = {gx, gy, gz};
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &gravity[i], sizeof(float));
        index += sizeof(float);
    }

    // 8. Magnetometer (12 bytes: 3x float z, y, x) - fill with 0
    float mg[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; i++) {
        memcpy(&buffer[index], &mg[i], sizeof(float));
        index += sizeof(float);
    }

    // 9. Temperature (4 bytes: float)
    memcpy(&buffer[index], &temp, sizeof(float));
    index += sizeof(float);

    // 10. Battery level (4 bytes: float percentage)
    float battery = 50.0f;
    memcpy(&buffer[index], &battery, sizeof(float));
    index += sizeof(float);

    // 11. Checksum (1 byte: XOR of all previous bytes)
    uint8_t checksum = 0;
    for (size_t i = 0; i < index; i++) {
        checksum ^= buffer[i];
    }
    buffer[index++] = checksum;

    return index;
}

void legacy_ble_init(void) {
    k_thread_create(&legacy_thread_data, legacy_stack, LEGACY_THREAD_STACK_SIZE,
                    legacy_thread_func, NULL, NULL, NULL,
                    LEGACY_THREAD_PRIORITY, 0, K_NO_WAIT);
    LOG_INF("Legacy BLE service initialized");
}

#endif // CONFIG_LEGACY_BLE_ENABLED

/**
 * @file device_info_service.cpp
 * @author
 * @brief
 * @version 2.4.1
 * @date 2025-05-12
 *
 * @copyright Botz Innovation 2025
 *
 */

#define MODULE bluetooth

#include <array>
#include <cstdint>
#include <cstring>
#include <errno.h>
#include <iomanip>
#include <sstream>
#include <stddef.h>
#include <string.h>
#include <string_view>

#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#include "ble_services.hpp"
#include <app_version.h>

static ssize_t read_str(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                        uint16_t offset);

LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);

static uint8_t dis_manuf[16] = "Botz Innovation";
static uint8_t dis_model[10] = "S-01";
static uint8_t dis_serial[10] = "12345";
static uint8_t dis_hw_rev[10] = "v1.1.0";
static uint8_t dis_fw_rev[10] = {(uint8_t)(((APP_VERSION_MAJOR / 100) % 10) + '0'), // Hundreds digit (if applicable)
                                 (uint8_t)(((APP_VERSION_MAJOR / 10) % 10) + '0'),  // Tens digit
                                 (uint8_t)((APP_VERSION_MAJOR % 10) + '0'),         // Units digit
                                 '.',
                                 (uint8_t)(((APP_VERSION_MINOR / 10) % 10) + '0'), // Tens digit
                                 (uint8_t)((APP_VERSION_MINOR % 10) + '0'),        // Units digit
                                 '.',
                                 (uint8_t)(((APP_PATCHLEVEL / 10) % 10) + '0'), // Tens digit
                                 (uint8_t)((APP_PATCHLEVEL % 10) + '0'),        // Units digit
                                 '\0'};
// clang-format off

/* Device Information Service Declaration */
BT_GATT_SERVICE_DEFINE(
    dis_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_DIS),

    BT_GATT_CHARACTERISTIC
    (
        BT_UUID_DIS_MANUFACTURER_NAME,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_str, nullptr, dis_manuf
    ),

    BT_GATT_CHARACTERISTIC
    (
        BT_UUID_DIS_MODEL_NUMBER,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_str, nullptr, dis_model
    ),

    BT_GATT_CHARACTERISTIC
    (
        BT_UUID_DIS_HARDWARE_REVISION,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_str, nullptr, dis_hw_rev
    ),

    BT_GATT_CHARACTERISTIC
    (
        BT_UUID_DIS_FIRMWARE_REVISION,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_str, nullptr, dis_fw_rev
    ),

    BT_GATT_CHARACTERISTIC
    (
        BT_UUID_DIS_SERIAL_NUMBER,
        BT_GATT_CHRC_READ,
        BT_GATT_PERM_READ,
        read_str, nullptr, dis_serial
    )

);

static ssize_t read_str(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    auto value_len = strlen(static_cast<const char *>(attr->user_data));
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, value_len);
}

// Getter functions for device information
const char* dis_get_manufacturer(void)
{
    return (const char*)dis_manuf;
}

const char* dis_get_model(void)
{
    return (const char*)dis_model;
}

const char* dis_get_serial(void)
{
    return (const char*)dis_serial;
}

const char* dis_get_hw_revision(void)
{
    return (const char*)dis_hw_rev;
}

const char* dis_get_fw_revision(void)
{
    return (const char*)dis_fw_rev;
}

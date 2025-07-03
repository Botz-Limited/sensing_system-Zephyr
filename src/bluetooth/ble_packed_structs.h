/**
 * @file ble_packed_structs.h
 * @brief Packed BLE data structures for efficient transmission
 * @version 1.0
 * @date 2025-01
 *
 * @copyright Copyright (c) 2025 Botz Innovation
 */

#ifndef BLE_PACKED_STRUCTS_H
#define BLE_PACKED_STRUCTS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Packed device status structure
 * Combines multiple status fields into a single characteristic
 */
typedef struct __attribute__((packed)) {
    uint32_t status_bitfield;      // 0-3: Status flags (see status_codes.h)
    uint8_t  battery_percent;      // 4: Battery percentage (0-100)
    uint8_t  charge_status;        // 5: Charging status (0=not charging, 1=charging, 2=charged)
    uint8_t  temperature_c;        // 6: Device temperature in Celsius
    uint8_t  activity_state;       // 7: Activity state (0=idle, 1=active, 2=paused)
    uint32_t uptime_seconds;       // 8-11: Device uptime in seconds
    uint16_t free_storage_mb;      // 12-13: Free storage in MB
    uint8_t  connected_devices;    // 14: Number of connected devices (0-2)
    uint8_t  reserved;             // 15: Reserved for future use
} device_status_packed_t;

/**
 * @brief Packed secondary device info structure
 * Combines all secondary device information into one characteristic
 */
typedef struct __attribute__((packed)) {
    char     manufacturer[16];     // 0-15: Manufacturer name
    char     model[16];           // 16-31: Model name
    char     serial[12];          // 32-43: Serial number
    char     hw_rev[8];           // 44-51: Hardware revision
    char     fw_rev[8];           // 52-59: Firmware revision
    uint8_t  battery_percent;     // 60: Secondary battery percentage
    uint8_t  status;              // 61: Secondary status (0=disconnected, 1=connected, 2=error)
    uint16_t reserved;            // 62-63: Reserved for alignment
} secondary_device_info_packed_t;

/**
 * @brief Packed file notification structure
 * Combines file availability and path into one notification
 */
typedef struct __attribute__((packed)) {
    uint8_t  file_id;             // 0: File ID
    uint8_t  file_type;           // 1: File type (0=foot, 1=bhi360, 2=activity)
    uint8_t  device_source;       // 2: Device source (0=primary, 1=secondary)
    uint8_t  reserved;            // 3: Reserved for alignment
    char     file_path[60];       // 4-63: File path
} file_notification_packed_t;

#ifdef __cplusplus
}
#endif

#endif // BLE_PACKED_STRUCTS_H
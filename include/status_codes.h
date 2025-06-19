#ifndef STATUS_CODES_H
#define STATUS_CODES_H

#include <stdint.h>

// Device status bitfield definitions
// You can OR these together for combined status

// General status bits (0-7)
#define STATUS_ERROR           (1 << 0)  // General error flag
#define STATUS_CALIBRATING     (1 << 1)  // Device is calibrating
#define STATUS_READY           (1 << 2)  // Device is ready
#define STATUS_IDLE            (1 << 3)  // Device is idle
#define STATUS_LOGGING_ACTIVE  (1 << 4)  // Data logging is active
#define STATUS_LOW_BATTERY     (1 << 5)  // Battery is low
#define STATUS_CHARGING        (1 << 6)  // Device is charging
#define STATUS_FOTA_ACTIVE     (1 << 7)  // FOTA update in progress

// Error-specific bits (8-31) - mapped from err_t values
#define STATUS_BATTERY_FAULT          (1 << 8)   // Battery fault detected
#define STATUS_BLUETOOTH_ERROR        (1 << 9)   // Bluetooth error
#define STATUS_HARDWARE_ERROR         (1 << 10)  // Hardware error
#define STATUS_DATA_ERROR             (1 << 11)  // Data/filesystem error
#define STATUS_DFU_ERROR              (1 << 12)  // DFU/FOTA error
#define STATUS_ADC_ERROR              (1 << 13)  // ADC error
#define STATUS_I2C_ERROR              (1 << 14)  // I2C communication error
#define STATUS_BATTERY_DISCONNECTED   (1 << 15)  // Battery disconnected
#define STATUS_MOTION_ERROR           (1 << 16)  // Motion sensor error
#define STATUS_RTC_ERROR              (1 << 17)  // RTC error
#define STATUS_FILE_SYSTEM_ERROR      (1 << 18)  // File system error
#define STATUS_PROTO_ENCODE_ERROR     (1 << 19)  // Protocol encoding error
#define STATUS_FILE_SYSTEM_NO_FILES   (1 << 20)  // No files in filesystem
#define STATUS_FILE_SYSTEM_FULL       (1 << 21)  // Storage full
#define STATUS_FLASH_FAILURE          (1 << 22)  // Flash memory failure

// Mask for all error bits
#define STATUS_ALL_ERRORS_MASK        (0x7FFF00)  // Bits 8-22

#endif // STATUS_CODES_H

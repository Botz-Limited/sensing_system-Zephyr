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

// Error-specific bits (8-22) - mapped from err_t values
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

// Additional status bits (23-31)
#define STATUS_WEIGHT_NOT_CALIBRATED  (1 << 23)  // Weight sensor not calibrated
#define STATUS_WEIGHT_CALIBRATING     (1 << 24)  // Weight calibration in progress

// Activity state bits (25-29) - represent different activity states
#define STATUS_ACTIVITY_1_RUNNING      (1 << 25)  // Activity 1 running (normal activity)
#define STATUS_ACTIVITY_3_FOOT_STREAM  (1 << 26)  // Activity 3 (foot streaming only)
#define STATUS_ACTIVITY_4_QUAT_STREAM  (1 << 27)  // Activity 4 (quaternion streaming only)
#define STATUS_ACTIVITY_5_BOTH_STREAM  (1 << 28)  // Activity 5 (both streaming)
#define STATUS_ACTIVITY_PAUSED         (1 << 29)  // Activity is paused
// Note: STATUS_ACTIVITY_IDLE is when none of the above bits are set

// Connection status bits
#define STATUS_D2D_CONNECTED           (1 << 30)  // Secondary device (D2D) connected

// Mask for activity state bits
#define STATUS_ACTIVITY_MASK           (0x3E000000)  // Bits 25-29

// Mask for all error bits
#define STATUS_ALL_ERRORS_MASK        (0x7FFF00)  // Bits 8-22

#endif // STATUS_CODES_H

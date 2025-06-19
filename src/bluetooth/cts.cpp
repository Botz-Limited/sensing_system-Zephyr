/**
 * @file cts.cpp
 * @author Andy Bond (andrew.bond@chiaro.co.uk)
 * @brief 
 * @version 0.1
 * @date 2024-11-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <cstdint>
#define MODULE bluetooth

#include <cmath>
#include <ctime>
#include <cstring>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/types.h>

#include "ble_services.hpp"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/counter.h>
#include "ble_services.hpp"
#include <errors.hpp>
#include <inttypes.h> // For PRIu32 macro
#include <time.h>     // For time_t, struct tm, gmtime, strftime


LOG_MODULE_DECLARE(MODULE, CONFIG_BLUETOOTH_MODULE_LOG_LEVEL);


// --- RTC Configuration Defines ---
// Ensure 'rtc0' alias is correctly defined in your board's devicetree or overlay file
#define RTC_NODE DT_ALIAS(rtc0)

// Max value of the RTC counter (24-bit for nRF RTC). Used for wraparound calculations.
static constexpr uint32_t RTC_MAX_VALUE = (1U << 24) - 1;

// --- Global Time Tracking Variables ---
static const struct device *const rtc_dev = DEVICE_DT_GET(RTC_NODE);

// last_synced_epoch_time_s: The Unix epoch time in seconds when the RTC was last synchronized.
static uint32_t last_synced_epoch_time_s = 0;

// last_synced_rtc_ticks: The RTC counter value (ticks) when `last_synced_epoch_time_s` was set.
static uint32_t last_synced_rtc_ticks = 0;

// rtc_ticks_per_second: The frequency of the RTC counter in ticks per second.
static uint32_t rtc_ticks_per_second = 1; // Default, will be updated by counter_get_frequency()


// --- Unused (from original) defines ---
// These appear to be unused with the Zephyr Counter RTC approach,
// but I'm keeping them as you had them in your original file.
constexpr uint8_t max_retries = CONFIG_SENSOR_FAILURE_MAX_RETRIES;
constexpr uint8_t sleep_timer_ms = CONFIG_RETRY_SLEEP_TIMER;

// --- RTC Counter Callback Function (Optional for simple timekeeping) ---
// This specific callback is not used in the current time calculation logic,
// as time is derived from counter_get_value() directly.
// Currently unused - kept for future RTC implementation
__attribute__((unused))
static void rtc_counter_callback(const struct device *dev, void *user_data,
uint32_t ticks, uint32_t status)
{
ARG_UNUSED(dev);
ARG_UNUSED(user_data);
ARG_UNUSED(ticks);
ARG_UNUSED(status);
    // LOG_DBG("RTC callback: ticks=%u, status=%u", ticks, status);
}

// --- Bluetooth Current Time Service (CTS) Characteristic Data Structure ---
// This struct maps directly to the "Current Time" characteristic value format (UUID 0x2A2B)
// as defined by the Bluetooth SIG.
// Total size: 10 (Exact Time 256) + 1 (DST Offset) + 1 (Time Zone) = 12 bytes
struct bt_cts_current_time_val {
    // Exact Time 256 (10 bytes)
    uint16_t year;          // LSB first
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t day_of_week;    // 1 = Monday, ..., 7 = Sunday
    uint8_t fractions_256;  // 1/256th of a second (0-255)
    uint8_t adjust_reason;  // See Bluetooth Core Spec for values (e.g., 0=No adjustment)

    // Remainder of Current Time characteristic (2 bytes)
    uint8_t dst_offset;     // 0=Standard, 1=Half hour, 2=Hour, 255=Unknown
    int8_t  time_zone;      // -48 to +56 (multiples of 15 min), 0=UTC, -128=Unknown
} __packed; // `__packed` is crucial to ensure no padding for correct BLE format

// --- Static Characteristic Value Buffer for CTS ---
// This buffer will hold the time data in the specific format for the BLE characteristic.
static struct bt_cts_current_time_val current_time_char_val_buffer;

// --- Helper function to populate CTS struct from `struct tm` ---
// This function takes a standard `struct tm` (like what `gmtime()` returns) and
// correctly populates the `bt_cts_current_time_val` struct according to the
// Bluetooth Current Time characteristic's specified format.
static void populate_cts_value(struct bt_cts_current_time_val *cts_val, const struct tm *tm_gmt) {
    std::memset(cts_val, 0, sizeof(*cts_val)); // Clear the struct before populating

    // Populate Exact Time 256 (10 bytes)
    cts_val->year = sys_cpu_to_le16(static_cast<uint16_t>(tm_gmt->tm_year + 1900)); // Year since 1900, LSB first
    cts_val->month = static_cast<uint8_t>(tm_gmt->tm_mon + 1); // Month (0-11 -> 1-12)
    cts_val->day = static_cast<uint8_t>(tm_gmt->tm_mday);
    cts_val->hours = static_cast<uint8_t>(tm_gmt->tm_hour);
    cts_val->minutes = static_cast<uint8_t>(tm_gmt->tm_min);
    cts_val->seconds = static_cast<uint8_t>(tm_gmt->tm_sec);
    // Day of Week: `struct tm` uses 0 for Sunday, 1 for Monday, ..., 6 for Saturday.
    // CTS characteristic uses 1 for Monday, ..., 7 for Sunday.
    cts_val->day_of_week = static_cast<uint8_t>(tm_gmt->tm_wday == 0 ? 7 : tm_gmt->tm_wday);
    cts_val->fractions_256 = 0; // Not tracking sub-second fractions, so set to 0.
    cts_val->adjust_reason = 0; // No adjustment, as per Bluetooth Core Spec (BT_CTS_ADJUST_REASON_NO_CHANGE)

    // Populate remaining CTS fields (2 bytes)
    cts_val->dst_offset = 0;    // 0 = Standard Time (no DST offset within characteristic)
                                // Adjust this if your device tracks and reports DST.
    cts_val->time_zone = 0;     // 0 = UTC (as we use `gmtime`).
                                // Use -128 for Unknown, or specific values for fixed time zones.
                                // e.g., for New Addington (UTC+0 in standard time), 0 is correct.
}


// --- RTC Timekeeping Functions ---

/**
 * @brief Initializes the RTC counter.
 *
 * This function sets up the RTC counter, including its top value and starts it.
 * It also determines the actual frequency of the RTC ticks per second.
 *
 * @return 0 on success, or a negative error code.
 */
int init_rtc_time()
{
    if (!device_is_ready(rtc_dev)) {
        LOG_ERR("RTC device not ready at %s!", rtc_dev->name);
        return -ENODEV;
    }

    struct counter_top_cfg top_cfg = {
        .ticks = RTC_MAX_VALUE, // Configure the counter to run to its full 24-bit range
        .callback = NULL,       // No callback needed for simple time reading
        .user_data = NULL,
        .flags = COUNTER_TOP_CFG_DONT_RESET, // Ensures the counter wraps naturally at RTC_MAX_VALUE
    };

    int ret = counter_set_top_value(rtc_dev, &top_cfg);
    if (ret != 0) {
        LOG_ERR("Failed to set RTC top value: %d", ret);
        return ret;
    }

    ret = counter_start(rtc_dev);
    if (ret != 0) {
        LOG_ERR("Failed to start RTC: %d", ret);
        return ret;
    }

    // Get the actual frequency the RTC is configured to tick at.
    // For nRF RTC with LFXO, this should typically be 1 Hz (after prescaler).
    rtc_ticks_per_second = counter_get_frequency(rtc_dev);
    if (rtc_ticks_per_second == 0) {
        LOG_ERR("RTC frequency reported as 0, this is unexpected. Defaulting to 1 Hz.");
        rtc_ticks_per_second = 1; // Fallback to a safe assumption
    } else if (rtc_ticks_per_second != 1) {
        LOG_WRN("RTC configured to %u Hz. Calculations will adapt accordingly.",
                rtc_ticks_per_second);
    }

    LOG_INF("RTC initialized. Actual frequency: %u Hz, Max value: %u",
            rtc_ticks_per_second, RTC_MAX_VALUE);
    return 0;
}

/**
 * @brief Sets (synchronizes) the current RTC time based on a given Unix epoch time.
 *
 * This function records the received epoch time and the current RTC counter value,
 * serving as a reference point for future time calculations.
 *
 * @param[in] new_epoch_time_s The Unix epoch time (seconds since 1970-01-01 00:00:00 UTC).
 */
void set_current_time_from_epoch(uint32_t new_epoch_time_s)
{
    uint32_t current_rtc_ticks_val;

    int ret = counter_get_value(rtc_dev, &current_rtc_ticks_val);
    if (ret != 0) {
        LOG_ERR("Failed to get RTC value during sync: %d", ret);
        return; // Unable to read RTC, so cannot sync reliably
    }

    last_synced_epoch_time_s = new_epoch_time_s;
    last_synced_rtc_ticks = current_rtc_ticks_val;
    LOG_INF("RTC synchronized: Epoch Time %" PRIu32 ", RTC ticks %u",
            last_synced_epoch_time_s, last_synced_rtc_ticks);
}

/**
 * @brief Gets the current estimated Unix epoch time.
 *
 * This function calculates the current epoch time by adding the elapsed seconds
 * from the last synchronization point to the last synchronized epoch time.
 * It handles RTC counter wraparound.
 *
 * @return The current estimated Unix epoch time, or 0 if RTC was never synchronized.
 */
uint32_t get_current_epoch_time()
{
    if (last_synced_epoch_time_s == 0) {
        LOG_WRN("RTC not yet synchronized. Returning 0.");
        //assigned default time 
        set_current_time_from_epoch(1749574850);
    }

    uint32_t current_rtc_ticks_val;

    int ret = counter_get_value(rtc_dev, &current_rtc_ticks_val);
    if (ret != 0) {
        LOG_ERR("Failed to get RTC value: %d", ret);
        // If RTC read fails, return the last known good time as a fallback
        return last_synced_epoch_time_s;
    }

    uint32_t elapsed_rtc_ticks;
    if (current_rtc_ticks_val >= last_synced_rtc_ticks) {
        elapsed_rtc_ticks = current_rtc_ticks_val - last_synced_rtc_ticks;
    } else {
        // Handle counter wraparound (e.g., 24-bit counter from 0xFFFFFF to 0x000000)
        elapsed_rtc_ticks = (RTC_MAX_VALUE - last_synced_rtc_ticks) + current_rtc_ticks_val + 1;
    }

    // Convert elapsed ticks to elapsed seconds using the actual RTC frequency
    uint32_t elapsed_seconds = elapsed_rtc_ticks / rtc_ticks_per_second;

    return last_synced_epoch_time_s + elapsed_seconds;
}


// --- Public Accessor Functions for BLE Characteristics ---
// These functions allow other files (e.g., your BLE service definition file)
// to get a pointer to the static characteristic value buffer and its size.
// This is necessary for the BT_GATT_CHARACTERISTIC macro and for sending
// notifications with `bt_gatt_notify()`.

/**
 * @brief Get a const pointer to the Current Time characteristic's value buffer.
 *
 * This function retrieves a pointer to the internally managed buffer that holds
 * the current time formatted for the Bluetooth CTS characteristic.
 *
 * @return A const void pointer to the static `current_time_char_val_buffer`.
 */
const void* get_current_time_char_value_ptr() {
    return static_cast<const void*>(&current_time_char_val_buffer);
}

/**
 * @brief Get the size of the Current Time characteristic's value buffer.
 *
 * This function provides the size in bytes of the buffer used for the
 * Bluetooth CTS characteristic value.
 *
 * @return The size in bytes of the `current_time_char_val_buffer`.
 */
size_t get_current_time_char_value_size() {
    return sizeof(current_time_char_val_buffer);
}

/**
 * @brief Update the internal CTS characteristic buffer.
 *
 * This function calculates the current time from the RTC and populates the
 * internal buffer used for the Current Time Service characteristic. This
 * should be called before a GATT read or notification if the time has changed.
 */
void update_cts_characteristic_buffer() {
    uint32_t epoch_time = get_current_epoch_time();
    struct tm *tm_gmt = nullptr;

    if (epoch_time > 0) {
        time_t time_val = static_cast<time_t>(epoch_time);
        tm_gmt = gmtime(&time_val);
        if (tm_gmt == nullptr) {
            LOG_ERR("Failed to convert epoch time to tm struct for CTS buffer update.");
        }
    } else {
        LOG_WRN("Cannot update CTS buffer: RTC not synchronized.");
    }

    if (tm_gmt != nullptr) {
        populate_cts_value(&current_time_char_val_buffer, tm_gmt);
    } else {
        std::memset(&current_time_char_val_buffer, 0, sizeof(current_time_char_val_buffer));
    }
}



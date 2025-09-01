/**
 * @file ccc_callback_fix.hpp
 * @brief Common CCC callback handling to prevent crashes on disconnect
 * @version 1.0

 *
 * @copyright Botz Innovation 2025
 */

#ifndef CCC_CALLBACK_FIX_HPP
#define CCC_CALLBACK_FIX_HPP

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

/**
 * @brief Safe CCC callback wrapper that prevents crashes
 * 
 * This macro creates a safe CCC callback function that properly handles
 * null pointers and validates parameters before use.
 * 
 * @param name Function name for the CCC callback
 * @param subscribed_var Variable to update with subscription state
 */
#define DEFINE_SAFE_CCC_CALLBACK(name, subscribed_var) \
    static void name(const struct bt_gatt_attr *attr, uint16_t value) \
    { \
        if (!attr) { \
            LOG_ERR(#name ": attr is NULL"); \
            return; \
        } \
        subscribed_var = (value == BT_GATT_CCC_NOTIFY); \
        LOG_DBG(#name ": CCC Notify: %d", (value == BT_GATT_CCC_NOTIFY)); \
    }

/**
 * @brief Minimal safe CCC callback for unused callbacks
 * 
 * This macro creates a minimal CCC callback that just logs the change
 * without updating any variables. Use this for CCC callbacks that don't
 * need to track subscription state.
 */
#define DEFINE_MINIMAL_CCC_CALLBACK(name) \
    static void name(const struct bt_gatt_attr *attr, uint16_t value) \
    { \
        if (!attr) { \
            LOG_ERR(#name ": attr is NULL"); \
            return; \
        } \
        ARG_UNUSED(value); \
        LOG_DBG(#name ": CCC changed to: %d", value); \
    }

#endif // CCC_CALLBACK_FIX_HPP
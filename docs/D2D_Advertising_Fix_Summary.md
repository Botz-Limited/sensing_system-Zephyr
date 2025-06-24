# D2D Advertising Fix Summary

## Problem Description
After the D2D (device-to-device) fix, the primary device's Bluetooth advertising was not visible to phones. The advertising appeared to start successfully according to logs, but phones couldn't discover the device.

## Root Causes Identified

1. **Identity Address Timing**: The Bluetooth identity address might not be fully initialized when advertising starts, especially after the D2D changes.

2. **Advertising Packet Size**: The advertising data included a 128-bit UUID along with other data, which could exceed the maximum advertising packet size (31 bytes).

3. **Initialization Timing**: The settings load and Bluetooth name setting might need more time to complete before advertising can start successfully.

## Fixes Applied

### 1. Extended Initialization Delay
```c
// Increased from 10ms to 100ms to ensure settings and identity are fully loaded
k_msleep(100);
```

### 2. Added Identity Address Verification
```c
// Check if we have a valid identity address before advertising
bt_addr_le_t addr;
size_t count = 1;
bt_id_get(&addr, &count);
if (count > 0) {
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
    LOG_INF("Bluetooth identity address: %s", addr_str);
} else {
    LOG_WRN("No Bluetooth identity address available");
}
```

### 3. Advertising Fallback Mechanism
Added a fallback mechanism that retries advertising without the `BT_LE_ADV_OPT_USE_IDENTITY` flag if the initial attempt fails:

```c
if (err == -ENOENT || err == -EINVAL) {
    LOG_WRN("Retrying advertising without USE_IDENTITY option");
    
    // Create temporary advertising params without USE_IDENTITY
    struct bt_le_adv_param temp_adv_params = {
        .options = BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_NAME,
        // ... other params
    };
    
    err = bt_le_adv_start(&temp_adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}
```

### 4. Split Advertising and Scan Response Data
Moved the 128-bit UUID to scan response data to reduce the advertising packet size:

```c
/* Advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL), BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
};

/* Scan response data - put the 128-bit UUID here */
static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0x5cb36a14, 0xca69, 0x4d97, 0x89a8, 0x001ffc9ec8cd)),
};
```

### 5. Enhanced Logging
Added detailed logging for:
- Advertising parameters before starting
- Device name setting success/failure
- Identity address availability
- Specific error codes and retry attempts

## Testing Recommendations

1. **Build and flash** the primary device with these changes
2. **Monitor logs** during startup to see:
   - "Bluetooth identity address: XX:XX:XX:XX:XX:XX"
   - "Bluetooth device name set to: SensingGR"
   - "Starting advertising with params: ..."
   - "Advertising successfully started"

3. **Test with phone** to verify:
   - Device appears in Bluetooth scan
   - Device name shows as "SensingGR"
   - Can connect successfully

4. **Test D2D functionality** to ensure:
   - Secondary device can still connect
   - D2D communication works properly

## Additional Notes

- The `BT_LE_ADV_OPT_USE_IDENTITY` flag requires a valid identity address from the Bluetooth controller
- Splitting advertising and scan response data helps avoid packet size limitations
- The fallback mechanism ensures advertising works even if identity initialization is delayed
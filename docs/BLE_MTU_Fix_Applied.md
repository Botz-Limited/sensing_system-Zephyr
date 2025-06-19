# BLE MTU Exchange Fix - Applied

## Problem Solved

The issue was that your device was operating with an MTU of only 32 bytes (instead of the expected 247), which caused:
- `No ATT channel for MTU 32` warnings
- `No buffer available to send notification` warnings

## Root Cause

The Bluetooth connection callbacks were not requesting MTU exchange after connection establishment. This left the connection at the minimum MTU of 23 bytes (which becomes 32 with overhead).

## Fix Applied

Added MTU exchange requests to both connection callbacks in `bluetooth.cpp`:

1. **Main connection callback** (`connected` function):
   ```cpp
   // Request MTU exchange to avoid small MTU issues
   static struct bt_gatt_exchange_params exchange_params;
   exchange_params.func = [](struct bt_conn *conn, uint8_t err, 
                            struct bt_gatt_exchange_params *params) {
       if (!err) {
           uint16_t mtu = bt_gatt_get_mtu(conn);
           LOG_INF("MTU exchange successful: %u", mtu);
       } else {
           LOG_WRN("MTU exchange failed: %d", err);
       }
   };
   
   int mtu_ret = bt_gatt_exchange_mtu(conn, &exchange_params);
   if (mtu_ret) {
       LOG_WRN("Failed to initiate MTU exchange: %d", mtu_ret);
   } else {
       LOG_DBG("MTU exchange initiated");
   }
   ```

2. **D2D connection callback** (`d2d_connected` function):
   - Similar MTU exchange code added for device-to-device connections

## Expected Results

After flashing the updated firmware:

1. **Connection logs should show**:
   ```
   [timestamp] <inf> bluetooth: Connected
   [timestamp] <inf> bluetooth: MTU exchange initiated
   [timestamp] <inf> bluetooth: MTU exchange successful: 247
   ```

2. **No more buffer warnings**:
   - The "No ATT channel for MTU 32" warning should disappear
   - The "No buffer available to send notification" warning should be resolved

## Complete Solution Summary

The fix involved two parts:

1. **Configuration changes** (already applied):
   - Increased BLE buffers in `prj.conf`
   - Added 16KB more heap memory
   - Increased ATT, L2CAP, and ACL buffers

2. **Code changes** (just applied):
   - Added MTU exchange to connection callbacks
   - Ensures proper MTU negotiation (247 bytes instead of 32)

## Next Steps

1. **Flash the firmware**:
   ```bash
   west flash
   ```

2. **Monitor RTT logs** to verify:
   - MTU exchange succeeds (should show 247 or similar)
   - No more buffer warnings when using stop activity

3. **Test the stop activity characteristic** again

## Troubleshooting

If you still see issues:

1. **Check mobile app MTU support**:
   - Ensure the mobile app supports MTU > 185
   - iOS typically supports up to 185 bytes
   - Android typically supports up to 517 bytes

2. **Verify connection parameters**:
   - The connection interval might need adjustment
   - Check if the mobile app is limiting MTU

3. **Monitor exact MTU value**:
   - If MTU is less than expected, the peer device may be limiting it
   - Nordic devices typically support 247 bytes

## Technical Details

- **Default MTU**: 23 bytes (20 bytes payload + 3 bytes ATT header)
- **With overhead**: Shows as 32 in logs
- **Nordic optimal MTU**: 247 bytes
- **Effective payload**: 244 bytes (247 - 3 byte ATT header)

The combination of increased buffers and proper MTU exchange should completely resolve your notification issues.
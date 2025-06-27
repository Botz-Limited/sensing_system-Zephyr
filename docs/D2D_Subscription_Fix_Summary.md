# D2D Subscription ENOMEM Fix Summary

## Problem
The primary device fails to subscribe to all 15 D2D characteristics from the secondary device with ENOMEM (-12) error.

## Root Causes
1. Insufficient ATT channels/buffers in the application core
2. Network core RAM overflow when buffers increased too much
3. Possible GATT database size limitations

## Solution Applied

### Application Core (prj.conf)
- Increased `CONFIG_BT_ATT_PREPARE_COUNT` from 5 to 16
- Increased `CONFIG_BT_ATT_TX_COUNT` from 15 to 20
- Added `CONFIG_BT_CONN_TX_MAX=20`
- Added `CONFIG_BT_GATT_DYNAMIC_DB=y` for dynamic GATT support
- Increased heap size to 32KB
- Increased main stack size to 8KB

### Network Core (sysbuild/ipc_radio/)
- Kept conservative settings due to 64KB RAM limit:
  - `CONFIG_BT_MAX_CONN=9`
  - `CONFIG_BT_BUF_EVT_RX_COUNT=20`
  - `CONFIG_BT_CTLR_RX_BUFFERS=16`
  - Heap size: 4KB
  - System workqueue: 2KB

### Key Insights
1. The network core (BLE controller) doesn't need to know about the 15 subscriptions - that's handled at the host level
2. The network core has only 64KB RAM, so buffer increases must be conservative
3. Dynamic GATT database is needed for the large number of attributes (175+)
4. The subscription limit is primarily an application core issue, not network core

## Testing
After rebuilding with these changes, the primary device should be able to subscribe to all 15 D2D characteristics without ENOMEM errors.
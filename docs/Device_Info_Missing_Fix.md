# Device Info Missing Fix

## Problem
The secondary device sends device info only once, 3 seconds after connection. If the primary hasn't subscribed to the device info characteristic by then (due to subscription issues), the notification is lost.

## Root Cause
1. Secondary schedules device info to be sent at 3 seconds after connection
2. Primary was failing to subscribe to device info characteristic (index 14) due to ENOMEM
3. When secondary sends the notification, primary isn't subscribed, so it never receives it

## Solutions

### Option 1: Retry Mechanism (Recommended)
Add a retry mechanism in the secondary device to periodically send device info until acknowledged.

### Option 2: On-Demand Request
Add a characteristic that allows the primary to request device info after subscriptions are complete.

### Option 3: Increase Delay
Increase the delay from 3 seconds to ensure subscriptions are complete (not recommended as it's not reliable).

## Current Workaround
With the subscription fixes we implemented (increased buffers, dynamic GATT), the device info subscription should now succeed, and the 3-second delay should be sufficient.

## Testing
1. Ensure all subscriptions complete successfully
2. Look for "Successfully subscribed to handle XX at index 8" (device info is around index 8)
3. Verify "RECEIVED DEVICE INFO FROM SECONDARY" appears in logs
4. If still missing, implement retry mechanism
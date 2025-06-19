# Information Service Troubleshooting Guide

## Problem Description
When running as primary device (CONFIG_PRIMARY_DEVICE=y), many of the information service characteristics don't work properly.

## Root Cause Analysis

### 1. Security Requirements
All characteristics in the information service require encryption (`BT_GATT_PERM_READ_ENCRYPT`). This means:
- The device must be paired/bonded before characteristics can be accessed
- The connection must have at least security level L2 (authenticated pairing with encryption)

### 2. Common Issues

#### Issue 1: Device Not Paired
**Symptoms:**
- Characteristics appear in service discovery but return errors when read/written
- GATT error codes like "Insufficient Authentication" or "Insufficient Encryption"

**Solution:**
1. Ensure the device is properly paired in your phone's Bluetooth settings
2. If already paired, try unpairing and re-pairing
3. Check that pairing completes successfully (look for "Pairing Complete (bonded)" in logs)

#### Issue 2: Security Level Not Established
**Symptoms:**
- Connection established but characteristics still not accessible
- Security level remains at L0 or L1

**Solution:**
The code automatically requests security level L2 on connection. Check logs for:
- "Security changed: ... level 2" (successful)
- "Security failed: ..." (failed - check error code)

#### Issue 3: Subscription Issues
**Symptoms:**
- Notifications not being sent even though data is available
- CCC (Client Characteristic Configuration) not properly set

**Solution:**
Ensure the client (phone app) is properly subscribing to notifications by writing to the CCC descriptor.

## Debugging Steps

### 1. Enable Debug Logging
The code now includes debug utilities. When a connection is established, it will log:
- Current security level
- List of bonded devices
- Service registration status
- Characteristic access guidance

### 2. Check Logs for Key Messages
Look for these log messages:
```
[INF] Connected
[INF] Bluetooth Information Service Debug Info
[INF] Current security level: X
[INF] Security Level 2: Authenticated pairing with encryption
[INF] Bond X: <address>
[INF] Total bonds: X
```

### 3. Manual Testing Steps
1. **Clear all bonds:**
   - On phone: Forget the device in Bluetooth settings
   - On device: Device should clear bonds automatically or on reset

2. **Fresh pairing:**
   - Start advertising on device
   - Scan and connect from phone
   - Complete pairing process
   - Verify "Pairing Complete (bonded)" in logs

3. **Test characteristics:**
   - After security is established (level >= 2)
   - Try reading device status characteristic first
   - Then test notification characteristics

## Code Modifications Made

### 1. Added Debug Utilities
- `bluetooth_debug.cpp/hpp`: Debug functions to check security, bonds, and service status
- Integrated into connection and security callbacks

### 2. Enhanced Logging
- Added security level checking on connection
- Added debug output when security changes
- Better visibility into pairing/bonding status

## Potential Code Fixes

### Option 1: Reduce Security Requirements (NOT RECOMMENDED)
Change `BT_GATT_PERM_READ_ENCRYPT` to `BT_GATT_PERM_READ` for testing only.
This removes encryption requirement but compromises security.

### Option 2: Add Pairing Retry Logic
Implement automatic re-pairing attempts if security establishment fails.

### Option 3: Implement Just Works Pairing
For easier pairing, implement "Just Works" pairing mode:
```c
static struct bt_conn_auth_cb auth_cb = {
    .passkey_display = NULL,
    .passkey_entry = NULL,
    .passkey_confirm = NULL,
    .cancel = NULL,
    .pairing_confirm = pairing_confirm,
};

static void pairing_confirm(struct bt_conn *conn)
{
    bt_conn_auth_pairing_confirm(conn);
}
```

## Testing with nRF Connect

1. Install nRF Connect on your phone
2. Scan for "SensingGR" device
3. Connect and pair when prompted
4. Navigate to Information Service (UUID: 0c372eaa-27eb-437e-bef4-775aefaf3c97)
5. Try reading characteristics
6. Enable notifications on characteristics with notify property

## Next Steps

1. Build and flash the firmware with debug utilities
2. Monitor logs during connection/pairing
3. Share the debug output for further analysis
4. Based on findings, implement appropriate fix from options above
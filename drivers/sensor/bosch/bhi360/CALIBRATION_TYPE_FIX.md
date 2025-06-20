# BHI360 Calibration Type Fix

## Issue
Compiler warning in `bhi360_calibration.c`:
```
warning: passing argument 4 of 'bhy2_get_calibration_profile' from incompatible pointer type
```

## Root Cause
The BHY2 API function `bhy2_get_calibration_profile` expects a `uint32_t*` for the `actual_len` parameter, but we were using `uint16_t`.

## Fix Applied
Changed the variable type in `bhi360_get_calibration_profile()`:
```c
// Before:
uint16_t actual_len;

// After:
uint32_t actual_len;
```

## Verification
- The fix matches the BHY2 API signature
- Other functions using similar parameters (`bhi360_get_calibration_status`) already use `uint32_t`
- No other type mismatches found in the calibration module

## Impact
- Fixes compiler warning
- Prevents potential memory corruption from type mismatch
- No functional changes required
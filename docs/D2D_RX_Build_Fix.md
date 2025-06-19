# D2D RX Build Fix

## Issue
When building with `CONFIG_PRIMARY_DEVICE=y`, the following compilation error occurred:
```
/home/ee/sensing_fw/src/bluetooth/ble_d2d_rx.cpp:64:5: error: 'set_current_time_from_epoch' was not declared in this scope
```

## Root Cause
The D2D RX module (`ble_d2d_rx.cpp`) was calling `set_current_time_from_epoch()` function but was not including the header file where this function is declared.

## Solution
Added the missing include:
```cpp
#include <ble_services.hpp>
```

This header file contains the declaration:
```cpp
void set_current_time_from_epoch(uint32_t new_epoch_time_s);
```

## Function Location
- **Declaration**: `/home/ee/sensing_fw/include/ble_services.hpp`
- **Definition**: `/home/ee/sensing_fw/src/bluetooth/cts.cpp`
- **Usage**: 
  - `control_service.cpp` - When phone sets time
  - `ble_d2d_rx.cpp` - When secondary receives forwarded time command

## Build Verification
The build should now succeed for both configurations:
- `CONFIG_PRIMARY_DEVICE=y` - Primary device build
- `CONFIG_PRIMARY_DEVICE=n` - Secondary device build

Both builds include the D2D RX module which needs access to the RTC time setting function.
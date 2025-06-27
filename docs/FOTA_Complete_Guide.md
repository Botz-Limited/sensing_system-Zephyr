# FOTA Complete Guide

## Overview

This guide provides comprehensive information about implementing Firmware Over-The-Air (FOTA) updates for the sensing firmware using the SMP (Simple Management Protocol) Proxy approach. This unified approach supports both FOTA updates and file operations through a single, standardized interface.

## Key Benefits

- **Unified Interface**: Single service for both FOTA and file operations
- **Standard Protocol**: Uses MCUmgr/SMP protocol - no custom implementations needed
- **Library Support**: Leverage existing MCUmgr libraries for iOS, Android, and other platforms
- **Future-Proof**: Automatically supports new MCUmgr features as they're added
- **Simplified Development**: Same code works for both primary and secondary devices

## System Architecture

```
┌─────────────────┐
│   Mobile App    │
│  (MCUmgr Lib)   │
└────────┬────────┘
         │ BLE
         │ SMP Protocol
┌────────▼────────┐
│  Primary Device │
│   (SMP Proxy)   │
└────────┬────────┘
         │ UART
         │ SMP Protocol
┌────────▼────────┐
│Secondary Device │
│   (MCUmgr)      │
└─────────────────┘
```

## BLE Service Details

### SMP Proxy Service
- **Service UUID**: `14387800-130c-49e7-b877-2881c89cb258`
- **Characteristic UUID**: `14387802-130c-49e7-b877-2881c89cb258`
- **Properties**: Write Without Response, Notify

This single service handles all MCUmgr operations including:
- Firmware updates (image upload, list, test, confirm, erase)
- File system operations (upload, download, status)
- OS management (echo, reset, taskstat)
- Statistics and shell access

## Implementation Guide

### Mobile App Implementation

The mobile app should use the standard MCUmgr libraries available for each platform:

#### iOS (Swift)
```swift
import iOSMcuManagerLibrary

// Initialize the BLE transporter
let bleTransport = McuMgrBleTransport(cbPeripheral)

// For FOTA updates
let imageManager = ImageManager(transporter: bleTransport)
let firmwareUpgrade = FirmwareUpgradeManager(transporter: bleTransport)

// For file operations
let fileManager = FileSystemManager(transporter: bleTransport)
```

#### Android (Kotlin)
```kotlin
import io.runtime.mcumgr.McuMgrTransport
import io.runtime.mcumgr.ble.McuMgrBleTransport
import io.runtime.mcumgr.managers.ImageManager
import io.runtime.mcumgr.managers.FsManager

// Initialize the BLE transporter
val transport = McuMgrBleTransport(context, bluetoothDevice)

// For FOTA updates
val imageManager = ImageManager(transport)

// For file operations
val fileManager = FsManager(transport)
```

### Firmware Implementation

The firmware automatically handles SMP proxy operations when configured for primary devices. The implementation is transparent to the mobile app.

## FOTA Update Process

### 1. Image Upload
Upload the new firmware image in chunks:
```python
# Example using Python mcumgr library
from smpclient import SMPClient

client = SMPClient(transport)
with open('new_firmware.bin', 'rb') as f:
    client.upload_image(f.read(), slot=1)
```

### 2. Image List
Check available images and their states:
```python
images = client.list_images()
for image in images:
    print(f"Slot {image.slot}: Version {image.version}, Active: {image.active}")
```

### 3. Image Test
Mark the new image for testing on next boot:
```python
client.test_image(hash=new_image_hash)
```

### 4. Reset Device
Trigger a reset to boot into the new image:
```python
client.reset_device()
```

### 5. Image Confirm
After successful testing, confirm the new image:
```python
client.confirm_image()
```

## File Operations

The same SMP proxy service supports file operations:

### Upload File
```python
# Upload a configuration file
with open('config.json', 'rb') as f:
    client.upload_file('/lfs/config.json', f.read())
```

### Download File
```python
# Download a log file
data = client.download_file('/lfs/sensor.log')
with open('sensor.log', 'wb') as f:
    f.write(data)
```

### File Status
```python
# Check file information
status = client.file_status('/lfs/config.json')
print(f"File size: {status.size} bytes")
```

## Testing

### Using mcumgr CLI
Test the implementation using the mcumgr command-line tool:

```bash
# Connect via BLE
mcumgr --conntype ble --connstring peer_name=SensingDevice

# List images
mcumgr image list

# Upload new firmware
mcumgr image upload firmware.bin

# Test the new image
mcumgr image test <hash>

# Reset device
mcumgr reset

# Confirm image after successful boot
mcumgr image confirm
```

### Using Python Test Script
A comprehensive test script is available at `test/smp_proxy_test.py`:

```bash
python test/smp_proxy_test.py --device "Sensing Device"
```

## Best Practices

1. **Chunk Size**: Use appropriate chunk sizes (typically 256-512 bytes) for reliable BLE transmission
2. **Timeouts**: Implement proper timeouts for each operation
3. **Error Handling**: Handle connection losses and retry failed operations
4. **Progress Tracking**: Provide user feedback during long operations like firmware upload
5. **Verification**: Always verify image hash after upload
6. **Testing**: Test the new image before confirming to allow rollback

## Troubleshooting

### Common Issues

1. **Connection Timeouts**
   - Ensure device is advertising
   - Check BLE permissions on mobile device
   - Verify service and characteristic UUIDs

2. **Upload Failures**
   - Reduce chunk size for unstable connections
   - Implement retry logic
   - Check available storage space

3. **Image Not Booting**
   - Verify image is built for correct device
   - Check image signature/validation
   - Ensure bootloader compatibility

### Debug Tools

1. **RTT Logging**: Monitor real-time logs during FOTA process
2. **mcumgr CLI**: Test individual operations
3. **BLE Scanners**: Verify service advertisement

## Migration from Legacy Approach

If you have existing implementations using the legacy FOTA proxy approach, migration is straightforward:

1. Update service UUID to use SMP Proxy service
2. Replace custom protocol code with MCUmgr library calls
3. Remove separate file proxy implementation
4. Test thoroughly with both primary and secondary devices

## Summary

The SMP Proxy approach provides a clean, standardized way to implement FOTA updates and file operations. By leveraging the MCUmgr protocol and existing libraries, development is simplified while maintaining full functionality. This approach is recommended for all new implementations and provides a clear path forward for future enhancements.

For additional MCUmgr features and detailed protocol documentation, refer to the [MCUmgr documentation](https://docs.zephyrproject.org/latest/services/device_mgmt/mcumgr.html).
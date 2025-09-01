# Sensing Firmware

This repository contains the sensing firmware for the nRF5340 Development Kit, implementing a multi-core sensing application with support for primary (right foot) and non-primary device (left foot) configurations.

## Overview

This firmware is designed for the Nordic nRF5340 SoC, utilizing both the application core and network core for efficient sensing operations. The project supports configuration as either a primary or non-primary device in a sensing network.

## Prerequisites

All the necessary tools are installed by the docker file the first time the project is oppened.

- Nordic nRF Connect SDK
- West build tool
- nRF5340 Development Kit
- Python 3.x (for flashing scripts)

## Building the Firmware

To build the firmware, use the following West command:

```bash
west build --build-dir /home/ee/sensing_fw/build /home/ee/sensing_fw/ --board nrf5340dk/nrf5340/cpuapp --sysbuild -- -DCONFIG_PRIMARY_DEVICE=y
```

### Build Configuration Options

- **`-DCONFIG_PRIMARY_DEVICE=y`**: Configures the device as a primary device in the sensing network
- **`-DCONFIG_PRIMARY_DEVICE=n`**: Configures the device as a non-primary (secondary) device

Example for building as a non-primary device:
```bash
west build --build-dir /home/ee/sensing_fw/build /home/ee/sensing_fw/ --board nrf5340dk/nrf5340/cpuapp --sysbuild -- -DCONFIG_PRIMARY_DEVICE=n
```

## Flashing the Firmware

### Automated Flashing (Recommended)

The project includes a convenient script that builds and flashes both cores (application and network cores) automatically:

```bash
./tools/build_flash.sh
```

This script will:
1. Build the firmware for both cores
2. Flash the application core firmware
3. Flash the network core firmware
4. Verify the flashing process

### Manual Flashing

If you prefer to flash manually after building:

```bash
west flash --build-dir /home/ee/sensing_fw/build
```

## Project Structure

```
sensing_fw/
├── src/              # Source code files
├── include/          # Header files
├── boards/           # Board-specific configurations
├── tools/            # Utility scripts
│   └── build_flash.sh    # Build and flash script
├── prj.conf          # Project configuration
├── CMakeLists.txt    # CMake configuration
└── README.md         # This file
```

## Features

- Multi-core architecture utilizing nRF5340's dual-core system
- Configurable primary/non-primary device roles
- Sysbuild integration for coordinated multi-image builds
- Automated build and flash tooling
- FOTA (Firmware Over-The-Air) updates for both primary and secondary devices
- FOTA proxy service for updating secondary devices through the primary

## Configuration

The firmware behavior can be customized through:
- Kconfig options in `prj.conf`
- Device tree overlays in the `boards/` directory
- Build-time configuration flags (like `CONFIG_PRIMARY_DEVICE`)

## Development

### Clean Build

To perform a clean build, remove the build directory first:
```bash
rm -rf /home/ee/sensing_fw/build
```

Then run the build command again.

### Debug Output

Enable debug output by adding the following to your build command:
```bash
-- -DCONFIG_LOG_LEVEL_DBG=y
```

## FOTA (Firmware Over-The-Air) Updates

The system supports FOTA updates for both primary and secondary devices. The primary device can be updated directly from the phone, while the secondary device is updated through the primary device acting as a proxy.

### Architecture Overview

```
┌─────────────┐         ┌─────────────────┐         ┌──────────────────┐
│   Phone     │  BLE    │ Primary Device  │  BLE    │ Secondary Device │
│             │ <-----> │ (Right Foot)    │ <-----> │ (Left Foot)      │
│ Mobile App  │         │ - SMP Server    │         │ - SMP Server     │
│             │         │ - FOTA Proxy    │         │                  │
└─────────────┘         └─────────────────┘         └──────────────────┘
```

### 1. Primary Device FOTA (Direct Update)

The primary device can be updated directly from the phone using the standard MCUmgr protocol over BLE.

#### Prerequisites
- Primary device must be paired/bonded with the phone
- MCUmgr is enabled (already configured in prj.conf)
- Standard SMP BT service is running

#### Mobile App Implementation

**Using nRF Connect Device Manager app (recommended for testing):**
1. Connect to the primary device ("SensingGR")
2. Go to "Image Upload" tab
3. Select your firmware file (app_update.bin)
4. Click "Upload" and monitor progress
5. After upload, click "Test" then "Confirm"
6. Reset the device to apply the update

**Using MCUmgr command line tool:**
```bash
# List current images
mcumgr --conntype ble --connstring peer_name='SensingGR' image list

# Upload new firmware
mcumgr --conntype ble --connstring peer_name='SensingGR' image upload app_update.bin

# Mark image for test
mcumgr --conntype ble --connstring peer_name='SensingGR' image test <hash>

# Reset device
mcumgr --conntype ble --connstring peer_name='SensingGR' reset

# After device boots with new image, confirm it
mcumgr --conntype ble --connstring peer_name='SensingGR' image confirm
```

**Custom Mobile App Integration:**

For iOS (Swift):
```swift
// Use a library like McuManager-iOS
import McuManager

let bleTransport = McuMgrBleTransport(cbPeripheral)
let dfuManager = ImageManager(transporter: bleTransport)

// Upload firmware
dfuManager.upload(imageData: firmwareData) { progress in
    print("Progress: \(progress)%")
}

// Test and reset
dfuManager.test(hash: imageHash)
dfuManager.reset()
```

For Android (Kotlin):
```kotlin
// Use McuManager-Android library
val transport = McuMgrBleTransport(context, bluetoothDevice)
val imageManager = ImageManager(transport)

// Upload firmware
imageManager.upload(firmwareData, callback)

// Test and reset
imageManager.test(imageHash, callback)
imageManager.reset(callback)
```

### 2. Secondary Device FOTA (Proxy Update)

Secondary devices are updated through the primary device using the FOTA proxy service. The phone sends the firmware to the primary device, which forwards it to the secondary device.

#### Prerequisites
- Primary device must be connected to the phone
- Secondary device must be connected to the primary device
- FOTA proxy service is running on the primary device
- Both devices have SMP servers enabled

#### FOTA Proxy Service Details

**Service UUID:** `6e400001-b5a3-f393-e0a9-e50e24dcca9e`

**Characteristics:**

| Characteristic | UUID | Properties | Description |
|----------------|------|------------|-------------|
| Target Select | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | Write | Select update target |
| Command | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | Write, Write No Response | Send commands |
| Data | `6e400004-b5a3-f393-e0a9-e50e24dcca9e` | Write, Write No Response | Send firmware data |
| Status | `6e400005-b5a3-f393-e0a9-e50e24dcca9e` | Read, Notify | Get status updates |

**Target Values:**
- `0x00`: Primary device
- `0x01`: Secondary device
- `0xFF`: All devices

**Commands:**
- `0x01`: Start FOTA (followed by 4 bytes total size, little-endian)
- `0x02`: Data chunk (followed by firmware data)
- `0x03`: End FOTA
- `0x04`: Abort FOTA
- `0x05`: Query status
- `0x06`: Reset device

**Status Values:**
- `0x00`: Idle
- `0x01`: In progress
- `0x02`: Success
- `0x03`: Error
- `0x04`: No target device connected

#### Step-by-Step Update Process

1. **Connect to Primary Device**
   ```python
   # Example using Python bleak library
   device = await BleakScanner.find_device_by_name("SensingGR")
   client = BleakClient(device.address)
   await client.connect()
   ```

2. **Subscribe to Status Notifications**
   ```python
   def status_handler(sender, data):
       status = data[0]
       print(f"Status: {status}")
   
   await client.start_notify(STATUS_UUID, status_handler)
   ```

3. **Set Target to Secondary Device**
   ```python
   await client.write_gatt_char(TARGET_UUID, bytes([0x01]))
   ```

4. **Start FOTA Transfer**
   ```python
   firmware_size = len(firmware_data)
   start_cmd = bytes([0x01]) + struct.pack('<I', firmware_size)
   await client.write_gatt_char(COMMAND_UUID, start_cmd)
   ```

5. **Send Firmware in Chunks**
   ```python
   chunk_size = 240  # Adjust based on MTU
   for i in range(0, len(firmware_data), chunk_size):
       chunk = firmware_data[i:i+chunk_size]
       data_cmd = bytes([0x02]) + chunk
       await client.write_gatt_char(COMMAND_UUID, data_cmd, response=False)
       await asyncio.sleep(0.05)  # Small delay between chunks
   ```

6. **Complete Transfer**
   ```python
   await client.write_gatt_char(COMMAND_UUID, bytes([0x03]))
   ```

7. **Reset Secondary Device**
   ```python
   await client.write_gatt_char(COMMAND_UUID, bytes([0x06]))
   ```

#### Complete Example Scripts

**Python Test Script:**
```bash
# Use the provided test script
python tools/test_fota_proxy.py /path/to/firmware.bin

# Or specify device address
python tools/test_fota_proxy.py /path/to/firmware.bin AA:BB:CC:DD:EE:FF
```

**Mobile App Integration Example (iOS Swift):**
```swift
class SecondaryDeviceFOTA {
    let fotaProxyService = CBUUID(string: "6e400001-b5a3-f393-e0a9-e50e24dcca9e")
    let targetChar = CBUUID(string: "6e400002-b5a3-f393-e0a9-e50e24dcca9e")
    let commandChar = CBUUID(string: "6e400003-b5a3-f393-e0a9-e50e24dcca9e")
    let statusChar = CBUUID(string: "6e400005-b5a3-f393-e0a9-e50e24dcca9e")
    
    func updateSecondaryDevice(peripheral: CBPeripheral, firmware: Data) {
        // 1. Set target to secondary
        peripheral.writeValue(Data([0x01]), for: targetCharacteristic, type: .withResponse)
        
        // 2. Start FOTA
        var startCmd = Data([0x01])
        var size = UInt32(firmware.count).littleEndian
        startCmd.append(Data(bytes: &size, count: 4))
        peripheral.writeValue(startCmd, for: commandCharacteristic, type: .withResponse)
        
        // 3. Send firmware chunks
        let chunkSize = 240
        for i in stride(from: 0, to: firmware.count, by: chunkSize) {
            let chunk = firmware[i..<min(i + chunkSize, firmware.count)]
            var dataCmd = Data([0x02])
            dataCmd.append(chunk)
            peripheral.writeValue(dataCmd, for: commandCharacteristic, type: .withoutResponse)
            Thread.sleep(forTimeInterval: 0.05)
        }
        
        // 4. Complete and reset
        peripheral.writeValue(Data([0x03]), for: commandCharacteristic, type: .withResponse)
        Thread.sleep(forTimeInterval: 1)
        peripheral.writeValue(Data([0x06]), for: commandCharacteristic, type: .withResponse)
    }
}
```

### 3. Updating Both Devices

To update both devices in sequence:

1. First update the secondary device through the proxy
2. Wait for secondary update to complete
3. Switch target to primary device
4. Use standard MCUmgr to update the primary device

```python
# Update secondary first
await client.write_gatt_char(TARGET_UUID, bytes([0x01]))  # Target secondary
# ... perform FOTA update ...

# Then update primary
await client.write_gatt_char(TARGET_UUID, bytes([0x00]))  # Target primary
# ... use standard MCUmgr protocol ...
```

### Building Firmware for FOTA

1. **Build the firmware:**
   ```bash
   west build --build-dir /home/ee/sensing_fw/build /home/ee/sensing_fw/ \
     --board nrf5340dk/nrf5340/cpuapp --sysbuild \
     -- -DCONFIG_PRIMARY_DEVICE=y  # or =n for secondary
   ```

2. **The update file is located at:**
   ```
   /home/ee/sensing_fw/build/zephyr/app_update.bin
   ```

3. **Sign the image (if using secure boot):**
   ```bash
   west sign -t imgtool -- --key root-rsa-2048.pem
   ```

### Monitoring FOTA Progress

**On Primary Device Console:**
```bash
minicom -D /dev/ttyACM0 -b 115200

# Look for these logs:
# For proxy operations:
[INF] FOTA proxy service initialized
[INF] Secondary device connected
[INF] FOTA target set to: SECONDARY
[INF] Starting FOTA update, total size: XXXXX bytes
[INF] FOTA update completed

# For direct updates:
[INF] mcumgr: Upload request received
[INF] mcumgr: Image upload complete
```

**Status Monitoring from Phone:**
- Subscribe to status characteristic for real-time updates
- Query status using command 0x05
- Check for error codes if update fails

### Troubleshooting FOTA Issues

**Primary Device FOTA Issues:**
1. **Connection fails**: Ensure device is bonded/paired
2. **Upload fails**: Check available flash space
3. **Image not confirmed**: Verify image signature and version
4. **Device doesn't reset**: Check reset permissions in prj.conf

**Secondary Device FOTA Issues:**
1. **Status: No Target (0x04)**: Secondary device not connected to primary
2. **Transfer timeout**: Increase delays between chunks or reduce chunk size
3. **Status: Error (0x03)**: Check logs on both devices for specific errors
4. **Update not applied**: Ensure proper reset command is sent

**Common Solutions:**
- Reduce chunk size if experiencing disconnections
- Increase delays between chunks for stability
- Ensure both devices have sufficient battery
- Verify firmware compatibility and signing
- Check that CONFIG_PRIMARY_DEVICE is set correctly for each build

### Security Considerations

1. **Authentication**: Both devices require bonding for FOTA
2. **Encryption**: BLE connection should use encryption (BT_SECURITY_L2)
3. **Image Signing**: Use signed images for production
4. **Access Control**: Implement proper authentication in mobile app

For more implementation details and examples, see [docs/fota_proxy_usage.md](docs/fota_proxy_usage.md)

## Troubleshooting

1. **Build Errors**: Ensure you have the correct nRF Connect SDK version installed and properly configured
2. **Flashing Issues**: Make sure the nRF5340 DK is properly connected and the debugger is recognized
3. **Permission Errors**: The build_flash.sh script may need execute permissions: `chmod +x ./tools/build_flash.sh`
4. **FOTA Issues**: 
   - For primary device: Ensure MCUmgr is properly configured and the device is bonded
   - For secondary device: Verify the device is connected to the primary and the FOTA proxy service is running

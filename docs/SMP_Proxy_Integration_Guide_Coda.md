# SMP Proxy Integration Guide

## Overview

The SMP Proxy service provides a unified interface for mobile applications to access both primary and secondary devices using the standard MCUmgr/SMP protocol. This eliminates the need for custom protocols and allows developers to reuse existing MCUmgr libraries.

## Benefits

1. **Single Protocol** - Use standard MCUmgr for everything
2. **Code Reuse** - Same code works for both devices
3. **Standard Tools** - Compatible with existing MCUmgr tools
4. **Simplified Testing** - Test with standard MCUmgr utilities
5. **Future Proof** - New MCUmgr features work automatically

## Service Definition

### SMP Proxy Service
**UUID:** `8D53DC1E-1DB7-4CD3-868B-8A527460AA84`  
**Availability:** Primary device only

| Characteristic | UUID | Properties | Description |
|----------------|------|------------|-------------|
| Target Select | `DA2E7829-FBCE-4E01-AE9E-261174997C48` | Read/Write | Device selection |
| SMP Data | `DA2E7828-FBCE-4E01-AE9E-261174997C48` | Write/Notify | SMP frames |

### Target Values
- `0x00` - Primary device
- `0x01` - Secondary device

## How It Works

DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Primary",
    "participant Secondary",
    "Note over App: Want to update secondary",
    "App->>Primary: Write Target = 0x01",
    "App->>Primary: Standard SMP command",
    "Primary->>Primary: Check target",
    "Primary->>Secondary: Forward SMP via D2D",
    "Secondary->>Secondary: Process SMP",
    "Secondary-->>Primary: SMP Response",
    "Primary-->>App: Forward Response",
    "Note over App: Looks like direct connection!"
  ),
  "default"
)

## Detailed FOTA Sequences

### FOTA Update - Primary Device

The following sequence shows the exact steps to update the primary device firmware:

DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Primary",
    "Note over App,Primary: Step 1: Set Target to Primary",
    "App->>Primary: Write Target Char = 0x00",
    "Primary-->>App: Write Response",
    "Note over App,Primary: Step 2: Get Current Image Info",
    "App->>Primary: SMP: Image List Request",
    "Primary-->>App: SMP: Image List Response<br/>(slot 0: active, slot 1: empty/old)",
    "Note over App,Primary: Step 3: Upload New Firmware",
    "App->>Primary: SMP: Image Upload Init<br/>(len, sha256, image=1)",
    "Primary-->>App: SMP: Upload Response (offset=0)",
    "loop Upload Chunks",
    "App->>Primary: SMP: Image Upload Data<br/>(offset, data[up to 512 bytes])",
    "Primary-->>App: SMP: Upload Response<br/>(next offset)",
    "end",
    "Note over App,Primary: Step 4: Verify Upload",
    "App->>Primary: SMP: Image List Request",
    "Primary-->>App: SMP: Image List Response<br/>(slot 1: new image, pending)",
    "Note over App,Primary: Step 5: Test New Image",
    "App->>Primary: SMP: Image Test<br/>(hash of new image)",
    "Primary-->>App: SMP: Test Response",
    "Note over App,Primary: Step 6: Reset to Apply",
    "App->>Primary: SMP: OS Reset",
    "Primary-->>App: SMP: Reset Response",
    "Primary->>Primary: Reboot",
    "Note over App,Primary: Step 7: Confirm After Reboot",
    "App->>Primary: SMP: Image Confirm",
    "Primary-->>App: SMP: Confirm Response"
  ),
  "default"
)

### FOTA Update - Secondary Device

The following sequence shows the exact steps to update the secondary device firmware:

DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Primary",
    "participant Secondary",
    "Note over App,Secondary: Step 1: Set Target to Secondary",
    "App->>Primary: Write Target Char = 0x01",
    "Primary-->>App: Write Response",
    "Note over App,Secondary: Step 2: Verify D2D Connection",
    "App->>Primary: SMP: Echo Request",
    "Primary->>Secondary: Forward SMP via UART",
    "Secondary-->>Primary: SMP: Echo Response",
    "Primary-->>App: Forward Response",
    "Note over App,Secondary: Step 3: Get Current Image Info",
    "App->>Primary: SMP: Image List Request",
    "Primary->>Secondary: Forward Request",
    "Secondary-->>Primary: Image List Response",
    "Primary-->>App: Forward Response<br/>(slot 0: active, slot 1: empty/old)",
    "Note over App,Secondary: Step 4: Upload New Firmware",
    "App->>Primary: SMP: Image Upload Init<br/>(len, sha256, image=1)",
    "Primary->>Secondary: Forward Request",
    "Secondary-->>Primary: Upload Response (offset=0)",
    "Primary-->>App: Forward Response",
    "loop Upload Chunks (slower due to UART)",
    "App->>Primary: SMP: Image Upload Data<br/>(offset, data[256 bytes max])",
    "Primary->>Secondary: Forward via UART",
    "Secondary-->>Primary: Upload Response",
    "Primary-->>App: Forward Response",
    "end",
    "Note over App,Secondary: Step 5: Test New Image",
    "App->>Primary: SMP: Image Test",
    "Primary->>Secondary: Forward Request",
    "Secondary-->>Primary: Test Response",
    "Primary-->>App: Forward Response",
    "Note over App,Secondary: Step 6: Reset Secondary",
    "App->>Primary: SMP: OS Reset",
    "Primary->>Secondary: Forward Request",
    "Secondary-->>Primary: Reset Response",
    "Primary-->>App: Forward Response",
    "Secondary->>Secondary: Reboot",
    "Note over App,Secondary: Step 7: Wait and Confirm",
    "Note over App: Wait 5-10 seconds for boot",
    "App->>Primary: SMP: Image Confirm",
    "Primary->>Secondary: Forward Request",
    "Secondary-->>Primary: Confirm Response",
    "Primary-->>App: Forward Response"
  ),
  "default"
)

### Key Differences Between Primary and Secondary FOTA

| Aspect | Primary Device | Secondary Device |
|--------|----------------|------------------|
| Target Value | 0x00 | 0x01 |
| Chunk Size | Up to 512 bytes | 256 bytes recommended |
| Transfer Speed | Fast (BLE direct) | Slower (BLE→UART→Device) |
| Connection Check | Not needed | Echo test recommended |
| Typical Time | 30-60 seconds | 2-5 minutes |
| Error Recovery | BLE retransmit | May need full restart |

## Detailed File Operations

### File Download - Primary Device

DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Primary",
    "Note over App,Primary: Step 1: Set Target to Primary",
    "App->>Primary: Write Target Char = 0x00",
    "Primary-->>App: Write Response",
    "Note over App,Primary: Step 2: Get File Info",
    "App->>Primary: SMP: FS Stat Request<br/>(name=\"/lfs/config.json\")",
    "Primary-->>App: SMP: FS Stat Response<br/>(size=1024, type=file)",
    "Note over App,Primary: Step 3: Download File",
    "App->>Primary: SMP: FS Download Request<br/>(name, offset=0)",
    "Primary-->>App: SMP: FS Download Response<br/>(data[up to 512], len=1024)",
    "loop While offset < len",
    "App->>Primary: SMP: FS Download Request<br/>(name, offset)",
    "Primary-->>App: SMP: FS Download Response<br/>(data chunk, len)",
    "end",
    "Note over App: File download complete"
  ),
  "default"
)

### File Download - Secondary Device

DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Primary",
    "participant Secondary",
    "Note over App,Secondary: Step 1: Set Target to Secondary",
    "App->>Primary: Write Target Char = 0x01",
    "Primary-->>App: Write Response",
    "Note over App,Secondary: Step 2: List Files (Optional)",
    "App->>Primary: SMP: FS List Request<br/>(path=\"/lfs\")",
    "Primary->>Secondary: Forward Request",
    "Secondary-->>Primary: FS List Response",
    "Primary-->>App: Forward Response<br/>(file names and sizes)",
    "Note over App,Secondary: Step 3: Get File Info",
    "App->>Primary: SMP: FS Stat Request<br/>(name=\"/lfs/foot_5.bin\")",
    "Primary->>Secondary: Forward Request",
    "Secondary-->>Primary: FS Stat Response",
    "Primary-->>App: Forward Response<br/>(size=8192, type=file)",
    "Note over App,Secondary: Step 4: Download File",
    "App->>Primary: SMP: FS Download Request<br/>(name, offset=0)",
    "Primary->>Secondary: Forward Request",
    "Secondary-->>Primary: FS Download Response",
    "Primary-->>App: Forward Response<br/>(data[256 bytes], len=8192)",
    "loop While offset < len (slower)",
    "App->>Primary: SMP: FS Download Request<br/>(name, offset)",
    "Primary->>Secondary: Forward via UART",
    "Secondary-->>Primary: FS Download Response",
    "Primary-->>App: Forward Response",
    "end",
    "Note over App: File download complete"
  ),
  "default"
)

### File Upload Sequences

#### Upload to Primary Device
DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Primary",
    "App->>Primary: Write Target Char = 0x00",
    "App->>Primary: SMP: FS Upload Request<br/>(name, len, offset=0, data)",
    "Primary-->>App: SMP: FS Upload Response<br/>(offset=512)",
    "loop Until complete",
    "App->>Primary: SMP: FS Upload Request<br/>(name, len, offset, data)",
    "Primary-->>App: SMP: FS Upload Response<br/>(next offset)",
    "end"
  ),
  "default"
)

#### Upload to Secondary Device
DrawSequenceDiagram(
  Syntax(
    "participant App",
    "participant Primary",
    "participant Secondary",
    "App->>Primary: Write Target Char = 0x01",
    "App->>Primary: SMP: FS Upload Request<br/>(name, len, offset=0, data[256])",
    "Primary->>Secondary: Forward Request",
    "Secondary-->>Primary: FS Upload Response",
    "Primary-->>App: Forward Response<br/>(offset=256)",
    "loop Until complete (slower)",
    "App->>Primary: SMP: FS Upload Request<br/>(offset, data[256])",
    "Primary->>Secondary: Forward via UART",
    "Secondary-->>Primary: FS Upload Response",
    "Primary-->>App: Forward Response",
    "end"
  ),
  "default"
)

## Implementation Details

### SMP Protocol Basics

The SMP (Simple Management Protocol) uses CBOR-encoded messages with the following structure:

```
Header (8 bytes):
- OP (1 byte): Operation (0=read, 1=read_rsp, 2=write, 3=write_rsp)
- Flags (1 byte): Bit 3 = target device (0=primary, 1=secondary)
- Length (2 bytes): Payload length
- Group ID (2 bytes): Command group (1=OS, 2=Image, 8=FS)
- Sequence (1 byte): Message sequence number
- Command ID (1 byte): Specific command

Payload (CBOR encoded):
- Command-specific data
```

### FOTA Implementation Details

#### 1. Image Upload Process

```python
# Example: Upload firmware with progress tracking
async def upload_firmware_with_details(self, firmware_data, target_device):
    """
    Detailed firmware upload implementation
    """
    # Step 1: Calculate firmware hash
    import hashlib
    fw_hash = hashlib.sha256(firmware_data).digest()
    
    # Step 2: Set target device
    await self.select_device(target_device)
    
    # Step 3: Initialize upload
    chunk_size = 256 if target_device == self.TARGET_SECONDARY else 512
    total_size = len(firmware_data)
    
    # Step 4: Send initial upload request
    init_request = {
        "image": 1,  # Slot 1 (inactive slot)
        "len": total_size,
        "sha": fw_hash,
        "off": 0
    }
    
    response = await self.smp_request(
        group=IMG_MGMT_GROUP,  # 0x0001
        command=IMG_UPLOAD,     # 0x0001
        payload=init_request
    )
    
    # Step 5: Upload chunks
    offset = 0
    while offset < total_size:
        chunk = firmware_data[offset:offset + chunk_size]
        
        upload_request = {
            "image": 1,
            "off": offset,
            "data": chunk
        }
        
        response = await self.smp_request(
            group=IMG_MGMT_GROUP,
            command=IMG_UPLOAD,
            payload=upload_request
        )
        
        # Verify response
        if response.get("off") != offset + len(chunk):
            raise Exception(f"Upload error at offset {offset}")
        
        offset += len(chunk)
        progress = (offset * 100) // total_size
        print(f"Upload progress: {progress}%")
    
    return fw_hash
```

#### 2. Image Management Commands

```swift
// Swift implementation for image management
extension UnifiedDeviceManager {
    
    // List all images on device
    func listImages(target: DeviceTarget) async throws -> [ImageInfo] {
        selectDevice(target)
        
        let request = SMPMessage(
            op: .read,
            flags: target == .secondary ? 0x08 : 0x00,
            group: .imageMgmt,
            command: .imageList
        )
        
        let response = try await sendSMPRequest(request)
        return parseImageList(response)
    }
    
    // Test image (mark for next boot)
    func testImage(hash: Data, target: DeviceTarget) async throws {
        selectDevice(target)
        
        let request = SMPMessage(
            op: .write,
            flags: target == .secondary ? 0x08 : 0x00,
            group: .imageMgmt,
            command: .imageTest,
            payload: ["hash": hash]
        )
        
        try await sendSMPRequest(request)
    }
    
    // Confirm image (make permanent)
    func confirmImage(target: DeviceTarget) async throws {
        selectDevice(target)
        
        let request = SMPMessage(
            op: .write,
            flags: target == .secondary ? 0x08 : 0x00,
            group: .imageMgmt,
            command: .imageConfirm,
            payload: [:] // Empty payload
        )
        
        try await sendSMPRequest(request)
    }
    
    // Reset device
    func resetDevice(target: DeviceTarget) async throws {
        selectDevice(target)
        
        let request = SMPMessage(
            op: .write,
            flags: target == .secondary ? 0x08 : 0x00,
            group: .osMgmt,
            command: .reset,
            payload: [:] // Empty payload
        )
        
        try await sendSMPRequest(request)
    }
}
```

### File Operation Implementation Details

#### 1. File Download with Resume Support

```kotlin
// Kotlin implementation with resume support
class FileDownloadManager(private val deviceManager: UnifiedDeviceManager) {
    
    suspend fun downloadFileWithResume(
        path: String,
        target: Byte,
        localFile: File
    ): ByteArray {
        // Check if partial download exists
        val existingSize = if (localFile.exists()) localFile.length() else 0L
        
        // Get file info from device
        val fileInfo = deviceManager.getFileInfo(path, target)
        val totalSize = fileInfo.size
        
        if (existingSize >= totalSize) {
            // File already complete
            return localFile.readBytes()
        }
        
        // Resume from existing offset
        var offset = existingSize.toInt()
        val chunkSize = if (target == TARGET_SECONDARY) 256 else 512
        
        FileOutputStream(localFile, true).use { output ->
            while (offset < totalSize) {
                val response = deviceManager.downloadChunk(
                    path = path,
                    offset = offset,
                    target = target
                )
                
                val data = response.data
                output.write(data)
                
                offset += data.size
                val progress = (offset * 100) / totalSize
                Log.d(TAG, "Download progress: $progress%")
                
                // Handle connection loss
                if (data.isEmpty()) {
                    throw IOException("Connection lost at offset $offset")
                }
            }
        }
        
        return localFile.readBytes()
    }
}
```

#### 2. File Upload with Verification

```python
async def upload_file_with_verification(self, local_path, remote_path, target):
    """
    Upload file with hash verification
    """
    import hashlib
    
    # Read file and calculate hash
    with open(local_path, 'rb') as f:
        file_data = f.read()
    
    file_hash = hashlib.sha256(file_data).hexdigest()
    file_size = len(file_data)
    
    # Set target
    await self.select_device(target)
    
    # Upload file
    chunk_size = 256 if target == self.TARGET_SECONDARY else 512
    offset = 0
    
    while offset < file_size:
        chunk = file_data[offset:offset + chunk_size]
        
        request = {
            "name": remote_path,
            "off": offset,
            "data": chunk,
            "len": file_size if offset == 0 else None  # Only send on first chunk
        }
        
        response = await self.smp_request(
            group=FS_MGMT_GROUP,  # 0x0008
            command=FS_UPLOAD,     # 0x0001
            payload=request
        )
        
        offset = response.get("off", offset + len(chunk))
    
    # Verify upload by downloading hash
    hash_response = await self.smp_request(
        group=FS_MGMT_GROUP,
        command=FS_HASH_CHECKSUM,  # 0x0003
        payload={"name": remote_path, "type": "sha256"}
    )
    
    if hash_response.get("output") != file_hash:
        raise Exception("File verification failed")
    
    return True
```

### Error Handling and Recovery

#### Connection Loss During FOTA

```swift
func performFOTAWithRecovery(firmware: Data, target: DeviceTarget) async throws {
    var lastOffset = 0
    var retryCount = 0
    let maxRetries = 3
    
    while retryCount < maxRetries {
        do {
            // Check current upload status
            let status = try await getUploadStatus(target: target)
            if let offset = status?.offset {
                lastOffset = offset
                print("Resuming from offset \(offset)")
            }
            
            // Continue upload from last offset
            try await resumeUpload(
                firmware: firmware,
                fromOffset: lastOffset,
                target: target
            )
            
            // Upload complete, proceed with test
            try await testImage(hash: firmware.sha256(), target: target)
            try await resetDevice(target: target)
            
            // Wait for device to reboot
            try await Task.sleep(nanoseconds: 10_000_000_000) // 10 seconds
            
            // Reconnect and confirm
            try await reconnect()
            try await confirmImage(target: target)
            
            break // Success
            
        } catch {
            retryCount += 1
            print("FOTA failed (attempt \(retryCount)): \(error)")
            
            if retryCount >= maxRetries {
                throw FOTAError.maxRetriesExceeded
            }
            
            // Wait before retry
            try await Task.sleep(nanoseconds: 2_000_000_000) // 2 seconds
        }
    }
}
```

### Best Practices

#### 1. Chunk Size Selection
```python
def get_optimal_chunk_size(target, connection_quality):
    """
    Select chunk size based on target and connection
    """
    if target == TARGET_PRIMARY:
        # Direct BLE connection
        if connection_quality == "excellent":
            return 512  # Maximum
        elif connection_quality == "good":
            return 384
        else:
            return 256  # Conservative
    else:
        # Secondary via UART
        if connection_quality == "excellent":
            return 256  # UART limited
        else:
            return 128  # Very conservative
```

#### 2. Progress Tracking
```kotlin
class ProgressTracker {
    private val startTime = System.currentTimeMillis()
    private var lastUpdate = startTime
    
    fun updateProgress(current: Int, total: Int) {
        val progress = (current * 100) / total
        val elapsed = System.currentTimeMillis() - startTime
        val rate = if (elapsed > 0) (current * 1000) / elapsed else 0
        val remaining = if (rate > 0) ((total - current) * 1000) / rate else 0
        
        // Update UI every second
        val now = System.currentTimeMillis()
        if (now - lastUpdate >= 1000) {
            lastUpdate = now
            
            Log.d(TAG, "Progress: $progress%")
            Log.d(TAG, "Rate: ${rate / 1024} KB/s")
            Log.d(TAG, "Remaining: ${remaining / 1000} seconds")
            
            // Notify UI
            progressCallback?.invoke(progress, rate, remaining)
        }
    }
}
```

## Mobile Integration

### iOS Swift Example

```swift
import CoreBluetooth
import McuManager

class UnifiedDeviceManager {
    let smpProxyService = CBUUID(string: "8D53DC1E-1DB7-4CD3-868B-8A527460AA84")
    let targetCharUUID = CBUUID(string: "DA2E7829-FBCE-4E01-AE9E-261174997C48")
    let smpDataCharUUID = CBUUID(string: "DA2E7828-FBCE-4E01-AE9E-261174997C48")
    
    var peripheral: CBPeripheral!
    var targetChar: CBCharacteristic?
    var smpDataChar: CBCharacteristic?
    var dfuManager: FirmwareUpgradeManager?
    
    enum DeviceTarget: UInt8 {
        case primary = 0x00
        case secondary = 0x01
    }
    
    // Select which device to communicate with
    func selectDevice(_ target: DeviceTarget) {
        guard let targetChar = targetChar else { return }
        let data = Data([target.rawValue])
        peripheral.writeValue(data, for: targetChar, type: .withResponse)
    }
    
    // Update firmware on any device
    func updateFirmware(_ firmware: Data, target: DeviceTarget) {
        // Select target
        selectDevice(target)
        
        // Use standard MCUmgr - it doesn't know about proxy!
        let transport = McuMgrBleTransport(peripheral)
        transport.smpCharacteristic = smpDataChar // Use proxy characteristic
        
        dfuManager = FirmwareUpgradeManager(transporter: transport)
        dfuManager?.delegate = self
        dfuManager?.start(data: firmware)
    }
    
    // List files on any device
    func listFiles(path: String = "/lfs", target: DeviceTarget) async throws -> [FileInfo] {
        // Select target
        selectDevice(target)
        
        // Use standard file manager
        let transport = McuMgrBleTransport(peripheral)
        transport.smpCharacteristic = smpDataChar
        
        let fileManager = FileSystemManager(transporter: transport)
        return try await fileManager.list(path)
    }
    
    // Download file from any device
    func downloadFile(_ path: String, target: DeviceTarget) async throws -> Data {
        // Select target
        selectDevice(target)
        
        // Use standard file manager
        let transport = McuMgrBleTransport(peripheral)
        transport.smpCharacteristic = smpDataChar
        
        let fileManager = FileSystemManager(transporter: transport)
        return try await fileManager.download(path)
    }
}

// Usage example
let manager = UnifiedDeviceManager()

// Update secondary device
await manager.updateFirmware(secondaryFirmware, target: .secondary)

// List files on secondary
let files = try await manager.listFiles(target: .secondary)

// Download log from secondary
let logData = try await manager.downloadFile("/lfs/foot_5.bin", target: .secondary)
```

### Android Kotlin Example

```kotlin
import io.runtime.mcumgr.McuMgrTransport
import io.runtime.mcumgr.ble.McuMgrBleTransport
import io.runtime.mcumgr.dfu.FirmwareUpgradeManager
import io.runtime.mcumgr.managers.FsManager

class UnifiedDeviceManager(private val context: Context) {
    companion object {
        const val SMP_PROXY_SERVICE = "8D53DC1E-1DB7-4CD3-868B-8A527460AA84"
        const val TARGET_CHAR = "DA2E7829-FBCE-4E01-AE9E-261174997C48"
        const val SMP_DATA_CHAR = "DA2E7828-FBCE-4E01-AE9E-261174997C48"
        
        const val TARGET_PRIMARY: Byte = 0x00
        const val TARGET_SECONDARY: Byte = 0x01
    }
    
    private var gatt: BluetoothGatt? = null
    private var targetChar: BluetoothGattCharacteristic? = null
    private var smpDataChar: BluetoothGattCharacteristic? = null
    
    // Select device target
    private fun selectDevice(target: Byte) {
        targetChar?.let { char ->
            char.value = byteArrayOf(target)
            gatt?.writeCharacteristic(char)
        }
    }
    
    // Create transport that uses proxy
    private fun createProxyTransport(): McuMgrTransport {
        return McuMgrBleTransport(context, gatt!!).apply {
            // Override to use proxy characteristic
            setSmpCharacteristic(smpDataChar)
        }
    }
    
    // Update firmware on any device
    fun updateFirmware(firmware: ByteArray, target: Byte) {
        selectDevice(target)
        
        val transport = createProxyTransport()
        val dfuManager = FirmwareUpgradeManager(transport, null)
        
        dfuManager.setUploadCallback(object : FirmwareUpgradeManager.UploadCallback {
            override fun onUploadProgressChanged(current: Int, total: Int, timestamp: Long) {
                val progress = (current * 100) / total
                Log.d(TAG, "Upload progress: $progress%")
            }
            
            override fun onUploadCompleted() {
                Log.d(TAG, "Upload completed!")
            }
            
            override fun onUploadFailed(error: McuMgrException) {
                Log.e(TAG, "Upload failed: $error")
            }
        })
        
        dfuManager.start(firmware)
    }
    
    // List files on any device
    suspend fun listFiles(path: String = "/lfs", target: Byte): List<FileInfo> {
        selectDevice(target)
        
        val transport = createProxyTransport()
        val fsManager = FsManager(transport)
        
        return fsManager.list(path)
    }
    
    // Download file from any device
    suspend fun downloadFile(path: String, target: Byte): ByteArray {
        selectDevice(target)
        
        val transport = createProxyTransport()
        val fsManager = FsManager(transport)
        
        return fsManager.download(path)
    }
}

// Usage
val manager = UnifiedDeviceManager(context)

// Update secondary device
manager.updateFirmware(secondaryFirmware, TARGET_SECONDARY)

// List files on secondary
val files = manager.listFiles(target = TARGET_SECONDARY)

// Download log from secondary
val logData = manager.downloadFile("/lfs/foot_5.bin", TARGET_SECONDARY)
```

### Python Example

```python
from smpclient import SMPClient
from smpclient.transport.ble import SMPBLETransport
import asyncio
from bleak import BleakClient

class UnifiedDeviceManager:
    SMP_PROXY_SERVICE = "8D53DC1E-1DB7-4CD3-868B-8A527460AA84"
    TARGET_CHAR = "DA2E7829-FBCE-4E01-AE9E-261174997C48"
    SMP_DATA_CHAR = "DA2E7828-FBCE-4E01-AE9E-261174997C48"
    
    TARGET_PRIMARY = 0x00
    TARGET_SECONDARY = 0x01
    
    def __init__(self, address):
        self.address = address
        self.client = None
        
    async def connect(self):
        self.client = BleakClient(self.address)
        await self.client.connect()
        
    async def select_device(self, target):
        """Select which device to communicate with"""
        await self.client.write_gatt_char(self.TARGET_CHAR, bytes([target]))
        
    async def create_smp_client(self, target):
        """Create SMP client for the specified target"""
        await self.select_device(target)
        
        # Create transport that uses proxy characteristic
        transport = SMPBLETransport(self.client)
        transport.smp_char_uuid = self.SMP_DATA_CHAR
        await transport.connect()
        
        return SMPClient(transport)
        
    async def update_firmware(self, firmware_path, target):
        """Update firmware on any device"""
        smp = await self.create_smp_client(target)
        
        with open(firmware_path, 'rb') as f:
            firmware = f.read()
            
        # Upload firmware
        async for progress in smp.image_upload(firmware):
            if isinstance(progress, dict):
                percent = (progress['off'] * 100) // len(firmware)
                print(f"Progress: {percent}%")
                
        # Confirm and reset
        await smp.image_confirm()
        await smp.os_reset()
        
    async def list_files(self, path="/lfs", target=TARGET_PRIMARY):
        """List files on any device"""
        smp = await self.create_smp_client(target)
        return await smp.fs_list(path)
        
    async def download_file(self, path, target=TARGET_PRIMARY):
        """Download file from any device"""
        smp = await self.create_smp_client(target)
        
        data = bytearray()
        offset = 0
        
        while True:
            response = await smp.fs_download(path, offset)
            if 'data' in response:
                chunk = response['data']
                data.extend(chunk)
                offset += len(chunk)
                
                if response.get('len', 0) <= offset:
                    break
                    
        return bytes(data)

# Usage
async def main():
    manager = UnifiedDeviceManager("XX:XX:XX:XX:XX:XX")
    await manager.connect()
    
    # Update secondary device
    await manager.update_firmware("secondary.bin", manager.TARGET_SECONDARY)
    
    # List files on secondary
    files = await manager.list_files(target=manager.TARGET_SECONDARY)
    
    # Download log from secondary
    data = await manager.download_file("/lfs/foot_5.bin", manager.TARGET_SECONDARY)
```

## Comparison with Previous Approach

| Feature | Old Proxy Services | SMP Proxy |
|---------|-------------------|-----------|
| Protocol | Custom | Standard MCUmgr |
| Mobile code | Different for each proxy | Same for everything |
| Libraries | Custom implementation | Existing MCUmgr libs |
| Testing | Custom scripts | Standard tools |
| Documentation | Extensive | "Use MCUmgr + set target" |
| Future features | Need proxy updates | Work automatically |

## Migration Guide

### From FOTA Proxy

```swift
// Old approach
fotaProxy.setTarget(.secondary)
fotaProxy.startUpdate(size: firmware.count)
for chunk in firmware.chunks {
    fotaProxy.sendData(chunk)
}
fotaProxy.endUpdate()

// New approach
selectDevice(.secondary)
dfuManager.start(data: firmware) // Standard MCUmgr!
```

### From File Proxy

```swift
// Old approach
fileProxy.setTarget(.secondary)
fileProxy.sendCommand(.listFiles)
// Wait for custom response format

// New approach
selectDevice(.secondary)
let files = try await fileManager.list("/lfs") // Standard MCUmgr!
```

## Testing

Use the provided test script to verify functionality:

```bash
# Install dependencies
pip install smpclient bleak

# Test unified access
python tools/test_smp_proxy.py demo

# List images on both devices
python tools/test_smp_proxy.py images 0  # Primary
python tools/test_smp_proxy.py images 1  # Secondary

# Update secondary device
python tools/test_smp_proxy.py upload 1 secondary_firmware.bin

# List files on secondary
python tools/test_smp_proxy.py files 1 /lfs

# Download file from secondary
python tools/test_smp_proxy.py download 1 /lfs/foot_5.bin
```

## Troubleshooting

### "No SMP response"
- Ensure target is set before sending commands
- Check that secondary device is connected (D2D)
- Verify SMP service is running on secondary

### "Invalid target"
- Target must be 0x00 (primary) or 0x01 (secondary)
- Secondary must be connected for target 0x01

### "Command not supported"
- Not all MCUmgr commands may be implemented
- Check firmware version supports the command

## Summary

The SMP Proxy service dramatically simplifies mobile app development by:
1. Allowing standard MCUmgr libraries to work with both devices
2. Eliminating the need for custom protocol implementations
3. Providing a consistent interface for all operations
4. Enabling use of existing MCUmgr tools and documentation

Mobile developers can now treat both devices as if they have direct connections, with the firmware handling all the complexity of D2D communication transparently.

> **Note**: For historical reference, the legacy FOTA proxy implementation is documented in [FOTA_Legacy_Guide.md](FOTA_Legacy_Guide.md). However, all new development should use the SMP Proxy approach described in this guide.
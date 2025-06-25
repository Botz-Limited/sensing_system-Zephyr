#!/usr/bin/env python3
"""
Test script for SMP Proxy functionality
This demonstrates how to use standard MCUmgr for both primary and secondary devices
Requires: pip install smpclient bleak asyncio
"""

import asyncio
import sys
import os
from bleak import BleakClient, BleakScanner
import logging
from smpclient import SMPClient
from smpclient.transport.ble import SMPBLETransport

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# SMP Proxy Service UUIDs (slightly different from standard SMP)
SMP_PROXY_SERVICE = "8D53DC1E-1DB7-4CD3-868B-8A527460AA84"
TARGET_CHAR = "DA2E7829-FBCE-4E01-AE9E-261174997C48"
SMP_DATA_CHAR = "DA2E7828-FBCE-4E01-AE9E-261174997C48"

# Standard SMP Service UUIDs (for comparison)
STANDARD_SMP_SERVICE = "8D53DC1D-1DB7-4CD3-868B-8A527460AA84"
STANDARD_SMP_CHAR = "DA2E7828-FBCE-4E01-AE9E-261174997C48"

# Target device types
TARGET_PRIMARY = 0x00
TARGET_SECONDARY = 0x01

class SMPProxyTransport(SMPBLETransport):
    """Custom transport that uses SMP Proxy service instead of standard SMP"""
    
    def __init__(self, client, target=TARGET_PRIMARY):
        super().__init__(client)
        self.target = target
        self.proxy_mode = True
        
    async def connect(self):
        """Override connect to use proxy service"""
        if not self.client.is_connected:
            await self.client.connect()
            
        # Set target device
        await self.client.write_gatt_char(TARGET_CHAR, bytes([self.target]))
        logger.info(f"Set SMP proxy target to: {'PRIMARY' if self.target == TARGET_PRIMARY else 'SECONDARY'}")
        
        # Use proxy data characteristic instead of standard SMP
        self.smp_char_uuid = SMP_DATA_CHAR
        
        # Subscribe to notifications
        await self.client.start_notify(self.smp_char_uuid, self._notification_handler)
        logger.info("Connected to SMP proxy service")

class UnifiedSMPClient:
    """Unified SMP client that can work with both primary and secondary devices"""
    
    def __init__(self, address):
        self.address = address
        self.client = None
        self.smp_client = None
        self.has_proxy = False
        
    async def connect(self):
        """Connect and detect available services"""
        self.client = BleakClient(self.address)
        await self.client.connect()
        logger.info(f"Connected to {self.address}")
        
        # Check which services are available
        services = await self.client.get_services()
        
        for service in services:
            if service.uuid.lower() == SMP_PROXY_SERVICE.lower():
                self.has_proxy = True
                logger.info("Found SMP Proxy service - can access both devices!")
            elif service.uuid.lower() == STANDARD_SMP_SERVICE.lower():
                logger.info("Found standard SMP service")
                
    async def disconnect(self):
        """Disconnect from device"""
        if self.client:
            await self.client.disconnect()
            
    async def create_smp_client(self, target=TARGET_PRIMARY):
        """Create SMP client for the specified target"""
        if self.has_proxy:
            # Use proxy transport
            transport = SMPProxyTransport(self.client, target)
            logger.info(f"Using SMP Proxy for {'PRIMARY' if target == TARGET_PRIMARY else 'SECONDARY'} device")
        else:
            # Use standard transport
            transport = SMPBLETransport(self.client)
            logger.info("Using standard SMP transport")
            
        await transport.connect()
        self.smp_client = SMPClient(transport)
        return self.smp_client
        
    async def get_device_info(self, target=TARGET_PRIMARY):
        """Get device information using os_mgmt echo command"""
        smp = await self.create_smp_client(target)
        
        # Send echo command to test connectivity
        response = await smp.os_echo("Hello from mobile!")
        logger.info(f"Echo response: {response}")
        
        # Get mcumgr parameters
        response = await smp.os_mgmt_get_mcumgr_params()
        logger.info(f"MCUmgr params: {response}")
        
        return response
        
    async def list_images(self, target=TARGET_PRIMARY):
        """List firmware images on device"""
        smp = await self.create_smp_client(target)
        
        response = await smp.image_list()
        logger.info(f"Images on {'PRIMARY' if target == TARGET_PRIMARY else 'SECONDARY'}:")
        
        for slot, images in response.items():
            for img in images:
                logger.info(f"  Slot {slot}: version={img.get('version', 'unknown')}, "
                          f"hash={img.get('hash', 'unknown')[:16]}...")
                          
        return response
        
    async def upload_firmware(self, firmware_path, target=TARGET_PRIMARY):
        """Upload firmware to device"""
        if not os.path.exists(firmware_path):
            logger.error(f"Firmware file not found: {firmware_path}")
            return False
            
        with open(firmware_path, 'rb') as f:
            firmware_data = f.read()
            
        logger.info(f"Firmware size: {len(firmware_data)} bytes")
        
        smp = await self.create_smp_client(target)
        
        # Upload image
        logger.info(f"Uploading firmware to {'PRIMARY' if target == TARGET_PRIMARY else 'SECONDARY'}...")
        
        async for progress in smp.image_upload(firmware_data):
            if isinstance(progress, dict):
                offset = progress.get('off', 0)
                total = len(firmware_data)
                percent = (offset * 100) // total
                logger.info(f"Upload progress: {percent}% ({offset}/{total} bytes)")
                
        logger.info("Upload complete!")
        
        # Confirm image
        logger.info("Confirming image...")
        await smp.image_confirm()
        
        # Reset device
        logger.info("Resetting device...")
        await smp.os_reset()
        
        return True
        
    async def list_files(self, path="/lfs", target=TARGET_PRIMARY):
        """List files on device"""
        smp = await self.create_smp_client(target)
        
        response = await smp.fs_list(path)
        logger.info(f"Files on {'PRIMARY' if target == TARGET_PRIMARY else 'SECONDARY'} at {path}:")
        
        for entry in response.get('entries', []):
            name = entry.get('name', 'unknown')
            size = entry.get('size', 0)
            logger.info(f"  {name} - {size} bytes")
            
        return response
        
    async def download_file(self, path, target=TARGET_PRIMARY):
        """Download file from device"""
        smp = await self.create_smp_client(target)
        
        logger.info(f"Downloading {path} from {'PRIMARY' if target == TARGET_PRIMARY else 'SECONDARY'}...")
        
        data = bytearray()
        offset = 0
        
        while True:
            response = await smp.fs_download(path, offset)
            
            if 'data' in response:
                chunk = response['data']
                data.extend(chunk)
                offset += len(chunk)
                logger.info(f"Downloaded {offset} bytes...")
                
                if response.get('len', 0) <= offset:
                    break
            else:
                break
                
        logger.info(f"Download complete: {len(data)} bytes")
        return bytes(data)

async def scan_for_device(name_prefix="SensingGR"):
    """Scan for primary device"""
    logger.info(f"Scanning for devices with name starting with '{name_prefix}'...")
    
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name and device.name.startswith(name_prefix):
            logger.info(f"Found device: {device.name} ({device.address})")
            return device.address
            
    logger.error(f"No device found with name starting with '{name_prefix}'")
    return None

async def demo_unified_access():
    """Demonstrate unified access to both devices"""
    # Find device
    device_address = await scan_for_device()
    if not device_address:
        return
        
    client = UnifiedSMPClient(device_address)
    
    try:
        await client.connect()
        
        if client.has_proxy:
            logger.info("\n=== Demonstrating Unified SMP Access ===")
            
            # Access primary device
            logger.info("\n--- Primary Device ---")
            await client.get_device_info(TARGET_PRIMARY)
            await client.list_images(TARGET_PRIMARY)
            await client.list_files("/lfs", TARGET_PRIMARY)
            
            # Access secondary device  
            logger.info("\n--- Secondary Device ---")
            await client.get_device_info(TARGET_SECONDARY)
            await client.list_images(TARGET_SECONDARY)
            await client.list_files("/lfs", TARGET_SECONDARY)
            
            logger.info("\n=== Success! Same MCUmgr code works for both devices! ===")
        else:
            logger.info("Device doesn't have SMP proxy - using standard SMP")
            await client.get_device_info()
            
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        await client.disconnect()

async def main():
    """Main test function"""
    if len(sys.argv) < 2:
        print("Usage: python test_smp_proxy.py <command> [args]")
        print("Commands:")
        print("  demo              - Demonstrate unified access")
        print("  info <target>     - Get device info (0=primary, 1=secondary)")
        print("  images <target>   - List images")
        print("  upload <target> <file> - Upload firmware")
        print("  files <target> [path]  - List files")
        print("  download <target> <path> - Download file")
        sys.exit(1)
        
    command = sys.argv[1]
    
    # Find device
    device_address = await scan_for_device()
    if not device_address:
        sys.exit(1)
        
    client = UnifiedSMPClient(device_address)
    
    try:
        await client.connect()
        
        if command == "demo":
            await demo_unified_access()
            
        elif command == "info" and len(sys.argv) >= 3:
            target = int(sys.argv[2])
            await client.get_device_info(target)
            
        elif command == "images" and len(sys.argv) >= 3:
            target = int(sys.argv[2])
            await client.list_images(target)
            
        elif command == "upload" and len(sys.argv) >= 4:
            target = int(sys.argv[2])
            firmware_file = sys.argv[3]
            await client.upload_firmware(firmware_file, target)
            
        elif command == "files" and len(sys.argv) >= 3:
            target = int(sys.argv[2])
            path = sys.argv[3] if len(sys.argv) >= 4 else "/lfs"
            await client.list_files(path, target)
            
        elif command == "download" and len(sys.argv) >= 4:
            target = int(sys.argv[2])
            path = sys.argv[3]
            data = await client.download_file(path, target)
            
            # Save to local file
            filename = os.path.basename(path)
            with open(filename, 'wb') as f:
                f.write(data)
            logger.info(f"Saved to {filename}")
            
        else:
            print("Invalid command or missing arguments")
            
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        await client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
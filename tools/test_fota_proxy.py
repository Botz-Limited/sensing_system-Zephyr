#!/usr/bin/env python3
"""
Test script for FOTA proxy functionality
Requires: pip install bleak asyncio
"""

import asyncio
import struct
import sys
import os
from bleak import BleakClient, BleakScanner
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# FOTA Proxy Service UUIDs
FOTA_PROXY_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
TARGET_CHAR = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
COMMAND_CHAR = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
DATA_CHAR = "6e400004-b5a3-f393-e0a9-e50e24dcca9e"
STATUS_CHAR = "6e400005-b5a3-f393-e0a9-e50e24dcca9e"

# Target device types
TARGET_PRIMARY = 0x00
TARGET_SECONDARY = 0x01
TARGET_ALL = 0xFF

# Commands
CMD_START = 0x01
CMD_DATA = 0x02
CMD_END = 0x03
CMD_ABORT = 0x04
CMD_STATUS = 0x05
CMD_RESET = 0x06

# Status codes
STATUS_IDLE = 0x00
STATUS_IN_PROGRESS = 0x01
STATUS_SUCCESS = 0x02
STATUS_ERROR = 0x03
STATUS_NO_TARGET = 0x04

status_names = {
    STATUS_IDLE: "Idle",
    STATUS_IN_PROGRESS: "In Progress",
    STATUS_SUCCESS: "Success",
    STATUS_ERROR: "Error",
    STATUS_NO_TARGET: "No Target Device"
}

class FOTAProxyClient:
    def __init__(self, address):
        self.address = address
        self.client = None
        self.status = STATUS_IDLE
        
    async def connect(self):
        """Connect to the primary device"""
        logger.info(f"Connecting to {self.address}...")
        self.client = BleakClient(self.address)
        await self.client.connect()
        logger.info("Connected!")
        
        # Subscribe to status notifications
        await self.client.start_notify(STATUS_CHAR, self._status_handler)
        
    async def disconnect(self):
        """Disconnect from the device"""
        if self.client:
            await self.client.disconnect()
            logger.info("Disconnected")
            
    def _status_handler(self, sender, data):
        """Handle status notifications"""
        if len(data) > 0:
            self.status = data[0]
            logger.info(f"Status update: {status_names.get(self.status, 'Unknown')}")
            
    async def set_target(self, target):
        """Set the target device for FOTA"""
        target_name = {
            TARGET_PRIMARY: "Primary",
            TARGET_SECONDARY: "Secondary",
            TARGET_ALL: "All devices"
        }.get(target, "Unknown")
        
        logger.info(f"Setting target to: {target_name}")
        await self.client.write_gatt_char(TARGET_CHAR, bytes([target]))
        
    async def query_status(self):
        """Query current FOTA status"""
        await self.client.write_gatt_char(COMMAND_CHAR, bytes([CMD_STATUS]))
        await asyncio.sleep(0.5)
        return self.status
        
    async def update_firmware(self, firmware_path, target=TARGET_SECONDARY):
        """Perform FOTA update"""
        # Read firmware file
        if not os.path.exists(firmware_path):
            logger.error(f"Firmware file not found: {firmware_path}")
            return False
            
        with open(firmware_path, 'rb') as f:
            firmware_data = f.read()
            
        logger.info(f"Firmware size: {len(firmware_data)} bytes")
        
        # Set target
        await self.set_target(target)
        await asyncio.sleep(0.5)
        
        # Start FOTA
        logger.info("Starting FOTA update...")
        start_cmd = bytes([CMD_START]) + struct.pack('<I', len(firmware_data))
        await self.client.write_gatt_char(COMMAND_CHAR, start_cmd)
        await asyncio.sleep(0.5)
        
        # Send firmware in chunks
        chunk_size = 240  # Adjust based on MTU
        total_chunks = (len(firmware_data) + chunk_size - 1) // chunk_size
        
        logger.info(f"Sending {total_chunks} chunks...")
        for i in range(0, len(firmware_data), chunk_size):
            chunk_num = i // chunk_size + 1
            chunk = firmware_data[i:i+chunk_size]
            data_cmd = bytes([CMD_DATA]) + chunk
            
            await self.client.write_gatt_char(COMMAND_CHAR, data_cmd, response=False)
            
            # Progress update
            if chunk_num % 10 == 0:
                progress = (i + len(chunk)) * 100 // len(firmware_data)
                logger.info(f"Progress: {progress}% ({chunk_num}/{total_chunks} chunks)")
                
            await asyncio.sleep(0.05)  # Small delay between chunks
            
        # End FOTA
        logger.info("Completing FOTA update...")
        await self.client.write_gatt_char(COMMAND_CHAR, bytes([CMD_END]))
        
        # Wait for completion
        for _ in range(10):  # Wait up to 10 seconds
            await asyncio.sleep(1)
            if self.status == STATUS_SUCCESS:
                logger.info("FOTA update completed successfully!")
                return True
            elif self.status == STATUS_ERROR:
                logger.error("FOTA update failed!")
                return False
                
        logger.warning("FOTA update timed out")
        return False
        
    async def reset_device(self, target=TARGET_SECONDARY):
        """Reset the target device"""
        await self.set_target(target)
        await asyncio.sleep(0.5)
        
        logger.info(f"Resetting device...")
        await self.client.write_gatt_char(COMMAND_CHAR, bytes([CMD_RESET]))
        
    async def abort_update(self):
        """Abort ongoing FOTA update"""
        logger.info("Aborting FOTA update...")
        await self.client.write_gatt_char(COMMAND_CHAR, bytes([CMD_ABORT]))

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

async def main():
    """Main test function"""
    if len(sys.argv) < 2:
        print("Usage: python test_fota_proxy.py <firmware_file> [device_address]")
        print("If device_address is not provided, will scan for 'SensingGR'")
        sys.exit(1)
        
    firmware_file = sys.argv[1]
    
    # Get device address
    if len(sys.argv) >= 3:
        device_address = sys.argv[2]
    else:
        device_address = await scan_for_device()
        if not device_address:
            sys.exit(1)
    
    # Create FOTA proxy client
    client = FOTAProxyClient(device_address)
    
    try:
        # Connect to device
        await client.connect()
        
        # Query initial status
        status = await client.query_status()
        logger.info(f"Initial status: {status_names.get(status, 'Unknown')}")
        
        # Perform FOTA update on secondary device
        success = await client.update_firmware(firmware_file, TARGET_SECONDARY)
        
        if success:
            # Reset secondary device after successful update
            await client.reset_device(TARGET_SECONDARY)
            logger.info("Secondary device will reset to apply the update")
        else:
            logger.error("FOTA update failed")
            
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        await client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
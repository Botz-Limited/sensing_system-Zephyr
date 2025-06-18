#!/usr/bin/env python3
"""
Test script for file proxy functionality
Requires: pip install bleak asyncio
"""

import asyncio
import struct
import sys
import os
from bleak import BleakClient, BleakScanner
import logging
from datetime import datetime

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# File Proxy Service UUIDs
FILE_PROXY_SERVICE = "7e500001-b5a3-f393-e0a9-e50e24dcca9e"
TARGET_CHAR = "7e500002-b5a3-f393-e0a9-e50e24dcca9e"
COMMAND_CHAR = "7e500003-b5a3-f393-e0a9-e50e24dcca9e"
DATA_CHAR = "7e500004-b5a3-f393-e0a9-e50e24dcca9e"
STATUS_CHAR = "7e500005-b5a3-f393-e0a9-e50e24dcca9e"

# Target device types
TARGET_PRIMARY = 0x00
TARGET_SECONDARY = 0x01

# Commands
CMD_LIST_LOGS = 0x01
CMD_READ_FILE = 0x02
CMD_DELETE_FILE = 0x03
CMD_GET_FILE_INFO = 0x04
CMD_ABORT = 0x05

# Status codes
STATUS_IDLE = 0x00
STATUS_BUSY = 0x01
STATUS_SUCCESS = 0x02
STATUS_ERROR = 0x03
STATUS_NO_TARGET = 0x04
STATUS_FILE_NOT_FOUND = 0x05
STATUS_TRANSFER_IN_PROGRESS = 0x06

# File types
FILE_TYPE_FOOT_SENSOR = 0x01
FILE_TYPE_BHI360 = 0x02
FILE_TYPE_ALL = 0xFF

status_names = {
    STATUS_IDLE: "Idle",
    STATUS_BUSY: "Busy",
    STATUS_SUCCESS: "Success",
    STATUS_ERROR: "Error",
    STATUS_NO_TARGET: "No Target Device",
    STATUS_FILE_NOT_FOUND: "File Not Found",
    STATUS_TRANSFER_IN_PROGRESS: "Transfer In Progress"
}

class FileProxyClient:
    def __init__(self, address):
        self.address = address
        self.client = None
        self.status = STATUS_IDLE
        self.received_data = bytearray()
        self.file_list = []
        
    async def connect(self):
        """Connect to the primary device"""
        logger.info(f"Connecting to {self.address}...")
        self.client = BleakClient(self.address)
        await self.client.connect()
        logger.info("Connected!")
        
        # Subscribe to notifications
        await self.client.start_notify(STATUS_CHAR, self._status_handler)
        await self.client.start_notify(DATA_CHAR, self._data_handler)
        
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
            
    def _data_handler(self, sender, data):
        """Handle data notifications"""
        self.received_data.extend(data)
        logger.debug(f"Received {len(data)} bytes of data")
        
    async def set_target(self, target):
        """Set the target device"""
        target_name = "Primary" if target == TARGET_PRIMARY else "Secondary"
        logger.info(f"Setting target to: {target_name}")
        await self.client.write_gatt_char(TARGET_CHAR, bytes([target]))
        
    async def list_files(self, target=TARGET_SECONDARY):
        """List files on target device"""
        await self.set_target(target)
        await asyncio.sleep(0.5)
        
        self.received_data.clear()
        self.file_list.clear()
        
        logger.info("Listing files...")
        await self.client.write_gatt_char(COMMAND_CHAR, bytes([CMD_LIST_LOGS]))
        
        # Wait for response
        await asyncio.sleep(2)
        
        # Parse file list
        if len(self.received_data) >= 2:
            cmd = self.received_data[0]
            count = self.received_data[1]
            logger.info(f"Found {count} files")
            
            # Parse each file entry (46 bytes each)
            offset = 2
            entry_size = 46  # 1+1+4+4+32 bytes
            
            for i in range(count):
                if offset + entry_size <= len(self.received_data):
                    entry_data = self.received_data[offset:offset+entry_size]
                    file_id = entry_data[0]
                    file_type = entry_data[1]
                    size = struct.unpack('<I', entry_data[2:6])[0]
                    timestamp = struct.unpack('<I', entry_data[6:10])[0]
                    name = entry_data[10:42].decode('utf-8').rstrip('\x00')
                    
                    file_info = {
                        'id': file_id,
                        'type': 'Foot Sensor' if file_type == FILE_TYPE_FOOT_SENSOR else 'BHI360',
                        'size': size,
                        'timestamp': datetime.fromtimestamp(timestamp),
                        'name': name
                    }
                    self.file_list.append(file_info)
                    
                    logger.info(f"  [{file_id}] {name} - {file_info['type']} - {size} bytes - {file_info['timestamp']}")
                    
                    offset += entry_size
                    
        return self.file_list
        
    async def get_file_info(self, file_id, file_type, target=TARGET_SECONDARY):
        """Get detailed information about a file"""
        await self.set_target(target)
        await asyncio.sleep(0.5)
        
        self.received_data.clear()
        
        logger.info(f"Getting info for file ID {file_id}, type {file_type}")
        cmd_data = bytes([CMD_GET_FILE_INFO, file_id, file_type])
        await self.client.write_gatt_char(COMMAND_CHAR, cmd_data)
        
        # Wait for response
        await asyncio.sleep(2)
        
        # Parse file info
        if len(self.received_data) >= 17:  # 1+1+1+4+4+4 bytes
            cmd = self.received_data[0]
            if cmd == CMD_GET_FILE_INFO:
                info_data = self.received_data[1:17]
                status = info_data[0]
                file_id = info_data[1]
                file_type = info_data[2]
                size = struct.unpack('<I', info_data[3:7])[0]
                timestamp = struct.unpack('<I', info_data[7:11])[0]
                crc32 = struct.unpack('<I', info_data[11:15])[0]
                
                logger.info(f"File Info:")
                logger.info(f"  ID: {file_id}")
                logger.info(f"  Type: {'Foot Sensor' if file_type == FILE_TYPE_FOOT_SENSOR else 'BHI360'}")
                logger.info(f"  Size: {size} bytes")
                logger.info(f"  Timestamp: {datetime.fromtimestamp(timestamp)}")
                logger.info(f"  CRC32: 0x{crc32:08X}")
                
                return {
                    'id': file_id,
                    'type': file_type,
                    'size': size,
                    'timestamp': timestamp,
                    'crc32': crc32
                }
                
        return None
        
    async def delete_file(self, file_id, file_type, target=TARGET_SECONDARY):
        """Delete a file"""
        await self.set_target(target)
        await asyncio.sleep(0.5)
        
        logger.info(f"Deleting file ID {file_id}, type {file_type}")
        cmd_data = bytes([CMD_DELETE_FILE, file_id, file_type])
        await self.client.write_gatt_char(COMMAND_CHAR, cmd_data)
        
        # Wait for completion
        await asyncio.sleep(2)
        
        if self.status == STATUS_SUCCESS:
            logger.info("File deleted successfully")
            return True
        else:
            logger.error("Failed to delete file")
            return False
            
    async def read_file(self, file_id, file_type, target=TARGET_SECONDARY):
        """Read a file (note: for primary device, use SMP instead)"""
        await self.set_target(target)
        await asyncio.sleep(0.5)
        
        self.received_data.clear()
        
        logger.info(f"Reading file ID {file_id}, type {file_type}")
        cmd_data = bytes([CMD_READ_FILE, file_id, file_type])
        await self.client.write_gatt_char(COMMAND_CHAR, cmd_data)
        
        # Wait for transfer to complete
        # In a real implementation, this would monitor STATUS_TRANSFER_IN_PROGRESS
        await asyncio.sleep(5)
        
        if self.status == STATUS_SUCCESS:
            logger.info(f"File read complete, received {len(self.received_data)} bytes")
            return self.received_data
        else:
            logger.error("Failed to read file")
            return None

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
        print("Usage: python test_file_proxy.py <command> [device_address]")
        print("Commands:")
        print("  list     - List files on secondary device")
        print("  info     - Get file information")
        print("  delete   - Delete a file")
        print("  read     - Read a file")
        sys.exit(1)
        
    command = sys.argv[1]
    
    # Get device address
    if len(sys.argv) >= 3:
        device_address = sys.argv[2]
    else:
        device_address = await scan_for_device()
        if not device_address:
            sys.exit(1)
    
    # Create file proxy client
    client = FileProxyClient(device_address)
    
    try:
        # Connect to device
        await client.connect()
        
        if command == "list":
            # List files on secondary device
            files = await client.list_files(TARGET_SECONDARY)
            if not files:
                logger.info("No files found")
                
        elif command == "info":
            # First list files
            files = await client.list_files(TARGET_SECONDARY)
            if files:
                # Get info for first file
                file = files[0]
                await client.get_file_info(file['id'], 
                                          FILE_TYPE_FOOT_SENSOR if file['type'] == 'Foot Sensor' else FILE_TYPE_BHI360,
                                          TARGET_SECONDARY)
                                          
        elif command == "delete":
            # First list files
            files = await client.list_files(TARGET_SECONDARY)
            if files:
                # Delete first file
                file = files[0]
                await client.delete_file(file['id'],
                                       FILE_TYPE_FOOT_SENSOR if file['type'] == 'Foot Sensor' else FILE_TYPE_BHI360,
                                       TARGET_SECONDARY)
                                       
        elif command == "read":
            logger.info("Note: For secondary device file reading, the D2D protocol needs to be fully implemented")
            logger.info("For primary device files, use SMP protocol directly")
            
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        await client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
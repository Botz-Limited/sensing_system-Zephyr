#!/usr/bin/env python3
"""
Test script for switchable SMP functionality
Demonstrates how to switch between primary and secondary device updates
"""

import asyncio
import sys
import logging
from bleak import BleakClient, BleakScanner

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# UUIDs
SMP_SERVICE_UUID = "8D53DC1D-1DB7-4CD3-868B-8A527460AA84"
SMP_CHAR_UUID = "DA2E7828-FBCE-4E01-AE9E-261174997C48"
TARGET_SELECT_UUID = "DA2E7829-FBCE-4E01-AE9E-261174997C48"

# Device name
PRIMARY_DEVICE_NAME = "BotzRightSh"

class SwitchableSMPTest:
    def __init__(self, address=None):
        self.address = address
        self.client = None
        
    async def find_device(self):
        """Find the primary device by name"""
        if self.address:
            return self.address
            
        logger.info(f"Scanning for {PRIMARY_DEVICE_NAME}...")
        devices = await BleakScanner.discover()
        
        for device in devices:
            if device.name == PRIMARY_DEVICE_NAME:
                logger.info(f"Found {PRIMARY_DEVICE_NAME} at {device.address}")
                return device.address
                
        raise Exception(f"Device {PRIMARY_DEVICE_NAME} not found")
        
    async def connect(self):
        """Connect to the device"""
        if not self.address:
            self.address = await self.find_device()
            
        logger.info(f"Connecting to {self.address}...")
        self.client = BleakClient(self.address)
        await self.client.connect()
        logger.info("Connected!")
        
    async def get_current_target(self):
        """Read the current target mode"""
        try:
            target = await self.client.read_gatt_char(TARGET_SELECT_UUID)
            mode = target.decode('utf-8').strip()
            logger.info(f"Current target mode: {mode}")
            return mode
        except Exception as e:
            logger.error(f"Failed to read target mode: {e}")
            return None
            
    async def set_target(self, target):
        """Set the target mode (PRIMARY or SECONDARY)"""
        try:
            logger.info(f"Setting target to: {target}")
            await self.client.write_gatt_char(TARGET_SELECT_UUID, target.encode('utf-8'))
            logger.info(f"Target set to {target}")
            
            # Verify the change
            await asyncio.sleep(0.5)
            current = await self.get_current_target()
            if current == target:
                logger.info("✓ Target successfully changed")
            else:
                logger.warning(f"Target may not have changed. Current: {current}")
                
        except Exception as e:
            logger.error(f"Failed to set target: {e}")
            
    async def test_mode_switching(self):
        """Test switching between PRIMARY and SECONDARY modes"""
        logger.info("\n=== Testing Mode Switching ===")
        
        # Get initial mode
        initial_mode = await self.get_current_target()
        
        # Test switching to PRIMARY
        logger.info("\nSwitching to PRIMARY mode...")
        await self.set_target("PRIMARY")
        
        # Test switching to SECONDARY
        logger.info("\nSwitching to SECONDARY mode...")
        await self.set_target("SECONDARY")
        
        # Restore initial mode
        if initial_mode and initial_mode != "SECONDARY":
            logger.info(f"\nRestoring initial mode: {initial_mode}")
            await self.set_target(initial_mode)
            
    async def disconnect(self):
        """Disconnect from device"""
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            logger.info("Disconnected")

async def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--help":
        print("Usage: python test_switchable_smp.py [device_address]")
        print("\nThis script tests the switchable SMP service that allows")
        print("updating both primary and secondary devices via BLE.")
        print("\nSteps to update both devices:")
        print("1. Connect to device")
        print("2. Default mode is SECONDARY - update secondary device")
        print("3. Switch to PRIMARY mode - update primary device")
        print("4. Switch back to SECONDARY when done")
        sys.exit(0)
        
    device_address = sys.argv[1] if len(sys.argv) > 1 else None
    
    # Create test client
    client = SwitchableSMPTest(device_address)
    
    try:
        # Connect
        await client.connect()
        
        # Test mode switching
        await client.test_mode_switching()
        
        print("\n=== How to Update Both Devices ===")
        print("\n1. For SECONDARY device (default):")
        print("   - Just upload firmware normally in Nordic Device Manager")
        print("   - Or: mcumgr --conntype ble --connstring peer_name='BotzRightSh' image upload secondary.bin")
        
        print("\n2. For PRIMARY device:")
        print("   - First: Write 'PRIMARY' to Target Select characteristic")
        print("   - Then: Upload firmware in Nordic Device Manager")
        print("   - Or: mcumgr --conntype ble --connstring peer_name='BotzRightSh' image upload primary.bin")
        print("   - Finally: Write 'SECONDARY' to switch back")
        
        print("\n✓ Both devices can now be updated via BLE!")
        
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        await client.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
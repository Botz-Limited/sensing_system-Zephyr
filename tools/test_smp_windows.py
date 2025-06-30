#!/usr/bin/env python3
"""
Windows-friendly SMP Proxy Test Script
Simplified version for testing dual-core updates on Windows
"""

import asyncio
import sys
import os
from bleak import BleakClient, BleakScanner
import logging
from datetime import datetime

# Configure logging with timestamp
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# SMP Proxy Service UUIDs
SMP_PROXY_SERVICE = "8D53DC1E-1DB7-4CD3-868B-8A527460AA84"
TARGET_CHAR = "DA2E7829-FBCE-4E01-AE9E-261174997C48"
SMP_DATA_CHAR = "DA2E7828-FBCE-4E01-AE9E-261174997C48"

# Target device types
TARGET_PRIMARY = 0x00
TARGET_SECONDARY = 0x01

class SimpleSMPProxy:
    """Simplified SMP Proxy client for Windows"""
    
    def __init__(self, address):
        self.address = address
        self.client = None
        
    async def connect(self):
        """Connect to device"""
        logger.info(f"Connecting to {self.address}...")
        self.client = BleakClient(self.address)
        await self.client.connect()
        logger.info("Connected successfully!")
        
        # List services
        services = await self.client.get_services()
        logger.info("Available services:")
        for service in services:
            logger.info(f"  - {service.uuid}: {service.description}")
            
    async def disconnect(self):
        """Disconnect from device"""
        if self.client:
            await self.client.disconnect()
            logger.info("Disconnected")
            
    async def select_target(self, target):
        """Select target device (0=primary, 1=secondary)"""
        target_name = "PRIMARY" if target == TARGET_PRIMARY else "SECONDARY"
        logger.info(f"Selecting target: {target_name}")
        
        try:
            await self.client.write_gatt_char(TARGET_CHAR, bytes([target]))
            logger.info(f"Target set to {target_name} device")
            return True
        except Exception as e:
            logger.error(f"Failed to set target: {e}")
            return False
            
    async def read_target(self):
        """Read current target selection"""
        try:
            value = await self.client.read_gatt_char(TARGET_CHAR)
            target = value[0] if value else 0
            target_name = "PRIMARY" if target == TARGET_PRIMARY else "SECONDARY"
            logger.info(f"Current target: {target_name}")
            return target
        except Exception as e:
            logger.error(f"Failed to read target: {e}")
            return None
            
    async def test_smp_communication(self):
        """Test basic SMP communication"""
        logger.info("Testing SMP communication...")
        
        # Enable notifications on SMP data characteristic
        try:
            await self.client.start_notify(SMP_DATA_CHAR, self._notification_handler)
            logger.info("Notifications enabled on SMP data characteristic")
            
            # Here you would send actual SMP commands
            # For now, just show that we can interact with the characteristic
            logger.info("SMP proxy is ready for commands")
            
            # Wait a bit to see if we get any notifications
            await asyncio.sleep(2)
            
            await self.client.stop_notify(SMP_DATA_CHAR)
            
        except Exception as e:
            logger.error(f"SMP communication test failed: {e}")
            
    def _notification_handler(self, sender, data):
        """Handle notifications from SMP characteristic"""
        logger.info(f"Notification from {sender}: {data.hex()}")

async def scan_for_device(name_prefix="SensingGR", timeout=10):
    """Scan for primary device"""
    logger.info(f"Scanning for devices with name starting with '{name_prefix}'...")
    logger.info(f"Make sure your device is advertising and not connected to anything else")
    
    start_time = datetime.now()
    devices_found = []
    
    # Scan with timeout
    devices = await BleakScanner.discover(timeout=timeout)
    
    logger.info(f"Found {len(devices)} devices")
    
    for device in devices:
        if device.name:
            logger.info(f"  - {device.name} ({device.address}) RSSI: {device.rssi}")
            if device.name.startswith(name_prefix):
                devices_found.append(device)
                
    if devices_found:
        # Sort by RSSI (strongest signal first)
        devices_found.sort(key=lambda d: d.rssi, reverse=True)
        selected = devices_found[0]
        logger.info(f"\nSelected device: {selected.name} ({selected.address}) with RSSI: {selected.rssi}")
        return selected.address
    else:
        logger.error(f"No device found with name starting with '{name_prefix}'")
        logger.info("Tips:")
        logger.info("  - Make sure the device is powered on")
        logger.info("  - Ensure it's not connected to another device/app")
        logger.info("  - Check if it's in advertising mode")
        logger.info("  - Try moving closer to the device")
        return None

async def interactive_menu():
    """Interactive menu for testing"""
    device_address = await scan_for_device()
    if not device_address:
        return
        
    proxy = SimpleSMPProxy(device_address)
    
    try:
        await proxy.connect()
        
        while True:
            print("\n=== SMP Proxy Test Menu ===")
            print("1. Read current target")
            print("2. Select PRIMARY device")
            print("3. Select SECONDARY device")
            print("4. Test SMP communication")
            print("5. Rescan for devices")
            print("0. Exit")
            
            choice = input("\nEnter your choice: ").strip()
            
            if choice == '0':
                break
            elif choice == '1':
                await proxy.read_target()
            elif choice == '2':
                await proxy.select_target(TARGET_PRIMARY)
            elif choice == '3':
                await proxy.select_target(TARGET_SECONDARY)
            elif choice == '4':
                await proxy.test_smp_communication()
            elif choice == '5':
                await proxy.disconnect()
                device_address = await scan_for_device()
                if device_address:
                    proxy = SimpleSMPProxy(device_address)
                    await proxy.connect()
                else:
                    break
            else:
                print("Invalid choice")
                
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        await proxy.disconnect()

async def quick_test(target=None):
    """Quick test mode"""
    device_address = await scan_for_device()
    if not device_address:
        return
        
    proxy = SimpleSMPProxy(device_address)
    
    try:
        await proxy.connect()
        
        # Read current target
        await proxy.read_target()
        
        # If target specified, set it
        if target is not None:
            await proxy.select_target(target)
            await asyncio.sleep(0.5)
            await proxy.read_target()
            
        # Test SMP communication
        await proxy.test_smp_communication()
        
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        await proxy.disconnect()

def main():
    """Main entry point"""
    print("=== Windows SMP Proxy Test Tool ===")
    print("Make sure Bluetooth is enabled on your computer")
    print()
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "primary":
            asyncio.run(quick_test(TARGET_PRIMARY))
        elif sys.argv[1] == "secondary":
            asyncio.run(quick_test(TARGET_SECONDARY))
        elif sys.argv[1] == "scan":
            asyncio.run(scan_for_device())
        else:
            print("Usage:")
            print("  python test_smp_windows.py          # Interactive mode")
            print("  python test_smp_windows.py scan     # Just scan for devices")
            print("  python test_smp_windows.py primary  # Quick test with primary")
            print("  python test_smp_windows.py secondary # Quick test with secondary")
    else:
        # Interactive mode
        asyncio.run(interactive_menu())

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
Quick verification script for SMP Proxy service
Checks if the service is advertised and accessible
"""

import asyncio
import sys
from bleak import BleakClient, BleakScanner
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# SMP Proxy Service UUID
SMP_PROXY_SERVICE = "8D53DC1E-1DB7-4CD3-868B-8A527460AA84"
TARGET_CHAR = "DA2E7829-FBCE-4E01-AE9E-261174997C48"
SMP_DATA_CHAR = "DA2E7828-FBCE-4E01-AE9E-261174997C48"

# Standard SMP Service UUID (for comparison)
STANDARD_SMP_SERVICE = "8D53DC1D-1DB7-4CD3-868B-8A527460AA84"

async def verify_smp_proxy(address):
    """Verify SMP proxy service is available and working"""
    
    async with BleakClient(address) as client:
        logger.info(f"Connected to {address}")
        
        # Get all services
        services = await client.get_services()
        
        has_smp_proxy = False
        has_standard_smp = False
        
        logger.info("\nAvailable services:")
        for service in services:
            logger.info(f"  {service.uuid}: {service.description}")
            
            if service.uuid.lower() == SMP_PROXY_SERVICE.lower():
                has_smp_proxy = True
                logger.info("  ✓ Found SMP Proxy service!")
                
                # List characteristics
                for char in service.characteristics:
                    logger.info(f"    - {char.uuid}: {char.description}")
                    logger.info(f"      Properties: {char.properties}")
                    
            elif service.uuid.lower() == STANDARD_SMP_SERVICE.lower():
                has_standard_smp = True
                logger.info("  ✓ Found standard SMP service")
        
        if has_smp_proxy:
            logger.info("\n✅ SMP Proxy service is available!")
            logger.info("Mobile apps can now use standard MCUmgr for both devices")
            
            # Try to read target characteristic
            try:
                target_value = await client.read_gatt_char(TARGET_CHAR)
                target = target_value[0] if target_value else 0
                logger.info(f"\nCurrent target: {'PRIMARY' if target == 0 else 'SECONDARY'} (0x{target:02x})")
            except Exception as e:
                logger.warning(f"Could not read target characteristic: {e}")
                
            # Check if we can write to target
            try:
                # Try to set target to primary (safe operation)
                await client.write_gatt_char(TARGET_CHAR, bytes([0x00]))
                logger.info("✓ Successfully wrote to target characteristic")
            except Exception as e:
                logger.warning(f"Could not write to target characteristic: {e}")
                
        else:
            logger.error("\n❌ SMP Proxy service NOT found!")
            logger.error("Make sure you flashed the primary device with the new firmware")
            
        if has_standard_smp:
            logger.info("\nNote: Standard SMP service is also available for direct primary access")
            
        return has_smp_proxy

async def scan_and_verify():
    """Scan for device and verify SMP proxy"""
    logger.info("Scanning for SensingGR device...")
    
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name and device.name.startswith("SensingGR"):
            logger.info(f"Found device: {device.name} ({device.address})")
            return await verify_smp_proxy(device.address)
            
    logger.error("No SensingGR device found!")
    return False

async def main():
    """Main function"""
    if len(sys.argv) > 1:
        # Use provided address
        address = sys.argv[1]
        await verify_smp_proxy(address)
    else:
        # Scan for device
        await scan_and_verify()

if __name__ == "__main__":
    asyncio.run(main())
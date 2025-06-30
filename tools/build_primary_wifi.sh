#!/bin/bash
# Standard build script for PRIMARY device (Right foot - connects to phone) with WiFi

cd /home/ee/sensing_fw

echo "Building PRIMARY device firmware (Right foot) with WiFi..."
echo "This device will:"
echo "  - Accept connections from phones"
echo "  - Accept connections from secondary device"
echo "  - Advertise as 'SensingGR'"
echo "  - Have WiFi enabled"

# Clean previous build
rm -rf build_primary_wifi

# Build with primary device configuration and network core without central role
west build --build-dir /home/ee/sensing_fw/build_primary_wifi /home/ee/sensing_fw/ \
    --board nrf5340dk/nrf5340/cpuapp \
    --sysbuild \
    -- -DCONFIG_PRIMARY_DEVICE=y \
       -Dipc_radio_EXTRA_CONF_FILE=/home/ee/sensing_fw/sysbuild/ipc_radio/prj_primary.conf \
       -DEXTRA_CONF_FILE=/home/ee/sensing_fw/prj_wifi.conf

if [ $? -eq 0 ]; then
    echo ""
    echo "Build successful!"
    echo ""
    echo "Firmware files:"
    echo "  - build_primary_wifi/merged.hex (application core)"
    echo "  - build_primary_wifi/merged_CPUNET.hex (network core)"
else
    echo "Build failed!"
    exit 1
fi
#!/bin/bash
# Build script for SECONDARY device with BT_CENTRAL enabled on network core and WiFi

cd /home/ee/sensing_fw

echo "Building SECONDARY device firmware with BT_CENTRAL support and WiFi..."
echo "This will enable the central role on the network core for connecting to primary device"

# Clean previous build
rm -rf build_secondary_wifi

# Build with secondary device configuration and network core central support
west build --build-dir /home/ee/sensing_fw/build_secondary_wifi /home/ee/sensing_fw/ \
    --board nrf5340dk/nrf5340/cpuapp \
    --sysbuild \
    -- -DCONFIG_PRIMARY_DEVICE=n \
       -Dipc_radio_EXTRA_CONF_FILE=/home/ee/sensing_fw/sysbuild/ipc_radio/prj_secondary.conf \
       -DEXTRA_CONF_FILE=/home/ee/sensing_fw/prj_wifi.conf

if [ $? -eq 0 ]; then
    echo ""
    echo "Build successful!"
    echo ""
    echo "Firmware files:"
    echo "  - build_secondary_wifi/merged.hex (application core)"
    echo "  - build_secondary_wifi/merged_CPUNET.hex (network core)"
    echo ""
    echo "To flash the secondary device:"
    echo "  nrfjprog --program build_secondary_wifi/merged_CPUNET.hex --verify --chiperase --reset"
    echo "  sleep 2"
    echo "  nrfjprog --program build_secondary_wifi/merged.hex --verify --chiperase --reset"
else
    echo "Build failed!"
    exit 1
fi
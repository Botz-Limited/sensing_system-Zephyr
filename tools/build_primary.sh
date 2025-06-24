#!/bin/bash
# Standard build script for PRIMARY device (Right foot - connects to phone)

cd /home/ee/sensing_fw

echo "Building PRIMARY device firmware (Right foot)..."
echo "This device will:"
echo "  - Accept connections from phones"
echo "  - Accept connections from secondary device"
echo "  - Advertise as 'SensingGR'"

# Clean previous build
rm -rf build_primary

# Build with primary device configuration and network core without central role
west build --build-dir /home/ee/sensing_fw/build_primary /home/ee/sensing_fw/ \
    --board nrf5340dk/nrf5340/cpuapp \
    --sysbuild \
    -- -DCONFIG_PRIMARY_DEVICE=y \
       -Dipc_radio_EXTRA_CONF_FILE=/home/ee/sensing_fw/sysbuild/ipc_radio/prj_primary.conf

if [ $? -eq 0 ]; then
    echo ""
    echo "Build successful!"
    echo ""
    echo "Network core configuration:"
    echo "  - BT_PERIPHERAL=y (to accept connections from phones and secondary)"
    echo "  - BT_CENTRAL=n (disabled - not needed for primary device)"
    echo "  - BT_OBSERVER=y (for scanning if needed)"
    echo ""
    echo "Application core configuration:"
    echo "  - PRIMARY_DEVICE=y"
    echo "  - Device name: SensingGR"
    echo "  - Can connect to phones: YES"
    echo "  - Can accept secondary device: YES"
    echo ""
    echo "Firmware files:"
    echo "  - build_primary/merged.hex (application core)"
    echo "  - build_primary/merged_CPUNET.hex (network core)"
else
    echo "Build failed!"
    exit 1
fi
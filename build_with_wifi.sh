#!/bin/bash

# Build script for sensing_fw with WiFi support

echo "Building sensing_fw with WiFi support..."

# Clean previous builds (optional)
# rm -rf build_primary build_secondary

# Build with WiFi overlay
west build -b nrf5340dk/nrf5340/cpuapp \
    --sysbuild \
    -p auto \
    -- \
    -DEXTRA_CONF_FILE="prj_wifi.conf" \
    -DEXTRA_DTC_OVERLAY_FILE="boards/nrf5340dk_nrf5340_cpuapp_wifi.overlay"

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Flash with: west flash"
else
    echo "Build failed!"
    exit 1
fi
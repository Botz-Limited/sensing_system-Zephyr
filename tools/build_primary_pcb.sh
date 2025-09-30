#!/bin/bash
# Standard build script for PRIMARY device (Right foot - connects to phone)
# with PCB overlay

cd /home/ee/sensing_fw

echo "Building PRIMARY device firmware (Right foot) for PCB..."
echo "This device will:"
echo "  - Accept connections from phones"
echo "  - Accept connections from secondary device"
echo "  - Advertise as 'BotzRightSh'"

# Clean previous build
#rm -rf build_primary_pcb

# Base west command
WEST_CMD="west build --build-dir /home/ee/sensing_fw/build_primary_pcb /home/ee/sensing_fw/ --board nrf5340dk/nrf5340/cpuapp --sysbuild -- -DCONFIG_PRIMARY_DEVICE=y -Dipc_radio_EXTRA_CONF_FILE=/home/ee/sensing_fw/sysbuild/ipc_radio/prj_primary.conf -DDTC_OVERLAY_FILE=/home/ee/sensing_fw/boards/nrf5340dk_nrf5340_cpuapp_pcb.overlay"
# For PCB build, we need to specify MCUboot overlay with Winbond flash
WEST_CMD+=" -Dmcuboot_DTC_OVERLAY_FILE=/home/ee/sensing_fw/sysbuild/mcuboot/boards/nrf5340dk_nrf5340_cpuapp_pcb.overlay"

# Build EXTRA_CONF_FILE list for PCB and optional Wi-Fi
EXTRA_CONF="/home/ee/sensing_fw/prj_pcb.conf"



# Add to WEST_CMD if EXTRA_CONF is set
if [ -n "$EXTRA_CONF" ]; then
    WEST_CMD+=" -DEXTRA_CONF_FILE=$EXTRA_CONF"
fi

# Execute the build command
$WEST_CMD

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
    echo "  - Device name: BotzRightSh"
    echo "  - Can connect to phones: YES"
    echo "  - Can accept secondary device: YES"
    echo ""
    echo "Firmware files:"
    echo "  - build_primary_pcb/merged.hex (application core)"
    echo "  - build_primary_pcb/merged_CPUNET.hex (network core)"
else
    echo "Build failed!"
    exit 1
fi
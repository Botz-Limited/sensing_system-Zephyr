#!/bin/bash
# Recovery and flash script for nRF5340 PCB with protected cores

echo "=== nRF5340 PCB Recovery and Flash Script ==="
echo ""

# Function to check if a command succeeded
check_result() {
    if [ $? -ne 0 ]; then
        echo "ERROR: $1 failed!"
        exit 1
    fi
}

# Step 1: Recover both cores
echo "Step 1: Recovering both cores..."
echo "  - Recovering network core..."
nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK
check_result "Network core recovery"
sleep 1

echo "  - Recovering application core..."
nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION
check_result "Application core recovery"
sleep 1

# Step 2: Erase both cores
echo ""
echo "Step 2: Erasing both cores..."
echo "  - Erasing network core..."
nrfjprog --eraseall -f NRF53 --coprocessor CP_NETWORK
check_result "Network core erase"
sleep 1

echo "  - Erasing application core..."
nrfjprog --eraseall -f NRF53 --coprocessor CP_APPLICATION
check_result "Application core erase"
sleep 1

# Step 3: Check if firmware files exist
if [ "$1" == "primary" ]; then
    BUILD_DIR="build_primary_pcb"
    DEVICE_TYPE="PRIMARY"
elif [ "$1" == "secondary" ]; then
    BUILD_DIR="build_secondary_pcb"
    DEVICE_TYPE="SECONDARY"
else
    echo ""
    echo "Usage: $0 [primary|secondary]"
    echo "  primary   - Flash primary device firmware"
    echo "  secondary - Flash secondary device firmware"
    exit 1
fi

NETWORK_HEX="/home/ee/sensing_fw/${BUILD_DIR}/merged_CPUNET.hex"
APP_HEX="/home/ee/sensing_fw/${BUILD_DIR}/merged.hex"

if [ ! -f "$NETWORK_HEX" ] || [ ! -f "$APP_HEX" ]; then
    echo ""
    echo "ERROR: Firmware files not found!"
    echo "  Looking for:"
    echo "    - $NETWORK_HEX"
    echo "    - $APP_HEX"
    echo ""
    echo "Please build the firmware first using:"
    echo "  ./tools/build_${1}_pcb.sh"
    exit 1
fi

# Step 4: Flash network core
echo ""
echo "Step 3: Flashing $DEVICE_TYPE device firmware..."
echo "  - Flashing network core..."
nrfjprog --program "$NETWORK_HEX" -f NRF53 --coprocessor CP_NETWORK --verify
check_result "Network core flash"
sleep 1

# Step 5: Flash application core
echo "  - Flashing application core..."
nrfjprog --program "$APP_HEX" -f NRF53 --coprocessor CP_APPLICATION --verify
check_result "Application core flash"
sleep 1

# Step 6: Reset the device
echo ""
echo "Step 4: Resetting device..."
nrfjprog --reset -f NRF53
check_result "Device reset"

echo ""
echo "=== Flash complete! ==="
echo "Device type: $DEVICE_TYPE"
echo "The device should now be running the new firmware."
echo ""

# Additional debug info
echo "To verify the device is working:"
echo "  - Check RTT logs: JLinkRTTViewer"
echo "  - For PRIMARY device: Look for 'BotzRightSh' in Bluetooth scanner"
echo "  - For SECONDARY device: Look for 'BotzLeftSh' in Bluetooth scanner"
#!/bin/bash
# Robust flash script for PRIMARY device with better error handling

cd /home/ee/sensing_fw

echo "=== Robust PRIMARY device firmware flashing ==="
echo "Make sure the RIGHT foot device is connected!"
read -p "Press Enter to continue..."

# Function to check if command succeeded
check_result() {
    if [ $? -ne 0 ]; then
        echo "ERROR: $1 failed!"
        exit 1
    fi
}

# Step 1: Full recovery of both cores
echo ""
echo "Step 1: Full device recovery..."
echo "Recovering network core..."
nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK --log
check_result "Network core recovery"

echo "Recovering application core..."
nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION --log
check_result "Application core recovery"

# Step 2: Erase both cores explicitly
echo ""
echo "Step 2: Erasing cores..."
sleep 2

echo "Erasing network core..."
nrfjprog --eraseall -f NRF53 --coprocessor CP_NETWORK
check_result "Network core erase"

echo "Erasing application core..."
nrfjprog --eraseall -f NRF53 --coprocessor CP_APPLICATION
check_result "Application core erase"

# Step 3: Program network core
echo ""
echo "Step 3: Programming network core..."
sleep 1

if [ -f "build_primary/merged_CPUNET.hex" ]; then
    nrfjprog --program build_primary/merged_CPUNET.hex --verify -f NRF53 --coprocessor CP_NETWORK
    check_result "Network core programming"
else
    echo "WARNING: Network core hex file not found, skipping..."
fi

# Step 4: Program application core
echo ""
echo "Step 4: Programming application core..."
sleep 1

nrfjprog --program build_primary/merged.hex --verify -f NRF53 --coprocessor CP_APPLICATION
check_result "Application core programming"

# Step 5: Reset the device
echo ""
echo "Step 5: Resetting device..."
nrfjprog --reset -f NRF53
check_result "Device reset"

echo ""
echo "=== Primary device flashed successfully! ==="
echo "The device should now be running the new firmware."
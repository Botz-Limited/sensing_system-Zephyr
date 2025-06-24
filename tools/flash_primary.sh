#!/bin/bash
# Flash script for PRIMARY device (Right foot)

cd /home/ee/sensing_fw

echo "Flashing PRIMARY device firmware..."
echo "Make sure the RIGHT foot device is connected!"
read -p "Press Enter to continue..."

# Check if recovery is needed by attempting to flash network core
echo "Flashing network core..."
if ! nrfjprog --program build_primary/merged_CPUNET.hex --verify --chiperase --reset 2>&1 | grep -q "Access protection is enabled"; then
    echo "Network core flashed successfully"
else
    echo "Access protection detected on network core. Recovering..."
    nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK
    echo "Recovery complete. Retrying network core flash..."
    nrfjprog --program build_primary/merged_CPUNET.hex --verify --chiperase --reset
fi

# Small delay
sleep 2

# Flash application core
echo "Flashing application core..."
if ! nrfjprog --program build_primary/merged.hex --verify --chiperase --reset 2>&1 | grep -q "Access protection is enabled"; then
    echo "Application core flashed successfully"
else
    echo "Access protection detected on application core. Recovering..."
    nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION
    echo "Recovery complete. Retrying application core flash..."
    nrfjprog --program build_primary/merged.hex --verify --chiperase --reset
fi

echo "Primary device flashed successfully!"
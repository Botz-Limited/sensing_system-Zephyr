#!/bin/bash
# Flash script for SECONDARY device (Right foot) with recovery option

cd /home/ee/sensing_fw

echo "Flashing Secondary device firmware with recovery..."
echo "Make sure the RIGHT foot device is connected!"
read -p "Press Enter to continue..."

# First recover both cores to ensure no access protection issues
echo "Recovering network core..."
nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK

echo "Recovering application core..."
nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION

# Small delay after recovery
sleep 2

# Flash network core first
echo "Flashing network core..."
# Use --sectorerase instead of --chiperase for network core to avoid access protection issues
nrfjprog --program build_secondary/merged_CPUNET.hex --verify --sectorerase -f NRF53 --coprocessor CP_NETWORK

# Small delay
sleep 2

# Flash application core
echo "Flashing application core..."
# For application core, we can use chiperase
nrfjprog --program build_secondary/merged.hex --verify --chiperase -f NRF53 --coprocessor CP_APPLICATION

# Final reset to start the application
echo "Resetting device..."
nrfjprog --reset -f NRF53

echo "Secondary device flashed successfully!"
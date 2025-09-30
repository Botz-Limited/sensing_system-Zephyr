#!/bin/bash
# Flash script for SECONDARY device (Left foot)

cd /home/ee/sensing_fw

echo "Flashing SECONDARY device firmware..."
echo "Make sure the LEFT foot device is connected!"
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
nrfjprog --program build_secondary_pcb/merged_CPUNET.hex --verify --sectorerase -f NRF53 --coprocessor CP_NETWORK

# Small delay
sleep 2

# Flash application core
echo "Flashing application core..."
nrfjprog --program build_secondary_pcb/merged.hex --verify --chiperase -f NRF53 --coprocessor CP_APPLICATION

# Final reset to start the application
echo "Resetting device..."
nrfjprog --reset -f NRF53

echo "Secondary device flashed successfully!"
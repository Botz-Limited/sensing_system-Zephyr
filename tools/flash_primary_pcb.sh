#!/bin/bash
# Flash script for PRIMARY device (Right foot)

cd /home/ee/sensing_fw

echo "Flashing PRIMARY device firmware..."
echo "Make sure the RIGHT foot device is connected!"
read -p "Press Enter to continue..."

# Flash network core first
echo "Flashing network core..."
nrfjprog --program build_primary_pcb/merged_CPUNET.hex --verify --chiperase --reset

# Small delay
sleep 2

# Flash application core
echo "Flashing application core..."
nrfjprog --program build_primary_pcb/merged.hex --verify --chiperase --reset

echo "Primary device flashed successfully!"
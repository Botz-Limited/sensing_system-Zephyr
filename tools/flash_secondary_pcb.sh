#!/bin/bash
# Flash script for SECONDARY device (Left foot)

cd /home/ee/sensing_fw

echo "Flashing SECONDARY device firmware..."
echo "Make sure the LEFT foot device is connected!"
read -p "Press Enter to continue..."

# Flash network core first
echo "Flashing network core..."
nrfjprog --program build_secondary_pcb/merged_CPUNET.hex --verify --chiperase --reset

# Small delay
sleep 2

# Flash application core
echo "Flashing application core..."
nrfjprog --program build_secondary_pcb/merged.hex --verify --chiperase --reset

echo "Secondary device flashed successfully!"
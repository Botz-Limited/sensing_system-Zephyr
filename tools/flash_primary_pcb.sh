#!/bin/bash
# Flash script for PRIMARY device (Right foot)

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
nrfjprog --program build_primary_pcb/merged_CPUNET.hex --verify --sectorerase -f NRF53 --coprocessor CP_NETWORK

# Small delay
sleep 2

# Flash application core
echo "Flashing application core..."
nrfjprog --program build_primary_pcb/merged.hex --verify --sectorerase -f NRF53 --coprocessor CP_APPLICATION

# Final reset to start the application
echo "Resetting device..."
nrfjprog --reset -f NRF53

echo "Primary device flashed successfully!"
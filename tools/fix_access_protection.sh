#!/bin/bash
# Script to fix access protection issues on nRF5340

echo "=== Access Protection Fix for nRF5340 ==="
echo "This script will completely unlock your device."
echo "WARNING: This will erase ALL data on the device!"
read -p "Press Enter to continue or Ctrl+C to cancel..."

# Function to check result
check_result() {
    if [ $? -ne 0 ]; then
        echo "ERROR: $1 failed!"
        echo "Trying alternative method..."
        return 1
    fi
    return 0
}

# Method 1: Standard recovery
echo ""
echo "Method 1: Standard recovery..."
nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK
if check_result "Network core recovery"; then
    echo "Network core recovered successfully"
else
    # Method 2: Force recovery with CTRL-AP
    echo "Method 2: Forced recovery via CTRL-AP..."
    nrfjprog --recover -f NRF53 --coprocessor CP_NETWORK --qspiini /dev/null
fi

nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION
if check_result "Application core recovery"; then
    echo "Application core recovered successfully"
else
    # Method 2: Force recovery with CTRL-AP
    echo "Method 2: Forced recovery via CTRL-AP..."
    nrfjprog --recover -f NRF53 --coprocessor CP_APPLICATION --qspiini /dev/null
fi

# Wait for recovery to complete
sleep 3

# Verify cores are accessible
echo ""
echo "Verifying cores are accessible..."
nrfjprog --memrd 0x00000000 -f NRF53 --coprocessor CP_APPLICATION
if [ $? -eq 0 ]; then
    echo "✓ Application core is accessible"
else
    echo "✗ Application core still protected!"
fi

nrfjprog --memrd 0x01000000 -f NRF53 --coprocessor CP_NETWORK
if [ $? -eq 0 ]; then
    echo "✓ Network core is accessible"
else
    echo "✗ Network core still protected!"
fi

echo ""
echo "Access protection fix complete."
echo "You can now flash your device using the normal flash scripts."
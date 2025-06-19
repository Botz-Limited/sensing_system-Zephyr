#!/bin/bash

# Script to run the log decoder tests

echo "=== Running Log Decoder Tests ==="
echo

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    echo "Error: Please run this script from the sensing_fw directory"
    exit 1
fi

# Build the project first to ensure protobuf files are generated
echo "Building project to generate protobuf files..."
west build -p auto

if [ $? -ne 0 ]; then
    echo "Error: Build failed"
    exit 1
fi

echo
echo "Running decoder tests..."
echo

# Run the specific decoder tests
west twister -T tests/data \
    --platform native_posix \
    --inline-logs \
    -s data.log_decoder.test_list_all_logs \
    -s data.log_decoder.test_decode_foot_sensor_log \
    -s data.log_decoder.test_decode_bhi360_log \
    -s data.log_decoder.test_validate_timing

echo
echo "=== Decoder Tests Complete ==="

# Build standalone decoder if requested
if [ "$1" == "--build-standalone" ]; then
    echo
    echo "Building standalone decoder..."
    cd tools
    make clean
    make
    cd ..
    echo "Standalone decoder built: tools/log_decoder"
fi
#!/bin/bash
# Script to run D2D integration tests

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "D2D Integration Test Runner"
echo "=========================="

# Default serial ports (adjust as needed)
PRIMARY_PORT="${PRIMARY_PORT:-/dev/ttyACM0}"
SECONDARY_PORT="${SECONDARY_PORT:-/dev/ttyACM1}"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if serial ports exist
check_serial_ports() {
    local missing=0
    
    if [ ! -e "$PRIMARY_PORT" ]; then
        echo -e "${RED}Error: Primary device port $PRIMARY_PORT not found${NC}"
        missing=1
    fi
    
    if [ ! -e "$SECONDARY_PORT" ]; then
        echo -e "${RED}Error: Secondary device port $SECONDARY_PORT not found${NC}"
        missing=1
    fi
    
    if [ $missing -eq 1 ]; then
        echo ""
        echo "Available serial ports:"
        ls -la /dev/tty* | grep -E "(ACM|USB)" || echo "No USB serial ports found"
        echo ""
        echo "Usage: PRIMARY_PORT=/dev/ttyACM0 SECONDARY_PORT=/dev/ttyACM1 $0"
        exit 1
    fi
}

# Function to run unit tests
run_unit_tests() {
    echo -e "\n${YELLOW}Running D2D Unit Tests...${NC}"
    cd "$PROJECT_ROOT"
    
    # Run bluetooth module tests (includes D2D tests)
    west twister -T tests/bluetooth \
        --platform native_posix \
        --filter "d2d" \
        -v || true
    
    echo -e "${GREEN}Unit tests completed${NC}"
}

# Function to run integration tests
run_integration_tests() {
    echo -e "\n${YELLOW}Running D2D Integration Tests...${NC}"
    
    # Check if devices are connected
    check_serial_ports
    
    # Run the Python integration test script
    python3 "$SCRIPT_DIR/test_d2d_integration.py" \
        --primary "$PRIMARY_PORT" \
        --secondary "$SECONDARY_PORT" \
        --verbose
}

# Function to run manual test commands
run_manual_tests() {
    echo -e "\n${YELLOW}Manual D2D Test Commands${NC}"
    echo "========================"
    echo ""
    echo "Connect to devices using minicom or screen:"
    echo "  Primary:   minicom -D $PRIMARY_PORT -b 115200"
    echo "  Secondary: minicom -D $SECONDARY_PORT -b 115200"
    echo ""
    echo "Test Commands:"
    echo "1. Check D2D status:"
    echo "   shell> d2d status"
    echo ""
    echo "2. Trigger foot sensor data (on secondary):"
    echo "   shell> foot_sensor test"
    echo ""
    echo "3. Trigger BHI360 data (on secondary):"
    echo "   shell> motion_sensor test"
    echo ""
    echo "4. Send time sync (on primary):"
    echo "   shell> control set_time 1234567890"
    echo ""
    echo "5. Check connection (on primary):"
    echo "   shell> bt info"
    echo ""
}

# Main menu
show_menu() {
    echo ""
    echo "Select test type:"
    echo "1) Run unit tests only"
    echo "2) Run integration tests only"
    echo "3) Run all tests"
    echo "4) Show manual test commands"
    echo "5) Exit"
    echo ""
    read -p "Enter choice [1-5]: " choice
    
    case $choice in
        1)
            run_unit_tests
            ;;
        2)
            run_integration_tests
            ;;
        3)
            run_unit_tests
            run_integration_tests
            ;;
        4)
            run_manual_tests
            ;;
        5)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid choice${NC}"
            show_menu
            ;;
    esac
}

# Check if running with arguments
if [ "$1" == "--unit" ]; then
    run_unit_tests
elif [ "$1" == "--integration" ]; then
    run_integration_tests
elif [ "$1" == "--all" ]; then
    run_unit_tests
    run_integration_tests
elif [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --unit         Run unit tests only"
    echo "  --integration  Run integration tests only"
    echo "  --all          Run all tests"
    echo "  --help         Show this help message"
    echo ""
    echo "Environment variables:"
    echo "  PRIMARY_PORT   Primary device serial port (default: /dev/ttyACM0)"
    echo "  SECONDARY_PORT Secondary device serial port (default: /dev/ttyACM1)"
    echo ""
    echo "Example:"
    echo "  PRIMARY_PORT=/dev/ttyUSB0 SECONDARY_PORT=/dev/ttyUSB1 $0 --integration"
else
    # Interactive mode
    show_menu
fi
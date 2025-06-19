#!/bin/bash
# Script to run Bluetooth unit tests using Twister

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Running Bluetooth Module Unit Tests${NC}"
echo "======================================"

# Change to the project root directory
cd ../..

# Run twister for bluetooth tests
echo -e "${YELLOW}Executing Twister tests...${NC}"
west twister -T tests/bluetooth \
    --platform native_posix \
    --inline-logs \
    --verbose \
    --coverage \
    --coverage-tool gcovr

# Check the result
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed!${NC}"
    
    # Generate coverage report if tests passed
    if command -v gcovr &> /dev/null; then
        echo -e "${YELLOW}Generating coverage report...${NC}"
        gcovr --html-details -o coverage.html
        echo -e "${GREEN}Coverage report generated: coverage.html${NC}"
    fi
else
    echo -e "${RED}✗ Some tests failed!${NC}"
    exit 1
fi

# Show test summary
echo ""
echo -e "${GREEN}Test Summary:${NC}"
echo "============="
cat twister-out/twister.log | grep -E "(PASS|FAIL|SKIP)" | tail -20

# Clean up if requested
if [ "$1" == "--clean" ]; then
    echo -e "${YELLOW}Cleaning up test artifacts...${NC}"
    rm -rf twister-out/
    rm -f coverage.html
    echo -e "${GREEN}Cleanup complete${NC}"
fi
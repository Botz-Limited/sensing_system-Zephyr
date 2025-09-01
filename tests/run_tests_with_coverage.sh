#!/bin/bash
# Script to run tests with coverage report generation

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Running Tests with Coverage Analysis${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Change to project root
cd "$(dirname "$0")/.."

# Clean previous results
echo -e "${YELLOW}Cleaning previous test results...${NC}"
rm -rf twister-out* twister-coverage

# Detect the correct gcov version
echo -e "${YELLOW}Detecting gcov version...${NC}"
GCC_VERSION=$(gcc --version | head -1 | grep -oP '\d+\.\d+\.\d+' | cut -d. -f1)
GCOV_TOOL="gcov"

# Check if we need a specific gcov version
if command -v gcov-${GCC_VERSION} &> /dev/null; then
    GCOV_TOOL="gcov-${GCC_VERSION}"
    echo -e "${GREEN}Using ${GCOV_TOOL} to match GCC version${NC}"
else
    echo -e "${YELLOW}Using default gcov (may cause version mismatch warnings)${NC}"
fi

# Run all tests with coverage
echo -e "${YELLOW}Running all tests with coverage...${NC}"
echo ""

if west twister -T tests \
    --platform native_posix \
    --coverage \
    --coverage-tool gcovr \
    --gcov-tool "${GCOV_TOOL}" \
    --coverage-formats html \
    --inline-logs \
    --no-skipped-report \
    2>&1 | tee test_coverage.log; then
    
    echo ""
    echo -e "${GREEN}✓ All tests completed successfully!${NC}"
    
    # Check if coverage report was generated
    if [ -f "twister-out/coverage/index.html" ]; then
        echo -e "${GREEN}✓ Coverage report generated successfully!${NC}"
        echo ""
        echo -e "${BLUE}Coverage Report Location:${NC}"
        echo -e "  HTML: ${PWD}/twister-out/coverage/index.html"
        
        # Try to extract coverage percentage from text report
        if [ -f "twister-out/coverage.txt" ]; then
            echo -e "  Text: ${PWD}/twister-out/coverage.txt"
            echo ""
            echo -e "${BLUE}Coverage Summary:${NC}"
            grep -E "TOTAL|lines\.\.\." twister-out/coverage.txt | tail -5
        fi
    else
        echo -e "${YELLOW}⚠ Coverage report not found in expected location${NC}"
    fi
else
    echo -e "${RED}✗ Test execution failed!${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Coverage analysis complete!${NC}"
echo -e "${BLUE}========================================${NC}"
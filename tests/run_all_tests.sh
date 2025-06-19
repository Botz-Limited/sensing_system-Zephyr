#!/bin/bash
# Master script to run all unit tests in the project

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test results
PASSED=0
FAILED=0
SKIPPED=0

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Running All Sensing FW Unit Tests${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Change to project root
cd "$(dirname "$0")/.."

# List of test directories
TEST_DIRS=(
    "app"
    "app_events"
    "battery"
    "bluetooth"
    "control_service"
    "data"
    "foot_sensor"
    "motion_sensor"
    "packetizer"
)

# Function to run tests for a module
run_module_tests() {
    local module=$1
    echo -e "${YELLOW}Testing module: $module${NC}"
    echo "----------------------------------------"
    
    if [ -d "tests/$module" ]; then
        west twister -T tests/$module \
            --platform native_posix \
            --inline-logs \
            --no-skipped-report \
            2>&1 | tee /tmp/${module}_test.log
        
        if [ ${PIPESTATUS[0]} -eq 0 ]; then
            echo -e "${GREEN}✓ $module tests PASSED${NC}"
            ((PASSED++))
        else
            echo -e "${RED}✗ $module tests FAILED${NC}"
            ((FAILED++))
        fi
    else
        echo -e "${YELLOW}⚠ $module test directory not found${NC}"
        ((SKIPPED++))
    fi
    echo ""
}

# Run tests for each module
for module in "${TEST_DIRS[@]}"; do
    run_module_tests "$module"
done

# Generate coverage report if all tests pass
if [ $FAILED -eq 0 ] && [ $PASSED -gt 0 ]; then
    echo -e "${YELLOW}Generating coverage report...${NC}"
    
    # Try to generate coverage report, but don't fail if it doesn't work
    if west twister -T tests \
        --platform native_posix \
        --coverage \
        --coverage-tool gcovr \
        --coverage-formats html \
        --outdir twister-coverage \
        2>&1 | tee /tmp/coverage.log; then
        
        if [ -f "twister-coverage/coverage/index.html" ]; then
            echo -e "${GREEN}Coverage report generated: twister-coverage/coverage/index.html${NC}"
        else
            echo -e "${YELLOW}⚠ Coverage report generation completed but no HTML output found${NC}"
        fi
    else
        echo -e "${YELLOW}⚠ Coverage report generation failed (this is optional)${NC}"
        echo -e "${YELLOW}  Tests still passed successfully!${NC}"
    fi
fi

# Summary
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Test Summary${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Passed:${NC} $PASSED"
echo -e "${RED}Failed:${NC} $FAILED"
echo -e "${YELLOW}Skipped:${NC} $SKIPPED"
echo ""

# Exit with appropriate code
if [ $FAILED -gt 0 ]; then
    echo -e "${RED}Some tests failed!${NC}"
    exit 1
else
    echo -e "${GREEN}All tests passed!${NC}"
    exit 0
fi
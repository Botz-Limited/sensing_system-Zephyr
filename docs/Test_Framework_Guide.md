# Sensing FW Test Framework Guide

**Version:** 1.0  
**Date:** June 2025  
**Scope:** Complete guide for testing the sensing firmware using Zephyr's test framework  
**Purpose:** Comprehensive reference for writing, running, and analyzing tests for firmware validation

---

## Table of Contents
1. [Overview](#overview)
2. [Test Architecture](#test-architecture)
3. [Running Tests](#running-tests)
4. [Writing Tests](#writing-tests)
5. [Coverage Analysis](#coverage-analysis)
6. [Troubleshooting](#troubleshooting)
7. [Best Practices](#best-practices)

## Overview

The Sensing FW project uses Zephyr's testing framework (ztest) for unit testing. The test suite covers all major modules and provides comprehensive validation of the firmware functionality.

### Test Modules

The project includes tests for the following modules:
- **app**: Application core functionality and FOTA progress tracking
- **app_events**: Event management system
- **battery**: Battery monitoring and status
- **bluetooth**: BLE connectivity and state management
- **control_service**: Control service GATT implementation
- **data**: Data logging and file system operations
- **foot_sensor**: Foot sensor data handling
- **motion_sensor**: Motion sensor (BHI360) data handling
- **packetizer**: Data packet creation and parsing

## Test Architecture

### Directory Structure
```
tests/
├── app/
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── test_app.cpp
│   └── testcase.yaml
├── app_events/
├── battery/
├── bluetooth/
├── control_service/
├── data/
├── foot_sensor/
├── motion_sensor/
├── packetizer/
├── run_all_tests.sh
├── run_tests_simple.sh
└── run_tests_with_coverage.sh
```

### Test Framework: Zephyr Test (ztest)

The tests use Zephyr's built-in testing framework which provides:
- Test suite organization
- Setup and teardown functions
- Assertion macros
- Fixture support
- Native POSIX execution

## Running Tests

### Quick Start

1. **Run all tests (simple)**:
   ```bash
   cd /home/ee/sensing_fw
   ./tests/run_tests_simple.sh
   ```

2. **Run all tests with detailed output**:
   ```bash
   ./tests/run_all_tests.sh
   ```

3. **Run tests with coverage analysis**:
   ```bash
   ./tests/run_tests_with_coverage.sh
   ```

### Running Individual Test Modules

To run tests for a specific module:

```bash
# Run app module tests
west twister -T tests/app --platform native_posix

# Run bluetooth module tests
west twister -T tests/bluetooth --platform native_posix

# Run with verbose output
west twister -T tests/data --platform native_posix -v
```

### Running with Coverage

To generate coverage reports:

```bash
# Run with coverage (requires gcov-12)
west twister -T tests \
    --platform native_posix \
    --coverage \
    --coverage-tool gcovr \
    --gcov-tool gcov-12 \
    --coverage-formats html

# Coverage report will be in: twister-out/coverage/index.html
```

## Writing Tests

### Basic Test Structure

```cpp
#include <zephyr/ztest.h>
#include <zephyr/kernel.h>

// Test fixture
struct my_fixture {
    int value;
    bool initialized;
};

// Setup function
static void *my_setup(void)
{
    struct my_fixture *fixture = k_malloc(sizeof(struct my_fixture));
    fixture->value = 0;
    fixture->initialized = true;
    return fixture;
}

// Teardown function
static void my_teardown(void *f)
{
    k_free(f);
}

// Define test suite
ZTEST_SUITE(my_module, NULL, my_setup, NULL, NULL, my_teardown);

// Write tests
ZTEST_F(my_module, test_initialization)
{
    zassert_true(fixture->initialized, "Should be initialized");
    zassert_equal(fixture->value, 0, "Initial value should be 0");
}

ZTEST(my_module, test_without_fixture)
{
    int result = 2 + 2;
    zassert_equal(result, 4, "Math should work");
}
```

### Assertion Macros

Common ztest assertions:
- `zassert_true(cond, msg, ...)` - Assert condition is true
- `zassert_false(cond, msg, ...)` - Assert condition is false
- `zassert_equal(a, b, msg, ...)` - Assert values are equal
- `zassert_not_equal(a, b, msg, ...)` - Assert values are not equal
- `zassert_not_null(ptr, msg, ...)` - Assert pointer is not NULL
- `zassert_mem_equal(buf1, buf2, size, msg, ...)` - Assert memory regions are equal
- `zassert_within(val, target, delta, msg, ...)` - Assert value is within range

### Test Configuration

Each test module has a `prj.conf` file for configuration:

```conf
# Enable testing
CONFIG_ZTEST=y
CONFIG_ZTEST_NEW_API=y

# Enable coverage (optional)
CONFIG_COVERAGE=y
CONFIG_COVERAGE_GCOV=y

# Module-specific configs
CONFIG_HEAP_MEM_POOL_SIZE=4096
```

### CMakeLists.txt

Each test module needs a CMakeLists.txt:

```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(my_module_test)

# Include directories
target_include_directories(app PRIVATE 
    ${CMAKE_SOURCE_DIR}/../../include
)

# Add test sources
target_sources(app PRIVATE
    test_my_module.cpp
)

# Set compile options
target_compile_options(app PRIVATE -std=c++20)
```

## Coverage Analysis

### Prerequisites

The system must have matching GCC and gcov versions. The project uses:
- GCC 12.3.0
- gcov-12

### Generating Coverage Reports

1. **Using the automated script**:
   ```bash
   ./tests/run_tests_with_coverage.sh
   ```

2. **Manual coverage generation**:
   ```bash
   west twister -T tests \
       --platform native_posix \
       --coverage \
       --coverage-tool gcovr \
       --gcov-tool gcov-12 \
       --coverage-formats html
   ```

### Viewing Coverage

- HTML report: `twister-out/coverage/index.html`
- Can be viewed in a browser or served via web server

### Coverage Goals

- Aim for >80% code coverage for critical modules
- Focus on testing error paths and edge cases
- Exclude test code itself from coverage metrics

## Troubleshooting

### Common Issues

1. **Build Failures**
   - Check that all includes are available
   - Verify CMakeLists.txt paths are correct
   - Ensure prj.conf has required configs

2. **Test Failures**
   - Check test fixture initialization
   - Verify mock implementations match expectations
   - Look for state persistence between tests

3. **Coverage Issues**
   - Ensure gcov version matches GCC version
   - Use `gcov-12` instead of default `gcov`
   - Clean build artifacts before coverage run

### Debugging Tests

```bash
# Run with verbose output
west twister -T tests/module_name -v

# Check build logs
cat twister-out/native_posix_native/host/module.unit/build.log

# Check test execution logs
cat twister-out/native_posix_native/host/module.unit/handler.log
```

## Best Practices

### Test Design

1. **Isolation**: Each test should be independent
2. **Repeatability**: Tests should produce consistent results
3. **Clarity**: Test names should describe what they test
4. **Coverage**: Test both success and failure paths
5. **Mocking**: Mock external dependencies appropriately

### Code Organization

1. **One test file per module**: Keep tests close to the code they test
2. **Descriptive names**: Use clear, descriptive test names
3. **Setup/Teardown**: Use fixtures for common initialization
4. **Comments**: Document complex test scenarios

### Performance

1. **Fast tests**: Keep individual tests under 1 second
2. **Minimal I/O**: Avoid file system or network operations
3. **Native platform**: Use native_posix for speed

### Maintenance

1. **Update tests with code**: Keep tests synchronized with implementation
2. **Remove obsolete tests**: Delete tests for removed features
3. **Refactor tests**: Improve test quality over time
4. **Document changes**: Update this guide when framework changes

## Example Test Scenarios

### Testing State Machines

```cpp
ZTEST_F(bluetooth, test_state_transitions)
{
    // Start in disconnected state
    fixture->state = BT_STATE_DISCONNECTED;
    
    // Transition to advertising
    bluetooth_start_advertising_mock(fixture);
    zassert_equal(fixture->state, BT_STATE_ADVERTISING, "Should be advertising");
    
    // Connect
    bluetooth_connect_mock(fixture);
    zassert_equal(fixture->state, BT_STATE_CONNECTED, "Should be connected");
}
```

### Testing Error Handling

```cpp
ZTEST_F(data, test_start_logging_without_mount)
{
    // Ensure file system is not mounted
    fixture->file_system_mounted = false;
    
    // Try to start logging
    err_t ret = start_logging_mock(fixture);
    
    // Verify error
    zassert_equal(ret, err_t::FILE_SYSTEM_ERROR, "Should fail without mount");
}
```

### Testing Data Processing

```cpp
ZTEST_F(packetizer, test_crc_calculation)
{
    uint8_t test_data[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
    uint32_t crc = calculate_crc32_mock(test_data, sizeof(test_data));
    
    // CRC-32 of "123456789" should be 0xCBF43926
    zassert_equal(crc, 0xCBF43926, "CRC should match expected value");
}
```

## Continuous Integration

The test suite can be integrated into CI/CD pipelines:

```yaml
# Example GitHub Actions workflow
test:
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v2
    - name: Install dependencies
      run: |
        # Install Zephyr SDK
        # Install west
    - name: Run tests
      run: |
        cd sensing_fw
        ./tests/run_all_tests.sh
    - name: Generate coverage
      run: |
        ./tests/run_tests_with_coverage.sh
    - name: Upload coverage
      uses: actions/upload-artifact@v2
      with:
        name: coverage-report
        path: twister-out/coverage/
```

## Summary

The Sensing FW test framework provides comprehensive testing capabilities:
- Unit tests for all major modules
- Coverage analysis support
- Easy-to-use test scripts
- Integration with Zephyr's testing infrastructure

Regular testing ensures code quality and helps catch regressions early in the development process.

---

**End of Guide**
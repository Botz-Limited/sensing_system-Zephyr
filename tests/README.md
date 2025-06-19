# Sensing Firmware Unit Tests

This directory contains comprehensive unit tests for all modules in the Sensing Firmware project using Zephyr's Twister test framework.

## Test Coverage

The test suite covers the following modules:

| Module | Description | Test Files | Coverage |
|--------|-------------|------------|----------|
| **app** | Main application and FOTA progress tracking | `test_app.cpp` | FOTA callbacks, progress tracking, event handling |
| **app_events** | Application event system | `test_app_events.cpp` | All event types, event creation/submission |
| **battery** | Battery monitoring | `test_battery.cpp` | ADC reading, voltage conversion, notifications |
| **bluetooth** | Complete BLE stack | `test_bluetooth_*.cpp` | All BLE services, D2D communication, proxies |
| **data** | File system and logging | `test_data.cpp` | File operations, log management, protobuf |
| **foot_sensor** | Foot sensor hardware | `test_foot_sensor.cpp` | SAADC, timer, DPPI, data acquisition |
| **motion_sensor** | BHI360 motion sensor | `test_motion_sensor.cpp` | Sensor init, data parsing, FIFO handling |
| **packetizer** | Data packetization | `test_packetizer.cpp`, `test_crc32.cpp`, `test_safe_buffer.cpp` | Packet creation, CRC32, buffer safety |

## Running Tests

### Run All Tests
```bash
cd /home/ee/sensing_fw
./tests/run_all_tests.sh
```

### Run Tests for Specific Module
```bash
# Run only bluetooth tests
west twister -T tests/bluetooth --platform native_posix --inline-logs

# Run only data module tests
west twister -T tests/data --platform native_posix --inline-logs
```

### Run with Coverage Analysis
```bash
# All modules with coverage
west twister -T tests --platform native_posix --coverage --coverage-tool gcovr

# Single module with coverage
west twister -T tests/bluetooth --platform native_posix --coverage
```

### Run with Verbose Output
```bash
west twister -T tests --platform native_posix --verbose --inline-logs
```

## Test Structure

Each test module follows this structure:
```
tests/<module>/
├── CMakeLists.txt      # Build configuration
├── prj.conf            # Project configuration
├── testcase.yaml       # Test metadata
└── test_<module>.cpp   # Test implementation
```

## Writing New Tests

### 1. Create Test Directory
```bash
mkdir -p tests/new_module
```

### 2. Create testcase.yaml
```yaml
tests:
  new_module.unit:
    platform_allow: native_posix native_posix_64
    tags: new_module unit
    harness: ztest
```

### 3. Create prj.conf
```conf
CONFIG_ZTEST=y
CONFIG_ZTEST_NEW_API=y
# Add module-specific configs
```

### 4. Create CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(new_module_test)

target_sources(app PRIVATE test_new_module.cpp)
```

### 5. Write Test File
```cpp
#include <zephyr/ztest.h>

ZTEST(new_module, test_function)
{
    // Test implementation
    zassert_equal(1, 1, "Test should pass");
}
```

## Test Framework

### Ztest Macros
- `ZTEST(suite, test_name)` - Define a test
- `ZTEST_F(suite, test_name)` - Test with fixture
- `ZTEST_SUITE(name, predicate, setup, before, after, teardown)` - Define test suite

### Assertions
- `zassert_true(cond, msg, ...)` - Assert condition is true
- `zassert_false(cond, msg, ...)` - Assert condition is false
- `zassert_equal(a, b, msg, ...)` - Assert values are equal
- `zassert_not_equal(a, b, msg, ...)` - Assert values are not equal
- `zassert_null(ptr, msg, ...)` - Assert pointer is NULL
- `zassert_not_null(ptr, msg, ...)` - Assert pointer is not NULL
- `zassert_mem_equal(buf1, buf2, size, msg, ...)` - Assert memory regions are equal

### Mocking with FFF
The tests use FFF (Fake Function Framework) for mocking:

```cpp
DEFINE_FFF_GLOBALS;
FAKE_VALUE_FUNC(int, function_name, param_type1, param_type2);
FAKE_VOID_FUNC(function_name, param_type);

// In test
RESET_FAKE(function_name);
function_name_fake.return_val = 0;
function_name_fake.custom_fake = custom_implementation;
```

## Coverage Goals

- **Function Coverage**: 100% of public functions
- **Line Coverage**: >80% of executable lines
- **Branch Coverage**: >70% of decision branches

## CI/CD Integration

The test suite is designed to be integrated into CI/CD pipelines:

```yaml
# Example GitHub Actions
- name: Run Unit Tests
  run: |
    west twister -T tests --platform native_posix --inline-logs
    
- name: Generate Coverage
  run: |
    west twister -T tests --platform native_posix --coverage --coverage-tool gcovr
```

## Troubleshooting

### Common Issues

1. **Build Failures**
   ```bash
   # Clean build directory
   rm -rf build/
   west build -t pristine
   ```

2. **Missing Dependencies**
   ```bash
   # Ensure Zephyr SDK is installed
   west update
   ```

3. **Test Timeouts**
   ```bash
   # Increase timeout
   west twister -T tests --timeout-multiplier 2
   ```

### Debug Failed Tests
```bash
# Check detailed logs
cat twister-out/native_posix/<test_name>/handler.log

# Run with debugger
west twister -T tests/<module> --platform native_posix --gdb
```

## Module-Specific Test Details

### Bluetooth Tests
- Comprehensive coverage of all BLE services
- D2D communication protocol testing
- FOTA and file proxy functionality
- Mock BLE stack functions

### Data Module Tests
- File system operations
- Log rotation and management
- Protobuf encoding/decoding
- Flash memory operations

### Sensor Tests
- Hardware abstraction testing
- Interrupt handling
- Data acquisition and buffering
- Command processing

## Future Improvements

1. **Integration Tests** - Test module interactions
2. **Performance Tests** - Benchmark critical paths
3. **Stress Tests** - Test under high load
4. **Hardware-in-Loop** - Test with real hardware
5. **Fuzzing** - Test with random inputs
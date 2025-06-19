# Bluetooth Module Unit Tests

This directory contains comprehensive unit tests for all functions in the Bluetooth module using the Zephyr Test Framework (Ztest) and Twister.

## Test Coverage

The test suite covers the following modules:

### 1. **bluetooth_debug.cpp**
- `bt_debug_check_security()` - Tests security level checking with various connection states
- `bt_debug_list_bonds()` - Tests bond listing functionality
- `bt_debug_check_services()` - Tests service registration verification
- `bt_debug_force_repairing()` - Tests forced re-pairing functionality
- `bt_debug_test_characteristic_access()` - Tests characteristic access diagnostics
- `bt_debug_info_service_status()` - Tests comprehensive debug information output

### 2. **ble_d2d_tx.cpp**
- `ble_d2d_tx_init()` - Tests initialization
- `ble_d2d_tx_set_connection()` - Tests connection management
- `ble_d2d_tx_send_foot_sensor_data()` - Tests foot sensor data transmission
- `ble_d2d_tx_send_fota_complete()` - Tests FOTA completion notification
- `ble_d2d_tx_send_*_log_available()` - Tests log availability notifications
- `ble_d2d_tx_send_*_req_id_path()` - Tests file path notifications
- `ble_d2d_tx_send_bhi360_data*()` - Tests BHI360 data transmission
- `ble_d2d_tx_send_status()` - Tests status updates
- `ble_d2d_tx_send_*_command()` - Tests control command forwarding

### 3. **ble_d2d_rx.cpp**
- Tests for receiving and processing D2D data
- Tests for command forwarding on primary devices
- Tests for message queue integration
- Tests for FOTA proxy integration

### 4. **ble_d2d_file_transfer.cpp**
- `ble_d2d_file_transfer_init()` - Tests initialization
- `ble_d2d_file_client_init()` - Tests client initialization
- `ble_d2d_file_set_callbacks()` - Tests callback registration
- `ble_d2d_file_send_command()` - Tests file commands (LIST, READ, DELETE, INFO)
- `ble_d2d_file_send_data()` - Tests file data transmission
- `ble_d2d_file_send_status()` - Tests status notifications
- File system operation handlers

### 5. **fota_proxy.cpp**
- `fota_proxy_init()` - Tests initialization
- `fota_proxy_set_secondary_conn()` - Tests connection management
- `fota_proxy_notify_status()` - Tests status notifications
- `fota_proxy_handle_secondary_complete()` - Tests completion handling
- Timeout and reset handling
- Command and data forwarding

### 6. **file_proxy.cpp**
- `file_proxy_init()` - Tests initialization
- `file_proxy_set_secondary_conn()` - Tests connection management
- `file_proxy_notify_status()` - Tests status notifications
- `file_proxy_notify_data()` - Tests data notifications
- Command forwarding for file operations
- Timeout handling

### 7. **information_service.cpp**
- `set_device_status()` - Tests device status updates
- `jis_clear_err_status_notify()` - Tests error status clearing
- `jis_foot_sensor_notify()` - Tests foot sensor data notifications
- `jis_bhi360_data*_notify()` - Tests BHI360 data notifications
- `jis_charge_status_notify()` - Tests charge status notifications
- `jis_*_log_available_notify()` - Tests log availability notifications
- `jis_*_req_id_path_notify()` - Tests file path notifications
- `jis_fota_progress_notify()` - Tests FOTA progress notifications

### 8. **control_service.cpp**
- Set time command handling
- Delete log command handling (foot sensor and BHI360)
- Start/stop activity command handling
- Command forwarding for primary devices
- Message queue integration for secondary devices

### 9. **cts.cpp**
- `init_rtc_time()` - Tests RTC initialization
- `set_current_time_from_epoch()` - Tests time setting
- `get_current_epoch_time()` - Tests time retrieval
- `update_cts_characteristic_buffer()` - Tests CTS buffer updates
- `cts_notify()` - Tests time change notifications

## Running the Tests

### Prerequisites
- Zephyr SDK installed and configured
- West tool available
- Native POSIX platform support

### Basic Test Run
```bash
cd /home/ee/sensing_fw/tests/bluetooth
./run_tests.sh
```

### Run with Twister Directly
```bash
cd /home/ee/sensing_fw
west twister -T tests/bluetooth --platform native_posix --inline-logs
```

### Run Specific Test Suite
```bash
west twister -T tests/bluetooth --platform native_posix --test bluetooth.unit
```

### Generate Coverage Report
```bash
west twister -T tests/bluetooth --platform native_posix --coverage --coverage-tool gcovr
```

### Clean Test Artifacts
```bash
./run_tests.sh --clean
```

## Test Configuration

### Platform Support
The tests are configured to run on:
- `native_posix` - Native POSIX execution (Linux/macOS)
- `native_posix_64` - 64-bit native POSIX execution

### Test Framework
- **Ztest**: Zephyr's native testing framework
- **FFF (Fake Function Framework)**: Used for mocking C functions
- **Twister**: Zephyr's test runner

### Configuration Files
- `testcase.yaml` - Defines test metadata and requirements
- `prj.conf` - Project configuration for test builds
- `CMakeLists.txt` - Build configuration

## Mocking Strategy

The tests use FFF (Fake Function Framework) to mock:
- Bluetooth stack functions (`bt_gatt_*`, `bt_conn_*`)
- File system operations (`fs_*`)
- System functions (`k_work_*`, `k_msgq_*`)
- Hardware interfaces (RTC, counter)

## Adding New Tests

1. Create a new test file: `test_<module_name>.cpp`
2. Include necessary headers and define mocks
3. Create test fixture if needed
4. Write test cases using ZTEST macros
5. Add the test file to `CMakeLists.txt`
6. Add test suite call to `test_bluetooth_main.cpp`

### Test Template
```cpp
ZTEST_F(suite_name, test_function_name)
{
    // Setup
    mock_function_fake.return_val = expected_value;
    
    // Execute
    int result = function_under_test(parameters);
    
    // Verify
    zassert_equal(result, expected_result, "Error message");
    zassert_equal(mock_function_fake.call_count, 1, "Should call mock once");
}
```

## Coverage Goals

The test suite aims for:
- **Function Coverage**: 100% of public functions
- **Line Coverage**: >80% of executable lines
- **Branch Coverage**: >70% of decision branches
- **Error Handling**: All error paths tested

## Known Limitations

1. Some static functions cannot be directly tested
2. Hardware-dependent code is mocked
3. Interrupt handlers are not directly testable
4. Some timing-dependent behaviors are simplified

## Troubleshooting

### Common Issues

1. **Build Failures**
   - Ensure all dependencies are installed
   - Check that CONFIG_ZTEST=y is set in prj.conf
   - Verify mock functions match actual function signatures

2. **Test Failures**
   - Check mock setup matches expected behavior
   - Verify test data initialization
   - Ensure proper cleanup between tests

3. **Coverage Issues**
   - Some code may be unreachable in test environment
   - Conditional compilation may exclude code
   - Consider using different CONFIG options for testing

## Future Improvements

1. Add integration tests for module interactions
2. Add performance benchmarks
3. Add stress tests for concurrent operations
4. Improve coverage of error conditions
5. Add tests for memory leak detection
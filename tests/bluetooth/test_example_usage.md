# Example Test Usage

## Running All Bluetooth Tests

```bash
# From the project root
cd /home/ee/sensing_fw
west twister -T tests/bluetooth --platform native_posix --inline-logs

# Or use the convenience script
cd tests/bluetooth
./run_tests.sh
```

## Running Individual Test Suites

### Test bluetooth_debug module only
```bash
west twister -T tests/bluetooth --platform native_posix \
    --test-only bluetooth.unit \
    --inline-logs \
    --filter "bluetooth_debug"
```

### Test with verbose output
```bash
west twister -T tests/bluetooth --platform native_posix \
    --verbose \
    --inline-logs
```

### Test with coverage analysis
```bash
west twister -T tests/bluetooth --platform native_posix \
    --coverage \
    --coverage-tool gcovr \
    --coverage-formats html
```

## Understanding Test Output

### Successful Test Run
```
INFO    - 1/1 bluetooth.unit PASSED (native_posix 1.234s)

INFO    - SUITE PASS - 100.00% [1/1]: 1 test suites passed
INFO    - 1 test suites skipped
INFO    - 0 test suites failed
```

### Failed Test Run
```
ERROR   - bluetooth.unit FAILED (native_posix 1.234s)
ERROR   - see: twister-out/native_posix/bluetooth.unit/handler.log

INFO    - SUITE FAIL - 0.00% [0/1]: 0 test suites passed
INFO    - 0 test suites skipped  
INFO    - 1 test suites failed
```

## Debugging Failed Tests

1. Check the detailed log:
```bash
cat twister-out/native_posix/bluetooth.unit/handler.log
```

2. Run with debug output:
```bash
west twister -T tests/bluetooth --platform native_posix \
    --verbose --inline-logs --log-level debug
```

3. Run under debugger:
```bash
west twister -T tests/bluetooth --platform native_posix \
    --gdb --gdb-server
```

## Test Development Workflow

1. **Write the test**
```cpp
ZTEST(bluetooth_debug, test_new_function)
{
    // Your test code here
}
```

2. **Build and run locally**
```bash
west build -t run -b native_posix tests/bluetooth
```

3. **Run with Twister**
```bash
west twister -T tests/bluetooth --platform native_posix
```

4. **Check coverage**
```bash
west twister -T tests/bluetooth --platform native_posix --coverage
firefox twister-out/coverage/index.html
```

## Common Test Patterns

### Testing Success Cases
```cpp
ZTEST_F(suite, test_success)
{
    // Setup
    mock_func_fake.return_val = 0;
    
    // Execute
    int ret = function_under_test();
    
    // Verify
    zassert_equal(ret, 0, "Should succeed");
}
```

### Testing Error Handling
```cpp
ZTEST_F(suite, test_error_handling)
{
    // Setup
    mock_func_fake.return_val = -EINVAL;
    
    // Execute
    int ret = function_under_test();
    
    // Verify
    zassert_equal(ret, -EINVAL, "Should propagate error");
}
```

### Testing Callbacks
```cpp
ZTEST_F(suite, test_callback)
{
    // Setup
    mock_func_fake.custom_fake = [](callback_t cb, void *data) {
        cb(test_data, data);
        return 0;
    };
    
    // Execute and verify in callback
}
```

### Testing State Changes
```cpp
ZTEST_F(suite, test_state_change)
{
    // Initial state
    zassert_equal(get_state(), STATE_IDLE);
    
    // Trigger state change
    trigger_event();
    
    // Verify new state
    zassert_equal(get_state(), STATE_ACTIVE);
}
```
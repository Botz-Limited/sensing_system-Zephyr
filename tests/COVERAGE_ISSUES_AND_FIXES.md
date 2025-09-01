# Coverage Tool Issues and Fixes

## Problem Summary

The coverage tool was failing with gcov version mismatch errors when trying to generate coverage reports.

## Root Cause

The system has different versions of GCC and gcov installed:
- **GCC version**: 12.3.0
- **gcov version**: 13.3.0 (default)

This version mismatch causes gcov to fail when processing the coverage data files (.gcda and .gcno files) with errors like:
```
version 'B23*' prefer 'B33*'
```

## Solution

### 1. Use Version-Matched gcov

The system has `gcov-12` installed which matches the GCC version. We need to explicitly specify this tool:

```bash
west twister -T tests \
    --platform native_posix \
    --coverage \
    --coverage-tool gcovr \
    --gcov-tool gcov-12 \
    --coverage-formats html,txt
```

### 2. Automated Script

Created `run_tests_with_coverage.sh` that:
- Automatically detects the GCC version
- Looks for a matching gcov version (e.g., gcov-12 for GCC 12)
- Falls back to default gcov if no match is found
- Runs all tests with coverage analysis
- Generates HTML and text coverage reports

### 3. Usage

```bash
# Run tests with coverage
./tests/run_tests_with_coverage.sh

# Coverage reports will be generated in:
# - HTML: twister-out/coverage/index.html
# - Text: twister-out/coverage.txt
```

## Technical Details

### Why Version Mismatch Occurs

1. **Ubuntu 24.04** ships with multiple GCC versions
2. The default `gcov` symlink points to the newest version (13.3.0)
3. But the code is compiled with GCC 12.3.0 (possibly the system default for builds)
4. gcov file formats change between major versions

### Coverage File Formats

- `.gcno` files: Generated during compilation, contain program flow graph
- `.gcda` files: Generated during execution, contain execution counts
- Both files have version-specific formats that must match

### Alternative Solutions

1. **Use matching compiler**: 
   ```bash
   export CC=gcc-13
   export CXX=g++-13
   ```

2. **Use lcov instead of gcovr**:
   ```bash
   --coverage-tool lcov
   ```

3. **Build with specific toolchain**:
   ```bash
   west build -- -DCMAKE_C_COMPILER=gcc-13
   ```

## Best Practices

1. Always ensure gcov version matches GCC version
2. Use the provided script for consistent results
3. Check coverage reports after significant code changes
4. Aim for >80% code coverage for critical modules

## Troubleshooting

If coverage still fails:

1. Check installed versions:
   ```bash
   gcc --version
   gcov --version
   ls /usr/bin/gcov*
   ```

2. Clean build artifacts:
   ```bash
   rm -rf twister-out* build/
   ```

3. Verify coverage is enabled in prj.conf:
   ```
   CONFIG_COVERAGE=y
   CONFIG_COVERAGE_GCOV=y
   ```
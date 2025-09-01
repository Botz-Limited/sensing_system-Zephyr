# Makefile for Bluetooth Unit Tests

.PHONY: all test coverage clean help

# Default target
all: test

# Run all tests
test:
	@echo "Running Bluetooth unit tests..."
	@cd ../.. && west twister -T tests/bluetooth --platform native_posix --inline-logs

# Run tests with coverage
coverage:
	@echo "Running tests with coverage analysis..."
	@cd ../.. && west twister -T tests/bluetooth --platform native_posix \
		--coverage --coverage-tool gcovr --coverage-formats html
	@echo "Coverage report available at: ../../twister-out/coverage/index.html"

# Run tests verbosely
verbose:
	@echo "Running tests with verbose output..."
	@cd ../.. && west twister -T tests/bluetooth --platform native_posix \
		--verbose --inline-logs

# Run specific test suite
test-debug:
	@echo "Running bluetooth_debug tests only..."
	@cd ../.. && west twister -T tests/bluetooth --platform native_posix \
		--inline-logs --filter "bluetooth_debug"

test-d2d:
	@echo "Running D2D tests only..."
	@cd ../.. && west twister -T tests/bluetooth --platform native_posix \
		--inline-logs --filter "d2d"

test-services:
	@echo "Running service tests only..."
	@cd ../.. && west twister -T tests/bluetooth --platform native_posix \
		--inline-logs --filter "service"

# Clean test artifacts
clean:
	@echo "Cleaning test artifacts..."
	@rm -rf ../../twister-out/
	@rm -f ../../coverage.html
	@rm -rf ../../build/
	@echo "Clean complete."

# Build tests without running
build:
	@echo "Building tests..."
	@cd ../.. && west build -b native_posix tests/bluetooth

# Run tests under debugger
debug:
	@echo "Running tests under debugger..."
	@cd ../.. && west twister -T tests/bluetooth --platform native_posix \
		--gdb --gdb-server

# Show help
help:
	@echo "Bluetooth Unit Tests - Make Targets"
	@echo "=================================="
	@echo "  make test        - Run all unit tests"
	@echo "  make coverage    - Run tests with coverage analysis"
	@echo "  make verbose     - Run tests with verbose output"
	@echo "  make test-debug  - Run bluetooth_debug tests only"
	@echo "  make test-d2d    - Run D2D communication tests only"
	@echo "  make test-services - Run service tests only"
	@echo "  make clean       - Remove test artifacts"
	@echo "  make build       - Build tests without running"
	@echo "  make debug       - Run tests under debugger"
	@echo "  make help        - Show this help message"
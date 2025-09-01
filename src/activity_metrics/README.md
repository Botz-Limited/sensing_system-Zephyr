# Activity Metrics Module

This module processes sensor data from both foot pressure sensors and BHI360 IMU to calculate real-time running metrics.

## Module Structure

```
activity_metrics/
├── activity_metrics.cpp    # Main module implementation
├── CMakeLists.txt         # Build configuration
├── Kconfig                 # Module configuration options
├── README.md              # This file
└── zephyr/
    └── module.yml         # Zephyr module definition
```

## Features

- **Step Detection**: Processes 8-channel pressure data to detect ground contact events
- **Gait Analysis**: Calculates contact time, flight time, and foot strike patterns
- **Cadence Calculation**: Uses BHI360 step count to determine running cadence
- **Form Scoring**: Evaluates running form based on multiple metrics
- **Balance Assessment**: Compares left/right foot metrics for symmetry
- **Efficiency Calculation**: Based on duty factor (contact time ratio)
- **Fatigue Detection**: Monitors changes from baseline performance

## Data Flow

1. Receives `MSG_TYPE_FOOT_SAMPLES` from foot sensor module via message queue
2. Receives `MSG_TYPE_BHI360_*` messages from motion sensor module
3. Processes data in dedicated thread (non-blocking)
4. Sends real-time metrics to Bluetooth module for BLE transmission
5. Sends periodic records to data module for logging

## Configuration

Enable in `prj.conf`:
```
CONFIG_ACTIVITY_METRICS_MODULE=y
CONFIG_ACTIVITY_METRICS_MODULE_LOG_LEVEL_INF=y
```

Kconfig options:
- `CONFIG_ACTIVITY_METRICS_MODULE_STACK_SIZE` - Thread stack size (default: 4096)
- `CONFIG_ACTIVITY_METRICS_MODULE_PRIORITY` - Thread priority (default: 5)

## Dependencies

- Foot sensor module (for pressure data)
- Motion sensor module (for IMU data)
- Step detection algorithm (in app directory)
- Activity session data structures (in include directory)

## Thread Safety

All inter-module communication uses message queues. No shared global variables.
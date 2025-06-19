# Log Decoder Usage Guide

This guide explains how to decode and analyze sensor log files from the device.

## Overview

The log decoder provides tools to:
- List all log files on the device
- Decode foot sensor and BHI360 log files
- Display timing statistics and data samples
- Analyze packet timing and data integrity

## Methods to Decode Logs

### 1. Shell Commands (On Device)

If shell is enabled in your build, you can use these commands directly on the device:

```bash
# List all log files
log list

# Decode latest foot sensor log
log decode foot

# Decode latest BHI360 log
log decode bhi360

# Decode specific file
log decode file /lfs1/hardware/foot_001.pb

# Delete all log files
log delete_all
```

#### Example Output:
```
uart:~$ log list
=== LOG FILE INVENTORY ===
Directory: /lfs1/hardware

FILENAME                  SIZE       TYPE
-------------------------  ---------- ----
foot_001.pb               2048       Foot Sensor
foot_002.pb               4096       Foot Sensor
bhi360_001.pb             8192       BHI360
bhi360_002.pb             16384      BHI360

SUMMARY:
  Foot Sensor Logs: 2
  BHI360 Logs: 2
  Total Files: 4
  Total Size: 30720 bytes (30.00 KB)
```

### 2. Python Script (On Host)

After downloading log files from the device, use the Python decoder:

```bash
# First, build the project to generate protobuf Python files
cd /home/ee/sensing_fw
west build

# Decode a specific log file
python tools/decode_log.py /path/to/foot_001.pb

# List all logs in a directory
python tools/decode_log.py --list /path/to/logs/
```

#### Example Output:
```
=== FOOT SENSOR LOG DECODER ===
File: foot_001.pb
Size: 2048 bytes

--- DECODING LOG CONTENTS ---

HEADER:
  Firmware: 1.0.0
  Frequency: 20 Hz

Packet 0:
  Delta: 0 ms
  Time: 0 ms (0.00 s)
  Readings: [1023, 512, 768, 256, 384, 640, 896, 128]

Packet 1:
  Delta: 50 ms
  Time: 50 ms (0.05 s)
  Readings: [1024, 513, 769, 257, 385, 641, 897, 129]

...

TIMING STATISTICS:
  Min Delta: 49 ms
  Max Delta: 52 ms
  Avg Delta: 50.10 ms
  Jitter: 3 ms
  Expected: 50.00 ms
  Deviation: 0.20%
  Packet Loss: 0.00%
```

### 3. Standalone C++ Decoder (On Host)

Build and use the standalone decoder:

```bash
cd tools
make
./log_decoder /path/to/foot_001.pb
```

## Understanding the Output

### Header Information
- **Firmware Version**: Version of firmware that created the log
- **Sampling Frequency**: Data collection rate in Hz

### Packet Data
- **Delta**: Time in milliseconds since previous packet
- **Time**: Absolute time since logging started
- **Readings**: Sensor data values (8 channels for foot sensor)
- **Quaternion**: Orientation data (BHI360 only)
- **Linear Accel**: Acceleration data (BHI360 only)
- **Step Count**: Cumulative step count (BHI360 only)

### Timing Statistics
- **Min/Max Delta**: Range of time intervals between packets
- **Avg Delta**: Average time between packets
- **Jitter**: Variation in packet timing (max - min)
- **Expected**: Expected delta based on sampling frequency
- **Deviation**: Percentage difference from expected timing
- **Packet Loss**: Estimated packet loss percentage

## Interpreting Results

### Good Timing
- Deviation < 5%
- Low jitter (< 10ms for 20Hz sampling)
- Minimal packet loss (< 1%)

### Timing Issues
- High deviation indicates clock drift
- Large jitter suggests system load issues
- Packet loss indicates buffer overruns or communication problems

## Troubleshooting

### No Log Files Found
- Ensure logging was started and stopped properly
- Check file system is mounted: `fs ls /lfs1/hardware`
- Verify sufficient storage space

### Decode Errors
- Ensure protobuf files are generated (run `west build`)
- Check file is not corrupted
- Verify file is complete (has session end marker)

### Python Script Issues
- Install protobuf: `pip install protobuf`
- Ensure build directory exists with generated .pb files
- Check Python path includes the build directory

## Integration with Application

To enable shell commands in your build:

```conf
# In prj.conf
CONFIG_SHELL=y
CONFIG_LOG_DECODER_SHELL_CMD=y
```

The shell commands are automatically registered and available when the device boots.

## File Format Details

Log files use Protocol Buffers encoding with the following structure:

1. **Header Message**: Contains metadata (firmware version, sampling rate)
2. **Data Messages**: Repeated packets with sensor data and delta timestamps
3. **Session End Message**: Marks end of logging session with final uptime

Each packet includes a `delta_ms` field (2 bytes) indicating milliseconds since the previous packet, enabling accurate timestamp reconstruction without storing absolute timestamps.
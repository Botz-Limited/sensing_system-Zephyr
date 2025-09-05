# Raw Data File Format Specification

**Version:** 1.0  
**File Extension:** `.dat`  
**File Naming:** `rawXXX.dat` where XXX is a 3-digit sequence number (001-999)  
**Storage Location:** `/sd/` (SD card mount point)

## File Structure Overview

Each raw data file consists of three sections:
1. **File Header** - Metadata about the recording session
2. **Data Packets** - Sensor data packets (variable number)
3. **File Footer** - Session summary statistics

All multi-byte values are stored in little-endian format.

## 1. File Header (16 bytes)

```c
struct RawFileHeader {
    char magic[4];           // "BRAW" - File identifier
    uint8_t version;         // Format version (currently 1)
    uint32_t start_time_ms;  // System uptime at recording start (ms)
    uint8_t foot_channels;   // Number of foot sensor channels (8)
    uint8_t reserved[7];     // Reserved for future use (set to 0)
}
```

## 2. Data Packets

Three types of data packets can appear in the file:

### 2.1 Foot Sensor Packet (21 bytes)

```c
struct RawFootPacket {
    uint8_t type;            // 0x02 (PACKET_TYPE_FOOT)
    uint32_t timestamp_ms;   // System uptime (ms)
    uint32_t packet_num;     // Global packet counter
    uint16_t values[8];      // Raw ADC values from 8 channels
}
```

### 2.2 Motion Sensor Packet (29 bytes)

```c
struct RawMotionPacket {
    uint8_t type;            // 0x03 (PACKET_TYPE_MOTION)
    uint32_t timestamp_ms;   // System uptime (ms)
    uint32_t packet_num;     // Global packet counter
    int16_t quat_x;          // Quaternion X (×10,000)
    int16_t quat_y;          // Quaternion Y (×10,000)
    int16_t quat_z;          // Quaternion Z (×10,000)
    int16_t quat_w;          // Quaternion W (×10,000)
    int16_t lacc_x;          // Linear acceleration X (×1,000) m/s²
    int16_t lacc_y;          // Linear acceleration Y (×1,000) m/s²
    int16_t lacc_z;          // Linear acceleration Z (×1,000) m/s²
    int16_t gyro_x;          // Gyroscope X (×10,000) rad/s
    int16_t gyro_y;          // Gyroscope Y (×10,000) rad/s
    int16_t gyro_z;          // Gyroscope Z (×10,000) rad/s
}
```

### 2.3 Combined Packet (45 bytes)

When foot and motion sensor data have similar timestamps (within 50ms), they are combined into a single packet:

```c
struct RawCombinedPacket {
    uint8_t type;            // 0x04 (PACKET_TYPE_COMBINED)
    uint32_t timestamp_ms;   // Average timestamp (ms)
    uint32_t packet_num;     // Global packet counter
    // Foot sensor data
    uint16_t foot_values[8]; // Raw ADC values from 8 channels
    // Motion sensor data (same scaling as above)
    int16_t quat_x, quat_y, quat_z, quat_w;
    int16_t lacc_x, lacc_y, lacc_z;
    int16_t gyro_x, gyro_y, gyro_z;
}
```

## 3. File Footer (21 bytes)

```c
struct RawFileFooter {
    uint8_t type;               // 0xFF (PACKET_TYPE_FOOTER)
    uint32_t end_time_ms;       // System uptime at recording end (ms)
    uint32_t total_packets;     // Total number of packets written
    uint32_t foot_packets;      // Number of foot-only packets
    uint32_t motion_packets;    // Number of motion-only packets
    uint32_t combined_packets;  // Number of combined packets
}
```

## Packet Type Constants

| Type | Value | Description |
|------|-------|-------------|
| PACKET_TYPE_HEADER | 0x01 | File header (not used as packet type) |
| PACKET_TYPE_FOOT | 0x02 | Foot sensor data only |
| PACKET_TYPE_MOTION | 0x03 | Motion sensor data only |
| PACKET_TYPE_COMBINED | 0x04 | Combined foot + motion data |
| PACKET_TYPE_FOOTER | 0xFF | End of file marker |

## Data Scaling

### Motion Sensor Fixed-Point Encoding

| Sensor | Scale Factor | Resolution | Range |
|--------|--------------|------------|-------|
| Quaternion | ×10,000 | 0.0001 | ±3.2767 |
| Linear Acceleration | ×1,000 | 0.001 m/s² | ±32.767 m/s² |
| Gyroscope | ×10,000 | 0.0001 rad/s | ±3.2767 rad/s |

To convert back to floating-point:
- Quaternion: `float_value = int16_value / 10000.0`
- Acceleration: `float_value = int16_value / 1000.0`
- Gyroscope: `float_value = int16_value / 10000.0`

## Sampling Rates

- **Foot Sensor**: 100 Hz
- **Motion Sensor**: 100 Hz (quaternion, acceleration, gyroscope)
- **Combined Packets**: Variable (depends on timestamp alignment)

## Example File Layout

```
[Header (16 bytes)]
[Foot Packet (21 bytes)]      // timestamp: 100ms
[Motion Packet (29 bytes)]     // timestamp: 102ms
[Combined Packet (45 bytes)]   // timestamp: 200ms (averaged)
[Foot Packet (21 bytes)]       // timestamp: 300ms
[Motion Packet (29 bytes)]     // timestamp: 305ms
...
[Footer (21 bytes)]
```

## Notes

1. **Packet Ordering**: Packets are written in the order they are received, not necessarily in timestamp order
2. **Timestamp Synchronization**: Combined packets use the average of foot and motion timestamps
3. **Buffer Flushing**: Data is buffered in 512-byte blocks and flushed periodically (every second) or when buffer is full
4. **File Size**: At 100Hz for both sensors with perfect synchronization, expect ~16.2 KB/second of combined packets
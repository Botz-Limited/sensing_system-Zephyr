# Activity File Format Specification

## Overview
This document describes the binary file format used for storing activity session data. Activity files are stored with the naming convention `activity_XXX.dat` where XXX is a 3-digit sequence number (001-255).

## File Structure
The activity file consists of three main sections:
1. **Header** - Written when the file is opened at session start
2. **Data Records** - Written every second during active sessions
3. **Footer** - Written when the file is closed at session end

All multi-byte values are stored in little-endian format.

---

## 1. File Header (Version 2)

The header is written once at the beginning of each activity file when logging starts.

### Header Structure (52 bytes)
```c
typedef struct __attribute__((packed)) {
    char magic[4];          // "BOTZ" - File signature
    uint8_t version;        // File format version (currently 2)
    uint32_t start_time;    // Milliseconds since boot
    uint32_t sample_rate;   // Sampling frequency in Hz
    char fw_version[16];    // Firmware version (null-terminated string)
    uint8_t user_height_cm; // User height in centimeters
    uint8_t user_weight_kg; // User weight in kilograms
    uint8_t user_age_years; // User age in years
    uint8_t user_sex;       // User sex (0=female, 1=male)
    battery_info_t battery; // Battery state at session start (4 bytes)
    uint8_t reserved[4];    // Reserved for future use
} ActivityFileHeaderV2;
```

### Battery Info Structure (4 bytes)
```c
typedef struct __attribute__((packed)) {
    uint16_t voltage_mV;    // Battery voltage in millivolts
    int8_t percentage;      // Battery percentage (0-100, -1 if unknown)
    uint8_t status;         // Charging status (0=discharging, 1=charging, 2=full)
} battery_info_t;
```

---

## 2. Activity Metrics Records

Activity metrics are written every second during an active session. Each record contains real-time biomechanical and performance data.

### Record Structure (29 bytes per record)
```c
typedef struct __attribute__((packed)) {
    uint32_t packet_number;         // Sequential packet counter
    uint32_t timestamp_ms;          // Milliseconds since boot
    uint16_t cadence_spm;           // Steps per minute
    uint16_t pace_sec_km;           // Seconds per kilometer
    uint8_t form_score;             // Form quality (0-100%)
    int8_t balance_lr_pct;          // L/R balance (-50 to +50%, negative=left dominant)
    uint16_t ground_contact_ms;     // Ground contact time in milliseconds
    uint16_t flight_time_ms;        // Flight time in milliseconds
    int8_t contact_time_asymmetry;  // Contact time asymmetry (0-100%)
    int8_t force_asymmetry;         // Force asymmetry (0-100%)
    int8_t pronation_asymmetry;     // Pronation asymmetry (0-100%)
    uint8_t left_strike_pattern;    // Left foot strike (0=heel, 1=midfoot, 2=forefoot)
    uint8_t right_strike_pattern;   // Right foot strike (0=heel, 1=midfoot, 2=forefoot)
    int8_t avg_pronation_deg;       // Average pronation in degrees (signed)
    uint8_t vertical_ratio;         // Vertical oscillation ratio (0-100%)
    uint8_t efficiency_score;       // Running efficiency (0-100%)
    uint8_t alerts;                 // Alert bitmask (see below)
} activity_metrics_binary_t;
```

### Alert Bitmask Definition
The `alerts` field is an 8-bit bitmask with the following flags:
- **Bit 0**: High asymmetry detected
- **Bit 1**: Poor form detected
- **Bit 2**: High impact detected
- **Bit 3**: Fatigue detected
- **Bit 4**: Overpronation detected
- **Bit 5**: Unilateral data (only one foot sensor active)
- **Bit 6-7**: Reserved

---

## 3. File Footer (Version 2)

The footer is written once at the end of the activity file when logging stops.

### Footer Structure (20 bytes)
```c
typedef struct __attribute__((packed)) {
    uint32_t end_time;      // Session duration in milliseconds
    uint32_t record_count;  // Total number of data records written
    uint32_t packet_count;  // Total packets processed
    uint32_t file_crc;      // CRC-32 checksum (placeholder, currently 0)
    battery_info_t battery; // Battery state at session end (4 bytes)
} ActivityFileFooterV2;
```

---

## Field Descriptions and Units

### Timing Fields
- **packet_number**: Sequential counter starting from 0, resets if approaching UINT32_MAX
- **timestamp_ms**: System uptime in milliseconds when the record was created
- **end_time**: Total session duration calculated as (current_uptime - start_time)

### Performance Metrics
- **cadence_spm**: Steps per minute (uint16)
- **pace_sec_km**: Running pace in seconds per kilometer (uint16)
- **form_score**: Overall form quality score from 0-100%
- **efficiency_score**: Running efficiency score from 0-100%

### Balance and Asymmetry
- **balance_lr_pct**: Left/right balance as percentage (-50 to +50)
  - Negative values indicate left foot dominance
  - Positive values indicate right foot dominance
  - 0 indicates perfect balance
- **contact_time_asymmetry**: Percentage difference in ground contact time between feet (0-100%)
- **force_asymmetry**: Percentage difference in force application between feet (0-100%)
- **pronation_asymmetry**: Percentage difference in pronation between feet (0-100%)

### Biomechanical Measurements
- **ground_contact_ms**: Time foot is in contact with ground per step (milliseconds)
- **flight_time_ms**: Time both feet are off the ground (milliseconds)
- **avg_pronation_deg**: Average pronation angle in degrees (signed int8)
  - Negative values indicate supination
  - Positive values indicate pronation
- **vertical_ratio**: Vertical oscillation as percentage of stride length (0-100%)

### Strike Pattern
- **left_strike_pattern** / **right_strike_pattern**: Foot strike classification
  - 0 = Heel strike
  - 1 = Midfoot strike
  - 2 = Forefoot strike

### User Profile (in header)
- **user_height_cm**: Height in centimeters (0-255)
- **user_weight_kg**: Weight in kilograms (0-255)
- **user_age_years**: Age in years (0-255)
- **user_sex**: Biological sex (0=female, 1=male)

---

## File Management

### Naming Convention
Files are named as `activity_XXX.dat` where XXX is a zero-padded 3-digit sequence number from 001 to 255.

### Sequence Number Management
- Sequence numbers start at 001
- Numbers increment for each new session
- When reaching 255, the system wraps back to 001
- The system maintains up to 255 activity files

### Storage Location
Activity files are stored in the `/lfs1/hardware/` directory on the external flash filesystem.

### Data Batching
- Records are batched in groups of 2 before writing to optimize flash operations
- Data is buffered in 256-byte pages aligned to flash page boundaries
- Periodic flush occurs every second to ensure data persistence

---

## Example Data Flow

1. **Session Start**:
   - Header (52 bytes) written with magic="BOTZ", version=2, user profile, and initial battery state
   
2. **During Activity** (every second):
   - 29-byte record written containing all real-time metrics
   - Records are batched and page-aligned for efficient flash writes
   
3. **Session End**:
   - Footer (20 bytes) written with session summary and final battery state
   - File is closed and notification sent to Bluetooth module

---

## Version History


---

## Notes

1. **Endianness**: All multi-byte values are stored in little-endian format
2. **Alignment**: Structures are packed with no padding (`__attribute__((packed))`)
3. **String Fields**: The `fw_version` field is null-terminated and padded to 16 bytes
4. **CRC Field**: The `file_crc` in the footer is currently not calculated (always 0)
5. **Battery Status**: Battery percentage of -1 indicates unknown/invalid reading
6. **Missing Data**: When only one foot sensor is active, the alerts field will have bit 5 set
7. **Overflow Protection**: Packet counter resets to 0 when approaching UINT32_MAX


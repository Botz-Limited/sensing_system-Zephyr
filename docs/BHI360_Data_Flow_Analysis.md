# BHI360 Data Flow Analysis

## Current Data Pipeline

### 1. Data Collection (motion_sensor.cpp)
The BHI360 collects and processes the following sensor data:
- **Quaternion** (QUAT_SENSOR_ID): x, y, z, w, accuracy
- **Linear Acceleration** (LACC_SENSOR_ID): x, y, z (gravity removed)
- **Gyroscope** (GYRO_SENSOR_ID): x, y, z (angular velocity)
- **Step Counter** (STEP_COUNTER_SENSOR_ID): step count

### 2. Data Structures

#### For Logging (data_msgq):
```cpp
bhi360_log_record_t {
    float quat_x, quat_y, quat_z, quat_w, quat_accuracy;
    float lacc_x, lacc_y, lacc_z;
    float gyro_x, gyro_y, gyro_z;
    uint32_t step_count;
    uint64_t timestamp;
}
```

#### For Bluetooth (bluetooth_msgq):
```cpp
// MSG_TYPE_BHI360_3D_MAPPING
bhi360_3d_mapping_t {
    float accel_x, accel_y, accel_z;  // Actually quaternion x,y,z
    float gyro_x, gyro_y, gyro_z;     // Gyroscope data
    float quat_w;                     // Quaternion W
}

// MSG_TYPE_BHI360_LINEAR_ACCEL
bhi360_linear_accel_t {
    float x, y, z;  // Linear acceleration
}

// MSG_TYPE_BHI360_STEP_COUNT
bhi360_step_count_t {
    uint32_t step_count;
    uint32_t activity_duration_s;
}
```

### 3. Data Flow

```
BHI360 Hardware
    ↓ (Interrupt)
parse_all_sensors() callback
    ↓ (Accumulates data)
When all motion sensors updated:
    ├─→ data_msgq (MSG_TYPE_BHI360_LOG_RECORD)
    │     ↓
    │   data.cpp → Protobuf → External Flash
    │
    └─→ bluetooth_msgq (3 separate messages)
          ├─→ MSG_TYPE_BHI360_3D_MAPPING
          ├─→ MSG_TYPE_BHI360_LINEAR_ACCEL
          └─→ MSG_TYPE_BHI360_STEP_COUNT
                ↓
              Bluetooth notifications
```

### 4. Critical Data Transformations

1. **Quaternion Scaling**: Raw data divided by 16384.0f
2. **Linear Acceleration**: Raw data divided by 100.0f (m/s²)
3. **Gyroscope**: Raw data divided by 16384.0f (rad/s)
4. **Timestamp**: Raw timestamp * 15625 (to nanoseconds)

### 5. Protobuf Encoding (in data.cpp)

The data module converts float values to fixed-point for storage:
- Quaternion: scaled by 10000 (FixedPoint::QUAT_SCALE)
- Linear Acceleration: scaled by 1000 (FixedPoint::ACCEL_SCALE)
- Gyroscope: scaled by 10000 (FixedPoint::GYRO_SCALE)
- Quaternion accuracy: scaled by 100 (FixedPoint::ACCURACY_SCALE)

## Requirements for Driver Integration

### Must Maintain:
1. **Exact same data values** after scaling
2. **Same message structures** for data and bluetooth queues
3. **Same timing** for data synchronization
4. **Same protobuf encoding** in data module
5. **Same bluetooth notification format**

### Cannot Change:
1. Message queue formats
2. Data structure definitions
3. Scaling factors
4. Protobuf schemas
5. Bluetooth characteristics

## Implementation Strategy

### Phase 1: Minimal Changes
- Keep all data processing in motion_sensor.cpp
- Only change device initialization and interrupt handling
- Verify data output matches exactly

### Phase 2: Code Cleanup
- Remove unused SPI functions
- Remove manual GPIO handling
- Keep all data processing logic intact

### Phase 3: Future Enhancement
- Consider moving data processing to driver
- But maintain exact same output format
- Add compatibility layer if needed
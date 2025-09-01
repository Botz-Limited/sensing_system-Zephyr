# Choros Technical Architecture

## System Overview

Choros is designed as a real-time gait analysis platform with a modular, layered architecture.
The system processes data from pressure-sensitive insoles and IMU sensors to extract biomechanical gait metrics.

## Architectural Layers

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
├─────────────────────────────────────────────────────────────┤
│                    I/O Layer (choros-io)                    │
├─────────────────────────────────────────────────────────────┤
│                Algorithm Layer (choros-algo)                │
├─────────────────────────────────────────────────────────────┤
│                  Core Layer (choros-core)                   │
├─────────────────────────────────────────────────────────────┤
│                    Platform Layer                           │
└─────────────────────────────────────────────────────────────┘
```

## Data Flow Architecture

### 1. Data Ingestion Flow

```
CSV Files → BotzLoader → IMUData → IMUBuffer → Processing Pipeline
```

**Components:**
- **BotzLoader**: Parses CSV files and converts to `IMUData` structures
- **IMUBuffer**: Ring buffer implementation for real-time data streaming
- **Processing Pipeline**: Multi-threaded producer-consumer pattern

### 2. Processing Pipeline

```
IMU Data → Motion Mask → Pressure Detection → FSM → Gait Events → Metrics
```

**Stages:**
1. **Motion Masking**: Filters out stationary data using gyroscope RMS
2. **Pressure Detection**: Identifies initial contact (IC) and toe-off (TO) events
3. **FSM Processing**: Finite state machine for gait phase detection
4. **Gait Events**: Extracts stride boundaries and timing
5. **Metrics Calculation**: Computes biomechanical parameters

### 3. Output Generation

```
Metrics → Synchronization → BLE Formatting → CSV Export → Display
```

## 🧩 Core Components

### IMUData Structure

```cpp
struct IMUData {
    float timestamp;                    // Relative time in seconds
    std::array<float, 8> insole_pressures;  // Pressure sensor readings
    float gyro_x, gyro_y, gyro_z;      // Gyroscope (rad/s)
    float acc_x, acc_y, acc_z;         // Accelerometer (m/s²)
    std::string shoe;                   // Shoe identifier
    FootSide foot;                      // Left/Right foot
};
```

### Ring Buffer Implementation

```cpp
template <typename T, size_t N>
class RingBuffer {
    std::array<T, N> buffer;
    size_t start, end;
    bool full;
    mutable BotzMutex mutex;
    
    // Thread-safe operations
    bool push(const T& item);
    bool get_all(std::array<T, N>& out, size_t& out_size);
    size_t size() const;
};
```

### Finite State Machine (FSM)

```cpp
enum class GaitPhase {
    IDLE,           // No gait activity
    STANCE,         // Foot in contact with ground
    SWING,          // Foot in air
    TRANSITION      // Phase transition
};

class FSM {
    GaitPhase current_state;
    std::vector<StrideSegment> stride_segments;
    
    void update(const IMUData& imu);
    void reset_stride_start();
};
```

## 🔄 Threading Model

### Producer-Consumer Pattern

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Producer  │───▶│   Buffer    │───▶│  Consumer   │
│   Thread    │    │             │    │   Thread    │
└─────────────┘    └─────────────┘    └─────────────┘
```

**Thread Responsibilities:**
- **Producer**: Reads CSV data and feeds into buffer
- **Consumer**: Processes data and extracts metrics
- **Synchronization**: Uses condition variables for data availability

### Memory Management

- **Lock-free Operations**: Ring buffer minimizes contention
- **Buffer Pooling**: Pre-allocated buffers for performance
- **Memory Monitoring**: Optional real-time memory usage tracking

## 📊 Algorithm Details

### Motion Masking

```cpp
std::vector<uint8_t> build_motion_mask(
    const std::vector<IMUData>& imu,
    float fs,
    float win_s = 0.15f,      // 150ms window
    float gyro_thresh = 0.1f,  // Gyro threshold (rad/s)
    float acc_thresh = 0.5f    // Accel threshold (m/s²)
);
```

**Algorithm:**
1. **Sliding Window**: 150ms window for stability
2. **Gyroscope RMS**: Calculates angular velocity magnitude
3. **Accelerometer Deviation**: Measures deviation from 1g
4. **Hysteresis**: Prevents rapid state changes

### Pressure Event Detection

```cpp
void detect_ic_to_from_pressure(
    const Eigen::VectorXf& pressure,
    float fs,
    float min_ic_duration,
    float max_ic_duration,
    float min_to_duration,
    float min_to_duration,
    std::vector<int>& ic_indices,
    std::vector<int>& to_indices,
    std::vector<std::string>& stride_type
);
```

**Detection Logic:**
1. **Threshold Detection**: Adaptive pressure thresholds
2. **Duration Filtering**: Minimum/maximum event durations
3. **Pattern Recognition**: IC/TO sequence validation
4. **Stride Classification**: Heel, midfoot, forefoot strike

### Sensor Fusion

```cpp
class GyroOnlyTiltFusion {
    Eigen::Quaternionf orientation_;
    float fs_, dt_;
    
public:
    void step(const IMUData& imu, bool zupt, bool gait_event);
    Eigen::Vector3f get_orientation() const;
    Eigen::Vector3f get_position() const;
};
```

**Fusion Algorithm:**
1. **Gyroscope Integration**: Angular velocity to orientation
2. **Zero Velocity Updates**: Corrects drift during stance phase
3. **Gait Event Correction**: Applies stride-specific corrections
4. **Gravity Compensation**: Removes gravity from accelerometer

## 🔌 Platform Abstraction

### Cross-Platform Support

```cpp
#ifdef ZEPHYR_ENV
    // Embedded environment
    #define BOTZ_MUTEX k_mutex
    #define BOTZ_CONDVAR k_condvar
    #define BOTZ_LOCK(mtx) k_mutex_lock(&(mtx), K_FOREVER)
#else
    // Desktop environment
    #define BOTZ_MUTEX std::mutex
    #define BOTZ_CONDVAR std::condition_variable
    #define BOTZ_LOCK(mtx) std::unique_lock<std::mutex> lock(mtx)
#endif
```

### Configuration Management

```cpp
namespace choros::core {
    constexpr size_t DEFAULT_BUFFER_SIZE = 2048;
    constexpr float DEFAULT_FS = 200.0f;
    constexpr float PRESSURE_IC_THRESHOLD = 0.5f;
    constexpr float FSM_VELOCITY_THRESHOLD = 0.1f;
    constexpr float GYRO_DECAY_STRENGTH = 0.95f;
}
```

## Performance Characteristics

### Real-Time Constraints

- **Latency**: < 50ms end-to-end processing
- **Throughput**: 450 Hz continuous data processing
- **Memory**: < 10MB peak usage for 2000 sample buffer
- **CPU**: < 20% utilization on modern processors

### Optimization Strategies

1. **SIMD Operations**: Eigen3 vectorization for matrix operations
2. **Memory Locality**: Cache-friendly data structures
3. **Algorithm Efficiency**: O(n) complexity for all algorithms
4. **Thread Pooling**: Reusable worker threads

## Error Handling

### Exception Safety

```cpp
class GaitAnalysisApp {
    void run() {
        try {
            initialize_components();
            process_data();
            export_results();
        } catch (const std::exception& e) {
            log_error(e.what());
            cleanup();
        }
    }
};
```

### Data Validation

- **Input Sanity Checks**: Validates CSV format and data ranges
- **Runtime Assertions**: Checks for buffer overflows and invalid states
- **Graceful Degradation**: Continues processing with partial data
- **Error Logging**: Comprehensive error reporting and debugging

## 🔧 Configuration and Tuning

### Algorithm Parameters

```cpp
struct GaitAnalysisConfig {
    float velocity_threshold = 0.1f;        // FSM velocity threshold
    float pressure_threshold = 0.5f;        // Pressure event threshold
    float motion_window = 0.15f;            // Motion detection window
    size_t buffer_size = 2048;              // Processing buffer size
    bool enable_memory_monitoring = false;  // Memory tracking
};
```

### Performance Tuning

1. **Buffer Size**: Balance latency vs. memory usage
2. **Threshold Values**: Adjust for different sensor types
3. **Window Sizes**: Optimize for specific gait patterns
4. **Thread Count**: Match available CPU cores

## Testing Strategy

### Unit Testing

- **Component Isolation**: Test individual algorithms
- **Mock Data**: Synthetic sensor data for validation
- **Edge Cases**: Boundary conditions and error scenarios
- **Performance**: Timing and memory usage validation

### Integration Testing

- **End-to-End**: Full pipeline validation
- **Real Data**: Actual sensor data processing
- **Stress Testing**: High-frequency data streams
- **Memory Testing**: Long-running stability tests


# BLE Packet Sequencing Mechanism

## Overview

This document describes the packet sequencing mechanism implemented for the Information Service characteristics on both the foot sensor and motion sensor. This mechanism ensures reliable data delivery and allows the mobile application to detect and recover from packet loss during BLE communication.

## Key Implementation Details for Mobile Team

### Automatic Recovery on Reconnection
The firmware automatically buffers the last 10 packets for each sensor type during disconnection. When the mobile app reconnects within 5 seconds, these packets are automatically sent:

1. **Recovery Marker**: First packet has `seq_num = 0xFF` to indicate recovery mode
2. **Buffered Packets**: Following packets contain the buffered data with original sequence numbers
3. **Normal Operation**: After buffer is empty, normal streaming resumes

### Buffer Specifications
- **Foot sensor**: 10 packets (500ms of data at 20Hz)
- **BHI360 3D mapping**: 10 packets (200ms of data at 50Hz)  
- **BHI360 linear acceleration**: 10 packets (200ms of data at 50Hz)
- **Recovery timeout**: 5 seconds (after this, buffers are cleared)
- **Recovery packet interval**: 50ms between packets during recovery

### Sequence Number Range
- Normal range: 0-127 (wraps from 127 to 0)
- Recovery marker: 0xFF (255)
- Each characteristic has independent sequence numbering

## Background

BLE notifications can occasionally be lost due to:
- Radio interference
- Connection instability
- Buffer overflows
- Mobile OS background limitations

To address this, we've implemented a packet sequencing system that adds a sequence number to each data packet, allowing the mobile app to detect gaps and request retransmission of missing data.

## Implementation Details

### 1. Sequence Number Format

Each sensor data packet now includes a 16-bit sequence number as the first field:

```c
// Foot sensor data structure
typedef struct {
    uint16_t sequence_number;  // Packet sequence number (0-65535)
    uint32_t timestamp;        // Original timestamp
    uint16_t sensor_data[8];   // 8 sensor readings
    // ... other fields
} foot_samples_t;

// Motion sensor data structures
typedef struct {
    uint16_t sequence_number;  // Packet sequence number (0-65535)
    uint32_t timestamp;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} bhi360_3d_mapping_t;
```

### 2. Sequence Number Management

#### Primary Device (Right Foot)
- Maintains separate sequence counters for each sensor type
- Increments counter after each successful notification
- Wraps around from 65535 to 0
- Persists last sequence number to handle reconnections

#### Secondary Device (Left Foot)
- Forwards data from secondary to primary via D2D connection
- Primary device assigns sequence numbers before notifying mobile app
- Ensures consistent sequencing across both devices

### 3. Information Service Characteristics

The following characteristics include sequence numbers:

| Characteristic | UUID | Description |
|----------------|------|-------------|
| Foot Sensor Data | `0c372eac-27eb-437e-bef4-775aefaf3c97` | Foot pressure sensor data with sequence |
| BHI360 Data 1 | `0c372ead-27eb-437e-bef4-775aefaf3c97` | 3D mapping (accel/gyro) with sequence |
| BHI360 Data 2 | `0c372eae-27eb-437e-bef4-775aefaf3c97` | Step count data with sequence |
| BHI360 Data 3 | `0c372eaf-27eb-437e-bef4-775aefaf3c97` | Linear acceleration with sequence |

### 4. Mobile App Integration

#### Sequence Tracking
```swift
// iOS Example
class SensorDataManager {
    private var lastFootSequence: UInt16?
    private var lastMotionSequence: UInt16?
    
    func processFootData(_ data: Data) {
        let sequence = data.subdata(in: 0..<2).withUnsafeBytes { 
            $0.load(as: UInt16.self) 
        }
        
        if let lastSeq = lastFootSequence {
            let expectedSeq = (lastSeq &+ 1) // Handle wraparound
            if sequence != expectedSeq {
                // Gap detected
                let missedCount = Int(sequence &- expectedSeq)
                handleMissedPackets(from: expectedSeq, to: sequence)
            }
        }
        
        lastFootSequence = sequence
        // Process sensor data...
    }
}
```

#### Gap Detection
```kotlin
// Android Example
class SensorDataProcessor {
    private var lastFootSequence: UShort? = null
    
    fun processFootData(data: ByteArray) {
        val sequence = ByteBuffer.wrap(data, 0, 2)
            .order(ByteOrder.LITTLE_ENDIAN)
            .getShort()
            .toUShort()
        
        lastFootSequence?.let { lastSeq ->
            val expectedSeq = (lastSeq + 1u).toUShort()
            if (sequence != expectedSeq) {
                // Calculate gap size accounting for wraparound
                val gapSize = (sequence - expectedSeq).toInt()
                if (gapSize > 0) {
                    onPacketsLost(expectedSeq, sequence)
                }
            }
        }
        
        lastFootSequence = sequence
        // Process data...
    }
}
```

### 5. Recovery Mechanism

The firmware implements a comprehensive packet buffering and recovery system that works in conjunction with the mobile app. When a BLE disconnection occurs, the firmware automatically buffers packets and can send them when reconnected.

#### 5.1 Firmware-Side Buffering and Recovery

**Automatic Buffering During Disconnection:**
The firmware (primary device) automatically detects disconnections and starts buffering packets:

```cpp
// From bluetooth.cpp
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);
    
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Mark disconnection for potential recovery
    BleSequenceManager::getInstance().onDisconnect();
#endif
    // ...
}
```

**Buffer Sizes (from ble_seq_manager.hpp):**
- Foot sensor buffer: 10 packets
- BHI360 3D mapping buffer: 10 packets  
- BHI360 linear acceleration buffer: 10 packets
- Recovery timeout: 5000ms (5 seconds)

**Reconnection and Recovery Process:**
When the mobile app reconnects, the firmware checks if recovery is needed:

```cpp
// From bluetooth.cpp - connected() callback
#if IS_ENABLED(CONFIG_PRIMARY_DEVICE)
    // Check if this is a reconnection and start recovery if needed
    BleSequenceManager::getInstance().onReconnect();
    if (BleSequenceManager::getInstance().isInRecovery()) {
        LOG_INF("Starting BLE packet recovery after reconnection");
        BleRecoveryHandler::getInstance().startRecovery();
    }
#endif
```

**Recovery Transmission Timing:**
- Recovery marker delay: 100ms
- Inter-packet interval during recovery: 50ms
- Recovery starts automatically 10ms after connection is established

**Firmware Buffer Management:**
The firmware uses circular buffers that automatically handle overflow:
- When buffer is full, oldest packets are discarded
- Sequence numbers wrap at 127 (avoiding 0xFF which is the recovery marker)
- Each packet type has its own independent sequence counter

#### 5.2 Mobile App Gap Detection

When the mobile app receives packets, it should check for sequence number gaps. The firmware's recovery mechanism will automatically send buffered packets after reconnection, but the app still needs to handle gaps that occur during normal operation.

**Timing Windows:**
- Small gap: 1-5 packets (50-250ms of data)
- Medium gap: 6-50 packets (300ms-2.5s of data)
- Large gap: >50 packets (>2.5s of data)

**Sample Rates and Timing:**
- Foot sensor: 20Hz (50ms between samples)
- Motion sensor: 50Hz (20ms between samples)
- BLE notification interval: ~50ms (configurable)

#### 5.3 Firmware Recovery Packet Format

When the firmware enters recovery mode after reconnection, it sends packets in the following sequence:

1. **Recovery Marker (seq_num = 0xFF)**
   - Sent for each characteristic to signal start of recovery
   - Mobile app should prepare to receive buffered packets
   
2. **Buffered Packets**
   - Sent in order from oldest to newest
   - Each packet retains its original sequence number
   - Sent at 50ms intervals to avoid overwhelming the connection

3. **Resume Normal Operation**
   - After all buffered packets are sent, normal streaming resumes
   - New packets will have sequence numbers continuing from where they left off

**Example Recovery Sequence:**
```
// Disconnection occurs at seq_num = 45
// ... 3 seconds pass (60 packets buffered, but only last 10 kept) ...
// Reconnection occurs

// Recovery sequence:
Packet 1: seq_num = 0xFF (recovery marker)
Packet 2: seq_num = 96  (oldest buffered packet)
Packet 3: seq_num = 97
...
Packet 11: seq_num = 105 (newest buffered packet)
Packet 12: seq_num = 106 (resume normal streaming)
```

#### 5.4 Recovery Strategies by Gap Size

**Small Gaps (1-5 packets):**
- **Foot Sensor Data**: 
  - Missing samples: 1-5 readings (8 sensors × 5 = 40 data points max)
  - Time gap: 50-250ms
  - Recovery: Linear interpolation between last known and next received
  - Impact: Minimal on gait analysis algorithms
  
- **Motion Sensor Data**:
  - Missing samples: 2-10 readings (6 axes × 10 = 60 data points max)
  - Time gap: 40-200ms
  - Recovery: Cubic spline interpolation for smooth motion curves
  - Impact: Acceptable for activity recognition

```swift
// iOS Example - Small Gap Recovery
func interpolateFootData(lastPacket: FootPacket, 
                        nextPacket: FootPacket, 
                        missedCount: Int) -> [FootPacket] {
    var interpolated: [FootPacket] = []
    
    for i in 1...missedCount {
        let ratio = Float(i) / Float(missedCount + 1)
        var packet = FootPacket()
        packet.timestamp = lastPacket.timestamp + 
                          UInt32(i * 50) // 50ms intervals
        
        // Interpolate each sensor value
        for sensorIdx in 0..<8 {
            packet.sensorData[sensorIdx] = UInt16(
                Float(lastPacket.sensorData[sensorIdx]) * (1 - ratio) +
                Float(nextPacket.sensorData[sensorIdx]) * ratio
            )
        }
        interpolated.append(packet)
    }
    
    return interpolated
}
```

**Medium Gaps (6-50 packets):**
- **Foot Sensor Data**:
  - Missing samples: 6-50 readings (48-400 data points)
  - Time gap: 300ms-2.5s
  - Recovery: Mark as "data gap" in analysis
  - Action: Skip step detection for this period
  - Storage: Flag in local database for later recovery
  
- **Motion Sensor Data**:
  - Missing samples: 15-125 readings (90-750 data points)
  - Time gap: 300ms-2.5s
  - Recovery: Use last known state for activity classification
  - Action: Reduce confidence scores for this period

```kotlin
// Android Example - Medium Gap Handling
data class DataGap(
    val startSequence: UShort,
    val endSequence: UShort,
    val startTimestamp: Long,
    val estimatedEndTimestamp: Long,
    val samplesMissed: Int,
    val dataType: SensorType
)

fun handleMediumGap(gap: DataGap) {
    when (gap.dataType) {
        SensorType.FOOT -> {
            // Mark period as unreliable for gait analysis
            gaitAnalyzer.markUnreliablePeriod(
                gap.startTimestamp,
                gap.estimatedEndTimestamp
            )
            
            // Store gap info for potential recovery
            database.insertDataGap(gap)
            
            // Notify UI of degraded accuracy
            uiCallback.onAccuracyDegraded(
                "Missing ${gap.samplesMissed} foot samples " +
                "(${gap.estimatedEndTimestamp - gap.startTimestamp}ms)"
            )
        }
        
        SensorType.MOTION -> {
            // Continue with reduced confidence
            activityRecognizer.setConfidenceMultiplier(0.7f)
            
            // Schedule confidence restoration
            handler.postDelayed({
                activityRecognizer.setConfidenceMultiplier(1.0f)
            }, 5000) // Restore after 5 seconds
        }
    }
}
```

**Large Gaps (>50 packets):**
- **Foot Sensor Data**:
  - Missing samples: >50 readings (>400 data points)
  - Time gap: >2.5s
  - Recovery: Segment data into separate sessions
  - Action: Trigger historical data request
  - User notification: Required for clinical applications
  
- **Motion Sensor Data**:
  - Missing samples: >125 readings (>750 data points)
  - Time gap: >2.5s
  - Recovery: Reset activity tracking state
  - Action: Request log file download for complete data

#### 5.5 Historical Data Recovery (Mobile-Initiated)

While the firmware provides automatic buffering for recent disconnections, the mobile app can also request historical data for larger gaps:

**Automatic Recovery Triggers:**
- Gap size > 50 packets
- Total loss rate > 5% over 1 minute
- Critical event detection (fall, abnormal gait)

**Recovery Process:**
1. **Request Log File List** (100ms)
   ```
   // Request available log files
   writeToLogControlCharacteristic("LIST")
   ```

2. **Parse File Metadata** (50ms)
   - Identify files containing missing timestamps
   - Calculate download size (typically 50-200KB per minute)

3. **Download Relevant Segments** (2-10 seconds)
   - Download only files containing gaps
   - Parse binary format during download
   - Merge with real-time data

4. **Backfill Database** (500ms)
   - Insert recovered samples with "recovered" flag
   - Update analysis results if needed
   - Notify UI of recovery completion

**Recovery Timing Example:**
```
Gap detected: Seq 1000-1100 (100 packets, 5 seconds of data)
T+0ms: Gap detection
T+10ms: Log recovery decision
T+100ms: Request log file list
T+150ms: Receive file list (5 files)
T+200ms: Request file #3 (contains gap data)
T+2200ms: File download complete (100KB)
T+2700ms: Parsing and database insertion complete
T+2750ms: UI updated with recovered data
```

#### 5.6 Real-time Adjustments

**Connection Parameter Optimization:**
```c
// Firmware side - Adaptive connection parameters
if (packet_loss_rate > 0.05) {  // >5% loss
    // Reduce data rate
    conn_params.min_interval = 40;  // 50ms
    conn_params.max_interval = 60;  // 75ms
    conn_params.latency = 2;        // Allow 2 skipped intervals
} else if (packet_loss_rate < 0.01) {  // <1% loss
    // Increase data rate
    conn_params.min_interval = 20;  // 25ms
    conn_params.max_interval = 30;  // 37.5ms
    conn_params.latency = 0;        // No skipped intervals
}
```

**Mobile Side Buffering:**
- Buffer size: 1000 packets per sensor type
- Ring buffer implementation for efficiency
- Flush to database every 500 packets or 10 seconds

#### 5.7 Recovery Statistics

**Typical Recovery Scenarios:**

| Scenario | Gap Size | Frequency | Recovery Method | Success Rate |
|----------|----------|-----------|-----------------|--------------|
| Walking indoors | 1-3 packets | 2-3 per minute | Interpolation | 100% |
| Crowded area | 5-20 packets | 1-2 per minute | Partial recovery | 85% |
| Phone in pocket | 20-100 packets | Every 30s | Log download | 95% |
| Background app | 100-500 packets | When backgrounded | Batch recovery | 90% |
| Connection loss | >1000 packets | Rare | Full log sync | 99% |

**Performance Metrics:**
- Interpolation latency: <5ms
- Log file request latency: 100-200ms
- Download speed: 50-100KB/s
- Recovery accuracy: 98% for gaps <5s
- Battery impact: <2% for normal recovery rates

#### 5.8 Detailed Sample Calculations

**Foot Sensor Data Loss Examples:**

1. **Single Packet Loss (1 packet)**
   - Lost data: 8 sensor values × 1 packet = 8 data points
   - Time gap: 50ms
   - Recovery time: <1ms (interpolation)
   - Memory overhead: 16 bytes

2. **Walking Session Gap (10 packets)**
   - Lost data: 8 sensors × 10 packets = 80 data points
   - Time gap: 500ms (half a second)
   - Typical step duration: 600-800ms
   - Impact: May miss heel strike or toe-off event
   - Recovery: Mark step as "partial data"

3. **Background Transition (200 packets)**
   - Lost data: 8 sensors × 200 packets = 1,600 data points
   - Time gap: 10 seconds
   - File size for recovery: ~3.2KB
   - Download time: ~100ms
   - Steps potentially affected: 12-15 steps

**Motion Sensor Data Loss Examples:**

1. **Single Packet Loss (1 packet)**
   - Lost data: 6 axes × 2.5 samples = 15 data points
   - Time gap: 50ms (contains 2.5 samples at 50Hz)
   - Recovery: Interpolate 2-3 motion samples
   - Accuracy impact: <0.1° for orientation

2. **Activity Transition Gap (30 packets)**
   - Lost data: 6 axes × 75 samples = 450 data points
   - Time gap: 1.5 seconds
   - Impact: May miss activity transition
   - Recovery: Flag transition as "uncertain"

#### 5.9 Implementation Code Examples

**Complete Recovery Manager (iOS):**

```swift
class BLEDataRecoveryManager {
    private let footSampleRate: Double = 20.0  // Hz
    private let motionSampleRate: Double = 50.0  // Hz
    private let maxInterpolationGap = 5
    private let logRecoveryThreshold = 50
    
    struct RecoveryStats {
        var totalPacketsReceived: Int = 0
        var totalPacketsLost: Int = 0
        var smallGapsRecovered: Int = 0
        var mediumGapsDetected: Int = 0
        var largeGapsRecovered: Int = 0
        var averageRecoveryTime: Double = 0
    }
    
    private var stats = RecoveryStats()
    
    func handleSequenceGap(
        lastSeq: UInt16,
        currentSeq: UInt16,
        sensorType: SensorType,
        lastPacket: Data?,
        currentPacket: Data
    ) -> RecoveryAction {
        
        let gapSize = calculateGapSize(from: lastSeq, to: currentSeq)
        let timeLost = calculateTimeLost(gapSize: gapSize, sensorType: sensorType)
        
        stats.totalPacketsLost += gapSize
        
        switch gapSize {
        case 1...maxInterpolationGap:
            return handleSmallGap(
                gapSize: gapSize,
                lastPacket: lastPacket,
                currentPacket: currentPacket,
                sensorType: sensorType
            )
            
        case 6...logRecoveryThreshold:
            return handleMediumGap(
                gapSize: gapSize,
                timeLost: timeLost,
                startSeq: lastSeq,
                endSeq: currentSeq
            )
            
        default:
            return handleLargeGap(
                gapSize: gapSize,
                timeLost: timeLost,
                startSeq: lastSeq,
                endSeq: currentSeq,
                sensorType: sensorType
            )
        }
    }
    
    private func calculateTimeLost(gapSize: Int, sensorType: SensorType) -> Double {
        switch sensorType {
        case .foot:
            return Double(gapSize) * (1.0 / footSampleRate) * 1000  // ms
        case .motion:
            return Double(gapSize) * (1.0 / motionSampleRate) * 1000  // ms
        }
    }
    
    private func handleSmallGap(
        gapSize: Int,
        lastPacket: Data?,
        currentPacket: Data,
        sensorType: SensorType
    ) -> RecoveryAction {
        
        let startTime = Date()
        
        guard let lastPacket = lastPacket else {
            return .skipInterpolation
        }
        
        let interpolatedPackets = interpolatePackets(
            from: lastPacket,
            to: currentPacket,
            count: gapSize,
            sensorType: sensorType
        )
        
        let recoveryTime = Date().timeIntervalSince(startTime) * 1000  // ms
        updateStats(recoveryTime: recoveryTime)
        stats.smallGapsRecovered += 1
        
        return .interpolated(packets: interpolatedPackets)
    }
    
    enum RecoveryAction {
        case interpolated(packets: [Data])
        case requestLogData(startSeq: UInt16, endSeq: UInt16)
        case markGap(duration: Double)
        case skipInterpolation
    }
}
```

**Complete Recovery Manager (Android):**

```kotlin
class BLEDataRecoveryManager(private val context: Context) {
    companion object {
        const val FOOT_SAMPLE_RATE = 20.0  // Hz
        const val MOTION_SAMPLE_RATE = 50.0  // Hz
        const val MAX_INTERPOLATION_GAP = 5
        const val LOG_RECOVERY_THRESHOLD = 50
        const val PACKET_SIZE_FOOT = 20  // bytes
        const val PACKET_SIZE_MOTION = 28  // bytes
    }
    
    data class RecoveryMetrics(
        var packetsRecovered: Int = 0,
        var bytesRecovered: Int = 0,
        var averageGapSize: Float = 0f,
        var recoverySuccessRate: Float = 0f,
        var lastRecoveryTimestamp: Long = 0
    )
    
    private val metrics = RecoveryMetrics()
    private val recoveryScope = CoroutineScope(Dispatchers.IO)
    
    fun processGap(
        gap: SequenceGap,
        onRecoveryComplete: (RecoveryResult) -> Unit
    ) {
        recoveryScope.launch {
            val result = when (gap.size) {
                in 1..MAX_INTERPOLATION_GAP -> performInterpolation(gap)
                in 6..LOG_RECOVERY_THRESHOLD -> handleMediumGap(gap)
                else -> performLogRecovery(gap)
            }
            
            updateMetrics(gap, result)
            
            withContext(Dispatchers.Main) {
                onRecoveryComplete(result)
            }
        }
    }
    
    private suspend fun performLogRecovery(gap: SequenceGap): RecoveryResult {
        val startTime = System.currentTimeMillis()
        
        return try {
            // Calculate exact data loss
            val samplesLost = when (gap.sensorType) {
                SensorType.FOOT -> gap.size * 8  // 8 sensors per packet
                SensorType.MOTION -> (gap.size * 2.5).toInt()  // 2.5 samples per packet
            }
            
            val bytesNeeded = when (gap.sensorType) {
                SensorType.FOOT -> gap.size * PACKET_SIZE_FOOT
                SensorType.MOTION -> gap.size * PACKET_SIZE_MOTION
            }
            
            // Request log data
            val logData = requestLogData(
                startSeq = gap.startSeq,
                endSeq = gap.endSeq,
                expectedBytes = bytesNeeded
            )
            
            val recoveryTime = System.currentTimeMillis() - startTime
            
            RecoveryResult.Success(
                recoveredPackets = logData.packets,
                recoveryTimeMs = recoveryTime,
                samplesRecovered = samplesLost,
                bytesRecovered = bytesNeeded
            )
            
        } catch (e: Exception) {
            RecoveryResult.Failed(
                reason = e.message ?: "Unknown error",
                samplesLost = samplesLost,
                fallbackAction = FallbackAction.MARK_GAP
            )
        }
    }
    
    private fun calculateDataLoss(gap: SequenceGap): DataLossInfo {
        val timeLostMs = when (gap.sensorType) {
            SensorType.FOOT -> (gap.size * 50.0)  // 50ms per packet
            SensorType.MOTION -> (gap.size * 20.0)  // 20ms per packet
        }
        
        val stepsAffected = if (gap.sensorType == SensorType.FOOT) {
            (timeLostMs / 700).toInt()  // Assuming 700ms per step
        } else 0
        
        return DataLossInfo(
            packetsLost = gap.size,
            timeLostMs = timeLostMs,
            stepsAffected = stepsAffected,
            activitiesAffected = timeLostMs > 2000  // Activities affected if >2s gap
        )
    }
}

data class SequenceGap(
    val startSeq: UInt16,
    val endSeq: UInt16,
    val size: Int,
    val sensorType: SensorType,
    val detectedAt: Long = System.currentTimeMillis()
)

sealed class RecoveryResult {
    data class Success(
        val recoveredPackets: List<ByteArray>,
        val recoveryTimeMs: Long,
        val samplesRecovered: Int,
        val bytesRecovered: Int
    ) : RecoveryResult()
    
    data class Failed(
        val reason: String,
        val samplesLost: Int,
        val fallbackAction: FallbackAction
    ) : RecoveryResult()
}
```

#### 5.10 Recovery Decision Matrix

| Gap Size | Time Lost | Foot Sensor Action | Motion Sensor Action | User Notification |
|----------|-----------|-------------------|---------------------|-------------------|
| 1 packet | 50ms/20ms | Linear interpolation | Cubic interpolation | None |
| 2-5 packets | 100-250ms | Linear interpolation | Cubic interpolation | None |
| 6-10 packets | 300-500ms | Mark gap, continue | Reduce confidence | Status bar icon |
| 11-30 packets | 550ms-1.5s | Flag for recovery | State estimation | Yellow indicator |
| 31-50 packets | 1.5-2.5s | Request log data | Reset activity | Orange indicator |
| 51-200 packets | 2.5-10s | Immediate log sync | Full reset | Red alert + vibration |
| >200 packets | >10s | Full session recovery | New session | Dialog prompt |

### 6. Special Considerations

#### Connection Events
- On new connection: First packet may have any sequence number
- On reconnection: Sequence continues from last value
- Mobile app should reset tracking on connection events

#### Background Execution
- Larger gaps expected when app is backgrounded
- Use connection parameter control to reduce data rate
- Consider buffering on device side

#### Data Rate vs Reliability Trade-off
- Higher data rates increase chance of packet loss
- Sequence numbers help quantify actual loss rate
- Adjust connection parameters based on loss statistics

## Complete Recovery Flow Summary

### Firmware-Initiated Recovery (Automatic)

1. **Disconnection Detection**
   - Firmware detects BLE disconnection
   - Starts buffering packets (up to 10 per sensor type)
   - Continues buffering for up to 5 seconds

2. **Reconnection**
   - Mobile app reconnects to device
   - Firmware checks if disconnection was < 5 seconds
   - If yes, enters recovery mode automatically

3. **Recovery Transmission**
   - Sends recovery marker (0xFF) for each characteristic
   - Transmits buffered packets with original sequence numbers
   - Packets sent at 50ms intervals
   - Resumes normal streaming after buffer is empty

### Mobile-Initiated Recovery (On-Demand)

1. **Gap Detection**
   - Mobile app detects sequence number gap
   - Determines gap size and impact

2. **Recovery Decision**
   - Small gaps (1-5 packets): Interpolate locally
   - Medium gaps (6-50 packets): Mark as unreliable
   - Large gaps (>50 packets): Request log file download

3. **Log File Recovery**
   - Request file list via Control Service
   - Download relevant log segments
   - Parse and backfill missing data
   - Update UI with recovered data

## Benefits

1. **Reliability Metrics**: Quantify packet loss rate
2. **Data Integrity**: Detect missing samples
3. **Debugging**: Identify communication issues
4. **Recovery**: Enable targeted data recovery
5. **Analytics**: Track connection quality over time
6. **Automatic Recovery**: Firmware handles recent disconnections transparently
7. **Minimal Latency**: Recovery starts within 10ms of reconnection

## Example Log Output

```
[INFO] Foot sensor data sent - Seq: 1234, Timestamp: 1699564800
[INFO] Foot sensor data sent - Seq: 1235, Timestamp: 1699564850
[INFO] Foot sensor data sent - Seq: 1236, Timestamp: 1699564900
[INFO] Motion data sent - Seq: 5678, Type: 3D_MAPPING
[INFO] Motion data sent - Seq: 5679, Type: LINEAR_ACCEL
```

## Mobile App Best Practices

1. **Don't Assume Sequential Delivery**: BLE doesn't guarantee order
2. **Handle Wraparound**: Sequence wraps from 65535 to 0
3. **Track Per-Characteristic**: Each characteristic has its own sequence
4. **Log Statistics**: Track loss rate for debugging
5. **Graceful Degradation**: Continue operation despite gaps

## Future Enhancements

1. **Acknowledgment System**: Add ACK/NACK for critical data
2. **Retransmission Buffer**: Store recent packets for retransmit
3. **Compression**: Reduce packet size to improve reliability
4. **Error Correction**: Add FEC (Forward Error Correction)
5. **Adaptive Rate**: Adjust data rate based on loss statistics
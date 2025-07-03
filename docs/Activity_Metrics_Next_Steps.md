# Activity Metrics Multi-Thread Architecture - Next Development Steps

**Date:** June 2025  
**Version:** 1.0  
**Status:** Development Roadmap

---

## Overview

This document outlines the remaining implementation steps for the multi-thread activity metrics architecture. The sensor modules have been updated to use the new `sensor_data_msgq` queue.

---

## Completed Tasks ✅

### 1. Update Sensor Modules to Use New Queue (COMPLETED)
- ✅ Modified `motion_sensor` to send to `sensor_data_msgq` instead of `activity_metrics_msgq`
- ✅ Modified `foot_sensor` to send to `sensor_data_msgq` instead of `activity_metrics_msgq`
- ✅ Both modules already check `logging_active` flag before sending data
- ✅ Data flow now ready for new architecture

---

## Remaining Implementation Tasks

### 2. Implement Basic sensor_data Processing (HIGH PRIORITY - 4 hours)

**Objective:** Add actual sensor data processing to the sensor_data module

**Tasks:**
1. **Implement Ground Contact Detection**
   ```c
   // Use fast algorithms from sensor_data_fast_processing.h
   - Detect contact/flight phases for each foot
   - Track contact duration
   - Identify heel strike, mid-stance, toe-off phases
   ```

2. **Calculate Peak Forces**
   ```c
   - Sum all 8 pressure sensors per foot
   - Track peak values during contact phase
   - Calculate force distribution (heel/mid/fore)
   ```

3. **Add Timestamp Management**
   ```c
   - Track delta time between packets
   - Ensure microsecond precision for 100Hz operation
   - Handle timestamp rollover
   ```

4. **Create Consolidated Messages**
   ```c
   // Define consolidated message structure
   typedef struct {
       uint8_t left_contact_phase;
       uint8_t right_contact_phase;
       uint16_t left_peak_force;
       uint16_t right_peak_force;
       int8_t left_pronation_hint;
       int8_t right_pronation_hint;
       uint32_t step_count;
       float quaternion[4];
       float linear_acc[3];
       float gyro[3];
   } sensor_data_consolidated_t;
   ```

5. **Send to Downstream Modules**
   ```c
   - Send consolidated data via sensor_data_queue
   - Ensure 100Hz timing is maintained
   ```

**Testing:**
- Verify 100Hz output rate
- Check processing time < 2ms
- Monitor ring buffer statistics

---

### 3. Implement Basic realtime_metrics Processing (HIGH PRIORITY - 4 hours)

**Objective:** Calculate real-time metrics for BLE updates

**Tasks:**
1. **Extract Cadence from Step Count**
   ```c
   - Track step count changes over time
   - Calculate steps per minute
   - Apply smoothing filter
   - Handle both feet independently
   ```

2. **Calculate Current Pace**
   ```c
   - If GPS available: distance / time
   - If no GPS: estimate from cadence + stride length
   - Convert to seconds per km
   - Apply moving average
   ```

3. **Calculate Form Score**
   ```c
   - Ground contact time ratio
   - Vertical oscillation (from IMU)
   - Left/right balance
   - Combine into 0-100 score
   ```

4. **Prepare BLE Messages**
   ```c
   // At 1Hz rate:
   - Pack metrics into BLE format
   - Send via bluetooth_msgq
   - Include timestamp
   ```

**Testing:**
- Verify 1Hz BLE update rate
- Check metric accuracy
- Test with/without GPS

---

### 4. Test Multi-Thread Communication (HIGH PRIORITY - 2 hours)

**Objective:** Verify the complete data flow works correctly

**Test Cases:**
1. **Message Flow Testing**
   - Start activity via BLE characteristic
   - Verify sensors → sensor_data → realtime_metrics flow
   - Check message types and content
   - Monitor queue depths

2. **Timing Verification**
   - Measure actual rates: 100Hz, 10Hz, 1Hz
   - Check for timing drift
   - Verify no message drops

3. **CPU and Memory Monitoring**
   ```c
   - Add CPU usage tracking
   - Monitor stack usage per thread
   - Check heap/memory pool usage
   - Log ring buffer statistics
   ```

4. **Error Handling**
   - Test queue overflow scenarios
   - Verify graceful degradation
   - Check error propagation

**Tools:**
- Use RTT logging for timing
- Add performance counters
- Create test commands

---

### 5. Refactor activity_metrics Module (MEDIUM PRIORITY - 8 hours)

**Objective:** Transform activity_metrics into pure session management

**Tasks:**
1. **Remove Real-time Processing**
   ```c
   - Delete step detection code (moved to sensor_data)
   - Remove direct sensor handling
   - Keep only session-level logic
   ```

2. **Implement Session Management**
   ```c
   - Track session start/stop times
   - Calculate session duration
   - Manage session ID generation
   - Handle pause/resume
   ```

3. **Create Periodic Records**
   ```c
   // Every 1-2 seconds:
   - Aggregate metrics from analytics module
   - Create PeriodicRecord structure
   - Calculate averages/totals
   - Send to data module for file writing
   ```

4. **Add Session Statistics**
   ```c
   - Total distance (from GPS)
   - Average pace/cadence
   - Total steps
   - Calories burned
   - Maximum/minimum values
   ```

5. **GPS Integration**
   ```c
   - Process GPS data for distance
   - Calculate elevation changes
   - Generate kilometer splits
   - Handle GPS signal loss
   ```

**Testing:**
- Verify file format compatibility
- Test session start/stop
- Check statistics accuracy

---

### 6. Implement Basic analytics Processing (MEDIUM PRIORITY - 8 hours)

**Objective:** Add complex biomechanical calculations

**Tasks:**
1. **Running Efficiency Calculation**
   ```c
   - Vertical oscillation ratio
   - Ground contact time percentage
   - Stride angle analysis
   - Energy return estimation
   ```

2. **Fatigue Index Tracking**
   ```c
   - Establish 2-minute baseline
   - Track metric degradation:
     * Contact time increase
     * Cadence decrease
     * Form score decline
   - Calculate fatigue percentage
   ```

3. **Basic Injury Risk Assessment**
   ```c
   - Asymmetry detection (>10% difference)
   - Impact force trends
   - Pronation excessive detection
   - Generate risk score (0-100)
   ```

4. **Pronation Analysis**
   ```c
   - Combine IMU roll angle + pressure distribution
   - Classify: under/normal/over pronation
   - Track changes during run
   ```

**Testing:**
- Validate against known data
- Check baseline establishment
- Verify calculations

---

### 7. Add Ring Buffers to Other Modules (LOW PRIORITY - 2 hours)

**Objective:** Add ring buffers only if testing shows drops

**Conditional Implementation:**
1. **Monitor During Testing**
   ```c
   - Check message drop statistics
   - Identify which modules need buffers
   - Size buffers appropriately
   ```

2. **Likely Candidates**
   ```c
   - realtime_metrics: Size 2-4 if needed
   - analytics: Probably not needed
   - activity_metrics: Probably not needed
   ```

---

## Implementation Guidelines

### Code Quality Standards
1. **Performance First**
   - Profile all code paths
   - Optimize inner loops
   - Minimize memory allocations

2. **Thread Safety**
   - Use atomic operations
   - No shared state between threads
   - Message passing only

3. **Error Handling**
   - Graceful degradation
   - Log but don't crash
   - Report errors via BLE

### Testing Strategy
1. **Unit Tests**
   - Test each algorithm independently
   - Use known input/output data
   - Verify edge cases

2. **Integration Tests**
   - Full system data flow
   - Timing under load
   - Memory leak detection

3. **Field Tests**
   - Real running scenarios
   - Battery life impact
   - BLE connection stability

---

## Success Metrics

### Performance Targets
- sensor_data: < 2ms processing @ 100Hz
- realtime_metrics: < 20ms @ 10Hz
- analytics: < 200ms @ 1Hz
- Total CPU: < 40%
- RAM usage: < 100KB

### Functional Goals
- Zero data loss during normal operation
- BLE updates within 100ms
- Accurate metrics (±5% of reference)
- 4+ hour battery life

---

## Risk Mitigation

### High Risks
1. **100Hz Timing Miss**
   - Mitigation: Pre-optimize sensor_data
   - Fallback: Reduce to 50Hz if needed

2. **Memory Overflow**
   - Mitigation: Fixed-size buffers
   - Fallback: Drop old data

### Medium Risks
1. **BLE Congestion**
   - Mitigation: Priority queue for BLE
   - Fallback: Reduce update rate

2. **Complex Algorithm Time**
   - Mitigation: Incremental calculation
   - Fallback: Simplified algorithms

---

## Development Schedule

### Week 1 (Immediate)
- Day 1-2: Implement sensor_data processing
- Day 3-4: Implement realtime_metrics
- Day 5: Test multi-thread communication

### Week 2
- Day 1-3: Refactor activity_metrics
- Day 4-5: Begin analytics implementation

### Week 3
- Day 1-3: Complete analytics
- Day 4-5: System integration testing

### Week 4
- Day 1-2: Performance optimization
- Day 3-4: Field testing
- Day 5: Documentation and cleanup

---

## Conclusion

The architecture is well-designed with clear separation of concerns. The critical path is implementing the 100Hz sensor_data processing efficiently. With careful implementation and testing, the system should meet all performance and functional requirements.
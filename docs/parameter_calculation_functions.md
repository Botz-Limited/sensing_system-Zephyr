# Parameter Calculation Functions - Comprehensive Documentation

This document provides a complete reference of all parameter calculation functions found in the sensor data processing modules. Functions are organized by module and include descriptions, parameters, return values, and source file locations.

## Table of Contents
1. [Activity Metrics Module](#activity-metrics-module)
2. [Analytics Module](#analytics-module)
3. [Realtime Metrics Module](#realtime-metrics-module)
4. [Sensor Data Module](#sensor-data-module)
5. [Advanced Metrics Summary](#advanced-metrics-summary)

---

## Activity Metrics Module

### Core Calculation Functions

#### **calculate_pace_from_cadence**
- **Location:** [`src/activity_metrics/activity_metrics.cpp:1143`](../src/activity_metrics/activity_metrics.cpp#L1143)
- **Purpose:** Calculates running pace based on cadence
- **Parameters:** 
  - `float cadence` - Current cadence in steps per minute
- **Returns:** `float` - Estimated pace in seconds per kilometer
- **Algorithm:** Estimates stride length based on height and cadence, then calculates speed and converts to pace
- **Data Requirements:** Motion sensor data (BHI360 for cadence)

#### **calculate_balance_score**
- **Location:** [`src/activity_metrics/activity_metrics.cpp:1167`](../src/activity_metrics/activity_metrics.cpp#L1167)
- **Purpose:** Calculates balance score between left and right foot contact times
- **Returns:** `int8_t` - Balance score as signed integer representing asymmetry percentage (-100 to +100)
- **Algorithm:** Compares left vs right contact times to determine asymmetry
- **Data Requirements:** Foot sensor data (both feet)

#### **calculate_form_score**
- **Location:** [`src/activity_metrics/activity_metrics.cpp:1200`](../src/activity_metrics/activity_metrics.cpp#L1200)
- **Purpose:** Calculates overall form score for running technique
- **Returns:** `uint8_t` - Form score as percentage (0-100)
- **Algorithm:** Assesses form based on:
  - Cadence (optimal: 160-200 spm)
  - Contact time (<300ms preferred)
  - Balance/asymmetry
  - Strike pattern (penalizes heel striking)
- **Data Requirements:** Motion sensor (cadence), Foot sensors (both feet for contact and strike)

#### **calculate_efficiency_score**
- **Location:** [`src/activity_metrics/activity_metrics.cpp:1240`](../src/activity_metrics/activity_metrics.cpp#L1240)
- **Parameters:**
  - `float contact_time` - Ground contact time in milliseconds
  - `float flight_time` - Flight time in milliseconds
- **Returns:** `uint8_t` - Efficiency score as percentage (0-100)
- **Algorithm:** Uses duty factor (contact time / total stride time), optimal around 0.35
- **Data Requirements:** Foot sensor data (both feet for contact and flight times)

#### **calculate_fatigue_level**
- **Location:** [`src/activity_metrics/activity_metrics.cpp:1264`](../src/activity_metrics/activity_metrics.cpp#L1264)
- **Purpose:** Calculates fatigue level based on changes in contact time
- **Returns:** `uint8_t` - Fatigue level as percentage (0-100)
- **Algorithm:** Estimates fatigue from contact time increase compared to baseline
- **Data Requirements:** Foot sensor data (both feet for contact time)

### Weight Measurement Functions

#### **calculate_weight_from_pressure**
- **Location:** [`src/activity_metrics/activity_metrics.cpp:1682`](../src/activity_metrics/activity_metrics.cpp#L1682)
- **Purpose:** Calculates weight from foot pressure data
- **Returns:** `float` - Calculated weight in kilograms
- **Algorithm:** 
  - Averages buffered pressure readings
  - Applies linear or polynomial calibration
  - Converts force (N) to mass (kg) using g=9.81 m/s²
- **Calibration Parameters:**
  - `scale_factor` - Newtons per ADC unit
  - `nonlinear_a/b/c` - Polynomial coefficients for FSR sensors
  - `temp_coeff` - Temperature compensation factor
- **Data Requirements:** Foot sensor data (both feet)

#### **apply_weight_calibration**
- **Location:** [`src/activity_metrics/activity_metrics.cpp:1725`](../src/activity_metrics/activity_metrics.cpp#L1725)
- **Parameters:** `float raw_weight_kg` - Raw weight in kilograms
- **Returns:** `float` - Calibrated weight in kilograms
- **Algorithm:** Applies temperature compensation and sanity limits (0-500kg)
- **Data Requirements:** Foot sensor data (both feet for raw weight)

#### **is_person_standing_still**
- **Location:** [`src/activity_metrics/activity_metrics.cpp:1746`](../src/activity_metrics/activity_metrics.cpp#L1746)
- **Purpose:** Checks if person is standing still based on motion data
- **Returns:** `bool` - True if standing still
- **Algorithm:** Checks acceleration magnitude < 0.5 m/s² and gyro magnitude < 0.1 rad/s
- **Data Requirements:** Motion sensor data (BHI360 for acceleration and gyro)

### Bilateral Metrics Functions (Primary Device Only)

#### **calculate_gait_symmetry**
- **Location:** [`src/analytics/bilateral_metrics.cpp:138`](../src/analytics/bilateral_metrics.cpp#L138)
- **Purpose:** Calculate gait symmetry index
- **Returns:** `float` - Symmetry percentage (100% = perfect symmetry)
- **Algorithm:** Compares GCT, stride length, and cadence between feet
- **Data Requirements:** D2D metrics from both feet

#### **calculate_phase_coordination**
- **Location:** [`src/analytics/bilateral_metrics.cpp:194`](../src/analytics/bilateral_metrics.cpp#L194)
- **Purpose:** Calculate phase coordination between feet
- **Returns:** `float` - Phase coordination index (0-100)
- **Algorithm:** Analyzes stance/swing phase relationships, ideal offset is 0.5 (180°)
- **Data Requirements:** Stance and swing times from both feet

#### **calculate_load_distribution**
- **Location:** [`src/analytics/bilateral_metrics.cpp:246`](../src/analytics/bilateral_metrics.cpp#L246)
- **Purpose:** Calculate load distribution between feet
- **Returns:** `float` - Load distribution ratio (-100 to +100, 0 = balanced)
- **Algorithm:** Uses peak pressure or GCT as proxy for load
- **Data Requirements:** Peak pressure data from both feet

---

## Analytics Module

### Complex Analytics Functions

#### **calculate_fatigue_index_placeholder**
- **Location:** [`src/analytics/analytics.cpp:546`](../src/analytics/analytics.cpp#L546)
- **Purpose:** Implement fatigue detection algorithm
- **Returns:** `float` - Fatigue index (0-100)
- **TODO:** Should track:
  - Contact time increase over baseline
  - Form score degradation
  - Asymmetry increase
  - Cadence decrease

#### **calculate_injury_risk_placeholder**
- **Location:** [`src/analytics/analytics.cpp:558`](../src/analytics/analytics.cpp#L558)
- **Purpose:** Implement injury risk assessment
- **Returns:** `float` - Injury risk percentage (0-100)
- **TODO:** Should analyze:
  - Loading rate trends
  - Pronation angles (excessive)
  - Asymmetry levels
  - Fatigue level
  - Previous injury history

#### **calculate_stride_length_placeholder**
- **Location:** [`src/analytics/analytics.cpp:571`](../src/analytics/analytics.cpp#L571)
- **Purpose:** Implement stride length calculation
- **Returns:** `float` - Stride length in meters
- **TODO:** Should use:
  - User height
  - Current cadence
  - Current speed (GPS or estimated)
  - Terrain adjustment factor

#### **calculate_pronation_analysis_placeholder**
- **Location:** [`src/analytics/analytics.cpp:584`](../src/analytics/analytics.cpp#L584)
- **Purpose:** Implement pronation trend analysis
- **Returns:** `float` - Pronation angle in degrees
- **TODO:** Should track:
  - Average pronation angle
  - Left/right differences
  - Changes with fatigue
  - Correlation with strike pattern

#### **calculate_vertical_stiffness_placeholder**
- **Location:** [`src/analytics/analytics.cpp:596`](../src/analytics/analytics.cpp#L596)
- **Purpose:** Implement vertical stiffness calculation
- **Returns:** `float` - Stiffness value (typical: 200-250)
- **TODO:** Should calculate:
  - Spring-mass model stiffness
  - Based on vertical oscillation and ground contact
  - Normalized by body weight

#### **calculate_recovery_score_placeholder**
- **Location:** [`src/analytics/analytics.cpp:607`](../src/analytics/analytics.cpp#L607)
- **Purpose:** Implement recovery quality assessment
- **Returns:** `float` - Recovery score (0-100)
- **TODO:** Should evaluate:
  - Heart rate recovery (if available)
  - Gait quality post-activity
  - Asymmetry resolution
  - Return to baseline metrics

### Baseline and Session Functions

#### **establish_baseline**
- **Location:** [`src/analytics/analytics.cpp:483`](../src/analytics/analytics.cpp#L483)
- **Purpose:** Establishes baseline metrics during initial activity period
- **Algorithm:** Calculates averages from history buffer for:
  - Contact time
  - Cadence
  - Pronation
  - Efficiency
- **Data Requirements:** Motion and foot sensor data accumulated over time

#### **perform_complex_analytics**
- **Location:** [`src/analytics/analytics.cpp:420`](../src/analytics/analytics.cpp#L420)
- **Purpose:** Performs complex analytics calculations at 5Hz
- **Calculations:**
  - Running efficiency (multi-factor)
  - Fatigue index (baseline comparison)
  - Injury risk assessment (composite)
  - Stride length estimation
  - Pronation analysis
- **Data Requirements:** Motion sensor (BHI360), Foot sensor data (both feet)

### Choros Buffer Functions

#### **choros_get_gct_safe**
- **Location:** [`src/analytics/choros_buffer_impl.cpp:216`](../src/analytics/choros_buffer_impl.cpp#L216)
- **Parameters:** `float fallback_value` - Default value if calculation fails
- **Returns:** `float` - Ground contact time in milliseconds
- **Algorithm:** Detects IC (Initial Contact) to TO (Toe Off) transitions using 1000.0 pressure threshold
- **Data Requirements:** Raw pressure data from buffer

#### **choros_get_stride_length_safe**
- **Location:** [`src/analytics/choros_buffer_impl.cpp:296`](../src/analytics/choros_buffer_impl.cpp#L296)
- **Parameters:** `float fallback_value` - Default value if calculation fails
- **Returns:** `float` - Stride length in meters
- **Algorithm:** Velocity integration between IC events
- **Data Requirements:** IMU acceleration data and pressure data

#### **choros_get_pronation_safe**
- **Location:** [`src/analytics/choros_buffer_impl.cpp:371`](../src/analytics/choros_buffer_impl.cpp#L371)
- **Parameters:** `float fallback_value` - Default value if calculation fails
- **Returns:** `float` - Pronation angle in degrees
- **Algorithm:** Gyro integration over last 50ms before IC
- **Data Requirements:** Gyroscope and pressure data

#### **choros_get_cadence_safe**
- **Location:** [`src/analytics/choros_buffer_impl.cpp:427`](../src/analytics/choros_buffer_impl.cpp#L427)
- **Parameters:** `float fallback_value` - Default value if calculation fails
- **Returns:** `float` - Cadence in steps per minute
- **Algorithm:** Counts IC events over time window
- **Data Requirements:** Pressure data for IC detection

---

## Realtime Metrics Module

### Core Tracking Functions

#### **cadence_tracker_update**
- **Location:** [`src/realtime_metrics/realtime_metrics_algorithms.c:21`](../src/realtime_metrics/realtime_metrics_algorithms.c#L21)
- **Parameters:**
  - `cadence_tracker_t *tracker` - Tracker state structure
  - `uint32_t step_count` - Current step count
  - `uint32_t timestamp_ms` - Current timestamp
- **Algorithm:** 
  - Maintains circular buffer of step counts
  - Applies exponential smoothing (70% old, 30% new)
- **Data Requirements:** Step count from motion sensor

#### **pace_estimator_update**
- **Location:** [`src/realtime_metrics/realtime_metrics_algorithms.c:81`](../src/realtime_metrics/realtime_metrics_algorithms.c#L81)
- **Parameters:**
  - `pace_estimator_t *estimator` - Estimator state
  - `float cadence_spm` - Current cadence
  - `uint32_t delta_time_ms` - Time delta
- **Algorithm:**
  - Adjusts stride length based on cadence
  - Calculates speed from cadence × stride
  - Converts to pace (sec/km)
- **Data Requirements:** Cadence from motion sensor

#### **pace_estimator_update_with_gps**
- **Location:** [`src/realtime_metrics/realtime_metrics_algorithms.c:126`](../src/realtime_metrics/realtime_metrics_algorithms.c#L126)
- **Parameters:**
  - `pace_estimator_t *estimator` - Estimator state
  - `float gps_speed_cms` - GPS speed in cm/s
  - `uint32_t delta_time_ms` - Time delta
- **Algorithm:** Direct pace calculation from GPS speed with smoothing
- **Data Requirements:** GPS speed data

#### **form_score_calculate**
- **Location:** [`src/realtime_metrics/realtime_metrics_algorithms.c:149`](../src/realtime_metrics/realtime_metrics_algorithms.c#L149)
- **Parameters:**
  - `form_score_t *score` - Score structure
  - `uint16_t contact_time_ms` - Ground contact time
  - `int8_t balance_pct` - Balance percentage
  - `float variability` - Contact time variability
  - `int8_t pronation_deg` - Pronation angle
- **Algorithm:** Weighted average of:
  - Contact time score (30%)
  - Balance score (30%)
  - Consistency score (20%)
  - Pronation score (20%)
- **Data Requirements:** Multiple metrics from foot and motion sensors

#### **calculate_asymmetry_percentage**
- **Location:** [`src/realtime_metrics/realtime_metrics_algorithms.c:214`](../src/realtime_metrics/realtime_metrics_algorithms.c#L214)
- **Parameters:**
  - `uint16_t left_value` - Left foot metric
  - `uint16_t right_value` - Right foot metric
- **Returns:** `uint8_t` - Asymmetry percentage (0-100)
- **Algorithm:** `(max-min)/max × 100`
- **Data Requirements:** Bilateral foot data

#### **calculate_balance_percentage**
- **Location:** [`src/realtime_metrics/realtime_metrics_algorithms.c:236`](../src/realtime_metrics/realtime_metrics_algorithms.c#L236)
- **Parameters:**
  - `uint16_t left_force` - Left foot force
  - `uint16_t right_force` - Right foot force
- **Returns:** `int8_t` - Balance percentage (-50 to +50)
- **Algorithm:** `(right-left)/total × 100`
- **Data Requirements:** Force data from both feet

#### **calibrate_stride_with_gps**
- **Location:** [`src/realtime_metrics/realtime_metrics_algorithms.c:258`](../src/realtime_metrics/realtime_metrics_algorithms.c#L258)
- **Parameters:**
  - `pace_estimator_t *estimator` - Estimator state
  - `float gps_distance_m` - GPS distance
  - `uint32_t step_count_delta` - Steps taken
- **Returns:** `float` - Calibrated stride length in meters
- **Algorithm:** `distance/steps` with exponential smoothing
- **Data Requirements:** GPS distance and step count

#### **estimate_stride_without_gps**
- **Location:** [`src/realtime_metrics/realtime_metrics_algorithms.c:271`](../src/realtime_metrics/realtime_metrics_algorithms.c#L271)
- **Parameters:**
  - `float cadence` - Current cadence
  - `uint16_t contact_time` - Ground contact time
  - `float height_cm` - User height
- **Returns:** `float` - Estimated stride length in cm
- **Algorithm:** Base from height × cadence factor × contact factor
- **Data Requirements:** Cadence, contact time, user height

---

## Sensor Data Module

### Contact Detection Functions

#### **detect_ground_contact** (in sensor_data_enhanced_algorithms.hpp)
- **Purpose:** Detects if foot is in contact with ground
- **Parameters:**
  - `uint16_t pressure[8]` - Pressure sensor array
  - `bool was_in_contact` - Previous contact state
- **Returns:** `bool` - True if in contact
- **Algorithm:** Uses adaptive thresholds with hysteresis
- **Data Requirements:** Raw pressure sensor data

#### **detect_contact_phase** (in sensor_data_enhanced_algorithms.hpp)
- **Purpose:** Determines current phase of ground contact
- **Parameters:**
  - `uint16_t pressure[8]` - Pressure sensor array
  - `bool was_in_contact` - Previous contact state
- **Returns:** `contact_phase_t` - Contact phase enum
- **Phases:**
  - PHASE_NO_CONTACT
  - PHASE_LOADING (heel → mid)
  - PHASE_MIDSTANCE (full contact)
  - PHASE_PUSH_OFF (toe off)
- **Data Requirements:** Pressure sensor data

#### **detect_peak_force** (in sensor_data_enhanced_algorithms.hpp)
- **Purpose:** Detects peak force during contact
- **Parameters:** `uint16_t pressure[8]` - Pressure sensor array
- **Returns:** `uint16_t` - Peak force value
- **Algorithm:** Sums all pressure sensors
- **Data Requirements:** Pressure sensor data

### Spatial Analysis Functions

#### **calculate_center_of_pressure** (in sensor_data_enhanced_algorithms.hpp)
- **Purpose:** Calculates center of pressure location
- **Parameters:**
  - `uint16_t pressure[8]` - Pressure sensor array
  - `int16_t *cop_x` - Output X coordinate
  - `int16_t *cop_y` - Output Y coordinate
  - `bool is_left_foot` - Foot identifier
- **Algorithm:** Weighted average of sensor positions by pressure
- **Sensor Layout:**
  - Sensors 0-3: Forefoot
  - Sensors 4-5: Midfoot/arch
  - Sensors 6-7: Heel
- **Data Requirements:** Pressure sensor data and sensor position map

#### **calculate_pressure_distribution** (in sensor_data_enhanced_algorithms.hpp)
- **Purpose:** Calculates pressure distribution across foot regions
- **Parameters:**
  - `uint16_t pressure[8]` - Pressure sensor array
  - `uint8_t *heel_pct` - Heel pressure percentage
  - `uint8_t *mid_pct` - Midfoot pressure percentage
  - `uint8_t *fore_pct` - Forefoot pressure percentage
- **Algorithm:** Normalizes regional pressure to percentages
- **Data Requirements:** Pressure sensor data

### Strike Pattern Analysis

#### **detect_strike_pattern** (in sensor_data_enhanced_algorithms.hpp)
- **Purpose:** Identifies foot strike pattern
- **Parameters:**
  - `uint8_t heel_pct` - Heel pressure percentage
  - `uint8_t mid_pct` - Midfoot pressure percentage
  - `uint8_t fore_pct` - Forefoot pressure percentage
- **Returns:** `uint8_t` - Strike pattern:
  - 0: Unknown
  - 1: Heel strike (heel > 60%)
  - 2: Midfoot strike (mid > 40%)
  - 3: Forefoot strike (fore > 60%)
- **Data Requirements:** Pressure distribution data

#### **calculate_foot_strike_angle** (in sensor_data_enhanced_algorithms.hpp)
- **Purpose:** Calculates foot strike angle using IMU and pressure
- **Parameters:**
  - `float quaternion[4]` - IMU quaternion
  - `uint16_t heel_force` - Heel pressure
  - `uint16_t fore_force` - Forefoot pressure
- **Returns:** `int8_t` - Strike angle in degrees (-30 to +30)
- **Algorithm:** Combines IMU orientation with pressure distribution
- **Data Requirements:** IMU quaternion and pressure data

### Dynamic Analysis Functions

#### **calculate_loading_rate** (in sensor_data_enhanced_algorithms.hpp)
- **Purpose:** Calculates vertical loading rate
- **Parameters:**
  - `uint16_t current_force` - Current force
  - `uint16_t previous_force` - Previous force
  - `uint16_t delta_time_ms` - Time between samples
- **Returns:** `uint16_t` - Loading rate in N/s
- **Algorithm:** `(force_delta / time_delta) × 1000`
- **Data Requirements:** Sequential force measurements

#### **detect_arch_collapse** (in sensor_data_enhanced_algorithms.hpp)
- **Purpose:** Detects arch collapse during stance
- **Parameters:**
  - `uint16_t pressure[8]` - Pressure sensor array
  - `contact_phase_t phase` - Current contact phase
- **Returns:** `uint8_t` - Arch collapse index (0-100)
- **Algorithm:** Monitors midfoot pressure increase during midstance
- **Data Requirements:** Pressure data and contact phase

### Bilateral Timing Functions

#### **update_true_flight_time** (in sensor_data_fast_processing.hpp)
- **Purpose:** Updates true flight time (both feet off ground)
- **Parameters:**
  - `bool left_contact` - Left foot contact state
  - `bool right_contact` - Right foot contact state
  - `uint16_t delta_time_ms` - Time delta
  - `uint16_t current_flight_time` - Current flight time
- **Returns:** `uint16_t` - Updated flight time in ms
- **Algorithm:** Accumulates time when both feet are off ground
- **Data Requirements:** Bilateral contact states

#### **update_double_support_time** (in sensor_data_fast_processing.hpp)
- **Purpose:** Updates double support time (both feet on ground)
- **Parameters:**
  - `bool left_contact` - Left foot contact state
  - `bool right_contact` - Right foot contact state
  - `uint16_t delta_time_ms` - Time delta
  - `uint16_t current_double_support` - Current double support time
- **Returns:** `uint16_t` - Updated double support time in ms
- **Algorithm:** Accumulates time when both feet are on ground
- **Data Requirements:** Bilateral contact states

### Advanced Biomechanics Functions

#### **calculate_cpei**
- **Location:** [`src/sensor_data/sensor_data.cpp:1785`](../src/sensor_data/sensor_data.cpp#L1785)
- **Purpose:** Calculate Center of Pressure Excursion Index
- **Parameters:**
  - `const uint16_t pressure[8]` - Pressure array
  - `int16_t cop_x` - COP X coordinate
  - `int16_t cop_y` - COP Y coordinate
- **Returns:** `uint16_t` - CPEI percentage
- **Algorithm:** (COP deviation / foot width) × 100
- **TODO:** Full implementation should track COP trajectory over entire stance
- **Data Requirements:** Pressure data and COP coordinates

#### **calculate_push_off_power**
- **Location:** [`src/sensor_data/sensor_data.cpp:1816`](../src/sensor_data/sensor_data.cpp#L1816)
- **Purpose:** Calculate push-off power during toe-off
- **Parameters:**
  - `uint16_t force` - Push-off force
  - `float gyro_y` - Y-axis gyroscope
  - `uint16_t delta_time_ms` - Time delta
- **Returns:** `uint16_t` - Push-off power
- **TODO:** Implement proper calculation using Force × Velocity
- **Data Requirements:** Force and gyroscope data

---

## Advanced Metrics Summary

### Primary Device Exclusive Metrics
These metrics require data from both feet and are only calculated on the primary device:

1. **Bilateral Timing**
   - True flight time (both feet off ground)
   - Double support time (both feet on ground)
   - Phase coordination index

2. **Symmetry Analysis**
   - Gait symmetry index
   - Contact time asymmetry
   - Stride length asymmetry
   - Force asymmetry
   - Pronation asymmetry

3. **Load Distribution**
   - Left/right balance percentage
   - Peak pressure comparison
   - Weight distribution ratio

### Real-Time Processing Metrics (100Hz)
High-frequency metrics calculated in sensor_data module:

1. **Contact Detection**
   - Ground contact state
   - Contact phase (loading/midstance/push-off)
   - Contact duration tracking

2. **Force Metrics**
   - Peak force
   - Loading rate
   - Center of pressure (COP)
   - Pressure distribution

3. **Strike Analysis**
   - Strike pattern (heel/mid/fore)
   - Strike angle
   - Arch collapse detection

### Mid-Frequency Metrics (10-50Hz)
Calculated in realtime_metrics module:

1. **Core Metrics**
   - Cadence (with smoothing)
   - Pace (sensor-based or GPS)
   - Ground contact time
   - Flight time

2. **Form Analysis**
   - Form score (composite)
   - Balance score
   - Efficiency score
   - Consistency score

3. **Asymmetry Detection**
   - Contact time asymmetry
   - Force asymmetry
   - Stride asymmetry

### Low-Frequency Analytics (1-5Hz)
Complex calculations in analytics module:

1. **Baseline Tracking**
   - Baseline establishment (2-minute window)
   - Trend analysis
   - Fatigue detection

2. **Advanced Biomechanics**
   - Running efficiency (multi-factor)
   - Injury risk assessment
   - Vertical stiffness
   - Recovery quality

3. **GPS Integration**
   - Stride calibration
   - Distance correction
   - Elevation tracking

### Weight Measurement System
Specialized subsystem in activity_metrics:

1. **Calibration**
   - Linear calibration (scale factor)
   - Polynomial calibration (FSR sensors)
   - Temperature compensation

2. **Measurement Process**
   - Motion detection (standing still)
   - Pressure averaging
   - Force-to-mass conversion

3. **Safety Features**
   - Sanity limits (20-300kg)
   - Timeout protection (10 seconds)
   - Buffer overflow prevention

---

## Data Flow Summary

### Sensor Data Collection (100Hz)
1. **Foot Sensors** → pressure[8] arrays
2. **IMU (BHI360)** → quaternion, linear_acc, gyro
3. **Step Counter** → step_count

### Processing Pipeline
1. **sensor_data** (100Hz): Raw processing, contact detection, COP
2. **realtime_metrics** (10-50Hz): Cadence, pace, form scores
3. **analytics** (1-5Hz): Complex analytics, baseline tracking
4. **activity_metrics** (1Hz): Session management, BLE updates

### D2D Communication (Secondary → Primary)
- **Raw Data Mode**: foot_samples_t, bhi360_log_record_t
- **Parameter Mode**: d2d_metrics_packet_t with calculated metrics
- **Bilateral Processing**: Only on primary device

### Output Interfaces
1. **Bluetooth (BLE)**: RealtimeMetricsPacket at 1Hz
2. **Data Storage**: Activity records for logging
3. **User Feedback**: Alerts and notifications

---

## Notes and TODOs

### Placeholder Functions
Several functions in the analytics module are marked as placeholders and need full implementation:
- Fatigue index calculation
- Injury risk assessment
- Stride length estimation
- Pronation analysis
- Vertical stiffness
- Recovery score

### Optimization Opportunities
1. **Choros Buffer**: Currently implements basic algorithms, could be enhanced with:
   - Kalman filtering for drift correction
   - Machine learning for pattern recognition
   - Adaptive thresholds based on user profile

2. **D2D Synchronization**: Network core optimization for lower latency
   - Current: 50-300ms window
   - Target: <50ms for real-time bilateral analysis

3. **Weight Measurement**: Could be enhanced with:
   - Multi-point calibration
   - Automatic drift compensation
   - User-specific profiles

### Data Requirements Summary
- **Motion Sensor (BHI360)**: Required for cadence, orientation, step count
- **Foot Sensors**: Required for contact, force, strike pattern
- **Both Feet**: Required for bilateral metrics (primary device only)
- **GPS**: Optional but improves distance/pace accuracy
- **User Profile**: Height, weight, age for personalized calculations
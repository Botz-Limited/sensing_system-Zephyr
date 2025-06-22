# BHI360 Metrics Capability Analysis

**Date:** December 2024  
**Purpose:** Analysis of which proposed metrics can be calculated using BHI360 sensors alone versus requiring pressure sensors

---

## BHI360 Available Data

The BHI360 provides the following sensor data:
- **Quaternion** (orientation): 50Hz, 4 values (w,x,y,z)
- **Linear Acceleration** (without gravity): 50Hz, 3-axis (x,y,z) in m/s²
- **Gyroscope** (angular velocity): 50Hz, 3-axis (x,y,z) in rad/s
- **Step Counter**: 5Hz, cumulative count
- **Derived**: Euler angles from quaternion, velocity/position from acceleration integration

---

## Metrics Analysis by Category

### 1. Core Step Metrics

| Metric | BHI360 Only | Requires Pressure | Method | Computational Impact |
|--------|-------------|-------------------|---------|---------------------|
| **Ground Contact Time** | ❌ Partial | ✅ Full accuracy | BHI360: Detect impact/push-off from acceleration peaks<br>Pressure: Direct measurement when force > threshold | Medium (peak detection) |
| **Flight Time** | ❌ Partial | ✅ Full accuracy | BHI360: Time between push-off and landing accelerations<br>Pressure: Time when force = 0 | Medium (state machine) |
| **Step Frequency** | ✅ Yes | ✅ Better | BHI360: Built-in step counter or acceleration periodicity<br>Pressure: More accurate timing | Low (already computed) |
| **Peak Force** | ❌ No | ✅ Required | Cannot derive force from acceleration alone<br>Need pressure sensors for force measurement | N/A |
| **Loading Rate** | ❌ No | ✅ Required | Rate of force increase requires force sensors | N/A |
| **Push-off Power** | ❌ Estimated | ✅ Accurate | BHI360: Estimate from acceleration × body mass<br>Pressure: Direct force × velocity calculation | High (integration) |

### 2. Pressure Distribution

| Metric | BHI360 Only | Requires Pressure | Method | Computational Impact |
|--------|-------------|-------------------|---------|---------------------|
| **Heel/Midfoot/Forefoot %** | ❌ No | ✅ Required | Requires spatial pressure distribution | N/A |
| **Center of Pressure** | ❌ No | ✅ Required | Requires pressure sensor array | N/A |
| **Pressure Path Length** | ❌ No | ✅ Required | Requires CoP trajectory from pressure data | N/A |

### 3. Motion Dynamics

| Metric | BHI360 Only | Requires Pressure | Method | Computational Impact |
|--------|-------------|-------------------|---------|---------------------|
| **Foot Strike Angle** | ✅ Yes | ✅ Enhanced | BHI360: Foot angle from quaternion at impact detection<br>Pressure: More precise impact timing | Low (quaternion to euler) |
| **Pronation Angle** | ✅ Yes | ✅ Enhanced | BHI360: Roll angle from quaternion during stance<br>Pressure: Correlate with medial shift | Low (angle extraction) |
| **Pronation Velocity** | ✅ Yes | ✅ Enhanced | BHI360: Gyroscope Z-axis (roll rate)<br>Pressure: Timing refinement | Low (direct from gyro) |
| **Vertical Oscillation** | ✅ Yes | ❌ Optional | BHI360: Double integrate vertical acceleration<br>Pressure: Not needed | High (double integration) |
| **Impact G-force** | ✅ Yes | ❌ Optional | BHI360: Peak acceleration at landing<br>Pressure: Better timing | Low (peak detection) |
| **Movement Smoothness** | ✅ Yes | ❌ Optional | BHI360: Jerk (acceleration derivative) analysis<br>Pressure: Not needed | Medium (signal analysis) |

### 4. Gait Symmetry

| Metric | BHI360 Only | Requires Pressure | Method | Computational Impact |
|--------|-------------|-------------------|---------|---------------------|
| **Contact Time Asymmetry** | ❌ Limited | ✅ Accurate | Need accurate contact detection from both feet | Low (comparison) |
| **Flight Time Asymmetry** | ❌ Limited | ✅ Accurate | Need accurate flight phase detection | Low (comparison) |
| **Force Asymmetry** | ❌ No | ✅ Required | Cannot measure force without pressure sensors | N/A |
| **Step Length Asymmetry** | ✅ Estimated | ✅ Better | BHI360: Integrate acceleration<br>Pressure: Better step timing | High (integration) |
| **Pronation Asymmetry** | ✅ Yes | ✅ Enhanced | BHI360: Compare roll angles between feet | Low (comparison) |

### 5. Performance Indicators

| Metric | BHI360 Only | Requires Pressure | Method | Computational Impact |
|--------|-------------|-------------------|---------|---------------------|
| **Running Efficiency** | ❌ Rough | ✅ Accurate | BHI360: Estimate from motion only<br>Pressure: Include ground reaction forces | High (complex model) |
| **Stride Length** | ✅ Estimated | ✅ Better | BHI360: Integrate acceleration or use step count + speed<br>Pressure: Precise step timing | High (integration) |
| **Estimated Speed** | ✅ Yes | ✅ Better | BHI360: Integrate acceleration or cadence × stride<br>Pressure: More accurate step timing | High (integration/filtering) |
| **Vertical Stiffness** | ❌ Limited | ✅ Full | BHI360: Estimate from vertical motion<br>Pressure: Need force for spring constant | High (modeling) |
| **Duty Factor** | ❌ Limited | ✅ Accurate | Ratio needs accurate contact/flight times | Low (ratio) |

### 6. Health & Risk Indicators

| Metric | BHI360 Only | Requires Pressure | Method | Computational Impact |
|--------|-------------|-------------------|---------|---------------------|
| **Cumulative Impact Load** | ✅ Estimated | ✅ Accurate | BHI360: Sum of acceleration impacts<br>Pressure: True force accumulation | Medium (accumulation) |
| **Fatigue Index** | ✅ Partial | ✅ Complete | BHI360: Motion pattern changes<br>Pressure: Include loading changes | High (pattern analysis) |
| **Form Deterioration** | ✅ Partial | ✅ Complete | BHI360: Motion quality degradation<br>Pressure: Include pressure patterns | High (ML/statistics) |
| **Overstriding Indicator** | ✅ Estimated | ✅ Accurate | BHI360: Foot angle + acceleration pattern<br>Pressure: Heel strike force pattern | Medium (pattern matching) |
| **Lateral Instability** | ✅ Yes | ✅ Enhanced | BHI360: Gyro/acceleration variability<br>Pressure: Add CoP lateral movement | Medium (variance analysis) |

---

## Summary Statistics

### Metrics Feasibility with BHI360 Alone

| Category | BHI360 Sufficient | Partial/Estimated | Requires Pressure |
|----------|------------------|-------------------|-------------------|
| Core Step Metrics (6) | 1 (17%) | 3 (50%) | 2 (33%) |
| Pressure Distribution (3) | 0 (0%) | 0 (0%) | 3 (100%) |
| Motion Dynamics (6) | 5 (83%) | 1 (17%) | 0 (0%) |
| Gait Symmetry (5) | 1 (20%) | 3 (60%) | 1 (20%) |
| Performance Indicators (5) | 1 (20%) | 4 (80%) | 0 (0%) |
| Health & Risk (5) | 2 (40%) | 3 (60%) | 0 (0%) |
| **TOTAL (30)** | **10 (33%)** | **14 (47%)** | **6 (20%)** |

### Computational Impact Summary

| Impact Level | Count | Percentage | Typical CPU Load |
|--------------|-------|------------|------------------|
| Low | 8 | 27% | <5% per metric |
| Medium | 9 | 30% | 5-15% per metric |
| High | 7 | 23% | 15-30% per metric |
| N/A (needs pressure) | 6 | 20% | - |

---

## Implementation Recommendations

### 1. BHI360-Only Configuration (Reduced Feature Set)

For a system using only BHI360, you could provide:

**High Confidence Metrics:**
- Step frequency/cadence
- Foot strike angle
- Pronation angle and velocity
- Vertical oscillation
- Impact G-force
- Movement smoothness
- Basic lateral instability

**Estimated Metrics (Lower Accuracy):**
- Approximate contact/flight time
- Estimated stride length and speed
- Basic fatigue detection
- Overstriding warnings

**Missing Capabilities:**
- No force/power measurements
- No pressure distribution
- No accurate loading rates
- Limited gait symmetry analysis

### 2. Computational Optimization Strategies

**For High-Impact Calculations:**
1. **Integration-based metrics** (speed, distance, vertical oscillation):
   - Use Kalman filtering to reduce drift
   - Reset integration at known events (steps)
   - Downsample to reduce computation

2. **Pattern Analysis** (fatigue, form deterioration):
   - Use sliding window statistics
   - Implement efficient feature extraction
   - Consider edge ML models

3. **Real-time vs Post-processing:**
   - Calculate only essential metrics in real-time
   - Defer complex analysis to post-activity
   - Use event-driven processing

### 3. Hybrid Approach Benefits

Combining BHI360 + Pressure Sensors provides:
- **Sensor Fusion**: More accurate metrics through redundancy
- **Validation**: Cross-check BHI360 estimates with pressure data
- **Complete Picture**: Motion dynamics + force dynamics
- **Reduced Computation**: Pressure sensors simplify contact detection

---

## Conclusion

While the BHI360 can calculate or estimate about 80% of the proposed metrics (with varying accuracy), the pressure sensors are essential for:
1. **Force-based measurements** (peak force, loading rate, power)
2. **Accurate timing** (contact/flight phases)
3. **Spatial information** (pressure distribution, center of pressure)
4. **High-accuracy biomechanics** (true asymmetry, efficiency)

The computational impact of BHI360-only implementation would be significant (especially for integration-based metrics), but manageable with proper optimization. The combination of BHI360 + pressure sensors provides the most accurate and computationally efficient solution.
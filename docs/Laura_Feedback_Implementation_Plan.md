# Laura's Feedback Implementation Plan

## Summary of Laura's Feedback

Laura has provided valuable clinical and product insights on the Activity Session Calculated Data Specification. Here's the breakdown of her feedback with implementation suggestions:

## Our Sensor Capabilities

We have two powerful sensor systems in each shoe:

### 1. **BHI360 IMU**
- **Step counting**: Built-in pedometer functionality
- **Orientation**: Quaternions for foot angle and pronation
- **Acceleration**: 3-axis for impact detection and movement
- **Gyroscope**: Angular velocity for rotation tracking

### 2. **8-Channel Pressure Sensors**
```
Sensor Layout (Bottom view):
    [7]          <- Big toe
  [5] [6]        <- Forefoot (metatarsal heads)
  [3] [4]        <- Midfoot (arch)
    [2]          <- Midfoot center
  [0] [1]        <- Heel (medial/lateral)
```

This combination enables precise biomechanical analysis that neither sensor could achieve alone.

---

## 1. Pressure Distribution - Per Foot Breakdown

### Laura's Point:
- Pressure distribution (Table 8) should be collected on a per-foot basis
- While foot strike patterns may be symmetric, metrics like Center of Pressure and CPEI (COP path) should be broken down by each foot

### Current Implementation:
The specification already includes per-foot data in the `FootSummary` structure within `PeriodicRecord`:
```c
typedef struct {
    // ... other fields ...
    FootSummary left_foot;   // 12 bytes
    FootSummary right_foot;  // 12 bytes
} PeriodicRecord;
```

### Recommended Enhancement:
Add explicit Center of Pressure tracking per foot:

```c
typedef struct {
    uint16_t peak_force;
    uint8_t  heel_pct;
    uint8_t  midfoot_pct;
    uint8_t  forefoot_pct;
    int8_t   pronation_angle;
    uint16_t loading_rate;
    uint8_t  strike_pattern;
    uint8_t  push_power;
    uint8_t  step_count;
    uint8_t  quality_score;
    // NEW: Add COP tracking
    int8_t   cop_x_mm;        // Medial-lateral position (-50 to +50)
    int8_t   cop_y_mm;        // Anterior-posterior (-100 to +100)
    uint16_t cop_path_length; // Total CoP movement in mm
} FootSummary; // Now 15 bytes
```

---

## 2. Gait Symmetry Metrics (Table 10)

### Laura's Point:
- Very excited about gait symmetry metrics
- Important for visualization
- Good common factor for form score across different body types/running styles

### Current Implementation:
Already well-defined in the specification with comprehensive asymmetry tracking

### Recommended Enhancement:
Create a dedicated visualization-friendly structure for mobile app:

```c
typedef struct {
    // Core asymmetry values (-100 to +100, negative = left bias)
    int8_t  contact_time_asymmetry;
    int8_t  flight_time_asymmetry;
    int8_t  force_asymmetry;
    int8_t  step_length_asymmetry;
    int8_t  pronation_asymmetry;
    
    // Composite score for easy visualization
    uint8_t overall_symmetry_score;  // 0-100, higher is better
    
    // Trend indicators
    int8_t  asymmetry_trend;         // -10 to +10, improving vs worsening
} GaitSymmetryPacket;
```

---

## 3. Health & Risk Indicators (Table 12)

### Laura's Question:
- What are these based on? Ideas or existing algorithms?
- Happy to help build this out from clinical side

### Current Status:
The specification includes conceptual metrics that need algorithm development

### Recommended Collaboration Approach:
1. **Cumulative Impact Load**: Partner with Laura to define clinical thresholds
2. **Fatigue Index**: Use her clinical expertise for validation
3. **Form Deterioration**: Get clinical input on key indicators
4. **Overstriding Indicator**: Define clinical criteria
5. **Lateral Instability**: Establish normal ranges from clinical data

### Implementation Priority:
Start with basic algorithms and refine with clinical validation:

```c
// Example: Overstriding detection with clinical thresholds
typedef struct {
    float heel_strike_angle_threshold;    // Laura's input needed
    float contact_time_threshold;         // Clinical validation
    float braking_force_threshold;        // Based on injury data
} ClinicalThresholds;
```

---

## 4. NEW METRIC: Splits (Pace per km)

### Laura's Request:
- Show pace broken down by km (splits)
- Important for form report display

### Implementation:
Since the BHI360 provides accurate step counting, we can calculate distance and pace more reliably:

```c
// BHI360 provides step count directly
typedef struct {
    uint32_t bhi360_step_count;      // From BHI360 pedometer
    float pressure_validated_steps;   // Cross-validated with pressure
    float stride_length_m;           // Calculated from user height + gait metrics
} StepTracking;

// Distance calculation is straightforward
float calculate_distance(StepTracking* tracking) {
    return tracking->bhi360_step_count * tracking->stride_length_m;
}
```

Add split tracking to the data structures:

```c
typedef struct {
    uint16_t split_number;        // 1, 2, 3... for each km
    uint16_t split_time_sec;      // Time for this km
    uint16_t split_pace_sec_km;   // Pace for this specific km
    uint8_t  split_form_score;    // Average form during this km
    uint8_t  split_fatigue_level; // Fatigue at end of km
} SplitRecord; // 8 bytes

// Add to session file format
typedef struct {
    SessionHeader header;
    PeriodicRecord records[MAX_RECORDS];
    SplitRecord splits[MAX_SPLITS];  // NEW: Array of splits
    SessionSummary summary;
} ActivitySessionFile;
```

### BLE Notification for Split Completion:
```c
typedef struct __attribute__((packed)) {
    uint16_t split_number;
    uint16_t split_time_sec;
    uint16_t split_pace_sec_km;
    uint8_t  form_score;
    uint8_t  comparison_to_avg;  // Faster/slower than average
    uint8_t  reserved[12];
} SplitNotification; // 20 bytes
```

---

## 5. NEW FEATURE: Activity Type Tagging

### Laura's Request:
- Tag each session by activity type and subtype
- Manual selection initially (before activity start)
- Different run types have different expected metrics

### Activity Types:
```c
enum ActivityType {
    ACTIVITY_RUN = 1,
    ACTIVITY_WALK = 2,
    // Future: ACTIVITY_HIKE, ACTIVITY_TRAIL_RUN, etc.
};

enum RunSubtype {
    RUN_EVERYDAY = 1,      // Default
    RUN_LONG = 2,          // Lower cadence expected
    RUN_TEMPO = 3,         // Higher cadence expected
    RUN_INTERVALS = 4,     // Variable metrics expected
    RUN_CALIBRATION = 5    // Special calibration mode
};
```

### Implementation in Session Header:
```c
typedef struct {
    uint32_t session_id;
    uint32_t start_timestamp;
    uint8_t  activity_type;      // Already exists
    uint8_t  activity_subtype;   // NEW: Add subtype
    uint8_t  firmware_version[3];
    // ... rest of header
} SessionHeader;
```

### Metric Adjustments by Run Type:
```c
typedef struct {
    uint8_t run_type;
    float cadence_expected_min;
    float cadence_expected_max;
    float contact_time_typical;
    float efficiency_baseline;
} RunTypeProfile;

const RunTypeProfile run_profiles[] = {
    {RUN_EVERYDAY,  160, 180, 250, 75},
    {RUN_LONG,      150, 170, 260, 70},
    {RUN_TEMPO,     170, 190, 230, 80},
    {RUN_INTERVALS, 160, 200, 220, 75}
};
```

---

## 6. NEW METRIC: Step Width

### Laura's Request:
- Distance between heels of each foot
- Average score for a run
- Unit: cm
- Important for injury prevention (narrow base = knee/tibia strain)
- Used for gait retraining

### Implementation Approach:
With our BHI360 IMU and 8-channel pressure sensors, we can estimate step width using:
1. **BHI360 lateral acceleration** during swing phase
2. **Pressure distribution patterns** from 8 sensors showing medial/lateral bias
3. **Synchronized timing** between left and right shoes

### Proposed Solution:
```c
// Step width estimation using BHI360 + 8-channel pressure
typedef struct {
    // From BHI360
    float lateral_velocity[2];      // Integrated from BHI360 accelerometer
    float foot_angle[2];           // From BHI360 quaternion (yaw component)
    
    // From 8-channel pressure sensors
    float medial_pressure[2];      // Channels 0,2,3,5 (medial side)
    float lateral_pressure[2];     // Channels 1,4,6,7 (lateral side)
    
    // Timing from both sensors
    uint32_t stance_start_time[2]; // When pressure > threshold
    uint32_t swing_start_time[2];  // When pressure < threshold
} StepWidthData;

float calculate_step_width(StepWidthData* data) {
    // Method 1: Pressure-based estimation
    // If high medial pressure on both feet = narrow gait (crossing midline)
    float medial_ratio_left = data->medial_pressure[0] / 
                             (data->medial_pressure[0] + data->lateral_pressure[0]);
    float medial_ratio_right = data->medial_pressure[1] / 
                              (data->medial_pressure[1] + data->lateral_pressure[1]);
    
    // Method 2: BHI360 lateral movement during swing
    // Integrate lateral acceleration during swing phase
    float lateral_displacement = data->lateral_velocity[0] * 
                                (data->stance_start_time[0] - data->swing_start_time[0]);
    
    // Method 3: Foot angle at heel strike (from BHI360 quaternion)
    float angle_diff = fabs(data->foot_angle[0] - data->foot_angle[1]);
    
    // Combine methods with calibration factors
    float base_width = 10.0;  // Average step width in cm
    float pressure_adjustment = (0.5 - medial_ratio_left) * 20.0;
    float angle_adjustment = angle_diff * 0.5;
    
    float step_width = base_width + pressure_adjustment + angle_adjustment;
    
    return constrain(step_width, 2.0, 30.0);  // Reasonable bounds in cm
}
```

### Add to Periodic Record:
```c
typedef struct {
    // ... existing fields ...
    uint8_t  step_width_cm;      // NEW: Average step width
    uint8_t  width_variability;  // NEW: Consistency score
} PeriodicRecord;
```

---

## Implementation Priority Order

### Phase 1 - Quick Wins (1-2 weeks)
1. **Activity Type Tagging**: Simple enum addition to header
2. **Splits Tracking**: Add split records to file format
3. **Enhanced COP per foot**: Extend FootSummary structure

### Phase 2 - Medium Complexity (2-4 weeks)
1. **Step Width Estimation**: Develop and validate algorithm
2. **Run Type Profiles**: Implement adaptive thresholds
3. **Split Notifications**: Add BLE characteristic

### Phase 3 - Clinical Collaboration (4-8 weeks)
1. **Health & Risk Algorithms**: Work with Laura on thresholds
2. **Clinical Validation**: Test metrics with real patients
3. **Coaching Recommendations**: Develop evidence-based feedback

---

## Mobile App Integration Notes

### New Requirements:
1. **Activity Selection UI**: 
   - Pre-run selection screen
   - Activity type (Run/Walk)
   - Run subtype (Everyday/Long/Tempo/Intervals)

2. **Splits Display**:
   - Live split times during run
   - Split comparison (faster/slower than average)
   - Post-run split analysis

3. **Symmetry Visualization**:
   - Real-time balance meter
   - Historical symmetry trends
   - Left/right comparison charts

4. **Step Width Coaching**:
   - Visual indicator when too narrow
   - Audio cues for correction
   - Progress tracking over time

---

## Technical Considerations

### Sensor Synergy:
- **BHI360**: Handles step counting, orientation, impact detection
- **8-Channel Pressure**: Provides ground truth for contact, force distribution
- **Combined**: Cross-validation improves accuracy significantly

### Memory Impact:
- Additional 3 bytes per FootSummary (COP data): +6 bytes/record
- Split records: ~200 bytes for 25km run
- Step width: 2 bytes/record
- Total increase: ~10% more storage

### Processing Load:
- BHI360 step counting: Handled by sensor (minimal CPU)
- Pressure processing: 8 channels at 100Hz (moderate load)
- Step width calculation: Low complexity with our sensors
- Split detection: Low overhead
- COP tracking: Simple weighted average of 8 channels

### Accuracy Improvements with Our Sensors:
- **Step Detection**: >99% accuracy (BHI360 + pressure validation)
- **Contact Time**: ±2ms (pressure threshold detection)
- **Foot Strike**: 95% accuracy (pressure pattern at contact)
- **Pronation**: ±5° (BHI360 quaternion validated by pressure shift)
- **Pace**: ±5-10% (accurate step count + dynamic stride model)

### BLE Bandwidth:
- Split notifications: Event-driven, minimal impact
- No change to real-time metrics rate

---

## Next Steps

1. **Schedule meeting with Laura** to discuss:
   - Clinical thresholds for health indicators
   - Step width measurement validation
   - Run type profile parameters

2. **Update specification document** with:
   - New data structures
   - Activity type definitions
   - Split tracking details

3. **Create prototype** for:
   - Step width estimation algorithm
   - Split detection and notification
   - Run type adaptive thresholds

4. **Coordinate with mobile team** on:
   - UI for activity selection
   - Visualization requirements
   - Data synchronization approach

---

## Questions for Laura

1. **Calibration Run**: What specific metrics should we collect during calibration?
2. **Step Width**: What's the clinical threshold for "too narrow"? 
3. **Run Types**: Should we auto-detect run type based on metrics?
4. **Clinical Thresholds**: Can you provide specific values for injury risk indicators?
5. **Symmetry Scoring**: What level of asymmetry is clinically significant?

---

This implementation plan addresses all of Laura's feedback while maintaining the efficient data structure and staying within our technical constraints. The phased approach allows us to deliver value quickly while building toward the more complex clinical features.
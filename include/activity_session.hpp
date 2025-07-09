/**
 * @file activity_session.hpp
 * @brief Activity session data structures and processing
 * @version 1.0
 * @date 2025
 *
 * @copyright Botz Innovation 2025
 */

#ifndef APP_INCLUDE_ACTIVITY_SESSION_HEADER_
#define APP_INCLUDE_ACTIVITY_SESSION_HEADER_

#include <stdint.h>
#include <zephyr/kernel.h>

// Activity types
enum ActivityType {
    ACTIVITY_TYPE_RUNNING = 0,
    ACTIVITY_TYPE_WALKING = 1,
    ACTIVITY_TYPE_TRAINING = 2,
    ACTIVITY_TYPE_RECOVERY = 3,
    ACTIVITY_TYPE_CUSTOM = 4
};

// Running sub-types
enum RunningSubType {
    RUN_TYPE_EVERYDAY = 0,
    RUN_TYPE_LONG = 1,
    RUN_TYPE_TEMPO = 2,
    RUN_TYPE_INTERVALS = 3,
    RUN_TYPE_CALIBRATION = 4
};

// GPS modes
enum GPSMode {
    GPS_MODE_OFF = 0,
    GPS_MODE_CALIBRATION = 1,
    GPS_MODE_PRECISE = 2,
    GPS_MODE_RACE = 3
};

// Foot strike patterns
enum FootStrikePattern {
    STRIKE_HEEL = 0,
    STRIKE_MIDFOOT = 1,
    STRIKE_FOREFOOT = 2
};

// Alert types
enum AlertType {
    ALERT_POOR_FORM = 0,
    ALERT_HIGH_ASYMMETRY = 1,
    ALERT_OVERSTRIDING = 2,
    ALERT_HIGH_IMPACT = 3,
    ALERT_FATIGUE = 4,
    ALERT_UNUSUAL_PATTERN = 5
};

// Packed structures for efficient storage
#pragma pack(push, 1)

// Session header - written once at start (32 bytes)
typedef struct {
    uint32_t session_id;              // Unique session identifier
    uint32_t start_timestamp;         // Unix epoch time
    uint8_t  activity_type;           // RUNNING, WALKING, etc.
    uint8_t  activity_subtype;        // 0=Everyday, 1=Long, 2=Tempo, 3=Intervals, 4=Calibration
    uint8_t  firmware_version[3];     // Major.Minor.Patch
    uint16_t user_weight_kg;          // ×10 for 0.1kg precision
    uint16_t user_height_cm;          
    uint8_t  user_age;
    uint8_t  user_gender;             // 0=M, 1=F, 2=Other
    uint8_t  left_battery_pct;        // At start
    uint8_t  right_battery_pct;       // At start
    uint16_t calibration_id;          // Reference to calibration data
    uint8_t  gps_mode;                // GPS mode selected for session
    uint8_t  reserved[6];             // Future use
} SessionHeader;

// Per-foot summary data (12 bytes)
typedef struct {
    uint16_t peak_force;              // Normalized units
    uint8_t  heel_pct;                // 0-100
    uint8_t  midfoot_pct;             // 0-100
    uint8_t  forefoot_pct;            // 0-100
    int8_t   pronation_angle;         // -45 to +45 degrees
    uint16_t loading_rate;            // Normalized
    uint8_t  strike_pattern;          // 0=heel, 1=mid, 2=fore
    uint8_t  push_power;              // Normalized 0-255
    uint8_t  step_count;              // Steps in this period
    uint8_t  quality_score;           // 0-100
} FootSummary;

// Periodic record - logged every 1-2 seconds (48 bytes)
typedef struct {
    uint16_t delta_time_ms;           // Time since last record
    
    // Basic metrics (both feet averaged where applicable)
    uint16_t cadence_x2;              // Steps/min × 2 (0.5 precision)
    uint16_t avg_contact_time_ms;     
    uint16_t avg_flight_time_ms;
    uint16_t distance_delta_cm;       // Distance since last record
    
    // Per-foot summary
    FootSummary left_foot;            // 12 bytes
    FootSummary right_foot;           // 12 bytes
    
    // Composite metrics
    int8_t   balance_lr;              // -100 to +100
    uint8_t  form_score;              // 0-100
    uint8_t  efficiency_score;        // 0-100
    uint8_t  fatigue_level;           // 0-100
    
    // Flags for events
    uint8_t  event_flags;             // Bit flags for events
    uint8_t  reserved[3];
} PeriodicRecord;

// Session summary - written once at end (40 bytes)
typedef struct {
    uint32_t end_timestamp;
    uint32_t total_duration_sec;
    uint32_t total_steps;
    uint32_t total_distance_m;
    uint16_t avg_pace_sec_per_km;
    uint16_t calories_burned;
    
    // Performance summary
    uint8_t  overall_form_score;      // 0-100
    uint8_t  max_fatigue_reached;     // 0-100
    uint8_t  injury_risk_score;       // 0-100
    uint8_t  consistency_score;       // 0-100
    
    // Key statistics
    uint16_t avg_cadence;
    uint16_t avg_contact_time_ms;
    int8_t   avg_balance;             // -100 to +100
    uint8_t  primary_strike_pattern;  // Most common
    
    // Alerts summary
    uint8_t  alert_counts[8];         // Count per alert type
    
    uint32_t crc32;                   // File integrity check
} SessionSummary;

// Real-time metrics for BLE transmission (20 bytes)
typedef struct {
    uint16_t delta_time_ms;           // Time since last packet
    uint16_t cadence_x2;              // Steps/min × 2
    uint16_t pace_sec_per_km;         // Current pace
    uint16_t distance_m;              // Total distance
    uint16_t contact_time_ms;         // Average both feet
    uint8_t  form_score;              // 0-100
    int8_t   balance_lr;              // -100 to +100
    uint8_t  efficiency;              // 0-100
    uint8_t  fatigue_level;           // 0-100
    uint8_t  foot_strike;             // 0=heel, 1=mid, 2=fore
    uint8_t  flags;                   // Status flags
    uint32_t step_count;              // Total steps
} RealtimeMetricsPacket;

// Alert notification packet (20 bytes)
typedef struct {
    uint16_t delta_time_ms;           // Time since last packet
    uint8_t  alert_type;              // See AlertType enum
    uint8_t  severity;                // 1-10
    uint16_t current_value;           // Metric that triggered
    uint16_t threshold_value;         // Threshold exceeded
    uint8_t  affected_foot;           // 0=both, 1=left, 2=right
    uint8_t  recommendation;          // Coaching tip code
    uint8_t  duration_sec;            // How long detected
    uint8_t  reserved[9];             // Padding to 20 bytes
} AlertPacket;

// GPS update command from mobile app (20 bytes)
typedef struct {
    uint8_t  opcode;                  // = 0x10
    uint32_t timestamp;               // Unix time
    int32_t  latitude_e7;             // Latitude × 10^7
    int32_t  longitude_e7;            // Longitude × 10^7
    uint16_t speed_cms;               // Speed in cm/s
    uint16_t distance_m;              // Distance since last update
    uint8_t  accuracy_m;              // GPS accuracy
    int16_t  elevation_change_m;      // Elevation change
} GPSUpdateCommand;

#pragma pack(pop)

// Processing state for activity session
struct ActivitySessionState {
    // Session info
    bool session_active;
    SessionHeader header;
    uint32_t session_start_time;
    uint32_t last_record_time;
    uint32_t last_ble_update_time;
    
    // Accumulated metrics
    uint32_t total_steps_left;
    uint32_t total_steps_right;
    uint32_t total_distance_cm;
    
    // Current step data
    uint32_t left_contact_start;
    uint32_t right_contact_start;
    bool left_in_contact;
    bool right_in_contact;
    
    // Rolling buffers for calculations
    float contact_time_buffer[10];
    float flight_time_buffer[10];
    int buffer_index;
    
    // Baseline for fatigue detection
    float baseline_contact_time;
    float baseline_form_score;
    bool baseline_established;
    
    // GPS calibration
    float stride_correction_factor;
    uint32_t last_gps_update_time;
    uint32_t distance_at_last_gps;    // Distance at last GPS update (cm)
    int32_t total_elevation_gain_cm;  // Total elevation gain
};

// Function declarations
void activity_session_init(void);
void activity_session_start(const SessionHeader* header);
void activity_session_stop(void);
void activity_session_process_pressure(uint8_t foot, float pressure_values[8], uint32_t timestamp);
void activity_session_process_imu(uint8_t foot, float quaternion[4], float accel[3], uint32_t timestamp);
void activity_session_get_realtime_metrics(RealtimeMetricsPacket* packet);
void activity_session_process_gps_update(const GPSUpdateCommand* gps_data);

#endif // APP_INCLUDE_ACTIVITY_SESSION_HEADER_
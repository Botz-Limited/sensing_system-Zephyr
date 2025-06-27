# Step Count Implementation - Final Summary

## To Recap:

### BLE Characteristics (Always Active):

1. **Individual Foot Steps** (Global counts):
   - Right Foot (Primary device) - UUID: `0c372eb3-...`
   - Left Foot (Secondary device) - UUID: `0c372eb3-...` (via D2D)
   - Always counting from device boot
   - Continue counting regardless of activity state

2. **Total Steps** (Global count):
   - UUID: `0c372ec4-...`
   - Sum of both feet global counts
   - Always active and updating

3. **Activity Step Count** (Activity-specific):
   - UUID: `0c372ec5-...`
   - Sum of both feet activity counts
   - **Shows 0 when no activity is active**
   - **Resets to 0 at each activity start**
   - Only counts during active sessions

### Control Service (Commands from Mobile):

**Control Service UUID**: `4fd5b67f-9d89-4061-92aa-319ca786baae`

**Characteristics**:
- **Start Activity**: UUID `4fd5b684-9d89-4061-92aa-319ca786baae` (Write, value = 1)
- **Stop Activity**: UUID `4fd5b685-9d89-4061-92aa-319ca786baae` (Write, value = 1)

### Control Flow:

**Activity Start Command Flow:**
```
Mobile Phone → Control Service (BLE Write) → Start Activity Event → Motion Sensor
                        ↓ (Primary only)
                   D2D TX to Secondary → Secondary Motion Sensor
```

**Activity Stop Command Flow:**
```
Mobile Phone → Control Service (BLE Write) → Stop Activity Event → Motion Sensor
                        ↓ (Primary only)
                   D2D TX to Secondary → Secondary Motion Sensor
```

### Key Behaviors:

1. **When No Activity is Active**:
   - Global counts continue incrementing
   - Activity step count characteristic shows 0
   - No activity step notifications sent
   - Logging flag is false

2. **Activity Start** (triggered by mobile phone command):
   - Control Service receives write command (value = 1)
   - Events submitted to foot sensor and motion sensor modules
   - Activity step counts reset to 0 for both feet
   - Baseline global count captured internally
   - Activity logging flag set to true
   - BLE notification sent with 0 activity steps
   - Command forwarded to secondary device via D2D

3. **During Activity**:
   - Global counts continue incrementing normally
   - Activity counts = current global - baseline (per foot)
   - Activity count notifications sent periodically
   - Both individual and total activity counts updated
   - Logging active for data files

4. **Activity Stop** (triggered by mobile phone command):
   - Control Service receives write command (value = 1)
   - Events submitted to foot sensor and motion sensor modules
   - Activity counts freeze at final value
   - Global counts continue incrementing
   - Activity logging flag cleared (set to false)
   - Last activity count remains visible until next activity
   - Command forwarded to secondary device via D2D

### Log Files:

1. **BHI360 Log File**:
   - Saves global step count in each record
   - Continuous tracking throughout device operation

2. **Activity Log File**:
   - Saves activity-specific step counts only
   - Separate left and right foot counts preserved
   - Only records during active sessions

### Data Structure (All Characteristics):
```
4 bytes: step_count (uint32)
4 bytes: activity_duration_s (deprecated, always 0)
```

### Example Scenario:

```
Time    Event                   Global(R/L)    Total    Activity
----    -----                   -----------    -----    --------
10:00   Device boot            0/0            0        0
10:30   Walking around         1000/950       1950     0
11:00   Start Activity         1500/1450      2950     0 (reset)
11:30   During Activity        3000/2900      5900     1500/1450 = 2950
12:00   Stop Activity          3500/3400      6900     2000/1950 = 3950
12:30   Walking around         4000/3850      7850     0 (no activity)
13:00   Start New Activity     4200/4050      8250     0 (reset)
```

### Implementation Notes:

- All step count characteristics provide count only (no duration)
- Time-based metrics should be calculated by the mobile app
- Activity step count is always the sum of both feet during activity
- Secondary device sends its activity steps to primary for aggregation
- The system maintains both global and activity counts independently
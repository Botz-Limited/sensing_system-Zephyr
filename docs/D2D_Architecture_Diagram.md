# D2D Communication Architecture

## System Overview

```
┌─────────────────┐
│   Mobile Phone  │
│                 │
└────────┬────────┘
         │ BLE
         │ (Control & Info Services)
         │
┌────────▼────────┐                    ┌─────────────────┐
│ Primary Device  │◄───────D2D────────►│Secondary Device │
│   (Right Foot)  │                    │  (Left Foot)    │
│                 │                    │                 │
│ - SensingGR     │                    │ - SensingGL     │
│ - Advertises    │                    │ - Scans         │
│ - Phone facing  │                    │ - No phone conn │
└─────────────────┘                    └─────────────────┘
```

## Service Architecture

### Primary Device Services
```
┌─────────────────────────────────────┐
│         PRIMARY DEVICE              │
├─────────────────────────────────────┤
│ Phone-Facing Services:              │
│ ┌─────────────────┐                 │
│ │ Control Service │ ◄── From Phone  │
│ └────────┬────────┘                 │
│          │ Forwards                 │
│          ▼                          │
│ ┌─────────────────┐                 │
│ │   D2D TX Module │ ──► To Secondary│
│ └─────────────────┘                 │
│                                     │
│ ┌─────────────────┐                 │
│ │   D2D RX Module │ ◄── From Sec.  │
│ └────────┬────────┘                 │
│          │ Forwards                 │
│          ▼                          │
│ ┌──────────────────┐                │
│ │ Information Svc  │ ──► To Phone   │
│ └──────────────────┘                │
└────��────────────────────────────────┘
```

### Secondary Device Services
```
┌─────────────────────────────────────┐
│        SECONDARY DEVICE             │
├─────────────────────────────────────┤
│ D2D Services Only:                  │
│ ┌─────────────────┐                 │
│ │   D2D RX Module │ ◄── From Primary│
│ └────────┬────────┘                 │
│          │ Processes                │
│          ▼                          │
│ ┌─────────────────┐                 │
│ │  Local Handlers │                 │
│ └─────────────────┘                 │
│                                     │
│ ┌─────────────────┐                 │
│ │  Sensor Data    │                 │
│ └────────┬────────┘                 │
│          │ Sends                    │
│          ▼                          │
│ ┌─────────────────┐                 │
│ │   D2D TX Module │ ──► To Primary  │
│ └─────────────────┘                 │
└─────────────────────────────────────┘
```

## Command Flow Example

### Set Time Command
```
Phone ──[SetTime:1234567890]──► Primary
                                   │
                                   ├── Sets local RTC
                                   │
                                   └──[Forward:1234567890]──► Secondary
                                                                  │
                                                                  └── Sets local RTC
```

### Delete Log Command
```
Phone ──[DeleteFootLog:ID=5]──► Primary
                                   │
                                   ├── Deletes local log
                                   │
                                   └──[Forward:ID=5]──► Secondary
                                                           │
                                                           └── Deletes local log
```

### Start Activity Command
```
Phone ──[StartActivity:1]──► Primary
                               │
                               ├── Triggers local events
                               │   ├── foot_sensor_start_activity_event
                               │   └── motion_sensor_start_activity_event
                               │
                               └──[Forward:1]──► Secondary
                                                    │
                                                    └── Triggers local events
                                                        ├── foot_sensor_start_activity_event
                                                        └── motion_sensor_start_activity_event
```

## Data Flow Example

### Foot Sensor Data
```
Secondary                          Primary                           Phone
    │                                 │                                │
    ├── Collects sensor data          │                                │
    │                                 │                                │
    └──[FootSamples]──────────────►   │                                │
                                      ├── Receives via D2D RX          │
                                      │                                │
                                      └──[Notify:FootSamples]───────►  │
                                                                       │
                                                                   Displays
```

### BHI360 Motion Data
```
Secondary                          Primary                           Phone
    │                                 │                                │
    ├── Collects motion data          │                                │
    │                                 │                                │
    └──[BHI360Data]───────────────►   │                                │
                                      ├── Receives via D2D RX          │
                                      │                                │
                                      └──[Notify:BHI360Data]────────►  │
                                                                       │
                                                                   Displays
```

## Connection States

### Primary Device
```
State: IDLE
    │
    ├──► Advertising "SensingGR"
    │
    ├──► Phone connects ──► State: PHONE_CONNECTED
    │
    └──► Secondary connects ──► State: D2D_CONNECTED
                                        │
                                        └──► State: FULLY_CONNECTED
```

### Secondary Device
```
State: IDLE
    │
    └──► Scanning for "SensingGR"
            │
            └──► Found & Connected ──► State: D2D_CONNECTED
                                              │
                                              └──► Ready to send/receive
```

## Key Design Principles

1. **Unidirectional Phone Communication**
   - Only Primary ↔ Phone
   - Never Secondary ↔ Phone

2. **Bidirectional D2D Communication**
   - Primary ↔ Secondary for both commands and data

3. **Service Isolation**
   - Phone services only on Primary
   - D2D services on both devices

4. **Clear Role Definition**
   - Compile-time role selection
   - No runtime role switching
   - Clear naming convention (GR/GL)
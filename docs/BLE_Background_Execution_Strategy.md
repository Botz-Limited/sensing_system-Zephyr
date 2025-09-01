# BLE Background Execution Strategy for Mobile Apps

## Overview

This document outlines the strategy for maintaining BLE connectivity and data collection when the mobile app is in the background during long fitness activities. It addresses platform-specific limitations and provides implementation guidelines for both iOS and Android.

## Platform Limitations

### iOS Background Execution Limits

1. **Background Task Time**: ~30 seconds after app enters background
2. **Background Modes**: Requires specific capabilities
3. **State Preservation**: Apps can be terminated at any time
4. **BLE Specific**: Core Bluetooth background mode allows extended operation

### Android Background Restrictions

1. **Doze Mode**: Restricts network and CPU after ~1 hour of inactivity
2. **App Standby**: Limits background activity for unused apps
3. **Battery Optimization**: Can kill background services
4. **Foreground Service**: Required for continuous operation

## Recommended Architecture

### Service Types by Platform

#### iOS Strategy
```
┌─────────────────────────────────────────┐
│           iOS Background Modes          │
├─────────────────────────────────────────┤
│ 1. Core Bluetooth Background Mode       │
│    - Continuous BLE communication       │
│    - State preservation/restoration     │
│                                         │
│ 2. Background Processing (if needed)    │
│    - Periodic data uploads              │
│    - File management                    │
└─────────────────────────────────────────┘
```

#### Android Strategy
```
┌─────────────────────────────────────────┐
│        Android Service Architecture      │
├─────────────────────────────────────────┤
│ 1. Foreground Service (Recommended)     │
│    - Persistent notification            │
│    - Continuous BLE operation           │
│    - Exempt from Doze mode             │
│                                         │
│ 2. WorkManager (For periodic tasks)     │
│    - Data sync                          │
│    - Log file uploads                   │
└─────────────────────────────────────────┘
```

## Implementation Guidelines

### iOS Implementation

#### 1. Enable Core Bluetooth Background Mode

**Info.plist Configuration:**
```xml
<key>UIBackgroundModes</key>
<array>
    <string>bluetooth-central</string>
    <string>bluetooth-peripheral</string>
</array>
```

#### 2. State Preservation and Restoration

```swift
class BLEManager: NSObject {
    // State preservation keys
    private let restorationKey = "com.company.app.blemanager"
    
    func application(_ application: UIApplication, 
                    willRestoreStateWith coder: NSCoder) {
        // Restore BLE state
        let manager = CBCentralManager(
            delegate: self,
            queue: nil,
            options: [
                CBCentralManagerOptionRestoreIdentifierKey: restorationKey,
                CBCentralManagerOptionShowPowerAlertKey: true
            ]
        )
    }
    
    func centralManager(_ central: CBCentralManager, 
                       willRestoreState dict: [String : Any]) {
        // Restore connections and subscriptions
        if let peripherals = dict[CBCentralManagerRestoredStatePeripheralsKey] 
            as? [CBPeripheral] {
            // Reconnect to devices
        }
    }
}
```

#### 3. Background Data Handling

```swift
class SensorDataManager {
    private var backgroundTask: UIBackgroundTaskIdentifier = .invalid
    
    func handleBackgroundData(_ data: Data) {
        // Start background task for processing
        backgroundTask = UIApplication.shared.beginBackgroundTask {
            self.endBackgroundTask()
        }
        
        // Process data efficiently
        processIncomingData(data)
        
        // End task when done
        endBackgroundTask()
    }
    
    private func processIncomingData(_ data: Data) {
        // Buffer data in memory or local storage
        // Avoid heavy processing in background
    }
}
```

### Android Implementation

#### 1. Foreground Service Setup

**AndroidManifest.xml:**
```xml
<uses-permission android:name="android.permission.FOREGROUND_SERVICE" />
<uses-permission android:name="android.permission.FOREGROUND_SERVICE_CONNECTED_DEVICE" />
<uses-permission android:name="android.permission.WAKE_LOCK" />

<service
    android:name=".BLESensorService"
    android:foregroundServiceType="connectedDevice"
    android:exported="false" />
```

**Service Implementation:**
```kotlin
class BLESensorService : Service() {
    private val NOTIFICATION_ID = 1001
    private lateinit var wakeLock: PowerManager.WakeLock
    
    override fun onCreate() {
        super.onCreate()
        
        // Acquire wake lock for continuous operation
        val powerManager = getSystemService(Context.POWER_SERVICE) as PowerManager
        wakeLock = powerManager.newWakeLock(
            PowerManager.PARTIAL_WAKE_LOCK,
            "SensingApp:BLESensorWakeLock"
        )
        wakeLock.acquire()
        
        // Start as foreground service
        startForeground(NOTIFICATION_ID, createNotification())
    }
    
    private fun createNotification(): Notification {
        val channel = NotificationChannel(
            "ble_sensor_channel",
            "Sensor Data Collection",
            NotificationManager.IMPORTANCE_LOW
        ).apply {
            description = "Collecting fitness data from sensors"
            setShowBadge(false)
        }
        
        val notificationManager = getSystemService(NotificationManager::class.java)
        notificationManager.createNotificationChannel(channel)
        
        return NotificationCompat.Builder(this, "ble_sensor_channel")
            .setContentTitle("Sensing Active")
            .setContentText("Recording activity data")
            .setSmallIcon(R.drawable.ic_sensor)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .setOngoing(true)
            .build()
    }
    
    override fun onDestroy() {
        super.onDestroy()
        if (wakeLock.isHeld) {
            wakeLock.release()
        }
    }
}
```

#### 2. Doze Mode Exemption

```kotlin
class PowerManagementHelper(private val context: Context) {
    
    fun requestBatteryOptimizationExemption() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            val intent = Intent().apply {
                action = Settings.ACTION_REQUEST_IGNORE_BATTERY_OPTIMIZATIONS
                data = Uri.parse("package:${context.packageName}")
            }
            context.startActivity(intent)
        }
    }
    
    fun isIgnoringBatteryOptimizations(): Boolean {
        val powerManager = context.getSystemService(Context.POWER_SERVICE) as PowerManager
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            powerManager.isIgnoringBatteryOptimizations(context.packageName)
        } else {
            true
        }
    }
}
```

## Data Collection Strategies

### Real-time Mode (15-30ms interval)

#### Foreground Operation
- Full data rate collection
- Immediate processing and display
- No restrictions

#### Background Operation

**iOS:**
- Continue receiving at full rate
- Buffer data in memory
- Batch process when returning to foreground

**Android (Foreground Service):**
- Maintain full data rate
- Process data normally
- Update notification with activity status

### Power-Saving Strategies

#### 1. Adaptive Sampling
```
Foreground: 15-30ms (full rate)
Background Active: 50-100ms (reduced rate)
Background Idle: 200-500ms (monitoring only)
```

#### 2. Data Buffering
```
┌─────────────────────────────────────────┐
│          Circular Buffer Strategy        │
├─────────────────────────────────────────┤
│ Foreground:                             │
│   - Process immediately                 │
│   - Display real-time                   │
│                                         │
│ Background:                             │
│   - Buffer up to 5 minutes              │
│   - Compress if needed                  │
│   - Process on foreground return        │
└─────────────────────────────────────────┘
```

#### 3. Connection Parameters

**Background BLE Parameters:**
```
Connection Interval: 30-50ms (vs 15ms foreground)
Slave Latency: 4 (allow 4 missed intervals)
Supervision Timeout: 2 seconds
```

## State Management

### Connection State Machine

```
┌──────────────┐     App Background    ┌──────────────────┐
│  Foreground  │ ──────────────────> │    Background     │
│   Active     │                      │  (Keep Connected) │
└──────────────┘                      └──────────────────┘
       ↑                                        │
       │                                        │ > 5 min idle
       │                                        ↓
       │                              ┌──────────────────┐
       │     User Returns             │   Background     │
       └──────────────────────────────│   (Suspended)    │
                                      └──────────────────┘
```

### Data Persistence Strategy

```swift
// iOS
class DataPersistenceManager {
    func saveBackgroundData(_ samples: [SensorSample]) {
        // Use Core Data or SQLite for persistence
        // Avoid UserDefaults for large data
    }
}
```

```kotlin
// Android
class DataPersistenceManager(context: Context) {
    private val database = Room.databaseBuilder(
        context,
        SensorDatabase::class.java,
        "sensor_data"
    ).build()
    
    suspend fun saveBackgroundData(samples: List<SensorSample>) {
        database.sensorDao().insertAll(samples)
    }
}
```

## Best Practices

### 1. User Communication

**Inform users about:**
- Need for background permissions
- Battery optimization exemptions
- Impact on battery life
- Data collection status

### 2. Battery Life Optimization

- Reduce sampling rate in background
- Batch BLE operations
- Use connection parameter updates
- Implement smart buffering

### 3. Error Handling

```kotlin
class BLEConnectionManager {
    private var reconnectAttempts = 0
    private val MAX_RECONNECT_ATTEMPTS = 5
    
    fun handleDisconnection() {
        when {
            isInForeground() -> reconnectImmediately()
            reconnectAttempts < MAX_RECONNECT_ATTEMPTS -> {
                scheduleReconnect(backoffDelay())
                reconnectAttempts++
            }
            else -> notifyUserAndStop()
        }
    }
    
    private fun backoffDelay(): Long {
        return min(30000L, 1000L * (1 shl reconnectAttempts))
    }
}
```

### 4. Testing Scenarios

**iOS Testing:**
- Background fetch simulation
- State restoration after termination
- Low memory conditions
- Extended background operation

**Android Testing:**
- Doze mode activation
- App standby buckets
- Battery saver mode
- Force-stop recovery

## Compliance and User Experience

### Permission Requests

**iOS:**
```swift
func requestBackgroundPermissions() {
    // Bluetooth permissions
    CBCentralManager.authorization
    
    // Motion permissions if needed
    CMMotionActivityManager.authorizationStatus()
}
```

**Android:**
```kotlin
fun requestBackgroundPermissions() {
    val permissions = arrayOf(
        Manifest.permission.BLUETOOTH_SCAN,
        Manifest.permission.BLUETOOTH_CONNECT,
        Manifest.permission.ACCESS_FINE_LOCATION,
        Manifest.permission.FOREGROUND_SERVICE
    )
    
    ActivityCompat.requestPermissions(activity, permissions, REQUEST_CODE)
}
```

### User Settings

Provide in-app settings for:
- Background data collection toggle
- Sampling rate preferences
- Battery optimization level
- Notification preferences

## Implementation Status

### Firmware Implementation (Completed)

The firmware now includes full support for mobile background execution:

1. **Connection Parameter Control Characteristic**
   - UUID: `4fd5b68b-9d89-4061-92aa-319ca786baae`
   - Located in Control Service
   - Write 1 byte: 0=FOREGROUND, 1=BACKGROUND, 2=BACKGROUND_IDLE
   - Read returns current profile

2. **Connection Profiles Implemented**
   ```
   FOREGROUND (0):
   - Interval: 15-30ms
   - Latency: 0
   - Timeout: 4s
   
   BACKGROUND (1):
   - Interval: 50-100ms
   - Latency: 4
   - Timeout: 6s
   
   BACKGROUND_IDLE (2):
   - Interval: 200-500ms
   - Latency: 10
   - Timeout: 10s
   ```

3. **Automatic Adjustments**
   - Sensor sampling rates adapt to connection profile
   - Data aggregation enabled in background modes
   - Connection parameters update dynamically

### Mobile App Integration

To use the background execution support:

```swift
// iOS Example
func setBackgroundMode() {
    // Write 1 to conn param control characteristic
    let data = Data([0x01]) // BACKGROUND mode
    peripheral.writeValue(data, for: connParamCharacteristic, type: .withResponse)
}

func setForegroundMode() {
    // Write 0 to conn param control characteristic
    let data = Data([0x00]) // FOREGROUND mode
    peripheral.writeValue(data, for: connParamCharacteristic, type: .withResponse)
}
```

```kotlin
// Android Example
fun setBackgroundMode() {
    val data = byteArrayOf(0x01) // BACKGROUND mode
    gatt.writeCharacteristic(connParamCharacteristic, data)
}

fun setForegroundMode() {
    val data = byteArrayOf(0x00) // FOREGROUND mode
    gatt.writeCharacteristic(connParamCharacteristic, data)
}
```

## Summary

For reliable background BLE operation during fitness activities:

1. **iOS**: Use Core Bluetooth background mode with state preservation
2. **Android**: Implement a foreground service with persistent notification
3. **Both**: Request appropriate connection profile via the control characteristic
4. **Firmware**: Automatically adapts sampling rates and connection parameters
5. **User Experience**: Clear communication about battery impact and permissions

This strategy ensures continuous data collection while respecting platform limitations and user battery life. The firmware implementation is complete and ready for mobile app integration.

## Revision History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 2025-01-XX | Initial background execution strategy | Botz Innovation |

---

*This document is part of the Botz Innovation Sensing Firmware technical documentation.*
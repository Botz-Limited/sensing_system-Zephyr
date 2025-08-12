# Category 1 Metrics Implementation Specification

**Version:** 1.0  
**Date:** August 2025  
**Scope:** Category 1 metrics from colleague's proposal  
**Service:** Activity Metrics Service  
**Service UUID:** `4fd5b690-9d89-4061-92aa-319ca786baae`

---

## 1. Characteristics Overview

| Metric | UUID | Properties | Data Type | Format | Range | Update Rate |
|--------|------|------------|-----------|--------|-------|-------------|
| **Cadence SPM** | `4fd5b691-9d89-4061-92aa-319ca786baae` | Read, Notify | uint16_t | Little Endian | 0-300 spm | 1Hz |
| **Stride Duration Asymmetry** | `4fd5b6b4-9d89-4061-92aa-319ca786baae` | Read, Notify | uint8_t | Single Byte | 0-100% | 1Hz |
| **Stride Length Asymmetry** | `4fd5b6b6-9d89-4061-92aa-319ca786baae` | Read, Notify | uint8_t | Single Byte | 0-100% | 1Hz |
| **Ground Contact Time Asymmetry** | `4fd5b6a0-9d89-4061-92aa-319ca786baae` | Read, Notify | uint8_t | Single Byte | 0-100% | 1Hz |
| **Ground Contact Time MS** | `4fd5b69c-9d89-4061-92aa-319ca786baae` | Read, Notify | uint16_t | Little Endian | 0-1000 ms | 1Hz |
| **Flight Time MS** | `4fd5b69d-9d89-4061-92aa-319ca786baae` | Read, Notify | uint16_t | Little Endian | 0-500 ms | 1Hz |

---

## 2. Characteristic Details

### 2.1 Cadence SPM

| Field | Value |
|-------|-------|
| **UUID** | `4fd5b691-9d89-4061-92aa-319ca786baae` |
| **Properties** | Read, Notify |
| **Permissions** | Read Encrypted |
| **Data Type** | uint16_t |
| **Data Length** | 2 bytes |
| **Byte Order** | Little Endian |
| **Unit** | Steps per minute |
| **Valid Range** | 0-300 |
| **Update Frequency** | 1Hz |
| **Handler Functions** | `ams_cadence_spm_read()`, `ams_cadence_spm_ccc_cfg_changed()` |
| **BLE Attribute Index** | 2 |

#### Data Structure
```c
uint16_t cadence_spm_value;  // Steps per minute
```

#### Calculation
```c
// From realtime_metrics.cpp
cadence_tracker_update(&metrics_state.cadence_tracker, 
                      data->step_count, 
                      metrics_state.current_time_ms);
float current_cadence = cadence_tracker_get_current(&metrics_state.cadence_tracker);
metrics_state.current_metrics.cadence_spm = (uint16_t)current_cadence;
```

---

### 2.2 Stride Duration Asymmetry

| Field | Value |
|-------|-------|
| **UUID** | `4fd5b6b4-9d89-4061-92aa-319ca786baae` |
| **Properties** | Read, Notify |
| **Permissions** | Read Encrypted |
| **Data Type** | uint8_t |
| **Data Length** | 1 byte |
| **Unit** | Percentage |
| **Valid Range** | 0-100 |
| **Update Frequency** | 1Hz |
| **Handler Functions** | `ams_stride_duration_asym_read()`, `ams_stride_duration_asym_ccc_cfg_changed()` |
| **BLE Attribute Index** | 89 |

#### Data Structure
```c
uint8_t stride_duration_asym_value;  // Asymmetry percentage
```

#### Calculation
```c
// From realtime_metrics.cpp
metrics_state.current_metrics.stride_duration_asymmetry = 
    calculate_asymmetry_percentage(
        metrics_state.left_stride_duration,
        metrics_state.right_stride_duration
    );
```

---

### 2.3 Stride Length Asymmetry

| Field | Value |
|-------|-------|
| **UUID** | `4fd5b6b6-9d89-4061-92aa-319ca786baae` |
| **Properties** | Read, Notify |
| **Permissions** | Read Encrypted |
| **Data Type** | uint8_t |
| **Data Length** | 1 byte |
| **Unit** | Percentage |
| **Valid Range** | 0-100 |
| **Update Frequency** | 1Hz |
| **Handler Functions** | `ams_stride_length_asym_read()`, `ams_stride_length_asym_ccc_cfg_changed()` |
| **BLE Attribute Index** | 95 |

#### Data Structure
```c
uint8_t stride_length_asym_value;  // Asymmetry percentage
```

#### Calculation
```c
// From realtime_metrics.cpp
metrics_state.current_metrics.stride_length_asymmetry = 
    calculate_asymmetry_percentage(
        (uint16_t)(metrics_state.left_stride_length * 100),
        (uint16_t)(metrics_state.right_stride_length * 100)
    );
```

---

### 2.4 Ground Contact Time Asymmetry

| Field | Value |
|-------|-------|
| **UUID** | `4fd5b6a0-9d89-4061-92aa-319ca786baae` |
| **Properties** | Read, Notify |
| **Permissions** | Read Encrypted |
| **Data Type** | uint8_t |
| **Data Length** | 1 byte |
| **Unit** | Percentage |
| **Valid Range** | 0-100 |
| **Update Frequency** | 1Hz |
| **Handler Functions** | `ams_contact_time_asym_read()`, `ams_contact_time_asym_ccc_cfg_changed()` |
| **BLE Attribute Index** | 29 |

#### Data Structure
```c
uint8_t contact_time_asym_value;  // Asymmetry percentage
```

#### Calculation
```c
// From realtime_metrics.cpp
uint16_t avg_left_contact = metrics_state.left_contact_sum_ms / metrics_state.contact_count;
uint16_t avg_right_contact = metrics_state.right_contact_sum_ms / metrics_state.contact_count;

metrics_state.current_metrics.contact_time_asymmetry = 
    calculate_asymmetry_percentage(avg_left_contact, avg_right_contact);
```

---

### 2.5 Ground Contact Time MS

| Field | Value |
|-------|-------|
| **UUID** | `4fd5b69c-9d89-4061-92aa-319ca786baae` |
| **Properties** | Read, Notify |
| **Permissions** | Read Encrypted |
| **Data Type** | uint16_t |
| **Data Length** | 2 bytes |
| **Byte Order** | Little Endian |
| **Unit** | Milliseconds |
| **Valid Range** | 0-1000 |
| **Update Frequency** | 1Hz |
| **Handler Functions** | `ams_ground_contact_ms_read()`, `ams_ground_contact_ms_ccc_cfg_changed()` |
| **BLE Attribute Index** | 17 |

#### Data Structure
```c
uint16_t ground_contact_ms_value;  // Average ground contact time in milliseconds
```

#### Calculation
```c
// From realtime_metrics.cpp
metrics_state.current_metrics.ground_contact_ms = (avg_left_contact + avg_right_contact) / 2;
```

---

### 2.6 Flight Time MS

| Field | Value |
|-------|-------|
| **UUID** | `4fd5b69d-9d89-4061-92aa-319ca786baae` |
| **Properties** | Read, Notify |
| **Permissions** | Read Encrypted |
| **Data Type** | uint16_t |
| **Data Length** | 2 bytes |
| **Byte Order** | Little Endian |
| **Unit** | Milliseconds |
| **Valid Range** | 0-500 |
| **Update Frequency** | 1Hz |
| **Handler Functions** | `ams_flight_time_ms_read()`, `ams_flight_time_ms_ccc_cfg_changed()` |
| **BLE Attribute Index** | 20 |

#### Data Structure
```c
uint16_t flight_time_ms_value;  // Average flight time in milliseconds
```

#### Calculation
```c
// From realtime_metrics.cpp
metrics_state.current_metrics.flight_time_ms = data->true_flight_time_ms;
```

---

## 3. Common Functions

### 3.1 Asymmetry Calculation

```c
static uint8_t calculate_asymmetry_percentage(uint16_t left, uint16_t right) {
    if (left == 0 && right == 0) {
        return 0;
    }
    
    uint16_t max_val = (left > right) ? left : right;
    uint16_t min_val = (left < right) ? left : right;
    
    if (max_val == 0) {
        return 0;
    }
    
    return (uint8_t)(((max_val - min_val) * 100) / max_val);
}
```

---

## 4. BLE Service Definition

```c
BT_GATT_SERVICE_DEFINE(
    activity_metrics_service,
    BT_GATT_PRIMARY_SERVICE(&ACTIVITY_METRICS_SERVICE_UUID),
    
    // Cadence SPM - Index 1-3
    BT_GATT_CHARACTERISTIC(&cadence_spm_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ, ams_cadence_spm_read,
                          nullptr, static_cast<void *>(&cadence_spm_value)),
    BT_GATT_CCC(ams_cadence_spm_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Ground Contact MS - Index 16-18
    BT_GATT_CHARACTERISTIC(&ground_contact_ms_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          ams_ground_contact_ms_read, nullptr,
                          static_cast<void *>(&ground_contact_ms_value)),
    BT_GATT_CCC(ams_ground_contact_ms_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Flight Time MS - Index 19-21
    BT_GATT_CHARACTERISTIC(&flight_time_ms_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ, ams_flight_time_ms_read,
                          nullptr, static_cast<void *>(&flight_time_ms_value)),
    BT_GATT_CCC(ams_flight_time_ms_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Contact Time Asymmetry - Index 28-30
    BT_GATT_CHARACTERISTIC(&contact_time_asym_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          ams_contact_time_asym_read, nullptr,
                          static_cast<void *>(&contact_time_asym_value)),
    BT_GATT_CCC(ams_contact_time_asym_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Stride Duration Asymmetry - Index 88-90
    BT_GATT_CHARACTERISTIC(&stride_duration_asym_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          ams_stride_duration_asym_read, nullptr,
                          static_cast<void *>(&stride_duration_asym_value)),
    BT_GATT_CCC(ams_stride_duration_asym_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Stride Length Asymmetry - Index 94-96
    BT_GATT_CHARACTERISTIC(&stride_length_asym_uuid.uuid,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          ams_stride_length_asym_read, nullptr,
                          static_cast<void *>(&stride_length_asym_value)),
    BT_GATT_CCC(ams_stride_length_asym_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);
```

---

## 5. Implementation Files

| Component | File | Lines |
|-----------|------|-------|
| **Metric Calculations** | `src/realtime_metrics/realtime_metrics.cpp` | 337-494 |
| **BLE Service** | `src/bluetooth/activity_metrics_service.cpp` | 163-1830 |
| **Data Structures** | `src/realtime_metrics/realtime_metrics.h` | 38-75 |
| **Service Header** | `src/bluetooth/activity_metrics_service.h` | Full file |
| **Asymmetry Utils** | `src/realtime_metrics/realtime_metrics_utils.cpp` | 12-25 |

---

## 6. Update Function

```c
void ams_update_realtime_metrics(const realtime_metrics_t *metrics) {
    if (!metrics) {
        return;
    }
    
    // Update individual metric values
    cadence_spm_value = metrics->cadence_spm;
    ground_contact_ms_value = metrics->ground_contact_ms;
    flight_time_ms_value = metrics->flight_time_ms;
    contact_time_asym_value = metrics->contact_time_asymmetry;
    stride_duration_asym_value = metrics->stride_duration_asymmetry;
    stride_length_asym_value = metrics->stride_length_asymmetry;
    
    // Send notifications if enabled and connected
    if (current_conn) {
        if (cadence_spm_notify_enabled) {
            bt_gatt_notify(current_conn, &activity_metrics_service.attrs[2],
                          &cadence_spm_value, sizeof(cadence_spm_value));
        }
        if (ground_contact_ms_notify_enabled) {
            bt_gatt_notify(current_conn, &activity_metrics_service.attrs[17],
                          &ground_contact_ms_value, sizeof(ground_contact_ms_value));
        }
        if (flight_time_ms_notify_enabled) {
            bt_gatt_notify(current_conn, &activity_metrics_service.attrs[20],
                          &flight_time_ms_value, sizeof(flight_time_ms_value));
        }
        if (contact_time_asym_notify_enabled) {
            bt_gatt_notify(current_conn, &activity_metrics_service.attrs[29],
                          &contact_time_asym_value, sizeof(contact_time_asym_value));
        }
        if (stride_duration_asym_notify_enabled) {
            bt_gatt_notify(current_conn, &activity_metrics_service.attrs[89],
                          &stride_duration_asym_value, sizeof(stride_duration_asym_value));
        }
        if (stride_length_asym_notify_enabled) {
            bt_gatt_notify(current_conn, &activity_metrics_service.attrs[95],
                          &stride_length_asym_value, sizeof(stride_length_asym_value));
        }
    }
}
```

---

**End of Specification**
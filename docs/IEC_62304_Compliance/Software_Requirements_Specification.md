# Software Requirements Specification

**Version:** 1.0  
**Date:** July 2025  
**Status:** Draft  
**Classification:** IEC 62304 Class A

---

## Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | July 2025 | Team | Initial draft consolidating existing requirements |

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [System Requirements](#2-system-requirements)
3. [Functional Requirements](#3-functional-requirements)
4. [Performance Requirements](#4-performance-requirements)
5. [Interface Requirements](#5-interface-requirements)
6. [Data Requirements](#6-data-requirements)
7. [Safety Requirements](#7-safety-requirements)
8. [Security Requirements](#8-security-requirements)
9. [Usability Requirements](#9-usability-requirements)
10. [Regulatory Requirements](#10-regulatory-requirements)
11. [Requirements Traceability](#11-requirements-traceability)

---

## 1. Introduction

### 1.1 Purpose
This document specifies the software requirements for the dual-device biomechanical sensing system intended for clinical studies and sports performance analysis.

### 1.2 Scope
The software controls two synchronized devices that monitor foot pressure and motion, calculate biomechanical metrics, and transmit data to mobile applications.

### 1.3 Definitions
- **SHALL**: Mandatory requirement
- **SHOULD**: Recommended requirement
- **MAY**: Optional requirement

---

## 2. System Requirements

### 2.1 System Overview
The system SHALL consist of:
- SYS-001: Two synchronized sensing devices (primary and secondary)
- SYS-002: Mobile application interface via Bluetooth Low Energy
- SYS-003: Real-time biomechanical analysis capabilities
- SYS-004: Data logging for offline analysis

### 2.2 Operating Environment
- SYS-005: The system SHALL operate on nRF5340 dual-core processors
- SYS-006: The system SHALL use Zephyr RTOS version 3.5.0 or later
- SYS-007: The system SHALL support Bluetooth 5.0 or later

---

## 3. Functional Requirements

### 3.1 Sensor Data Acquisition

#### 3.1.1 Pressure Sensing
- FUNC-001: The system SHALL read 8 pressure sensors per device
- FUNC-002: The system SHALL sample pressure sensors at 100Hz ± 1%
- FUNC-003: The system SHALL detect ground contact with <10ms latency
- FUNC-004: The system SHALL calculate pressure distribution across foot regions

#### 3.1.2 Motion Sensing
- FUNC-005: The system SHALL read BHI360 IMU data at 50Hz
- FUNC-006: The system SHALL acquire quaternion orientation data
- FUNC-007: The system SHALL acquire linear acceleration data
- FUNC-008: The system SHALL track step count

### 3.2 Data Processing

#### 3.2.1 Real-time Metrics
- FUNC-009: The system SHALL calculate cadence in real-time
- FUNC-010: The system SHALL calculate pace from sensor data
- FUNC-011: The system SHALL detect foot strike patterns
- FUNC-012: The system SHALL calculate ground contact time
- FUNC-013: The system SHALL calculate flight time
- FUNC-014: The system SHALL detect gait asymmetry

#### 3.2.2 Activity Management
- FUNC-015: The system SHALL start/stop activity sessions on command
- FUNC-016: The system SHALL track session duration
- FUNC-017: The system SHALL generate session summaries
- FUNC-018: The system SHALL support multiple activity types
- FUNC-019: The system SHALL accept GPS updates from mobile applications
- FUNC-020: The system SHALL use GPS data to enhance distance/pace accuracy
- FUNC-021: The system SHALL operate correctly without GPS data

### 3.3 Communication

#### 3.3.1 Bluetooth Services
- FUNC-022: The system SHALL implement BLE GATT services
- FUNC-023: The system SHALL encrypt all BLE communications
- FUNC-024: The system SHALL support bonding with mobile devices
- FUNC-025: The system SHALL advertise as "BotzRightSh" (primary) or "BotzLeftSh" (secondary)

#### 3.3.2 Device-to-Device
- FUNC-026: Primary device SHALL connect to secondary device
- FUNC-027: Primary device SHALL aggregate data from both devices
- FUNC-028: Primary device SHALL synchronize time between devices
- FUNC-029: D2D connection SHALL auto-reconnect if lost

### 3.4 Data Storage

#### 3.4.1 Logging
- FUNC-030: The system SHALL log sensor data to flash storage
- FUNC-031: The system SHALL create separate files for each session
- FUNC-032: The system SHALL use protobuf format for logs
- FUNC-033: The system SHALL support file retrieval via BLE

#### 3.4.2 Configuration
- FUNC-034: The system SHALL store device configuration
- FUNC-035: The system SHALL store calibration data
- FUNC-036: The system SHALL persist settings across reboots

### 3.5 Firmware Updates
- FUNC-037: The system SHALL support over-the-air firmware updates
- FUNC-038: The system SHALL verify firmware integrity
- FUNC-039: The system SHALL support rollback on failed updates
- FUNC-040: Both devices SHALL update synchronously

---

## 4. Performance Requirements

### 4.1 Timing Requirements
- PERF-001: Sensor sampling jitter SHALL be <1ms
- PERF-002: BLE notification latency SHALL be <100ms
- PERF-003: D2D communication latency SHALL be <50ms
- PERF-004: System boot time SHALL be <5 seconds

### 4.2 Resource Requirements
- PERF-005: CPU usage SHALL not exceed 70%
- PERF-006: RAM usage SHALL not exceed 256KB
- PERF-007: Flash storage SHALL support >1000 sessions
- PERF-008: Battery life SHALL exceed 8 hours continuous use

### 4.3 Accuracy Requirements
- PERF-009: Step count accuracy SHALL be >95%
- PERF-010: Cadence accuracy SHALL be ±2 SPM
- PERF-011: Contact time accuracy SHALL be ±5ms
- PERF-012: Pressure measurement accuracy SHALL be ±5%
- PERF-013: Distance accuracy SHALL be ±15-20% without GPS
- PERF-014: Distance accuracy SHALL be ±1-3% with GPS
- PERF-015: GPS updates with accuracy >20m SHALL be ignored

---

## 5. Interface Requirements

### 5.1 Hardware Interfaces
- INT-001: The system SHALL interface with ADC for pressure sensors
- INT-002: The system SHALL interface with I2C for BHI360
- INT-003: The system SHALL interface with SPI for flash storage
- INT-004: The system SHALL provide UART for debug output

### 5.2 Software Interfaces
- INT-005: The system SHALL provide BLE GATT interface
- INT-006: The system SHALL implement MCUmgr SMP protocol
- INT-007: The system SHALL provide message queue interfaces
- INT-008: The system SHALL use Zephyr device driver model

### 5.3 User Interfaces
- INT-009: The system SHALL indicate status via LEDs
- INT-010: The system SHALL respond to button inputs
- INT-011: The system SHALL provide BLE control interface

---

## 6. Data Requirements

### 6.1 Data Formats
- DATA-001: Sensor data SHALL use fixed-point representation
- DATA-002: Timestamps SHALL use millisecond resolution
- DATA-003: Activity logs SHALL use protobuf encoding
- DATA-004: BLE data SHALL use packed structures

### 6.2 Data Rates
- DATA-005: System SHALL handle 100Hz pressure data
- DATA-006: System SHALL handle 50Hz motion data
- DATA-007: System SHALL transmit metrics at 1Hz via BLE
- DATA-008: System SHALL log at configurable rates

### 6.3 Data Integrity
- DATA-009: Logged data SHALL include CRC checksums
- DATA-010: System SHALL detect and report data corruption
- DATA-011: System SHALL handle storage full conditions

---

## 7. Safety Requirements

### 7.1 Class A Safety
- SAFE-001: System failure SHALL NOT cause injury
- SAFE-002: System SHALL NOT provide medical diagnosis
- SAFE-003: System SHALL NOT control therapy delivery
- SAFE-004: System SHALL be classified as IEC 62304 Class A

### 7.2 Fail-Safe Operation
- SAFE-005: System SHALL continue operating with sensor failures
- SAFE-006: System SHALL report all error conditions
- SAFE-007: System SHALL prevent data corruption on power loss
- SAFE-008: System SHALL limit surface temperature to <45°C

---

## 8. Security Requirements

### 8.1 Access Control
- SEC-001: System SHALL require BLE bonding
- SEC-002: System SHALL use encrypted BLE connections
- SEC-003: System SHALL limit connection attempts
- SEC-004: System SHALL timeout inactive connections

### 8.2 Data Protection
- SEC-005: System SHALL NOT store personal information
- SEC-006: System SHALL use secure boot if available
- SEC-007: System SHALL verify firmware signatures
- SEC-008: System SHALL protect against buffer overflows

---

## 9. Usability Requirements

### 9.1 Setup
- USE-001: System SHALL be discoverable within 30 seconds
- USE-002: System SHALL complete pairing within 60 seconds
- USE-003: System SHALL auto-connect to known devices
- USE-004: System SHALL indicate connection status

### 9.2 Operation
- USE-005: System SHALL start logging within 2 seconds
- USE-006: System SHALL provide clear error indications
- USE-007: System SHALL support one-button operation
- USE-008: System SHALL work with standard mobile apps

---

## 10. Regulatory Requirements

### 10.1 Standards Compliance
- REG-001: Software SHALL comply with IEC 62304 Class A
- REG-002: Software SHALL support FDA 510(k) submission
- REG-003: Software SHALL support CE marking requirements
- REG-004: Software SHALL maintain audit trails

### 10.2 Documentation
- REG-005: All requirements SHALL be traceable
- REG-006: All changes SHALL be documented
- REG-007: Test evidence SHALL be maintained
- REG-008: Risk analysis SHALL be performed

---

## 11. Requirements Traceability

### 11.1 Traceability Matrix

| Requirement | Design Element | Test Case | Risk |
|-------------|----------------|-----------|------|
| FUNC-001 | foot_sensor module | TC-001 | Low |
| FUNC-002 | 100Hz timer | TC-002 | Medium |
| FUNC-009 | realtime_metrics | TC-009 | Low |
| PERF-001 | Thread priorities | TC-101 | High |
| SAFE-005 | Error handling | TC-201 | Medium |

### 11.2 Coverage Analysis
- Total Requirements: 100+
- Implemented: 85%
- Tested: 60%
- Verified: 40%

---

## Appendices

### A. Requirement Sources
- Clinical study protocols
- User feedback
- Regulatory standards
- Technical constraints

### B. Change History
- Initial requirements from product specification
- Updates from prototype testing
- Additions for clinical compliance

### C. Glossary
- **SPM**: Steps Per Minute
- **GATT**: Generic Attribute Profile
- **SMP**: Simple Management Protocol
- **CRC**: Cyclic Redundancy Check

---

**End of Document**
# Documentation Index

This directory contains all technical documentation for the Sensing Firmware project. Documents are available in both Markdown (.md) and Coda (.coda) formats.

## 📋 Quick Navigation

### Core Specifications
- **[Bluetooth GATT Specification](Bluetooth_GATT_Specification.md)** - Complete BLE services, characteristics, and protocols
  - Includes Information Service, Control Service, Proxy Services, D2D Services
  - NEW: SMP Proxy Service (Section 7.3) - Unified MCUmgr access for both devices
  - NEW: 3D Orientation Service (Section 7.2)
  - Appendix A: Data rate analysis for 3D orientation

- **[Activity Session Calculated Data Specification](Activity_Session_Calculated_Data_Specification.md)** - Pre-calculated metrics for activity sessions
  - Binary format specification for efficient storage
  - Real-time BLE transmission protocol
  - 99.86% data reduction vs raw sensor data

- **[Sensor Logging Specification](Sensor_Logging_Specification.md)** - Raw sensor data logging format and protocols
  - NEW: Activity log file type for session-based metrics
  - Includes foot pressure, BHI360 motion, and activity data logging

### Implementation Guides
- **[BHI360 Complete Integration Guide](BHI360_Complete_Integration_Guide.md)** - Step-by-step BHI360 IMU integration
- **[D2D Communication Complete Guide](D2D_Communication_Complete_Guide.md)** - Device-to-device BLE communication
- **[FOTA Complete Guide](FOTA_Complete_Guide.md)** - Firmware Over-The-Air update procedures
- **[SMP Proxy Integration Guide](SMP_Proxy_Integration_Guide.md)** - NEW: Simplified MCUmgr access for mobile apps
- **[Test Framework Guide](Test_Framework_Guide.md)** - Testing infrastructure and procedures

### Technical References
- **[Developer Quick Reference](Developer_Quick_Reference.md)** - Common commands, build instructions, troubleshooting
- **[nRF5340 Power and Battery Guide](nRF5340_Power_and_Battery_Guide.md)** - Consolidated power management, battery usage, and optimization guide
- **[Pinout nrf5340dk](Pinout_nrf5340dk.md)** - Hardware pin assignments
- **[Doxygen Usage](doxygen_usage.md)** - Code documentation generation

### Analysis Documents
- **[BHI360 Metrics Capability Analysis](BHI360_Metrics_Capability_Analysis.md)** - What metrics can be extracted from BHI360
- **[Feasibility Analysis BHI360 Plus Pressure](Feasibility_Analysis_BHI360_Plus_Pressure.md)** - Combined sensor capabilities

### Legacy/Alternative Proposals
- **[Activity Session Data Specification](Activity_Session_Data_Specification.coda)** - Original protobuf-based proposal (kept for reference)

## 🎯 Where to Start

### For Mobile App Developers
1. Start with **[Bluetooth GATT Specification](Bluetooth_GATT_Specification.md)**
2. Review **[Activity Session Calculated Data Specification](Activity_Session_Calculated_Data_Specification.md)** for understanding data formats
3. Check Section 7.2 in GATT spec for 3D Orientation Service

### For Firmware Developers
1. Read **[Developer Quick Reference](Developer_Quick_Reference.md)**
2. Review relevant implementation guides (BHI360, D2D, FOTA)
3. Use **[Test Framework Guide](Test_Framework_Guide.md)** for testing

### For System Architects
1. Review **[Activity Session Calculated Data Specification](Activity_Session_Calculated_Data_Specification.md)** for data architecture
2. Check analysis documents for sensor capabilities
3. Review **[nRF5340 Power and Battery Guide](nRF5340_Power_and_Battery_Guide.md)** for power budgets

## 📝 Document Formats

- **.md files** - Standard Markdown format, viewable in any text editor or GitHub
- **.coda files** - Coda-specific format with enhanced formatting and diagrams

Both formats contain the same content. Use whichever format works best for your workflow.

## 🔄 Recent Updates

- **Activity Logging** added to sensor logging system
  - New activity log file type for session-based metrics
  - BLE characteristics for activity log management
  - Integration with existing foot and BHI360 logging
- **3D Orientation Service** added to Bluetooth GATT Specification
- **Activity Session Calculated Data Specification** finalized with binary format
- Consolidated 3D orientation documentation into main GATT spec

## 📧 Questions?

For documentation questions or clarifications, please contact the firmware team.
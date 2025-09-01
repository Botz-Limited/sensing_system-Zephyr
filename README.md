# Sensing Firmware

This repository contains the sensing firmware for the nRF5340 Development Kit, implementing a multi-core sensing application with support for primary (right foot) and non-primary device (left foot) configurations.

## Overview

This firmware is designed for the Nordic nRF5340 SoC, utilizing both the application core and network core for efficient sensing operations. The project supports configuration as either a primary or non-primary device in a sensing network.

## Prerequisites

All the necessary tools are installed by the docker file the first time the project is oppened.

- Nordic nRF Connect SDK
- West build tool
- nRF5340 Development Kit
- Python 3.x (for flashing scripts)

## Building the Firmware

To build the firmware, use the following West command:

```bash
west build --build-dir /home/ee/sensing_fw/build /home/ee/sensing_fw/ --board nrf5340dk/nrf5340/cpuapp --sysbuild -- -DCONFIG_PRIMARY_DEVICE=y
```

### Build Configuration Options

- **`-DCONFIG_PRIMARY_DEVICE=y`**: Configures the device as a primary device in the sensing network
- **`-DCONFIG_PRIMARY_DEVICE=n`**: Configures the device as a non-primary (secondary) device


3. **Permission Errors**: The build_flash.sh script may need execute permissions: `chmod +x ./tools/build_flash.sh`
4. **FOTA Issues**: 
   - For primary device: Ensure MCUmgr is properly configured and the device is bonded
   - For secondary device: Verify the device is connected to the primary and the FOTA proxy service is running

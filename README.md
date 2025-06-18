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

Example for building as a non-primary device:
```bash
west build --build-dir /home/ee/sensing_fw/build /home/ee/sensing_fw/ --board nrf5340dk/nrf5340/cpuapp --sysbuild -- -DCONFIG_PRIMARY_DEVICE=n
```

## Flashing the Firmware

### Automated Flashing (Recommended)

The project includes a convenient script that builds and flashes both cores (application and network cores) automatically:

```bash
./tools/build_flash.sh
```

This script will:
1. Build the firmware for both cores
2. Flash the application core firmware
3. Flash the network core firmware
4. Verify the flashing process

### Manual Flashing

If you prefer to flash manually after building:

```bash
west flash --build-dir /home/ee/sensing_fw/build
```

## Project Structure

```
sensing_fw/
├── src/              # Source code files
├── include/          # Header files
├── boards/           # Board-specific configurations
├── tools/            # Utility scripts
│   └── build_flash.sh    # Build and flash script
├── prj.conf          # Project configuration
├── CMakeLists.txt    # CMake configuration
└── README.md         # This file
```

## Features

- Multi-core architecture utilizing nRF5340's dual-core system
- Configurable primary/non-primary device roles
- Sysbuild integration for coordinated multi-image builds
- Automated build and flash tooling

## Configuration

The firmware behavior can be customized through:
- Kconfig options in `prj.conf`
- Device tree overlays in the `boards/` directory
- Build-time configuration flags (like `CONFIG_PRIMARY_DEVICE`)

## Development

### Clean Build

To perform a clean build, remove the build directory first:
```bash
rm -rf /home/ee/sensing_fw/build
```

Then run the build command again.

### Debug Output

Enable debug output by adding the following to your build command:
```bash
-- -DCONFIG_LOG_LEVEL_DBG=y
```

## Troubleshooting

1. **Build Errors**: Ensure you have the correct nRF Connect SDK version installed and properly configured
2. **Flashing Issues**: Make sure the nRF5340 DK is properly connected and the debugger is recognized
3. **Permission Errors**: The build_flash.sh script may need execute permissions: `chmod +x ./tools/build_flash.sh`

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

```



#!/bin/bash

# This script starts a JLink server to allow for RTT clients

# Requires no arguments

JLinkExe -device nRF52 -autoconnect 1 -speed 4000 -if SWD
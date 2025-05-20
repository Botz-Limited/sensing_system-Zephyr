#!/bin/bash

# This script starts a JLink server to allow for RTT clients

# Requires no arguments

JLinkExe  -USB 1057715872 -nogui 1 -if swd -speed 4000 -device cortex-m33
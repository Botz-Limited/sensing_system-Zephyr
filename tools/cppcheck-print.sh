#!/bin/bash

# Runs cppcheck on all the source code and prints the outcome

# Requires no arguments


output=$(cppcheck -q --error-exitcode=1 --check-level=exhaustive --suppressions-list=/home/ee/sensing_fw/tools/cppcheck_suppressions.txt  --inline-suppr --force --enable=all  /home/ee/sensing_fw/src/)

# Check the exit status of cppcheck
if [ $? -ne 0 ]; then
    echo "Cppcheck found issues:"
    echo "$output"
    exit 0
else
    echo "Cppcheck passed with no issues."
    exit 0
fi

#!/bin/python
import os
import sys
import json
from pathlib import Path

"""
This script is to remove entries and flags from a compile_commands.json
"""

flags_to_remove = ["-fno-freestanding", "-fno-reorder-functions", "-fno-defer-pop", "-Wno-volatile", "--param=min-pagesize=0"]

def filter_entries_in(file_content, path_to_remove):
    data = json.loads(file_content)
    filtered_data = [entry for entry in data if path_to_remove in entry['file']]
    return json.dumps(filtered_data, indent=2)

def filter_entries_out(file_content, path_to_remove):
    data = json.loads(file_content)
    filtered_data = [entry for entry in data if path_to_remove not in entry['file']]
    return json.dumps(filtered_data, indent=2)

def remove_flags(file_content, flags):
    for flag in flags:
        file_content = file_content.replace(flag, '')
    return file_content


def main(input_file, output_file, build_folder):
    with open(input_file, 'r') as f:
        file_content = f.read()

        file_content = remove_flags(file_content, flags_to_remove)
        file_content = filter_entries_in(file_content, "sensing_fw")
        file_content = filter_entries_out(file_content, "generated")
        file_content = filter_entries_out(file_content, build_folder)

        with open(output_file, 'w') as f:
            f.write(file_content)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Removes flags not compatible with clang and out of application files from the provided compile_commands.json for use with clang-tidy")
        print("Usage: modify_compile_commands.py <input_file> <output_file> <build_folder>")
    else:
        main(sys.argv[1], sys.argv[2], sys.argv[3])

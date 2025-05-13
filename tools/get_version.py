#!/bin/python
import sys

# Extracts the version number from the root CMakeList.txt file
 
# Requires no arguments

# Prints version if used standalone and exits with 0 if successful and 1 if not.

# If used as a function returns an empty string 

def extract_version_from_cmakelists(path_to_cmakelist: str) -> str:
    try:
        f =  open(path_to_cmakelist, encoding = 'utf-8')
    except:
        print(f"ERROR: Could not find/open {path_to_cmakelist}")
        return ""
    else:
        lines = f.readlines()
        for line in lines:
            if "project(" in line and "VERSION" in line:
                after_version = line.split("VERSION",1)[1]
                version = after_version.split(" ")[1]
                return version
                
        print(f"ERROR: Could not find a version number in {path}")   
        return "" 
        

if __name__ == "__main__":
    version = extract_version_from_cmakelists("CMakeLists.txt")
    if version == "":
        sys.exit(1)
    else:
        print(version)
        sys.exit(0)

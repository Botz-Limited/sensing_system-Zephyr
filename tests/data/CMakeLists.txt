cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(data_test)

# Include directories
target_include_directories(app PRIVATE 
    ${CMAKE_SOURCE_DIR}/../../include
)

# Add test sources - only include the basic test for now
target_sources(app PRIVATE
    test_data.cpp
)

# Set compile options
target_compile_options(app PRIVATE -std=c++20)
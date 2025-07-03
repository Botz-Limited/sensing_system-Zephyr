cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(realtime_metrics_test)

# Add test source
target_sources(app PRIVATE test_realtime_metrics.cpp)

# Include paths
target_include_directories(app PRIVATE
    ${CMAKE_SOURCE_DIR}/../../include
    ${CMAKE_SOURCE_DIR}/../../src/realtime_metrics
)

# Add module source files (excluding main file with thread)
target_sources(app PRIVATE
    ${CMAKE_SOURCE_DIR}/../../src/app/app.cpp
    ${CMAKE_SOURCE_DIR}/../../src/app/app_events.cpp
)
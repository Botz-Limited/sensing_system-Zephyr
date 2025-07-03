#
# Copyright (c) 2025 Botz Innovation
#

target_sources(app PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/realtime_metrics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/realtime_metrics_algorithms.c
)

zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
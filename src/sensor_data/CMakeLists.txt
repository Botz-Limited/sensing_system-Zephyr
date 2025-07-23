#
# Copyright (c) 2025 Botz Innovation
#

target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_data_sync.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_data_sync_enhanced.cpp
)

zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
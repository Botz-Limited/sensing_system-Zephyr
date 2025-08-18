#
# Copyright (c) 2025 Botz Innovation
#

target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_data.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_data_sync.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_data_sync_enhanced.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_data_events.cpp
    ${CMAKE_SOURCE_DIR}/src/gait_events.cpp
    ${CMAKE_SOURCE_DIR}/src/gait_advanced_features.cpp
)

zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
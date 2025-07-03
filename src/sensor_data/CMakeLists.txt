#
# Copyright (c) 2025 Botz Innovation
#

target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/sensor_data.cpp)

zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
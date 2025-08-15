#
# Copyright (c) 2025 Botz Innovation
#

target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/analytics.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/choros_buffer_impl.cpp
)

zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
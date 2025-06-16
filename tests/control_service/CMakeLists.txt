cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(control_service_test)

target_sources(app PRIVATE test_control_service.c)

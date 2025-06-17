cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(data_test)

target_sources(app PRIVATE test_data_log_decode.c ../../src/data/foot_sensor_messages.pb.c ../../src/data/bhi360_sensor_messages.pb.c)
target_include_directories(app PRIVATE ../../src/data)

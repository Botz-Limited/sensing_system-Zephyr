# Choose implementation based on Kconfig

    zephyr_library_sources(foot_sensor_simple.cpp)
    message(STATUS "Using simplified foot sensor implementation")

zephyr_library_compile_options(${SYS_COMPILE_FLAGS})

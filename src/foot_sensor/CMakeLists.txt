# Choose implementation based on Kconfig
if(CONFIG_FOOT_SENSOR_USE_SIMPLE)
    zephyr_library_sources(foot_sensor_simple.cpp)
    message(STATUS "Using simplified foot sensor implementation")
else()
    zephyr_library_sources(foot_sensor.cpp)
    message(STATUS "Using complex NRFX foot sensor implementation")
endif()

zephyr_library_compile_options(${SYS_COMPILE_FLAGS})

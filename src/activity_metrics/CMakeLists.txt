FILE(GLOB app_sources *.cpp)
zephyr_library_sources(${app_sources})
zephyr_library_compile_options(${SYS_COMPILE_FLAGS})
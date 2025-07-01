cmake_minimum_required(VERSION 3.20.0)

# Only compile WiFi module when WiFi is enabled
if(CONFIG_WIFI_MODULE)
    FILE(GLOB app_sources *.cpp)
    FILE(GLOB c_sources *.c)
    zephyr_library_compile_options(${SYS_COMPILE_FLAGS})
    zephyr_library_sources(${app_sources} ${c_sources})
endif()
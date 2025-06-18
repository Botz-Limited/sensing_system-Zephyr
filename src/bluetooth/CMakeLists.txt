# Get all source files
FILE(GLOB app_sources *.cpp)

# Remove files that will be conditionally included
list(REMOVE_ITEM app_sources ${CMAKE_CURRENT_SOURCE_DIR}/fota_proxy.cpp)
list(REMOVE_ITEM app_sources ${CMAKE_CURRENT_SOURCE_DIR}/file_proxy.cpp)
list(REMOVE_ITEM app_sources ${CMAKE_CURRENT_SOURCE_DIR}/ble_d2d_file_transfer.cpp)
list(REMOVE_ITEM app_sources ${CMAKE_CURRENT_SOURCE_DIR}/information_service.cpp)
list(REMOVE_ITEM app_sources ${CMAKE_CURRENT_SOURCE_DIR}/control_service.cpp)
list(REMOVE_ITEM app_sources ${CMAKE_CURRENT_SOURCE_DIR}/ble_d2d_rx.cpp)
list(REMOVE_ITEM app_sources ${CMAKE_CURRENT_SOURCE_DIR}/ble_d2d_tx.cpp)

zephyr_library_compile_options(${SYS_COMPILE_FLAGS})
zephyr_library_sources(${app_sources})

# Primary device specific services
zephyr_library_sources_ifdef(CONFIG_PRIMARY_DEVICE 
    fota_proxy.cpp 
    file_proxy.cpp
    information_service.cpp
    control_service.cpp
    ble_d2d_rx.cpp  # Primary receives data from secondary
)

# Secondary device specific services
if(NOT CONFIG_PRIMARY_DEVICE)
    zephyr_library_sources(
        ble_d2d_tx.cpp  # Secondary sends data to primary
    )
endif()

# Include D2D file transfer for all devices
zephyr_library_sources(ble_d2d_file_transfer.cpp)
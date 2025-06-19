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
list(REMOVE_ITEM app_sources ${CMAKE_CURRENT_SOURCE_DIR}/ble_d2d_tx_service.cpp)
list(REMOVE_ITEM app_sources ${CMAKE_CURRENT_SOURCE_DIR}/ble_d2d_rx_client.cpp)
# Don't remove bluetooth_debug.cpp - it's needed for all builds

zephyr_library_compile_options(${SYS_COMPILE_FLAGS})
zephyr_library_sources(${app_sources})

# Primary device specific services
zephyr_library_sources_ifdef(CONFIG_PRIMARY_DEVICE 
    fota_proxy.cpp 
    file_proxy.cpp
    information_service.cpp
    control_service.cpp
    ble_d2d_rx.cpp  # Primary receives data/status from secondary
    ble_d2d_tx.cpp  # Primary sends commands to secondary
    ble_d2d_rx_client.cpp  # Primary subscribes to secondary's TX service
)

# Secondary device specific services
if(NOT CONFIG_PRIMARY_DEVICE)
    zephyr_library_sources(
        ble_d2d_tx.cpp  # Secondary sends data/status to primary
        ble_d2d_rx.cpp  # Secondary receives commands from primary
        ble_d2d_tx_service.cpp  # Secondary's GATT service for sending data
    )
endif()

# Include D2D file transfer for all devices
zephyr_library_sources(ble_d2d_file_transfer.cpp)
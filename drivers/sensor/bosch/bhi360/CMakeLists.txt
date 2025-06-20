zephyr_library()

# Add BHY2 sensor API source files
file(GLOB BHY2_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/BHY2-Sensor-API/*.c"
)
zephyr_library_sources(
    bhi360_zephyr.c
    bhi360_calibration.c
    ${BHY2_SOURCES}
)


  


zephyr_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/BHY2-Sensor-API
)

zephyr_compile_definitions(
    BHY2_CFG_DELEGATE_FIFO_PARSE_CB_INFO_MGMT=0
)
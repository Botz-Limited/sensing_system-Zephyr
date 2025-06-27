add_subdirectory(app)
add_subdirectory(data)
add_subdirectory(bluetooth)
add_subdirectory(app_events)
add_subdirectory(foot_sensor)
add_subdirectory(motion_sensor)
add_subdirectory(packetizer)
add_subdirectory(battery)

# Add FOTA fix utilities
target_sources(app PRIVATE fota_fix.cpp)

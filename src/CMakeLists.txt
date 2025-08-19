add_subdirectory(app)
add_subdirectory(data)
add_subdirectory(bluetooth)
add_subdirectory(app_events)
add_subdirectory(foot_sensor)
add_subdirectory(motion_sensor)
add_subdirectory(activity_metrics)
add_subdirectory(sensor_data)
add_subdirectory(realtime_metrics)
add_subdirectory(analytics)
add_subdirectory(packetizer)
add_subdirectory(battery)
add_subdirectory(wifi)

# Add gait analysis source files to the build
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/gait_events.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/gait_advanced_features.cpp
)

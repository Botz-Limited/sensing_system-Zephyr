cmake_minimum_required(VERSION 3.20.0)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(util)
#include(MunkeiVersionFromGit)

# Before find_package(Zephyr)
set(BUILD_VERSION_FROM_GIT OFF)
set(CONFIG_APP_VERSION "1.0.0")

#export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Ensure our custom boards are found
list(APPEND BOARD_ROOT $ENV{APPLICATION_ROOT_PATH})
set(BOARD_ROOT "/home/ee/sensing_fw")

list(APPEND ZEPHYR_EXTRA_MODULES
 # ${CMAKE_CURRENT_SOURCE_DIR}/drivers
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bluetooth
)

set(CONFIG_MCUBOOT_IMGTOOL_SIGN_VERSION "\"${VERSION}\"" CACHE STRING "MCUboot image version")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C CXX ASM)

test_sysbuild()

target_include_directories(app PUBLIC ${CMAKE_BINARY_DIR})
zephyr_include_directories(${CMAKE_BINARY_DIR})


#This must be added to remove unused variables, and initialise the variables not used.
target_compile_options(app PRIVATE ${SYS_COMPILE_FLAGS})

# Sets all the zephyr headers to be system include this is prevent
set_target_properties(zephyr_interface PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES $<TARGET_PROPERTY:zephyr_interface,INTERFACE_INCLUDE_DIRECTORIES>)

target_include_directories(app PUBLIC ${CMAKE_BINARY_DIR})
zephyr_include_directories(${CMAKE_BINARY_DIR})

zephyr_include_directories(
  $ENV{APPLICATION_ROOT_PATH}/include
  $ENV{APPLICATION_ROOT_PATH}/src
  configuration
  ${ZEPHYR_BASE}/../modules/fs/littlefs
  ${CMAKE_BINARY_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)

target_sources(app PRIVATE src/main.c)

# Moves the compile_commands.json file into root for clangd
add_custom_command(
    TARGET app PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/compile_commands.json ${CMAKE_SOURCE_DIR}/compile_commands.json
)

add_custom_command (
    TARGET app POST_BUILD
    COMMAND /home/ee/sensing_fw/tools/cppcheck-print.sh  $ENV{APPLICATION_ROOT_PATH}/src
   # WORKING_DIRECTORY $ENV{APPLICATION_ROOT_PATH}
    COMMENT "Running Cppcheck..."
)




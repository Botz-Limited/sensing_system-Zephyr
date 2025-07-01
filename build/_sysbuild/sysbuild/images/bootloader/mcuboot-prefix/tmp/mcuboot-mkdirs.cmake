# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/ee/ncs/bootloader/mcuboot/boot/zephyr")
  file(MAKE_DIRECTORY "/home/ee/ncs/bootloader/mcuboot/boot/zephyr")
endif()
file(MAKE_DIRECTORY
  "/home/ee/sensing_fw/build/mcuboot"
  "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/bootloader/mcuboot-prefix"
  "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/bootloader/mcuboot-prefix/tmp"
  "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/bootloader/mcuboot-prefix/src/mcuboot-stamp"
  "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/bootloader/mcuboot-prefix/src"
  "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/bootloader/mcuboot-prefix/src/mcuboot-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/bootloader/mcuboot-prefix/src/mcuboot-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/bootloader/mcuboot-prefix/src/mcuboot-stamp${cfgdir}") # cfgdir has leading slash
endif()

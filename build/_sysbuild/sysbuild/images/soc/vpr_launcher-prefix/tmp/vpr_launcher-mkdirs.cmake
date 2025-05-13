# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/ee/ncs/zephyr/samples/basic/minimal")
  file(MAKE_DIRECTORY "/home/ee/ncs/zephyr/samples/basic/minimal")
endif()
file(MAKE_DIRECTORY
  "/home/ee/sensing_fw/build/vpr_launcher"
  "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/soc/vpr_launcher-prefix"
  "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/soc/vpr_launcher-prefix/tmp"
  "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/soc/vpr_launcher-prefix/src/vpr_launcher-stamp"
  "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/soc/vpr_launcher-prefix/src"
  "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/soc/vpr_launcher-prefix/src/vpr_launcher-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/soc/vpr_launcher-prefix/src/vpr_launcher-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/ee/sensing_fw/build/_sysbuild/sysbuild/images/soc/vpr_launcher-prefix/src/vpr_launcher-stamp${cfgdir}") # cfgdir has leading slash
endif()

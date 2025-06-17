# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/home/ee/sensing_fw/tests/data")
  file(MAKE_DIRECTORY "/home/ee/sensing_fw/tests/data")
endif()
file(MAKE_DIRECTORY
  "/home/ee/sensing_fw/tests/data/build/data"
  "/home/ee/sensing_fw/tests/data/build/_sysbuild/sysbuild/images/data-prefix"
  "/home/ee/sensing_fw/tests/data/build/_sysbuild/sysbuild/images/data-prefix/tmp"
  "/home/ee/sensing_fw/tests/data/build/_sysbuild/sysbuild/images/data-prefix/src/data-stamp"
  "/home/ee/sensing_fw/tests/data/build/_sysbuild/sysbuild/images/data-prefix/src"
  "/home/ee/sensing_fw/tests/data/build/_sysbuild/sysbuild/images/data-prefix/src/data-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/ee/sensing_fw/tests/data/build/_sysbuild/sysbuild/images/data-prefix/src/data-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/ee/sensing_fw/tests/data/build/_sysbuild/sysbuild/images/data-prefix/src/data-stamp${cfgdir}") # cfgdir has leading slash
endif()

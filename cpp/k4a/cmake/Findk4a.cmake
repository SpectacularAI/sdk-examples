#.rst:
# Findk4a
# -------
#
# Find Azure Kinect Sensor SDK include dirs, and libraries.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the :prop_tgt:`IMPORTED` targets:
#
# ``k4a::k4a``
#  Defined if the system has Azure Kinect Sensor SDK.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module sets the following variables:
#
# ::
#
#   k4a_FOUND               True in case Azure Kinect Sensor SDK is found, otherwise false
#   k4a_ROOT                Path to the root of found Azure Kinect Sensor SDK installation
#
# Example usage
# ^^^^^^^^^^^^^
#
# ::
#
#     find_package(k4a REQUIRED)
#
#     add_executable(foo foo.cc)
#     target_link_libraries(foo k4a::k4a)
#
# License
# ^^^^^^^
#
# Copyright (c) 2019 Tsukasa SUGIURA
# Distributed under the MIT License.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

find_path(k4a_INCLUDE_DIR
  NAMES
    k4a/k4a.h
  HINTS
    ${k4a_DIR}/sdk/
  PATHS
    "${k4a_PATH_DIR}/sdk/"
  PATH_SUFFIXES
    include
)

find_library(k4a_LIBRARY
  NAMES
    k4a.lib
  HINTS
    ${k4a_DIR}/sdk/windows-desktop/amd64/release
  PATHS
    "${k4a_PATH_DIR}/sdk/windows-desktop/amd64/release"
  PATH_SUFFIXES
    lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  k4a DEFAULT_MSG
  k4a_LIBRARY k4a_INCLUDE_DIR
)

if(k4a_FOUND)
  add_library(k4a::k4a SHARED IMPORTED)

  set_target_properties(k4a::k4a PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${k4a_INCLUDE_DIR}")
  set_property(TARGET k4a::k4a APPEND PROPERTY IMPORTED_CONFIGURATIONS "RELEASE")
  set_target_properties(k4a::k4a PROPERTIES IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX")

  # TODO: Ugly "hack" to find the DLL. Couldn't find any better way of doing this
  string(REPLACE "lib/k4a.lib" "bin/k4a.dll" k4a_RUNTIME_LIBRARY "${k4a_LIBRARY}")

  set_target_properties(k4a::k4a PROPERTIES IMPORTED_IMPLIB "${k4a_LIBRARY}")
  set_target_properties(k4a::k4a PROPERTIES IMPORTED_LOCATION "${k4a_RUNTIME_LIBRARY}")

  get_filename_component(K4a_ROOT "${k4a_INCLUDE_DIR}" PATH)
endif()

cmake_minimum_required(VERSION 3.5)
project(rtabmap_mapper)
set(CMAKE_CXX_STANDARD 20)

option(BUILD_REPLAY "Replay support" OFF)
option(BUILD_K4A "Azure Kinect support" OFF)
option(BUILD_REALSENSE "Realsense support" OFF)
option(BUILD_OAK "OAK-D support" OFF)

if (NOT BUILD_REPLAY AND NOT BUILD_K4A AND NOT BUILD_REALSENSE AND NOT BUILD_OAK)
  message(FATAL_ERROR " \
You must build with support for at least one data source.\n \
It can be one or more of: \n \
  -DBUILD_REPLAY=ON       # With Core SDK\n \
  -DBUILD_K4A=ON          # With Azure Kinect plugin\n \
  -DBUILD_REALSENSE=ON    # With RealSense plugin\n \
  -DBUILD_OAK=ON          # With OAK-D plugin\n \
")
endif()

find_package(RTABMap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Threads REQUIRED)

SET(SOURCES
  src/main.cpp
  src/camera_spectacularai.cpp
  src/camera_replay.cpp
  src/camera_k4a.cpp
  src/camera_realsense.cpp
  src/camera_oak.cpp
  src/util.cpp)

# Find Qt5 first
find_package(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
if(NOT Qt5_FOUND)
  find_package(Qt4 COMPONENTS QtCore QtGui QtSvg)
endif(NOT Qt5_FOUND)

if(POLICY CMP0020)
	cmake_policy(SET CMP0020 NEW)
endif()

if(QT4_FOUND)
    include(${QT_USE_FILE})
endif(QT4_FOUND)

set(INCLUDE_DIRS
  ${RTABMap_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(LIBRARIES
  ${RTABMap_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  Threads::Threads
)

if (BUILD_REPLAY)
  find_package(spectacularAI REQUIRED)
  list(APPEND LIBRARIES spectacularAI::spectacularAI)
  add_definitions(-DSPECTACULARAI_CORE)
endif()

if (BUILD_K4A)
  find_package(spectacularAI_k4aPlugin REQUIRED)
  list(APPEND LIBRARIES spectacularAI::k4aPlugin)
  add_definitions(-DSPECTACULARAI_CAMERA_K4A)
endif()

if (BUILD_REALSENSE)
  find_package(realsense2 REQUIRED)
  find_package(spectacularAI_realsensePlugin REQUIRED)
  list(APPEND LIBRARIES
    realsense2::realsense2
    spectacularAI::realsensePlugin)
  add_definitions(-DSPECTACULARAI_CAMERA_REALSENSE)
endif()

if (BUILD_OAK)
  find_package(depthai REQUIRED)
  find_package(spectacularAI_depthaiPlugin REQUIRED)
  list(APPEND LIBRARIES
    depthai::core
    spectacularAI::depthaiPlugin)
  add_definitions(-DSPECTACULARAI_CAMERA_OAK)
endif()

if(QT4_FOUND)
    QT4_WRAP_CPP(moc_srcs include/spectacularAI/rtabmap/map_builder.h)
else()
    QT5_WRAP_CPP(moc_srcs include/spectacularAI/rtabmap/map_builder.h)
endif()

add_executable(rtabmap_mapper ${SOURCES} ${moc_srcs})
target_link_libraries(rtabmap_mapper ${LIBRARIES})
target_include_directories(rtabmap_mapper PRIVATE ${INCLUDE_DIRS})

if(MSVC)
  add_custom_command(TARGET rtabmap_mapper POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:rtabmap_mapper> $<TARGET_FILE_DIR:rtabmap_mapper>
    COMMAND_EXPAND_LISTS
  )
endif()

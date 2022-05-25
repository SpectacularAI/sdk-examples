cmake_minimum_required(VERSION 3.5)
project(rtabmap_mapper)
set(CMAKE_CXX_STANDARD 20)

option(BUILD_K4A "Azure Kinect support" OFF)

if (NOT BUILD_K4A)
  set(AVAILABILITY_CAMERA_K4A "//")
endif()

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/include/spectacularAI/rtabmap/camera_availability.h.in
               ${CMAKE_CURRENT_SOURCE_DIR}/include/spectacularAI/rtabmap/camera_availability.h)

find_package(spectacularAI REQUIRED)
find_package(RTABMap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Threads REQUIRED)

if (BUILD_K4A)
  find_package(k4a REQUIRED)
  find_package(spectacularAI_k4aPlugin REQUIRED)
endif()

SET(SOURCES
  src/main.cpp
  src/camera_spectacularai.cpp
  src/camera_replay.cpp
  src/camera_k4a.cpp
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
  spectacularAI::spectacularAI
)

if (BUILD_K4A)
    set(LIBRARIES
      ${LIBRARIES}
      k4a::k4a
      spectacularAI::k4aPlugin)
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

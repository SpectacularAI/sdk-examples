if(MSVC)
  # Windows build uses newer features
  cmake_minimum_required(VERSION 3.21)
else()
  cmake_minimum_required(VERSION 3.3)
endif()

project(spectacularAI_depthaiPlugin_example)

if(MSVC)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
else()
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

set(USE_OPENCV OFF CACHE STRING "Use OpenCV debug visualizations")

find_package(depthai REQUIRED)
find_package(spectacularAI_depthaiPlugin REQUIRED)

set(EXAMPLE_LIBS
  depthai::core
  spectacularAI::depthaiPlugin)

if (USE_OPENCV)
  find_package(OpenCV REQUIRED)
  add_definitions(-DEXAMPLE_USE_OPENCV)
  list(APPEND EXAMPLE_LIBS "${OpenCV_LIBS}")
  list(APPEND EXAMPLE_LIBS depthai::opencv)
endif()

# enables searching for dynamic libraries from the relative path ../lib
if(NOT MSVC)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath='$ORIGIN/../lib:$ORIGIN/../lib/3rdparty'")
endif()
add_executable(vio_jsonl vio_jsonl.cpp)
target_link_libraries(vio_jsonl ${EXAMPLE_LIBS})

add_executable(vio_replay vio_replay.cpp)
target_link_libraries(vio_replay ${EXAMPLE_LIBS})

if(MSVC)
  add_custom_command(TARGET vio_jsonl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:vio_jsonl> $<TARGET_FILE_DIR:vio_jsonl>
    COMMAND_EXPAND_LISTS
  )

  add_custom_command(TARGET vio_replay POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:vio_replay> $<TARGET_FILE_DIR:vio_replay>
    COMMAND_EXPAND_LISTS
  )
endif()

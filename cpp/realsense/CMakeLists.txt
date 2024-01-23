if(MSVC)
  # Windows build uses newer features
  cmake_minimum_required(VERSION 3.21)
else()
  cmake_minimum_required(VERSION 3.3)
endif()

project(spectacularAI_realsense_example)

if(MSVC)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
else()
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

find_package(realsense2 REQUIRED)
find_package(spectacularAI_realsensePlugin REQUIRED)

set(EXAMPLE_LIBS
  realsense2::realsense2
  spectacularAI::realsensePlugin)

# enables searching for dynamic libraries from the relative path ../lib
if(NOT MSVC)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath='$ORIGIN/../lib:$ORIGIN/../lib/3rdparty'")
endif()
add_executable(vio_jsonl vio_jsonl.cpp)
target_link_libraries(vio_jsonl ${EXAMPLE_LIBS})

if(MSVC)
  add_custom_command(TARGET vio_jsonl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:vio_jsonl> $<TARGET_FILE_DIR:vio_jsonl>
    COMMAND_EXPAND_LISTS
  )
endif()

set(BUILD_MAPPER ON CACHE STRING "Build VIO mapper example (requires OpenCV)")
if (BUILD_MAPPER)
  find_package(Threads REQUIRED)
  set(USE_STATIC_OPENCV OFF CACHE STRING "Use OpenCV as statically linked library (internal and unsupported flag, do not use)")
  if (USE_STATIC_OPENCV)
    find_package(mobile-cv-suite REQUIRED)
    set(MAPPER_LIBS mobile-cv-suite::static mobile-cv-suite::imgcodecs Threads::Threads)
  else()
    find_package(OpenCV REQUIRED)
    set(MAPPER_LIBS "${OpenCV_LIBS}" Threads::Threads)
  endif()

  add_executable(vio_mapper vio_mapper.cpp)
  target_link_libraries(vio_mapper ${EXAMPLE_LIBS} ${MAPPER_LIBS})
  if(MSVC)
    add_custom_command(TARGET vio_mapper POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:vio_mapper> $<TARGET_FILE_DIR:vio_mapper>
      COMMAND_EXPAND_LISTS
    )
  endif()

  add_executable(vio_mapper_legacy vio_mapper_legacy.cpp)
  target_link_libraries(vio_mapper_legacy ${EXAMPLE_LIBS} ${MAPPER_LIBS})
  if(MSVC)
    add_custom_command(TARGET vio_mapper_legacy POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:vio_mapper_legacy> $<TARGET_FILE_DIR:vio_mapper_legacy>
      COMMAND_EXPAND_LISTS
    )
  endif()
endif()

# Mapping example, used together with mapping_visu.py visulization
set(BUILD_MAPPING_VISU ON CACHE STRING "Build VIO mapper example (requires nlohmann_json)")
if (BUILD_MAPPING_VISU)
  find_package(Threads REQUIRED)
  set(JSON_Install OFF CACHE INTERNAL "")
  set(JSON_BuildTests OFF CACHE INTERNAL "")
  add_subdirectory("../offline/json" json)

  add_executable(mapping_visu mapping_visu.cpp ../common/serialize_output.cpp)
  target_link_libraries(mapping_visu PRIVATE ${EXAMPLE_LIBS} nlohmann_json::nlohmann_json Threads::Threads)
  target_include_directories(mapping_visu PRIVATE "../common")
  if(MSVC)
    add_custom_command(TARGET mapping_visu POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:mapping_visu> $<TARGET_FILE_DIR:mapping_visu>
      COMMAND_EXPAND_LISTS
    )
  endif()
endif()

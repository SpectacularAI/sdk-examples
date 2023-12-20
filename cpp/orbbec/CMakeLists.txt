if(MSVC)
  # Windows build uses newer features
  cmake_minimum_required(VERSION 3.21)
else()
  cmake_minimum_required(VERSION 3.3)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

project(spectacularAI_orbbec_example)

if(MSVC)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
else()
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

find_package(Threads REQUIRED)
find_package(spectacularAI_orbbecPlugin REQUIRED)

if(MSVC) # Must be after project() is called
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /Gy")
  find_package(OrbbecSDK CONFIG REQUIRED PATHS "${OrbbecSDK_DIR}")
else()
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  find_package(OrbbecSDK CONFIG REQUIRED PATHS "${OrbbecSDK_DIR}")
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--exclude-libs=ALL")
endif()

set(EXAMPLE_LIBS
  ${OrbbecSDK_LIBS}
  spectacularAI::orbbecPlugin)

# enables searching for dynamic libraries from the relative path ../lib
if(NOT MSVC)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath='$ORIGIN/../lib:$ORIGIN/../lib/3rdparty'")
endif()

# Minimal example
add_executable(vio_jsonl vio_jsonl.cpp)
target_link_libraries(vio_jsonl ${EXAMPLE_LIBS})
target_include_directories(vio_jsonl PRIVATE ${OrbbecSDK_INCLUDE_DIRS})

if(MSVC)
  add_custom_command(TARGET vio_jsonl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:vio_jsonl> $<TARGET_FILE_DIR:vio_jsonl>
    COMMAND_EXPAND_LISTS
  )
endif()

# Mapping example, used together with mapping_visu.py visulization
set(BUILD_MAPPING_VISU ON CACHE STRING "Build VIO mapper example (requires nlohmann_json)")
if (BUILD_MAPPING_VISU)
  set(JSON_Install OFF CACHE INTERNAL "")
  set(JSON_BuildTests OFF CACHE INTERNAL "")
  add_subdirectory("../offline/json" json)

  add_executable(mapping_visu mapping_visu.cpp ../common/serialize_output.cpp)
  target_link_libraries(mapping_visu PRIVATE ${EXAMPLE_LIBS} nlohmann_json::nlohmann_json Threads::Threads)
  target_include_directories(mapping_visu PRIVATE "../common" ${OrbbecSDK_INCLUDE_DIRS})
endif()

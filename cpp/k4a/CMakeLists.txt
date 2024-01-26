if(MSVC)
  # Windows build uses newer features
  cmake_minimum_required(VERSION 3.21)
else()
  cmake_minimum_required(VERSION 3.3)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

project(spectacularAI_k4a_example)

if(MSVC)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
else()
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

find_package(Threads REQUIRED)
find_package(spectacularAI_k4aPlugin REQUIRED)

if(MSVC) # Must be after project() is called
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /Gy")
  # ./cmake/Findk4a.cmake is only tested on Windows and ideally we would rely on system dependency
  find_package(k4a MODULE REQUIRED)
else()
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  find_package(k4a REQUIRED PATHS "${k4a_DIR}")
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--exclude-libs=ALL")
endif()

set(EXAMPLE_LIBS
  k4a::k4a
  spectacularAI::k4aPlugin)

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

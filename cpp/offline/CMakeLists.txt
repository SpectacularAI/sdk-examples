cmake_minimum_required(VERSION 3.3)

project(spectacularAI_offline_example)

if(MSVC)
  set(CMAKE_CXX_STANDARD 20)
endif()

add_executable(vio_jsonl
  vio_jsonl.cpp
  input.cpp
)

find_package(spectacularAI REQUIRED)
add_library(lodepng "lodepng/lodepng.cpp")
add_subdirectory(json)

target_link_libraries(vio_jsonl PRIVATE spectacularAI::spectacularAI lodepng nlohmann_json::nlohmann_json)
target_include_directories(vio_jsonl PRIVATE ".")

if(MSVC)
  # Copy spectacularAI.dll in build directory so that vio_jsonl.exe finds it
  add_custom_command(TARGET vio_jsonl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:vio_jsonl> $<TARGET_FILE_DIR:vio_jsonl>
    COMMAND_EXPAND_LISTS
  )
endif()
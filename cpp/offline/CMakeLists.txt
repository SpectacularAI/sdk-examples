cmake_minimum_required(VERSION 3.3)

project(spectacularAI_offline_example)

add_executable(vio_jsonl
  vio_jsonl.cpp
  input.cpp
)

find_package(spectacularAI REQUIRED)
add_library(lodepng "lodepng/lodepng.cpp")

target_link_libraries(vio_jsonl spectacularAI::spectacularAI lodepng)
target_include_directories(vio_jsonl PRIVATE ".")

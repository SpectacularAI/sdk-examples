cmake_minimum_required(VERSION 3.3)

project(spectacularAI_replay_example)

add_executable(replay replay.cpp)

find_package(spectacularAI REQUIRED)

target_link_libraries(replay PRIVATE spectacularAI::spectacularAI)
target_include_directories(replay PRIVATE ".")

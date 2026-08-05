cmake_minimum_required(VERSION 3.3)

project(spectacularAI_replay_example)

add_executable(replay replay.cpp)

find_package(spectacularAI REQUIRED)

target_link_libraries(replay PRIVATE spectacularAI::spectacularAI)
target_include_directories(replay PRIVATE ".")

if(MSVC)
  # Copy spectacularAI.dll in build directory so that replay.exe finds it
  add_custom_command(TARGET replay POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:replay> $<TARGET_FILE_DIR:replay>
    COMMAND_EXPAND_LISTS
  )
endif()
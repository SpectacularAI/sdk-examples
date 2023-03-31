cmake_minimum_required(VERSION 3.3)

project(spectacularAI_mapping_visu_example)

find_package(spectacularAI REQUIRED)
find_package(Threads REQUIRED)

set(JSON_Install OFF CACHE INTERNAL "")
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory("../offline/json" json)

add_executable(mapping_visu mapping_visu.cpp serialize_output.cpp)
target_link_libraries(mapping_visu PRIVATE spectacularAI::spectacularAI nlohmann_json::nlohmann_json Threads::Threads)
target_include_directories(mapping_visu PRIVATE ".")

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
find_package(OrbbecSDK REQUIRED PATHS "${OrbbecSDK_DIR}")

if(MSVC) # Must be after project() is called
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /Gy")
else()
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--exclude-libs=ALL")
endif()

set(EXAMPLE_LIBS
  spectacularAI::orbbecPlugin
  OrbbecSDK::OrbbecSDK)

# enables searching for dynamic libraries from the relative path ../lib
if(NOT MSVC)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath='$ORIGIN/../lib:$ORIGIN/../lib/3rdparty'")
endif()

# Minimal example
add_executable(vio_jsonl vio_jsonl.cpp)
target_link_libraries(vio_jsonl ${EXAMPLE_LIBS})

if(MSVC)
  add_custom_command(TARGET vio_jsonl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:vio_jsonl> $<TARGET_FILE_DIR:vio_jsonl>
    COMMAND_EXPAND_LISTS
  )

  # Explicitly copy live555.dll & ob_usb.dll (not included in TARGET_RUNTIME_DLLS:vio_jsonl)
  # NOTE: remove these lines if using older OrbbecSDK without live555.dll and ob_usb.dll
  set(LIVE555_DLL_PATH "${OrbbecSDK_LIBS_DIR}/live555.dll")
  set(OB_USB_DLL_PATH "${OrbbecSDK_LIBS_DIR}/ob_usb.dll")
  add_custom_command(TARGET vio_jsonl POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIVE555_DLL_PATH} $<TARGET_FILE_DIR:vio_jsonl>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${OB_USB_DLL_PATH} $<TARGET_FILE_DIR:vio_jsonl>
  )
endif()
project(planning)
cmake_minimum_required(VERSION 3.0)

# set release mode
# Debug: X86  Release: FDC
# set(CMAKE_BUILD_TYPE Release)

# settings
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fPIC -fpermissive")

# adapter
set(ADAPTER "cyber")
if (ADAPTER STREQUAL "cyber")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCYBER_ENV")
elseif (ADAPTER STREQUAL "ap")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DAP_ENV")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  # release mode
  message("Building in Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
else()
  # debug mode
  message("Building in Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -DX86")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
endif()

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(tools/planning_player)

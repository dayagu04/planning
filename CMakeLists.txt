project(planning)
cmake_minimum_required(VERSION 3.0)

# set release mode
# set(CMAKE_BUILD_TYPE Release)

# settings
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fPIC -fpermissive -DCYBER_ENV -DLINUX")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  message("Building in Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
else()
  message("Building in Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
  add_subdirectory(test)
endif()

# main src
add_subdirectory(src)

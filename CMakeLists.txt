project(planning)
cmake_minimum_required(VERSION 3.0)

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  message (STATUS "Set CMAKE_BUILD_TYPE to default: ${CMAKE_BUILD_TYPE}")
endif()

if(NOT ADAPTER OR ADAPTER STREQUAL "")
  set(ADAPTER "cyber" CACHE STRING "" FORCE)
  message (STATUS "Set ADAPTER to default: ${ADAPTER}")
endif()

if(NOT PLATFORM OR ADAPTER STREQUAL "")
  set(PLATFORM "BZT" CACHE STRING "" FORCE)
  message (STATUS "Set PLATFORM to default: ${PLATFORM}")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fPIC -fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${PLATFORM} -D${PRODUCT}")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  message("Building in Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
else()
  message("Building in Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
endif()

message("CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")

if(ADAPTER STREQUAL "cyber")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCYBER_ENV")
elseif(ADAPTER STREQUAL "ap")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DAP_ENV")
endif()

set(PYTHON_EXECUTABLE /root/miniconda3/bin/python)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-DDEBUG_PRINT_ENABLE)
add_subdirectory(proto)
add_subdirectory(src)
#add_subdirectory(ad_common)

if (UNIT_TEST_ENABLE)
  add_subdirectory(test)
endif()

if (PP_ENABLE)
  execute_process(COMMAND pip install empy==3.3.4
                    RESULT_VARIABLE result
                    OUTPUT_VARIABLE output
                    ERROR_VARIABLE error)
  if(NOT STRUCT_MSGS_ADDED)
    add_subdirectory(${PROJECT_SOURCE_DIR}/interface/type_convert/struct_msgs)
    set(STRUCT_MSGS_ADDED TRUE CACHE INTERNAL "Flag to indicate that subdir has been added")
  endif()
  add_subdirectory(${PROJECT_SOURCE_DIR}/interface/type_convert/proto_msgs)
  add_subdirectory(${PROJECT_SOURCE_DIR}/interface/type_convert/sensor_interface)
  add_subdirectory(tools/planning_player)
endif()

if (PYBIND_TOOL_ENABLE)
  execute_process(COMMAND pip install empy==3.3.4
                    RESULT_VARIABLE result
                    OUTPUT_VARIABLE output
                    ERROR_VARIABLE error)
  if(NOT STRUCT_MSGS_ADDED)
    add_subdirectory(${PROJECT_SOURCE_DIR}/interface/type_convert/struct_msgs)
    set(STRUCT_MSGS_ADDED TRUE CACHE INTERNAL "Flag to indicate that subdir has been added")
  endif()
  add_subdirectory(jupyter_pybind)
endif()
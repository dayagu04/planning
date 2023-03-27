cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)
project (mjson)

if (TARGET ${PROJECT_NAME})
  # pass
else()
  add_library(${PROJECT_NAME} INTERFACE)
  target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
endif()

# if project is root project, compile and archive static library
if (${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  add_subdirectory(sample)
endif()

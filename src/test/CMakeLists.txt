cmake_minimum_required (VERSION 3.0)
set (CMAKE_CXX_STANDARD 14)
project (test)
message(STATUS ">>> [${PROJECT_NAME}] @ ${PROJECT_SOURCE_DIR}/CMakeLists.txt")

add_subdirectory (unit_tests)

set (LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
project(unittest)
message(STATUS ">>> [${PROJECT_NAME}] @ ${PROJECT_SOURCE_DIR}/MakeLists.txt")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/thirdparty/cmake/modules)
file(GLOB_RECURSE TEST_CPP_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cc)

add_definitions(-DPROJECT_UNITTEST_DATA_DIR="${PROJECT_SOURCE_DIR}/data")

add_executable(${PROJECT_NAME} ${TEST_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
    ${CONAN_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    npp
    ${TORCH_LIBRARIES}
    planning_define
    osqpstatic
    gtest
    gtest_main
    gmock
    gmock_main
    yaml-cpp
    dl
    rt
    pthread
    ${CONAN_LIBS}
)

if (RUN_ON_CPU)
  target_include_directories(${PROJECT_NAME} PRIVATE ${LIBTORCH_PYTHON_INCLUDE_DIRS})
  target_link_libraries (${PROJECT_NAME} PRIVATE ${TORCH_LIBRARIES})
endif ()

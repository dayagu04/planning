cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)
project(unit_test)
message(STATUS ">>> [${PROJECT_NAME}] @ ${PROJECT_SOURCE_DIR}/MakeLists.txt")

# 递归遍历所有.cc文件
# file(GLOB_RECURSE TEST_CPP_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cc)
# file(GLOB_RECURSE TEST_CPP_FILES ${CMAKE_CURRENT_LIST_DIR}/src/scenario_state_machine_test.cc)
file(GLOB_RECURSE TEST_CPP_FILES ${CMAKE_CURRENT_LIST_DIR}/src/scenario_state_machine_test.cc)
# CMAKE_SOURCE_DIR: 主cmakelist所在路径
file(GLOB_RECURSE SRC_MATH ${CMAKE_SOURCE_DIR}/src/modules/common/math/*.cc)
file(GLOB_RECURSE SRC_UTILS ${CMAKE_SOURCE_DIR}/src/modules/common/utils/*.cc)
file(GLOB_RECURSE SRC_BEHAVIOR_TASK ${CMAKE_SOURCE_DIR}/src/modules/tasks/*.cc)
file(GLOB_RECURSE SRC_SCC_FUNC ${CMAKE_SOURCE_DIR}/src/modules/scc_function/*.cc)
file(GLOB_RECURSE SRC_CONTEXT ${CMAKE_SOURCE_DIR}/src/modules/context/*.cc)
file(GLOB_RECURSE SRC_SCENARIO ${CMAKE_SOURCE_DIR}/src/modules/scenario/*.cc)

# proto files
# file(GLOB_RECURSE SRC_PROTO ${CMAKE_SOURCE_DIR}/src/modules/proto/*.cc)
# file(GLOB_RECURSE SRC_PROTO_LIB ${CMAKE_SOURCE_DIR}../res/include/proto/*.cc)
SUBDIRLIST_SRC(${CMAKE_SOURCE_DIR}/proto SRC_PROTO)
SUBDIRLIST_SRC(${CMAKE_SOURCE_DIR}/../res/include/proto SRC_PROTO_LIB)

set(SRC_FILES
  ${CMAKE_SOURCE_DIR}/src/modules/context/reference_path.cc
  ${CMAKE_SOURCE_DIR}/src/modules/context/frenet_ego_state.cc
  # ${CMAKE_SOURCE_DIR}/src/modules/context/obstacle_manager.cc
)

add_executable(${PROJECT_NAME} ../main_test.cc
                              ${TEST_CPP_FILES}
                              ${SRC_MATH}
                              ${SRC_UTILS}
                              ${SRC_BEHAVIOR_TASK}
                              ${SRC_SCC_FUNC}
                              ${SRC_CONTEXT}
                              ${SRC_SCENARIO}
                              ${SRC_PROTO}
                              ${SRC_PROTO_LIB}
                              )

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
    cyber
    protobuf
    gflags
    autoplt
    fastrtps
    cyber_proto
    gtest
    gtest_main
    gflags
    osqpstatic
    dl
)


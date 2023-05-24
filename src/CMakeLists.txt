cmake_minimum_required(VERSION 3.0)
# no project name in src

# include directories
set(INCLUDE_DIRS
  # general directories: to be removed
  ${CMAKE_CURRENT_LIST_DIR}/common
  ${CMAKE_CURRENT_LIST_DIR}/framework
  ${CMAKE_CURRENT_LIST_DIR}/modules
  ${CMAKE_CURRENT_LIST_DIR}/apa_simulation

  # detailed directories: all subdirectories ought to be included
  ${CMAKE_CURRENT_LIST_DIR}/modules/tasks/motion_planners/lateral_motion_planner
  ${CMAKE_CURRENT_LIST_DIR}/modules/tasks/motion_planners/lateral_motion_planner/src
  ${CMAKE_CURRENT_LIST_DIR}/modules/tasks/motion_planners/longitudinal_motion_planner
  ${CMAKE_CURRENT_LIST_DIR}/modules/tasks/motion_planners/longitudinal_motion_planner/src
  ${CMAKE_CURRENT_LIST_DIR}/modules/tasks/motion_planners/vision_only_lateral_motion_planner)

# source files
file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/planning_component.cc
  ${CMAKE_CURRENT_LIST_DIR}/common/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/framework/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/modules/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/modules/*.cpp)

file(GLOB_RECURSE APA_SIMULATION_SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/apa_simulation/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/apa_simulation/common/*.cc)

# planning proto
add_subdirectory(proto)

# libraries
add_subdirectory(library/advanced_ctrl_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/ilqr_lib EXCLUDE_FROM_ALL)

# thirdparties
add_subdirectory(thirdparty/mjson EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/eigen EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/rapidjson EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/osqp EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/nlohmann_json EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/hfsm EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/interface/src/proto EXCLUDE_FROM_ALL)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${INCLUDE_DIRS}
  ${cyber_SOURCE_DIR}
  ${cyber_BINARY_DIR})

# shared libraries
add_library(planning_component SHARED ${SRC_FILES})
target_include_directories(planning_component PUBLIC ${INCLUDE_DIRS})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  # release for FDC
  target_link_libraries(planning_component PUBLIC
    eigen
    cyber
    protobuf
    proto_interface
    gflags
    autoplt
    osqpstatic
    advanced_ctrl_lib
    ilqr_lib
    mjson
    rapidjson
    nlohmann_json
    hfsm
    planning_proto
    NanoLog
  )
else()
  # debug for X86
  target_link_libraries(planning_component PUBLIC
    eigen
    cyber
    protobuf
    proto_interface
    gflags
    autoplt
    osqpstatic
    advanced_ctrl_lib
    ilqr_lib
    mjson
    rapidjson
    nlohmann_json
    hfsm
    planning_proto
  )
endif()

add_library(apa_simulation_component SHARED ${APA_SIMULATION_SRC_FILES})
target_link_libraries(apa_simulation_component PUBLIC
  autoplt
  cyber
  planning_proto
  proto_interface
  protobuf)

target_compile_definitions(planning_component PRIVATE
    CONFIG_PATH="/asw/planning/res/conf")

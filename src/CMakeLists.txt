cmake_minimum_required(VERSION 3.0)
# no project name in src

# settings
add_compile_options(-std=c++17 -Wall -g -O2 -fPIC -fpermissive -DCYBER_ENV -DLINUX)

# include directories
set(INCLUDE_DIRS
  # general directories: to be removed
  ${CMAKE_CURRENT_LIST_DIR}/common
  ${CMAKE_CURRENT_LIST_DIR}/framework
  ${CMAKE_CURRENT_LIST_DIR}/modules
  ${CMAKE_CURRENT_LIST_DIR}/apa_simulation

  # detailed directories: all subdirectories ought to be included
  ${CMAKE_CURRENT_LIST_DIR}/modules/tasks/motion_planners/general_lateral_motion_planner
  ${CMAKE_CURRENT_LIST_DIR}/modules/tasks/motion_planners/general_longitudinal_motion_planner
  ${CMAKE_CURRENT_LIST_DIR}/modules/tasks/motion_planners/vision_only_lateral_motion_planner)

# source files
file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/planning_component.cc
  ${CMAKE_CURRENT_LIST_DIR}/common/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/framework/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/modules/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/modules/*.cpp
  ${CMAKE_CURRENT_LIST_DIR}/apa_simulation/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/apa_simulation/*.cpp)

# planning proto
add_subdirectory(proto)

# thirdparty
add_subdirectory(thirdparty/mjson)
add_subdirectory(thirdparty/eigen)
add_subdirectory(thirdparty/rapidjson)
add_subdirectory(thirdparty/osqp)
add_subdirectory(thirdparty/nlohmann_json)
add_subdirectory(thirdparty/hfsm)
add_subdirectory(thirdparty/interface) # to be replaced by submodule

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${INCLUDE_DIRS}
  ${cyber_SOURCE_DIR}
  ${cyber_BINARY_DIR})

# shared libraries
add_library(planning_component SHARED ${SRC_FILES})
target_include_directories(planning_component PUBLIC ${INCLUDE_DIRS})

target_link_libraries(planning_component PUBLIC
  eigen
  cyber
  protobuf
  proto_bsw
  gflags
  autoplt
  osqp
  mjson
  rapidjson
  nlohmann_json
  hfsm
  planning_proto)

target_compile_definitions(planning_component PRIVATE
    CONFIG_PATH="/asw/Planning/Res/config")

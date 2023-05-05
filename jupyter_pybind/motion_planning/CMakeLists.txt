cmake_minimum_required(VERSION 3.2)
project(jupyter_pybind)
set(CMAKE_CXX_STANDARD 14)
set(PYBIND11_PYTHON_VERSION 3.9)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED PATHS /usr/local/lib/python3.9/site-packages/pybind11/share/cmake/pybind11)


set(INCLUDE_DIRS
    ../../src/modules/tasks/motion_planners/lateral_motion_planner/src
    ../../src/modules/tasks/motion_planners/longitudinal_motion_planner/src
    ../../src/library/advanced_ctrl_lib/include
    ../../src/library/ilqr_lib/include
    src/lateral_control/)

file(GLOB_RECURSE SRC_FILES
    ../../src/modules/tasks/motion_planners/lateral_motion_planner/src/*.cc
    ../../src/modules/tasks/motion_planners/longitudinal_motion_planner/src/*.cc
    ../../src/library/advanced_ctrl_lib/src/*.cpp
    ../../src/library/ilqr_lib/src/*.cpp
    src/lateral_control/*.cpp)

include_directories(
    ${INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_subdirectory(src/acado)

# add libraries
add_library(${PROJECT_NAME} STATIC ${SRC_FILES})
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)
target_link_libraries(${PROJECT_NAME} PUBLIC acado_lib)

pybind11_add_module(mpc_common MODULE src/acado_lat_mpc.py.cpp)
target_link_libraries(mpc_common PRIVATE ${PROJECT_NAME})

pybind11_add_module(acado_lat_mpc_py MODULE src/acado_lat_mpc.py.cpp)
target_link_libraries(acado_lat_mpc_py PRIVATE ${PROJECT_NAME})

pybind11_add_module(ilqr_lat_mpc MODULE src/ilqr_lat_mpc.py.cpp)
target_link_libraries(ilqr_lat_mpc PRIVATE ${PROJECT_NAME})

pybind11_add_module(lateral_motion_planning_py MODULE src/lateral_motion_planning.py.cpp)
target_link_libraries(lateral_motion_planning_py PRIVATE ${PROJECT_NAME})

pybind11_add_module(longitudinal_motion_planning_py MODULE src/longitudinal_motion_planning.py.cpp)
target_link_libraries(longitudinal_motion_planning_py PRIVATE ${PROJECT_NAME})
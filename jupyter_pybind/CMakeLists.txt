project(jupyter_pybind)
cmake_minimum_required(VERSION 3.2)

set(CMAKE_CXX_STANDARD 14)
set(PYBIND11_PYTHON_VERSION 3.7)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DX86")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -DX86")

find_package(pybind11 REQUIRED PATHS ~/miniconda3/lib/python3.7/site-packages/pybind11/share/cmake/pybind11)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/../interface/src
  ${CMAKE_CURRENT_LIST_DIR}/../interface/type_convert
  ${CMAKE_CURRENT_LIST_DIR}/../build/devel/include
)

install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../build/devel/lib/python3/dist-packages/struct_msgs
  DESTINATION ${CMAKE_CURRENT_LIST_DIR}/../jupyter_pybind/notebooks_apa/scripts
)
find_package(OpenCV REQUIRED)

pybind11_add_module(lateral_motion_planning_py MODULE src/lateral_motion_planning_pybind.cc)
target_link_libraries(lateral_motion_planning_py PRIVATE
  autoplt
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(longitudinal_motion_planning_py MODULE src/longitudinal_motion_planning_pybind.cc)
target_link_libraries(longitudinal_motion_planning_py PRIVATE
  autoplt
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(scc_lon_motion_planning_py MODULE src/scc_lon_motion_planning_pybind.cc)
target_link_libraries(scc_lon_motion_planning_py PRIVATE
  autoplt
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(dubins_lib_py MODULE src/dubins_lib_pybind.cc)
target_link_libraries(dubins_lib_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(collision_detection_py MODULE src/collision_detection_pybind.cc)
target_link_libraries(collision_detection_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(slant_planning_py MODULE src/slant_planning_pybind.cc)
target_link_libraries(slant_planning_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(perpendicular_parking_out_py MODULE src/perpendicular_park_out_pybind.cc)
target_link_libraries(perpendicular_parking_out_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(parallel_planning_py MODULE src/parallel_planning_pybind.cc)
target_link_libraries(parallel_planning_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(parallel_planning_out_py MODULE src/parallel_planning_out_pybind.cc)
target_link_libraries(parallel_planning_out_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(parallel_preparing_planning_py MODULE src/parallel_preparing_plan_pybind.cc)
target_link_libraries(parallel_preparing_planning_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(uss_obstacle_avoidance_py MODULE src/uss_obstacle_avoidance_pybind.cc)
target_link_libraries(uss_obstacle_avoidance_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(apa_simulation_py MODULE src/apa_simulation_pybind.cc)
target_link_libraries(apa_simulation_py PRIVATE
  ${catkin_LIBRARIES}
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface
  common
)

pybind11_add_module(replay_simulation_hybrid_astar MODULE src/apa_replay_simulation_hybrid_astar.cc)
target_link_libraries(replay_simulation_hybrid_astar PRIVATE
  ${catkin_LIBRARIES}
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  ad_common
  hybrid_astar_lib
  reeds_shepp
  occupancy_grid_map
  ${OpenCV_LIBRARIES}
  spiral
  )

pybind11_add_module(geometry_math_validation_py MODULE src/geometry_math_validation_pybind.cc)
target_link_libraries(geometry_math_validation_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(lateral_path_optimizer_py MODULE src/lateral_path_optimizer_pybind.cc)
target_link_libraries(lateral_path_optimizer_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(optimizer_apa_simulation_py MODULE src/optimizer_apa_simulation_pybind.cc)
target_link_libraries(optimizer_apa_simulation_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(jerk_limited_trajectory_py MODULE src/jerk_limited_trajectory_pybind.cc)
target_link_libraries(jerk_limited_trajectory_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(sample_poly_speed_adjust_decider_py MODULE src/quartic_curve_sample_pybind.cc)
target_link_libraries(sample_poly_speed_adjust_decider_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  legacy_c_interface)

pybind11_add_module(gap_selector_py MODULE src/gap_selector_pybind.cc)
target_link_libraries(gap_selector_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  legacy_c_interface
  modules_tasks)

pybind11_add_module(hybrid_astar_py MODULE src/hybrid_astar_path_generator_pybind.cc)
target_link_libraries(hybrid_astar_py PRIVATE
  autoplt
  ad_common
  modules
  modules_apa_function
  modules_common
  modules_context
  modules_tasks
  hybrid_astar_lib
  reeds_shepp
  occupancy_grid_map
  ${OpenCV_LIBRARIES}
  spiral
  )

pybind11_add_module(hybrid_astar_out_py MODULE src/hybrid_astar_out_path_generator_pybind.cc)
target_link_libraries(hybrid_astar_out_py PRIVATE
  autoplt
  ad_common
  modules
  modules_apa_function
  modules_common
  modules_context
  modules_tasks
  hybrid_astar_lib
  reeds_shepp
  occupancy_grid_map
  ${OpenCV_LIBRARIES}
  spiral
  )

pybind11_add_module(astar_parallel_py MODULE src/astar_parallel_pybind.cc)
target_link_libraries(astar_parallel_py PRIVATE
  autoplt
  ad_common
  modules
  modules_apa_function
  modules_common
  modules_context
  modules_tasks
  hybrid_astar_lib
  reeds_shepp
  occupancy_grid_map
  ${OpenCV_LIBRARIES}
)

pybind11_add_module(astar_parallel_replay_py MODULE src/astar_parallel_pybind_replay.cc)
target_link_libraries(astar_parallel_replay_py PRIVATE
  ${catkin_LIBRARIES}
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks
  ad_common
  hybrid_astar_lib
  reeds_shepp
  occupancy_grid_map
  ${OpenCV_LIBRARIES}
  )

pybind11_add_module(headin_planning_py MODULE src/headin_planning_pybind.cc)
target_link_libraries(headin_planning_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(perpendicular_slant_tail_in_py MODULE src/perpendicular_slant_tail_in_pybind.cc)
  target_link_libraries(perpendicular_slant_tail_in_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(perpendicular_slant_tail_in_with_json_py MODULE src/perpendicular_slant_tail_in_with_json_pybind.cc)
  target_link_libraries(perpendicular_slant_tail_in_with_json_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(scc_lon_motion_planning_v3_py MODULE src/scc_lon_motion_planning_pybind_v3.cc)
  target_link_libraries(scc_lon_motion_planning_v3_py PRIVATE
  autoplt
  modules
  modules_common
  modules_context
  modules_tasks)

pybind11_add_module(spiral_py MODULE src/spiral_pybind.cc)
target_link_libraries(spiral_py PRIVATE
  spiral
# autoplt
# modules
# modules_common
# modules_context
# modules_tasks
)


pybind11_add_module(apa_speed_optimizer_py MODULE src/apa_speed_optimizer_pybind.cc)
target_link_libraries(apa_speed_optimizer_py PRIVATE
  autoplt
  ad_common
  modules
  modules_apa_function
  modules_common
  modules_context
  modules_tasks
  hybrid_astar_lib
  )

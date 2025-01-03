project(hybrid_astar_lib)

file(GLOB hybrid_astar_file
    "*.cc"
)
message(${hybrid_astar_file})

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${hybrid_astar_file})
target_compile_options(${PROJECT_NAME} PRIVATE -Wunknown-pragmas)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC
  planning_proto
  common
  modules_common
  collision_detection
  reeds_shepp
  occupancy_grid_map
  pthread
  spiral
  )
project(occupancy_grid_map)

file(GLOB euler_distance_transform_file
    "*.cc"
)

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${euler_distance_transform_file})
target_compile_options(${PROJECT_NAME} PRIVATE -Wunknown-pragmas)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME} PUBLIC
  common
  modules_common
  collision_detection
  ${OpenCV_LIBRARIES}
  )
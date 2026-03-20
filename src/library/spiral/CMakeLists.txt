project(spiral)

file(GLOB spiral_file
    "*.cc"
)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${spiral_file})
target_compile_options(${PROJECT_NAME} PRIVATE -Wunknown-pragmas)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  )

target_link_libraries(${PROJECT_NAME} PUBLIC
  common
  modules_common
  collision_detection
  )
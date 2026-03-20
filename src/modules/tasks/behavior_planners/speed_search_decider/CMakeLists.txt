project (st_search_lib)
# cmake_minimum_required (VERSION 3.0)

file(GLOB st_search_lib_file
  "*.cc"
)
message(${st_search_lib_file})
add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})
# add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${hybrid_astar_file})
target_compile_options(${PROJECT_NAME} PRIVATE -Wunknown-pragmas)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC
modules_tasks
modules_context
  )
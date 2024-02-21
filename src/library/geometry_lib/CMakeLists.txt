project(geometry_lib)

file(GLOB_RECURSE CPP_FILES src/*.cc)

add_library (${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} eigen advanced_ctrl_lib)

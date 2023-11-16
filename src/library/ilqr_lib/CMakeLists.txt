project(ilqr_lib)

include_directories(${EIGEN3_INCLUDE_DIRS})

file(GLOB_RECURSE CPP_FILES src/*.cpp)
message(${CPP_FILES})
add_library (${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PRIVATE advanced_ctrl_lib eigen)
project(ilqr_lib)

include_directories(${EIGEN3_INCLUDE_DIRS})

file(GLOB_RECURSE CPP_FILES src/*.cpp)

add_library (${PROJECT_NAME} STATIC ${CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PRIVATE advanced_ctrl_lib eigen)
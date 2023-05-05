project(proto_bsw)

file(GLOB_RECURSE SRC_FILES src/*.cc)
add_library(${PROJECT_NAME} STATIC ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

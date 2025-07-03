project(idm_lib)

file(GLOB_RECURSE CPP_FILES src/*.cpp)

add_library (${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${CPP_FILES})
message(${CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC common)
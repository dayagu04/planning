project (pp)
cmake_minimum_required (VERSION 3.0)

set(CMAKE_CXX_FLAGS "-O2 -DX86 -g")

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

add_executable(${PROJECT_NAME} EXCLUDE_FROM_ALL ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  modules
  cyber
  fastrtps #?
  autoplt
  cyber_proto
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

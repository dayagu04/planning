project (apa_simulation_component)
cmake_minimum_required (VERSION 3.0)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/**.cc
  ${CMAKE_CURRENT_LIST_DIR}/**.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC
  autoplt
  proto_interface
  common
  modules_common
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

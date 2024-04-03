project (planning_proto)
cmake_minimum_required (VERSION 3.0)

find_package(Protobuf REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

file(GLOB PROTOS "*.proto")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOS})

set(WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../jupyter_pybind/scripts/")
set(SH_COMMAND "python proto_gen.py")

execute_process(
  COMMAND sh -c "${SH_COMMAND}"
  RESULT_VARIABLE SH_RESULT
  OUTPUT_VARIABLE SH_OUTPUT
  WORKING_DIRECTORY ${WORKING_DIRECTORY}
)

# 编译proto
add_library(${PROJECT_NAME} STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
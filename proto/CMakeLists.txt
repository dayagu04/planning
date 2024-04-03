project (planning_proto)
cmake_minimum_required (VERSION 3.0)

find_package(Protobuf REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

file(GLOB PROTOS "*.proto")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOS})

# 编译proto
add_library(${PROJECT_NAME} STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
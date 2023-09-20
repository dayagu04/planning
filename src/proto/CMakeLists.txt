project (planning_proto)

find_package(Protobuf REQUIRED)
file(GLOB PROTO_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(${PROJECT_NAME} PUBLIC protobuf::libprotobuf proto_interface)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src)
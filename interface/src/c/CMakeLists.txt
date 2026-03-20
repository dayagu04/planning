find_package(Protobuf REQUIRED)

add_compile_options(-fPIC)

include_directories(${PROTOBUF_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS struct_container.proto)

add_library(c_interface STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(c_interface ${PROTOBUF_LIBRARIES})
target_include_directories(c_interface PUBLIC ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_BINARY_DIR})

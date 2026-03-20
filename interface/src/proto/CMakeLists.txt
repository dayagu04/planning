# proto生成.cc文件和.h文件
find_package(Protobuf REQUIRED)

add_compile_options(-fPIC)

file(GLOB PROTOS "*.proto")

include_directories(${PROTOBUF_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOS})

# 编译proto
add_library(proto_interface STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(proto_interface ${PROTOBUF_LIBRARIES})
target_include_directories(proto_interface PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
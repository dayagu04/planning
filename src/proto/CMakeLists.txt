# 编译参数
cmake_minimum_required(VERSION 2.8.3)
project(planning_proto)
set(CMAKE_CXX_STANDARD 17)

message (STATUS ">>> [${PROJECT_NAME}] @ ${PROJECT_SOURCE_DIR}/MakeLists.txt")

file(GLOB PROTO_FILES
     ${PROJECT_SOURCE_DIR}/*.proto)
message (${PROJECT_SOURCE_DIR})

# Set the root directory to SRC
# and the output path to the folder where the proto files reside
SET(PROTO_BASE_DIR ${PROJECT_SOURCE_DIR}/generated_files)

# Set the source file path
LIST(APPEND PROTO_FLAGS ${CMAKE_CURRENT_SOURCE_DIR})
message("PROTO_FLAGS : ${PROTO_FLAGS}")


# Generate proto files in the 'proto' folders
FOREACH(FIL ${PROTO_FILES})
  message("proto file name: ${FIL}")
  EXECUTE_PROCESS(COMMAND protoc -I=${PROTO_FLAGS} --cpp_out=${PROTO_BASE_DIR} ${FIL})
  message("----------------------------")
ENDFOREACH()

include_directories(${PROJECT_SOURCE_DIR})

# 将proto生成的.cc文件加入PROTO_SRCS
# aux_source_directory(./generated_files PROTO_SRCS)
# 生成动态库
# add_library(${PROJECT_NAME} SHARED ${PROTO_SRCS})



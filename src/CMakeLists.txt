# 编译参数
cmake_minimum_required(VERSION 2.8.3)
project(planning_component)
set(CMAKE_CXX_STANDARD 17)
set(PYBIND11_PYTHON_VERSION 3.9)

#通过查找pybind来判断是在 pybind调试环境还是cyber 环境
find_package(pybind11  PATHS /usr/local/lib/python3.9/site-packages/pybind11/share/cmake/pybind11)
#在pybind 环境
if(pybind11_FOUND)
    message("############# FIND_PYBIND #############")
    message(${pybind11_INCLUDE_DIR})
    include_directories(
      ${pybind11_INCLUDE_DIR}
      /usr/local/include/python3.9
    )
    set(SRCS  ./planning_pybind.cpp)
    message(${SRCS})

#在cyber 环境
else()
    message("############# NOT_FIND_PYBIND !!! #############")
    add_definitions("-DCYBER_ENV")
    include_directories(${cyber_SOURCE_DIR})
    include_directories(${cyber_BINARY_DIR})
    set(SRCS  ./planning_component.cc)
    message(${SRCS})
endif()


if(pybind11_FOUND)
    message("current platform: pybind")
elseif($ENV{AS} MATCHES "aarch64-bst-linux-as")
    message("current platform: A1000")
else()
    message("current platform: X86")
    add_definitions("-Wall -g")
    add_compile_options(-O0 -fpermissive)
    add_definitions(-DLINUX)
endif()

# 添加头文件目录
include_directories(.)

# 调用proto以及第三方库中的Cmakelist
add_subdirectory(proto)
add_subdirectory(thirdparty/mjson EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/eigen EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/rapidjson EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/osqp EXCLUDE_FROM_ALL)
# add_subdirectory(common)
# add_subdirectory(planning)

# 添加宏开关
# add_definitions("-DNDEBUG")

# 遍历源文件
# aux_source_directory(${PROJECT_SOURCE_DIR}/../test SRC_TEST)
SUBDIRLIST_SRC(${PROJECT_SOURCE_DIR}/framework SRC_FRAMEWORK)
SUBDIRLIST_SRC(${PROJECT_SOURCE_DIR}/common SRC_COMMON)
SUBDIRLIST_SRC(${PROJECT_SOURCE_DIR}/modules SRC_MODULES)
SUBDIRLIST_SRC(${PROJECT_SOURCE_DIR}/proto SRC_PROTO)
SUBDIRLIST_SRC(${PROJECT_SOURCE_DIR}/../res/include/proto SRC_PROTO_LIB)
SUBDIRLIST_SRC(${PROJECT_SOURCE_DIR}/apa_simulation SRC_APA_SIMULATION)

# 组件间通讯协议
# link_directories(../lib/protolib)

find_package(Eigen3 REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

# 生成动态库
add_library(${PROJECT_NAME} SHARED
            ${SRCS}
            ${SRC_COMMON}
            ${SRC_FRAMEWORK}
            ${SRC_MODULES}
            ${SRC_PROTO}
            ${SRC_PROTO_LIB}
            )

add_library(apa_simulation_component SHARED
            ${SRC_APA_SIMULATION}
            ${SRC_PROTO_LIB}
            )

#在pybind 环境的链接配置
if(pybind11_FOUND)
    pybind11_add_module(planning_pybind11 planning_pybind.cpp
                        ${SRCS}
                        ${SRC_COMMON}
                        ${SRC_FRAMEWORK}
                        ${SRC_MODULES}
                        ${SRC_PROTO}
                        ${SRC_PROTO_LIB}
                        )
    target_link_libraries(planning_pybind11 PUBLIC
                        protobuf
                        gflags
                        eigen
                        dl
                        pthread
                        mjson
                        rapidjson
                        )

#在Cyber 环境的链接配置
else()
    # A1000
    if($ENV{AS} MATCHES "aarch64-bst-linux-as")
        target_link_libraries(${PROJECT_NAME} PUBLIC
                              eigen
                              cyber
                              protobuf
                              gflags
                              autoplt
                              NanoLog
                              )
        target_link_libraries(apa_simulation_component PUBLIC
                              cyber
                              protobuf
                              autoplt
                              )
    # X86
    else()
        target_link_libraries(${PROJECT_NAME} PUBLIC
                              eigen
                              cyber
                              protobuf
                              gflags
                              autoplt
                              osqp
                              mjson
                              )
        target_link_libraries(apa_simulation_component PUBLIC
                              cyber
                              protobuf
                              autoplt
                              )
    endif()
endif()

# -----------GTEST 单元测试-----------
option (PNC_ENABLE_TEST "build unit tests" ON)
message (STATUS "PNC_ENABLE_TEST=" ${PNC_ENABLE_TEST})
if (PNC_ENABLE_TEST)
  set(CMAKE_BUILD_TYPE Debug)
  message (STATUS "include googletets and enable test")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0")

  add_subdirectory (test)
endif ()

target_compile_definitions(${PROJECT_NAME} PRIVATE
    CONFIG_PATH="/asw/Planning/Res/config")

# 编译参数
cmake_minimum_required(VERSION 2.8.3)
project(planning_component)
set(CMAKE_CXX_STANDARD 17)
set(PYBIND11_PYTHON_VERSION 3.9)
add_definitions("-Wall -g")
add_compile_options(-O0 -fpermissive )
if($ENV{AS} MATCHES "aarch64-bst-linux-as")
	message("current platform: A1000")
else()
	message("current platform: linux")
  add_definitions(-DLINUX)
endif()

#ros相关
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)
catkin_package(
CATKIN_DEPENDS
roscpp
message_runtime
std_msgs
)

include_directories(/opt/ros/melodic/include)
link_directories(/usr/lib/x86_64-linux-gnu/)
add_subdirectory(proto_msgs)
include_directories(${PROJECT_SOURCE_DIR}/build/devel/include)
include_directories(${PROJECT_SOURCE_DIR}/old_include)
file(GLOB_RECURSE OLD_PROTO ${PROJECT_SOURCE_DIR}/old_include/*.cc)

MACRO(SUBDIRLIST_SRC DIR SRC_LIST)
  IF(IS_DIRECTORY ${DIR})
    message("The DIRs are :")
    message(${DIR})
    include_directories(${DIR})
    FILE(GLOB SRC_TEMP ${DIR}/*.cc ${DIR}/*.cpp)
    FOREACH(SRCS_SUB ${SRC_TEMP})
      LIST(APPEND SRC_LIST_ ${SRCS_SUB})
    ENDFOREACH()
    FILE(GLOB DIRS ${DIR}/*)
    FOREACH(SUB ${DIRS})
      SUBDIRLIST_SRC(${SUB} SRC_LIST_)
    ENDFOREACH()
  ENDIF()
  SET(${SRC_LIST} ${SRC_LIST_})
ENDMACRO()
# SUBDIRLIST_SRC(${PROJECT_SOURCE_DIR}/../src/module SRC_COMON)
# SUBDIRLIST_SRC(${PROJECT_SOURCE_DIR}/../src/planning SRC_PLANNING)
# 添加头文件目录
include_directories(${PROJECT_SOURCE_DIR}/../src)
include_directories(${PROJECT_SOURCE_DIR}/../)
include_directories(${cyber_SOURCE_DIR})
include_directories(${cyber_BINARY_DIR})

# 组件间通讯协议
link_directories(../build/src/)

find_package(Eigen3 REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)
message(${catkin_LIBRARIES})
# # test
add_executable(planning_main_test planning_main_test.cc ${OLD_PROTO})
target_link_libraries(planning_main_test PUBLIC
                    ${PROJECT_NAME}
                    cyber
                    protobuf
                    gflags
                    autoplt
                    fastrtps
                    cyber_proto
                    ${catkin_LIBRARIES}
          )
target_compile_definitions(planning_main_test PRIVATE
    CONFIG_PATH="/asw/Planning/Res/config")




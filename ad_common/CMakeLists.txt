project(ad_common)
cmake_minimum_required(VERSION 3.0)

option(BUILD_TEST "build test or not" OFF)

if (BUILD_TEST)
    add_definitions(-DBUILD_TEST)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fPIC -fpermissive")
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE SD_FILES 
  ${CMAKE_CURRENT_LIST_DIR}/sdmap/*.cpp
  ${CMAKE_CURRENT_LIST_DIR}/sdmap/*.cc
)

file(GLOB_RECURSE SDPRO_FILES 
  ${CMAKE_CURRENT_LIST_DIR}/sdpromap/*.cpp
  ${CMAKE_CURRENT_LIST_DIR}/sdpromap/*.cc
  ${CMAKE_CURRENT_LIST_DIR}/sdpromap/*.hpp
)

add_library(ad_common STATIC
  hdmap/hdmap_impl.cc
  hdmap/hdmap_lane_boundary_info.cc
  hdmap/hdmap_lane_boundary_info.cc
  hdmap/hdmap_lane_info.cc
  hdmap/hdmap_utils.cc
  hdmap/hdmap.cc
  math/aabox2d.cc
  math/box2d.cc
  math/line_segment2d.cc
  math/linear_interpolation.cc
  math/math_utils.cc
  math/polygon2d.cc
  math/vec2d.cc
  decrypt_encrypt/E541_decrypt_encrypt.cc
  decrypt_encrypt/M32T_decrypt_encrypt.cc
  ${SD_FILES}
  ${SDPRO_FILES}
)

if (BUILD_TEST)
  set(INTERFACE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../interface")
  add_subdirectory(${INTERFACE_PATH} ifly_interface.out)
  add_subdirectory(${INTERFACE_PATH}/rosmsg/sensor_interface sensor_interface.out)
  add_subdirectory(${INTERFACE_PATH}/rosmsg/struct_msgs struct_msgs.out)
  find_package(catkin REQUIRED COMPONENTS
      rosbag
      roscpp
      rospy
      std_msgs
      message_generation
      sensor_interface
      struct_msgs
  )
  include_directories(
      ${catkin_INCLUDE_DIRS}
      ${INTERFACE_PATH}/src
      ${INTERFACE_PATH}/src/proto
      ${INTERFACE_PATH}/type_convert
      ${INTERFACE_PATH}/rosmsg
      ${IFLYX_DATA_HUB_PATH}/iflyx_data_hub/interface
  )
  include_directories(
        ${INTERFACE_PATH}/src
  )
  add_executable(sdpromap_test 
    test/sdpromap_test.cpp 
    ${SDPRO_FILES}
    math/aabox2d.cc
    math/box2d.cc
    math/line_segment2d.cc
    math/linear_interpolation.cc
    math/math_utils.cc
    math/polygon2d.cc
    math/vec2d.cc
    )
    add_dependencies(sdpromap_test ${catkin_EXPORTED_TARGETS})
    add_dependencies(sdpromap_test
      sensor_interface_generate_messages_cpp
      struct_msgs_generate_messages_cpp
    )
  target_link_libraries(sdpromap_test 
    protobuf  
    pthread
    proto_interface
    c_interface
    ${catkin_LIBRARIES}
  )
  target_include_directories(sdpromap_test PUBLIC 
    ${CMAKE_CURRENT_LIST_DIR}/../ 
    ${CMAKE_CURRENT_LIST_DIR} 
    ${CMAKE_CURRENT_LIST_DIR}/sdpromap
    ${EIGEN3_INCLUDE_DIRS}
  )

  # VIN 解析测试可执行文件（读取一行 VehicleId 输出 VIN）- 仅在 BESTUNE_E541 产品中编译
  if(PRODUCT STREQUAL "BESTUNE_E541")
    add_executable(E541_decrypt_encrypt
      test/E541_test_decrypt_encrypt.cc
      decrypt_encrypt/E541_decrypt_encrypt.cc
    )
    target_include_directories(E541_decrypt_encrypt PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/../
      ${CMAKE_CURRENT_LIST_DIR}
      ${CMAKE_CURRENT_LIST_DIR}/decrypt_encrypt  # 包含本地 OpenSSL 头文件路径
    )
    # 查找并链接 OpenSSL 库（运行时需要）
    find_package(OpenSSL QUIET)
    if(OpenSSL_FOUND)
      target_link_libraries(E541_decrypt_encrypt PRIVATE
        ${OPENSSL_LIBRARIES}
      )
      # 如果系统有 OpenSSL，也添加系统路径（优先级低于本地路径）
      target_include_directories(E541_decrypt_encrypt PRIVATE
        ${OPENSSL_INCLUDE_DIR}
      )
    else()
      # 如果找不到系统 OpenSSL，尝试链接标准库名
      target_link_libraries(E541_decrypt_encrypt PRIVATE
        ssl
        crypto
      )
    endif()
  endif()

  # VIN 解析测试可执行文件（读取一行 VehicleId 输出 VIN）- 仅在 CHERY_M32T 产品中编译
  if(PRODUCT STREQUAL "CHERY_M32T")
    add_executable(M32T_decrypt_encrypt
      test/M32T_test_decrypt_encrypt.cc
      decrypt_encrypt/M32T_decrypt_encrypt.cc
    )
    target_include_directories(M32T_decrypt_encrypt PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}/../
      ${CMAKE_CURRENT_LIST_DIR}
      ${CMAKE_CURRENT_LIST_DIR}/decrypt_encrypt
    )
  endif()
endif()

target_link_libraries(ad_common PUBLIC
  proto_interface
)

target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_LIST_DIR}/../
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/decrypt_encrypt
  ${EIGEN3_INCLUDE_DIRS}
)

find_package(OpenSSL QUIET)
if(OpenSSL_FOUND)
  target_link_libraries(ad_common PUBLIC
    ${OPENSSL_LIBRARIES}
  )
  target_include_directories(${PROJECT_NAME} PUBLIC
    ${OPENSSL_INCLUDE_DIR}
  )
else()
  target_link_libraries(ad_common PUBLIC
    ssl
    crypto
  )
endif()

# add_executable(hdmap_impl_test test/hdmap_impl_test.cc)
# target_link_libraries(hdmap_impl_test
#   ad_common
#   gtest
#   gflags
# )

# file(MAKE_DIRECTORY /asw/ad_common/test_data)
# file(MAKE_DIRECTORY /asw/ad_common/Lib)
# install(TARGETS
#   hdmap_impl_test
#   RUNTIME DESTINATION /asw/ad_common/Lib)
# install(FILES ${CMAKE_CURRENT_LIST_DIR}/test_data/road_map.pb.txt
#   DESTINATION /asw/ad_common/test_data)

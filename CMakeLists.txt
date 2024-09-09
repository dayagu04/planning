project(planning)
cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 17)
set(PYTHON_EXECUTABLE /root/miniconda3/bin/python)
if (PP_ENABLE OR PYBIND_TOOL_ENABLE)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}/install")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-DDEBUG_PRINT_ENABLE)
add_subdirectory(proto)
add_subdirectory(src)

if (UNIT_TEST_ENABLE)
  add_subdirectory(test)
endif()

if (PP_ENABLE OR PYBIND_TOOL_ENABLE)
  execute_process(COMMAND pip install empy==3.3.4
                    RESULT_VARIABLE result
                    OUTPUT_VARIABLE output
                    ERROR_VARIABLE error)
  include_directories(${CMAKE_CURRENT_LIST_DIR}/interface/type_convert)
  include_directories(${CMAKE_CURRENT_LIST_DIR}/interface/src)

  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/interface/src/c)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/interface/src/proto)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/interface/src/legacy)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/interface/type_convert/struct_msgs struct_msgs.out)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/interface/type_convert/struct_msgs_legacy/interface2.4.5 interface2.4.5.out)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/interface/type_convert/struct_msgs_legacy/interface2.4.6 interface2.4.6.out)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/interface/type_convert/proto_msgs proto_msgs.out)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/interface/type_convert/sensor_interface sensor_interface.out)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/thirdparty/nlohmann_json)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/thirdparty/osqp)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/thirdparty/rapidjson)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/ad_common)
endif()

if (PP_ENABLE)
  add_subdirectory(tools/planning_player)
endif()

if (PYBIND_TOOL_ENABLE)
  add_subdirectory(jupyter_pybind)
endif()
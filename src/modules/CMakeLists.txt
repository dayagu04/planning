project (modules)
cmake_minimum_required(VERSION 3.0)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if(ADAPTER STREQUAL "cyber")
  add_definitions(-DPLANNING_ENGINE_CONFIG_PATH="/asw/planning/res/conf/engine_configs/planning_engine_config_cyber.json")
  message("cyber config")
elseif (ADAPTER STREQUAL "ap")
  add_definitions(-DPLANNING_ENGINE_CONFIG_PATH="${MODULE_RES_DIR}/res/conf/engine_configs/planning_engine_config_ap.json")
  message("ap config")
endif()

add_subdirectory(context EXCLUDE_FROM_ALL)
add_subdirectory(base_function EXCLUDE_FROM_ALL)
add_subdirectory(base_task_pipeline EXCLUDE_FROM_ALL)
add_subdirectory(apa_function EXCLUDE_FROM_ALL)
add_subdirectory(common EXCLUDE_FROM_ALL)
add_subdirectory(adas_function EXCLUDE_FROM_ALL)
add_subdirectory(tasks EXCLUDE_FROM_ALL)
add_subdirectory(hpp_function EXCLUDE_FROM_ALL)
add_subdirectory(scc_function EXCLUDE_FROM_ALL)
add_subdirectory(noa_function EXCLUDE_FROM_ALL)

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/*.cc
)
add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC
  framework
  modules_context
  modules_apa_function
  modules_adas_function
  planning_proto
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

set(ci_dir ${SYSTEM_INTEGRATION_PROJECT_CI_DIR})
if (PP_ENABLE OR PYBIND_TOOL_ENABLE)
  set(ci_dir ${CMAKE_CURRENT_LIST_DIR}/../../.ci)
endif()
include(${ci_dir}/version.cmake)
GenVersionHeaderHere(${ci_dir})

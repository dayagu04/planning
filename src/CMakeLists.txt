cmake_minimum_required(VERSION 3.0)
# no project name in src

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  message (STATUS "Set CMAKE_BUILD_TYPE to default: ${CMAKE_BUILD_TYPE}")
endif()

if(NOT ADAPTER OR ADAPTER STREQUAL "")
  set(ADAPTER "cyber" CACHE STRING "" FORCE)
  message (STATUS "Set ADAPTER to default: ${ADAPTER}")
endif()

if(NOT PLATFORM OR ADAPTER STREQUAL "")
  set(PLATFORM "BZT" CACHE STRING "" FORCE)
  message (STATUS "Set PLATFORM to default: ${PLATFORM}")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fPIC -fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${PLATFORM} -D${PRODUCT}")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  message("Building in Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
else()
  message("Building in Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
endif()

message("CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")

if(ADAPTER STREQUAL "cyber")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCYBER_ENV")
elseif(ADAPTER STREQUAL "ap")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DAP_ENV")
endif()

find_package(Eigen3 REQUIRED)

add_subdirectory(common EXCLUDE_FROM_ALL)
add_subdirectory(modules EXCLUDE_FROM_ALL)
add_subdirectory(framework EXCLUDE_FROM_ALL)

add_subdirectory(library/advanced_ctrl_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/ilqr_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/geometry_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/jlt_lib EXCLUDE_FROM_ALL)

add_subdirectory(thirdparty/mjson EXCLUDE_FROM_ALL)
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/eigen thirdparty/eigen.out EXCLUDE_FROM_ALL)
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/rapidjson thirdparty/rapidjson.out EXCLUDE_FROM_ALL)
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/osqp thirdparty/osqp.out EXCLUDE_FROM_ALL)
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/nlohmann_json thirdparty/nlohmann_json.out EXCLUDE_FROM_ALL)
#add_subdirectory(${PROJECT_SOURCE_DIR}/interface/src/proto interface.out EXCLUDE_FROM_ALL)
#add_subdirectory(${PROJECT_SOURCE_DIR}/interface/src/private/planning interface_planning.out EXCLUDE_FROM_ALL)
#add_subdirectory(${PROJECT_SOURCE_DIR}/interface/src/c interface_c.out EXCLUDE_FROM_ALL)

if(ADAPTER STREQUAL "cyber")
  add_subdirectory(adapter/cyber)
elseif(ADAPTER STREQUAL "ap")
  # add_subdirectory(adapter/ap)
endif()

if (ADAPTER STREQUAL "cyber")
  file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning)
  file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning/res)
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/conf
    DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/res)
  install(FILES ${CMAKE_CURRENT_LIST_DIR}/../CHANGELOG.md
      DESTINATION ${PROJECT_SOURCE_DIR}/install/planning)
  install(TARGETS
    planning_component
    LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/Lib)
  install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/../res/dag/planning.dag
    DESTINATION ${PROJECT_SOURCE_DIR}/install/planning)
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/scripts
    DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/res)
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/conf/${PRODUCT}/
    DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/res/conf)
  install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/../res/conf/${PRODUCT}/apa_params.json
    DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/res)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning
    DESTINATION /asw)
elseif (ADAPTER STREQUAL "ap")
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/conf
    DESTINATION res)
  install(FILES ${CMAKE_CURRENT_LIST_DIR}/../CHANGELOG.md
      DESTINATION .)
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/scripts
    DESTINATION res)
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/conf/${PRODUCT}/
    DESTINATION res/conf)
  install(FILES ${CMAKE_CURRENT_LIST_DIR}/../res/conf/${PRODUCT}/apa_params.json
    DESTINATION res)
endif()

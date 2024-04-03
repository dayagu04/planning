cmake_minimum_required(VERSION 3.0)
# no project name in src

add_subdirectory(common EXCLUDE_FROM_ALL)
add_subdirectory(modules EXCLUDE_FROM_ALL)
add_subdirectory(framework EXCLUDE_FROM_ALL)

add_subdirectory(library/advanced_ctrl_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/ilqr_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/geometry_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/jlt_lib EXCLUDE_FROM_ALL)

add_subdirectory(thirdparty/mjson EXCLUDE_FROM_ALL)
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/eigen thirdparty/eigen.out EXCLUDE_FROM_ALL)
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/rapidjson thirdparty/rapidjson.out EXCLUDE_FROM_ALL)
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/osqp thirdparty/osqp.out EXCLUDE_FROM_ALL)
add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/nlohmann_json thirdparty/nlohmann_json.out EXCLUDE_FROM_ALL)
add_subdirectory(${PROJECT_SOURCE_DIR}/interface/src/proto interface.out EXCLUDE_FROM_ALL)
# add_subdirectory(${PROJECT_SOURCE_DIR}/interface/src/private/planning interface_planning.out EXCLUDE_FROM_ALL)
add_subdirectory(${PROJECT_SOURCE_DIR}/interface/src/c interface_c.out EXCLUDE_FROM_ALL)

if(ADAPTER STREQUAL "cyber")
  add_subdirectory(adapter/cyber)
elseif(ADAPTER STREQUAL "ap")
  # add_subdirectory(adapter/ap)
endif()

file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning)
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning/res)
#file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning/log)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/conf
  DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/res)
install(FILES ${CMAKE_CURRENT_LIST_DIR}/../CHANGELOG.md
    DESTINATION ${PROJECT_SOURCE_DIR}/install/planning)
if (ADAPTER STREQUAL "cyber")
  install(TARGETS
    planning_component
    LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/Lib)
  install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/../res/dag/planning.dag
    DESTINATION ${PROJECT_SOURCE_DIR}/install/planning)
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/scripts
    DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/res)
elseif (ADAPTER STREQUAL "ap")
  # to be add
endif()

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/conf/${PRODUCT}/
  DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/res/conf)

install(FILES
  ${CMAKE_CURRENT_LIST_DIR}/../res/conf/${PRODUCT}/apa_params.json
  DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/res)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning
  DESTINATION /asw)

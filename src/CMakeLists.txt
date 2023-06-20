cmake_minimum_required(VERSION 3.0)
# no project name in src

if(ADAPTER STREQUAL "cyber")
  add_subdirectory(adapter/cyber)
  add_subdirectory(apa_simulation)
elseif(ADAPTER STREQUAL "ap")
  # add_subdirectory((adapter/ap)
endif()

add_subdirectory(proto EXCLUDE_FROM_ALL)
add_subdirectory(common EXCLUDE_FROM_ALL)
add_subdirectory(modules EXCLUDE_FROM_ALL)
add_subdirectory(framework EXCLUDE_FROM_ALL)

add_subdirectory(library/advanced_ctrl_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/ilqr_lib EXCLUDE_FROM_ALL)

add_subdirectory(thirdparty/mjson EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/eigen EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/rapidjson EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/osqp EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/nlohmann_json EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/hfsm EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/interface/src/proto EXCLUDE_FROM_ALL)

file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning)
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning/res)
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning/log)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/conf
  DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/res)
install(FILES ${CMAKE_CURRENT_LIST_DIR}/../CHANGELOG.md
    DESTINATION ${PROJECT_SOURCE_DIR}/../install/planning/log)
if (ADAPTER STREQUAL "cyber")
  install(TARGETS
    apa_simulation_component
    planning_component
    LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/Lib)
  install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/../res/dag/apa_simulation.dag
    ${CMAKE_CURRENT_LIST_DIR}/../res/dag/apa_simulation_and_planning.dag
    ${CMAKE_CURRENT_LIST_DIR}/../res/dag/planning_and_control.dag
    ${CMAKE_CURRENT_LIST_DIR}/../res/dag/planning.dag
    DESTINATION ${PROJECT_SOURCE_DIR}/install/planning)
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/scripts
    DESTINATION ${PROJECT_SOURCE_DIR}/install/planning/res)
elseif (ADAPTER STREQUAL "ap")
  # to be add
endif()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/install/planning
  DESTINATION /asw)

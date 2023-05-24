project(planning)
cmake_minimum_required(VERSION 3.0)

# set release mode
# Release ON: FDC, OFF: X86
set(RELEASE_ENABLE OFF)

# set unit test
set(UNIT_TEST_ENABLE OFF)

# install config
set(INSTALL_TO_CURRENT ON)
set(INSTALL_TO_SYSTEM ON)

# settings
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fPIC -fpermissive -DCYBER_ENV")

if(RELEASE_ENABLE)
  # release mode
  message("Building in Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
  add_subdirectory(src)
else()
  # debug mode
  message("Building in Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DX86")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
  add_subdirectory(src)
  if (UNIT_TEST_ENABLE)
    # unit test needs -O0
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -DX86")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
    add_subdirectory(test)
  endif()
endif()

# install to current directory
if (INSTALL_TO_CURRENT)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/install/Lib)
  file(MAKE_DIRECTORY /asw/${PROJECT_NAME}/res/conf)

  install(TARGETS ${PROJECT_NAME}_component
    LIBRARY DESTINATION ${CMAKE_CURRENT_LIST_DIR}/install/Lib)
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/res/conf
    DESTINATION ${CMAKE_CURRENT_LIST_DIR}/install/res)
  install(FILES ${CMAKE_CURRENT_LIST_DIR}/res/dag/${PROJECT_NAME}.dag
    DESTINATION ${CMAKE_CURRENT_LIST_DIR}/install)
endif()

# install to system
if (INSTALL_TO_SYSTEM)
  file(MAKE_DIRECTORY /asw/${PROJECT_NAME})
  file(MAKE_DIRECTORY /asw/${PROJECT_NAME}/log)
  file(MAKE_DIRECTORY /asw/${PROJECT_NAME}/res/conf)

  install(TARGETS
    ${PROJECT_NAME}_component
    apa_simulation_component
    LIBRARY DESTINATION /asw/${PROJECT_NAME}/Lib)
  install(FILES
    ${CMAKE_CURRENT_LIST_DIR}/res/dag/apa_simulation_and_planning.dag
    ${CMAKE_CURRENT_LIST_DIR}/res/dag/apa_simulation.dag
    ${CMAKE_CURRENT_LIST_DIR}/res/dag/planning_and_control.dag
    ${CMAKE_CURRENT_LIST_DIR}/res/dag/${PROJECT_NAME}.dag
    DESTINATION /asw/${PROJECT_NAME})
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/res/conf
    DESTINATION /asw/${PROJECT_NAME}/res)
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/res/scripts
    DESTINATION /asw/${PROJECT_NAME}/res)
  install(FILES ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.md
    DESTINATION /asw/planning/log)
endif()

project(planning)
cmake_minimum_required(VERSION 3.0)

# set release mode
# set(CMAKE_BUILD_TYPE Release)

# install config
set(INSTALL_TO_CURRENT ON)
set(INSTALL_TO_SYSTEM ON)

# settings
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -fPIC -fpermissive -DCYBER_ENV -DLINUX")

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  message("Building in Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
else()
  message("Building in Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
  add_subdirectory(test)
endif()

# main src
add_subdirectory(src)

# install to current directory
if (INSTALL_TO_CURRENT)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/install/Lib)

  install(TARGETS planning_component
    LIBRARY DESTINATION ${CMAKE_CURRENT_LIST_DIR}/install/Lib)
  install(FILES ${CMAKE_CURRENT_LIST_DIR}/res/dag/planning.dag
    DESTINATION ${CMAKE_CURRENT_LIST_DIR}/install)
endif()

# install to system
if (INSTALL_TO_SYSTEM)
  file(MAKE_DIRECTORY /asw/planning)
  file(MAKE_DIRECTORY /asw/planning/res/conf)

  install(TARGETS planning_component
    LIBRARY DESTINATION /asw/planning/Lib)
  install(FILES ${CMAKE_CURRENT_LIST_DIR}/res/dag/planning.dag
    DESTINATION /asw/planning)
  install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/res/conf
    DESTINATION /asw/planning/res)
endif()

cmake_minimum_required(VERSION 3.0)
# no project name in src

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)

if (PP_ENABLE OR PYBIND_TOOL_ENABLE)
  add_compile_options(-Werror)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT PLATFORM STREQUAL "X86")
  add_definitions(-DENABLE_IFLYTEK_LOG)
  message("ENABLE_IFLYTEK_LOG_SRC_CMAKE")
else()
  message("UNENABLE_IFLYTEK_LOG_SRC_CMAKE")
endif()

add_subdirectory(common EXCLUDE_FROM_ALL)
add_subdirectory(modules EXCLUDE_FROM_ALL)
add_subdirectory(framework EXCLUDE_FROM_ALL)

add_subdirectory(library/advanced_ctrl_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/ilqr_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/geometry_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/jlt_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/hybrid_astar_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/convex_collision_detection EXCLUDE_FROM_ALL)
add_subdirectory(library/reeds_shepp EXCLUDE_FROM_ALL)
add_subdirectory(library/occupancy_grid_map EXCLUDE_FROM_ALL)
add_subdirectory(library/spiral EXCLUDE_FROM_ALL)
add_subdirectory(library/arastar_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/glog EXCLUDE_FROM_ALL)

add_subdirectory(thirdparty/mjson EXCLUDE_FROM_ALL)

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/conf
  DESTINATION ${CMAKE_INSTALL_PREFIX}/res
  PATTERN "product_configs" EXCLUDE)
install(FILES ${CMAKE_CURRENT_LIST_DIR}/../CHANGELOG.md
  DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/scripts
  DESTINATION ${CMAKE_INSTALL_PREFIX}/res)
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/../res/conf/product_configs/${PRODUCT}/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/res/conf)

if (PP_ENABLE OR PYBIND_TOOL_ENABLE OR PLANNING_PLAYER_ENABLE)
  install(DIRECTORY ${CMAKE_INSTALL_PREFIX}/res
    DESTINATION /asw/planning)
endif()

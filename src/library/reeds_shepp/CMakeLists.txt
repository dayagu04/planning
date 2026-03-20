project(reeds_shepp)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL
  reeds_shepp_interface.cc
  rs_path_interpolate.cc
  reeds_shepp.cc
  rs_path_request.cc
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wunknown-pragmas)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC
  # planning_proto
  # common
  # modules_common
  collision_detection
  )


add_executable(reeds_shepp_test reeds_shepp_utest.cc)
target_link_libraries(reeds_shepp_test
  gtest
  gtest_main
  ${PROJECT_NAME}
  )

set(CMAKE_INSTALL_PREFIX "${CMAKE_HOME_DIRECTORY}/build")
install(
        TARGETS
          reeds_shepp_test
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)

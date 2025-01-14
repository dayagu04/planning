project (collision_detection)

## enable optimazition
# add_compile_options(-O3)

file(GLOB collision_detection_file
    "*.cc"
)


add_library(${PROJECT_NAME} STATIC
  EXCLUDE_FROM_ALL
  ${collision_detection_file}
  )
# target_compile_options(collision_detection PRIVATE -fPIC -Wall -Wextra -Wshadow -Wno-missing-field-initializers -Werror  -Wunreachable-code -Wunused -Wno-inconsistent-missing-override -Wno-ignored-qualifiers -Wno-deprecated-copy -Wno-unused-but-set-variable)
target_include_directories(${PROJECT_NAME} PUBLIC
     ${EIGEN3_INCLUDE_DIR}
     ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_CURRENT_SOURCE_DIR}/../../
     )
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ad_common
  common
  )

add_executable(gjk_main_test test/gjk_main_test.cc)
target_link_libraries(gjk_main_test
  gtest
  gtest_main
  ${PROJECT_NAME}
  common
  Eigen3::Eigen
  pthread
  )

add_executable(gjk_interface_test test/gjk_interface_test.cc)
target_link_libraries(gjk_interface_test
  gtest
  gtest_main
  ${PROJECT_NAME}
  common
  Eigen3::Eigen
  pthread
  )

add_executable(polygon_test test/polygon_test.cc)
target_link_libraries(polygon_test
    gtest
    gtest_main
    ${PROJECT_NAME}
    common
    Eigen3::Eigen
    pthread
)

set(CMAKE_INSTALL_PREFIX "${CMAKE_HOME_DIRECTORY}/build")
install(
        TARGETS
          polygon_test
          gjk_interface_test
          gjk_main_test
          ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)







set(BUILD_TYPE "STATIC")

set(OpenCV_DIR /usr/local/share/OpenCV/)
find_package(OpenCV REQUIRED)

add_compile_options(-fPIC)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

include_directories(include
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    )


#  opencv, 相关的wrapper
add_library(opencv_wrapper ${BUILD_TYPE} EXCLUDE_FROM_ALL
                    opencv_viz.cc
                    viz_window.cc
                    viz2d_geometry.cc
                    viz2d_key.cc
                    debug_mode.cc
                    system_state.cc
                    viz2d_perception.cc
                    viz2d_path.cc
                    viz_text.cc
)
target_include_directories(opencv_wrapper PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    ${OpenCV_INCLUDE_DIRS}
    )
target_link_libraries(opencv_wrapper PUBLIC
            ${OpenCV_LIBRARIES}
            ad_common
            common
            collision_detection
            glog
            planning_proto
            )


add_executable(test_viz2d
        ${CMAKE_CURRENT_SOURCE_DIR}/viz_test.cc
        )
target_include_directories(test_viz2d PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ${OpenCV_INCLUDE_DIRS}
        )
target_link_libraries(test_viz2d PUBLIC
        gtest
        gtest_main
        ${OpenCV_LIBRARIES}
        opencv_wrapper
)

add_executable(viz2d_component_main
                ${CMAKE_CURRENT_SOURCE_DIR}/viz2d_component_main.cc
                ${CMAKE_CURRENT_SOURCE_DIR}/viz2d_component.cc
                )
target_link_libraries(viz2d_component_main PUBLIC
                                                opencv_wrapper
                                                ${OpenCV_LIBRARIES}
                                                planning_proto
                                                common
                                                glog
                                                cyber_proto
                                                cyber
                                                fastrtps
                                                pthread
                                                )

set(CMAKE_INSTALL_PREFIX "${CMAKE_HOME_DIRECTORY}/build")
install(
        TARGETS
                test_viz2d
                viz2d_component_main
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)

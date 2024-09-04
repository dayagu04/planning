
set(BUILD_TYPE "STATIC")
add_compile_options(-fPIC)


add_library(virtual_chassis ${BUILD_TYPE}
                     virtual_chassis.cc
                     virtual_chassis_component.cc
                    )
target_include_directories(virtual_chassis PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(virtual_chassis PRIVATE
                common
                opencv_wrapper
                proto_interface
                Eigen3::Eigen
                glog
                cyber_proto
                cyber
                fastrtps
                )

add_executable(virtual_chassis_component_main
                ${CMAKE_CURRENT_SOURCE_DIR}/virtual_chassis_component_main.cc
                )
target_link_libraries(virtual_chassis_component_main PUBLIC
                                                opencv_wrapper
                                                common
                                                virtual_chassis
                                                proto_interface
                                                Eigen3::Eigen
                                                glog
                                                cyber_proto
                                                cyber
                                                fastrtps
                                                pthread
                                                )

add_executable(chassis_test
            ${CMAKE_CURRENT_SOURCE_DIR}/virtual_chassis_test.cc
)
target_link_libraries(chassis_test
                common
                virtual_chassis
                proto_interface
                Eigen3::Eigen
                cyber_proto
                cyber

)

set(CMAKE_INSTALL_PREFIX "${CMAKE_HOME_DIRECTORY}/build")
install(
        TARGETS
            chassis_test
            virtual_chassis_component_main
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)
add_executable(interface_${PROJECT_NAME} interface_test.cpp)
target_link_libraries(interface_${PROJECT_NAME} PUBLIC 
    c_interface
    proto_interface
    legacy_c_interface
)

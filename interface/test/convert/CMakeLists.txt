add_executable(convert_${PROJECT_NAME} struct_convert.cpp)
target_link_libraries(convert_${PROJECT_NAME} PUBLIC
  struct_convert
  struct_convert_legacy
)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../install/bin)
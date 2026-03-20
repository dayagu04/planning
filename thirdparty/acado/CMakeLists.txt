cmake_minimum_required(VERSION 3.0)
project(acado_lib)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

file(GLOB_RECURSE SRC_FILES
    src/*.c
    src/*.cpp
    thirdparty/qpoases/SRC/*.cpp
    thirdparty/qpoases/SRC/*.c
    )

add_library (acado_lib STATIC ${SRC_FILES})
target_include_directories(acado_lib PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/qpoases/
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/qpoases/INCLUDE/
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/qpoases/SRC/
)
target_include_directories(acado_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(acado_lib PRIVATE -fPIC)
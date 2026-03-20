cmake_minimum_required(VERSION 3.0)
project(system_monitor)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -mllvm -inline-threshold=1000 -fno-jump-tables -mllvm -unroll-count=8 -mllvm -enable-gvn-hoist")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -mllvm -inline-threshold=1000 -fno-jump-tables -mllvm -unroll-count=8 -mllvm -enable-gvn-hoist")


add_compile_options(-Werror)
file(GLOB_RECURSE INI_FILE_SOURCES ${CMAKE_CURRENT_LIST_DIR}/ini_parser.cpp ${CMAKE_CURRENT_LIST_DIR}/file_helper.cpp)
file(GLOB_RECURSE MONITOR_FILE_SOURCES ${CMAKE_CURRENT_LIST_DIR}/system_monitor.cpp)
file(GLOB_RECURSE EXEC_FILE_SOURCES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

include_directories(
    ${SYSTEM_INTEGRATION_PROJECT_SOURCE_DIR}/components/common/threadsafe_container
)

add_library(ini_parser SHARED ${INI_FILE_SOURCES})
add_library(${PROJECT_NAME} SHARED ${MONITOR_FILE_SOURCES})

add_executable(${PROJECT_NAME}_exec ${EXEC_FILE_SOURCES})

target_link_libraries(ini_parser PRIVATE
   pthread
   z
)

target_link_libraries(${PROJECT_NAME}_exec PRIVATE
   pthread
   z
)
#target_link_libraries(${PROJECT_NAME} libini_parser libsystem_monitor)

message(STATUS "project=${PROJECT_NAME}")
message(STATUS "CMAKE_CURRENT_LIST_DIR=${CMAKE_CURRENT_LIST_DIR}")
message(STATUS "CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}")
install(TARGETS ini_parser DESTINATION ${CMAKE_INSTALL_PREFIX}/entity_gea/lib)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/entity_gea/lib)
install(TARGETS ${PROJECT_NAME}_exec RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/gea_a/bin)
install(TARGETS ${PROJECT_NAME}_exec RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/gea_b/bin)
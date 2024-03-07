project(planning_exec)

scfi_package()

add_executable(${PROJECT_NAME} ${scfi_SOURCES})

# 指定目标引用哪些头文件路径
# ${scfi_INCLUDE_DIRS}为SCFI解析component.xml文件后自动添加头文件路径的变量
# target_include_directories(${PROJECT_NAME} PRIVATE ${scfi_INCLUDE_DIRS} <dir1>, <dir2>, ..., <dirN>)
target_include_directories(${PROJECT_NAME} PRIVATE ${scfi_INCLUDE_DIRS})

set(PROJECT_CORE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../..)
add_subdirectory(${PROJECT_CORE_DIR} ${PROJECT_NAME}.out)
message(${PROJECT_CORE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${PLATFORM} -D${PRODUCT}")

# 指定目标链接到哪些库
# ${scfi_LIBRARIES}为SCFI解析component.xml文件后自动链接库的变量
# target_link_libraries(${PROJECT_NAME} PUBLIC ${scfi_LIBRARIES} <lib1>, <lib2>, ... <libN>)
target_link_libraries(${PROJECT_NAME} PRIVATE ascend_hal "-Wl,--no-as-needed")
#  移除没必要要的符号引用
target_link_libraries(${PROJECT_NAME} PUBLIC ${scfi_LIBRARIES} "-Wl,--as-needed")
target_link_libraries(${PROJECT_NAME} PUBLIC
  -Wl,-whole-archive
  modules
  iflyauto_parse_file
  iflyauto_time
  iflyauto_timer_component
  iflyauto_container_pub
  visual
  securec
  -Wl,-no-whole-archive
)
# 该命令用于初始化SCFI组件包安装/导出信息
scfi_install()

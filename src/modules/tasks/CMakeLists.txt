project (modules_tasks)
cmake_minimum_required (VERSION 3.0)

# 查找 OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
    message("-- OpenMP found")
    # 同时设置编译和链接标志
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
    message("-- OpenMP not found, using -fopenmp flag")
    # 同时设置编译和链接标志
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fopenmp")
endif()

file(GLOB_RECURSE SRC_FILES
  ${CMAKE_CURRENT_LIST_DIR}/**.cc
)

add_library(${PROJECT_NAME} STATIC EXCLUDE_FROM_ALL ${SRC_FILES})

# 显式传递 OpenMP 链接标志给所有依赖此库的目标
if(OPENMP_FOUND)
    set_property(TARGET ${PROJECT_NAME} PROPERTY INTERFACE_LINK_LIBRARIES "${OpenMP_CXX_FLAGS}")
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
  common
  modules_common
  modules_adas_function
  ilqr_lib
  arastar_lib
  lc_pure_pursuit_lib
  lc_idm_lib
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})
add_subdirectory(behavior_planners/speed_search_decider)

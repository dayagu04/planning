project(common_swc_lib)

file(GLOB_RECURSE scfi_SOURCES ${CMAKE_CURRENT_LIST_DIR}/src/**.cpp)
file(GLOB_RECURSE BACKTRACE_FILE_SOURCES ${CMAKE_CURRENT_LIST_DIR}/sig_backtrace/sig_backtrace.cpp)
add_library(${PROJECT_NAME} SHARED ${scfi_SOURCES})
add_library(sig_backtrace SHARED ${BACKTRACE_FILE_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.0.0 SOVERSION 1) 

find_package(log QUIET CMAKE_FIND_ROOT_PATH_BOTH)
find_package(ara-com QUIET CMAKE_FIND_ROOT_PATH_BOTH)
find_package(rm QUIET CMAKE_FIND_ROOT_PATH_BOTH)

if(PRODUCT STREQUAL "CHERY_M32T" OR PRODUCT STREQUAL "BESTUNE_E541")
  find_package(ara_per QUIET CMAKE_FIND_ROOT_PATH_BOTH)
else()
  find_package(per QUIET CMAKE_FIND_ROOT_PATH_BOTH)
endif()

include_directories(
  ${SYSTEM_INTEGRATION_PROJECT_SOURCE_DIR}/components/interface/mdc/generated/includes
  ${CMAKE_CURRENT_LIST_DIR}/include/private
  ${CMAKE_CURRENT_LIST_DIR}/sig_backtrace
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include/public)

if(PRODUCT STREQUAL "CHERY_M32T" OR PRODUCT STREQUAL "BESTUNE_E541")
  target_include_directories(${PROJECT_NAME} PUBLIC
    /usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/sysroot/usr/lib64
  )
  target_link_libraries(${PROJECT_NAME} PRIVATE ara_per)
else()
  target_link_libraries(${PROJECT_NAME} PRIVATE ${per_LIBRARIES})
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${log_LIBRARIES}
  ${ara-com_LIBRARIES}
  ${rm_LIBRARIES}
  ascend_hal
  -Wl,--as-needed
  -Wl,-no-whole-archive
  yaml
  pthread
  expt_record
  sig_backtrace
)

if (PRODUCT STREQUAL "CHERY_M32T" OR PRODUCT STREQUAL "BESTUNE_E541")
  install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/entity_gea/lib)
  install(TARGETS sig_backtrace DESTINATION ${CMAKE_INSTALL_PREFIX}/entity_gea/lib)
else()
  install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/cluster_a/gea/lib)
  install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/cluster_b/gea/lib)
  install(TARGETS sig_backtrace DESTINATION ${CMAKE_INSTALL_PREFIX}/cluster_a/gea/lib)
  install(TARGETS sig_backtrace DESTINATION ${CMAKE_INSTALL_PREFIX}/cluster_b/gea/lib)
endif()

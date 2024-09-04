
# include(GNUInstallDirs)
set(PACKAGE_NAME matplotlib_cpp)
# set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)


# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_11
)

# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS matplotlib_cpp
  EXPORT install_targets
)


add_executable(minimal minimal.cc)
target_link_libraries(minimal PRIVATE matplotlib_cpp)
# set_target_properties(minimal PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

set(Python_ADDITIONAL_VERSIONS 3)
find_package(PythonLibs 3 REQUIRED)


add_executable(test_astar_log test_astar_by_log.cc
)
target_link_libraries(test_astar_log PUBLIC
  hybrid_astar_lib
  reeds_shepp
  cyber
  fastrtps
  matplotlib_cpp

)

set(CMAKE_INSTALL_PREFIX "${CMAKE_HOME_DIRECTORY}/build")
install(
        TARGETS
          minimal
          test_astar_log
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}
)
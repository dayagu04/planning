cmake_minimum_required(VERSION 2.8)

if (NOT TARGET mjson)
  add_library(mjson INTERFACE IMPORTED)
  set_target_properties(mjson PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/../include/"
  )
endif()

set(mjson_INCLUDE_DIRES ${CMAKE_CURRENT_LIST_DIR}/../include/)
set(mjson_FOUND TRUE)


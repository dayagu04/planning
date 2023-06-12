cmake_minimum_required(VERSION 3.0)
# no project name in src

if(ADAPTER STREQUAL "cyber")
  add_subdirectory(adapter/cyber)
elseif(ADAPTER STREQUAL "ap")
  # add_subdirectory((adapter/ap)
endif()

add_subdirectory(proto EXCLUDE_FROM_ALL)
add_subdirectory(common EXCLUDE_FROM_ALL)
add_subdirectory(modules EXCLUDE_FROM_ALL)
add_subdirectory(framework EXCLUDE_FROM_ALL)
add_subdirectory(apa_simulation EXCLUDE_FROM_ALL)

add_subdirectory(library/advanced_ctrl_lib EXCLUDE_FROM_ALL)
add_subdirectory(library/ilqr_lib EXCLUDE_FROM_ALL)

add_subdirectory(thirdparty/mjson EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/eigen EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/rapidjson EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/osqp EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/nlohmann_json EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/hfsm EXCLUDE_FROM_ALL)
add_subdirectory(thirdparty/interface/src/proto EXCLUDE_FROM_ALL)

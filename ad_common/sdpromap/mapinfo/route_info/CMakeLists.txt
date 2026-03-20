set(module_name route_info)

set(long_name ${PROJECT_NAME}_${module_name})

set(${module_name}_SOURCE_FILES
    route_info.cpp
)

add_library(${long_name} STATIC ${${module_name}_SOURCE_FILES})

set(module_name lane_info)

set(long_name ${PROJECT_NAME}_${module_name})

set(${module_name}_SOURCE_FILES
    lane_info.cpp
)

add_library(${long_name} STATIC ${${module_name}_SOURCE_FILES})

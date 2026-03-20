add_subdirectory(link_info)
add_subdirectory(lane_info)
add_subdirectory(route_info)

set(module_name mapinfos)

set(long_name ad_common_${module_name})

add_library(${long_name} STATIC 
    sdpromap_utils.hpp
)

target_link_libraries(${long_name}
    ${PROJECT_NAME}_link_info
    ${PROJECT_NAME}_lane_info
    ${PROJECT_NAME}_route_info          
)

set(AD_COMMON_LIBS "${AD_COMMON_LIBS};${long_name}" PARENT_SCOPE)

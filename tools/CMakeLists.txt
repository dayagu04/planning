
# todo: depend proto file, need to update it for c_struct.
option(ENABLE_VIZ "Enable uviz" OFF) # Disabled by default
if (ENABLE_VIZ)
    add_subdirectory(visualization2d)
    add_subdirectory(virtual_chassis)
    message("ENABLE_VIZ")
else()
    message("UNENABLE_VIZ")
endif()
